// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/test/unit_test.hpp>

#include "Acts/Definitions/Units.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "Acts/Utilities/ScopedTimer.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/Algorithms/TrackFinding/PhiHoughTransform.hpp"
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsExamples/Framework/WhiteBoard.hpp"
#include "ActsExamples/TrackFinding/MuonHoughSeeder.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"
#include "ActsExamples/Utilities/CudaUtilities.hpp"
#include "ActsExamples/Utilities/tbbWrap.hpp"
#include "ActsTests/CommonHelpers/WhiteBoardUtilities.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <TFile.h>
#include <TTree.h>
#include <cuda_runtime.h>
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>

#include "MuonHoughValidationData.hpp"

namespace ActsTests {
namespace {

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

// Retain only the best Phi extension of each Eta candidate. This prevents a
// combinatorial expansion of the final segment-seed collection.
constexpr std::size_t phiMaximaPerEtaMaximum = 1u;

struct ValidationMaximum {
  double slope = 0.0;
  double intercept = 0.0;
  double nHits = 0.0;
  double nLayers = 0.0;
  bool enoughHits = false;
  std::vector<std::uint32_t> associatedHitIndices{};
};

using ValidationMaximumBatch = std::vector<std::vector<ValidationMaximum>>;

std::size_t configuredBins(const char* name, std::size_t fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  std::size_t parsed = 0u;
  const std::string text{value};
  const std::size_t bins = std::stoull(text, &parsed);
  if (bins == 0u || parsed != text.size()) {
    throw std::invalid_argument(std::string{name} +
                                " must be a positive integer");
  }
  return bins;
}

template <typename MaximumBatch>
ValidationMaximumBatch copyEtaMaxima(const EtaValidationBatch& batch,
                                     const MaximumBatch& maxima) {
  ValidationMaximumBatch output(batch.buckets.size());
  for (std::size_t bucket = 0u; bucket < output.size(); ++bucket) {
    const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
    const std::size_t bucketEnd = batch.spacePoints.bucketEnd(bucket);
    for (std::size_t maximum = 0u; maximum < maxima.nMaxima(bucket);
         ++maximum) {
      ValidationMaximum value{};
      value.slope = maxima.tanBeta(bucket, maximum);
      value.intercept = maxima.interceptY(bucket, maximum);
      value.nHits = maxima.nHits(bucket, maximum);
      value.nLayers = maxima.nLayers(bucket, maximum);
      const auto associated = maxima.associatedHitIndices(bucket, maximum);
      value.associatedHitIndices.reserve(associated.size());
      for (const std::uint32_t globalHit : associated) {
        if (globalHit < bucketStart || globalHit >= bucketEnd) {
          throw std::runtime_error("Eta maximum contains a foreign hit");
        }
        value.associatedHitIndices.push_back(
            static_cast<std::uint32_t>(globalHit - bucketStart));
      }
      value.enoughHits =
          value.associatedHitIndices.size() >= 4u && value.nLayers >= 4.0;
      output[bucket].push_back(std::move(value));
    }
  }
  return output;
}

ValidationMaximumBatch copyPhiMaxima(
    const EtaValidationBatch& batch,
    const ActsExamples::CudaMuonSegmentSeedContainer& seeds) {
  ValidationMaximumBatch output(batch.buckets.size());
  for (const auto seed : seeds) {
    if (!seed->hasPhiExtension()) {
      continue;
    }
    const std::size_t bucket = seed->parentBucket();
    if (bucket >= output.size()) {
      throw std::runtime_error("Phi seed refers to an unknown bucket");
    }
    const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
    const std::size_t bucketEnd = batch.spacePoints.bucketEnd(bucket);

    ValidationMaximum value{};
    value.slope = seed->tanAlpha();
    value.intercept = seed->interceptX();
    value.nHits = seed->getCounts();
    std::unordered_set<unsigned> layers;
    for (const std::uint32_t globalHit : seed->associatedHitIndices()) {
      if (globalHit < bucketStart || globalHit >= bucketEnd) {
        throw std::runtime_error("Phi seed contains a foreign hit");
      }
      const std::uint32_t rawId = batch.spacePoints.muonId(globalHit);
      if (!ActsExamples::muonIdMeasuresPhi(rawId)) {
        continue;
      }
      value.associatedHitIndices.push_back(
          static_cast<std::uint32_t>(globalHit - bucketStart));
      layers.insert(ActsExamples::detLayer(rawId));
    }
    std::ranges::sort(value.associatedHitIndices);
    value.nLayers = static_cast<double>(layers.size());
    value.enoughHits = value.associatedHitIndices.size() >= 2u;
    output[bucket].push_back(std::move(value));
  }
  return output;
}

ValidationMaximumBatch copySegmentSeeds(
    const EtaValidationBatch& batch,
    const ActsExamples::CudaMuonSegmentSeedContainer& seeds) {
  ValidationMaximumBatch output(batch.buckets.size());
  for (const auto seed : seeds) {
    const std::size_t bucket = seed->parentBucket();
    if (bucket >= output.size()) {
      throw std::runtime_error("Segment seed refers to an unknown bucket");
    }
    const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
    const std::size_t bucketEnd = batch.spacePoints.bucketEnd(bucket);

    ValidationMaximum value{};
    value.slope = seed->tanBeta();
    value.intercept = seed->interceptY();
    std::unordered_set<unsigned> layers;
    for (const std::uint32_t globalHit : seed->associatedHitIndices()) {
      if (globalHit < bucketStart || globalHit >= bucketEnd) {
        throw std::runtime_error("Segment seed contains a foreign hit");
      }
      const std::uint32_t rawId = batch.spacePoints.muonId(globalHit);
      if (!ActsExamples::muonIdMeasuresEta(rawId)) {
        continue;
      }
      value.associatedHitIndices.push_back(
          static_cast<std::uint32_t>(globalHit - bucketStart));
      layers.insert(ActsExamples::detLayer(rawId));
    }
    std::ranges::sort(value.associatedHitIndices);
    value.nHits = static_cast<double>(value.associatedHitIndices.size());
    value.nLayers = static_cast<double>(layers.size());
    value.enoughHits =
        value.associatedHitIndices.size() >= 4u && value.nLayers >= 4.0;
    output[bucket].push_back(std::move(value));
  }
  return output;
}

void writeProjectionValidation(const EtaValidationBatch& batch,
                               const ValidationMaximumBatch& maxima,
                               bool phiProjection,
                               const std::filesystem::path& outputPath) {
  TFile outputFile{outputPath.c_str(), "RECREATE"};
  if (outputFile.IsZombie()) {
    throw std::runtime_error("Failed to create " + outputPath.string());
  }

  // Both projections use the established Eta-analysis schema. Phi slope and
  // intercept values occupy its generic tanBeta/y0 payload columns.
  TTree bucketTree{"EtaHoughBucketTree", "BucketTree"};
  TTree hitTree{"EtaHoughHitTree", "HitTree"};
  TTree truthTree{"EtaHoughTruthTree", "TruthTree"};
  TTree maximumTree{"EtaHoughTree", "MaximumTree"};

  std::uint32_t bucketNumber = 0u;
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint32_t nInputHits = 0u;
  std::uint16_t nTruthSegments = 0u;
  std::uint32_t nMaxima = 0u;
  bucketTree.Branch("bucketNumber", &bucketNumber);
  bucketTree.Branch("eventId", &eventId);
  bucketTree.Branch("sourceBucketId", &sourceBucketId);
  bucketTree.Branch("nInputHits", &nInputHits);
  bucketTree.Branch("nTruthSegments", &nTruthSegments);
  bucketTree.Branch("nMaxima", &nMaxima);

  std::uint32_t localHitIndex = 0u;
  double localX = 0.0;
  double localY = 0.0;
  double localZ = 0.0;
  double driftRadius = 0.0;
  double covarianceY = 0.0;
  std::uint32_t logicalLayer = 0u;
  hitTree.Branch("bucketNumber", &bucketNumber);
  hitTree.Branch("localHitIndex", &localHitIndex);
  hitTree.Branch("localX", &localX);
  hitTree.Branch("localY", &localY);
  hitTree.Branch("localZ", &localZ);
  hitTree.Branch("driftRadius", &driftRadius);
  hitTree.Branch("covarianceY", &covarianceY);
  hitTree.Branch("logicalLayer", &logicalLayer);

  std::uint32_t truthNumber = 0u;
  std::uint32_t segmentIndex = 0u;
  std::uint32_t nTruthHits = 0u;
  double trueTanBeta = 0.0;
  double trueY0 = 0.0;
  std::vector<std::uint32_t> truthHitIndices;
  truthTree.Branch("truthNumber", &truthNumber);
  truthTree.Branch("bucketNumber", &bucketNumber);
  truthTree.Branch("eventId", &eventId);
  truthTree.Branch("sourceBucketId", &sourceBucketId);
  truthTree.Branch("segmentIndex", &segmentIndex);
  truthTree.Branch("nTruthHits", &nTruthHits);
  truthTree.Branch("trueTanBeta", &trueTanBeta);
  truthTree.Branch("trueY0", &trueY0);
  truthTree.Branch("truthHitIndices", &truthHitIndices);

  std::uint32_t maximumId = 0u;
  std::uint32_t nAssociatedHits = 0u;
  double nHits = 0.0;
  double nLayers = 0.0;
  double recoTanBeta = 0.0;
  double recoY0 = 0.0;
  char enoughHits = 0;
  std::vector<std::uint32_t> associatedHitIndices;
  maximumTree.Branch("bucketNumber", &bucketNumber);
  maximumTree.Branch("maximumId", &maximumId);
  maximumTree.Branch("nAssociatedHits", &nAssociatedHits);
  maximumTree.Branch("nHits", &nHits);
  maximumTree.Branch("nLayers", &nLayers);
  maximumTree.Branch("recoTanBeta", &recoTanBeta);
  maximumTree.Branch("recoY0", &recoY0);
  maximumTree.Branch("enoughHits", &enoughHits);
  maximumTree.Branch("associatedHitIndices", &associatedHitIndices);

  for (std::size_t bucket = 0u; bucket < batch.buckets.size(); ++bucket) {
    bucketNumber = static_cast<std::uint32_t>(bucket);
    eventId = batch.buckets[bucket].eventId;
    sourceBucketId = batch.buckets[bucket].sourceBucketId;
    const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
    const std::size_t bucketEnd = batch.spacePoints.bucketEnd(bucket);
    nInputHits = static_cast<std::uint32_t>(bucketEnd - bucketStart);
    nTruthSegments = 0u;
    nMaxima = static_cast<std::uint32_t>(maxima[bucket].size());
    bucketTree.Fill();

    for (std::size_t hit = bucketStart; hit < bucketEnd; ++hit) {
      const auto point = batch.spacePoints[hit];
      const auto& position = point->localPosition();
      localHitIndex = static_cast<std::uint32_t>(hit - bucketStart);
      localX = phiProjection ? position.y() : position.x();
      localY = phiProjection ? position.x() : position.y();
      localZ = position.z();
      driftRadius = phiProjection ? 0.0 : std::abs(point->driftRadius());
      covarianceY = point->covariance()[phiProjection ? 0u : 1u];
      logicalLayer = ActsExamples::detLayer(batch.spacePoints.muonId(hit));
      hitTree.Fill();
    }

    for (std::size_t maximum = 0u; maximum < maxima[bucket].size(); ++maximum) {
      const ValidationMaximum& value = maxima[bucket][maximum];
      maximumId = static_cast<std::uint32_t>(maximum);
      nAssociatedHits =
          static_cast<std::uint32_t>(value.associatedHitIndices.size());
      nHits = value.nHits;
      nLayers = value.nLayers;
      recoTanBeta = value.slope;
      recoY0 = value.intercept;
      enoughHits = value.enoughHits ? 1 : 0;
      associatedHitIndices = value.associatedHitIndices;
      maximumTree.Fill();
    }
  }

  bucketTree.Write();
  hitTree.Write();
  truthTree.Write();
  maximumTree.Write();
  outputFile.Close();
}

void runCpuBenchmark(EtaValidationBatch& batch, std::size_t etaBinsX,
                     std::size_t etaBinsY, std::size_t phiBinsX,
                     std::size_t phiBinsY) {
  struct EventWork {
    std::uint32_t eventId = 0u;
    std::vector<std::size_t> buckets;
  };
  struct Timing {
    double seconds = 0.0;
    std::vector<double> eventMilliseconds;
    std::size_t threadsUsed = 0u;
  };

  // Build the production CPU EDM outside the measured interval.
  std::vector<ActsExamples::MuonSpacePointBucket> cpuBuckets(
      batch.buckets.size());
  for (std::size_t bucket = 0u; bucket < batch.buckets.size(); ++bucket) {
    auto& outputBucket = cpuBuckets[bucket];
    const std::size_t begin = batch.spacePoints.bucketStart(bucket);
    const std::size_t end = batch.spacePoints.bucketEnd(bucket);
    outputBucket.reserve(end - begin);
    for (std::size_t hit = begin; hit < end; ++hit) {
      const auto input = batch.spacePoints[hit];
      ActsExamples::MuonSpacePoint output;
      output.setId(input->id());
      output.setGeometryId(input->geometryId());
      output.setRadius(input->driftRadius());
      output.setTime(input->time());
      const auto& covariance = input->covariance();
      output.setCovariance(covariance[0], covariance[1], covariance[2]);
      output.defineCoordinates(Acts::Vector3{input->localPosition()},
                               Acts::Vector3{input->sensorDirection()},
                               Acts::Vector3{input->toNextSensor()});
      outputBucket.push_back(std::move(output));
    }
  }

  std::vector<EventWork> events;
  std::unordered_map<std::uint32_t, std::size_t> eventIndex;
  for (std::size_t bucket = 0u; bucket < batch.buckets.size(); ++bucket) {
    const std::uint32_t eventId = batch.buckets[bucket].eventId;
    const auto [entry, inserted] =
        eventIndex.try_emplace(eventId, events.size());
    if (inserted) {
      events.push_back({eventId, {}});
    }
    events[entry->second].buckets.push_back(bucket);
  }

  ActsExamples::MuonHoughSeeder::Config etaConfig{};
  etaConfig.inSpacePoints = "MuonSpacePoints";
  etaConfig.inTruthSegments = "MuonTruthSegments";
  etaConfig.outHoughMax = "CpuEtaHoughSeeds";
  etaConfig.nBinsTanTheta = static_cast<unsigned>(etaBinsX);
  etaConfig.nBinsY0 = static_cast<unsigned>(etaBinsY);
  etaConfig.nBinsTanPhi = static_cast<unsigned>(phiBinsX);
  etaConfig.nBinsX0 = static_cast<unsigned>(phiBinsY);
  etaConfig.extendWithPhi = false;
  auto e2eConfig = etaConfig;
  e2eConfig.outHoughMax = "CpuE2EHoughSeeds";
  e2eConfig.extendWithPhi = true;

  ActsExamples::MuonHoughSeeder etaSeeder{
      etaConfig,
      Acts::getDefaultLogger("CpuEtaHoughSeeder", Acts::Logging::Level::ERROR)};
  ActsExamples::MuonHoughSeeder e2eSeeder{
      e2eConfig,
      Acts::getDefaultLogger("CpuE2EHoughSeeder", Acts::Logging::Level::ERROR)};

  std::vector<std::unique_ptr<ActsExamples::WhiteBoard>> boards;
  boards.reserve(events.size());
  for (const EventWork& event : events) {
    ActsExamples::MuonSpacePointContainer input;
    input.reserve(event.buckets.size());
    for (const std::size_t bucket : event.buckets) {
      input.push_back(std::move(cpuBuckets[bucket]));
    }
    auto board =
        std::make_unique<ActsExamples::WhiteBoard>(Acts::getDefaultLogger(
            "CpuMuonHoughEventStore", Acts::Logging::Level::ERROR));
    ActsTests::addToWhiteBoard(etaConfig.inSpacePoints, std::move(input),
                               *board);
    ActsTests::addToWhiteBoard(etaConfig.inTruthSegments,
                               ActsExamples::MuonSegmentContainer{}, *board);
    boards.push_back(std::move(board));
  }
  cpuBuckets.clear();
  cpuBuckets.shrink_to_fit();

  int cpuThreads = tbb::task_arena::automatic;
  if (const char* value = std::getenv("ACTS_MUON_CPU_THREADS");
      value != nullptr) {
    cpuThreads = std::stoi(value);
    if (cpuThreads <= 0) {
      throw std::invalid_argument(
          "ACTS_MUON_CPU_THREADS must be a positive integer");
    }
  }

  ActsExamples::tbbWrap::task_arena arena{cpuThreads};
  const auto execute = [&](ActsExamples::MuonHoughSeeder& seeder,
                           const char* timerName) {
    Timing timing{};
    timing.eventMilliseconds.resize(events.size(), 0.0);
    std::atomic<std::size_t> nextThreadId{0u};
    tbb::enumerable_thread_specific<std::size_t> threadIds{
        [&nextThreadId]() { return nextThreadId++; }};
    auto logger = Acts::getDefaultLogger("CpuMuonHoughBenchmark",
                                         Acts::Logging::Level::INFO);
    const auto begin = Acts::ScopedTimer::clock_type::now();
    {
      Acts::ScopedTimer timer{timerName, *logger};
      arena.execute([&]() {
        ActsExamples::tbbWrap::parallel_for(
            tbb::blocked_range<std::size_t>{0u, events.size()},
            [&](const tbb::blocked_range<std::size_t>& range) {
              const std::size_t threadId = threadIds.local();
              for (std::size_t event = range.begin(); event < range.end();
                   ++event) {
                const auto eventBegin = Acts::ScopedTimer::clock_type::now();
                ActsExamples::AlgorithmContext context{
                    0u, events[event].eventId, *boards[event], threadId};
                if (seeder.internalExecute(context) !=
                    ActsExamples::ProcessCode::SUCCESS) {
                  throw std::runtime_error("CPU MuonHoughSeeder failed");
                }
                timing.eventMilliseconds[event] =
                    std::chrono::duration<double, std::milli>(
                        Acts::ScopedTimer::clock_type::now() - eventBegin)
                        .count();
              }
            });
      });
    }
    timing.seconds = std::chrono::duration<double>(
                         Acts::ScopedTimer::clock_type::now() - begin)
                         .count();
    timing.threadsUsed = nextThreadId.load();
    return timing;
  };

  const Timing etaTiming = execute(etaSeeder, "CPU Eta processing");
  const Timing e2eTiming = execute(e2eSeeder, "CPU Eta-to-Phi processing");
  // MuonHoughSeeder exposes Eta-only and complete Eta-to-Phi execution. Its
  // Phi-only interval is therefore the non-negative difference of those runs.
  const double phiSeconds =
      std::max(0.0, e2eTiming.seconds - etaTiming.seconds);
  const double nEvents = static_cast<double>(events.size());
  const auto perEvent = [nEvents](double seconds) {
    return nEvents == 0.0 ? 0.0 : 1000.0 * seconds / nEvents;
  };
  const auto mean = [](const std::vector<double>& values) {
    return values.empty() ? 0.0
                          : std::accumulate(values.begin(), values.end(), 0.0) /
                                static_cast<double>(values.size());
  };

  std::cout << "CPU worker threads used: "
            << std::max(etaTiming.threadsUsed, e2eTiming.threadsUsed) << '\n'
            << "Eta timing CPU processing: " << etaTiming.seconds << " s\n"
            << "Eta timing CPU processing per event (amortized): "
            << perEvent(etaTiming.seconds) << " ms\n"
            << "Eta timing CPU mean event latency: "
            << mean(etaTiming.eventMilliseconds) << " ms\n"
            << "Phi timing CPU derived processing: " << phiSeconds << " s\n"
            << "Phi timing CPU derived processing per event (amortized): "
            << perEvent(phiSeconds) << " ms\n"
            << "Phi CPU timing definition: max(E2E - Eta, 0)\n"
            << "E2E timing CPU processing: " << e2eTiming.seconds << " s\n"
            << "E2E timing CPU processing per event (amortized): "
            << perEvent(e2eTiming.seconds) << " ms\n"
            << "E2E timing CPU mean event latency: "
            << mean(e2eTiming.eventMilliseconds) << " ms" << std::endl;

  ValidationMaximumBatch etaOutput(batch.buckets.size());
  ValidationMaximumBatch phiOutput(batch.buckets.size());
  ValidationMaximumBatch segmentSeedOutput(batch.buckets.size());
  for (std::size_t event = 0u; event < events.size(); ++event) {
    ActsTests::DummySequenceElement element;
    ActsExamples::ReadDataHandle<ActsExamples::MuonSpacePointContainer>
        inputHandle{&element, "InputSpacePoints"};
    inputHandle.initialize(etaConfig.inSpacePoints);
    const auto& input = inputHandle(*boards[event]);

    std::unordered_map<const ActsExamples::MuonSpacePoint*,
                       std::pair<std::size_t, std::uint32_t>>
        hitIndices;
    for (std::size_t localBucket = 0u; localBucket < input.size();
         ++localBucket) {
      const std::size_t bucket = events[event].buckets[localBucket];
      for (std::size_t hit = 0u; hit < input[localBucket].size(); ++hit) {
        hitIndices.emplace(&input[localBucket][hit],
                           std::pair{bucket, static_cast<std::uint32_t>(hit)});
      }
    }

    const auto translate = [&](const std::string& outputName,
                               bool phiProjection,
                               ValidationMaximumBatch& output) {
      const auto maxima =
          ActsTests::getFromWhiteBoard<ActsExamples::MuonHoughMaxContainer>(
              outputName, *boards[event]);
      for (const auto& maximum : maxima) {
        ValidationMaximum converted{};
        converted.slope =
            phiProjection ? maximum.tanAlpha() : maximum.tanBeta();
        converted.intercept =
            phiProjection ? maximum.interceptX() : maximum.interceptY();
        std::optional<std::size_t> maximumBucket;
        std::unordered_set<unsigned> layers;
        for (const ActsExamples::MuonSpacePoint* hit : maximum.hits()) {
          const auto index = hitIndices.find(hit);
          if (index == hitIndices.end()) {
            throw std::runtime_error("CPU maximum contains a foreign hit");
          }
          if (maximumBucket.has_value() &&
              *maximumBucket != index->second.first) {
            throw std::runtime_error("CPU maximum mixes physical buckets");
          }
          maximumBucket = index->second.first;
          const bool selected =
              phiProjection ? hit->id().measuresPhi() : hit->id().measuresEta();
          if (selected) {
            converted.associatedHitIndices.push_back(index->second.second);
            layers.insert(hit->id().detLayer());
          }
        }
        if (!maximumBucket.has_value()) {
          throw std::runtime_error("CPU maximum has no associated hits");
        }
        // A pure-Eta fallback from the full CPU seeder is not a Phi maximum.
        if (phiProjection && converted.associatedHitIndices.size() < 2u) {
          continue;
        }
        std::ranges::sort(converted.associatedHitIndices);
        converted.nHits =
            static_cast<double>(converted.associatedHitIndices.size());
        converted.nLayers = static_cast<double>(layers.size());
        converted.enoughHits =
            phiProjection ? converted.associatedHitIndices.size() >= 2u
                          : converted.associatedHitIndices.size() >= 4u &&
                                converted.nLayers >= 4.0;
        output[*maximumBucket].push_back(std::move(converted));
      }
    };
    translate(etaConfig.outHoughMax, false, etaOutput);
    translate(e2eConfig.outHoughMax, true, phiOutput);
    translate(e2eConfig.outHoughMax, false, segmentSeedOutput);
  }

  const auto count = [](const ValidationMaximumBatch& maxima) {
    return std::accumulate(maxima.begin(), maxima.end(), std::size_t{0u},
                           [](std::size_t sum, const auto& bucket) {
                             return sum + bucket.size();
                           });
  };
  const double nBuckets = static_cast<double>(batch.buckets.size());
  const std::size_t etaCount = count(etaOutput);
  const std::size_t phiCount = count(phiOutput);
  const std::size_t segmentSeedCount = count(segmentSeedOutput);
  const std::size_t etaOnlySeedCount = segmentSeedCount - phiCount;
  const auto percentage = [](std::size_t value, std::size_t total) {
    return total == 0u ? 0.0
                       : 100.0 * static_cast<double>(value) /
                             static_cast<double>(total);
  };
  std::cout
      << "Average Eta Hough maxima per bucket: "
      << (nBuckets == 0.0 ? 0.0 : static_cast<double>(etaCount) / nBuckets)
      << " (" << etaCount << " maxima in " << batch.buckets.size()
      << " buckets)\n"
      << "Average Phi-extended SegmentSeeds per bucket: "
      << (nBuckets == 0.0 ? 0.0 : static_cast<double>(phiCount) / nBuckets)
      << " (" << phiCount << " seeds in " << batch.buckets.size()
      << " buckets)\n"
      << "Average SegmentSeeds per bucket: "
      << (nBuckets == 0.0 ? 0.0
                          : static_cast<double>(segmentSeedCount) / nBuckets)
      << " (" << segmentSeedCount << " seeds in " << batch.buckets.size()
      << " buckets)\n"
      << "Average Eta-only SegmentSeeds per bucket: "
      << (nBuckets == 0.0 ? 0.0
                          : static_cast<double>(etaOnlySeedCount) / nBuckets)
      << " (" << etaOnlySeedCount << " seeds in " << batch.buckets.size()
      << " buckets)\n"
      << "Phi extension fraction of SegmentSeeds: "
      << percentage(phiCount, segmentSeedCount) << " %\n"
      << "Phi extension fraction of Eta maxima: "
      << percentage(phiCount, etaCount) << " %" << std::endl;

  writeProjectionValidation(batch, etaOutput, false,
                            "CudaEtaHoughFileValidation.root");
  writeProjectionValidation(batch, phiOutput, true,
                            "CudaPhiHoughFileValidation.root");
  writeProjectionValidation(batch, segmentSeedOutput, false,
                            "CudaMuonSegmentSeedFileValidation.root");
}

template <CudaHT::PeakFinder peakFinder>
void runBenchmark(EtaValidationBatch& batch, std::size_t etaBinsX,
                  std::size_t etaBinsY, std::size_t phiBinsX,
                  std::size_t phiBinsY) {
  constexpr std::size_t etaMaximaPerBucket =
      peakFinder == CudaHT::PeakFinder::GlobalMaximum ? 1u : 4u;
  using Clock = Acts::ScopedTimer::clock_type;
  const auto seconds = [](Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double>(end - begin).count();
  };

  const std::size_t nBuckets = batch.buckets.size();
  std::unordered_set<std::uint32_t> eventIds;
  for (const EtaValidationBucket& bucket : batch.buckets) {
    eventIds.insert(bucket.eventId);
  }
  const std::size_t nEvents = eventIds.size();
  const Acts::HoughTransformUtils::HoughAxisRanges ranges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  CudaHT::CudaHoughPlaneBatch etaPlane{{etaBinsX, etaBinsY}, nBuckets};
  CudaHT::CudaHoughPlaneBatch phiPlane{{phiBinsX, phiBinsY},
                                       nBuckets * etaMaximaPerBucket};
  ActsExamples::CudaStream stream;

  const auto uploadStart = Clock::now();
  batch.spacePoints.moveToDevice(stream.get());
  etaPlane.moveToDevice(stream.get());
  phiPlane.moveToDevice(stream.get());
  const double uploadSeconds = seconds(uploadStart, Clock::now());

  const auto etaStart = Clock::now();
  auto etaMaxima =
      CudaHT::EtaHoughTransform::etaHoughTransform<etaMaximaPerBucket,
                                                   peakFinder>(
          etaPlane, batch.spacePoints, ranges, ActsExamples::YieldType{1.0},
          128u, 0u, stream.get());
  const double etaProcessingSeconds = seconds(etaStart, Clock::now());

  const auto phiStart = Clock::now();
  auto segmentSeeds =
      CudaHT::PhiHoughTransform::phiHoughTransform<phiMaximaPerEtaMaximum,
                                                   peakFinder>(
          phiPlane, batch.spacePoints, etaMaxima, ranges,
          ActsExamples::YieldType{1.0}, 128u, 0u, stream.get());
  const double phiProcessingSeconds = seconds(phiStart, Clock::now());

  const auto phiDownloadStart = Clock::now();
  segmentSeeds.moveToHost(stream.get());
  segmentSeeds.copyAssociatedHitIndicesToHost(stream.get());
  const double phiDownloadSeconds = seconds(phiDownloadStart, Clock::now());

  const auto etaDownloadStart = Clock::now();
  etaMaxima.moveToHost(stream.get());
  etaMaxima.copyAssociatedHitIndicesToHost(stream.get());
  const double etaDownloadSeconds = seconds(etaDownloadStart, Clock::now());

  const double processingSeconds = etaProcessingSeconds + phiProcessingSeconds;
  const double etaCompleteSeconds =
      uploadSeconds + etaProcessingSeconds + etaDownloadSeconds;
  const double phiCompleteSeconds = phiProcessingSeconds + phiDownloadSeconds;
  const double completeSeconds =
      uploadSeconds + processingSeconds + phiDownloadSeconds;
  const auto perEvent = [nEvents](double value) {
    return nEvents == 0u ? 0.0 : 1000.0 * value / static_cast<double>(nEvents);
  };

  std::size_t etaCount = 0u;
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    etaCount += etaMaxima.nMaxima(bucket);
  }
  std::size_t phiCount = 0u;
  for (const auto seed : segmentSeeds) {
    phiCount += seed->hasPhiExtension();
  }
  const std::size_t segmentSeedCount = segmentSeeds.size();
  const std::size_t etaOnlySeedCount = segmentSeedCount - phiCount;
  const auto percentage = [](std::size_t value, std::size_t total) {
    return total == 0u ? 0.0
                       : 100.0 * static_cast<double>(value) /
                             static_cast<double>(total);
  };

  std::cout << "Eta timing GPU upload: " << uploadSeconds << " s\n"
            << "Eta timing GPU processing: " << etaProcessingSeconds << " s\n"
            << "Eta timing GPU download: " << etaDownloadSeconds << " s\n"
            << "Eta timing GPU complete: " << etaCompleteSeconds << " s\n"
            << "Eta timing GPU upload per event (amortized): "
            << perEvent(uploadSeconds) << " ms\n"
            << "Eta timing GPU processing per event (amortized): "
            << perEvent(etaProcessingSeconds) << " ms\n"
            << "Eta timing GPU download per event (amortized): "
            << perEvent(etaDownloadSeconds) << " ms\n"
            << "Eta timing GPU complete per event (amortized): "
            << perEvent(etaCompleteSeconds) << " ms\n"
            << "Phi timing GPU processing: " << phiProcessingSeconds << " s\n"
            << "Phi timing GPU download: " << phiDownloadSeconds << " s\n"
            << "Phi timing GPU complete: " << phiCompleteSeconds << " s\n"
            << "Phi timing GPU processing per event (amortized): "
            << perEvent(phiProcessingSeconds) << " ms\n"
            << "Phi timing GPU download per event (amortized): "
            << perEvent(phiDownloadSeconds) << " ms\n"
            << "Phi timing GPU complete per event (amortized): "
            << perEvent(phiCompleteSeconds) << " ms\n"
            << "E2E timing GPU upload: " << uploadSeconds << " s\n"
            << "E2E timing GPU processing: " << processingSeconds << " s\n"
            << "E2E timing GPU download: " << phiDownloadSeconds << " s\n"
            << "E2E timing GPU complete: " << completeSeconds << " s\n"
            << "E2E timing GPU upload per event (amortized): "
            << perEvent(uploadSeconds) << " ms\n"
            << "E2E timing GPU processing per event (amortized): "
            << perEvent(processingSeconds) << " ms\n"
            << "E2E timing GPU download per event (amortized): "
            << perEvent(phiDownloadSeconds) << " ms\n"
            << "E2E timing GPU complete per event (amortized): "
            << perEvent(completeSeconds) << " ms\n"
            << "Average Eta Hough maxima per bucket: "
            << static_cast<double>(etaCount) / nBuckets << " (" << etaCount
            << " maxima in " << nBuckets << " buckets)\n"
            << "Average Phi-extended SegmentSeeds per bucket: "
            << static_cast<double>(phiCount) / nBuckets << " (" << phiCount
            << " seeds in " << nBuckets << " buckets)\n"
            << "Average SegmentSeeds per bucket: "
            << static_cast<double>(segmentSeedCount) / nBuckets << " ("
            << segmentSeedCount << " seeds in " << nBuckets << " buckets)\n"
            << "Average Eta-only SegmentSeeds per bucket: "
            << static_cast<double>(etaOnlySeedCount) / nBuckets << " ("
            << etaOnlySeedCount << " seeds in " << nBuckets << " buckets)\n"
            << "Phi extension fraction of SegmentSeeds: "
            << percentage(phiCount, segmentSeedCount) << " %\n"
            << "Phi extension fraction of Eta maxima: "
            << percentage(phiCount, etaCount) << " %" << std::endl;

  writeProjectionValidation(batch, copyEtaMaxima(batch, etaMaxima), false,
                            "CudaEtaHoughFileValidation.root");
  writeProjectionValidation(batch, copyPhiMaxima(batch, segmentSeeds), true,
                            "CudaPhiHoughFileValidation.root");
  writeProjectionValidation(batch, copySegmentSeeds(batch, segmentSeeds), false,
                            "CudaMuonSegmentSeedFileValidation.root");
}

}  // namespace

BOOST_AUTO_TEST_SUITE(CudaMuonHoughTransformBenchmarkSuite)

BOOST_AUTO_TEST_CASE(cuda_hough_e2e_file_validation) {
  const char* spacePointEnvironment = std::getenv("ACTS_MUON_SPACEPOINT_ROOT");
  if (spacePointEnvironment == nullptr) {
    BOOST_TEST_MESSAGE(
        "Set ACTS_MUON_SPACEPOINT_ROOT to run the E2E benchmark");
    return;
  }

  const std::filesystem::path spacePointPath{spacePointEnvironment};
  BOOST_REQUIRE(std::filesystem::exists(spacePointPath));

  EtaValidationBatch batch = fileReadEtaValidation(spacePointPath);
  const std::size_t etaBinsX = configuredBins("ACTS_MUON_ETA_BINS_X", 64u);
  const std::size_t etaBinsY = configuredBins("ACTS_MUON_ETA_BINS_Y", 32u);
  const std::size_t phiBinsX = configuredBins("ACTS_MUON_PHI_BINS_X", etaBinsX);
  const std::size_t phiBinsY = configuredBins("ACTS_MUON_PHI_BINS_Y", etaBinsY);
  const char* implementationEnvironment =
      std::getenv("ACTS_MUON_HOUGH_IMPLEMENTATION");
  if (implementationEnvironment == nullptr) {
    implementationEnvironment = std::getenv("ACTS_MUON_ETA_IMPLEMENTATION");
  }
  const std::string implementation =
      implementationEnvironment != nullptr ? implementationEnvironment : "cuda";

  if (implementation == "original-cpu") {
    std::cout << "Running original CPU Eta->Phi validation with Eta "
              << etaBinsX << 'x' << etaBinsY << ", Phi " << phiBinsX << 'x'
              << phiBinsY << " and IslandsAroundMax" << std::endl;
    runCpuBenchmark(batch, etaBinsX, etaBinsY, phiBinsX, phiBinsY);
    return;
  }
  BOOST_REQUIRE_MESSAGE(
      implementation == "cuda",
      "Unknown ACTS_MUON_HOUGH_IMPLEMENTATION: " << implementation);

  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
    BOOST_TEST_MESSAGE("No CUDA device found, skipping CUDA runtime test");
    return;
  }

  const std::string peakFinder =
      std::getenv("ACTS_MUON_CUDA_PEAK_FINDER") != nullptr
          ? std::getenv("ACTS_MUON_CUDA_PEAK_FINDER")
          : "relative-nms";

  std::cout << "Running Eta->Phi E2E validation with Eta " << etaBinsX << 'x'
            << etaBinsY << ", Phi " << phiBinsX << 'x' << phiBinsY << " and "
            << peakFinder << std::endl;
  if (peakFinder == "global") {
    runBenchmark<CudaHT::PeakFinder::GlobalMaximum>(batch, etaBinsX, etaBinsY,
                                                    phiBinsX, phiBinsY);
  } else if (peakFinder == "sliding-window") {
    runBenchmark<CudaHT::PeakFinder::SlidingWindow>(batch, etaBinsX, etaBinsY,
                                                    phiBinsX, phiBinsY);
  } else if (peakFinder == "relative-nms") {
    runBenchmark<CudaHT::PeakFinder::RelativeNms>(batch, etaBinsX, etaBinsY,
                                                  phiBinsX, phiBinsY);
  } else {
    BOOST_FAIL("Unknown ACTS_MUON_CUDA_PEAK_FINDER: " << peakFinder);
  }
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
