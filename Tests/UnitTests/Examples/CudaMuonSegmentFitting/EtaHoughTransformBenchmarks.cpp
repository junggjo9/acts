// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/test/unit_test.hpp>

#include "Acts/Definitions/Units.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"
#include "Acts/Seeding/HoughTransformUtils.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "Acts/Utilities/ScopedTimer.hpp"
#include "Acts/Utilities/UnitVectors.hpp"
#include "Acts/Utilities/VectorHelpers.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsExamples/Framework/WhiteBoard.hpp"
#include "ActsExamples/TrackFinding/MuonHoughSeeder.hpp"
#include "ActsExamples/Utilities/CudaUtilities.hpp"
#include "ActsExamples/Utilities/tbbWrap.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"
#include "ActsTests/CommonHelpers/WhiteBoardUtilities.hpp"

#include "../../Core/Seeding/StrawHitGeneratorHelper.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <TFile.h>
#include <TTree.h>
#include <TTreeReader.h>
#include <TTreeReaderArray.h>
#include <TTreeReaderValue.h>
#include <cuda_runtime.h>
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>

namespace ActsTests {
namespace {

struct EtaValidationTruth {
  std::uint32_t validationBucketId = 0u;
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint32_t segmentIndex = 0u;
  double tanBeta = 0.0;
  double y0 = 0.0;
  std::vector<std::uint32_t> truthHitIndices{};
};

struct EtaValidationBucket {
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint16_t nTruthSegments = 0u;
};

struct EtaValidationBatch {
  ActsExamples::CudaMuonSpacePointContainer spacePoints;
  std::vector<EtaValidationBucket> buckets;
  std::vector<EtaValidationTruth> truth;
};

struct GeneratedEtaTruth {
  double tanBeta = 0.0;
  double y0 = 0.0;
  double zReference = 0.0;
  std::uint32_t nGeneratedHits = 0u;
};

EtaValidationBatch makeGeneratedEtaValidationBatch(
    std::size_t nEvents, std::uint32_t seed, const Acts::Logger& logger,
    double minimumTanBeta, double maximumTanBeta) {
  MeasurementGenerator::Config generatorConfig{};
  generatorConfig.createStraws = true;
  generatorConfig.smearRadius = true;
  generatorConfig.twinStraw = false;
  generatorConfig.createStrips = false;

  RandomEngine engine{seed};
  std::vector<Container_t> eventMeasurements{};
  std::vector<GeneratedEtaTruth> generatedTruth{};
  eventMeasurements.reserve(nEvents);
  generatedTruth.reserve(nEvents);

  std::size_t totalHits = 0u;
  while (eventMeasurements.size() < nEvents) {
    const Line_t line = generateLine(engine, logger);
    const Acts::Vector3 direction = line.direction();
    if (std::abs(direction.z()) < 1.0e-12) {
      continue;
    }

    const double tanBeta = direction.y() / direction.z();
    if (!std::isfinite(tanBeta) || tanBeta < minimumTanBeta ||
        tanBeta > maximumTanBeta) {
      continue;
    }

    Container_t measurements =
        MeasurementGenerator::spawn(line, 0.0, engine, generatorConfig, logger);
    std::erase_if(measurements, [](const auto& measurement) {
      return !measurement->isStraw();
    });
    if (measurements.size() < 3u) {
      continue;
    }

    double minimumZ = std::numeric_limits<double>::max();
    double maximumZ = std::numeric_limits<double>::lowest();
    for (const auto& measurement : measurements) {
      const double z = measurement->localPosition().z();
      minimumZ = std::min(minimumZ, z);
      maximumZ = std::max(maximumZ, z);
    }

    const double zReference = 0.5 * (minimumZ + maximumZ);
    const double y0 =
        line.position().y() + tanBeta * (zReference - line.position().z());
    generatedTruth.push_back(
        {tanBeta, y0, zReference,
         static_cast<std::uint32_t>(measurements.size())});
    totalHits += measurements.size();
    eventMeasurements.push_back(std::move(measurements));
  }

  ActsExamples::CudaMuonSpacePointContainer spacePoints{totalHits};
  std::vector<EtaValidationBucket> buckets{};
  std::vector<EtaValidationTruth> truth{};
  buckets.reserve(nEvents);
  truth.reserve(nEvents);

  std::size_t hitIndex = 0u;
  for (std::size_t event = 0u; event < nEvents; ++event) {
    const std::size_t bucketStart = hitIndex;
    for (const auto& measurement : eventMeasurements[event]) {
      Acts::Vector3 position = measurement->localPosition();
      position.z() -= generatedTruth[event].zReference;
      const auto& covariance = measurement->covariance();

      spacePoints.setGeometryId(hitIndex, hitIndex);
      // Generated validation hits model Eta-measuring MDT space points.
      spacePoints.setId(hitIndex, 1u << 14u);
      spacePoints.defineCoordinates(hitIndex, position,
                                    measurement->sensorDirection(),
                                    measurement->toNextSensor());
      spacePoints.setRadius(hitIndex, std::abs(measurement->driftRadius()));
      spacePoints.setTime(hitIndex, measurement->time());
      spacePoints.setCovariance(hitIndex, covariance[0], covariance[1],
                                covariance[2]);
      spacePoints.setLogicalLayer(
          hitIndex, static_cast<std::uint32_t>(measurement->layer()));
      ++hitIndex;
    }

    spacePoints.addBucket(bucketStart, hitIndex);
    std::vector<std::uint32_t> truthHitIndices(
        generatedTruth[event].nGeneratedHits);
    std::iota(truthHitIndices.begin(), truthHitIndices.end(), 0u);
    buckets.push_back({static_cast<std::uint32_t>(event), 0u, 1u});
    truth.push_back(
        {static_cast<std::uint32_t>(event),
         static_cast<std::uint32_t>(event), 0u, 0u,
         generatedTruth[event].tanBeta, generatedTruth[event].y0,
         std::move(truthHitIndices)});
  }

  return {std::move(spacePoints), std::move(buckets), std::move(truth)};
}

/// Host-side copy of one hit row read from EtaValidationInput.
struct FileEtaHit {
  Acts::GeometryIdentifier geometryId{};
  std::uint32_t muonId = 0u;
  Acts::Vector3 localPosition = Acts::Vector3::Zero();
  Acts::Vector3 sensorDirection = Acts::Vector3::Zero();
  Acts::Vector3 toNextSensor = Acts::Vector3::Zero();
  double driftRadius = 0.0;
  double time = 0.0;
  std::array<double, 3> covariance{};
};

/// Read the two preprocessed ROOT trees and rebuild a CUDA validation batch.
EtaValidationBatch fileReadEtaValidation(
    const std::filesystem::path& inputPath,
    std::size_t maximumBuckets = std::numeric_limits<std::size_t>::max()) {
  using namespace Acts::UnitLiterals;

  TFile inputFile{inputPath.c_str(), "READ"};
  if (inputFile.IsZombie()) {
    throw std::runtime_error("Failed to open " + inputPath.string());
  }

  // 1. EtaValidationInput contains one row per hit. Consecutive rows with the
  // same validation_bucket_id are grouped back into a physical bucket.
  TTreeReader reader{"EtaValidationInput", &inputFile};
  if (reader.IsInvalid()) {
    throw std::runtime_error(
        "EtaValidationInput tree not found in " + inputPath.string());
  }

  TTreeReaderValue<UInt_t> validationBucketId{
      reader, "validation_bucket_id"};
  TTreeReaderValue<UInt_t> eventId{reader, "event_id"};
  TTreeReaderValue<UShort_t> sourceBucketId{reader, "source_bucket_id"};
  TTreeReaderValue<UInt_t> nInputHits{reader, "n_input_hits"};
  TTreeReaderValue<UShort_t> nTruthSegments{reader, "n_truth_segments"};

  TTreeReaderValue<ULong64_t> geometryId{reader, "geometry_id"};
  TTreeReaderValue<UInt_t> muonId{reader, "muon_id"};
  TTreeReaderValue<Float_t> localX{reader, "local_pos_x"};
  TTreeReaderValue<Float_t> localY{reader, "local_pos_y"};
  TTreeReaderValue<Float_t> localZ{reader, "local_pos_z"};
  TTreeReaderValue<Float_t> sensorPhi{reader, "sensor_dir_phi"};
  TTreeReaderValue<Float_t> sensorTheta{reader, "sensor_dir_theta"};
  TTreeReaderValue<Float_t> nextPhi{reader, "to_next_dir_phi"};
  TTreeReaderValue<Float_t> nextTheta{reader, "to_next_dir_theta"};
  TTreeReaderValue<Float_t> driftRadius{reader, "drift_radius"};
  TTreeReaderValue<Float_t> time{reader, "time"};
  TTreeReaderValue<Float_t> cov0{reader, "cov_loc0"};
  TTreeReaderValue<Float_t> cov1{reader, "cov_loc1"};
  TTreeReaderValue<Float_t> covT{reader, "cov_t"};

  std::vector<std::vector<FileEtaHit>> buckets;
  std::vector<EtaValidationBucket> bucketMetadata;
  std::vector<FileEtaHit> currentHits;

  bool hasCurrentBucket = false;
  UInt_t currentValidationBucket = 0u;
  UInt_t currentEvent = 0u;
  UShort_t currentSourceBucket = 0u;
  UInt_t currentInputHits = 0u;
  UShort_t currentTruthSegments = 0u;

  // Finalize a bucket when its identifier changes. These checks also validate
  // that the ROOT file is complete and ordered as promised by preprocessing.
  const auto flushBucket = [&]() {
    if (!hasCurrentBucket) {
      return;
    }

    if (currentValidationBucket != buckets.size()) {
      throw std::runtime_error(
          "EtaValidationInput bucket identifiers are not dense and ordered");
    }

    if (currentHits.size() != currentInputHits) {
      throw std::runtime_error(
          "Input-hit count mismatch in validation bucket " +
          std::to_string(currentValidationBucket));
    }

    if (buckets.size() < maximumBuckets) {
      buckets.push_back(std::move(currentHits));
      bucketMetadata.push_back(
          {currentEvent, currentSourceBucket, currentTruthSegments});
    }

    currentHits.clear();
  };

  const auto makeDirection = [](double phiDegrees, double thetaDegrees) {
    return Acts::makeDirectionFromPhiTheta<double>(
        phiDegrees * 1._degree, thetaDegrees * 1._degree);
  };

  // 2. reader.Next() loads one hit row and refreshes all bound branch values.
  while (reader.Next()) {
    const bool newBucket =
        !hasCurrentBucket ||
        *validationBucketId != currentValidationBucket;

    if (newBucket) {
      flushBucket();

      if (buckets.size() >= maximumBuckets) {
        break;
      }

      hasCurrentBucket = true;
      currentValidationBucket = *validationBucketId;
      currentEvent = *eventId;
      currentSourceBucket = *sourceBucketId;
      currentInputHits = *nInputHits;
      currentTruthSegments = *nTruthSegments;
    } else if (*eventId != currentEvent ||
               *sourceBucketId != currentSourceBucket ||
               *nInputHits != currentInputHits ||
               *nTruthSegments != currentTruthSegments) {
      throw std::runtime_error(
          "Metadata changed inside validation bucket " +
          std::to_string(currentValidationBucket));
    }

    FileEtaHit hit{};
    hit.geometryId = Acts::GeometryIdentifier{
        static_cast<Acts::GeometryIdentifier::Value>(*geometryId)};
    hit.muonId = *muonId;
    hit.localPosition = Acts::Vector3{*localX, *localY, *localZ};
    hit.sensorDirection = makeDirection(*sensorPhi, *sensorTheta);
    hit.toNextSensor = makeDirection(*nextPhi, *nextTheta);
    hit.driftRadius = std::abs(*driftRadius);
    hit.time = *time;
    hit.covariance = {*cov0, *cov1, *covT};

    currentHits.push_back(std::move(hit));
  }

  if (buckets.size() < maximumBuckets) {
    flushBucket();
  }

  if (buckets.empty()) {
    throw std::runtime_error(
        "EtaValidationInput contains no source bucket");
  }

  // 3. EtaValidationTruth is normalized separately: one row per segment.
  // TTreeReaderArray handles truth_hit_indices because that branch has a
  // variable number of bucket-local indices in each row.
  TTreeReader truthReader{"EtaValidationTruth", &inputFile};
  if (truthReader.IsInvalid()) {
    throw std::runtime_error(
        "EtaValidationTruth tree not found in " + inputPath.string());
  }

  TTreeReaderValue<UInt_t> validationTruthId{
      truthReader, "validation_truth_id"};
  TTreeReaderValue<UInt_t> truthBucketId{
      truthReader, "validation_bucket_id"};
  TTreeReaderValue<UInt_t> truthEventId{truthReader, "event_id"};
  TTreeReaderValue<UShort_t> truthSourceBucketId{
      truthReader, "source_bucket_id"};
  TTreeReaderValue<UInt_t> segmentIndex{truthReader, "segment_index"};
  TTreeReaderValue<UInt_t> nTruthHits{truthReader, "n_truth_hits"};
  TTreeReaderValue<Double_t> trueTanBeta{truthReader, "true_tanBeta"};
  TTreeReaderValue<Double_t> trueY0{truthReader, "true_y0"};
  TTreeReaderArray<UInt_t> truthHitIndices{
      truthReader, "truth_hit_indices"};

  std::vector<EtaValidationTruth> truth{};
  std::vector<std::uint16_t> countedTruthSegments(buckets.size(), 0u);

  // Validate every reference before copying it into the common truth model.
  while (truthReader.Next()) {
    if (*truthBucketId >= buckets.size()) {
      continue;
    }

    if (*validationTruthId != truth.size()) {
      throw std::runtime_error(
          "EtaValidationTruth identifiers are not dense and ordered");
    }

    const EtaValidationBucket& metadata = bucketMetadata[*truthBucketId];
    if (*truthEventId != metadata.eventId ||
        *truthSourceBucketId != metadata.sourceBucketId) {
      throw std::runtime_error(
          "Truth metadata disagrees with validation bucket metadata");
    }

    if (truthHitIndices.GetSize() != *nTruthHits) {
      throw std::runtime_error(
          "Truth-hit count mismatch in validation truth record " +
          std::to_string(*validationTruthId));
    }

    std::vector<std::uint8_t> seen(buckets[*truthBucketId].size(), 0u);
    for (const UInt_t localHitIndex : truthHitIndices) {
      if (localHitIndex >= seen.size() || seen[localHitIndex] != 0u) {
        throw std::runtime_error(
            "Invalid or duplicate local truth-hit index in truth record " +
            std::to_string(*validationTruthId));
      }
      seen[localHitIndex] = 1u;
    }

    truth.push_back(
        {*truthBucketId, *truthEventId, *truthSourceBucketId, *segmentIndex,
         *trueTanBeta, *trueY0,
         std::vector<std::uint32_t>(truthHitIndices.begin(),
                                    truthHitIndices.end())});
    ++countedTruthSegments[*truthBucketId];
  }

  for (std::size_t bucket = 0u; bucket < buckets.size(); ++bucket) {
    if (countedTruthSegments[bucket] !=
        bucketMetadata[bucket].nTruthSegments) {
      throw std::runtime_error(
          "Truth-segment count mismatch in validation bucket " +
          std::to_string(bucket));
    }
  }

  // 4. Flatten temporary host buckets into the contiguous representation used
  // by CUDA, while addBucket preserves every half-open bucket hit range.
  const std::size_t totalHits =
      std::accumulate(buckets.begin(), buckets.end(), std::size_t{0u},
                      [](std::size_t sum, const auto& bucket) {
                        return sum + bucket.size();
                      });

  ActsExamples::CudaMuonSpacePointContainer spacePoints{totalHits};

  std::size_t outputHit = 0u;
  for (const auto& bucket : buckets) {
    const std::size_t bucketStart = outputHit;

    for (const FileEtaHit& hit : bucket) {
      spacePoints.setGeometryId(outputHit, hit.geometryId.value());
      spacePoints.setId(outputHit, hit.muonId);
      spacePoints.defineCoordinates(outputHit, hit.localPosition,
                                    hit.sensorDirection, hit.toNextSensor);
      spacePoints.setRadius(outputHit, hit.driftRadius);
      spacePoints.setTime(outputHit, hit.time);
      spacePoints.setCovariance(outputHit, hit.covariance[0],
                                hit.covariance[1], hit.covariance[2]);
      ++outputHit;
    }

    spacePoints.addBucket(bucketStart, outputHit);
  }

  if (spacePoints.bucketCount() != bucketMetadata.size()) {
    throw std::runtime_error("Metadata and CUDA bucket counts differ");
  }

  return {std::move(spacePoints), std::move(bucketMetadata), std::move(truth)};
}

}  // namespace

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

BOOST_AUTO_TEST_SUITE(CudaHoughTransformUtilsSuite)

namespace {

std::uint32_t rawMuonIdLayer(std::uint32_t rawId) {
  // Particle Gun muon IDs encode the zero-based logical layer in bits 17--20.
  static constexpr std::uint32_t fourBit = 0xFu;
  static constexpr std::uint32_t layerShift = 17u;

  return (rawId >> layerShift) & fourBit;
}

struct CpuEtaMaximum {
  double tanBeta = 0.0;
  double interceptY = 0.0;
  double nHits = 0.0;
  double nLayers = 0.0;
  std::vector<std::uint32_t> associatedHitIndices{};
};

class CpuEtaMaximumBatch {
 public:
  explicit CpuEtaMaximumBatch(std::size_t nBuckets) : m_maxima(nBuckets) {}

  void add(std::size_t bucket, CpuEtaMaximum maximum) {
    m_maxima.at(bucket).push_back(std::move(maximum));
  }

  std::size_t nMaxima(std::size_t bucket) const {
    return m_maxima.at(bucket).size();
  }
  double tanBeta(std::size_t bucket, std::size_t maximum) const {
    return m_maxima.at(bucket).at(maximum).tanBeta;
  }
  double interceptY(std::size_t bucket, std::size_t maximum) const {
    return m_maxima.at(bucket).at(maximum).interceptY;
  }
  double nHits(std::size_t bucket, std::size_t maximum) const {
    return m_maxima.at(bucket).at(maximum).nHits;
  }
  double nLayers(std::size_t bucket, std::size_t maximum) const {
    return m_maxima.at(bucket).at(maximum).nLayers;
  }
  std::size_t nAssociatedHits(std::size_t bucket,
                              std::size_t maximum) const {
    return m_maxima.at(bucket).at(maximum).associatedHitIndices.size();
  }
  std::span<const std::uint32_t> associatedHitIndices(
      std::size_t bucket, std::size_t maximum) const {
    const auto& hits = m_maxima.at(bucket).at(maximum).associatedHitIndices;
    return {hits.data(), hits.size()};
  }

 private:
  std::vector<std::vector<CpuEtaMaximum>> m_maxima;
};

struct CpuEtaResult {
  CpuEtaMaximumBatch maxima;
  std::vector<double> eventMilliseconds;
  double processingSeconds = 0.0;
  std::size_t threadsUsed = 0u;
};

struct CudaEtaEventInput {
  std::vector<std::size_t> globalBuckets{};
  ActsExamples::CudaMuonSpacePointContainer spacePoints;
  CudaHT::CudaHoughPlaneBatch plane;

  CudaEtaEventInput(
      std::vector<std::size_t> buckets,
      ActsExamples::CudaMuonSpacePointContainer input,
      const Acts::HoughTransformUtils::HoughPlaneConfig& planeConfig)
      : globalBuckets{std::move(buckets)},
        spacePoints{std::move(input)},
        plane{planeConfig, spacePoints.bucketCount()} {}
};

std::vector<CudaEtaEventInput> makeCudaEtaEventInputs(
    const EtaValidationBatch& batch,
    const Acts::HoughTransformUtils::HoughPlaneConfig& planeConfig) {
  struct EventBuckets {
    std::vector<std::size_t> buckets{};
  };

  std::vector<EventBuckets> groupedEvents;
  std::unordered_map<std::uint32_t, std::size_t> eventIndex;
  for (std::size_t bucket = 0u; bucket < batch.buckets.size(); ++bucket) {
    const std::uint32_t eventId = batch.buckets[bucket].eventId;
    const auto [entry, inserted] =
        eventIndex.try_emplace(eventId, groupedEvents.size());
    if (inserted) {
      groupedEvents.emplace_back();
    }
    groupedEvents[entry->second].buckets.push_back(bucket);
  }

  std::vector<CudaEtaEventInput> result;
  result.reserve(groupedEvents.size());
  for (EventBuckets& event : groupedEvents) {
    const std::size_t nHits =
        std::accumulate(event.buckets.begin(), event.buckets.end(),
                        std::size_t{0u}, [&](std::size_t sum, std::size_t bucket) {
                          return sum + batch.spacePoints.bucketEnd(bucket) -
                                 batch.spacePoints.bucketStart(bucket);
                        });
    ActsExamples::CudaMuonSpacePointContainer eventSpacePoints{nHits};

    std::size_t outputHit = 0u;
    for (const std::size_t bucket : event.buckets) {
      const std::size_t outputBegin = outputHit;
      for (std::size_t hit = batch.spacePoints.bucketStart(bucket);
           hit < batch.spacePoints.bucketEnd(bucket); ++hit) {
        const auto input = batch.spacePoints[hit];
        const auto& covariance = input->covariance();
        eventSpacePoints.setGeometryId(outputHit, input->geometryId().value());
        eventSpacePoints.setId(outputHit, input->id().toInt());
        eventSpacePoints.defineCoordinates(
            outputHit, input->localPosition(), input->sensorDirection(),
            input->toNextSensor());
        eventSpacePoints.setRadius(outputHit, input->driftRadius());
        eventSpacePoints.setTime(outputHit, input->time());
        eventSpacePoints.setCovariance(outputHit, covariance[0], covariance[1],
                                       covariance[2]);
        ++outputHit;
      }
      eventSpacePoints.addBucket(outputBegin, outputHit);
    }

    result.emplace_back(std::move(event.buckets), std::move(eventSpacePoints),
                        planeConfig);
  }
  return result;
}

struct CudaEtaEventResult {
  CpuEtaMaximumBatch maxima;
  std::vector<double> uploadMilliseconds;
  std::vector<double> processingMilliseconds;
  std::vector<double> downloadMilliseconds;
  std::vector<double> totalMilliseconds;
  double wallSeconds = 0.0;
  std::size_t workersUsed = 0u;
  std::uint32_t maximumBlocksPerEvent = 0u;
};

CudaEtaEventResult runCudaEtaPerEvent(
    EtaValidationBatch& batch, std::size_t nBinsX, std::size_t nBinsY,
    std::string_view peakFinderName,
    const Acts::HoughTransformUtils::HoughAxisRanges& axisRanges,
    const Acts::Logger& timerLogger) {
  constexpr std::size_t maximumCapacityPerBucket = 8u;
  using BenchmarkClock = Acts::ScopedTimer::clock_type;

  const Acts::HoughTransformUtils::HoughPlaneConfig planeConfig{nBinsX,
                                                                nBinsY};
  std::vector<CudaEtaEventInput> events =
      makeCudaEtaEventInputs(batch, planeConfig);

  std::size_t workerCount = 8u;
  if (const char* value = std::getenv("ACTS_MUON_CUDA_STREAMS");
      value != nullptr) {
    const std::string text{value};
    std::size_t parsedCharacters = 0u;
    workerCount = std::stoull(text, &parsedCharacters);
    if (workerCount == 0u || parsedCharacters != text.size()) {
      throw std::invalid_argument(
          "ACTS_MUON_CUDA_STREAMS must be a positive integer");
    }
  }
  workerCount = std::min(workerCount, events.size());

  std::uint32_t maximumBlocksPerEvent = 8u;
  if (const char* value = std::getenv("ACTS_MUON_CUDA_BLOCKS_PER_EVENT");
      value != nullptr) {
    const std::string text{value};
    std::size_t parsedCharacters = 0u;
    const std::size_t blocks = std::stoull(text, &parsedCharacters);
    if (parsedCharacters != text.size() ||
        blocks > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument(
          "ACTS_MUON_CUDA_BLOCKS_PER_EVENT must fit into std::uint32_t");
    }
    maximumBlocksPerEvent = static_cast<std::uint32_t>(blocks);
  }

  std::vector<CudaHT::EtaHoughTransform::Processor> workers;
  workers.reserve(workerCount);
  for (std::size_t worker = 0u; worker < workerCount; ++worker) {
    workers.emplace_back();
  }

  struct EventMaximum {
    std::size_t globalBucket = 0u;
    CpuEtaMaximum maximum{};
  };
  std::vector<std::vector<EventMaximum>> eventMaxima(events.size());

  CudaEtaEventResult result{
      CpuEtaMaximumBatch{batch.buckets.size()},
      std::vector<double>(events.size(), 0.0),
      std::vector<double>(events.size(), 0.0),
      std::vector<double>(events.size(), 0.0),
      std::vector<double>(events.size(), 0.0),
      0.0,
      0u,
      maximumBlocksPerEvent};

  std::atomic<std::size_t> nextWorkerId{0u};
  tbb::enumerable_thread_specific<std::size_t> workerIds{
      [&nextWorkerId]() { return nextWorkerId++; }};
  ActsExamples::tbbWrap::task_arena arena{static_cast<int>(workerCount)};
  const auto wallStart = BenchmarkClock::now();
  {
    Acts::ScopedTimer timer{"CUDA Eta per-event processing", timerLogger};
    arena.execute([&]() {
      ActsExamples::tbbWrap::parallel_for(
          tbb::blocked_range<std::size_t>{0u, events.size()},
          [&](const tbb::blocked_range<std::size_t>& range) {
            const std::size_t workerId = workerIds.local();
            auto& processor = workers.at(workerId);
            const cudaStream_t stream = processor.stream();

            for (std::size_t event = range.begin(); event < range.end();
                 ++event) {
              auto& input = events[event];
              const auto eventStart = BenchmarkClock::now();

              const auto uploadStart = BenchmarkClock::now();
              input.spacePoints.moveToDevice(stream);
              input.plane.moveToDevice(stream);
              result.uploadMilliseconds[event] =
                  std::chrono::duration<double, std::milli>(
                      BenchmarkClock::now() - uploadStart)
                      .count();

              const auto processingStart = BenchmarkClock::now();
              auto maxima = [&]() {
                if (peakFinderName == "sliding-window") {
                  return processor.run<
                      maximumCapacityPerBucket,
                      CudaHT::PeakFinder::SlidingWindow>(
                      input.plane, input.spacePoints, axisRanges,
                      ActsExamples::YieldType{1.0}, 128u,
                      maximumBlocksPerEvent);
                }
                if (peakFinderName == "relative-nms") {
                  return processor.run<maximumCapacityPerBucket,
                                       CudaHT::PeakFinder::RelativeNms>(
                      input.plane, input.spacePoints, axisRanges,
                      ActsExamples::YieldType{1.0}, 128u,
                      maximumBlocksPerEvent);
                }
                return processor.run<maximumCapacityPerBucket,
                                     CudaHT::PeakFinder::GlobalMaximum>(
                    input.plane, input.spacePoints, axisRanges,
                    ActsExamples::YieldType{1.0}, 128u,
                    maximumBlocksPerEvent);
              }();
              result.processingMilliseconds[event] =
                  std::chrono::duration<double, std::milli>(
                      BenchmarkClock::now() - processingStart)
                      .count();

              const auto downloadStart = BenchmarkClock::now();
              maxima.moveToHost(stream);
              maxima.copyAssociatedHitIndicesToHost(stream);
              result.downloadMilliseconds[event] =
                  std::chrono::duration<double, std::milli>(
                      BenchmarkClock::now() - downloadStart)
                      .count();
              result.totalMilliseconds[event] =
                  std::chrono::duration<double, std::milli>(
                      BenchmarkClock::now() - eventStart)
                      .count();

              auto& output = eventMaxima[event];
              for (std::size_t localBucket = 0u;
                   localBucket < input.globalBuckets.size(); ++localBucket) {
                const std::size_t globalBucket =
                    input.globalBuckets[localBucket];
                const std::size_t localBucketStart =
                    input.spacePoints.bucketStart(localBucket);
                const std::size_t localBucketEnd =
                    input.spacePoints.bucketEnd(localBucket);
                const std::size_t globalBucketStart =
                    batch.spacePoints.bucketStart(globalBucket);

                for (std::size_t maximum = 0u;
                     maximum < maxima.nMaxima(localBucket); ++maximum) {
                  CpuEtaMaximum converted{};
                  converted.tanBeta = maxima.tanBeta(localBucket, maximum);
                  converted.interceptY =
                      maxima.interceptY(localBucket, maximum);
                  converted.nHits = maxima.nHits(localBucket, maximum);
                  converted.nLayers = maxima.nLayers(localBucket, maximum);
                  for (const std::uint32_t localHit :
                       maxima.associatedHitIndices(localBucket, maximum)) {
                    if (localHit < localBucketStart ||
                        localHit >= localBucketEnd) {
                      throw std::runtime_error(
                          "Per-event CUDA maximum references another bucket");
                    }
                    converted.associatedHitIndices.push_back(
                        static_cast<std::uint32_t>(
                            globalBucketStart + localHit - localBucketStart));
                  }
                  output.push_back({globalBucket, std::move(converted)});
                }
              }

              // Event device allocations are deliberately not retained or
              // cached for the next event.
              input.spacePoints.clearDevice();
              input.plane.clearDevice();
            }
          });
    });
  }
  result.wallSeconds =
      std::chrono::duration<double>(BenchmarkClock::now() - wallStart).count();
  result.workersUsed = nextWorkerId.load();

  for (auto& maxima : eventMaxima) {
    for (auto& [globalBucket, maximum] : maxima) {
      result.maxima.add(globalBucket, std::move(maximum));
    }
  }
  return result;
}

CpuEtaResult runOriginalCpuEta(EtaValidationBatch& batch,
                               std::size_t nBinsX, std::size_t nBinsY,
                               const Acts::Logger& timerLogger) {
  struct EventWork {
    std::uint32_t eventId = 0u;
    std::vector<std::size_t> buckets;
  };

  // Convert the already-loaded flat validation data to the production CPU EDM.
  // This adapter is deliberately outside the measured algorithm interval.
  std::vector<ActsExamples::MuonSpacePointBucket> cpuBuckets(
      batch.buckets.size());
  for (std::size_t bucket = 0u; bucket < batch.buckets.size(); ++bucket) {
    auto& outputBucket = cpuBuckets[bucket];
    const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
    const std::size_t bucketEnd = batch.spacePoints.bucketEnd(bucket);
    outputBucket.reserve(bucketEnd - bucketStart);

    for (std::size_t hit = bucketStart; hit < bucketEnd; ++hit) {
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

  ActsExamples::MuonHoughSeeder::Config config{};
  config.inSpacePoints = "MuonSpacePoints";
  config.inTruthSegments = "MuonTruthSegments";
  config.outHoughMax = "MuonHoughSeeds";
  config.nBinsTanTheta = static_cast<unsigned>(nBinsX);
  config.nBinsY0 = static_cast<unsigned>(nBinsY);
  config.extendWithPhi = false;
  ActsExamples::MuonHoughSeeder seeder{
      config, Acts::getDefaultLogger("CpuEtaHoughSeeder",
                                     Acts::Logging::Level::ERROR)};

  int cpuThreads = tbb::task_arena::automatic;
  if (const char* value = std::getenv("ACTS_MUON_CPU_THREADS");
      value != nullptr) {
    cpuThreads = std::stoi(value);
    if (cpuThreads <= 0) {
      throw std::invalid_argument(
          "ACTS_MUON_CPU_THREADS must be a positive integer");
    }
  }

  CpuEtaResult result{CpuEtaMaximumBatch{batch.buckets.size()},
                      std::vector<double>(events.size(), 0.0), 0.0, 0u};
  std::vector<std::unique_ptr<ActsExamples::WhiteBoard>> eventBoards;
  eventBoards.reserve(events.size());
  for (const EventWork& event : events) {
    ActsExamples::MuonSpacePointContainer inputContainer;
    inputContainer.reserve(event.buckets.size());
    for (const std::size_t bucket : event.buckets) {
      inputContainer.push_back(std::move(cpuBuckets[bucket]));
    }

    auto board = std::make_unique<ActsExamples::WhiteBoard>(
        Acts::getDefaultLogger("CpuEtaEventStore",
                               Acts::Logging::Level::ERROR));
    ActsTests::addToWhiteBoard(config.inSpacePoints,
                               std::move(inputContainer), *board);
    ActsTests::addToWhiteBoard(config.inTruthSegments,
                               ActsExamples::MuonSegmentContainer{}, *board);
    eventBoards.push_back(std::move(board));
  }
  cpuBuckets.clear();
  cpuBuckets.shrink_to_fit();

  std::atomic<std::size_t> nextThreadId{0u};
  tbb::enumerable_thread_specific<std::size_t> threadIds{
      [&nextThreadId]() { return nextThreadId++; }};
  ActsExamples::tbbWrap::task_arena arena{cpuThreads};
  const auto processingStart = Acts::ScopedTimer::clock_type::now();
  {
    Acts::ScopedTimer processingTimer{"CPU Eta event processing", timerLogger};
    arena.execute([&]() {
      ActsExamples::tbbWrap::parallel_for(
          tbb::blocked_range<std::size_t>{0u, events.size()},
          [&](const tbb::blocked_range<std::size_t>& range) {
            const std::size_t threadId = threadIds.local();
            for (std::size_t event = range.begin(); event < range.end();
                 ++event) {
              const auto eventStart = Acts::ScopedTimer::clock_type::now();
              ActsExamples::AlgorithmContext context{0u, events[event].eventId,
                                                      *eventBoards[event],
                                                      threadId};
              if (seeder.internalExecute(context) !=
                  ActsExamples::ProcessCode::SUCCESS) {
                throw std::runtime_error("CPU MuonHoughSeeder failed");
              }
              const auto eventStop = Acts::ScopedTimer::clock_type::now();
              result.eventMilliseconds[event] =
                  std::chrono::duration<double, std::milli>(eventStop -
                                                            eventStart)
                      .count();
            }
          });
    });
  }
  result.processingSeconds =
      std::chrono::duration<double>(Acts::ScopedTimer::clock_type::now() -
                                    processingStart)
          .count();
  result.threadsUsed = nextThreadId.load();

  // Translate production CPU output into the common validation representation.
  // This is serialization preparation and is not part of algorithm timing.
  for (std::size_t event = 0u; event < events.size(); ++event) {
    ActsTests::DummySequenceElement readerElement;
    ActsExamples::ReadDataHandle<ActsExamples::MuonSpacePointContainer>
        inputHandle{&readerElement, "InputSpacePoints"};
    inputHandle.initialize(config.inSpacePoints);
    const auto& storedInput = inputHandle(*eventBoards[event]);
    const auto maxima =
        ActsTests::getFromWhiteBoard<ActsExamples::MuonHoughMaxContainer>(
            config.outHoughMax, *eventBoards[event]);

    std::unordered_map<const ActsExamples::MuonSpacePoint*,
                       std::pair<std::size_t, std::uint32_t>>
        hitIndices;
    for (std::size_t localBucket = 0u; localBucket < storedInput.size();
         ++localBucket) {
      const std::size_t bucket = events[event].buckets[localBucket];
      const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
      const auto& storedBucket = storedInput[localBucket];
      for (std::size_t hit = 0u; hit < storedBucket.size(); ++hit) {
        hitIndices.emplace(
            &storedBucket[hit],
            std::pair{bucket,
                      static_cast<std::uint32_t>(bucketStart + hit)});
      }
    }

    for (const auto& maximum : maxima) {
      CpuEtaMaximum output{};
      output.tanBeta = maximum.tanBeta();
      output.interceptY = maximum.interceptY();
      std::unordered_set<unsigned> layers;
      std::optional<std::size_t> maximumBucket;
      output.associatedHitIndices.reserve(maximum.hits().size());
      for (const ActsExamples::MuonSpacePoint* hit : maximum.hits()) {
        const auto hitIndex = hitIndices.find(hit);
        if (hitIndex == hitIndices.end()) {
          throw std::runtime_error(
              "CPU Hough maximum references a foreign bucket hit");
        }
        if (maximumBucket.has_value() &&
            *maximumBucket != hitIndex->second.first) {
          throw std::runtime_error(
              "CPU Hough maximum mixes hits from multiple buckets");
        }
        maximumBucket = hitIndex->second.first;
        output.associatedHitIndices.push_back(hitIndex->second.second);
        layers.insert(hit->id().detLayer());
      }
      if (!maximumBucket.has_value()) {
        throw std::runtime_error("CPU Hough maximum has no associated hits");
      }
      std::ranges::sort(output.associatedHitIndices);
      output.nHits = static_cast<double>(output.associatedHitIndices.size());
      output.nLayers = static_cast<double>(layers.size());
      result.maxima.add(*maximumBucket, std::move(output));
    }
  }
  return result;
}

/// Serialize an Eta-transform result into the common analysis-ready schema.
template <typename MaximumBatch>
void writeEtaValidation(EtaValidationBatch& batch,
                        const MaximumBatch& maxima,
                        const std::filesystem::path& outputPath) {
  constexpr std::uint32_t minimumSeedHits = 4u;
  const std::size_t nBuckets = batch.buckets.size();

  std::vector<std::uint16_t> countedTruthSegments(nBuckets, 0u);
  for (const EtaValidationTruth& truth : batch.truth) {
    BOOST_REQUIRE_LT(truth.validationBucketId, nBuckets);
    ++countedTruthSegments[truth.validationBucketId];
  }
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    BOOST_REQUIRE_EQUAL(countedTruthSegments[bucket],
                        batch.buckets[bucket].nTruthSegments);
  }

  std::size_t totalMaxima = 0u;
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    totalMaxima += maxima.nMaxima(bucket);
  }
  const double averageMaximaPerBucket =
      nBuckets == 0u
          ? 0.0
          : static_cast<double>(totalMaxima) /
                static_cast<double>(nBuckets);
  std::cout << "Average Hough maxima per bucket: " << averageMaximaPerBucket
            << " (" << totalMaxima << " maxima in " << nBuckets
            << " buckets)" << std::endl;

  const std::string outputFileName = outputPath.string();

  // 3. Create the ROOT output.
  // TTrees behave like related tables keyed by bucketNumber:
  //   BucketTree: one row per bucket;
  //   HitTree: one row per input hit;
  //   TruthTree: one row per truth segment;
  //   MaximumTree: one row per returned Hough maximum.
  TFile outputFile{outputFileName.c_str(), "RECREATE"};
  BOOST_REQUIRE_MESSAGE(!outputFile.IsZombie(),
                        "Failed to create " << outputPath.string());

  TTree bucketTree{"EtaHoughBucketTree", "BucketTree"};
  TTree hitTree{"EtaHoughHitTree", "HitTree"};
  TTree truthTree{"EtaHoughTruthTree", "TruthTree"};
  TTree maximumTree{"EtaHoughTree", "MaximumTree"};

  bucketTree.SetDirectory(&outputFile);
  hitTree.SetDirectory(&outputFile);
  truthTree.SetDirectory(&outputFile);
  maximumTree.SetDirectory(&outputFile);

  // Keep baskets well below ROOT's 1 GB object-buffer limit and stream them
  // into the file throughout filling instead of retaining the complete trees
  // in memory until the end.
  constexpr Long64_t autoFlushBytes = 64ll * 1024ll * 1024ll;
  bucketTree.SetAutoFlush(-autoFlushBytes);
  hitTree.SetAutoFlush(-autoFlushBytes);
  truthTree.SetAutoFlush(-autoFlushBytes);
  maximumTree.SetAutoFlush(-autoFlushBytes);
  std::uint32_t bucketNumber = 0u;
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint32_t nInputHits = 0u;
  std::uint16_t nTruthSegments = 0u;
  std::uint32_t nMaxima = 0u;

  // Branch binds a ROOT column to a C++ variable address. Each later Fill()
  // snapshots the variables' current values as one new tree row.
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

  // Hit-tree columns. bucketNumber + localHitIndex uniquely identify a hit.
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
  std::vector<std::uint32_t> truthHitIndices{};

  // Truth-tree columns. std::vector creates a variable-length ROOT branch.
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
  std::vector<std::uint32_t> associatedHitIndices{};

  // Maximum-tree columns, including bucket-local associated hit indices.
  maximumTree.Branch("bucketNumber", &bucketNumber);
  maximumTree.Branch("maximumId", &maximumId);
  maximumTree.Branch("nAssociatedHits", &nAssociatedHits);
  maximumTree.Branch("nHits", &nHits);
  maximumTree.Branch("nLayers", &nLayers);
  maximumTree.Branch("recoTanBeta", &recoTanBeta);
  maximumTree.Branch("recoY0", &recoY0);
  maximumTree.Branch("enoughHits", &enoughHits);
  maximumTree.Branch("associatedHitIndices", &associatedHitIndices);

  // 4. Fill bucket, hit, and maximum trees together while bucket-local ranges
  // and CUDA maximum indices are readily available.
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    bucketNumber = static_cast<std::uint32_t>(bucket);
    eventId = batch.buckets[bucket].eventId;
    sourceBucketId = batch.buckets[bucket].sourceBucketId;
    nInputHits = static_cast<std::uint32_t>(
        batch.spacePoints.bucketEnd(bucket) -
        batch.spacePoints.bucketStart(bucket));
    nTruthSegments = batch.buckets[bucket].nTruthSegments;
    nMaxima = static_cast<std::uint32_t>(maxima.nMaxima(bucket));
    bucketTree.Fill();

    const std::size_t bucketStart = batch.spacePoints.bucketStart(bucket);
    const std::size_t bucketEnd = batch.spacePoints.bucketEnd(bucket);

    for (std::size_t hit = bucketStart; hit < bucketEnd; ++hit) {
      const auto spacePoint = batch.spacePoints[hit];
      const auto& position = spacePoint->localPosition();
      localHitIndex = static_cast<std::uint32_t>(hit - bucketStart);
      localX = position.x();
      localY = position.y();
      localZ = position.z();
      driftRadius = spacePoint->driftRadius();
      covarianceY = spacePoint->covariance()[1];
      logicalLayer = rawMuonIdLayer(batch.spacePoints.muonId(hit));
      hitTree.Fill();
    }

    for (std::size_t maximum = 0u; maximum < maxima.nMaxima(bucket);
         ++maximum) {
      maximumId = static_cast<std::uint32_t>(maximum);
      nAssociatedHits = static_cast<std::uint32_t>(
          maxima.nAssociatedHits(bucket, maximum));
      nHits = maxima.nHits(bucket, maximum);
      nLayers = maxima.nLayers(bucket, maximum);
      recoTanBeta = maxima.tanBeta(bucket, maximum);
      recoY0 = maxima.interceptY(bucket, maximum);
      enoughHits = nAssociatedHits >= minimumSeedHits &&
                           nLayers >= static_cast<double>(minimumSeedHits)
                       ? 1
                       : 0;

      associatedHitIndices.clear();
      const auto associatedSpan =
          maxima.associatedHitIndices(bucket, maximum);
      associatedHitIndices.reserve(associatedSpan.size());
      for (const std::uint32_t globalHitIndex : associatedSpan) {
        BOOST_REQUIRE_GE(static_cast<std::size_t>(globalHitIndex), bucketStart);
        BOOST_REQUIRE_LT(static_cast<std::size_t>(globalHitIndex), bucketEnd);
        associatedHitIndices.push_back(
            static_cast<std::uint32_t>(globalHitIndex - bucketStart));
      }
      BOOST_REQUIRE_EQUAL(associatedHitIndices.size(), nAssociatedHits);
      maximumTree.Fill();
    }
  }

  // 5. Truth is independent of the number of maxima, so fill its tree once
  // from the input truth collection after processing all buckets.
  for (const EtaValidationTruth& truth : batch.truth) {
    truthNumber = static_cast<std::uint32_t>(truthTree.GetEntries());
    bucketNumber = truth.validationBucketId;
    eventId = truth.eventId;
    sourceBucketId = truth.sourceBucketId;
    segmentIndex = truth.segmentIndex;
    nTruthHits = static_cast<std::uint32_t>(truth.truthHitIndices.size());
    trueTanBeta = truth.tanBeta;
    trueY0 = truth.y0;
    truthHitIndices = truth.truthHitIndices;
    truthTree.Fill();
  }

  // 6. Write the tree headers; their baskets have already been streamed.
  bucketTree.Write();
  hitTree.Write();
  truthTree.Write();
  maximumTree.Write();
  outputFile.Close();

  BOOST_TEST_MESSAGE("Wrote validation data to " << outputPath.string());
}

void runEtaValidation(EtaValidationBatch& batch,
                      const std::string& validationName,
                      const std::filesystem::path& outputPath,
                      const Acts::HoughTransformUtils::HoughAxisRanges&
                          axisRanges) {
  constexpr std::size_t maximumCapacityPerBucket = 8u;

  const char* implementationEnvironment =
      std::getenv("ACTS_MUON_ETA_IMPLEMENTATION");
  const std::string implementation = implementationEnvironment != nullptr
                                         ? implementationEnvironment
                                         : "cuda";
  BOOST_REQUIRE_MESSAGE(
      implementation == "cuda" || implementation == "original-cpu",
      "ACTS_MUON_ETA_IMPLEMENTATION must be 'cuda' or 'original-cpu'");

  const std::size_t nBuckets = batch.buckets.size();
  std::unordered_set<std::uint32_t> eventIds;
  eventIds.reserve(nBuckets);
  for (const EtaValidationBucket& bucket : batch.buckets) {
    eventIds.insert(bucket.eventId);
  }
  const std::size_t nEvents = eventIds.size();
  auto timerLogger = Acts::getDefaultLogger(
      "EtaHoughBenchmark", Acts::Logging::Level::INFO);
  using BenchmarkClock = Acts::ScopedTimer::clock_type;
  const auto elapsedSeconds = [](BenchmarkClock::time_point start,
                                 BenchmarkClock::time_point stop) {
    return std::chrono::duration<double>(stop - start).count();
  };

  const auto configuredBins = [](const char* name, std::size_t fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr) {
      return fallback;
    }
    const std::string text{value};
    std::size_t parsedCharacters = 0u;
    const std::size_t bins = std::stoull(text, &parsedCharacters);
    if (bins == 0u || parsedCharacters != text.size()) {
      throw std::invalid_argument(std::string{name} +
                                  " must be a positive integer");
    }
    return bins;
  };

  if (implementation == "original-cpu") {
    const std::size_t nBinsX =
        configuredBins("ACTS_MUON_ETA_BINS_X", 64u);
    const std::size_t nBinsY =
        configuredBins("ACTS_MUON_ETA_BINS_Y", 32u);
    std::cout << "Running " << validationName << " with the original "
              << nBinsX << " x " << nBinsY
              << " CPU Eta transform and IslandsAroundMax" << std::endl;

    CpuEtaResult result =
        runOriginalCpuEta(batch, nBinsX, nBinsY, *timerLogger);
    const double processingSeconds = result.processingSeconds;
    const double bucketsPerSecond =
        processingSeconds > 0.0
            ? static_cast<double>(nBuckets) / processingSeconds
            : 0.0;
    const double processingMillisecondsPerEvent =
        nEvents == 0u
            ? 0.0
            : 1000.0 * processingSeconds / static_cast<double>(nEvents);
    const double meanEventLatency =
        result.eventMilliseconds.empty()
            ? 0.0
            : std::accumulate(result.eventMilliseconds.begin(),
                              result.eventMilliseconds.end(), 0.0) /
                  static_cast<double>(result.eventMilliseconds.size());
    std::cout << "Eta timing CPU processing: " << processingSeconds << " s"
              << std::endl;
    std::cout << "Eta CPU worker threads used: " << result.threadsUsed
              << std::endl;
    std::cout << "Eta timing CPU processing per event (amortized): "
              << processingMillisecondsPerEvent << " ms" << std::endl;
    std::cout << "Eta timing CPU mean event latency: " << meanEventLatency
              << " ms" << std::endl;
    std::cout << "Eta transform wall time: " << processingSeconds << " s ("
              << bucketsPerSecond << " buckets/s)" << std::endl;
    writeEtaValidation(batch, result.maxima, outputPath);
    return;
  }

  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
    BOOST_TEST_MESSAGE("No CUDA device found, skipping CUDA runtime test");
    return;
  }

  const char* peakFinderEnvironment =
      std::getenv("ACTS_MUON_CUDA_PEAK_FINDER");
  const std::string peakFinderName = peakFinderEnvironment != nullptr
                                         ? peakFinderEnvironment
                                         : "global";
  BOOST_REQUIRE_MESSAGE(
      peakFinderName == "global" || peakFinderName == "sliding-window" ||
          peakFinderName == "relative-nms",
      "ACTS_MUON_CUDA_PEAK_FINDER must be 'global', 'sliding-window' or "
      "'relative-nms'");

  const std::size_t nBinsX = configuredBins("ACTS_MUON_ETA_BINS_X", 64u);
  const std::size_t nBinsY = configuredBins("ACTS_MUON_ETA_BINS_Y", 32u);
  const char* benchmarkModeEnvironment =
      std::getenv("ACTS_MUON_CUDA_BENCHMARK_MODE");
  const std::string benchmarkMode = benchmarkModeEnvironment != nullptr
                                        ? benchmarkModeEnvironment
                                        : "bulk";
  BOOST_REQUIRE_MESSAGE(
      benchmarkMode == "bulk" || benchmarkMode == "per-event",
      "ACTS_MUON_CUDA_BENCHMARK_MODE must be 'bulk' or 'per-event'");

  if (benchmarkMode == "per-event") {
    std::cout << "Running " << validationName << " event by event with "
              << nBinsX << " x " << nBinsY << " bins using "
              << peakFinderName << std::endl;
    CudaEtaEventResult result =
        runCudaEtaPerEvent(batch, nBinsX, nBinsY, peakFinderName, axisRanges,
                           *timerLogger);

    const auto mean = [](const std::vector<double>& values) {
      return values.empty()
                 ? 0.0
                 : std::accumulate(values.begin(), values.end(), 0.0) /
                       static_cast<double>(values.size());
    };
    const double bucketsPerSecond =
        result.wallSeconds > 0.0
            ? static_cast<double>(nBuckets) / result.wallSeconds
            : 0.0;
    const double wallMillisecondsPerEvent =
        nEvents == 0u
            ? 0.0
            : 1000.0 * result.wallSeconds / static_cast<double>(nEvents);

    std::cout << "Eta CUDA event workers/streams used: "
              << result.workersUsed << std::endl;
    std::cout << "Eta CUDA maximum blocks per event: ";
    if (result.maximumBlocksPerEvent == 0u) {
      std::cout << "automatic" << std::endl;
    } else {
      std::cout << result.maximumBlocksPerEvent << std::endl;
    }
    std::cout << "Eta timing GPU upload mean event latency: "
              << mean(result.uploadMilliseconds) << " ms" << std::endl;
    std::cout << "Eta timing GPU processing mean event latency: "
              << mean(result.processingMilliseconds) << " ms" << std::endl;
    std::cout << "Eta timing GPU download mean event latency: "
              << mean(result.downloadMilliseconds) << " ms" << std::endl;
    std::cout << "Eta timing GPU complete mean event latency: "
              << mean(result.totalMilliseconds) << " ms" << std::endl;
    std::cout << "Eta timing GPU wall time per event: "
              << wallMillisecondsPerEvent << " ms" << std::endl;
    std::cout << "Eta transform wall time: " << result.wallSeconds << " s ("
              << bucketsPerSecond << " buckets/s)" << std::endl;
    writeEtaValidation(batch, result.maxima, outputPath);
    return;
  }

  CudaHT::CudaHoughPlaneBatch plane{{nBinsX, nBinsY}, nBuckets};
  ActsExamples::CudaStream stream;
  std::cout << "Running " << validationName << " with " << plane.nBinsX()
            << " x " << plane.nBinsY() << " bins for " << nBuckets
            << " buckets using the " << peakFinderName << " peak finder"
            << std::endl;

  double uploadSeconds = 0.0;
  double processingSeconds = 0.0;
  double downloadSeconds = 0.0;
  const auto totalStart = BenchmarkClock::now();
  auto maxima = [&]() {
    Acts::ScopedTimer totalTimer{"CUDA Eta total", *timerLogger};

    const auto uploadStart = BenchmarkClock::now();
    {
      Acts::ScopedTimer uploadTimer{"CUDA Eta host-to-device", *timerLogger};
      batch.spacePoints.moveToDevice(stream.get());
      plane.moveToDevice(stream.get());
    }
    uploadSeconds = elapsedSeconds(uploadStart, BenchmarkClock::now());

    const auto processingStart = BenchmarkClock::now();
    auto result = [&]() {
      Acts::ScopedTimer processingTimer{"CUDA Eta processing", *timerLogger};
      if (peakFinderName == "sliding-window") {
        return CudaHT::EtaHoughTransform::etaHoughTransform<
            maximumCapacityPerBucket, CudaHT::PeakFinder::SlidingWindow>(
            plane, batch.spacePoints, axisRanges,
            ActsExamples::YieldType{1.0}, 128u, 0u, stream.get());
      }
      if (peakFinderName == "relative-nms") {
        return CudaHT::EtaHoughTransform::etaHoughTransform<
            maximumCapacityPerBucket, CudaHT::PeakFinder::RelativeNms>(
            plane, batch.spacePoints, axisRanges,
            ActsExamples::YieldType{1.0}, 128u, 0u, stream.get());
      }
      return CudaHT::EtaHoughTransform::etaHoughTransform<
          maximumCapacityPerBucket, CudaHT::PeakFinder::GlobalMaximum>(
          plane, batch.spacePoints, axisRanges,
          ActsExamples::YieldType{1.0}, 128u, 0u, stream.get());
    }();
    processingSeconds =
        elapsedSeconds(processingStart, BenchmarkClock::now());

    const auto downloadStart = BenchmarkClock::now();
    {
      Acts::ScopedTimer downloadTimer{"CUDA Eta device-to-host", *timerLogger};
      result.moveToHost(stream.get());
      result.copyAssociatedHitIndicesToHost(stream.get());
    }
    downloadSeconds = elapsedSeconds(downloadStart, BenchmarkClock::now());

    return result;
  }();

  const double totalSeconds =
      elapsedSeconds(totalStart, BenchmarkClock::now());

  const double bucketsPerSecond =
      totalSeconds > 0.0 ? static_cast<double>(nBuckets) / totalSeconds : 0.0;
  const double processingMillisecondsPerEvent =
      nEvents == 0u
          ? 0.0
          : 1000.0 * processingSeconds / static_cast<double>(nEvents);
  const double uploadMillisecondsPerEvent =
      nEvents == 0u
          ? 0.0
          : 1000.0 * uploadSeconds / static_cast<double>(nEvents);
  const double downloadMillisecondsPerEvent =
      nEvents == 0u
          ? 0.0
          : 1000.0 * downloadSeconds / static_cast<double>(nEvents);
  const double totalMillisecondsPerEvent =
      nEvents == 0u
          ? 0.0
          : 1000.0 * totalSeconds / static_cast<double>(nEvents);
  std::cout << "Eta timing GPU upload: " << uploadSeconds << " s" << std::endl;
  std::cout << "Eta timing GPU processing: " << processingSeconds << " s"
            << std::endl;
  std::cout << "Eta timing GPU download: " << downloadSeconds << " s"
            << std::endl;
  std::cout << "Eta timing GPU upload per event (amortized): "
            << uploadMillisecondsPerEvent << " ms" << std::endl;
  std::cout << "Eta timing GPU processing per event (amortized): "
            << processingMillisecondsPerEvent << " ms" << std::endl;
  std::cout << "Eta timing GPU download per event (amortized): "
            << downloadMillisecondsPerEvent << " ms" << std::endl;
  std::cout << "Eta timing GPU transfers plus processing per event: "
            << totalMillisecondsPerEvent << " ms" << std::endl;
  std::cout << "Eta transform wall time: " << totalSeconds << " s ("
            << bucketsPerSecond << " buckets/s)" << std::endl;
  writeEtaValidation(batch, maxima, outputPath);
}

}  // namespace

BOOST_AUTO_TEST_CASE(cuda_hough_eta_straw_generator_validation) {
  // Controlled end-to-end sample. This writes the same four ROOT trees as the
  // Particle Gun test, allowing one analysis program to process both outputs.
  constexpr std::size_t nEvents = 5000u;

  auto logger = Acts::getDefaultLogger(
      "CudaEtaHoughValidation", Acts::Logging::Level::INFO);

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  auto generated = makeGeneratedEtaValidationBatch(
      nEvents, 42u, *logger, axisRanges.xMin, axisRanges.xMax);

  runEtaValidation(
      generated,
      "Generated Eta Hough validation",
      "CudaEtaHoughValidation.root",
      axisRanges);
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_file_validation) {
  // Realistic end-to-end sample prepared by preprocessor_pg.py. The environment
  // variable permits the large flat ROOT input to live outside the build tree.
  // ACTS_MUON_ETA_IMPLEMENTATION selects the CUDA implementation or the
  // original CPU Eta transform. ACTS_MUON_CUDA_PEAK_FINDER selects the CUDA
  // peak finder. ACTS_MUON_ETA_BINS_X and ACTS_MUON_ETA_BINS_Y can force
  // common binning for a like-for-like timing comparison.
  // ACTS_MUON_CUDA_BENCHMARK_MODE selects the original bulk measurement or
  // event-granular processing, while ACTS_MUON_CUDA_STREAMS controls how many
  // per-worker CUDA processors are used by the latter.
  // ACTS_MUON_CUDA_BLOCKS_PER_EVENT limits the CUDA grids for each event; zero
  // preserves their default launch sizes.
  const char* environmentPath =
      std::getenv("ACTS_MUON_VALIDATION_FLAT_ROOT");

  const std::filesystem::path inputPath =
      environmentPath != nullptr ? environmentPath : "EtaHoughFlatInput.root";

  BOOST_TEST_MESSAGE("inputPath: " << inputPath);

  if (!std::filesystem::exists(inputPath)) {
    BOOST_TEST_MESSAGE("Flat validation file not found: "
                       << inputPath << ". Run preprocessor_pg.py first or set "
                                       "ACTS_MUON_VALIDATION_FLAT_ROOT.");

    BOOST_TEST_MESSAGE("Skipping test");
    return;
  }

  auto loaded = fileReadEtaValidation(inputPath);

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  runEtaValidation(
      loaded,
      "File Eta Hough validation",
      "CudaEtaHoughFileValidation.root",
      axisRanges);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
