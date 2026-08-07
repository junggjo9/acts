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
#include "Acts/Utilities/UnitVectors.hpp"
#include "Acts/Utilities/VectorHelpers.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

#include "../../Core/Seeding/StrawHitGeneratorHelper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <TFile.h>
#include <TTree.h>
#include <TTreeReader.h>
#include <TTreeReaderArray.h>
#include <TTreeReaderValue.h>
#include <cuda_runtime.h>

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
      spacePoints.setId(hitIndex, 0u);
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

/// Run CUDA and serialize the common, analysis-ready four-tree ROOT output.
void runEtaValidation(EtaValidationBatch& batch,
                      const std::string& validationName,
                      const std::filesystem::path& outputPath,
                      const Acts::HoughTransformUtils::HoughAxisRanges&
                          axisRanges) {
  // 1. Validate truth metadata before launching CUDA so data-preparation
  // failures remain distinct from transform failures.
  constexpr std::uint32_t minimumSeedHits = 4u;
  // Storage accommodates the algorithm-specific limits of three sliding-window
  // peaks and four relative-NMS peaks per bucket.
  constexpr std::size_t maximumCapacityPerBucket = 8u;

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

  CudaHT::CudaHoughPlaneBatch plane{{64u, 32u}, nBuckets};

  BOOST_TEST_MESSAGE("Running " << validationName << " with " << plane.nBinsX()
                                << " x " << plane.nBinsY() << " bins for "
                                << nBuckets << " buckets using the "
                                << peakFinderName << " peak finder");

  // 2. Run every physical bucket as one batch.
  auto maxima = [&]() {
    if (peakFinderName == "sliding-window") {
      return CudaHT::EtaHoughTransform::etaHoughTransform<
          maximumCapacityPerBucket, CudaHT::PeakFinder::SlidingWindow>(
          plane, batch.spacePoints, axisRanges);
    }
    if (peakFinderName == "relative-nms") {
      return CudaHT::EtaHoughTransform::etaHoughTransform<
          maximumCapacityPerBucket, CudaHT::PeakFinder::RelativeNms>(
          plane, batch.spacePoints, axisRanges);
    }

    return CudaHT::EtaHoughTransform::etaHoughTransform<
        maximumCapacityPerBucket, CudaHT::PeakFinder::GlobalMaximum>(
        plane, batch.spacePoints, axisRanges);
  }();

  // Full accumulator not saved
  maxima.moveToHost();
  maxima.copyAssociatedHitIndicesToHost();

  std::size_t totalMaxima = 0u;
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    totalMaxima += maxima.nMaxima(bucket);
  }
  const double averageMaximaPerBucket =
      nBuckets == 0u
          ? 0.0
          : static_cast<double>(totalMaxima) /
                static_cast<double>(nBuckets);
  BOOST_TEST_MESSAGE("Average Hough maxima per bucket: "
                     << averageMaximaPerBucket << " (" << totalMaxima
                     << " maxima in " << nBuckets << " buckets)");

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

}  // namespace

BOOST_AUTO_TEST_CASE(cuda_hough_eta_straw_generator_validation) {
  // Controlled end-to-end sample. This writes the same four ROOT trees as the
  // Particle Gun test, allowing one analysis program to process both outputs.
  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
    BOOST_TEST_MESSAGE("No CUDA device found, skipping CUDA runtime test");
    return;
  }

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
  // ACTS_MUON_CUDA_PEAK_FINDER selects "global", "sliding-window" or
  // "relative-nms" for both validation samples.
  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
    BOOST_TEST_MESSAGE("No CUDA device found, skipping CUDA runtime test");
    return;
  }

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
