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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <TFile.h>
#include <TH1D.h>
#include <TTree.h>
#include <TTreeReader.h>
#include <TTreeReaderArray.h>
#include <TTreeReaderValue.h>
#include <cuda_runtime.h>

#include "../../Core/Seeding/StrawHitGeneratorHelper.hpp"
using namespace ActsTests;

namespace {
/// One truth line and the bucket-local hits assigned to it by preprocessing.
struct EtaValidationTruth {
  std::uint32_t validationBucketId = 0u;
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint32_t segmentIndex = 0u;
  double tanBeta = 0.0;
  double y0 = 0.0;
  std::vector<std::uint32_t> truthHitIndices{};
};

/// Lightweight metadata for one physical Hough input bucket.
struct EtaValidationBucket {
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint16_t nTruthSegments = 0u;
};

/// Complete input to one validation run, shared by generated and file data.
struct EtaValidationBatch {
  ActsExamples::CudaMuonSpacePointContainer spacePoints;
  std::vector<EtaValidationBucket> buckets;
  std::vector<EtaValidationTruth> truth;
};

/// Generator-only values used while constructing the common validation batch.
struct GeneratedEtaTruth {
  double tanBeta = 0.0;
  double y0 = 0.0;
  double zReference = 0.0;
  std::uint32_t nGeneratedHits = 0u;
};

/// Generate simple straw events and convert them to the common validation form.
EtaValidationBatch makeGeneratedEtaValidationBatch(std::size_t nEvents,
                                                   RandomEngine& engine,
                                                   const Acts::Logger& logger,
                                                   double minimumTanBeta,
                                                   double maximumTanBeta) {
  // 1. Generate smeared straw measurements and retain only straight lines
  // inside the Hough transform's tanBeta range.
  MeasurementGenerator::Config generatorConfig{};
  generatorConfig.createStraws = true;
  generatorConfig.smearRadius = true;
  generatorConfig.twinStraw = false;
  generatorConfig.createStrips = false;

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

    measurements.erase(std::remove_if(measurements.begin(), measurements.end(),
                                      [](const auto& measurement) {
                                        return !measurement->isStraw();
                                      }),
                       measurements.end());

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

    // Intercept at z' = 0 after transforming z' = z - zReference.
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

  // 2. Flatten generated events into the contiguous, bucketed space-point
  // representation consumed by the CUDA implementation.
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

    // 3. There is no background in this sample, so every local bucket hit is
    // attached to its single generated truth segment.
    std::vector<std::uint32_t> truthHitIndices(
        generatedTruth[event].nGeneratedHits);
    std::iota(truthHitIndices.begin(), truthHitIndices.end(), 0u);

    buckets.push_back(
        {static_cast<std::uint32_t>(event), 0u, 1u});
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

  // ROOT model: TFile is the outer container, TTree is a column-oriented table,
  // and each branch is one named column. TTreeReader advances through rows;
  // TTreeReaderValue exposes the current scalar value of a bound branch.
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

namespace ActsTests {

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

BOOST_AUTO_TEST_SUITE(CudaHoughTransformUtilsSuite)

namespace {

/// Minimal, deterministic measurement used to test exact accumulator content.
struct DriftCircleInput {
  double y;
  double z;
  double r;
  double uncert;
};

/// Known hit pattern used by the deterministic global-maximum test.
std::vector<DriftCircleInput> driftCircleInputs() {
  constexpr double uncert = 0.3;

  return {
      {-427.981, -225.541, 14.5202, uncert},
      {-412.964, -199.530, 1.66237, uncert},
      {-427.981, -173.519, 12.3176, uncert},
      {-427.981, 173.519, 1.5412, uncert},
      {-442.999, 199.530, 12.3937, uncert},
      {-427.981, 225.541, 3.77967, uncert},
  };
}

ActsExamples::CudaMuonSpacePointContainer makeBatchedDriftCircleContainer(
    std::size_t nBuckets, const std::vector<DriftCircleInput>& driftCircles) {
  // Repeat the same known pattern with a y offset. This checks that batched
  // buckets remain independent inside one CUDA invocation.
  const std::size_t hitsPerBucket = driftCircles.size();

  ActsExamples::CudaMuonSpacePointContainer container{nBuckets * hitsPerBucket};

  for (std::size_t bucket = 0; bucket < nBuckets; ++bucket) {
    const double bucketYOffset = 10.0 * static_cast<double>(bucket);

    const std::size_t start = bucket * hitsPerBucket;
    const std::size_t end = start + hitsPerBucket;

    for (std::size_t local = 0; local < hitsPerBucket; ++local) {
      const std::size_t index = start + local;
      const auto& dc = driftCircles[local];

      container.setGeometryId(index, index);
      container.setId(index, 0u);

      container.defineCoordinates(
          index, Acts::Vector3{0.0, dc.y + bucketYOffset, dc.z},
          Acts::Vector3{1.0, 0.0, 0.0}, Acts::Vector3{0.0, 1.0, 0.0});

      container.setRadius(index, dc.r);
      container.setTime(index, 0.0);
      container.setCovariance(index, dc.uncert * dc.uncert,
                              dc.uncert * dc.uncert, 0.0);

      container.setLogicalLayer(index, static_cast<std::uint32_t>(local));
    }

    container.addBucket(start, end);
  }

  return container;
}

ActsExamples::CudaMuonSpacePointContainer makeBatchedDriftCircleContainer(
    std::size_t nBuckets) {
  // Convenience overload using the standard deterministic hit pattern.
  return makeBatchedDriftCircleContainer(nBuckets, driftCircleInputs());
}

// Export the small deterministic accumulator for external visual inspection.
// This helper is not used for the large validation samples.
void writeHoughHistogramCsv(
    const std::filesystem::path& path, const CudaHT::CudaHoughPlaneBatch& plane,
    const Acts::HoughTransformUtils::HoughAxisRanges& axisRanges) {
  std::ofstream out{path};
  BOOST_REQUIRE_MESSAGE(out, "Failed to open " << path.string());

  out << std::setprecision(17);
  out << "xBin,yBin,tanTheta,interceptY,nHits,nLayers,layerMask\n";

  // Only one bucket that is 0th
  std::uint32_t bucketId = 0;

  for (std::size_t yBin = 0; yBin < plane.nBinsY(); ++yBin) {
    for (std::size_t xBin = 0; xBin < plane.nBinsX(); ++xBin) {
      const double tanTheta = Acts::HoughTransformUtils::binCenter(
          axisRanges.xMin, axisRanges.xMax, plane.nBinsX(), xBin);
      const double interceptY = Acts::HoughTransformUtils::binCenter(
          axisRanges.yMin, axisRanges.yMax, plane.nBinsY(), yBin);

      out << xBin << "," << yBin << "," << tanTheta << "," << interceptY << ","
          << plane.nHits(bucketId, xBin, yBin) << ","
          << plane.nLayers(bucketId, xBin, yBin) << ","
          << static_cast<unsigned long long>(
                 plane.layerMask(bucketId, xBin, yBin))
          << "\n";
    }
  }
}

std::uint32_t rawMuonIdLayer(std::uint32_t rawId) {
  // Particle Gun muon IDs encode the zero-based logical layer in bits 17--20.
  static constexpr std::uint32_t fourBit = 0xFu;
  static constexpr std::uint32_t layerShift = 17u;

  return (rawId >> layerShift) & fourBit;
}

/// Export first-bucket measurements used by the deterministic visual check.
void writeFirstBucketHitsCsv(
    const std::filesystem::path& path,
    const ActsExamples::CudaMuonSpacePointContainer& container) {
  std::ofstream out{path};
  BOOST_REQUIRE_MESSAGE(out, "Failed to open " << path.string());

  BOOST_REQUIRE_GT(container.bucketCount(), 0u);

  const std::size_t bucketId = 0;
  const std::size_t start = container.bucketStart(bucketId);
  const std::size_t end = container.bucketEnd(bucketId);

  out << std::setprecision(17);
  out << "hitIndex,x,y,z,r,uncert,layer\n";

  for (std::size_t i = start; i < end; ++i) {
    auto sp = container[i];

    const Acts::Vector3& pos = sp->localPosition();
    const std::array<double, 3>& cov = sp->covariance();

    const double uncert = std::sqrt(std::max(cov[1], 0.0));

    out << (i - start) << "," << pos.x() << "," << pos.y() << "," << pos.z()
        << "," << sp->driftRadius() << "," << uncert << ","
        << rawMuonIdLayer(container.muonId(i)) << "\n";
  }
}

/// Run CUDA and serialize the common, analysis-ready four-tree ROOT output.
void runEtaValidation(EtaValidationBatch& batch,
                      const std::string& validationName,
                      const std::filesystem::path& outputPath) {
  // 1. Validate the common in-memory model before launching CUDA. These checks
  // make failures in data preparation distinct from failures in the transform.
  constexpr std::uint32_t minimumSeedHits = 4u;
  constexpr double minimumTanBeta = -3.0;
  constexpr double maximumTanBeta = 3.0;
  constexpr std::size_t maximumCapacityPerBucket = 16u;

  const std::size_t nBuckets = batch.buckets.size();

  BOOST_REQUIRE_GT(nBuckets, 0u);
  BOOST_REQUIRE_GT(batch.spacePoints.size(), 0u);
  BOOST_REQUIRE_EQUAL(batch.spacePoints.bucketCount(), nBuckets);

  std::vector<std::uint16_t> countedTruthSegments(nBuckets, 0u);
  for (const EtaValidationTruth& truth : batch.truth) {
    BOOST_REQUIRE_LT(truth.validationBucketId, nBuckets);
    ++countedTruthSegments[truth.validationBucketId];
  }
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    BOOST_REQUIRE_EQUAL(countedTruthSegments[bucket],
                        batch.buckets[bucket].nTruthSegments);
  }

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      minimumTanBeta, maximumTanBeta, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  CudaHT::CudaHoughPlaneBatch plane{{64u, 32u}, nBuckets};

  BOOST_TEST_MESSAGE("Running " << validationName << " with " << plane.nBinsX()
                                << " x " << plane.nBinsY() << " bins for "
                                << nBuckets << " buckets");

  // 2. Run every physical bucket as one batch. Capacity 16 keeps the schema
  // ready for peak finders that return multiple maxima per bucket.
  auto maxima =
      CudaHT::EtaHoughTransform::etaHoughTransform<maximumCapacityPerBucket>(
          plane, batch.spacePoints, axisRanges);

  // CUDA results live on the device. Copy only maxima and their associated hit
  // indices to host memory; the full accumulator is intentionally not saved.
  maxima.moveToHost();
  maxima.copyAssociatedHitIndicesToHost();

  const std::string outputFileName = outputPath.string();
  // 3. Create the ROOT output. RECREATE replaces an existing file. The four
  // TTrees behave like related tables keyed by bucketNumber:
  //   BucketTree: one row per bucket;
  //   HitTree: one row per input hit;
  //   TruthTree: one row per truth segment;
  //   MaximumTree: one row per returned Hough maximum.
  TFile outputFile{outputFileName.c_str(), "RECREATE"};
  BOOST_REQUIRE_MESSAGE(!outputFile.IsZombie(),
                        "Failed to create " << outputPath.string());

  auto bucketTree = std::make_unique<TTree>("EtaHoughBucketTree", "BucketTree");
  auto hitTree = std::make_unique<TTree>("EtaHoughHitTree", "HitTree");
  auto truthTree = std::make_unique<TTree>("EtaHoughTruthTree", "TruthTree");
  auto maximumTree = std::make_unique<TTree>("EtaHoughTree", "MaximumTree");
  // Keep explicit ownership in unique_ptr until WriteObject. Otherwise ROOT
  // would attach the trees to the currently open file and own their lifetime.
  bucketTree->SetDirectory(nullptr);
  hitTree->SetDirectory(nullptr);
  truthTree->SetDirectory(nullptr);
  maximumTree->SetDirectory(nullptr);
  std::uint32_t bucketNumber = 0u;
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint32_t nInputHits = 0u;
  std::uint16_t nTruthSegments = 0u;
  std::uint32_t nMaxima = 0u;

  // Branch binds a ROOT column to a C++ variable address. Each later Fill()
  // snapshots the variables' current values as one new tree row.
  bucketTree->Branch("bucketNumber", &bucketNumber);
  bucketTree->Branch("eventId", &eventId);
  bucketTree->Branch("sourceBucketId", &sourceBucketId);
  bucketTree->Branch("nInputHits", &nInputHits);
  bucketTree->Branch("nTruthSegments", &nTruthSegments);
  bucketTree->Branch("nMaxima", &nMaxima);

  std::uint32_t localHitIndex = 0u;
  double localX = 0.0;
  double localY = 0.0;
  double localZ = 0.0;
  double driftRadius = 0.0;
  double covarianceY = 0.0;
  std::uint32_t logicalLayer = 0u;

  // Hit-tree columns. bucketNumber + localHitIndex uniquely identify a hit.
  hitTree->Branch("bucketNumber", &bucketNumber);
  hitTree->Branch("localHitIndex", &localHitIndex);
  hitTree->Branch("localX", &localX);
  hitTree->Branch("localY", &localY);
  hitTree->Branch("localZ", &localZ);
  hitTree->Branch("driftRadius", &driftRadius);
  hitTree->Branch("covarianceY", &covarianceY);
  hitTree->Branch("logicalLayer", &logicalLayer);

  std::uint32_t truthNumber = 0u;
  std::uint32_t segmentIndex = 0u;
  std::uint32_t nTruthHits = 0u;
  double trueTanBeta = 0.0;
  double trueY0 = 0.0;
  std::vector<std::uint32_t> truthHitIndices{};

  // Truth-tree columns. std::vector creates a variable-length ROOT branch.
  truthTree->Branch("truthNumber", &truthNumber);
  truthTree->Branch("bucketNumber", &bucketNumber);
  truthTree->Branch("eventId", &eventId);
  truthTree->Branch("sourceBucketId", &sourceBucketId);
  truthTree->Branch("segmentIndex", &segmentIndex);
  truthTree->Branch("nTruthHits", &nTruthHits);
  truthTree->Branch("trueTanBeta", &trueTanBeta);
  truthTree->Branch("trueY0", &trueY0);
  truthTree->Branch("truthHitIndices", &truthHitIndices);

  std::uint32_t maximumId = 0u;
  std::uint32_t nAssociatedHits = 0u;
  double nHits = 0.0;
  double nLayers = 0.0;
  double recoTanBeta = 0.0;
  double recoY0 = 0.0;
  char enoughHits = 0;
  std::vector<std::uint32_t> associatedHitIndices{};

  // Maximum-tree columns, including bucket-local associated hit indices.
  maximumTree->Branch("bucketNumber", &bucketNumber);
  maximumTree->Branch("maximumId", &maximumId);
  maximumTree->Branch("nAssociatedHits", &nAssociatedHits);
  maximumTree->Branch("nHits", &nHits);
  maximumTree->Branch("nLayers", &nLayers);
  maximumTree->Branch("recoTanBeta", &recoTanBeta);
  maximumTree->Branch("recoY0", &recoY0);
  maximumTree->Branch("enoughHits", &enoughHits);
  maximumTree->Branch("associatedHitIndices", &associatedHitIndices);

  // 4. Fill bucket, hit, and maximum trees together while bucket-local ranges
  // and CUDA maximum indices are readily available.
  std::size_t totalMaxima = 0u;
  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    bucketNumber = static_cast<std::uint32_t>(bucket);
    eventId = batch.buckets[bucket].eventId;
    sourceBucketId = batch.buckets[bucket].sourceBucketId;
    nInputHits = static_cast<std::uint32_t>(
        batch.spacePoints.bucketEnd(bucket) -
        batch.spacePoints.bucketStart(bucket));
    nTruthSegments = batch.buckets[bucket].nTruthSegments;
    nMaxima = static_cast<std::uint32_t>(maxima.nMaxima(bucket));
    totalMaxima += nMaxima;
    bucketTree->Fill();

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
      hitTree->Fill();
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
      maximumTree->Fill();
    }

  }

  // 5. Truth is independent of the number of maxima, so fill its tree once
  // from the input truth collection after processing all buckets.
  for (const EtaValidationTruth& truth : batch.truth) {
    truthNumber = static_cast<std::uint32_t>(truthTree->GetEntries());
    bucketNumber = truth.validationBucketId;
    eventId = truth.eventId;
    sourceBucketId = truth.sourceBucketId;
    segmentIndex = truth.segmentIndex;
    nTruthHits = static_cast<std::uint32_t>(truth.truthHitIndices.size());
    trueTanBeta = truth.tanBeta;
    trueY0 = truth.y0;
    truthHitIndices = truth.truthHitIndices;
    truthTree->Fill();
  }

  BOOST_CHECK_EQUAL(bucketTree->GetEntries(), static_cast<Long64_t>(nBuckets));
  BOOST_CHECK_EQUAL(truthTree->GetEntries(),
                    static_cast<Long64_t>(batch.truth.size()));
  BOOST_CHECK_EQUAL(maximumTree->GetEntries(),
                    static_cast<Long64_t>(totalMaxima));
  // 6. Persist the four in-memory trees into the TFile and close it explicitly.
  outputFile.WriteObject(bucketTree.get(), bucketTree->GetName());
  outputFile.WriteObject(hitTree.get(), hitTree->GetName());
  outputFile.WriteObject(truthTree.get(), truthTree->GetName());
  outputFile.WriteObject(maximumTree.get(), maximumTree->GetName());
  outputFile.Close();

  BOOST_TEST_MESSAGE("Wrote validation data to " << outputPath.string());
}

}  // namespace

BOOST_AUTO_TEST_CASE(cuda_hough_eta_drift_circle_global_maximum) {
  // Small white-box test: compare the reported maximum with the maximum found
  // by inspecting the copied accumulator, then export both for visualization.
  auto spacePoints = makeBatchedDriftCircleContainer(1);

  constexpr double expectedTanTheta = -0.0401472 / 0.994974;
  constexpr double expectedInterceptY = -422.612;

  constexpr std::size_t bucketId = 0u;
  constexpr std::size_t maximumId = 0u;

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  CudaHT::CudaHoughPlaneBatch plane{{15, 15}, 1};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1>(
      plane, spacePoints, axisRanges);

  // Both objects remain device-backed after the transform.
  maxima.moveToHost();
  plane.moveToHost();

  BOOST_REQUIRE_EQUAL(maxima.nBuckets(), 1u);
  BOOST_REQUIRE_EQUAL(maxima.nMaxima(bucketId), 1u);

  const std::size_t xBin = maxima.xBin(bucketId, maximumId);
  const std::size_t yBin = maxima.yBin(bucketId, maximumId);

  const double foundTanTheta = maxima.tanBeta(bucketId, maximumId);

  const double foundInterceptY = maxima.interceptY(bucketId, maximumId);

  const auto [planeXBin, planeYBin] = plane.locMaxHits(bucketId);

  BOOST_TEST_MESSAGE("Maximum bin: x=" << xBin << ", y=" << yBin);
  BOOST_TEST_MESSAGE("Maximum parameters: tanTheta="
                     << foundTanTheta << ", interceptY=" << foundInterceptY);
  BOOST_TEST_MESSAGE("Maximum hits: " << maxima.nHits(bucketId, maximumId));
  BOOST_TEST_MESSAGE("Maximum layers: " << maxima.nLayers(bucketId, maximumId));

  BOOST_CHECK_EQUAL(xBin, planeXBin);
  BOOST_CHECK_EQUAL(yBin, planeYBin);

  BOOST_CHECK_EQUAL(maxima.nHits(bucketId, maximumId),
                    plane.nHits(bucketId, xBin, yBin));

  BOOST_CHECK_EQUAL(maxima.nLayers(bucketId, maximumId),
                    plane.nLayers(bucketId, xBin, yBin));

  BOOST_CHECK_EQUAL(maxima.layerMask(bucketId, maximumId),
                    plane.layerMask(bucketId, xBin, yBin));

  const auto bucketRanges = plane.bucketAxisRanges(bucketId, axisRanges);

  const double expectedBinTanTheta = Acts::HoughTransformUtils::binCenter(
      bucketRanges.xMin, bucketRanges.xMax, plane.nBinsX(), xBin);

  const double expectedBinInterceptY = Acts::HoughTransformUtils::binCenter(
      bucketRanges.yMin, bucketRanges.yMax, plane.nBinsY(), yBin);

  BOOST_CHECK_CLOSE(foundTanTheta, expectedBinTanTheta, 1.0e-10);

  BOOST_CHECK_CLOSE(foundInterceptY, expectedBinInterceptY, 1.0e-10);

  BOOST_CHECK_SMALL(std::abs(foundTanTheta - expectedTanTheta), 0.2);

  BOOST_CHECK_CLOSE(foundInterceptY, expectedInterceptY, 10.0);

  BOOST_CHECK_GE(maxima.nHits(bucketId, maximumId), 3.0f);

  BOOST_CHECK_GE(maxima.nLayers(bucketId, maximumId), 3.0f);

  const std::filesystem::path outputDirectory = std::filesystem::current_path();

  const std::filesystem::path hitsCsv =
      outputDirectory / "cuda_hough_visual_hits.csv";

  const std::filesystem::path histogramCsv =
      outputDirectory / "cuda_hough_visual_histogram.csv";

  writeFirstBucketHitsCsv(hitsCsv, spacePoints);
  writeHoughHistogramCsv(histogramCsv, plane, bucketRanges);

  BOOST_TEST_MESSAGE("Wrote Hough visual debug hits to: " << hitsCsv.string());

  BOOST_TEST_MESSAGE(
      "Wrote Hough visual debug histogram to: " << histogramCsv.string());
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_counts_overlapping_solutions_once) {
  // A zero-radius hit has two identical Hough solutions. Both solution fills
  // contribute to nHits, but the selected cell must associate the input hit
  // only once.
  auto spacePoints = makeBatchedDriftCircleContainer(
      1u, std::vector<DriftCircleInput>{{0.0, 0.0, 0.0, 0.0}});

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -1.0, 1.0, -1.0, 1.0};

  CudaHT::CudaHoughPlaneBatch plane{{1u, 1u}, 1u};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1u>(
      plane, spacePoints, axisRanges);

  maxima.moveToHost();
  plane.moveToHost();
  maxima.copyAssociatedHitIndicesToHost();

  BOOST_REQUIRE_EQUAL(maxima.nMaxima(0u), 1u);
  BOOST_CHECK_EQUAL(maxima.nHits(0u, 0u), 2.0f);
  BOOST_CHECK_EQUAL(plane.nHits(0u, 0u, 0u), 2.0f);
  BOOST_CHECK_EQUAL(maxima.nAssociatedHits(0u, 0u), 1u);

  const auto associated = maxima.associatedHitIndices(0u, 0u);
  BOOST_REQUIRE_EQUAL(associated.size(), 1u);
  BOOST_CHECK_EQUAL(associated.front(), 0u);
}

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

  RandomEngine engine{42u};

  auto generated = makeGeneratedEtaValidationBatch(
      nEvents, engine, *logger, -3.0, 3.0);

  runEtaValidation(
      generated,
      "Generated Eta Hough validation",
      "CudaEtaHoughValidation.root");
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_file_validation) {
  // Realistic end-to-end sample prepared by preprocessor_pg.py. The environment
  // variable permits the large flat ROOT input to live outside the build tree.
  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
    BOOST_TEST_MESSAGE("No CUDA device found, skipping CUDA runtime test");
    return;
  }

  const char* environmentPath =
      std::getenv("ACTS_MUON_VALIDATION_FLAT_ROOT");

  const std::filesystem::path inputPath =
      environmentPath != nullptr ? environmentPath : "EtaHoughFlatInput.root";

  if (std::filesystem::exists(inputPath)) {
      BOOST_TEST_MESSAGE("Flat validation file not found: " << inputPath
      << ". Run preprocessor_pg.py first or set "
         "ACTS_MUON_VALIDATION_FLAT_ROOT.");

      BOOST_TEST_MESSAGE("Test skipped.");
      return;
  }

  auto loaded = fileReadEtaValidation(inputPath);

  runEtaValidation(
      loaded,
      "File Eta Hough validation",
      "CudaEtaHoughFileValidation.root");
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
