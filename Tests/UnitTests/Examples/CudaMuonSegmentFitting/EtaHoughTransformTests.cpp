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
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsExamples/Framework/DataHandle.hpp"
#include "ActsExamples/Framework/WhiteBoard.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

#include <array>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include <TFile.h>
#include <TH1D.h>
#include <TTree.h>
#include <cuda_runtime.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>

#include "../../Core/Seeding/StrawHitGeneratorHelper.hpp"
using namespace ActsTests;

namespace {
struct EtaValidationTruth {
  double tanBeta = 0.0;
  double y0 = 0.0;
  double zReference = 0.0;
  std::uint32_t nGeneratedHits = 0u;
};

struct EtaValidationBatch {
  ActsExamples::CudaMuonSpacePointContainer spacePoints;
  std::vector<EtaValidationTruth> truth;
};

EtaValidationBatch makeGeneratedEtaValidationBatch(std::size_t nEvents,
                                                   RandomEngine& engine,
                                                   const Acts::Logger& logger,
                                                   double minimumTanBeta,
                                                   double maximumTanBeta) {
  MeasurementGenerator::Config generatorConfig{};
  generatorConfig.createStraws = true;
  generatorConfig.smearRadius = true;
  generatorConfig.twinStraw = false;
  generatorConfig.createStrips = false;

  std::vector<Container_t> eventMeasurements{};
  std::vector<EtaValidationTruth> truth{};

  eventMeasurements.reserve(nEvents);
  truth.reserve(nEvents);

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

    truth.push_back({tanBeta, y0, zReference,
                     static_cast<std::uint32_t>(measurements.size())});

    totalHits += measurements.size();
    eventMeasurements.push_back(std::move(measurements));
  }

  ActsExamples::CudaMuonSpacePointContainer spacePoints{totalHits};

  std::size_t hitIndex = 0u;

  for (std::size_t event = 0u; event < nEvents; ++event) {
    const std::size_t bucketStart = hitIndex;

    for (const auto& measurement : eventMeasurements[event]) {
      Acts::Vector3 position = measurement->localPosition();
      position.z() -= truth[event].zReference;

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
  }

  return {std::move(spacePoints), std::move(truth)};
}


struct FileEtaHit {
  Acts::GeometryIdentifier geometryId{};
  std::uint32_t muonId = 0u;
  Acts::Vector3 localPosition = Acts::Vector3::Zero();
  Acts::Vector3 sensorDirection = Acts::Vector3::Zero();
  Acts::Vector3 toNextSensor = Acts::Vector3::Zero();
  double driftRadius = 0.0;
  double time = 0.0;
  std::array<double, 3> covariance{};
  std::uint32_t logicalLayer = 0u;
};

constexpr std::size_t localY0Index = 0u;
constexpr std::size_t localThetaIndex = 1u;
constexpr std::size_t localPhiIndex = 3u;

EtaValidationBatch fileReadEtaValidation(
    const std::filesystem::path& inputPath,
    std::size_t maximumBuckets = std::numeric_limits<std::size_t>::max()) {
  using namespace Acts::UnitLiterals;

  TFile inputFile{inputPath.c_str(), "READ"};
  if (inputFile.IsZombie()) {
    throw std::runtime_error("Failed to open " + inputPath.string());
  }

  TTreeReader reader{"EtaValidationInput", &inputFile};
  if (reader.IsInvalid()) {
    throw std::runtime_error(
        "EtaValidationInput tree not found in " + inputPath.string());
  }

  TTreeReaderValue<UInt_t> eventId{reader, "event_id"};
  TTreeReaderValue<UInt_t> segmentIndex{reader, "segment_index"};
  TTreeReaderValue<Double_t> trueTanBeta{reader, "true_tanBeta"};
  TTreeReaderValue<Double_t> trueY0{reader, "true_y0"};

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
  TTreeReaderValue<UInt_t> logicalLayer{reader, "logical_layer"};

  std::vector<std::vector<FileEtaHit>> buckets;
  std::vector<EtaValidationTruth> truth;
  std::vector<FileEtaHit> currentHits;

  bool hasCurrentBucket = false;
  UInt_t currentEvent = 0u;
  UInt_t currentSegment = 0u;
  double currentTanBeta = 0.0;
  double currentY0 = 0.0;

  const auto flushBucket = [&]() {
    if (!hasCurrentBucket) {
      return;
    }

    if (currentHits.size() >= 3u && buckets.size() < maximumBuckets) {
      truth.push_back({currentTanBeta, currentY0, 0.0,
                       static_cast<std::uint32_t>(currentHits.size())});
      buckets.push_back(std::move(currentHits));
    }

    currentHits.clear();
  };

  const auto makeDirection = [](double phiDegrees, double thetaDegrees) {
    return Acts::makeDirectionFromPhiTheta<double>(
        phiDegrees * 1._degree, thetaDegrees * 1._degree);
  };

  while (reader.Next()) {
    const bool newBucket =
        !hasCurrentBucket || *eventId != currentEvent ||
        *segmentIndex != currentSegment;

    if (newBucket) {
      flushBucket();

      if (buckets.size() >= maximumBuckets) {
        break;
      }

      hasCurrentBucket = true;
      currentEvent = *eventId;
      currentSegment = *segmentIndex;
      currentTanBeta = *trueTanBeta;
      currentY0 = *trueY0;
    } else {
      if (std::abs(*trueTanBeta - currentTanBeta) > 1.0e-10 ||
          std::abs(*trueY0 - currentY0) > 1.0e-6) {
        throw std::runtime_error(
            "Truth parameters changed inside one event/segment group");
      }
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
    hit.logicalLayer = *logicalLayer;

    currentHits.push_back(std::move(hit));
  }

  if (buckets.size() < maximumBuckets) {
    flushBucket();
  }

  if (buckets.empty()) {
    throw std::runtime_error(
        "EtaValidationInput contains no bucket with at least three hits");
  }

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
      spacePoints.setLogicalLayer(outputHit, hit.logicalLayer);
      ++outputHit;
    }

    spacePoints.addBucket(bucketStart, outputHit);
  }

  if (spacePoints.bucketCount() != truth.size()) {
    throw std::runtime_error("Truth and CUDA bucket counts differ");
  }

  return {std::move(spacePoints), std::move(truth)};
}


}  // namespace

namespace ActsTests {

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

BOOST_AUTO_TEST_SUITE(CudaHoughTransformUtilsSuite)

namespace {

struct DriftCircleInput {
  double y;
  double z;
  double r;
  double uncert;
};

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
  return makeBatchedDriftCircleContainer(nBuckets, driftCircleInputs());
}

// Utility to save data to CSV  for python visualization
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
  static constexpr std::uint32_t fourBit = 0xFu;
  static constexpr std::uint32_t layerShift = 17u;

  return (rawId >> layerShift) & fourBit;
}

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

void writeHoughHistogramCsv(
    const std::filesystem::path& path, const CudaHT::CudaHoughPlaneBatch& plane,
    std::size_t bucket,
    const Acts::HoughTransformUtils::HoughAxisRanges& ranges,
    double trueTanBeta, double trueY0, double recoTanBeta, double recoY0) {
  std::ofstream out{path};
  BOOST_REQUIRE_MESSAGE(out, "Failed to open " << path.string());

  out << std::setprecision(17);
  out << "xBin,yBin,tanTheta,interceptY,nHits,nLayers,layerMask,"
         "trueTanBeta,trueY0,recoTanBeta,recoY0\n";

  for (std::size_t yBin = 0; yBin < plane.nBinsY(); ++yBin) {
    for (std::size_t xBin = 0; xBin < plane.nBinsX(); ++xBin) {
      const double tanTheta = Acts::HoughTransformUtils::binCenter(
          ranges.xMin, ranges.xMax, plane.nBinsX(), xBin);
      const double interceptY = Acts::HoughTransformUtils::binCenter(
          ranges.yMin, ranges.yMax, plane.nBinsY(), yBin);

      out << xBin << "," << yBin << "," << tanTheta << "," << interceptY << ","
          << plane.nHits(bucket, xBin, yBin) << ","
          << plane.nLayers(bucket, xBin, yBin) << ","
          << static_cast<unsigned long long>(
                 plane.layerMask(bucket, xBin, yBin))
          << "," << trueTanBeta << "," << trueY0 << "," << recoTanBeta << ","
          << recoY0 << "\n";
    }
  }
}

void writeBucketHitsCsv(
    const std::filesystem::path& path,
    const ActsExamples::CudaMuonSpacePointContainer& container,
    std::size_t bucket) {
  std::ofstream out{path};
  BOOST_REQUIRE_MESSAGE(out, "Failed to open " << path.string());

  const std::size_t start = container.bucketStart(bucket);
  const std::size_t end = container.bucketEnd(bucket);

  out << std::setprecision(17);
  out << "hitIndex,x,y,z,r,uncert,layer\n";

  for (std::size_t i = start; i < end; ++i) {
    auto sp = container[i];
    const Acts::Vector3& pos = sp->localPosition();
    const std::array<double, 3>& cov = sp->covariance();

    out << i - start << "," << pos.x() << "," << pos.y() << "," << pos.z()
        << "," << sp->driftRadius() << "," << std::sqrt(std::max(cov[1], 0.0))
        << "," << rawMuonIdLayer(container.muonId(i)) << "\n";
  }
}

void runEtaValidation(EtaValidationBatch& batch,
                      const std::string& validationName,
                      const std::filesystem::path& outputPath,
                      const std::filesystem::path& debugDirectory,
                      std::size_t numberErrorsToSave = 10u) {
  constexpr std::uint32_t minimumSeedHits = 4u;
  constexpr double minimumTanBeta = -3.0;
  constexpr double maximumTanBeta = 3.0;
  constexpr double matchingToleranceBins = 1.0;

  const std::size_t nEvents = batch.truth.size();

  BOOST_REQUIRE_GT(nEvents, 0u);
  BOOST_REQUIRE_GT(batch.spacePoints.size(), 0u);
  BOOST_REQUIRE_EQUAL(batch.spacePoints.bucketCount(), nEvents);

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      minimumTanBeta, maximumTanBeta, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  CudaHT::CudaHoughPlaneBatch plane{{64u, 32u}, nEvents};

  BOOST_TEST_MESSAGE("Running " << validationName << " with " << plane.nBinsX()
                                << " x " << plane.nBinsY() << " bins for "
                                << nEvents << " buckets");

  // Input reading and container construction happen before this helper, so they
  // are excluded from this timing.
  const auto start = std::chrono::steady_clock::now();

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1>(
      plane, batch.spacePoints, axisRanges);

  // This synchronizes the Hough work and copies the compact maximum results.
  maxima.moveToHost();

  const auto stop = std::chrono::steady_clock::now();
  const double elapsedSeconds =
      std::chrono::duration<double>(stop - start).count();

  // The full plane is copied only for validation and failed-event CSV output.
  // It is deliberately excluded from the performance timing above.
  plane.moveToHost();

  const double bucketsPerSecond =
      elapsedSeconds > 0.0 ? static_cast<double>(nEvents) / elapsedSeconds : 0.0;

  BOOST_TEST_MESSAGE("  Hough runtime: " << 1000.0 * elapsedSeconds << " ms");
  BOOST_TEST_MESSAGE("  Throughput: " << bucketsPerSecond << " buckets/s");

  const std::string outputFileName = outputPath.string();
  TFile outputFile{outputFileName.c_str(), "RECREATE"};
  BOOST_REQUIRE_MESSAGE(!outputFile.IsZombie(),
                        "Failed to create " << outputPath.string());

  auto outTree = std::make_unique<TTree>("EtaHoughTree", "MonitorTree");
  outTree->SetDirectory(nullptr);

  std::uint32_t eventNumber = 0u;
  std::uint32_t nGeneratedHits = 0u;
  std::uint32_t nAssociatedHits = 0u;
  double nLayers = 0.0;
  double trueTanBeta = 0.0;
  double trueY0 = 0.0;
  double recoTanBeta = 0.0;
  double recoY0 = 0.0;
  char foundMaximum = 0;
  char enoughHits = 0;
  char converged = 0;

  outTree->Branch("eventNumber", &eventNumber);
  outTree->Branch("nGeneratedHits", &nGeneratedHits);
  outTree->Branch("nAssociatedHits", &nAssociatedHits);
  outTree->Branch("nLayers", &nLayers);
  outTree->Branch("trueTanBeta", &trueTanBeta);
  outTree->Branch("trueY0", &trueY0);
  outTree->Branch("recoTanBeta", &recoTanBeta);
  outTree->Branch("recoY0", &recoY0);
  outTree->Branch("foundMaximum", &foundMaximum);
  outTree->Branch("enoughHits", &enoughHits);
  outTree->Branch("converged", &converged);

  std::size_t maximumEvents = 0u;
  std::size_t enoughHitEvents = 0u;
  std::size_t reconstructedEvents = 0u;
  std::size_t tanBetaMatchedEvents = 0u;
  std::size_t y0MatchedEvents = 0u;
  std::size_t failedTanBetaOnly = 0u;
  std::size_t failedY0Only = 0u;
  std::size_t failedBoth = 0u;
  std::size_t savedErrors = 0u;

  const double missingValue = std::numeric_limits<double>::quiet_NaN();

  std::filesystem::remove_all(debugDirectory);
  if (numberErrorsToSave > 0u) {
    std::filesystem::create_directories(debugDirectory);
  }

  for (std::size_t event = 0u; event < nEvents; ++event) {
    eventNumber = static_cast<std::uint32_t>(event);
    nGeneratedHits = batch.truth[event].nGeneratedHits;
    trueTanBeta = batch.truth[event].tanBeta;
    trueY0 = batch.truth[event].y0;

    nAssociatedHits = 0u;
    nLayers = 0.0;
    recoTanBeta = missingValue;
    recoY0 = missingValue;
    foundMaximum = maxima.nMaxima(event) > 0u ? 1 : 0;
    enoughHits = 0;
    converged = 0;

    if (foundMaximum != 0) {
      ++maximumEvents;

      constexpr std::size_t maximumId = 0u;

      recoTanBeta = maxima.tanBeta(event, maximumId);
      recoY0 = maxima.interceptY(event, maximumId);
      nAssociatedHits =
          static_cast<std::uint32_t>(maxima.nAssociatedHits(event, maximumId));
      nLayers = maxima.nLayers(event, maximumId);

      enoughHits = nAssociatedHits >= minimumSeedHits &&
                   nLayers >= static_cast<double>(minimumSeedHits);
      enoughHitEvents += enoughHits != 0;

      const auto ranges = plane.bucketAxisRanges(event, axisRanges);
      const double tanBetaBinWidth =
          (ranges.xMax - ranges.xMin) / static_cast<double>(plane.nBinsX());
      const double y0BinWidth =
          (ranges.yMax - ranges.yMin) / static_cast<double>(plane.nBinsY());

      const double tanBetaError = recoTanBeta - trueTanBeta;
      const double y0Error = recoY0 - trueY0;
      const double tanBetaErrorBins = tanBetaError / tanBetaBinWidth;
      const double y0ErrorBins = y0Error / y0BinWidth;

      const bool tanBetaMatched =
          std::abs(tanBetaErrorBins) <= matchingToleranceBins;
      const bool y0Matched = std::abs(y0ErrorBins) <= matchingToleranceBins;
      const bool truthMatched = tanBetaMatched && y0Matched;

      tanBetaMatchedEvents += tanBetaMatched;
      y0MatchedEvents += y0Matched;

      if (!tanBetaMatched && !y0Matched) {
        ++failedBoth;
      } else if (!tanBetaMatched) {
        ++failedTanBetaOnly;
      } else if (!y0Matched) {
        ++failedY0Only;
      }

      converged = enoughHits != 0 && truthMatched;
      reconstructedEvents += converged != 0;

      if (enoughHits != 0 && !truthMatched &&
          savedErrors < numberErrorsToSave) {
        const std::string prefix = "case_" + std::to_string(savedErrors) +
                                   "_bucket_" + std::to_string(event);

        const auto hitsPath = debugDirectory / (prefix + "_hits.csv");
        const auto histogramPath = debugDirectory / (prefix + "_histogram.csv");

        writeBucketHitsCsv(hitsPath, batch.spacePoints, event);
        writeHoughHistogramCsv(histogramPath, plane, event, ranges, trueTanBeta,
                               trueY0, recoTanBeta, recoY0);

        BOOST_TEST_MESSAGE(
            "Saved failed bucket " << event
            << ": true tanBeta=" << trueTanBeta
            << ", reco tanBeta=" << recoTanBeta
            << ", error=" << tanBetaError << " (" << tanBetaErrorBins
            << " bins), true y0=" << trueY0 << ", reco y0=" << recoY0
            << ", error=" << y0Error << " (" << y0ErrorBins
            << " bins), hits=" << nAssociatedHits << ", layers=" << nLayers);

        ++savedErrors;
      }
    }

    outTree->Fill();
  }

  const auto efficiency = [](std::size_t passed, std::size_t total) {
    return total == 0u
               ? 0.0
               : 100.0 * static_cast<double>(passed) /
                     static_cast<double>(total);
  };

  BOOST_TEST_MESSAGE(validationName << " summary:");
  BOOST_TEST_MESSAGE("  Buckets: " << nEvents);
  BOOST_TEST_MESSAGE("  Maximum found: " << maximumEvents << "/" << nEvents
                                          << " ("
                                          << efficiency(maximumEvents, nEvents)
                                          << "%)");
  BOOST_TEST_MESSAGE("  Enough hits: " << enoughHitEvents << "/" << nEvents
                                       << " ("
                                       << efficiency(enoughHitEvents, nEvents)
                                       << "%)");
  BOOST_TEST_MESSAGE("  tanBeta matched: "
                     << tanBetaMatchedEvents << "/" << maximumEvents << " ("
                     << efficiency(tanBetaMatchedEvents, maximumEvents) << "%)");
  BOOST_TEST_MESSAGE("  y0 matched: " << y0MatchedEvents << "/" << maximumEvents
                                      << " ("
                                      << efficiency(y0MatchedEvents,
                                                    maximumEvents)
                                      << "%)");
  BOOST_TEST_MESSAGE("  Reconstructed: "
                     << reconstructedEvents << "/" << nEvents << " ("
                     << efficiency(reconstructedEvents, nEvents) << "%)");
  BOOST_TEST_MESSAGE("  Failed tanBeta only: " << failedTanBetaOnly);
  BOOST_TEST_MESSAGE("  Failed y0 only: " << failedY0Only);
  BOOST_TEST_MESSAGE("  Failed both: " << failedBoth);
  BOOST_TEST_MESSAGE("  Saved failed buckets: " << savedErrors << " in "
                                                << debugDirectory.string());

  BOOST_REQUIRE_GT(maximumEvents, 0u);
  BOOST_REQUIRE_GT(reconstructedEvents, 0u);
  BOOST_CHECK_EQUAL(outTree->GetEntries(), static_cast<Long64_t>(nEvents));

  outputFile.WriteObject(outTree.get(), outTree->GetName());
  outputFile.Close();

  BOOST_TEST_MESSAGE("Wrote " << outTree->GetEntries() << " entries to "
                              << outputPath.string() << ":EtaHoughTree");
}

}  // namespace

BOOST_AUTO_TEST_CASE(cuda_hough_eta_drift_circle_global_maximum) {
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

BOOST_AUTO_TEST_CASE(cuda_hough_eta_straw_generator_validation) {
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
      "CudaEtaHoughValidation.root",
      std::filesystem::current_path() / "eta_hough_generated_failed_events");
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_file_validation) {
  int deviceCount = 0;
  if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
    BOOST_TEST_MESSAGE("No CUDA device found, skipping CUDA runtime test");
    return;
  }

  const char* environmentPath =
      std::getenv("ACTS_MUON_VALIDATION_FLAT_ROOT");

  const std::filesystem::path inputPath =
      environmentPath != nullptr ? environmentPath : "EtaHoughFlatInput.root";

  BOOST_REQUIRE_MESSAGE(
      std::filesystem::exists(inputPath),
      "Flat validation file not found: " << inputPath
      << ". Run flatten_eta_validation.py first or set "
         "ACTS_MUON_VALIDATION_FLAT_ROOT.");

  // File reading and CUDA container construction are intentionally excluded
  // from the performance timing inside runEtaValidation().
  auto loaded = fileReadEtaValidation(inputPath);

  runEtaValidation(
      loaded,
      "File Eta Hough validation",
      "CudaEtaHoughFileValidation.root",
      std::filesystem::current_path() / "eta_hough_file_failed_events");
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
