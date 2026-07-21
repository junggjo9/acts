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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <vector>

#include <TFile.h>
#include <TH1D.h>

#include "../../Core/Seeding/StrawHitGeneratorHelper.hpp"

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
    std::size_t nBuckets,
    const std::vector<DriftCircleInput>& driftCircles) {
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
          Acts::Vector3{1.0, 0.0, 0.0},
          Acts::Vector3{0.0, 1.0, 0.0});

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

  CudaHT::CudaHoughPlaneBatch plane{{25, 15}, 1};

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

BOOST_AUTO_TEST_CASE(cuda_hough_eta_associates_all_line_hits) {
  auto spacePoints = makeBatchedDriftCircleContainer(1);

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -0.5, 0.5, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  CudaHT::CudaHoughPlaneBatch plane{{10, 15}, 1};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1>(
      plane, spacePoints, axisRanges);

  maxima.copyAssociatedHitIndicesToHost();

  BOOST_REQUIRE_EQUAL(maxima.nMaxima(0u), 1u);
  BOOST_REQUIRE_EQUAL(maxima.nAssociatedHits(0u, 0u), 6u);
  BOOST_CHECK_EQUAL(maxima.totalAssociatedHits(), 6u);

  const auto associatedSpan = maxima.associatedHitIndices(0u, 0u);

  std::vector<std::uint32_t> associatedHits{
      associatedSpan.begin(), associatedSpan.end()};

  std::sort(associatedHits.begin(), associatedHits.end());

  const std::vector<std::uint32_t> expectedHits{
      0u, 1u, 2u, 3u, 4u, 5u};

  BOOST_CHECK_EQUAL_COLLECTIONS(
      associatedHits.begin(), associatedHits.end(),
      expectedHits.begin(), expectedHits.end());
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_rejects_non_line_hits) {
  std::vector<DriftCircleInput> inputs = driftCircleInputs();

  constexpr double uncert = 0.3;

  // These hits remain reasonably close to the original chamber region, but
  // neither drift-circle solution intersects the selected maximum cell.
  inputs.push_back({-440.0, 0.0, 1.0, uncert});
  inputs.push_back({-415.0, 0.0, 1.0, uncert});

  auto spacePoints = makeBatchedDriftCircleContainer(1, inputs);

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -0.5, 0.5, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  CudaHT::CudaHoughPlaneBatch plane{{10, 15}, 1};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1>(
      plane, spacePoints, axisRanges);

  maxima.copyAssociatedHitIndicesToHost();

  BOOST_REQUIRE_EQUAL(maxima.nMaxima(0u), 1u);
  BOOST_REQUIRE_EQUAL(maxima.nAssociatedHits(0u, 0u), 6u);
  BOOST_CHECK_EQUAL(maxima.totalAssociatedHits(), 6u);

  const auto associatedSpan = maxima.associatedHitIndices(0u, 0u);

  std::vector<std::uint32_t> associatedHits{
      associatedSpan.begin(), associatedSpan.end()};

  std::sort(associatedHits.begin(), associatedHits.end());

  const std::vector<std::uint32_t> expectedHits{
      0u, 1u, 2u, 3u, 4u, 5u};

  BOOST_CHECK_EQUAL_COLLECTIONS(
      associatedHits.begin(), associatedHits.end(),
      expectedHits.begin(), expectedHits.end());
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
