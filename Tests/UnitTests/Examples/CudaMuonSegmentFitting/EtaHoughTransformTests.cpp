// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/test/unit_test.hpp>

#include "Acts/Definitions/Units.hpp"
#include "Acts/Seeding/HoughTransformUtils.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string_view>
#include <vector>

namespace ActsTests {

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

BOOST_AUTO_TEST_SUITE(CudaHoughTransformUtilsSuite)

namespace {

struct DriftCircleInput {
  double y;
  double z;
  double r;
  double uncertainty;
};

std::vector<DriftCircleInput> driftCircleInputs() {
  constexpr double uncertainty = 0.3;
  return {
      {-427.981, -225.541, 14.5202, uncertainty},
      {-412.964, -199.530, 1.66237, uncertainty},
      {-427.981, -173.519, 12.3176, uncertainty},
      {-427.981, 173.519, 1.5412, uncertainty},
      {-442.999, 199.530, 12.3937, uncertainty},
      {-427.981, 225.541, 3.77967, uncertainty},
  };
}

ActsExamples::CudaMuonSpacePointContainer makeBatchedDriftCircleContainer(
    std::size_t nBuckets, const std::vector<DriftCircleInput>& driftCircles) {
  const std::size_t hitsPerBucket = driftCircles.size();
  ActsExamples::CudaMuonSpacePointContainer container{nBuckets * hitsPerBucket};

  for (std::size_t bucket = 0u; bucket < nBuckets; ++bucket) {
    const double bucketYOffset = 10.0 * static_cast<double>(bucket);
    const std::size_t start = bucket * hitsPerBucket;
    const std::size_t end = start + hitsPerBucket;

    for (std::size_t local = 0u; local < hitsPerBucket; ++local) {
      const std::size_t index = start + local;
      const DriftCircleInput& driftCircle = driftCircles[local];

      container.setGeometryId(index, index);
      container.setId(index, 0u);
      container.defineCoordinates(
          index,
          Acts::Vector3{0.0, driftCircle.y + bucketYOffset, driftCircle.z},
          Acts::Vector3{1.0, 0.0, 0.0}, Acts::Vector3{0.0, 1.0, 0.0});
      container.setRadius(index, driftCircle.r);
      container.setTime(index, 0.0);
      container.setCovariance(index,
                              driftCircle.uncertainty * driftCircle.uncertainty,
                              driftCircle.uncertainty * driftCircle.uncertainty,
                              0.0);
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

struct StraightSegmentInput {
  double tanBeta;
  double y0;
};

constexpr std::array<StraightSegmentInput, 3u> threeSegmentInputs{{
    {-2.0, -600.0},
    {0.0, 0.0},
    {2.0, 600.0},
}};

constexpr std::size_t hitsPerSegment = 4u;

ActsExamples::CudaMuonSpacePointContainer makeThreeSegmentContainer() {
  constexpr std::array<double, hitsPerSegment> zPositions{-90.0, -30.0, 30.0,
                                                           90.0};
  constexpr double uncertainty = 0.1;
  constexpr std::size_t nHits =
      threeSegmentInputs.size() * zPositions.size();

  ActsExamples::CudaMuonSpacePointContainer container{nHits};
  std::size_t hit = 0u;

  for (const StraightSegmentInput& segment : threeSegmentInputs) {
    for (std::size_t layer = 0u; layer < zPositions.size(); ++layer) {
      const double z = zPositions[layer];
      const double y = segment.tanBeta * z + segment.y0;

      container.setGeometryId(hit, hit);
      container.setId(hit, 0u);
      container.defineCoordinates(hit, Acts::Vector3{0.0, y, z},
                                  Acts::Vector3{1.0, 0.0, 0.0},
                                  Acts::Vector3{0.0, 1.0, 0.0});
      container.setRadius(hit, 0.0);
      container.setTime(hit, 0.0);
      container.setCovariance(hit, uncertainty * uncertainty,
                              uncertainty * uncertainty, 0.0);
      container.setLogicalLayer(hit, static_cast<std::uint32_t>(layer));
      ++hit;
    }
  }

  container.addBucket(0u, nHits);
  return container;
}

std::uint32_t rawMuonIdLayer(std::uint32_t rawId) {
  constexpr std::uint32_t fourBit = 0xFu;
  constexpr std::uint32_t layerShift = 17u;
  return (rawId >> layerShift) & fourBit;
}

void writeFirstBucketHitsCsv(
    const std::filesystem::path& path,
    const ActsExamples::CudaMuonSpacePointContainer& container) {
  std::ofstream out{path};
  BOOST_REQUIRE_MESSAGE(out, "Failed to open " << path.string());
  BOOST_REQUIRE_GT(container.bucketCount(), 0u);

  const std::size_t start = container.bucketStart(0u);
  const std::size_t end = container.bucketEnd(0u);

  out << std::setprecision(17);
  out << "hitIndex,x,y,z,r,uncert,layer\n";

  for (std::size_t hit = start; hit < end; ++hit) {
    const auto spacePoint = container[hit];
    const Acts::Vector3& position = spacePoint->localPosition();
    const std::array<double, 3>& covariance = spacePoint->covariance();
    const double uncertainty = std::sqrt(std::max(covariance[1], 0.0));

    out << (hit - start) << "," << position.x() << "," << position.y() << ","
        << position.z() << "," << spacePoint->driftRadius() << ","
        << uncertainty << "," << rawMuonIdLayer(container.muonId(hit)) << "\n";
  }
}

template <typename maximum_batch_t>
std::size_t countMatchedSegments(const maximum_batch_t& maxima) {
  std::size_t matchedSegments = 0u;

  for (std::size_t segment = 0u; segment < threeSegmentInputs.size();
       ++segment) {
    bool foundSegment = false;
    for (std::size_t maximum = 0u; maximum < maxima.nMaxima(0u); ++maximum) {
      const auto associated = maxima.associatedHitIndices(0u, maximum);
      bool containsAllHits = true;
      for (std::size_t localHit = 0u; localHit < hitsPerSegment; ++localHit) {
        const std::uint32_t expectedHit = static_cast<std::uint32_t>(
            segment * hitsPerSegment + localHit);
        containsAllHits =
            containsAllHits &&
            std::ranges::find(associated, expectedHit) != associated.end();
      }
      foundSegment = foundSegment || containsAllHits;
    }
    matchedSegments += foundSegment ? 1u : 0u;
  }

  return matchedSegments;
}

}  // namespace

BOOST_AUTO_TEST_CASE(cuda_hough_eta_drift_circle_global_maximum) {
  auto spacePoints = makeBatchedDriftCircleContainer(1u);

  constexpr double expectedTanTheta = -0.0401472 / 0.994974;
  constexpr double expectedInterceptY = -422.612;
  constexpr std::size_t bucketId = 0u;
  constexpr std::size_t maximumId = 0u;

  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};
  CudaHT::CudaHoughPlaneBatch plane{{15u, 15u}, 1u};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1u>(
      plane, spacePoints, axisRanges);
  maxima.moveToHost();
  plane.moveToHost();

  BOOST_REQUIRE_EQUAL(maxima.nBuckets(), 1u);
  BOOST_REQUIRE_EQUAL(maxima.nMaxima(bucketId), 1u);

  const std::size_t xBin = maxima.xBin(bucketId, maximumId);
  const std::size_t yBin = maxima.yBin(bucketId, maximumId);
  const double foundTanTheta = maxima.tanBeta(bucketId, maximumId);
  const double foundInterceptY = maxima.interceptY(bucketId, maximumId);
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

  const std::filesystem::path hitsCsv =
      std::filesystem::current_path() / "cuda_hough_visual_hits.csv";
  writeFirstBucketHitsCsv(hitsCsv, spacePoints);
  BOOST_TEST_MESSAGE("Wrote Hough visual debug hits to: " << hitsCsv.string());
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_counts_overlapping_solutions_once) {
  auto spacePoints = makeBatchedDriftCircleContainer(
      1u, std::vector<DriftCircleInput>{{0.0, 0.0, 0.0, 0.0}});
  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -1.0, 1.0, -1.0, 1.0};
  CudaHT::CudaHoughPlaneBatch plane{{1u, 1u}, 1u};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<1u>(
      plane, spacePoints, axisRanges);
  maxima.moveToHost();
  maxima.copyAssociatedHitIndicesToHost();

  BOOST_REQUIRE_EQUAL(maxima.nMaxima(0u), 1u);
  BOOST_CHECK_EQUAL(maxima.nHits(0u, 0u), 2.0f);
  BOOST_CHECK_EQUAL(maxima.nAssociatedHits(0u, 0u), 1u);
  const auto associated = maxima.associatedHitIndices(0u, 0u);
  BOOST_REQUIRE_EQUAL(associated.size(), 1u);
  BOOST_CHECK_EQUAL(associated.front(), 0u);
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_sliding_window_applies_threshold) {
  auto spacePoints = makeBatchedDriftCircleContainer(
      1u, std::vector<DriftCircleInput>{{0.0, 0.0, 0.0, 0.0}});
  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -1.0, 1.0, -1.0, 1.0};
  CudaHT::CudaHoughPlaneBatch plane{{5u, 5u}, 1u};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<
      8u, CudaHT::PeakFinder::SlidingWindow>(plane, spacePoints, axisRanges);
  maxima.moveToHost();
  BOOST_CHECK_EQUAL(maxima.nMaxima(0u), 0u);
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_sliding_window_finds_peak) {
  auto spacePoints = makeBatchedDriftCircleContainer(1u);
  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};
  CudaHT::CudaHoughPlaneBatch plane{{15u, 15u}, 1u};

  auto maxima = CudaHT::EtaHoughTransform::etaHoughTransform<
      8u, CudaHT::PeakFinder::SlidingWindow>(plane, spacePoints, axisRanges);
  maxima.moveToHost();
  BOOST_REQUIRE_EQUAL(maxima.nMaxima(0u), 1u);
  BOOST_CHECK_GE(maxima.nHits(0u, 0u), 3.0f);
}

BOOST_AUTO_TEST_CASE(cuda_hough_eta_three_segments_all_peak_finders) {
  struct PeakFinderCase {
    CudaHT::PeakFinder peakFinder;
    std::string_view name;
    std::size_t expectedMaxima;
    std::size_t expectedMatchedSegments;
  };

  constexpr std::array<PeakFinderCase, 3u> cases{{
      {CudaHT::PeakFinder::GlobalMaximum, "global", 1u, 1u},
      {CudaHT::PeakFinder::SlidingWindow, "sliding-window", 3u, 2u}, // SlidingWindow does not preform well and only finds 2
      {CudaHT::PeakFinder::RelativeNms, "relative-nms", 3u, 3u},
  }};
  const Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
      -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
      100.0 * Acts::UnitConstants::m};

  for (const PeakFinderCase& testCase : cases) {
    auto spacePoints = makeThreeSegmentContainer();
    CudaHT::CudaHoughPlaneBatch plane{{64u, 32u}, 1u};
    auto maxima = [&]() {
      if (testCase.peakFinder == CudaHT::PeakFinder::SlidingWindow) {
        return CudaHT::EtaHoughTransform::etaHoughTransform<
            4u, CudaHT::PeakFinder::SlidingWindow>(plane, spacePoints,
                                                    axisRanges);
      }
      if (testCase.peakFinder == CudaHT::PeakFinder::RelativeNms) {
        return CudaHT::EtaHoughTransform::etaHoughTransform<
            4u, CudaHT::PeakFinder::RelativeNms>(plane, spacePoints,
                                                  axisRanges);
      }
      return CudaHT::EtaHoughTransform::etaHoughTransform<
          4u, CudaHT::PeakFinder::GlobalMaximum>(plane, spacePoints,
                                                  axisRanges);
    }();

    maxima.moveToHost();
    maxima.copyAssociatedHitIndicesToHost();

    BOOST_TEST_CONTEXT("peak finder: " << testCase.name) {
      BOOST_REQUIRE_EQUAL(maxima.nMaxima(0u), testCase.expectedMaxima);
      BOOST_CHECK_EQUAL(countMatchedSegments(maxima),
                        testCase.expectedMatchedSegments);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
