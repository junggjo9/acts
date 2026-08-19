// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/test/unit_test.hpp>

#include "Acts/Definitions/Units.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/Algorithms/TrackFinding/PhiHoughTransform.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ranges>

namespace ActsTests {

namespace CudaHT = ActsExamples::CudaHoughTransformUtils;

BOOST_AUTO_TEST_SUITE(CudaPhiHoughTransformUtilsSuite)

namespace {

std::uint32_t stripMuonId(bool measuresEta, bool measuresPhi) {
  using MuonId = ActsExamples::MuonSpacePoint::MuonId;
  MuonId id{};
  id.setChamber(MuonId::StationName::BIS, MuonId::DetSide::A, 1u,
                MuonId::TechField::Rpc);
  id.setCoordFlags(measuresEta, measuresPhi);
  return id.toInt();
}

ActsExamples::CudaMuonSpacePointContainer makeEtaPhiSegment(
    std::size_t nPhiHits) {
  constexpr std::array<double, 4u> zPositions{-1500.0, -500.0, 500.0,
                                               1500.0};
  constexpr double tanBeta = 0.4;
  constexpr double interceptY = 10.0;
  constexpr double tanAlpha = 0.4;
  constexpr double interceptX = -20.0;
  constexpr double uncertainty = 0.1;

  ActsExamples::CudaMuonSpacePointContainer container{zPositions.size() + nPhiHits};
  const std::uint32_t etaId = stripMuonId(true, false);
  const std::uint32_t phiId = stripMuonId(false, true);
  const std::uint32_t etaPhiId = stripMuonId(true, true);

  for (std::size_t hit = 0u; hit < zPositions.size(); ++hit) {
    const double z = zPositions[hit];
    container.setGeometryId(hit, hit);
    container.setId(hit, etaId);
    container.defineCoordinates(
        hit, Acts::Vector3{0.0, tanBeta * z + interceptY, z},
        Acts::Vector3{1.0, 0.0, 0.0}, Acts::Vector3{0.0, 1.0, 0.0});
    container.setRadius(hit, 0.0);
    container.setTime(hit, 0.0);
    container.setCovariance(hit, uncertainty * uncertainty,
                            uncertainty * uncertainty, 0.0);
    container.setLogicalLayer(hit, static_cast<std::uint32_t>(hit));
  }

  for (std::size_t localHit = 0u; localHit < nPhiHits; ++localHit) {
    const std::size_t hit = zPositions.size() + localHit;
    const double z = zPositions[localHit];
    container.setGeometryId(hit, hit);
    // Exercise both Phi inputs: a dual-coordinate hit gated by the Eta
    // association and pure-Phi hits shared by all Eta maxima in the bucket.
    container.setId(hit, localHit == 0u ? etaPhiId : phiId);
    container.defineCoordinates(
        hit,
        Acts::Vector3{tanAlpha * z + interceptX,
                      tanBeta * z + interceptY, z},
        Acts::Vector3{0.0, 1.0, 0.0}, Acts::Vector3{1.0, 0.0, 0.0});
    container.setRadius(hit, 0.0);
    container.setTime(hit, 0.0);
    container.setCovariance(hit, uncertainty * uncertainty,
                            uncertainty * uncertainty, 0.0);
    container.setLogicalLayer(hit, static_cast<std::uint32_t>(localHit));
  }

  container.addBucket(0u, container.size());
  return container;
}

constexpr Acts::HoughTransformUtils::HoughAxisRanges axisRanges{
    -3.0, 3.0, -100.0 * Acts::UnitConstants::m,
    100.0 * Acts::UnitConstants::m};

}  // namespace

BOOST_AUTO_TEST_CASE(cuda_hough_phi_extends_eta_maximum) {
  auto spacePoints = makeEtaPhiSegment(4u);
  CudaHT::CudaHoughPlaneBatch etaPlane{{15u, 15u}, 1u};
  auto etaMaxima = CudaHT::EtaHoughTransform::etaHoughTransform<1u>(
      etaPlane, spacePoints, axisRanges);

  CudaHT::CudaHoughPlaneBatch phiPlane{{15u, 15u},
                                       etaMaxima.totalCapacity()};
  auto segmentSeeds = CudaHT::PhiHoughTransform::phiHoughTransform<1u>(
      phiPlane, spacePoints, etaMaxima, axisRanges);
  segmentSeeds.moveToHost(nullptr);
  segmentSeeds.copyAssociatedHitIndicesToHost(nullptr);

  BOOST_REQUIRE_EQUAL(segmentSeeds.groupCount(), 1u);
  BOOST_REQUIRE_EQUAL(segmentSeeds.nSeeds(0u), 1u);
  const auto seed = segmentSeeds.at(0u, 0u);
  BOOST_REQUIRE(static_cast<bool>(seed));
  BOOST_CHECK(seed->hasPhiExtension());
  BOOST_CHECK_EQUAL(seed->parentBucket(), 0u);
  BOOST_CHECK_SMALL(std::abs(seed->tanAlpha() - 0.4), 1.0e-10);
  BOOST_CHECK_SMALL(std::abs(seed->interceptX() + 20.0), 1.0e-10);
  BOOST_CHECK_SMALL(std::abs(seed->tanBeta() - 0.4), 1.0e-10);
  BOOST_CHECK_SMALL(std::abs(seed->interceptY() - 10.0), 1.0e-10);
  BOOST_CHECK_EQUAL(seed->getCounts(), 4.0f);
  BOOST_CHECK_EQUAL(seed->getHitsInMax().size(), 8u);

  const auto associated = seed->associatedHitIndices();
  BOOST_REQUIRE_EQUAL(associated.size(), 8u);
  for (std::uint32_t hit = 0u; hit < 8u; ++hit) {
    BOOST_CHECK(std::ranges::find(associated, hit) != associated.end());
  }
}

BOOST_AUTO_TEST_CASE(cuda_hough_phi_requires_two_measurements) {
  auto spacePoints = makeEtaPhiSegment(1u);
  CudaHT::CudaHoughPlaneBatch etaPlane{{15u, 15u}, 1u};
  auto etaMaxima = CudaHT::EtaHoughTransform::etaHoughTransform<1u>(
      etaPlane, spacePoints, axisRanges);

  CudaHT::CudaHoughPlaneBatch phiPlane{{15u, 15u},
                                       etaMaxima.totalCapacity()};
  auto segmentSeeds = CudaHT::PhiHoughTransform::phiHoughTransform<1u>(
      phiPlane, spacePoints, etaMaxima, axisRanges);
  segmentSeeds.moveToHost(nullptr);
  segmentSeeds.copyAssociatedHitIndicesToHost(nullptr);

  BOOST_REQUIRE_EQUAL(etaMaxima.nBuckets(), 1u);
  BOOST_REQUIRE_EQUAL(segmentSeeds.nSeeds(0u), 1u);
  const auto seed = segmentSeeds.at(0u, 0u);
  BOOST_REQUIRE(static_cast<bool>(seed));
  BOOST_CHECK(!seed->hasPhiExtension());
  BOOST_CHECK_SMALL(std::abs(seed->tanAlpha()), 1.0e-10);
  BOOST_CHECK_SMALL(std::abs(seed->interceptX()), 1.0e-10);
  BOOST_CHECK_SMALL(std::abs(seed->tanBeta() - 0.4), 1.0e-10);
  BOOST_CHECK_SMALL(std::abs(seed->interceptY() - 10.0), 1.0e-10);
  BOOST_CHECK_EQUAL(seed->getHitsInMax().size(), 5u);
}

BOOST_AUTO_TEST_SUITE_END()

}  // namespace ActsTests
