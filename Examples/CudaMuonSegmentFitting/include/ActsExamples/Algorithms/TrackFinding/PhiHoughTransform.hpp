// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ActsExamples/EventData/CudaMuonHoughMaximum.hpp"
#include "ActsExamples/EventData/CudaMuonSegmentSeed.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

#include <cstddef>
#include <cstdint>

namespace ActsExamples::CudaHoughTransformUtils::PhiHoughTransform {

namespace detail {

void phiHoughTransformImpl(CudaHoughPlaneBatch& plane,
                           CudaMuonSpacePointContainer& spacePoints,
                           CudaHoughMaximumBatchArrays etaMaxima,
                           CudaMuonSegmentSeedArrays segmentSeeds,
                           const HoughAxisRanges& axisRanges, YieldType weight,
                           std::uint32_t threadsPerBlock,
                           std::uint32_t numBlocks, PeakFinder peakFinder,
                           cudaStream_t stream);

void fillPhiHitAssociationsImpl(CudaHoughPlaneBatch& plane,
                                CudaMuonSpacePointContainer& spacePoints,
                                CudaHoughMaximumBatchArrays etaMaxima,
                                CudaMuonSegmentSeedArrays segmentSeeds,
                                const HoughAxisRanges& axisRanges,
                                std::uint32_t threadsPerBlock,
                                std::uint32_t numBlocks,
                                cudaStream_t stream);

}  // namespace detail

/// Extend every occupied Eta maximum with a Hough transform in Phi.
///
/// The Phi line model and uncertainty are identical to MuonHoughSeeder:
/// interceptX = localX - tanAlpha * localZ and a three-sigma strip width.
/// At least two Phi measurements must support a returned extension. Pure-Phi
/// hits are considered for every Eta maximum in their physical bucket, while
/// dual-coordinate hits must already be associated with that Eta maximum.
///
/// The output is self-contained: every segment seed stores all four line
/// parameters and the combined Eta/Phi hit association. If fewer than two Phi
/// measurements are available, the Eta seed is retained without a Phi
/// extension, as in the CPU implementation.
template <std::size_t MaximaPerEtaMaximum = 1u,
          PeakFinder peakFinder = PeakFinder::GlobalMaximum,
          std::size_t EtaMaximaPerBucket>
CudaMuonSegmentSeedContainer phiHoughTransform(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    const CudaHoughMaximumBatch<EtaMaximaPerBucket>& etaMaxima,
    const HoughAxisRanges& axisRanges, YieldType weight = YieldType{1.0},
    std::uint32_t threadsPerBlock = 128u, std::uint32_t numBlocks = 0u,
    cudaStream_t stream = nullptr) {
  static_assert(MaximaPerEtaMaximum > 0u);
  CudaMuonSegmentSeedContainer segmentSeeds{
      etaMaxima.totalCapacity(), MaximaPerEtaMaximum, spacePoints};
  segmentSeeds.moveToDevice(stream);

  detail::phiHoughTransformImpl(
      plane, spacePoints, etaMaxima.deviceArrays(),
      segmentSeeds.deviceArrays(), axisRanges, weight, threadsPerBlock,
      numBlocks, peakFinder, stream);

  segmentSeeds.copyAssociationMetadataToHost(stream);
  segmentSeeds.allocateAssociationStorage(stream);

  detail::fillPhiHitAssociationsImpl(
      plane, spacePoints, etaMaxima.deviceArrays(),
      segmentSeeds.deviceArrays(), axisRanges, threadsPerBlock, numBlocks,
      stream);

  return segmentSeeds;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::PhiHoughTransform
