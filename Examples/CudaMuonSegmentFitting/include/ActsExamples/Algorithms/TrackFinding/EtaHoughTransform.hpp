// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ActsExamples/EventData/CudaMuonHoughMaximum.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"
#include "ActsExamples/EventData/CudaMuonHoughMaximum.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform {

namespace detail {

void etaHoughTransformImpl(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    CudaHoughMaximumBatchArrays maxima, const HoughAxisRanges& axisRanges,
    YieldType weight, std::uint32_t threadsPerBlock, std::uint32_t numBlocks);

void fillEtaHitAssociationsImpl(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    CudaHoughMaximumBatchArrays maxima, const HoughAxisRanges& axisRanges,
    std::uint32_t threadsPerBlock);

}  // namespace detail

/// Fill the Eta Hough planes, find one global maximum in each bucket and
/// associate the contributing input space points with every maximum.
///
/// The returned maximum batch remains allocated on the device. Call
/// moveToHost() when CPU access is required, but hit association has still to
/// be done.
template <std::size_t MaximaPerBucket = 1u>
CudaHoughMaximumBatch<MaximaPerBucket> etaHoughTransform(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    const HoughAxisRanges& axisRanges, YieldType weight = YieldType{1.0},
    std::uint32_t threadsPerBlock = 128u, std::uint32_t numBlocks = 0u) {

  CudaHoughMaximumBatch<MaximaPerBucket> maxima{plane.nBuckets()};
  maxima.moveToDevice();

  // 1. Fill the Hough planes, find maxima and count their associated hits.
  detail::etaHoughTransformImpl(plane, spacePoints, maxima.deviceArrays(),
                                axisRanges, weight, threadsPerBlock, numBlocks);

  // 2. Copy only nMaxima and nAssociatedHits to the CPU.
  maxima.copyAssociationMetadataToHost();

  // 3. Calculate CSR offsets and allocate the exact index storage.
  maxima.allocateAssociationStorage();

  // 4. Fill the newly allocated index array.
  detail::fillEtaHitAssociationsImpl(
      plane, spacePoints, maxima.deviceArrays(), axisRanges, threadsPerBlock);

  return maxima;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform
