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

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform {

namespace detail {

void fillEtaDriftCirclesOnDeviceImpl(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    CudaHoughMaximumBatchArrays maxima, const HoughAxisRanges& axisRanges,
    YieldType weight, std::uint32_t threadsPerBlock, std::uint32_t numBlocks);

}  // namespace detail

/// Fill the eta Hough planes and find one global maximum in each bucket.
///
/// The returned maximum batch remains allocated on the device. Call
/// moveToHost() when CPU access is required, but hit association has still to
/// be done.
template <std::size_t MaximaPerBucket = 5u>
CudaHoughMaximumBatch<MaximaPerBucket> fillEtaDriftCirclesOnDevice(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    const HoughAxisRanges& axisRanges, YieldType weight = YieldType{1.0},
    std::uint32_t threadsPerBlock = 128u, std::uint32_t numBlocks = 0u) {
  if (plane.nBuckets() != spacePoints.bucketCount()) {
    throw std::invalid_argument(
        "Eta Hough plane and space-point container must have the same bucket "
        "count");
  }

  CudaHoughMaximumBatch<MaximaPerBucket> maxima{plane.nBuckets()};

  maxima.moveToDevice();

  detail::fillEtaDriftCirclesOnDeviceImpl(plane, spacePoints,
                                          maxima.deviceArrays(), axisRanges,
                                          weight, threadsPerBlock, numBlocks);

  return maxima;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform
