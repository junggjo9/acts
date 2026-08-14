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
#include "ActsExamples/Utilities/CudaUtilities.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform {

namespace detail {

void etaHoughTransformImpl(CudaHoughPlaneBatch& plane,
                           CudaMuonSpacePointContainer& spacePoints,
                           CudaHoughMaximumBatchArrays maxima,
                           const HoughAxisRanges& axisRanges, YieldType weight,
                           std::uint32_t threadsPerBlock,
                           std::uint32_t numBlocks, PeakFinder peakFinder,
                           cudaStream_t stream);

void fillEtaHitAssociationsImpl(CudaHoughPlaneBatch& plane,
                                CudaMuonSpacePointContainer& spacePoints,
                                CudaHoughMaximumBatchArrays maxima,
                                const HoughAxisRanges& axisRanges,
                                std::uint32_t threadsPerBlock,
                                std::uint32_t numBlocks,
                                cudaStream_t stream);

}  // namespace detail

/// Fill the Eta Hough planes, find the maxima selected by the configured peak
/// finder in each bucket and associate the contributing input space points with
/// every maximum.
///
/// The returned maximum batch remains allocated on the device. Call
/// moveToHost(stream) and copyAssociatedHitIndicesToHost(stream) when CPU
/// access is required. Supplying a stream confines all synchronization to that
/// stream.
/// A non-zero numBlocks limits all bucket/maximum processing grids, allowing
/// several event streams to share the device. Zero preserves the default
/// launch sizes.
template <std::size_t MaximaPerBucket = 1u,
          PeakFinder peakFinder = PeakFinder::GlobalMaximum>
CudaHoughMaximumBatch<MaximaPerBucket> etaHoughTransform(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    const HoughAxisRanges& axisRanges, YieldType weight = YieldType{1.0},
    std::uint32_t threadsPerBlock = 128u, std::uint32_t numBlocks = 0u,
    cudaStream_t stream = nullptr) {
  CudaHoughMaximumBatch<MaximaPerBucket> maxima{plane.nBuckets()};
  maxima.moveToDevice(stream);

  // 1. Fill the Hough planes, find maxima and count their associated hits.
  detail::etaHoughTransformImpl(plane, spacePoints, maxima.deviceArrays(),
                                axisRanges, weight, threadsPerBlock, numBlocks,
                                peakFinder, stream);

  // 2. Copy only nMaxima and nAssociatedHits to the CPU.
  maxima.copyAssociationMetadataToHost(stream);

  // 3. Calculate CSR offsets and allocate the exact index storage.
  maxima.allocateAssociationStorage(stream);

  // 4. Fill the newly allocated index array.
  detail::fillEtaHitAssociationsImpl(plane, spacePoints, maxima.deviceArrays(),
                                     axisRanges, threadsPerBlock, numBlocks,
                                     stream);

  return maxima;
}

/// Per-worker Eta transform entry point owning an independent CUDA stream.
/// Device allocations remain event-local and are not cached.
class Processor {
 public:
  Processor() = default;

  cudaStream_t stream() const noexcept { return m_stream.get(); }

  template <std::size_t MaximaPerBucket = 1u,
            PeakFinder peakFinder = PeakFinder::GlobalMaximum>
  CudaHoughMaximumBatch<MaximaPerBucket> run(
      CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
      const HoughAxisRanges& axisRanges, YieldType weight = YieldType{1.0},
      std::uint32_t threadsPerBlock = 128u,
      std::uint32_t numBlocks = 0u) const {
    return etaHoughTransform<MaximaPerBucket, peakFinder>(
        plane, spacePoints, axisRanges, weight, threadsPerBlock, numBlocks,
        stream());
  }

 private:
  CudaStream m_stream{};
};

}  // namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform
