// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ActsExamples/EventData/CudaMuonHoughMaximum.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <cuda_runtime.h>

namespace ActsExamples::CudaHoughTransformUtils::detail {

/// Round the value upward to nearest aligment multiply for sharedByte number
/// @param value: number to be rounded
/// @param alignment: value of which multiple to align
constexpr std::size_t alignUp(std::size_t value,
                              std::size_t alignment) noexcept {
  return ((value + alignment - 1u) / alignment) * alignment;
}

/// Convert layer index into bit mask representation
__device__ __host__ inline LayerMask layerBit(unsigned layer) {
  if (layer >= 8u * sizeof(LayerMask)) {
    return LayerMask{0ull};
  }

  return LayerMask{1ull} << layer;
}

// Return true if bit is not already in oldMask
__device__ inline bool notInMask(const LayerMask oldMask, const LayerMask bit) {
  return (oldMask & bit) == LayerMask{0ull};
}

/// Device mirrors of Acts:HoughTransformUtils bin helpers
/// convenience functions to link bin indices to axis coordinates

/// @brief Returns the lower bound of the bin specified by step
/// @param min: Start of axis range
/// @param max: End of axis range
/// @param nSteps: Number of bins in axis
/// @param binIndex: The index of the bin
/// @return the parameter value at the bin center.
/// No special logic to prevent over-/underflow, checking these is
/// left to the caller
__device__ inline double binCenterDevice(double min, double max,
                                         unsigned nSteps, unsigned binIndex) {
  return min + (max - min) * 0.5 * (2.0 * binIndex + 1.0) / nSteps;
}

/// @brief Find the bin index corresponding to a certain abscissa
/// of the coordinate axis, based on the axis limits and binning.
/// @param min: Start of axis range
/// @param max: End of axis range
/// @param nSteps: Number of bins in axis
/// @param val: value to find the corresponding bin for
/// @return the bin number.
/// No special logic to prevent over-/underflow, checking these is
/// left to the caller
__device__ inline int binIndexDevice(double min, double max, unsigned nSteps,
                                     double val) {
  return static_cast<int>((val - min) / (max - min) * nSteps);
}

/// @brief Fill one bin in shared memory
/// @param sharedHits: pointer to shared mem with num of hits
/// @param sharedLayers: pointer to shared mem with num of layers
/// @param sharedLayerMask: pointer to shared mem with LayerMask
/// @param nBinsX: number of bins in X
/// @param xBin: x bin
/// @param yBin: y bin
/// @param layer: Layer number of hit
/// @param weight: Weight of one hit, normaly 1
__device__ inline void fillSharedBin(YieldType* sharedHits,
                                     YieldType* sharedLayers,
                                     LayerMask* sharedLayerMask,
                                     std::uint32_t nBinsX, std::uint32_t xBin,
                                     std::uint32_t yBin, unsigned layer,
                                     YieldType weight) {
  const std::uint32_t localBin = yBin * nBinsX + xBin;

  atomicAdd(&sharedHits[localBin], weight);

  // Get layer as bit mask
  const LayerMask bit = layerBit(layer);

  // Check for error
  if (bit == LayerMask{0ull}) {
    return;
  }

  // atomicOr retrns oldMask and applies layer bit to bitMask
  const LayerMask oldMask = atomicOr(&sharedLayerMask[localBin], bit);

  // If bit was not in mask, add 1 (weight) to number of layers
  if (notInMask(oldMask, bit)) {
    atomicAdd(&sharedLayers[localBin], weight);
  }
}

/// @brief Fill bins in Y-column
__device__ inline void fillSharedYBand(
    YieldType* sharedHits, YieldType* sharedLayers, LayerMask* sharedLayerMask,
    const CudaHoughPlaneBatchArrays plane, const HoughAxisRanges ranges,
    std::uint32_t xBin, CoordType yCenter, CoordType yHalfWidth, unsigned layer,
    YieldType weight) {
  int yBinDown = binIndexDevice(ranges.yMin, ranges.yMax, plane.nBinsY,
                                yCenter - yHalfWidth);

  int yBinUp = binIndexDevice(ranges.yMin, ranges.yMax, plane.nBinsY,
                              yCenter + yHalfWidth);

  // Necessary checks
  if (yBinDown > yBinUp) {
    const int temporary = yBinDown;
    yBinDown = yBinUp;
    yBinUp = temporary;
  }

  if (yBinDown < 0) {
    yBinDown = 0;
  }

  if (yBinUp >= static_cast<int>(plane.nBinsY)) {
    yBinUp = static_cast<int>(plane.nBinsY) - 1;
  }

  // Top hat add, so 1 for all values
  for (int yBin = yBinDown; yBin <= yBinUp; ++yBin) {
    fillSharedBin(sharedHits, sharedLayers, sharedLayerMask, plane.nBinsX, xBin,
                  static_cast<std::uint32_t>(yBin), layer, weight);
  }
}

}  // namespace ActsExamples::CudaHoughTransformUtils::detail

namespace ActsExamples::CudaHoughTransformUtils::PeakFinders {

// Struct for GlobalMaximum Algorithm
struct GlobalMaximumCandidate {
  YieldType nHits = -std::numeric_limits<YieldType>::infinity();

  std::uint32_t localBin = std::numeric_limits<std::uint32_t>::max();
};

// @brief Check if better
// @param candidate To compare
// @param current Current best
__device__ inline bool betterCandidate(const GlobalMaximumCandidate& candidate,
                                       const GlobalMaximumCandidate& current) {
  if (candidate.nHits > current.nHits) {
    return true;
  }

  return candidate.nHits == current.nHits &&
         candidate.localBin < current.localBin;
}

/// Every thread in the block must call this function.
///
/// sharedCandidates must contain at least blockDim.x entries.
__device__ inline GlobalMaximumCandidate findGlobalMaximum(
    const YieldType* sharedHits, std::uint32_t nCells,
    GlobalMaximumCandidate* sharedCandidates) {

  // 1. Preprocess cells so there is one maximum per thread: 225->128
  GlobalMaximumCandidate localMaximum{};

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    const GlobalMaximumCandidate candidate{sharedHits[localBin], localBin};

    if (betterCandidate(candidate, localMaximum)) {
      localMaximum = candidate;
    }
  }

  // 2. Each thread has one maximum
  sharedCandidates[threadIdx.x] = localMaximum;
  __syncthreads();

  // 3. Shared-memory block reduction in general form
  // Works for both power-of-two and non-power-of-two block sizes.
  for (std::uint32_t active = blockDim.x; active > 1u;) {
    const std::uint32_t nextActive = (active + 1u) / 2u;

    if (threadIdx.x < nextActive) {
      const std::uint32_t partner = threadIdx.x + nextActive;

      if (partner < active && betterCandidate(sharedCandidates[partner],
                                              sharedCandidates[threadIdx.x])) {
        sharedCandidates[threadIdx.x] = sharedCandidates[partner];
      }
    }

    __syncthreads();
    active = nextActive;
  }

  return sharedCandidates[0];
}

/// @brief Atomically reserve one maximum slot in global memory
/// @note Avoids going over preallocated limit
__device__ inline std::uint32_t reserveMaximumSlot(
    CudaHoughMaximumBatchArrays maxima, std::uint32_t bucket) {
  std::uint32_t* counter = &maxima.nMaxima[bucket];

  // Just atomic read value of counter
  std::uint32_t current = atomicCAS(counter, 0u, 0u);

  while (current < maxima.capacityPerBucket) {
    // Attempt to reserve
    const std::uint32_t observed = atomicCAS(counter, current, current + 1u);

    // If counter was equal current it is reserved
    if (observed == current) {
      return current;
    }

    current = observed;
  }

  return maxima.capacityPerBucket;
}

/// @brief If possible, add Eta maximum to HoughMaximum batch
/// @param maxima: ptr to preallocated SoA of maximums
/// @param plane: ptr to global SoA of plane
/// @param ranges: range of bucket
/// @param sharedLayers: ptr to shared mem of Layers
/// @param sharedLayers: ptr to shared mem of Layers bit Mask
/// @param bucket: bucket idx
/// @param candidate: Hough Maximum object to append
///
/// @note Operation can fail (return false) if bucket has no more 
/// places for maximums -> number of maximums per bucket is 
/// prealocated before the operation
__device__ inline bool appendEtaMaximum(
    CudaHoughMaximumBatchArrays maxima, const CudaHoughPlaneBatchArrays plane,
    const HoughAxisRanges ranges, const YieldType* sharedLayers,
    const LayerMask* sharedLayerMask, std::uint32_t bucket,
    const GlobalMaximumCandidate& candidate) {
  if (candidate.nHits <= YieldType{0.0} ||
      candidate.localBin == std::numeric_limits<std::uint32_t>::max()) {
    return false;
  }

  const std::uint32_t maximum = reserveMaximumSlot(maxima, bucket);

  if (maximum >= maxima.capacityPerBucket) {
    return false;
  }

  const std::uint32_t xBin = candidate.localBin % plane.nBinsX;

  const std::uint32_t yBin = candidate.localBin / plane.nBinsX;

  const std::uint32_t outputIndex = maxima.index(bucket, maximum);

  maxima.tanBeta[outputIndex] =
      detail::binCenterDevice(ranges.xMin, ranges.xMax, plane.nBinsX, xBin);

  maxima.interceptY[outputIndex] =
      detail::binCenterDevice(ranges.yMin, ranges.yMax, plane.nBinsY, yBin);

  maxima.nHits[outputIndex] = candidate.nHits;
  maxima.nLayers[outputIndex] = sharedLayers[candidate.localBin];

  maxima.layerMask[outputIndex] = sharedLayerMask[candidate.localBin];

  maxima.xBin[outputIndex] = xBin;
  maxima.yBin[outputIndex] = yBin;

  return true;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::PeakFinders

namespace ActsExamples::CudaHoughTransformUtils::detail {

/// @brief Helper to get required number of shared Bytes
/// for whoel Eta operation
inline std::size_t sharedBytesForEtaHough(std::size_t nCells,
                                          std::size_t threadsPerBlock) {
  std::size_t bytes = 2u * nCells * sizeof(YieldType);

  bytes = alignUp(bytes, alignof(LayerMask));
  bytes += nCells * sizeof(LayerMask);

  bytes = alignUp(bytes, alignof(PeakFinders::GlobalMaximumCandidate));

  bytes += threadsPerBlock * sizeof(PeakFinders::GlobalMaximumCandidate);

  return bytes;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::detail
