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

/// Round the value upward to nearest alignment multiply for sharedByte number
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

struct YBinRange {
  int down = 0;
  int up = -1;
};

/// Return the clamped bin range covered by a top-hat Y band.
__device__ inline YBinRange yBinRange(
    const HoughAxisRanges ranges, std::uint32_t nBinsY, CoordType yCenter,
    CoordType yHalfWidth) {
  int down = binIndexDevice(ranges.yMin, ranges.yMax, nBinsY,
                            yCenter - yHalfWidth);
  int up = binIndexDevice(ranges.yMin, ranges.yMax, nBinsY,
                          yCenter + yHalfWidth);

  if (down > up) {
    const int temporary = down;
    down = up;
    up = temporary;
  }

  if (down < 0) {
    down = 0;
  }
  if (up >= static_cast<int>(nBinsY)) {
    up = static_cast<int>(nBinsY) - 1;
  }

  return {down, up};
}

/// @brief Fill one bin in shared memory
/// @param sharedHits: pointer to shared mem with num of hits
/// @param sharedLayers: pointer to shared mem with num of layers
/// @param sharedLayerMask: pointer to shared mem with LayerMask
/// @param nBinsX: number of bins in X
/// @param xBin: x bin
/// @param yBin: y bin
/// @param layer: Layer number of hit
/// @param weight: Weight of one hit, normally 1
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

  if (bit == LayerMask{0ull}) {
    return;
  }

  // atomicOr returns oldMask and applies layer bit to bitMask
  const LayerMask oldMask = atomicOr(&sharedLayerMask[localBin], bit);

  // If bit was not in mask, add 1 (weight) to number of layers
  if ((oldMask & bit) == LayerMask{0ull}) {
    atomicAdd(&sharedLayers[localBin], weight);
  }
}

/// @brief Fill bins in Y-column
__device__ inline YBinRange fillSharedYBand(
    YieldType* sharedHits, YieldType* sharedLayers, LayerMask* sharedLayerMask,
    const CudaHoughPlaneBatchArrays plane, const HoughAxisRanges ranges,
    std::uint32_t xBin, CoordType yCenter, CoordType yHalfWidth, unsigned layer,
    YieldType weight) {
  const YBinRange band =
      yBinRange(ranges, plane.nBinsY, yCenter, yHalfWidth);

  // Top hat add, so 1 for all values
  for (int yBin = band.down; yBin <= band.up; ++yBin) {
    fillSharedBin(sharedHits, sharedLayers, sharedLayerMask, plane.nBinsX, xBin,
                  static_cast<std::uint32_t>(yBin), layer, weight);
  }

  return band;
}

/// @brief Count one associated hit in every cell covered by a Y band.
__device__ inline void countSharedYBand(
    std::uint32_t* sharedAssociatedHits, std::uint32_t nBinsX,
    std::uint32_t xBin, YBinRange band) {
  for (int yBin = band.down; yBin <= band.up; ++yBin) {
    const std::uint32_t localBin =
        static_cast<std::uint32_t>(yBin) * nBinsX + xBin;
    atomicAdd(&sharedAssociatedHits[localBin], 1u);
  }
}

/// @brief Count one hit in the union of two Y bands.
///
/// The two drift-circle solutions are filled independently in the Hough
/// accumulator, but hit association treats them as alternatives: a hit is
/// associated with a cell when either solution covers it. This helper is
/// called once per hit/X-bin pair and increments every covered cell exactly
/// once, including where the two bands overlap.
__device__ inline void countSharedDistinctYBands(
    std::uint32_t* sharedAssociatedHits, std::uint32_t nBinsX,
    std::uint32_t xBin, YBinRange first, YBinRange second) {
  countSharedYBand(sharedAssociatedHits, nBinsX, xBin, first);

  // Only count the parts of the second band not already covered by the first.
  for (int yBin = second.down; yBin <= second.up; ++yBin) {
    if (yBin >= first.down && yBin <= first.up) {
      continue;
    }
    const std::uint32_t localBin =
        static_cast<std::uint32_t>(yBin) * nBinsX + xBin;
    atomicAdd(&sharedAssociatedHits[localBin], 1u);
  }
}

}  // namespace ActsExamples::CudaHoughTransformUtils::detail

namespace ActsExamples::CudaHoughTransformUtils::PeakFinders {

using SlidingWindowConfig =
    Acts::HoughTransformUtils::PeakFinders::SlidingWindowConfig;

/// @brief Configuration for a layer-aware relative non-maximum-suppression
/// peak finder.
///
/// The algorithm first rejects cells that do not satisfy the absolute yield,
/// layer-count and distinct-hit requirements. The supported cells are ranked
/// by layer count, distinct-hit count, accumulator yield and bin index. The
/// highest-ranked cell defines the reference yield used by the relative
/// threshold.
///
/// Local maxima are identified in a rectangular window. Windows are clipped at
/// plane boundaries, so edge cells remain eligible. Equal-yield plateaus are
/// resolved deterministically by retaining the cell with the lowest flattened
/// bin index.
///
/// Accepted maxima are emitted in rank order. After each selection, candidates
/// inside the configured X/Y spacing rectangle are suppressed. This removes
/// nearby responses from the same accumulator structure. No peak recentering
/// is performed.
///
/// Window and spacing values are expressed as fractions of the corresponding
/// axis bin count. They are rounded upward to a minimum radius of one bin,
/// allowing one configuration to be used with different plane dimensions.
struct RelativeNmsConfig {
  /// Absolute minimum accumulator yield.
  YieldType threshold = 3.0f;
  /// Minimum yield relative to the best layer-supported cell in the bucket.
  YieldType fractionCutoff = 0.50f;
  /// Minimum number of contributing detector layers for any retained peak.
  YieldType minimumLayers = 3.0f;
  /// Minimum number of distinct associated hits for any retained peak.
  std::uint32_t minimumAssociatedHits = 4u;
  /// Maximum number of peaks returned for one bucket.
  std::uint32_t maximumPeaks = 4u;
  /// Half-window for local-maximum detection as a fraction of each axis.
  /// With the default value, the X/Y radii (and full window dimensions) are
  /// 1/1 (3x3) for 15x15, 1/1 (3x3) for 25x25, and 3/2 (7x5) for 64x32.
  YieldType localWindowFraction = 0.04f;
  /// Minimum relative separation of retained peaks along the X axis.
  YieldType minimumXSpacingFraction = 0.08f;
  /// Minimum relative separation of retained peaks along the Y axis.
  YieldType minimumYSpacingFraction = 0.10f;
};

/// Accumulator yield and location used by the global-maximum finder.
struct GlobalMaximumCandidate {
  YieldType nHits = -std::numeric_limits<YieldType>::infinity();

  std::uint32_t localBin = std::numeric_limits<std::uint32_t>::max();
};

/// Candidate ordering used when only a finite number of marked peaks can be
/// retained. Layer coverage and distinct hit count are preferred over the
/// accumulator yield, which can count overlapping drift solutions twice.
struct RankedPeakCandidate {
  YieldType nLayers = -std::numeric_limits<YieldType>::infinity();
  std::uint32_t nAssociatedHits = 0u;
  YieldType nHits = -std::numeric_limits<YieldType>::infinity();
  std::uint32_t localBin = std::numeric_limits<std::uint32_t>::max();
};

__device__ inline bool betterCandidate(const GlobalMaximumCandidate& candidate,
                                       const GlobalMaximumCandidate& current) {
  return candidate.nHits > current.nHits ||
         (candidate.nHits == current.nHits &&
          candidate.localBin < current.localBin);
}

__device__ inline bool betterCandidate(
    const RankedPeakCandidate& candidate, const RankedPeakCandidate& current) {
  if (candidate.nLayers != current.nLayers) {
    return candidate.nLayers > current.nLayers;
  }
  if (candidate.nAssociatedHits != current.nAssociatedHits) {
    return candidate.nAssociatedHits > current.nAssociatedHits;
  }
  if (candidate.nHits != current.nHits) {
    return candidate.nHits > current.nHits;
  }
  return candidate.localBin < current.localBin;
}

/// Reduce one candidate per thread to a deterministic block-wide best value.
template <typename Candidate>
__device__ inline Candidate reduceBestCandidate(
    Candidate localBest, Candidate* sharedCandidates) {
  sharedCandidates[threadIdx.x] = localBest;
  __syncthreads();

  // This general reduction also supports non-power-of-two block sizes.
  for (std::uint32_t active = blockDim.x; active > 1u;) {
    const std::uint32_t nextActive = (active + 1u) / 2u;
    if (threadIdx.x < nextActive) {
      const std::uint32_t partner = threadIdx.x + nextActive;
      if (partner < active && betterCandidate(
                                  sharedCandidates[partner],
                                  sharedCandidates[threadIdx.x])) {
        sharedCandidates[threadIdx.x] = sharedCandidates[partner];
      }
    }
    __syncthreads();
    active = nextActive;
  }

  return sharedCandidates[0];
}

/// Every thread in the block must call this function.
///
/// sharedCandidates must contain at least blockDim.x entries.
__device__ inline GlobalMaximumCandidate findGlobalMaximum(
    const YieldType* sharedHits, std::uint32_t nCells,
    GlobalMaximumCandidate* sharedCandidates) {
  // First reduce the cells assigned to each thread locally.
  GlobalMaximumCandidate localMaximum{};

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    const GlobalMaximumCandidate candidate{sharedHits[localBin], localBin};

    if (betterCandidate(candidate, localMaximum)) {
      localMaximum = candidate;
    }
  }

  return reduceBestCandidate(localMaximum, sharedCandidates);
}

/// Mark all cells accepted by the sliding-window peak finder.
///
/// The comparison and tie-breaking rules mirror
/// Acts::HoughTransformUtils::PeakFinders::slidingWindowPeaks. Every thread in
/// the block must call this function, and sharedPeakMask must contain at least
/// nBinsX * nBinsY entries.
__device__ inline void findSlidingWindowPeaks(
    const YieldType* sharedHits, std::uint32_t nBinsX,
    std::uint32_t nBinsY, const SlidingWindowConfig& config,
    std::uint8_t* sharedPeakMask) {
  const std::uint32_t nCells = nBinsX * nBinsY;

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    sharedPeakMask[localBin] = 0u;
  }

  __syncthreads();

  const std::uint32_t xWindow =
      static_cast<std::uint32_t>(config.xWindowSize);
  const std::uint32_t yWindow =
      static_cast<std::uint32_t>(config.yWindowSize);

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    const std::uint32_t xBin = localBin % nBinsX;
    const std::uint32_t yBin = localBin / nBinsX;

    // As in the CPU implementation, bins whose comparison window would cross
    // an edge are not peak candidates.
    if (xBin < xWindow || xWindow >= nBinsX - xBin || yBin < yWindow ||
        yWindow >= nBinsY - yBin ||
        sharedHits[localBin] < static_cast<YieldType>(config.threshold)) {
      continue;
    }

    const YieldType maximum = sharedHits[localBin];
    bool passesWindow = true;

    for (std::uint32_t x = xBin - xWindow;
         passesWindow && x <= xBin + xWindow; ++x) {
      for (std::uint32_t y = yBin - yWindow; y <= yBin + yWindow; ++y) {
        const int xDistance = static_cast<int>(x) - static_cast<int>(xBin);
        const int yDistance = static_cast<int>(y) - static_cast<int>(yBin);

        // For integer bin distances this is equivalent to the CPU expression
        // yDistance + 0.1 > xDistance. Equal plateaus therefore retain the
        // upper-right cell only.
        const bool above = yDistance >= xDistance;
        const YieldType hits = sharedHits[y * nBinsX + x];

        if ((above && hits > maximum) || (!above && hits >= maximum)) {
          passesWindow = false;
          break;
        }
      }
    }

    sharedPeakMask[localBin] = passesWindow ? 1u : 0u;
  }

  __syncthreads();
}

/// Convert a fraction of an axis into a non-zero, upward-rounded bin radius.
__device__ inline std::uint32_t relativeBinRadius(std::uint32_t nBins,
                                                  YieldType fraction) {
  const YieldType scaled = fraction * static_cast<YieldType>(nBins);
  std::uint32_t radius = static_cast<std::uint32_t>(scaled);
  if (static_cast<YieldType>(radius) < scaled) {
    ++radius;
  }
  return radius > 0u ? radius : 1u;
}

/// Mark cells satisfying the absolute, layer and distinct-hit requirements.
/// The marked cells form the reference population for the relative cutoff.
__device__ inline void markSupportedPeakCells(
    const YieldType* sharedHits, const YieldType* sharedLayers,
    const std::uint32_t* sharedAssociatedHits, std::uint32_t nCells,
    const RelativeNmsConfig& config, std::uint8_t* sharedPeakMask) {
  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    sharedPeakMask[localBin] =
        sharedHits[localBin] >= config.threshold &&
                sharedLayers[localBin] >= config.minimumLayers &&
                sharedAssociatedHits[localBin] >= config.minimumAssociatedHits
            ? 1u
            : 0u;
  }

  __syncthreads();
}

/// Mark layer-supported local maxima passing an absolute and relative yield
/// threshold. Unlike the CPU sliding-window implementation, comparison
/// windows are clipped at plane boundaries rather than discarding edge peaks.
/// Every thread in the block must call this function.
__device__ inline void findRelativeNmsPeaks(
    const YieldType* sharedHits, const YieldType* sharedLayers,
    const std::uint32_t* sharedAssociatedHits, std::uint32_t nBinsX,
    std::uint32_t nBinsY, YieldType referenceYield,
    const RelativeNmsConfig& config, std::uint8_t* sharedPeakMask) {
  const std::uint32_t nCells = nBinsX * nBinsY;
  const std::uint32_t xWindow =
      relativeBinRadius(nBinsX, config.localWindowFraction);
  const std::uint32_t yWindow =
      relativeBinRadius(nBinsY, config.localWindowFraction);
  const YieldType relativeThreshold = config.fractionCutoff * referenceYield;
  const YieldType threshold = relativeThreshold > config.threshold
                                  ? relativeThreshold
                                  : config.threshold;

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    sharedPeakMask[localBin] = 0u;

    if (sharedHits[localBin] < threshold ||
        sharedLayers[localBin] < config.minimumLayers ||
        sharedAssociatedHits[localBin] < config.minimumAssociatedHits) {
      continue;
    }

    const std::uint32_t xBin = localBin % nBinsX;
    const std::uint32_t yBin = localBin / nBinsX;
    const std::uint32_t xBegin = xBin > xWindow ? xBin - xWindow : 0u;
    const std::uint32_t yBegin = yBin > yWindow ? yBin - yWindow : 0u;
    const std::uint32_t xEnd =
        xWindow < nBinsX - 1u - xBin ? xBin + xWindow : nBinsX - 1u;
    const std::uint32_t yEnd =
        yWindow < nBinsY - 1u - yBin ? yBin + yWindow : nBinsY - 1u;
    const YieldType candidateYield = sharedHits[localBin];
    bool localMaximum = true;

    for (std::uint32_t x = xBegin; localMaximum && x <= xEnd; ++x) {
      for (std::uint32_t y = yBegin; y <= yEnd; ++y) {
        const std::uint32_t neighbourBin = y * nBinsX + x;
        if (neighbourBin == localBin) {
          continue;
        }
        const YieldType neighbourYield = sharedHits[neighbourBin];
        if (neighbourYield < threshold ||
            sharedLayers[neighbourBin] < config.minimumLayers ||
            sharedAssociatedHits[neighbourBin] <
                config.minimumAssociatedHits) {
          continue;
        }
        if (neighbourYield > candidateYield ||
            (neighbourYield == candidateYield && neighbourBin < localBin)) {
          localMaximum = false;
          break;
        }
      }
    }

    sharedPeakMask[localBin] = localMaximum ? 1u : 0u;
  }

  __syncthreads();
}

/// Remove marked peaks that are close to a retained peak in both normalized
/// plane coordinates. Every thread in the block must call this function.
__device__ inline void suppressRelativeNmsNeighbours(
    std::uint8_t* sharedPeakMask, std::uint32_t nBinsX,
    std::uint32_t nBinsY, std::uint32_t selectedBin,
    const RelativeNmsConfig& config) {
  const std::uint32_t nCells = nBinsX * nBinsY;
  const std::uint32_t selectedX = selectedBin % nBinsX;
  const std::uint32_t selectedY = selectedBin / nBinsX;
  const std::uint32_t xSpacing =
      relativeBinRadius(nBinsX, config.minimumXSpacingFraction);
  const std::uint32_t ySpacing =
      relativeBinRadius(nBinsY, config.minimumYSpacingFraction);

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    if (sharedPeakMask[localBin] == 0u) {
      continue;
    }
    const std::uint32_t xBin = localBin % nBinsX;
    const std::uint32_t yBin = localBin / nBinsX;
    const std::uint32_t xDistance =
        xBin > selectedX ? xBin - selectedX : selectedX - xBin;
    const std::uint32_t yDistance =
        yBin > selectedY ? yBin - selectedY : selectedY - yBin;
    if (xDistance <= xSpacing && yDistance <= ySpacing) {
      sharedPeakMask[localBin] = 0u;
    }
  }

  __syncthreads();
}

/// Select the strongest remaining marked peak.
/// Every thread in the block must call this function, and sharedCandidates
/// must contain at least blockDim.x entries.
__device__ inline RankedPeakCandidate findBestMarkedPeak(
    const YieldType* sharedHits, const YieldType* sharedLayers,
    const std::uint32_t* sharedAssociatedHits,
    const std::uint8_t* sharedPeakMask, std::uint32_t nCells,
    RankedPeakCandidate* sharedCandidates) {
  RankedPeakCandidate localBest{};

  for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
       localBin += blockDim.x) {
    if (sharedPeakMask[localBin] == 0u) {
      continue;
    }

    const RankedPeakCandidate candidate{
        sharedLayers[localBin], sharedAssociatedHits[localBin],
        sharedHits[localBin], localBin};
    if (betterCandidate(candidate, localBest)) {
      localBest = candidate;
    }
  }

  return reduceBestCandidate(localBest, sharedCandidates);
}

/// Recenter one sliding-window peak using the same weighted-average rule as
/// the CPU implementation. The recentering window is clipped at plane edges
/// to keep device accesses valid for every configuration.
__device__ inline std::uint32_t slidingWindowRecenter(
    const YieldType* sharedHits, std::uint32_t nBinsX,
    std::uint32_t nBinsY, std::uint32_t localBin,
    const SlidingWindowConfig& config) {
  if (!config.recenter) {
    return localBin;
  }

  const std::uint32_t xCenter = localBin % nBinsX;
  const std::uint32_t yCenter = localBin / nBinsX;
  const std::uint32_t xRadius =
      static_cast<std::uint32_t>(config.xRecenterSize);
  const std::uint32_t yRadius =
      static_cast<std::uint32_t>(config.yRecenterSize);

  const std::uint32_t xBegin = xCenter > xRadius ? xCenter - xRadius : 0u;
  const std::uint32_t yBegin = yCenter > yRadius ? yCenter - yRadius : 0u;
  const std::uint32_t xEnd =
      xRadius < nBinsX - 1u - xCenter ? xCenter + xRadius : nBinsX - 1u;
  const std::uint32_t yEnd =
      yRadius < nBinsY - 1u - yCenter ? yCenter + yRadius : nBinsY - 1u;

  const YieldType maximum = sharedHits[localBin];
  YieldType weightedX = YieldType{0.0};
  YieldType weightedY = YieldType{0.0};
  YieldType total = YieldType{0.0};

  for (std::uint32_t x = xBegin; x <= xEnd; ++x) {
    for (std::uint32_t y = yBegin; y <= yEnd; ++y) {
      const YieldType hits = sharedHits[y * nBinsX + x];
      if (hits >= maximum) {
        weightedX += static_cast<YieldType>(x) * hits;
        weightedY += static_cast<YieldType>(y) * hits;
        total += hits;
      }
    }
  }

  const std::uint32_t xBin = static_cast<std::uint32_t>(weightedX / total);
  const std::uint32_t yBin = static_cast<std::uint32_t>(weightedY / total);
  return yBin * nBinsX + xBin;
}

/// @brief Atomically reserve one preallocated maximum slot.
__device__ inline std::uint32_t reserveMaximumSlot(
    CudaHoughMaximumBatchArrays maxima, std::uint32_t bucket) {
  std::uint32_t* counter = &maxima.nMaxima[bucket];

  // Read without changing the counter.
  std::uint32_t current = atomicCAS(counter, 0u, 0u);

  while (current < maxima.capacityPerBucket) {
    const std::uint32_t observed = atomicCAS(counter, current, current + 1u);

    if (observed == current) {
      return current;
    }

    current = observed;
  }

  return maxima.capacityPerBucket;
}

/// @brief Append an Eta maximum if the bucket still has a free slot.
/// @return Whether the candidate was stored.
__device__ inline bool appendEtaMaximum(
    CudaHoughMaximumBatchArrays maxima, const CudaHoughPlaneBatchArrays plane,
    const HoughAxisRanges ranges, const YieldType* sharedLayers,
    const LayerMask* sharedLayerMask,
    const std::uint32_t* sharedAssociatedHits, std::uint32_t bucket,
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
  maxima.nAssociatedHits[outputIndex] =
      sharedAssociatedHits[candidate.localBin];

  maxima.xBin[outputIndex] = xBin;
  maxima.yBin[outputIndex] = yBin;

  return true;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::PeakFinders

namespace ActsExamples::CudaHoughTransformUtils::detail {

/// Return the dynamic shared-memory size required by the Eta kernel.
inline std::size_t sharedBytesForEtaHough(std::size_t nCells,
                                          std::size_t threadsPerBlock,
                                          PeakFinder peakFinder) {
  std::size_t bytes = 2u * nCells * sizeof(YieldType);

  bytes = alignUp(bytes, alignof(LayerMask));
  bytes += nCells * sizeof(LayerMask);

  bytes = alignUp(bytes, alignof(std::uint32_t));

  bytes += nCells * sizeof(std::uint32_t);

  if (peakFinder != PeakFinder::GlobalMaximum) {
    bytes += nCells * sizeof(std::uint8_t);
    bytes = alignUp(bytes, alignof(PeakFinders::RankedPeakCandidate));
    bytes += threadsPerBlock * sizeof(PeakFinders::RankedPeakCandidate);
  } else {
    bytes = alignUp(bytes, alignof(PeakFinders::GlobalMaximumCandidate));
    bytes += threadsPerBlock * sizeof(PeakFinders::GlobalMaximumCandidate);
  }

  return bytes;
}

}  // namespace ActsExamples::CudaHoughTransformUtils::detail
