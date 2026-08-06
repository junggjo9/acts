// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Acts/Definitions/Units.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/EventData/CudaMuonHoughMaximum.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.cuh"
#include "ActsExamples/Utilities/CudaUtilities.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include <cuda_runtime.h>

namespace {

using ActsExamples::CudaHoughMaximumBatchArrays;
using ActsExamples::CudaMuonSpacePointArrays;
using ActsExamples::detLayer;

using ActsExamples::CudaHoughTransformUtils::CoordType;
using ActsExamples::CudaHoughTransformUtils::CudaHoughPlaneBatchArrays;
using ActsExamples::CudaHoughTransformUtils::HoughAxisRanges;
using ActsExamples::CudaHoughTransformUtils::LayerMask;
using ActsExamples::CudaHoughTransformUtils::PeakFinder;
using ActsExamples::CudaHoughTransformUtils::YieldType;

constexpr CoordType etaWidthScale = 1.0;  // mm
constexpr CoordType etaMaxWidth = 1.0;    // mm

__device__ inline CoordType etaYHalfWidth(const HoughAxisRanges& ranges,
                                          std::uint32_t nBinsX, CoordType z,
                                          CoordType radius,
                                          CoordType covariance,
                                          CoordType widthScale,
                                          CoordType maxMeasurementWidth) {
  const CoordType tanBetaBinWidth =
      (ranges.xMax - ranges.xMin) / static_cast<CoordType>(nBinsX);

  const CoordType tanBetaHalfWidth = CoordType{0.5} * tanBetaBinWidth;

  // Width caused by representing a complete tanBeta bin by its centre.
  const CoordType geometricWidth = (fabs(z) + fabs(radius)) * tanBetaHalfWidth;

  CoordType measurementWidth =
      sqrt(covariance > CoordType{0.0} ? covariance : CoordType{0.0}) *
      widthScale;

  if (maxMeasurementWidth > CoordType{0.0} &&
      measurementWidth > maxMeasurementWidth) {
    measurementWidth = maxMeasurementWidth;
  }

  return geometricWidth + measurementWidth;
}

namespace HoughDetail = ActsExamples::CudaHoughTransformUtils::detail;

namespace PeakFinders = ActsExamples::CudaHoughTransformUtils::PeakFinders;

__device__ inline CoordType minimum(CoordType first, CoordType second) {
  return first < second ? first : second;
}

__device__ inline CoordType maximum(CoordType first, CoordType second) {
  return first > second ? first : second;
}

__device__ inline bool yBinInsideBand(std::uint32_t selectedYBin,
                                      CoordType center, CoordType halfWidth,
                                      const HoughAxisRanges& ranges,
                                      std::uint32_t nBinsY) {
  const HoughDetail::YBinRange band =
      HoughDetail::yBinRange(ranges, nBinsY, center, halfWidth);

  const int selected = static_cast<int>(selectedYBin);
  return selected >= band.down && selected <= band.up;
}

__device__ inline bool etaHitContributesToMaximum(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays maxima, HoughAxisRanges baseRanges,
    CoordType widthScale, CoordType maxWidth, std::uint32_t bucket,
    std::uint32_t maximum, std::uint32_t hitIndex) {
  const std::uint32_t maximumIndex = maxima.index(bucket, maximum);

  const std::uint32_t selectedXBin = maxima.xBin[maximumIndex];
  const std::uint32_t selectedYBin = maxima.yBin[maximumIndex];

  const HoughAxisRanges ranges{baseRanges.xMin, baseRanges.xMax,
                               plane.yMin[bucket], plane.yMax[bucket]};

  const CoordType tanTheta = HoughDetail::binCenterDevice(
      ranges.xMin, ranges.xMax, plane.nBinsX, selectedXBin);

  const CoordType y = spacePoints.localPositionY[hitIndex];
  const CoordType z = spacePoints.localPositionZ[hitIndex];
  const CoordType radius = spacePoints.driftRadius[hitIndex];
  const CoordType covariance = spacePoints.covariance1[hitIndex];

  // Must use the same width definition as Hough filling.
  const CoordType width = etaYHalfWidth(ranges, plane.nBinsX, z, radius,
                                        covariance, widthScale, maxWidth);

  const CoordType centralIntercept = y - tanTheta * z;

  const CoordType projectedRadius =
      radius * sqrt(CoordType{1.0} + tanTheta * tanTheta);

  const CoordType negativeIntercept = centralIntercept - projectedRadius;

  const CoordType positiveIntercept = centralIntercept + projectedRadius;

  return yBinInsideBand(selectedYBin, negativeIntercept, width, ranges,
                        plane.nBinsY) ||
         yBinInsideBand(selectedYBin, positiveIntercept, width, ranges,
                        plane.nBinsY);
}

__global__ void computeEtaInterceptRangesMdtBatchKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    HoughAxisRanges baseRanges, CoordType interceptMargin) {
  const std::uint32_t bucket = blockIdx.x;

  if (bucket >= plane.nBuckets) {
    return;
  }

  extern __shared__ CoordType sharedRanges[];

  CoordType* sharedMinimum = sharedRanges;
  CoordType* sharedMaximum = sharedMinimum + blockDim.x;

  constexpr CoordType infinity = 1.0e100;

  CoordType localMinimum = infinity;
  CoordType localMaximum = -infinity;

  const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];

  const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

  for (std::uint32_t hit = bucketStart + threadIdx.x; hit < bucketEnd;
       hit += blockDim.x) {
    const CoordType y = spacePoints.localPositionY[hit];

    localMinimum = minimum(localMinimum, y - interceptMargin);

    localMaximum = maximum(localMaximum, y + interceptMargin);
  }

  sharedMinimum[threadIdx.x] = localMinimum;
  sharedMaximum[threadIdx.x] = localMaximum;

  __syncthreads();

  // Reduction supporting arbitrary block dimensions.
  for (std::uint32_t active = blockDim.x; active > 1u;) {
    const std::uint32_t nextActive = (active + 1u) / 2u;

    if (threadIdx.x < nextActive) {
      const std::uint32_t partner = threadIdx.x + nextActive;

      if (partner < active) {
        sharedMinimum[threadIdx.x] =
            minimum(sharedMinimum[threadIdx.x], sharedMinimum[partner]);

        sharedMaximum[threadIdx.x] =
            maximum(sharedMaximum[threadIdx.x], sharedMaximum[partner]);
      }
    }

    __syncthreads();
    active = nextActive;
  }

  if (threadIdx.x == 0u) {
    if (bucketStart == bucketEnd || sharedMinimum[0] > sharedMaximum[0]) {
      plane.yMin[bucket] = baseRanges.yMin;
      plane.yMax[bucket] = baseRanges.yMax;
    } else {
      plane.yMin[bucket] = sharedMinimum[0];
      plane.yMax[bucket] = sharedMaximum[0];
    }
  }
}

template <PeakFinder peakFinder>
__global__ void fillEtaDriftCirclesMdtBatchKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays maxima, HoughAxisRanges baseRanges,
    CoordType widthScale, CoordType maxWidth, YieldType weight) {
  const std::uint32_t nCells = plane.nBinsX * plane.nBinsY;

  // 1. Allocate the shared mem
  extern __shared__ unsigned char sharedMemory[];

  auto* sharedHits = reinterpret_cast<YieldType*>(sharedMemory);

  auto* sharedLayers = sharedHits + nCells;

  const std::size_t layerMaskOffset =
      HoughDetail::alignUp(2u * nCells * sizeof(YieldType), alignof(LayerMask));

  auto* sharedLayerMask =
      reinterpret_cast<LayerMask*>(sharedMemory + layerMaskOffset);

  const std::size_t associatedHitsOffset =
      HoughDetail::alignUp(layerMaskOffset + nCells * sizeof(LayerMask),
                           alignof(std::uint32_t));

  auto* sharedAssociatedHits = reinterpret_cast<std::uint32_t*>(
      sharedMemory + associatedHitsOffset);

  const std::size_t peakMaskOffset =
      associatedHitsOffset + nCells * sizeof(std::uint32_t);

  auto* sharedPeakMask =
      reinterpret_cast<std::uint8_t*>(sharedMemory + peakMaskOffset);

  constexpr std::size_t peakMaskElementSize =
      peakFinder == PeakFinder::SlidingWindow ? sizeof(std::uint8_t) : 0u;

  const std::size_t candidateOffset = HoughDetail::alignUp(
      peakMaskOffset + nCells * peakMaskElementSize,
      alignof(PeakFinders::GlobalMaximumCandidate));

  auto* sharedCandidates =
      reinterpret_cast<PeakFinders::GlobalMaximumCandidate*>(sharedMemory +
                                                             candidateOffset);

  // 2. Start grid stride loop over buckets
  for (std::uint32_t bucket = blockIdx.x; bucket < plane.nBuckets;
       bucket += gridDim.x) {
    // 2.1 Each bucket has own ranges
    const HoughAxisRanges ranges{baseRanges.xMin, baseRanges.xMax,
                                 plane.yMin[bucket], plane.yMax[bucket]};

    // 2.2 Prepare shared memory
    for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
         localBin += blockDim.x) {
      sharedHits[localBin] = YieldType{0.0};
      sharedLayers[localBin] = YieldType{0.0};
      sharedLayerMask[localBin] = LayerMask{0ull};
      sharedAssociatedHits[localBin] = 0u;
    }

    __syncthreads();

    // 2.3 Definitions for local ranges
    const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];

    const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

    const std::uint32_t nHits = bucketEnd - bucketStart;

    const std::uint32_t nTasks = nHits * plane.nBinsX;

    // 2.4 Start thread stride loop
    for (std::uint32_t task = threadIdx.x; task < nTasks; task += blockDim.x) {
      // 2.5.1 Calculate the variables
      const std::uint32_t xBin = task % plane.nBinsX;

      const std::uint32_t localHit = task / plane.nBinsX;

      const std::uint32_t hitIndex = bucketStart + localHit;

      const CoordType tanTheta = HoughDetail::binCenterDevice(
          ranges.xMin, ranges.xMax, plane.nBinsX, xBin);

      const CoordType y = spacePoints.localPositionY[hitIndex];

      const CoordType z = spacePoints.localPositionZ[hitIndex];

      const CoordType radius = spacePoints.driftRadius[hitIndex];

      const CoordType centralIntercept = y - tanTheta * z;

      const CoordType projectedRadius =
          radius * sqrt(CoordType{1.0} + tanTheta * tanTheta);

      const CoordType negativeIntercept = centralIntercept - projectedRadius;

      const CoordType positiveIntercept = centralIntercept + projectedRadius;

      const CoordType covariance = spacePoints.covariance1[hitIndex];

      const unsigned layer = detLayer(spacePoints.muonId[hitIndex]);

      const CoordType width = etaYHalfWidth(ranges, plane.nBinsX, z, radius,
                                            covariance, widthScale, maxWidth);

      // 2.5.2 Fill both solution bands. The accumulator deliberately counts
      // both contributions, including an overlap between the two bands.
      const HoughDetail::YBinRange negativeBand =
          HoughDetail::fillSharedYBand(
              sharedHits, sharedLayers, sharedLayerMask, plane, ranges, xBin,
              negativeIntercept, width, layer, weight);
      const HoughDetail::YBinRange positiveBand =
          HoughDetail::fillSharedYBand(
              sharedHits, sharedLayers, sharedLayerMask, plane, ranges, xBin,
              positiveIntercept, width, layer, weight);

      // Association counts are different: one input hit contributes at most
      // once to a cell, even when both drift-circle solutions cover it.
      HoughDetail::countSharedDistinctYBands(
          sharedAssociatedHits, plane.nBinsX, xBin, negativeBand,
          positiveBand);
    }

    __syncthreads();

    // 2.6 Find and append the configured maxima.
    if constexpr (peakFinder == PeakFinder::GlobalMaximum) {
      const PeakFinders::GlobalMaximumCandidate peak =
          PeakFinders::findGlobalMaximum(sharedHits, nCells, sharedCandidates);

      if (threadIdx.x == 0u) {
        PeakFinders::appendEtaMaximum(maxima, plane, ranges, sharedLayers,
                                      sharedLayerMask, sharedAssociatedHits,
                                      bucket, peak);
      }
    } else {
      constexpr PeakFinders::SlidingWindowConfig config{};
      PeakFinders::findSlidingWindowPeaks(
          sharedHits, plane.nBinsX, plane.nBinsY, config, sharedPeakMask);

      if (threadIdx.x == 0u) {
        // Preserve the CPU implementation's deterministic x-major ordering.
        for (std::uint32_t xBin = 0u; xBin < plane.nBinsX; ++xBin) {
          for (std::uint32_t yBin = 0u; yBin < plane.nBinsY; ++yBin) {
            std::uint32_t localBin = yBin * plane.nBinsX + xBin;
            if (sharedPeakMask[localBin] == 0u) {
              continue;
            }

            localBin = PeakFinders::slidingWindowRecenter(
                sharedHits, plane.nBinsX, plane.nBinsY, localBin, config);

            const PeakFinders::GlobalMaximumCandidate peak{
                sharedHits[localBin], localBin};
            PeakFinders::appendEtaMaximum(
                maxima, plane, ranges, sharedLayers, sharedLayerMask,
                sharedAssociatedHits, bucket, peak);
          }
        }
      }
    }

    __syncthreads();
  }
}

__global__ void fillEtaMaximumHitIndicesKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays maxima, HoughAxisRanges baseRanges,
    CoordType widthScale, CoordType maxWidth) {
  const std::uint32_t maximumIndex = blockIdx.x;
  const std::uint32_t totalMaximumSlots =
      maxima.nBuckets * maxima.capacityPerBucket;

  if (maximumIndex >= totalMaximumSlots) {
    return;
  }

  const std::uint32_t bucket = maximumIndex / maxima.capacityPerBucket;

  const std::uint32_t maximum = maximumIndex % maxima.capacityPerBucket;

  if (maximum >= maxima.nMaxima[bucket]) {
    return;
  }

  const std::uint32_t outputBegin = maxima.associatedHitOffsets[maximumIndex];

  const std::uint32_t outputEnd =
      maxima.associatedHitOffsets[maximumIndex + 1u];

  __shared__ std::uint32_t sharedWriteIndex;

  if (threadIdx.x == 0u) {
    sharedWriteIndex = 0u;
  }

  __syncthreads();

  const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];
  const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

  for (std::uint32_t hitIndex = bucketStart + threadIdx.x; hitIndex < bucketEnd;
       hitIndex += blockDim.x) {
    if (!etaHitContributesToMaximum(plane, spacePoints, maxima, baseRanges,
                                    widthScale, maxWidth, bucket, maximum,
                                    hitIndex)) {
      continue;
    }

    const std::uint32_t localWriteIndex = atomicAdd(&sharedWriteIndex, 1u);

    const std::uint32_t outputIndex = outputBegin + localWriteIndex;

    if (outputIndex < outputEnd) {
      maxima.associatedHitIndices[outputIndex] = hitIndex;
    }
  }
}

}  // namespace

namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform::detail {

void etaHoughTransformImpl(CudaHoughPlaneBatch& plane,
                           CudaMuonSpacePointContainer& spacePoints,
                           CudaHoughMaximumBatchArrays maxima,
                           const HoughAxisRanges& axisRanges, YieldType weight,
                           std::uint32_t threadsPerBlock,
                           std::uint32_t numBlocks, PeakFinder peakFinder) {
  if (threadsPerBlock == 0u) {
    throw std::invalid_argument("threadsPerBlock must be non-zero");
  }

  if (plane.nBuckets() != spacePoints.bucketCount()) {
    throw std::invalid_argument(
        "Eta Hough plane and space-point container must have the same bucket "
        "count");
  }

  if (maxima.nBuckets != plane.nBuckets()) {
    throw std::invalid_argument(
        "Eta Hough plane and maximum batch must have the same bucket count");
  }

  if (maxima.capacityPerBucket == 0u || maxima.nMaxima == nullptr ||
      maxima.nAssociatedHits == nullptr || maxima.xBin == nullptr ||
      maxima.yBin == nullptr) {
    throw std::invalid_argument("Invalid Hough maximum device storage");
  }

  if (!spacePoints.isOnDevice()) {
    spacePoints.moveToDevice();
  }

  if (!plane.isOnDevice()) {
    plane.moveToDevice();
  }

  ACTS_CUDA_CHECK(
      cudaMemset(maxima.nMaxima, 0, plane.nBuckets() * sizeof(std::uint32_t)));

  const std::size_t totalMaximumSlots =
      static_cast<std::size_t>(maxima.nBuckets) * maxima.capacityPerBucket;

  ACTS_CUDA_CHECK(cudaMemset(maxima.nAssociatedHits, 0,
                             totalMaximumSlots * sizeof(std::uint32_t)));

  int device = 0;
  ACTS_CUDA_CHECK(cudaGetDevice(&device));

  int multiprocessorCount = 0;
  ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
      &multiprocessorCount, cudaDevAttrMultiProcessorCount, device));

  int maximumSharedMemory = 0;
  ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
      &maximumSharedMemory, cudaDevAttrMaxSharedMemoryPerBlock, device));

  int maximumThreadsPerBlock = 0;
  ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
      &maximumThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, device));

  if (threadsPerBlock > static_cast<std::uint32_t>(maximumThreadsPerBlock)) {
    throw std::runtime_error("threadsPerBlock exceeds the CUDA device limit");
  }

  const std::size_t sharedBytes = HoughDetail::sharedBytesForEtaHough(
      plane.nCellsPerBucket(), threadsPerBlock, peakFinder);

  if (sharedBytes > static_cast<std::size_t>(maximumSharedMemory)) {
    throw std::runtime_error(
        "Eta Hough transform requires too much shared memory");
  }

  if (numBlocks == 0u) {
    numBlocks = static_cast<std::uint32_t>(multiprocessorCount);
  }

  numBlocks = std::min(numBlocks, static_cast<std::uint32_t>(plane.nBuckets()));

  if (numBlocks == 0u) {
    throw std::runtime_error("Resolved number of CUDA blocks is zero");
  }

  constexpr CoordType interceptMargin = 10.0 * Acts::UnitConstants::cm;

  const std::size_t rangeSharedBytes =
      2u * static_cast<std::size_t>(threadsPerBlock) * sizeof(CoordType);

  computeEtaInterceptRangesMdtBatchKernel<<<
      static_cast<unsigned>(plane.nBuckets()),
      static_cast<unsigned>(threadsPerBlock), rangeSharedBytes>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), axisRanges,
      interceptMargin);

  ACTS_CUDA_CHECK(cudaGetLastError());

  if (peakFinder == PeakFinder::GlobalMaximum) {
    fillEtaDriftCirclesMdtBatchKernel<PeakFinder::GlobalMaximum>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), maxima,
            axisRanges, etaWidthScale, etaMaxWidth, weight);
  } else if (peakFinder == PeakFinder::SlidingWindow) {
    fillEtaDriftCirclesMdtBatchKernel<PeakFinder::SlidingWindow>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), maxima,
            axisRanges, etaWidthScale, etaMaxWidth, weight);
  } else {
    throw std::invalid_argument("Unknown CUDA Hough peak finder");
  }

  ACTS_CUDA_CHECK(cudaGetLastError());

  ACTS_CUDA_CHECK(cudaDeviceSynchronize());
}

void fillEtaHitAssociationsImpl(CudaHoughPlaneBatch& plane,
                                CudaMuonSpacePointContainer& spacePoints,
                                CudaHoughMaximumBatchArrays maxima,
                                const HoughAxisRanges& axisRanges,
                                std::uint32_t threadsPerBlock) {
  if (threadsPerBlock == 0u) {
    throw std::invalid_argument("threadsPerBlock must be non-zero");
  }

  if (!plane.isOnDevice() || !spacePoints.isOnDevice()) {
    throw std::logic_error(
        "Eta Hough plane and space points must remain on the device");
  }

  if (maxima.nBuckets != plane.nBuckets()) {
    throw std::invalid_argument(
        "Eta Hough plane and maximum batch must have the same bucket count");
  }

  if (maxima.nMaxima == nullptr || maxima.associatedHitOffsets == nullptr) {
    throw std::invalid_argument("Invalid Hough maximum association storage");
  }

  if (maxima.totalAssociatedHits == 0u) {
    return;
  }

  if (maxima.associatedHitIndices == nullptr) {
    throw std::invalid_argument(
        "Associated-hit index storage is not allocated");
  }

  int device = 0;
  ACTS_CUDA_CHECK(cudaGetDevice(&device));

  int maximumThreadsPerBlock = 0;
  ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
      &maximumThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, device));

  if (threadsPerBlock > static_cast<std::uint32_t>(maximumThreadsPerBlock)) {
    throw std::runtime_error("threadsPerBlock exceeds the CUDA device limit");
  }

  const std::size_t totalMaximumSlots =
      static_cast<std::size_t>(maxima.nBuckets) * maxima.capacityPerBucket;

  fillEtaMaximumHitIndicesKernel<<<static_cast<unsigned>(totalMaximumSlots),
                                   static_cast<unsigned>(threadsPerBlock)>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), maxima, axisRanges,
      etaWidthScale, etaMaxWidth);

  ACTS_CUDA_CHECK(cudaGetLastError());
  ACTS_CUDA_CHECK(cudaDeviceSynchronize());
}

}  // namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform::detail
