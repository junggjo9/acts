// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Acts/Definitions/Units.hpp"
#include "ActsExamples/Algorithms/TrackFinding/EtaHoughTransform.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.cuh"
#include "ActsExamples/Utilities/CudaUtilities.hpp"
#include "ActsExamples/EventData/CudaMuonHoughMaximum.hpp"

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
using ActsExamples::CudaHoughTransformUtils::YieldType;

constexpr CoordType etaWidthScale = 0.0;
constexpr CoordType etaMaxWidth = 0.0;

namespace HoughDetail = ActsExamples::CudaHoughTransformUtils::detail;

namespace PeakFinders = ActsExamples::CudaHoughTransformUtils::PeakFinders;

__device__ inline CoordType minimum(CoordType first, CoordType second) {
  return first < second ? first : second;
}

__device__ inline CoordType maximum(CoordType first, CoordType second) {
  return first > second ? first : second;
}

// Reverse of fillSharedYBand
__device__ inline bool yBinInsideBand(
    std::uint32_t selectedYBin, CoordType center, CoordType halfWidth,
    const HoughAxisRanges& ranges, std::uint32_t nBinsY) {
  int yBinDown = HoughDetail::binIndexDevice(
      ranges.yMin, ranges.yMax, nBinsY, center - halfWidth);

  int yBinUp = HoughDetail::binIndexDevice(
      ranges.yMin, ranges.yMax, nBinsY, center + halfWidth);

  if (yBinDown > yBinUp) {
    const int temporary = yBinDown;
    yBinDown = yBinUp;
    yBinUp = temporary;
  }

  if (yBinDown < 0) {
    yBinDown = 0;
  }

  if (yBinUp >= static_cast<int>(nBinsY)) {
    yBinUp = static_cast<int>(nBinsY) - 1;
  }

  const int selected = static_cast<int>(selectedYBin);
  return selected >= yBinDown && selected <= yBinUp;
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

  const CoordType covariance = spacePoints.covariance1[hitIndex] > 0.0
                                   ? spacePoints.covariance1[hitIndex]
                                   : CoordType{0.0};

  CoordType width = sqrt(covariance) * widthScale;

  if (width > maxWidth) {
    width = maxWidth;
  }

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

  const std::size_t candidateOffset =
      HoughDetail::alignUp(layerMaskOffset + nCells * sizeof(LayerMask),
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
    }

    __syncthreads();

    // 2.3 Definitions for local ranges
    const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];

    const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

    const std::uint32_t nHits = bucketEnd - bucketStart;

    constexpr std::uint32_t nSolutions = 2u;

    const std::uint32_t nTasks = nHits * plane.nBinsX * nSolutions;

    // 2.4 Start thread stride loop
    for (std::uint32_t task = threadIdx.x; task < nTasks; task += blockDim.x) {
      // 2.5.1 Calculate the varaibles
      const std::uint32_t solution = task % nSolutions;

      const std::uint32_t xBin = (task / nSolutions) % plane.nBinsX;

      const std::uint32_t localHit = task / (nSolutions * plane.nBinsX);

      const std::uint32_t hitIndex = bucketStart + localHit;

      const CoordType tanTheta = HoughDetail::binCenterDevice(
          ranges.xMin, ranges.xMax, plane.nBinsX, xBin);

      const CoordType y = spacePoints.localPositionY[hitIndex];

      const CoordType z = spacePoints.localPositionZ[hitIndex];

      const CoordType radius = spacePoints.driftRadius[hitIndex];

      const CoordType sign = solution == 0u ? CoordType{-1.0} : CoordType{1.0};

      const CoordType intercept =
          y - tanTheta * z +
          sign * radius * sqrt(CoordType{1.0} + tanTheta * tanTheta);

      const CoordType covariance = spacePoints.covariance1[hitIndex] > 0.0
                                       ? spacePoints.covariance1[hitIndex]
                                       : CoordType{0.0};

      CoordType width = sqrt(covariance) * widthScale;

      if (width > maxWidth) {
        width = maxWidth;
      }

      const unsigned layer = detLayer(spacePoints.muonId[hitIndex]);

      // 2.5.2 Fill the band
      HoughDetail::fillSharedYBand(sharedHits, sharedLayers, sharedLayerMask,
                                   plane, ranges, xBin, intercept, width, layer,
                                   weight);
    }

    __syncthreads();

    // 2.6 Find global Maximum
    const PeakFinders::GlobalMaximumCandidate peak =
        PeakFinders::findGlobalMaximum(sharedHits, nCells, sharedCandidates);

    // 2.7 Append maximum to list of maximums of bucket
    if (threadIdx.x == 0u) {
      PeakFinders::appendEtaMaximum(maxima, plane, ranges, sharedLayers,
                                    sharedLayerMask, bucket, peak);
    }

    __syncthreads();

    // 2.8 Save the full Hough plane into global memory 
    // Can be later omitted
    const std::uint32_t globalBase = bucket * nCells;

    for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
         localBin += blockDim.x) {
      plane.nHits[globalBase + localBin] = sharedHits[localBin];

      plane.nLayers[globalBase + localBin] = sharedLayers[localBin];

      plane.layerMask[globalBase + localBin] = sharedLayerMask[localBin];
    }

    __syncthreads();
  }
}

__global__ void countEtaMaximumHitsKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays maxima, HoughAxisRanges baseRanges,
    CoordType widthScale, CoordType maxWidth) {
  const std::uint32_t maximumIndex = blockIdx.x;
  const std::uint32_t totalMaximumSlots =
      maxima.nBuckets * maxima.capacityPerBucket;

  if (maximumIndex >= totalMaximumSlots) {
    return;
  }

  const std::uint32_t bucket =
      maximumIndex / maxima.capacityPerBucket;

  const std::uint32_t maximum =
      maximumIndex % maxima.capacityPerBucket;

  if (maximum >= maxima.nMaxima[bucket]) {
    if (threadIdx.x == 0u) {
      maxima.nAssociatedHits[maximumIndex] = 0u;
    }
    return;
  }

  __shared__ std::uint32_t sharedCount;

  if (threadIdx.x == 0u) {
    sharedCount = 0u;
  }

  __syncthreads();

  const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];
  const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

  std::uint32_t localCount = 0u;

  for (std::uint32_t hitIndex = bucketStart + threadIdx.x;
       hitIndex < bucketEnd; hitIndex += blockDim.x) {
    if (etaHitContributesToMaximum(
            plane, spacePoints, maxima, baseRanges, widthScale, maxWidth,
            bucket, maximum, hitIndex)) {
      ++localCount;
    }
  }

  if (localCount != 0u) {
    atomicAdd(&sharedCount, localCount);
  }

  __syncthreads();

  if (threadIdx.x == 0u) {
    maxima.nAssociatedHits[maximumIndex] = sharedCount;
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

  const std::uint32_t bucket =
      maximumIndex / maxima.capacityPerBucket;

  const std::uint32_t maximum =
      maximumIndex % maxima.capacityPerBucket;

  if (maximum >= maxima.nMaxima[bucket]) {
    return;
  }

  const std::uint32_t outputBegin =
      maxima.associatedHitOffsets[maximumIndex];

  const std::uint32_t outputEnd =
      maxima.associatedHitOffsets[maximumIndex + 1u];

  __shared__ std::uint32_t sharedWriteIndex;

  if (threadIdx.x == 0u) {
    sharedWriteIndex = 0u;
  }

  __syncthreads();

  const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];
  const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

  for (std::uint32_t hitIndex = bucketStart + threadIdx.x;
       hitIndex < bucketEnd; hitIndex += blockDim.x) {
    if (!etaHitContributesToMaximum(
            plane, spacePoints, maxima, baseRanges, widthScale, maxWidth,
            bucket, maximum, hitIndex)) {
      continue;
    }

    const std::uint32_t localWriteIndex =
        atomicAdd(&sharedWriteIndex, 1u);

    const std::uint32_t outputIndex =
        outputBegin + localWriteIndex;

    if (outputIndex < outputEnd) {
      maxima.associatedHitIndices[outputIndex] = hitIndex;
    }
  }
}

}  // namespace

namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform::detail {

void etaHoughTransformImpl(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    CudaHoughMaximumBatchArrays maxima, const HoughAxisRanges& axisRanges,
    YieldType weight, std::uint32_t threadsPerBlock,
    std::uint32_t numBlocks) {
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

  if (maxima.capacityPerBucket == 0u || maxima.nMaxima == nullptr || maxima.nAssociatedHits == nullptr || maxima.xBin == nullptr || maxima.yBin == nullptr) {
    throw std::invalid_argument("Invalid Hough maximum device storage");
  }

  if (!spacePoints.isOnDevice()) {
    spacePoints.moveToDevice();
  }

  if (!plane.isOnDevice()) {
    plane.moveToDevice();
  }

  ACTS_CUDA_CHECK(cudaMemset(
      maxima.nMaxima, 0, plane.nBuckets() * sizeof(std::uint32_t)));

  const std::size_t totalMaximumSlots =
      static_cast<std::size_t>(maxima.nBuckets) *
      maxima.capacityPerBucket;

  ACTS_CUDA_CHECK(cudaMemset(
      maxima.nAssociatedHits, 0,
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

  if (threadsPerBlock >
      static_cast<std::uint32_t>(maximumThreadsPerBlock)) {
    throw std::runtime_error(
        "threadsPerBlock exceeds the CUDA device limit");
  }

  const std::size_t sharedBytes = HoughDetail::sharedBytesForEtaHough(
      plane.nCellsPerBucket(), threadsPerBlock);

  if (sharedBytes > static_cast<std::size_t>(maximumSharedMemory)) {
    throw std::runtime_error(
        "Eta Hough transform requires too much shared memory");
  }

  if (numBlocks == 0u) {
    numBlocks = static_cast<std::uint32_t>(multiprocessorCount);
  }

  numBlocks = std::min(
      numBlocks, static_cast<std::uint32_t>(plane.nBuckets()));

  if (numBlocks == 0u) {
    throw std::runtime_error(
        "Resolved number of CUDA blocks is zero");
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

  fillEtaDriftCirclesMdtBatchKernel<<<
      static_cast<unsigned>(numBlocks),
      static_cast<unsigned>(threadsPerBlock), sharedBytes>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), maxima, axisRanges,
      etaWidthScale, etaMaxWidth, weight);

  ACTS_CUDA_CHECK(cudaGetLastError());

  countEtaMaximumHitsKernel<<<
      static_cast<unsigned>(totalMaximumSlots),
      static_cast<unsigned>(threadsPerBlock)>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), maxima, axisRanges,
      etaWidthScale, etaMaxWidth);

  ACTS_CUDA_CHECK(cudaGetLastError());
  ACTS_CUDA_CHECK(cudaDeviceSynchronize());
}

void fillEtaHitAssociationsImpl(
    CudaHoughPlaneBatch& plane, CudaMuonSpacePointContainer& spacePoints,
    CudaHoughMaximumBatchArrays maxima, const HoughAxisRanges& axisRanges,
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

  if (maxima.nMaxima == nullptr ||
      maxima.associatedHitOffsets == nullptr) {
    throw std::invalid_argument(
        "Invalid Hough maximum association storage");
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

  if (threadsPerBlock >
      static_cast<std::uint32_t>(maximumThreadsPerBlock)) {
    throw std::runtime_error(
        "threadsPerBlock exceeds the CUDA device limit");
  }

  const std::size_t totalMaximumSlots =
      static_cast<std::size_t>(maxima.nBuckets) *
      maxima.capacityPerBucket;

  fillEtaMaximumHitIndicesKernel<<<
      static_cast<unsigned>(totalMaximumSlots),
      static_cast<unsigned>(threadsPerBlock)>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), maxima, axisRanges,
      etaWidthScale, etaMaxWidth);

  ACTS_CUDA_CHECK(cudaGetLastError());
  ACTS_CUDA_CHECK(cudaDeviceSynchronize());
}

}  // namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform::detail
