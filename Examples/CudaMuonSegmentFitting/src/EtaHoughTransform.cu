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
using ActsExamples::muonIdIsMdt;
using ActsExamples::muonIdMeasuresEta;

using ActsExamples::CudaHoughTransformUtils::CoordType;
using ActsExamples::CudaHoughTransformUtils::CudaHoughPlaneBatchArrays;
using ActsExamples::CudaHoughTransformUtils::HoughAxisRanges;
using ActsExamples::CudaHoughTransformUtils::LayerMask;
using ActsExamples::CudaHoughTransformUtils::PeakFinder;
using ActsExamples::CudaHoughTransformUtils::YieldType;

namespace HoughDetail = ActsExamples::CudaHoughTransformUtils::detail;
namespace PeakFinders = ActsExamples::CudaHoughTransformUtils::PeakFinders;

// Top-hat measurement widths. MDT bands also include the variation across one
// tanBeta bin in etaYHalfWidth.
constexpr CoordType etaWidthScale = 1.0;
constexpr CoordType etaMaxWidth = 1.0 * Acts::UnitConstants::mm;
constexpr CoordType etaStripWidthScale = 3.0;

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

__device__ inline CoordType etaStripHalfWidth(CoordType covariance) {
  return sqrt(covariance > CoordType{0.0} ? covariance : CoordType{0.0}) *
         etaStripWidthScale;
}

struct EtaHitBands {
  CoordType central = 0.0;
  CoordType negative = 0.0;
  CoordType positive = 0.0;
  CoordType halfWidth = 0.0;
  bool isMdt = false;
};

/// Evaluate the Hough bands once so filling and hit association use identical
/// geometry and uncertainty broadening.
__device__ inline EtaHitBands evaluateEtaHitBands(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    const HoughAxisRanges& ranges, std::uint32_t xBin,
    std::uint32_t hitIndex, std::uint32_t rawId, CoordType widthScale,
    CoordType maxWidth) {
  const bool isMdt = muonIdIsMdt(rawId);
  const CoordType tanBeta = HoughDetail::binCenterDevice(
      ranges.xMin, ranges.xMax, plane.nBinsX, xBin);
  const CoordType y = spacePoints.localPositionY[hitIndex];
  const CoordType z = spacePoints.localPositionZ[hitIndex];
  const CoordType radius =
      isMdt ? spacePoints.driftRadius[hitIndex] : CoordType{0.0};
  const CoordType central = y - tanBeta * z;
  const CoordType projectedRadius =
      radius * sqrt(CoordType{1.0} + tanBeta * tanBeta);
  const CoordType covariance = spacePoints.covariance1[hitIndex];
  const CoordType halfWidth =
      isMdt ? etaYHalfWidth(ranges, plane.nBinsX, z, radius, covariance,
                            widthScale, maxWidth)
            : etaStripHalfWidth(covariance);

  return {central, central - projectedRadius, central + projectedRadius,
          halfWidth, isMdt};
}

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

  const std::uint32_t rawId = spacePoints.muonId[hitIndex];
  if (!muonIdMeasuresEta(rawId)) {
    return false;
  }
  const EtaHitBands bands = evaluateEtaHitBands(
      plane, spacePoints, ranges, selectedXBin, hitIndex, rawId, widthScale,
      maxWidth);

  if (!bands.isMdt) {
    return yBinInsideBand(selectedYBin, bands.central, bands.halfWidth, ranges,
                          plane.nBinsY);
  }

  return yBinInsideBand(selectedYBin, bands.negative, bands.halfWidth, ranges,
                        plane.nBinsY) ||
         yBinInsideBand(selectedYBin, bands.positive, bands.halfWidth, ranges,
                        plane.nBinsY);
}

__global__ void computeEtaInterceptRangesBatchKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    HoughAxisRanges baseRanges, CoordType interceptMargin) {
  extern __shared__ CoordType sharedRanges[];

  CoordType* sharedMinimum = sharedRanges;
  CoordType* sharedMaximum = sharedMinimum + blockDim.x;

  constexpr CoordType infinity = 1.0e100;

  for (std::uint32_t bucket = blockIdx.x; bucket < plane.nBuckets;
       bucket += gridDim.x) {
    CoordType localMinimum = infinity;
    CoordType localMaximum = -infinity;

    const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];
    const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

    for (std::uint32_t hit = bucketStart + threadIdx.x; hit < bucketEnd;
         hit += blockDim.x) {
      if (!muonIdMeasuresEta(spacePoints.muonId[hit])) {
        continue;
      }
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
    __syncthreads();
  }
}

template <PeakFinder peakFinder>
__global__ void fillEtaSpacePointsBatchKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays maxima, HoughAxisRanges baseRanges,
    CoordType widthScale, CoordType maxWidth, YieldType weight) {
  const std::uint32_t nCells = plane.nBinsX * plane.nBinsY;

  // Lay out the transient accumulator and peak-finding workspace.
  extern __shared__ unsigned char sharedMemory[];
  const auto workspace =
      HoughDetail::makeHoughSharedWorkspace<peakFinder>(sharedMemory, nCells);
  YieldType* sharedHits = workspace.hits;
  YieldType* sharedLayers = workspace.layers;
  LayerMask* sharedLayerMask = workspace.layerMasks;
  std::uint32_t* sharedAssociatedHits = workspace.associatedHits;
  std::uint8_t* sharedPeakMask = workspace.peakMask;
  auto* sharedCandidates = workspace.candidates;

  for (std::uint32_t bucket = blockIdx.x; bucket < plane.nBuckets;
       bucket += gridDim.x) {
    const HoughAxisRanges ranges{baseRanges.xMin, baseRanges.xMax,
                                 plane.yMin[bucket], plane.yMax[bucket]};

    for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
         localBin += blockDim.x) {
      sharedHits[localBin] = YieldType{0.0};
      sharedLayers[localBin] = YieldType{0.0};
      sharedLayerMask[localBin] = LayerMask{0ull};
      sharedAssociatedHits[localBin] = 0u;
    }

    __syncthreads();

    const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];

    const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

    const std::uint32_t nHits = bucketEnd - bucketStart;

    const std::uint32_t nTasks = nHits * plane.nBinsX;

    for (std::uint32_t task = threadIdx.x; task < nTasks; task += blockDim.x) {
      const std::uint32_t xBin = task % plane.nBinsX;

      const std::uint32_t localHit = task / plane.nBinsX;

      const std::uint32_t hitIndex = bucketStart + localHit;

      const std::uint32_t rawId = spacePoints.muonId[hitIndex];
      if (!muonIdMeasuresEta(rawId)) {
        continue;
      }
      const unsigned layer = detLayer(rawId);
      const EtaHitBands bands = evaluateEtaHitBands(
          plane, spacePoints, ranges, xBin, hitIndex, rawId, widthScale,
          maxWidth);

      if (!bands.isMdt) {
        const HoughDetail::YBinRange stripBand =
            HoughDetail::fillSharedYBand(
                sharedHits, sharedLayers, sharedLayerMask, plane, ranges, xBin,
                bands.central, bands.halfWidth, layer, weight);
        HoughDetail::countSharedYBand(sharedAssociatedHits, plane.nBinsX, xBin,
                                      stripBand);
        continue;
      }

      // Fill both solution bands. The accumulator deliberately counts
      // both contributions, including an overlap between the two bands.
      const HoughDetail::YBinRange negativeBand =
          HoughDetail::fillSharedYBand(
              sharedHits, sharedLayers, sharedLayerMask, plane, ranges, xBin,
              bands.negative, bands.halfWidth, layer, weight);
      const HoughDetail::YBinRange positiveBand =
          HoughDetail::fillSharedYBand(
              sharedHits, sharedLayers, sharedLayerMask, plane, ranges, xBin,
              bands.positive, bands.halfWidth, layer, weight);

      // Association counts are different: one input hit contributes at most
      // once to a cell, even when both drift-circle solutions cover it.
      HoughDetail::countSharedDistinctYBands(
          sharedAssociatedHits, plane.nBinsX, xBin, negativeBand,
          positiveBand);
    }

    __syncthreads();

    PeakFinders::RelativeNmsConfig nmsConfig{};
    PeakFinders::SlidingWindowConfig slidingConfig{};
    slidingConfig.xWindowSize = PeakFinders::relativeBinRadius(
        plane.nBinsX, nmsConfig.localWindowFraction);
    slidingConfig.yWindowSize = PeakFinders::relativeBinRadius(
        plane.nBinsY, nmsConfig.localWindowFraction);
    PeakFinders::findAndAppendMaxima<peakFinder>(
        maxima, plane, ranges, sharedHits, sharedLayers, sharedLayerMask,
        sharedAssociatedHits, sharedPeakMask, sharedCandidates, bucket,
        slidingConfig, nmsConfig);

    __syncthreads();
  }
}

__global__ void fillEtaMaximumHitIndicesKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays maxima, HoughAxisRanges baseRanges,
    CoordType widthScale, CoordType maxWidth) {
  const std::uint32_t totalMaximumSlots =
      maxima.nBuckets * maxima.capacityPerBucket;

  __shared__ std::uint32_t sharedWriteIndex;

  for (std::uint32_t maximumIndex = blockIdx.x;
       maximumIndex < totalMaximumSlots; maximumIndex += gridDim.x) {
    const std::uint32_t bucket =
        maximumIndex / maxima.capacityPerBucket;
    const std::uint32_t maximum =
        maximumIndex % maxima.capacityPerBucket;

    if (maximum >= maxima.nMaxima[bucket]) {
      continue;
    }

    const std::uint32_t outputBegin =
        maxima.associatedHitOffsets[maximumIndex];
    const std::uint32_t outputEnd =
        maxima.associatedHitOffsets[maximumIndex + 1u];

    if (threadIdx.x == 0u) {
      sharedWriteIndex = 0u;
    }
    __syncthreads();

    const std::uint32_t bucketStart = spacePoints.bucketStart[bucket];
    const std::uint32_t bucketEnd = spacePoints.bucketEnd[bucket];

    for (std::uint32_t hitIndex = bucketStart + threadIdx.x;
         hitIndex < bucketEnd; hitIndex += blockDim.x) {
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
    __syncthreads();
  }
}

}  // namespace

namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform::detail {

void etaHoughTransformImpl(CudaHoughPlaneBatch& plane,
                           CudaMuonSpacePointContainer& spacePoints,
                           CudaHoughMaximumBatchArrays maxima,
                           const HoughAxisRanges& axisRanges, YieldType weight,
                           std::uint32_t threadsPerBlock,
                           std::uint32_t numBlocks, PeakFinder peakFinder,
                           cudaStream_t stream) {
  const bool limitAuxiliaryGrids = numBlocks != 0u;

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
    spacePoints.moveToDevice(stream);
  }

  if (!plane.isOnDevice()) {
    plane.moveToDevice(stream);
  }

  ACTS_CUDA_CHECK(cudaMemsetAsync(maxima.nMaxima, 0,
                                  plane.nBuckets() * sizeof(std::uint32_t),
                                  stream));

  const std::size_t totalMaximumSlots =
      static_cast<std::size_t>(maxima.nBuckets) * maxima.capacityPerBucket;

  ACTS_CUDA_CHECK(cudaMemsetAsync(
      maxima.nAssociatedHits, 0,
      totalMaximumSlots * sizeof(std::uint32_t), stream));

  int device = 0;
  ACTS_CUDA_CHECK(cudaGetDevice(&device));

  int maximumSharedMemory = 0;
  ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
      &maximumSharedMemory, cudaDevAttrMaxSharedMemoryPerBlock, device));

  int maximumThreadsPerBlock = 0;
  ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
      &maximumThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, device));

  if (threadsPerBlock > static_cast<std::uint32_t>(maximumThreadsPerBlock)) {
    throw std::runtime_error("threadsPerBlock exceeds the CUDA device limit");
  }

  const std::size_t sharedBytes = HoughDetail::sharedBytesForHough(
      plane.nCellsPerBucket(), threadsPerBlock, peakFinder);

  if (sharedBytes > static_cast<std::size_t>(maximumSharedMemory)) {
    throw std::runtime_error(
        "Eta Hough transform requires too much shared memory");
  }

  if (numBlocks == 0u) {
    int multiprocessorCount = 0;
    ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
        &multiprocessorCount, cudaDevAttrMultiProcessorCount, device));
    numBlocks = static_cast<std::uint32_t>(multiprocessorCount);
  }

  numBlocks = std::min(numBlocks, static_cast<std::uint32_t>(plane.nBuckets()));

  if (numBlocks == 0u) {
    throw std::runtime_error("Resolved number of CUDA blocks is zero");
  }

  constexpr CoordType interceptMargin = 10.0 * Acts::UnitConstants::cm;

  const std::size_t rangeSharedBytes =
      2u * static_cast<std::size_t>(threadsPerBlock) * sizeof(CoordType);
  const std::uint32_t rangeBlocks =
      limitAuxiliaryGrids ? numBlocks
                          : static_cast<std::uint32_t>(plane.nBuckets());

  computeEtaInterceptRangesBatchKernel<<<
      static_cast<unsigned>(rangeBlocks),
      static_cast<unsigned>(threadsPerBlock), rangeSharedBytes, stream>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), axisRanges,
      interceptMargin);

  ACTS_CUDA_CHECK(cudaGetLastError());

  if (peakFinder == PeakFinder::GlobalMaximum) {
    fillEtaSpacePointsBatchKernel<PeakFinder::GlobalMaximum>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes, stream>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), maxima,
            axisRanges, etaWidthScale, etaMaxWidth, weight);
  } else if (peakFinder == PeakFinder::SlidingWindow) {
    fillEtaSpacePointsBatchKernel<PeakFinder::SlidingWindow>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes, stream>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), maxima,
            axisRanges, etaWidthScale, etaMaxWidth, weight);
  } else if (peakFinder == PeakFinder::RelativeNms) {
    fillEtaSpacePointsBatchKernel<PeakFinder::RelativeNms>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes, stream>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), maxima,
            axisRanges, etaWidthScale, etaMaxWidth, weight);
  } else {
    throw std::invalid_argument("Unknown CUDA Hough peak finder");
  }

  ACTS_CUDA_CHECK(cudaGetLastError());

  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
}

void fillEtaHitAssociationsImpl(CudaHoughPlaneBatch& plane,
                                CudaMuonSpacePointContainer& spacePoints,
                                CudaHoughMaximumBatchArrays maxima,
                                const HoughAxisRanges& axisRanges,
                                std::uint32_t threadsPerBlock,
                                std::uint32_t numBlocks,
                                cudaStream_t stream) {
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

  if (numBlocks == 0u) {
    numBlocks = static_cast<std::uint32_t>(totalMaximumSlots);
  }
  numBlocks = std::min(
      numBlocks, static_cast<std::uint32_t>(totalMaximumSlots));

  fillEtaMaximumHitIndicesKernel<<<static_cast<unsigned>(numBlocks),
                                   static_cast<unsigned>(threadsPerBlock), 0u,
                                   stream>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), maxima, axisRanges,
      etaWidthScale, etaMaxWidth);

  ACTS_CUDA_CHECK(cudaGetLastError());
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
}

}  // namespace ActsExamples::CudaHoughTransformUtils::EtaHoughTransform::detail
