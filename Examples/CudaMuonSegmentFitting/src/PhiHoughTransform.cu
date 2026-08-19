// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Acts/Definitions/Units.hpp"
#include "ActsExamples/Algorithms/TrackFinding/PhiHoughTransform.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.cuh"
#include "ActsExamples/Utilities/CudaUtilities.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <cuda_runtime.h>

namespace {

using ActsExamples::CudaHoughMaximumBatchArrays;
using ActsExamples::CudaMuonSegmentSeedArrays;
using ActsExamples::CudaMuonSpacePointArrays;
using ActsExamples::detLayer;
using ActsExamples::muonIdMeasuresEta;
using ActsExamples::muonIdMeasuresPhi;

using ActsExamples::CudaHoughTransformUtils::CoordType;
using ActsExamples::CudaHoughTransformUtils::CudaHoughPlaneBatchArrays;
using ActsExamples::CudaHoughTransformUtils::HoughAxisRanges;
using ActsExamples::CudaHoughTransformUtils::LayerMask;
using ActsExamples::CudaHoughTransformUtils::PeakFinder;
using ActsExamples::CudaHoughTransformUtils::YieldType;

namespace HoughDetail = ActsExamples::CudaHoughTransformUtils::detail;
namespace PeakFinders = ActsExamples::CudaHoughTransformUtils::PeakFinders;

constexpr YieldType phiThreshold = 2.0f;
constexpr CoordType phiStripWidthScale = 3.0;

__device__ inline CoordType minimum(CoordType first, CoordType second) {
  return first < second ? first : second;
}

__device__ inline CoordType maximum(CoordType first, CoordType second) {
  return first > second ? first : second;
}

__device__ inline bool isPurePhiHit(std::uint32_t rawId) {
  return muonIdMeasuresPhi(rawId) && !muonIdMeasuresEta(rawId);
}

__device__ inline bool isEtaAssociatedPhiHit(std::uint32_t rawId) {
  return muonIdMeasuresPhi(rawId) && muonIdMeasuresEta(rawId);
}

__device__ inline bool etaMaximumIsOccupied(
    CudaHoughMaximumBatchArrays etaMaxima, std::uint32_t group) {
  const std::uint32_t etaBucket = group / etaMaxima.capacityPerBucket;
  const std::uint32_t etaMaximum = group % etaMaxima.capacityPerBucket;
  return etaMaximum < etaMaxima.nMaxima[etaBucket];
}

struct PhiInputRanges {
  std::uint32_t physicalBegin = 0u;
  std::uint32_t physicalEnd = 0u;
  std::uint32_t associatedBegin = 0u;
  std::uint32_t associatedEnd = 0u;
};

__device__ inline PhiInputRanges phiInputRanges(
    CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays etaMaxima, std::uint32_t group) {
  const std::uint32_t etaBucket = group / etaMaxima.capacityPerBucket;
  return {spacePoints.bucketStart[etaBucket],
          spacePoints.bucketEnd[etaBucket],
          etaMaxima.associatedHitOffsets[group],
          etaMaxima.associatedHitOffsets[group + 1u]};
}

__device__ inline CoordType phiStripHalfWidth(
    CudaMuonSpacePointArrays spacePoints, std::uint32_t hit) {
  const CoordType covariance = spacePoints.covariance0[hit];
  return phiStripWidthScale *
         sqrt(covariance > CoordType{0.0} ? covariance : CoordType{0.0});
}

__device__ inline CoordType phiIntercept(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    const HoughAxisRanges& ranges, std::uint32_t xBin, std::uint32_t hit) {
  const CoordType tanAlpha = HoughDetail::binCenterDevice(
      ranges.xMin, ranges.xMax, plane.nBinsX, xBin);
  return spacePoints.localPositionX[hit] -
         tanAlpha * spacePoints.localPositionZ[hit];
}

__device__ inline bool phiHitContributesToMaximum(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaMuonSegmentSeedArrays segmentSeeds, HoughAxisRanges baseRanges,
    std::uint32_t group, std::uint32_t maximum, std::uint32_t hit) {
  const std::uint32_t output = segmentSeeds.index(group, maximum);
  const HoughAxisRanges ranges{baseRanges.xMin, baseRanges.xMax,
                               plane.yMin[group], plane.yMax[group]};
  const CoordType intercept = phiIntercept(
      plane, spacePoints, ranges, segmentSeeds.xBin[output], hit);
  const HoughDetail::YBinRange band = HoughDetail::yBinRange(
      ranges, plane.nBinsY, intercept, phiStripHalfWidth(spacePoints, hit));
  const int selectedY = static_cast<int>(segmentSeeds.yBin[output]);
  return selectedY >= band.down && selectedY <= band.up;
}

__global__ void computePhiInterceptRangesBatchKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays etaMaxima, HoughAxisRanges baseRanges,
    CoordType interceptMargin) {
  extern __shared__ CoordType sharedRanges[];
  CoordType* sharedMinimum = sharedRanges;
  CoordType* sharedMaximum = sharedMinimum + blockDim.x;
  constexpr CoordType infinity = 1.0e100;

  for (std::uint32_t group = blockIdx.x; group < plane.nBuckets;
       group += gridDim.x) {
    if (!etaMaximumIsOccupied(etaMaxima, group)) {
      if (threadIdx.x == 0u) {
        plane.yMin[group] = baseRanges.yMin;
        plane.yMax[group] = baseRanges.yMax;
      }
      __syncthreads();
      continue;
    }

    const PhiInputRanges inputs =
        phiInputRanges(spacePoints, etaMaxima, group);
    CoordType localMinimum = infinity;
    CoordType localMaximum = -infinity;

    for (std::uint32_t hit = inputs.physicalBegin + threadIdx.x;
         hit < inputs.physicalEnd; hit += blockDim.x) {
      if (!isPurePhiHit(spacePoints.muonId[hit])) {
        continue;
      }
      const CoordType x = spacePoints.localPositionX[hit];
      localMinimum = minimum(localMinimum, x - interceptMargin);
      localMaximum = maximum(localMaximum, x + interceptMargin);
    }
    for (std::uint32_t offset = inputs.associatedBegin + threadIdx.x;
         offset < inputs.associatedEnd; offset += blockDim.x) {
      const std::uint32_t hit = etaMaxima.associatedHitIndices[offset];
      if (!isEtaAssociatedPhiHit(spacePoints.muonId[hit])) {
        continue;
      }
      const CoordType x = spacePoints.localPositionX[hit];
      localMinimum = minimum(localMinimum, x - interceptMargin);
      localMaximum = maximum(localMaximum, x + interceptMargin);
    }

    sharedMinimum[threadIdx.x] = localMinimum;
    sharedMaximum[threadIdx.x] = localMaximum;
    __syncthreads();

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
      const bool hasPhiHits = sharedMinimum[0] <= sharedMaximum[0];
      plane.yMin[group] = hasPhiHits ? sharedMinimum[0] : baseRanges.yMin;
      plane.yMax[group] = hasPhiHits ? sharedMaximum[0] : baseRanges.yMax;
    }
    __syncthreads();
  }
}

// Peak emission is serialized through thread zero of the owning block.
__device__ inline std::uint32_t reserveSegmentSeedSlot(
    CudaMuonSegmentSeedArrays seeds, std::uint32_t group) {
  const std::uint32_t seed = seeds.nSeeds[group];
  if (seed < seeds.capacityPerGroup) {
    seeds.nSeeds[group] = seed + 1u;
  }
  return seed;
}

__device__ inline void appendUnextendedEtaSeed(
    CudaMuonSegmentSeedArrays seeds,
    CudaHoughMaximumBatchArrays etaMaxima, std::uint32_t group,
    std::uint32_t nAssociatedHits) {
  const std::uint32_t seed = reserveSegmentSeedSlot(seeds, group);
  if (seed >= seeds.capacityPerGroup) {
    return;
  }
  const std::uint32_t output = seeds.index(group, seed);
  seeds.tanBeta[output] = etaMaxima.tanBeta[group];
  seeds.interceptY[output] = etaMaxima.interceptY[group];
  seeds.tanAlpha[output] = 0.0;
  seeds.interceptX[output] = 0.0;
  seeds.counts[output] = etaMaxima.nHits[group];
  seeds.parentBucket[output] = group / etaMaxima.capacityPerBucket;
  seeds.hasPhiExtension[output] = 0u;
  seeds.xBin[output] = 0u;
  seeds.yBin[output] = 0u;
  seeds.nAssociatedHits[output] = nAssociatedHits;
}

__device__ inline void appendPhiSeed(
    CudaMuonSegmentSeedArrays seeds,
    CudaHoughMaximumBatchArrays etaMaxima,
    CudaHoughPlaneBatchArrays plane, const HoughAxisRanges& ranges,
    const std::uint32_t* sharedAssociatedHits, std::uint32_t group,
    const PeakFinders::GlobalMaximumCandidate& candidate,
    std::uint32_t additionalAssociatedHits) {
  if (candidate.nHits < phiThreshold ||
      candidate.localBin == std::numeric_limits<std::uint32_t>::max()) {
    return;
  }
  const std::uint32_t seed = reserveSegmentSeedSlot(seeds, group);
  if (seed >= seeds.capacityPerGroup) {
    return;
  }

  const std::uint32_t xBin = candidate.localBin % plane.nBinsX;
  const std::uint32_t yBin = candidate.localBin / plane.nBinsX;
  const std::uint32_t output = seeds.index(group, seed);
  seeds.tanBeta[output] = etaMaxima.tanBeta[group];
  seeds.interceptY[output] = etaMaxima.interceptY[group];
  seeds.tanAlpha[output] = HoughDetail::binCenterDevice(
      ranges.xMin, ranges.xMax, plane.nBinsX, xBin);
  seeds.interceptX[output] = HoughDetail::binCenterDevice(
      ranges.yMin, ranges.yMax, plane.nBinsY, yBin);
  seeds.counts[output] = candidate.nHits;
  seeds.parentBucket[output] = group / etaMaxima.capacityPerBucket;
  seeds.hasPhiExtension[output] = 1u;
  seeds.xBin[output] = xBin;
  seeds.yBin[output] = yBin;
  seeds.nAssociatedHits[output] =
      sharedAssociatedHits[candidate.localBin] + additionalAssociatedHits;
}

template <PeakFinder peakFinder, typename Candidate>
__device__ inline void findAndAppendPhiSeeds(
    CudaMuonSegmentSeedArrays seeds,
    CudaHoughMaximumBatchArrays etaMaxima,
    CudaHoughPlaneBatchArrays plane, const HoughAxisRanges& ranges,
    const YieldType* sharedHits, const YieldType* sharedLayers,
    const std::uint32_t* sharedAssociatedHits, std::uint8_t* sharedPeakMask,
    Candidate* sharedCandidates, std::uint32_t group,
    std::uint32_t etaOnlyHits) {
  const std::uint32_t nCells = plane.nBinsX * plane.nBinsY;
  PeakFinders::RelativeNmsConfig nmsConfig{};
  nmsConfig.threshold = phiThreshold;
  nmsConfig.fractionCutoff = 0.7f;
  nmsConfig.minimumLayers = 0.0f;
  nmsConfig.minimumAssociatedHits = 2u;

  PeakFinders::SlidingWindowConfig slidingConfig{};
  slidingConfig.threshold = static_cast<std::size_t>(phiThreshold);
  slidingConfig.xWindowSize = PeakFinders::relativeBinRadius(
      plane.nBinsX, nmsConfig.localWindowFraction);
  slidingConfig.yWindowSize = PeakFinders::relativeBinRadius(
      plane.nBinsY, nmsConfig.localWindowFraction);

  if constexpr (peakFinder == PeakFinder::GlobalMaximum) {
    const auto peak = PeakFinders::findGlobalMaximum(
        sharedHits, nCells, sharedCandidates);
    if (threadIdx.x == 0u) {
      appendPhiSeed(seeds, etaMaxima, plane, ranges, sharedAssociatedHits,
                    group, peak, etaOnlyHits);
    }
  } else if constexpr (peakFinder == PeakFinder::SlidingWindow) {
    PeakFinders::findSlidingWindowPeaks(
        sharedHits, plane.nBinsX, plane.nBinsY, slidingConfig,
        sharedPeakMask);
    if (threadIdx.x == 0u) {
      for (std::uint32_t xBin = 0u; xBin < plane.nBinsX; ++xBin) {
        for (std::uint32_t yBin = 0u; yBin < plane.nBinsY; ++yBin) {
          std::uint32_t localBin = yBin * plane.nBinsX + xBin;
          if (sharedPeakMask[localBin] == 0u) {
            continue;
          }
          localBin = PeakFinders::slidingWindowRecenter(
              sharedHits, plane.nBinsX, plane.nBinsY, localBin,
              slidingConfig);
          appendPhiSeed(seeds, etaMaxima, plane, ranges,
                        sharedAssociatedHits, group,
                        {sharedHits[localBin], localBin}, etaOnlyHits);
        }
      }
    }
  } else {
    PeakFinders::markSupportedPeakCells(
        sharedHits, sharedLayers, sharedAssociatedHits, nCells, nmsConfig,
        sharedPeakMask);
    const auto reference = PeakFinders::findBestMarkedPeak(
        sharedHits, sharedLayers, sharedAssociatedHits, sharedPeakMask, nCells,
        sharedCandidates);
    PeakFinders::findRelativeNmsPeaks(
        sharedHits, sharedLayers, sharedAssociatedHits, plane.nBinsX,
        plane.nBinsY, reference.nHits, nmsConfig, sharedPeakMask);

    const std::uint32_t limit =
        seeds.capacityPerGroup < nmsConfig.maximumPeaks
            ? seeds.capacityPerGroup
            : nmsConfig.maximumPeaks;
    for (std::uint32_t seed = 0u; seed < limit; ++seed) {
      const auto peak = PeakFinders::findBestMarkedPeak(
          sharedHits, sharedLayers, sharedAssociatedHits, sharedPeakMask,
          nCells, sharedCandidates);
      if (peak.localBin == std::numeric_limits<std::uint32_t>::max()) {
        break;
      }
      if (threadIdx.x == 0u) {
        appendPhiSeed(seeds, etaMaxima, plane, ranges,
                      sharedAssociatedHits, group,
                      {peak.nHits, peak.localBin}, etaOnlyHits);
      }
      PeakFinders::suppressRelativeNmsNeighbours(
          sharedPeakMask, plane.nBinsX, plane.nBinsY, peak.localBin,
          nmsConfig);
    }
  }
}

template <PeakFinder peakFinder>
__global__ void fillPhiSpacePointsBatchKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays etaMaxima,
    CudaMuonSegmentSeedArrays segmentSeeds, HoughAxisRanges baseRanges,
    YieldType weight) {
  const std::uint32_t nCells = plane.nBinsX * plane.nBinsY;
  extern __shared__ unsigned char sharedMemory[];
  const auto workspace =
      HoughDetail::makeHoughSharedWorkspace<peakFinder>(sharedMemory, nCells);
  __shared__ std::uint32_t sharedEtaOnlyHits;
  __shared__ std::uint32_t sharedPhiHits;
  __shared__ std::uint32_t sharedUnextendedHits;

  for (std::uint32_t group = blockIdx.x; group < plane.nBuckets;
       group += gridDim.x) {
    if (!etaMaximumIsOccupied(etaMaxima, group)) {
      continue;
    }

    const HoughAxisRanges ranges{baseRanges.xMin, baseRanges.xMax,
                                 plane.yMin[group], plane.yMax[group]};
    const PhiInputRanges inputs =
        phiInputRanges(spacePoints, etaMaxima, group);

    for (std::uint32_t localBin = threadIdx.x; localBin < nCells;
         localBin += blockDim.x) {
      workspace.hits[localBin] = YieldType{0.0};
      workspace.layers[localBin] = YieldType{0.0};
      workspace.layerMasks[localBin] = LayerMask{0ull};
      workspace.associatedHits[localBin] = 0u;
    }
    if (threadIdx.x == 0u) {
      sharedEtaOnlyHits = 0u;
      sharedPhiHits = 0u;
      sharedUnextendedHits = 0u;
    }
    __syncthreads();

    for (std::uint32_t hit = inputs.physicalBegin + threadIdx.x;
         hit < inputs.physicalEnd; hit += blockDim.x) {
      if (isPurePhiHit(spacePoints.muonId[hit])) {
        atomicAdd(&sharedPhiHits, 1u);
        atomicAdd(&sharedUnextendedHits, 1u);
      }
    }
    for (std::uint32_t offset = inputs.associatedBegin + threadIdx.x;
         offset < inputs.associatedEnd; offset += blockDim.x) {
      const std::uint32_t hit = etaMaxima.associatedHitIndices[offset];
      const std::uint32_t rawId = spacePoints.muonId[hit];
      atomicAdd(&sharedUnextendedHits, 1u);
      if (!muonIdMeasuresPhi(rawId)) {
        atomicAdd(&sharedEtaOnlyHits, 1u);
      } else if (isEtaAssociatedPhiHit(rawId)) {
        atomicAdd(&sharedPhiHits, 1u);
      }
    }
    __syncthreads();

    if (sharedPhiHits < 2u) {
      if (threadIdx.x == 0u) {
        appendUnextendedEtaSeed(segmentSeeds, etaMaxima, group,
                                sharedUnextendedHits);
      }
      __syncthreads();
      continue;
    }

    const std::uint32_t nPhysical =
        inputs.physicalEnd - inputs.physicalBegin;
    const std::uint32_t nAssociated =
        inputs.associatedEnd - inputs.associatedBegin;
    const std::uint32_t nTasks =
        (nPhysical + nAssociated) * plane.nBinsX;

    for (std::uint32_t task = threadIdx.x; task < nTasks;
         task += blockDim.x) {
      const std::uint32_t xBin = task % plane.nBinsX;
      const std::uint32_t localHit = task / plane.nBinsX;
      std::uint32_t hit = 0u;

      if (localHit < nPhysical) {
        hit = inputs.physicalBegin + localHit;
        if (!isPurePhiHit(spacePoints.muonId[hit])) {
          continue;
        }
      } else {
        const std::uint32_t offset =
            inputs.associatedBegin + localHit - nPhysical;
        hit = etaMaxima.associatedHitIndices[offset];
        if (!isEtaAssociatedPhiHit(spacePoints.muonId[hit])) {
          continue;
        }
      }

      const HoughDetail::YBinRange band = HoughDetail::fillSharedYBand(
          workspace.hits, workspace.layers, workspace.layerMasks, plane,
          ranges, xBin, phiIntercept(plane, spacePoints, ranges, xBin, hit),
          phiStripHalfWidth(spacePoints, hit),
          detLayer(spacePoints.muonId[hit]), weight);
      HoughDetail::countSharedYBand(workspace.associatedHits, plane.nBinsX,
                                    xBin, band);
    }
    __syncthreads();

    findAndAppendPhiSeeds<peakFinder>(
        segmentSeeds, etaMaxima, plane, ranges, workspace.hits,
        workspace.layers, workspace.associatedHits, workspace.peakMask,
        workspace.candidates, group, sharedEtaOnlyHits);
    __syncthreads();
  }
}

__global__ void fillPhiSeedHitIndicesKernel(
    CudaHoughPlaneBatchArrays plane, CudaMuonSpacePointArrays spacePoints,
    CudaHoughMaximumBatchArrays etaMaxima,
    CudaMuonSegmentSeedArrays segmentSeeds, HoughAxisRanges baseRanges) {
  const std::uint32_t totalSlots =
      segmentSeeds.nGroups * segmentSeeds.capacityPerGroup;
  __shared__ std::uint32_t sharedWriteIndex;

  for (std::uint32_t output = blockIdx.x; output < totalSlots;
       output += gridDim.x) {
    const std::uint32_t group = output / segmentSeeds.capacityPerGroup;
    const std::uint32_t maximum = output % segmentSeeds.capacityPerGroup;
    if (maximum >= segmentSeeds.nSeeds[group]) {
      continue;
    }

    const std::uint32_t outputBegin =
        segmentSeeds.associatedHitOffsets[output];
    const std::uint32_t outputEnd =
        segmentSeeds.associatedHitOffsets[output + 1u];
    if (threadIdx.x == 0u) {
      sharedWriteIndex = 0u;
    }
    __syncthreads();

    const PhiInputRanges inputs =
        phiInputRanges(spacePoints, etaMaxima, group);
    const std::uint32_t nPhysical =
        inputs.physicalEnd - inputs.physicalBegin;
    const std::uint32_t nAssociated =
        inputs.associatedEnd - inputs.associatedBegin;
    const std::uint32_t nInputs = nPhysical + nAssociated;

    for (std::uint32_t localHit = threadIdx.x; localHit < nInputs;
         localHit += blockDim.x) {
      std::uint32_t hit = 0u;
      bool include = false;

      if (localHit < nPhysical) {
        hit = inputs.physicalBegin + localHit;
        include = isPurePhiHit(spacePoints.muonId[hit]) &&
                  (segmentSeeds.hasPhiExtension[output] == 0u ||
                   phiHitContributesToMaximum(
                       plane, spacePoints, segmentSeeds, baseRanges, group,
                       maximum, hit));
      } else {
        const std::uint32_t offset =
            inputs.associatedBegin + localHit - nPhysical;
        hit = etaMaxima.associatedHitIndices[offset];
        const std::uint32_t rawId = spacePoints.muonId[hit];
        include = segmentSeeds.hasPhiExtension[output] == 0u ||
                  !muonIdMeasuresPhi(rawId) ||
                  (isEtaAssociatedPhiHit(rawId) &&
                   phiHitContributesToMaximum(
                       plane, spacePoints, segmentSeeds, baseRanges, group,
                       maximum, hit));
      }

      if (!include) {
        continue;
      }
      const std::uint32_t localWrite = atomicAdd(&sharedWriteIndex, 1u);
      const std::uint32_t write = outputBegin + localWrite;
      if (write < outputEnd) {
        segmentSeeds.associatedHitIndices[write] = hit;
      }
    }
    __syncthreads();
  }
}

}  // namespace

namespace ActsExamples::CudaHoughTransformUtils::PhiHoughTransform::detail {

void phiHoughTransformImpl(CudaHoughPlaneBatch& plane,
                           CudaMuonSpacePointContainer& spacePoints,
                           CudaHoughMaximumBatchArrays etaMaxima,
                           CudaMuonSegmentSeedArrays segmentSeeds,
                           const HoughAxisRanges& axisRanges, YieldType weight,
                           std::uint32_t threadsPerBlock,
                           std::uint32_t numBlocks, PeakFinder peakFinder,
                           cudaStream_t stream) {
  const bool limitAuxiliaryGrids = numBlocks != 0u;
  if (threadsPerBlock == 0u) {
    throw std::invalid_argument("threadsPerBlock must be non-zero");
  }

  const std::size_t expectedGroups =
      static_cast<std::size_t>(etaMaxima.nBuckets) *
      etaMaxima.capacityPerBucket;
  if (plane.nBuckets() != expectedGroups ||
      segmentSeeds.nGroups != expectedGroups) {
    throw std::invalid_argument(
        "Phi Hough plane must have one bucket per Eta-maximum slot");
  }
  if (spacePoints.bucketCount() != etaMaxima.nBuckets) {
    throw std::invalid_argument(
        "Eta maxima and space points must have the same physical bucket "
        "count");
  }
  if (etaMaxima.nMaxima == nullptr ||
      etaMaxima.associatedHitOffsets == nullptr ||
      segmentSeeds.nSeeds == nullptr ||
      segmentSeeds.nAssociatedHits == nullptr) {
    throw std::invalid_argument("Invalid segment-seed device storage");
  }
  if (!spacePoints.isOnDevice()) {
    spacePoints.moveToDevice(stream);
  }
  if (!plane.isOnDevice()) {
    plane.moveToDevice(stream);
  }

  ACTS_CUDA_CHECK(cudaMemsetAsync(
      segmentSeeds.nSeeds, 0,
      expectedGroups * sizeof(std::uint32_t), stream));
  const std::size_t outputSlots =
      expectedGroups * segmentSeeds.capacityPerGroup;
  ACTS_CUDA_CHECK(cudaMemsetAsync(
      segmentSeeds.nAssociatedHits, 0,
      outputSlots * sizeof(std::uint32_t), stream));

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
        "Phi Hough transform requires too much shared memory");
  }

  if (numBlocks == 0u) {
    int multiprocessorCount = 0;
    ACTS_CUDA_CHECK(cudaDeviceGetAttribute(
        &multiprocessorCount, cudaDevAttrMultiProcessorCount, device));
    numBlocks = static_cast<std::uint32_t>(multiprocessorCount);
  }
  numBlocks = std::min(numBlocks, static_cast<std::uint32_t>(expectedGroups));
  if (numBlocks == 0u) {
    throw std::runtime_error("Resolved number of CUDA blocks is zero");
  }

  constexpr CoordType interceptMargin = 10.0 * Acts::UnitConstants::cm;
  const std::size_t rangeSharedBytes =
      2u * static_cast<std::size_t>(threadsPerBlock) * sizeof(CoordType);
  const std::uint32_t rangeBlocks =
      limitAuxiliaryGrids ? numBlocks
                          : static_cast<std::uint32_t>(expectedGroups);
  computePhiInterceptRangesBatchKernel<<<
      static_cast<unsigned>(rangeBlocks),
      static_cast<unsigned>(threadsPerBlock), rangeSharedBytes, stream>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), etaMaxima, axisRanges,
      interceptMargin);
  ACTS_CUDA_CHECK(cudaGetLastError());

  if (peakFinder == PeakFinder::GlobalMaximum) {
    fillPhiSpacePointsBatchKernel<PeakFinder::GlobalMaximum>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes, stream>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), etaMaxima,
            segmentSeeds, axisRanges, weight);
  } else if (peakFinder == PeakFinder::SlidingWindow) {
    fillPhiSpacePointsBatchKernel<PeakFinder::SlidingWindow>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes, stream>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), etaMaxima,
            segmentSeeds, axisRanges, weight);
  } else if (peakFinder == PeakFinder::RelativeNms) {
    fillPhiSpacePointsBatchKernel<PeakFinder::RelativeNms>
        <<<static_cast<unsigned>(numBlocks),
           static_cast<unsigned>(threadsPerBlock), sharedBytes, stream>>>(
            plane.deviceArrays(), spacePoints.deviceArrays(), etaMaxima,
            segmentSeeds, axisRanges, weight);
  } else {
    throw std::invalid_argument("Unknown CUDA Hough peak finder");
  }
  ACTS_CUDA_CHECK(cudaGetLastError());
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
}

void fillPhiHitAssociationsImpl(CudaHoughPlaneBatch& plane,
                                CudaMuonSpacePointContainer& spacePoints,
                                CudaHoughMaximumBatchArrays etaMaxima,
                                CudaMuonSegmentSeedArrays segmentSeeds,
                                const HoughAxisRanges& axisRanges,
                                std::uint32_t threadsPerBlock,
                                std::uint32_t numBlocks,
                                cudaStream_t stream) {
  if (threadsPerBlock == 0u) {
    throw std::invalid_argument("threadsPerBlock must be non-zero");
  }
  if (!plane.isOnDevice() || !spacePoints.isOnDevice()) {
    throw std::logic_error(
        "Phi Hough plane and space points must remain on the device");
  }
  if (segmentSeeds.totalAssociatedHits == 0u) {
    return;
  }
  if (segmentSeeds.associatedHitOffsets == nullptr ||
      segmentSeeds.associatedHitIndices == nullptr) {
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

  const std::size_t totalSlots =
      static_cast<std::size_t>(segmentSeeds.nGroups) *
      segmentSeeds.capacityPerGroup;
  if (numBlocks == 0u) {
    numBlocks = static_cast<std::uint32_t>(totalSlots);
  }
  numBlocks =
      std::min(numBlocks, static_cast<std::uint32_t>(totalSlots));

  fillPhiSeedHitIndicesKernel<<<static_cast<unsigned>(numBlocks),
                                static_cast<unsigned>(threadsPerBlock), 0u,
                                stream>>>(
      plane.deviceArrays(), spacePoints.deviceArrays(), etaMaxima,
      segmentSeeds, axisRanges);
  ACTS_CUDA_CHECK(cudaGetLastError());
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
}

}  // namespace ActsExamples::CudaHoughTransformUtils::PhiHoughTransform::detail
