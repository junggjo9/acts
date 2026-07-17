// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Acts/Definitions/Units.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.cuh"
#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"
#include "ActsExamples/Utilities/CudaUtilities.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime.h>

namespace {

using ActsExamples::CudaMuonSpacePointArrays;
using ActsExamples::detLayer;
using ActsExamples::CudaHoughTransformUtils::CoordType;
using ActsExamples::CudaHoughTransformUtils::CudaHoughPlaneBatchArrays;
using ActsExamples::CudaHoughTransformUtils::HoughAxisRanges;
using ActsExamples::CudaHoughTransformUtils::LayerMask;
using ActsExamples::CudaHoughTransformUtils::YieldType;
using ActsExamples::CudaHoughTransformUtils::detail::layerBit;

template <typename T>
void allocateDeviceColumn(T*& deviceColumn, std::size_t size) {
  if (size == 0) {
    return;
  }

  ACTS_CUDA_CHECK(
      cudaMalloc(reinterpret_cast<void**>(&deviceColumn), size * sizeof(T)));
}

template <typename T>
void freeDeviceColumn(T*& deviceColumn) noexcept {
  if (deviceColumn != nullptr) {
    cudaFree(deviceColumn);
    deviceColumn = nullptr;
  }
}

template <typename T>
void copyColumnToDevice(T* deviceColumn, const std::vector<T>& hostColumn) {
  if (hostColumn.empty()) {
    return;
  }

  ACTS_CUDA_CHECK(cudaMemcpy(deviceColumn, hostColumn.data(),
                             hostColumn.size() * sizeof(T),
                             cudaMemcpyHostToDevice));
}

template <typename T>
void copyColumnToHost(std::vector<T>& hostColumn, const T* deviceColumn) {
  if (hostColumn.empty() || deviceColumn == nullptr) {
    return;
  }

  ACTS_CUDA_CHECK(cudaMemcpy(hostColumn.data(), deviceColumn,
                             hostColumn.size() * sizeof(T),
                             cudaMemcpyDeviceToHost));
}

void allocateDeviceData(CudaHoughPlaneBatchArrays& device,
                        std::size_t totalCells, std::size_t nBuckets) {
  allocateDeviceColumn(device.nHits, totalCells);
  allocateDeviceColumn(device.nLayers, totalCells);
  allocateDeviceColumn(device.layerMask, totalCells);

  allocateDeviceColumn(device.yMin, nBuckets);
  allocateDeviceColumn(device.yMax, nBuckets);
}

void freeDeviceData(CudaHoughPlaneBatchArrays& device) noexcept {
  freeDeviceColumn(device.nHits);
  freeDeviceColumn(device.nLayers);
  freeDeviceColumn(device.layerMask);

  freeDeviceColumn(device.yMin);
  freeDeviceColumn(device.yMax);
}

void copyHostToDevice(CudaHoughPlaneBatchArrays& device,
                      const std::vector<YieldType>& hits,
                      const std::vector<YieldType>& layers,
                      const std::vector<LayerMask>& layerMask,
                      const std::vector<CoordType>& yMin,
                      const std::vector<CoordType>& yMax) {
  copyColumnToDevice(device.nHits, hits);
  copyColumnToDevice(device.nLayers, layers);
  copyColumnToDevice(device.layerMask, layerMask);

  copyColumnToDevice(device.yMin, yMin);
  copyColumnToDevice(device.yMax, yMax);
}

void copyDeviceToHost(std::vector<YieldType>& hits,
                      std::vector<YieldType>& layers,
                      std::vector<LayerMask>& layerMask,
                      std::vector<CoordType>& yMin,
                      std::vector<CoordType>& yMax,
                      const CudaHoughPlaneBatchArrays& device) {
  copyColumnToHost(hits, device.nHits);
  copyColumnToHost(layers, device.nLayers);
  copyColumnToHost(layerMask, device.layerMask);

  copyColumnToHost(yMin, device.yMin);
  copyColumnToHost(yMax, device.yMax);
}

}  // namespace

namespace ActsExamples::CudaHoughTransformUtils {

CudaHoughPlaneBatch::CudaHoughPlaneBatch(const HoughPlaneConfig& cfg,
                                         size_type nBuckets)
    : m_cfg{cfg}, m_nBuckets{nBuckets} {
  if (m_cfg.nBinsX == 0 || m_cfg.nBinsY == 0) {
    throw std::invalid_argument(
        "CudaHoughPlaneBatch requires non-zero nBinsX and nBinsY");
  }

  if (m_nBuckets == 0) {
    throw std::invalid_argument(
        "CudaHoughPlaneBatch requires non-zero nBuckets");
  }

  if (m_cfg.nBinsX > std::numeric_limits<std::uint32_t>::max() ||
      m_cfg.nBinsY > std::numeric_limits<std::uint32_t>::max() ||
      m_nBuckets > std::numeric_limits<std::uint32_t>::max()) {
    throw std::overflow_error(
        "CudaHoughPlaneBatch dimensions must fit into std::uint32_t");
  }

  m_hostHits.resize(totalCells(), 0.0f);
  m_hostLayers.resize(totalCells(), 0.0f);
  m_hostLayerMask.resize(totalCells(), LayerMask{0ull});

  m_hostYMin.resize(nBuckets, 0.0);
  m_hostYMax.resize(nBuckets, 0.0);
}

CudaHoughPlaneBatch::CudaHoughPlaneBatch(CudaHoughPlaneBatch&& other) noexcept
    : m_cfg{other.m_cfg},
      m_nBuckets{other.m_nBuckets},
      m_hostHits{std::move(other.m_hostHits)},
      m_hostLayers{std::move(other.m_hostLayers)},
      m_hostLayerMask{std::move(other.m_hostLayerMask)},
      m_hostYMin{std::move(other.m_hostYMin)},
      m_hostYMax{std::move(other.m_hostYMax)},
      m_device{std::exchange(other.m_device, {})},
      m_onDevice{std::exchange(other.m_onDevice, false)} {
  other.m_cfg = {};
  other.m_nBuckets = 0;
}

CudaHoughPlaneBatch& CudaHoughPlaneBatch::operator=(
    CudaHoughPlaneBatch&& other) noexcept {
  if (this != &other) {
    clearDevice();

    m_cfg = other.m_cfg;
    m_nBuckets = other.m_nBuckets;
    m_hostHits = std::move(other.m_hostHits);
    m_hostLayers = std::move(other.m_hostLayers);
    m_hostLayerMask = std::move(other.m_hostLayerMask);
    m_hostYMin = std::move(other.m_hostYMin);
    m_hostYMax = std::move(other.m_hostYMax);
    m_device = std::exchange(other.m_device, {});
    m_onDevice = std::exchange(other.m_onDevice, false);
    other.m_cfg = {};
    other.m_nBuckets = 0;
  }

  return *this;
}

CudaHoughPlaneBatch::~CudaHoughPlaneBatch() noexcept {
  clearDevice();
}

CudaHoughPlaneBatch::size_type CudaHoughPlaneBatch::globalBin(
    size_type bucket, size_type xBin, size_type yBin) const {
  checkBucket(bucket);
  checkIndices(xBin, yBin);
  return uncheckedGlobalBin(bucket, xBin, yBin);
}

void CudaHoughPlaneBatch::fillBin(size_type bucket, size_type xBin,
                                  size_type yBin, unsigned layer,
                                  YieldType weight) {
  checkBucket(bucket);
  checkIndices(xBin, yBin);

  if (weight == 0.0f) {
    return;
  }

  const size_type bin = uncheckedGlobalBin(bucket, xBin, yBin);

  m_hostHits[bin] += weight;

  const LayerMask bit = layerBit(layer);

  if ((m_hostLayerMask[bin] & bit) == LayerMask{0ull}) {
    m_hostLayerMask[bin] |= bit;
    m_hostLayers[bin] += weight;
  }
}

std::pair<std::size_t, std::size_t> CudaHoughPlaneBatch::axisBins(
    size_type globalBin) const {
  const size_type localBin = globalBin % nCellsPerBucket();
  return {localBin % nBinsX(), localBin / nBinsX()};
}

YieldType CudaHoughPlaneBatch::nHits(size_type bucket, size_type xBin,
                                     size_type yBin) const {
  return m_hostHits[globalBin(bucket, xBin, yBin)];
}

YieldType CudaHoughPlaneBatch::nLayers(size_type bucket, size_type xBin,
                                       size_type yBin) const {
  return m_hostLayers[globalBin(bucket, xBin, yBin)];
}

LayerMask CudaHoughPlaneBatch::layerMask(size_type bucket, size_type xBin,
                                         size_type yBin) const {
  return m_hostLayerMask[globalBin(bucket, xBin, yBin)];
}

bool CudaHoughPlaneBatch::hasLayer(size_type bucket, size_type xBin,
                                   size_type yBin, unsigned layer) const {
  if (layer >= 8u * sizeof(LayerMask)) {
    return false;
  }

  const LayerMask bit = LayerMask{1ull} << layer;
  return (layerMask(bucket, xBin, yBin) & bit) != LayerMask{0ull};
}

std::vector<unsigned> CudaHoughPlaneBatch::layers(size_type bucket,
                                                  size_type xBin,
                                                  size_type yBin) const {
  const LayerMask mask = layerMask(bucket, xBin, yBin);

  std::vector<unsigned> out{};

  for (unsigned layer = 0; layer < 8u * sizeof(LayerMask); ++layer) {
    const LayerMask bit = LayerMask{1ull} << layer;

    if ((mask & bit) != LayerMask{0ull}) {
      out.push_back(layer);
    }
  }

  return out;
}

YieldType CudaHoughPlaneBatch::maxHits(size_type bucket) const {
  checkBucket(bucket);

  const size_type bucket_offset = bucket * nCellsPerBucket();
  YieldType max_hits = 0;

  for (size_type local_bin = 0; local_bin < nCellsPerBucket(); ++local_bin) {
    max_hits = std::max(max_hits, m_hostHits[bucket_offset + local_bin]);
  }

  return max_hits;
}

YieldType CudaHoughPlaneBatch::maxLayers(size_type bucket) const {
  checkBucket(bucket);

  const size_type bucket_offset = bucket * nCellsPerBucket();
  YieldType max_layers = 0;

  for (size_type local_bin = 0; local_bin < nCellsPerBucket(); ++local_bin) {
    max_layers = std::max(max_layers, m_hostLayers[bucket_offset + local_bin]);
  }

  return max_layers;
}

std::pair<std::size_t, std::size_t> CudaHoughPlaneBatch::locMaxHits(
    size_type bucket) const {
  checkBucket(bucket);

  const size_type bucket_offset = bucket * nCellsPerBucket();

  YieldType max_hits = 0;
  size_type max_local_bin = 0;

  for (size_type local_bin = 0; local_bin < nCellsPerBucket(); ++local_bin) {
    const YieldType hits = m_hostHits[bucket_offset + local_bin];

    if (hits > max_hits) {
      max_hits = hits;
      max_local_bin = local_bin;
    }
  }

  const size_type x_bin = max_local_bin % nBinsX();
  const size_type y_bin = max_local_bin / nBinsX();

  return {x_bin, y_bin};
}

std::pair<std::size_t, std::size_t> CudaHoughPlaneBatch::locMaxLayers(
    size_type bucket) const {
  checkBucket(bucket);

  const size_type bucketOffset = bucket * nCellsPerBucket();

  size_type maximumLocalBin = 0u;

  for (size_type localBin = 1u; localBin < nCellsPerBucket(); ++localBin) {
    if (m_hostLayers[bucketOffset + localBin] >
        m_hostLayers[bucketOffset + maximumLocalBin]) {
      maximumLocalBin = localBin;
    }
  }

  return {maximumLocalBin % nBinsX(), maximumLocalBin / nBinsX()};
}

void CudaHoughPlaneBatch::moveToDevice() {
  clearDevice();

  m_device.nBuckets = static_cast<std::uint32_t>(nBuckets());
  m_device.nBinsX = static_cast<std::uint32_t>(nBinsX());
  m_device.nBinsY = static_cast<std::uint32_t>(nBinsY());

  allocateDeviceData(m_device, totalCells(), nBuckets());
  copyHostToDevice(m_device, m_hostHits, m_hostLayers, m_hostLayerMask,
                   m_hostYMin, m_hostYMax);

  m_onDevice = true;
}

void CudaHoughPlaneBatch::moveToHost() {
  if (!m_onDevice) {
    return;
  }

  copyDeviceToHost(m_hostHits, m_hostLayers, m_hostLayerMask, m_hostYMin,
                   m_hostYMax, m_device);
}

void CudaHoughPlaneBatch::clearDevice() noexcept {
  freeDeviceData(m_device);
  m_device = {};
  m_onDevice = false;
}

void CudaHoughPlaneBatch::checkBucket(size_type bucket) const {
  if (bucket >= nBuckets()) {
    throw std::out_of_range("CudaHoughPlaneBatch bucket index out of range");
  }
}

void CudaHoughPlaneBatch::checkIndices(size_type xBin, size_type yBin) const {
  if (xBin >= nBinsX()) {
    throw std::out_of_range("CudaHoughPlaneBatch x-bin index out of range");
  }

  if (yBin >= nBinsY()) {
    throw std::out_of_range("CudaHoughPlaneBatch y-bin index out of range");
  }
}

CoordType CudaHoughPlaneBatch::yMin(size_type bucket) const {
  checkBucket(bucket);
  return m_hostYMin[bucket];
}

CoordType CudaHoughPlaneBatch::yMax(size_type bucket) const {
  checkBucket(bucket);
  return m_hostYMax[bucket];
}

HoughAxisRanges CudaHoughPlaneBatch::bucketAxisRanges(
    size_type bucket, const HoughAxisRanges& baseRanges) const {
  checkBucket(bucket);
  return HoughAxisRanges{baseRanges.xMin, baseRanges.xMax, m_hostYMin[bucket],
                         m_hostYMax[bucket]};
}

}  // namespace ActsExamples::CudaHoughTransformUtils
