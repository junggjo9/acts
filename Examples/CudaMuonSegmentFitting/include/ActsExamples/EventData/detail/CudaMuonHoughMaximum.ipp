// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include <cuda_runtime_api.h>

namespace ActsExamples::detail {

inline void cudaHoughMaximumCheck(cudaError_t status) {
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

template <typename T>
void allocateHoughMaximumColumn(T*& deviceColumn, std::size_t size) {
  if (size == 0u) {
    return;
  }

  cudaHoughMaximumCheck(
      cudaMalloc(reinterpret_cast<void**>(&deviceColumn), size * sizeof(T)));
}

template <typename T>
void freeHoughMaximumColumn(T*& deviceColumn) noexcept {
  if (deviceColumn != nullptr) {
    cudaFree(deviceColumn);
    deviceColumn = nullptr;
  }
}

template <typename T>
void copyHoughMaximumColumnToDevice(T* deviceColumn,
                                    const std::vector<T>& hostColumn) {
  if (hostColumn.empty()) {
    return;
  }

  cudaHoughMaximumCheck(cudaMemcpy(deviceColumn, hostColumn.data(),
                                   hostColumn.size() * sizeof(T),
                                   cudaMemcpyHostToDevice));
}

template <typename T>
void copyHoughMaximumColumnToHost(std::vector<T>& hostColumn,
                                  const T* deviceColumn) {
  if (hostColumn.empty() || deviceColumn == nullptr) {
    return;
  }

  cudaHoughMaximumCheck(cudaMemcpy(hostColumn.data(), deviceColumn,
                                   hostColumn.size() * sizeof(T),
                                   cudaMemcpyDeviceToHost));
}

inline void allocateHoughMaximumDeviceData(CudaHoughMaximumBatchArrays& device,
                                           std::size_t totalCapacity,
                                           std::size_t nBuckets) {
  allocateHoughMaximumColumn(device.tanBeta, totalCapacity);
  allocateHoughMaximumColumn(device.interceptY, totalCapacity);

  allocateHoughMaximumColumn(device.nHits, totalCapacity);
  allocateHoughMaximumColumn(device.nLayers, totalCapacity);
  allocateHoughMaximumColumn(device.layerMask, totalCapacity);

  allocateHoughMaximumColumn(device.xBin, totalCapacity);
  allocateHoughMaximumColumn(device.yBin, totalCapacity);

  allocateHoughMaximumColumn(device.nMaxima, nBuckets);
}

inline void freeHoughMaximumDeviceData(
    CudaHoughMaximumBatchArrays& device) noexcept {
  freeHoughMaximumColumn(device.tanBeta);
  freeHoughMaximumColumn(device.interceptY);

  freeHoughMaximumColumn(device.nHits);
  freeHoughMaximumColumn(device.nLayers);
  freeHoughMaximumColumn(device.layerMask);

  freeHoughMaximumColumn(device.xBin);
  freeHoughMaximumColumn(device.yBin);

  freeHoughMaximumColumn(device.nMaxima);
}

}  // namespace ActsExamples::detail

namespace ActsExamples {

template <std::size_t MaximaPerBucket>
CudaHoughMaximumBatch<MaximaPerBucket>::CudaHoughMaximumBatch(
    size_type nBuckets)
    : m_nBuckets{nBuckets} {
  if (m_nBuckets == 0u) {
    throw std::invalid_argument(
        "CudaHoughMaximumBatch requires non-zero nBuckets");
  }

  constexpr size_type maxUint32 =
      static_cast<size_type>(std::numeric_limits<std::uint32_t>::max());

  if (m_nBuckets > maxUint32) {
    throw std::overflow_error(
        "CudaHoughMaximumBatch nBuckets must fit into std::uint32_t");
  }

  if (m_nBuckets > std::numeric_limits<size_type>::max() / MaximaPerBucket) {
    throw std::overflow_error(
        "CudaHoughMaximumBatch total capacity overflows std::size_t");
  }

  const size_type capacity = totalCapacity();

  // CudaHoughMaximumBatchArrays::index returns std::uint32_t.
  if (capacity > maxUint32) {
    throw std::overflow_error(
        "CudaHoughMaximumBatch total capacity must fit into std::uint32_t");
  }

  m_hostTanBeta.resize(capacity, CoordType{0.});
  m_hostInterceptY.resize(capacity, CoordType{0.});

  m_hostHits.resize(capacity, YieldType{0.});
  m_hostLayers.resize(capacity, YieldType{0.});
  m_hostLayerMask.resize(capacity, LayerMask{0ull});

  m_hostXBin.resize(capacity, 0u);
  m_hostYBin.resize(capacity, 0u);

  m_hostNMaxima.resize(m_nBuckets, 0u);
}

template <std::size_t MaximaPerBucket>
CudaHoughMaximumBatch<MaximaPerBucket>::CudaHoughMaximumBatch(
    CudaHoughMaximumBatch&& other) noexcept
    : m_nBuckets{std::exchange(other.m_nBuckets, 0u)},
      m_hostTanBeta{std::move(other.m_hostTanBeta)},
      m_hostInterceptY{std::move(other.m_hostInterceptY)},
      m_hostHits{std::move(other.m_hostHits)},
      m_hostLayers{std::move(other.m_hostLayers)},
      m_hostLayerMask{std::move(other.m_hostLayerMask)},
      m_hostXBin{std::move(other.m_hostXBin)},
      m_hostYBin{std::move(other.m_hostYBin)},
      m_hostNMaxima{std::move(other.m_hostNMaxima)},
      m_device{std::exchange(other.m_device, CudaHoughMaximumBatchArrays{})},
      m_onDevice{std::exchange(other.m_onDevice, false)} {}

template <std::size_t MaximaPerBucket>
CudaHoughMaximumBatch<MaximaPerBucket>&
CudaHoughMaximumBatch<MaximaPerBucket>::operator=(
    CudaHoughMaximumBatch&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  clearDevice();

  m_nBuckets = std::exchange(other.m_nBuckets, 0u);

  m_hostTanBeta = std::move(other.m_hostTanBeta);
  m_hostInterceptY = std::move(other.m_hostInterceptY);

  m_hostHits = std::move(other.m_hostHits);
  m_hostLayers = std::move(other.m_hostLayers);
  m_hostLayerMask = std::move(other.m_hostLayerMask);

  m_hostXBin = std::move(other.m_hostXBin);
  m_hostYBin = std::move(other.m_hostYBin);

  m_hostNMaxima = std::move(other.m_hostNMaxima);

  m_device = std::exchange(other.m_device, CudaHoughMaximumBatchArrays{});
  m_onDevice = std::exchange(other.m_onDevice, false);

  return *this;
}

template <std::size_t MaximaPerBucket>
CudaHoughMaximumBatch<MaximaPerBucket>::~CudaHoughMaximumBatch() noexcept {
  clearDevice();
}

template <std::size_t MaximaPerBucket>
typename CudaHoughMaximumBatch<MaximaPerBucket>::size_type
CudaHoughMaximumBatch<MaximaPerBucket>::nMaxima(size_type bucket) const {
  checkBucket(bucket);

  const size_type count = static_cast<size_type>(m_hostNMaxima[bucket]);

  if (count > capacityPerBucket()) {
    throw std::runtime_error(
        "CudaHoughMaximumBatch contains an invalid maximum count");
  }

  return count;
}

template <std::size_t MaximaPerBucket>
CoordType CudaHoughMaximumBatch<MaximaPerBucket>::tanBeta(
    size_type bucket, size_type maximum) const {
  checkMaximum(bucket, maximum);
  return m_hostTanBeta[slotIndex(bucket, maximum)];
}

template <std::size_t MaximaPerBucket>
CoordType CudaHoughMaximumBatch<MaximaPerBucket>::interceptY(
    size_type bucket, size_type maximum) const {
  checkMaximum(bucket, maximum);
  return m_hostInterceptY[slotIndex(bucket, maximum)];
}

template <std::size_t MaximaPerBucket>
YieldType CudaHoughMaximumBatch<MaximaPerBucket>::nHits(
    size_type bucket, size_type maximum) const {
  checkMaximum(bucket, maximum);
  return m_hostHits[slotIndex(bucket, maximum)];
}

template <std::size_t MaximaPerBucket>
YieldType CudaHoughMaximumBatch<MaximaPerBucket>::nLayers(
    size_type bucket, size_type maximum) const {
  checkMaximum(bucket, maximum);
  return m_hostLayers[slotIndex(bucket, maximum)];
}

template <std::size_t MaximaPerBucket>
LayerMask CudaHoughMaximumBatch<MaximaPerBucket>::layerMask(
    size_type bucket, size_type maximum) const {
  checkMaximum(bucket, maximum);
  return m_hostLayerMask[slotIndex(bucket, maximum)];
}

template <std::size_t MaximaPerBucket>
typename CudaHoughMaximumBatch<MaximaPerBucket>::size_type
CudaHoughMaximumBatch<MaximaPerBucket>::xBin(size_type bucket,
                                             size_type maximum) const {
  checkMaximum(bucket, maximum);
  return static_cast<size_type>(m_hostXBin[slotIndex(bucket, maximum)]);
}

template <std::size_t MaximaPerBucket>
typename CudaHoughMaximumBatch<MaximaPerBucket>::size_type
CudaHoughMaximumBatch<MaximaPerBucket>::yBin(size_type bucket,
                                             size_type maximum) const {
  checkMaximum(bucket, maximum);
  return static_cast<size_type>(m_hostYBin[slotIndex(bucket, maximum)]);
}

template <std::size_t MaximaPerBucket>
void CudaHoughMaximumBatch<MaximaPerBucket>::moveToDevice() {
  clearDevice();

  m_device.nBuckets = static_cast<std::uint32_t>(nBuckets());

  m_device.capacityPerBucket = static_cast<std::uint32_t>(capacityPerBucket());

  try {
    detail::allocateHoughMaximumDeviceData(m_device, totalCapacity(),
                                           nBuckets());

    detail::copyHoughMaximumColumnToDevice(m_device.tanBeta, m_hostTanBeta);
    detail::copyHoughMaximumColumnToDevice(m_device.interceptY,
                                           m_hostInterceptY);

    detail::copyHoughMaximumColumnToDevice(m_device.nHits, m_hostHits);
    detail::copyHoughMaximumColumnToDevice(m_device.nLayers, m_hostLayers);
    detail::copyHoughMaximumColumnToDevice(m_device.layerMask, m_hostLayerMask);

    detail::copyHoughMaximumColumnToDevice(m_device.xBin, m_hostXBin);
    detail::copyHoughMaximumColumnToDevice(m_device.yBin, m_hostYBin);

    detail::copyHoughMaximumColumnToDevice(m_device.nMaxima, m_hostNMaxima);
  } catch (...) {
    clearDevice();
    throw;
  }

  m_onDevice = true;
}

template <std::size_t MaximaPerBucket>
void CudaHoughMaximumBatch<MaximaPerBucket>::moveToHost() {
  if (!m_onDevice) {
    return;
  }

  detail::copyHoughMaximumColumnToHost(m_hostTanBeta, m_device.tanBeta);
  detail::copyHoughMaximumColumnToHost(m_hostInterceptY, m_device.interceptY);

  detail::copyHoughMaximumColumnToHost(m_hostHits, m_device.nHits);
  detail::copyHoughMaximumColumnToHost(m_hostLayers, m_device.nLayers);
  detail::copyHoughMaximumColumnToHost(m_hostLayerMask, m_device.layerMask);

  detail::copyHoughMaximumColumnToHost(m_hostXBin, m_device.xBin);
  detail::copyHoughMaximumColumnToHost(m_hostYBin, m_device.yBin);

  detail::copyHoughMaximumColumnToHost(m_hostNMaxima, m_device.nMaxima);
}

template <std::size_t MaximaPerBucket>
void CudaHoughMaximumBatch<MaximaPerBucket>::resetOnDevice() {
  if (!m_onDevice) {
    throw std::logic_error("CudaHoughMaximumBatch is not on the device");
  }

  std::fill(m_hostNMaxima.begin(), m_hostNMaxima.end(), 0u);

  detail::cudaHoughMaximumCheck(
      cudaMemset(m_device.nMaxima, 0, nBuckets() * sizeof(std::uint32_t)));
}

template <std::size_t MaximaPerBucket>
void CudaHoughMaximumBatch<MaximaPerBucket>::clearDevice() noexcept {
  detail::freeHoughMaximumDeviceData(m_device);

  m_device = {};
  m_onDevice = false;
}

template <std::size_t MaximaPerBucket>
void CudaHoughMaximumBatch<MaximaPerBucket>::checkBucket(
    size_type bucket) const {
  if (bucket >= nBuckets()) {
    throw std::out_of_range("CudaHoughMaximumBatch bucket index out of range");
  }
}

template <std::size_t MaximaPerBucket>
void CudaHoughMaximumBatch<MaximaPerBucket>::checkMaximum(
    size_type bucket, size_type maximum) const {
  const size_type count = nMaxima(bucket);

  if (maximum >= count) {
    throw std::out_of_range("CudaHoughMaximumBatch maximum index out of range");
  }
}

}  // namespace ActsExamples
