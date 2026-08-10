// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"
#include "ActsExamples/Utilities/CudaUtilities.hpp"

#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cuda_runtime.h>

namespace {

using namespace ActsExamples;
using ActsExamples::CudaHoughTransformUtils::CoordType;
using ActsExamples::CudaHoughTransformUtils::CudaHoughPlaneBatchArrays;
using ActsExamples::CudaHoughTransformUtils::HoughAxisRanges;

void allocateDeviceData(CudaHoughPlaneBatchArrays& device,
                        std::size_t nBuckets) {
  allocateDeviceColumn(device.yMin, nBuckets);
  allocateDeviceColumn(device.yMax, nBuckets);
}

void freeDeviceData(CudaHoughPlaneBatchArrays& device) noexcept {
  freeDeviceColumn(device.yMin);
  freeDeviceColumn(device.yMax);
}

void copyHostToDevice(CudaHoughPlaneBatchArrays& device,
                      const std::vector<CoordType>& yMin,
                      const std::vector<CoordType>& yMax,
                      cudaStream_t stream) {
  copyColumnToDevice(device.yMin, yMin, stream);
  copyColumnToDevice(device.yMax, yMax, stream);
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
}

void copyDeviceToHost(std::vector<CoordType>& yMin,
                      std::vector<CoordType>& yMax,
                      const CudaHoughPlaneBatchArrays& device,
                      cudaStream_t stream) {
  copyColumnToHost(yMin, device.yMin, stream);
  copyColumnToHost(yMax, device.yMax, stream);
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
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

  m_hostYMin.resize(nBuckets, 0.0);
  m_hostYMax.resize(nBuckets, 0.0);
}

CudaHoughPlaneBatch::CudaHoughPlaneBatch(CudaHoughPlaneBatch&& other) noexcept
    : m_cfg{other.m_cfg},
      m_nBuckets{other.m_nBuckets},
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

void CudaHoughPlaneBatch::moveToDevice() {
  clearDevice();

  m_device.nBuckets = static_cast<std::uint32_t>(nBuckets());
  m_device.nBinsX = static_cast<std::uint32_t>(nBinsX());
  m_device.nBinsY = static_cast<std::uint32_t>(nBinsY());

  allocateDeviceData(m_device, nBuckets());
  copyColumnToDevice(m_device.yMin, m_hostYMin);
  copyColumnToDevice(m_device.yMax, m_hostYMax);

  m_onDevice = true;
}

void CudaHoughPlaneBatch::moveToDevice(cudaStream_t stream) {
  clearDevice();

  m_device.nBuckets = static_cast<std::uint32_t>(nBuckets());
  m_device.nBinsX = static_cast<std::uint32_t>(nBinsX());
  m_device.nBinsY = static_cast<std::uint32_t>(nBinsY());

  allocateDeviceData(m_device, nBuckets());
  copyHostToDevice(m_device, m_hostYMin, m_hostYMax, stream);

  m_onDevice = true;
}

void CudaHoughPlaneBatch::moveToHost() {
  if (!m_onDevice) {
    return;
  }

  copyColumnToHost(m_hostYMin, m_device.yMin);
  copyColumnToHost(m_hostYMax, m_device.yMax);
}

void CudaHoughPlaneBatch::moveToHost(cudaStream_t stream) {
  if (!m_onDevice) {
    return;
  }

  copyDeviceToHost(m_hostYMin, m_hostYMax, m_device, stream);
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
