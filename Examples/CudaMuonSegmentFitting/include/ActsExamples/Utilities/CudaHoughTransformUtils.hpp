// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/Seeding/HoughTransformUtils.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

#include <cuda_runtime.h>

namespace ActsExamples::CudaHoughTransformUtils {

using YieldType = Acts::HoughTransformUtils::YieldType;
using CoordType = Acts::HoughTransformUtils::CoordType;
using HoughPlaneConfig = Acts::HoughTransformUtils::HoughPlaneConfig;
using HoughAxisRanges = Acts::HoughTransformUtils::HoughAxisRanges;

/// Peak-finding algorithm used after filling a CUDA Hough plane.
enum class PeakFinder : std::uint8_t {
  GlobalMaximum,
  SlidingWindow,
  RelativeNms,
};

/// @brief Bit mask encoding the logical detector layers contributing to a
/// Hough cell.
///
/// One bit corresponds to one zero-based logical layer:
///
///   layer 0 -> bit 0 -> 00000001
///   layer 1 -> bit 1 -> 00000010
using LayerMask = unsigned long long;

/// @brief Non-owning device-side metadata for a batch of Hough planes.
///
/// Per-cell accumulators are transient shared memory owned by the fill kernel.
/// Only the dynamic Y-axis range of each bucket persists in global memory.
struct CudaHoughPlaneBatchArrays {
  /// Dynamic y-intercept range for every bucket.
  CoordType* yMin = nullptr;
  CoordType* yMax = nullptr;

  /// Dimensions use the X/Y convention of HoughTransformUtils. For the Eta
  /// transform they represent tanBeta and y-intercept respectively.
  std::uint32_t nBuckets = 0;
  std::uint32_t nBinsX = 0;
  std::uint32_t nBinsY = 0;
};

/// @brief Event-level CUDA Hough-plane metadata.
///
/// The full accumulator is built and reduced in shared memory. This object owns
/// only the per-bucket dynamic Y-axis ranges required by the fill and later hit
/// association kernels.
class CudaHoughPlaneBatch {
 public:
  using size_type = std::size_t;

  CudaHoughPlaneBatch(const HoughPlaneConfig& cfg, size_type nBuckets);

  /// Device memory is owned, so instances are not copyable.
  CudaHoughPlaneBatch(const CudaHoughPlaneBatch&) = delete;
  CudaHoughPlaneBatch& operator=(const CudaHoughPlaneBatch&) = delete;

  /// Transfer ownership without copying device memory.
  CudaHoughPlaneBatch(CudaHoughPlaneBatch&& other) noexcept;
  CudaHoughPlaneBatch& operator=(CudaHoughPlaneBatch&& other) noexcept;

  ~CudaHoughPlaneBatch() noexcept;

  size_type nBuckets() const noexcept { return m_nBuckets; }
  size_type nBinsX() const noexcept { return m_cfg.nBinsX; }
  size_type nBinsY() const noexcept { return m_cfg.nBinsY; }
  size_type nCellsPerBucket() const noexcept { return nBinsX() * nBinsY(); }
  bool empty() const noexcept { return nBuckets() == 0; }

  /// @brief Allocate device axis-range arrays for use on a CUDA stream.
  void moveToDevice(cudaStream_t stream);
  /// @brief Copy dynamic axis ranges on a CUDA stream.
  void moveToHost(cudaStream_t stream);
  /// @brief Free device axis-range arrays.
  void clearDevice() noexcept;

  bool isOnDevice() const noexcept { return m_onDevice; }

  CudaHoughPlaneBatchArrays deviceArrays() const noexcept { return m_device; }

  CoordType yMin(size_type bucket) const;
  CoordType yMax(size_type bucket) const;

  HoughAxisRanges bucketAxisRanges(size_type bucket,
                                   const HoughAxisRanges& baseRanges) const;

 private:
  HoughPlaneConfig m_cfg{};
  size_type m_nBuckets = 0;

  std::vector<CoordType> m_hostYMin{};
  std::vector<CoordType> m_hostYMax{};

  CudaHoughPlaneBatchArrays m_device{};
  bool m_onDevice = false;

  void checkBucket(size_type bucket) const;
};

}  // namespace ActsExamples::CudaHoughTransformUtils
