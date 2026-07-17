// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ActsExamples/Utilities/CudaHoughTransformUtils.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <cuda_runtime.h>

namespace ActsExamples {

using YieldType = Acts::HoughTransformUtils::YieldType;
using CoordType = Acts::HoughTransformUtils::CoordType;
using LayerMask = unsigned long long;

/// @brief Non-owning device-side view of a batch of Hough maxima.
///
/// Maximum slots are stored bucket-by-bucket:
///
///   bucket 0: maximum 0, maximum 1, ..., maximum N - 1
///   bucket 1: maximum 0, maximum 1, ..., maximum N - 1
struct CudaHoughMaximumBatchArrays {
  CoordType* tanBeta = nullptr;
  CoordType* interceptY = nullptr;

  YieldType* nHits = nullptr;
  YieldType* nLayers = nullptr;
  LayerMask* layerMask = nullptr;

  std::uint32_t* xBin = nullptr;
  std::uint32_t* yBin = nullptr;

  /// Number of occupied maximum slots in each bucket.
  std::uint32_t* nMaxima = nullptr;

  std::uint32_t nBuckets = 0;
  std::uint32_t capacityPerBucket = 0;

  /// Return the flat array index for a bucket and maximum slot.
  __host__ __device__ std::uint32_t index(
      std::uint32_t bucket, std::uint32_t maximum) const noexcept {
    return bucket * capacityPerBucket + maximum;
  }
};

/// @brief Owning host/device storage for a fixed number of Hough-maximum slots
/// per bucket.
///
/// MaximaPerBucket specifies the maximum number of maxima that may be stored
/// for each bucket. The actual number stored is tracked independently by the
/// nMaxima counter for each bucket.
template <std::size_t MaximaPerBucket>
class CudaHoughMaximumBatch {
 public:
  static_assert(MaximaPerBucket > 0,
                "MaximaPerBucket must be greater than zero");

  static_assert(MaximaPerBucket <= std::numeric_limits<std::uint32_t>::max(),
                "MaximaPerBucket must fit into std::uint32_t");

  using size_type = std::size_t;

  explicit CudaHoughMaximumBatch(size_type nBuckets);

  CudaHoughMaximumBatch(const CudaHoughMaximumBatch&) = delete;
  CudaHoughMaximumBatch& operator=(const CudaHoughMaximumBatch&) = delete;

  CudaHoughMaximumBatch(CudaHoughMaximumBatch&& other) noexcept;

  CudaHoughMaximumBatch& operator=(CudaHoughMaximumBatch&& other) noexcept;

  ~CudaHoughMaximumBatch() noexcept;

  static constexpr size_type capacityPerBucket() noexcept {
    return MaximaPerBucket;
  }

  size_type nBuckets() const noexcept { return m_nBuckets; }

  size_type totalCapacity() const noexcept {
    return nBuckets() * capacityPerBucket();
  }

  bool empty() const noexcept { return nBuckets() == 0; }

  /// Return the number of stored maxima in one bucket.
  size_type nMaxima(size_type bucket) const;

  CoordType tanBeta(size_type bucket, size_type maximum) const;
  CoordType interceptY(size_type bucket, size_type maximum) const;

  YieldType nHits(size_type bucket, size_type maximum) const;
  YieldType nLayers(size_type bucket, size_type maximum) const;

  LayerMask layerMask(size_type bucket, size_type maximum) const;

  size_type xBin(size_type bucket, size_type maximum) const;
  size_type yBin(size_type bucket, size_type maximum) const;

  /// Allocate device storage and copy host data to it.
  void moveToDevice();

  /// Copy device data back into host storage.
  void moveToHost();

  /// Reset the per-bucket maximum counters on the device.
  ///
  /// The maximum slots themselves are not cleared because entries beyond
  /// nMaxima(bucket) are ignored.
  void resetOnDevice();

  /// Release all owned device storage.
  void clearDevice() noexcept;

  bool isOnDevice() const noexcept { return m_onDevice; }

  CudaHoughMaximumBatchArrays deviceArrays() const noexcept { return m_device; }

 private:
  size_type m_nBuckets = 0;

  std::vector<CoordType> m_hostTanBeta{};
  std::vector<CoordType> m_hostInterceptY{};

  std::vector<YieldType> m_hostHits{};
  std::vector<YieldType> m_hostLayers{};
  std::vector<LayerMask> m_hostLayerMask{};

  std::vector<std::uint32_t> m_hostXBin{};
  std::vector<std::uint32_t> m_hostYBin{};

  /// One counter per bucket, not one counter per maximum slot.
  std::vector<std::uint32_t> m_hostNMaxima{};

  CudaHoughMaximumBatchArrays m_device{};
  bool m_onDevice = false;

  size_type slotIndex(size_type bucket, size_type maximum) const noexcept {
    return bucket * capacityPerBucket() + maximum;
  }

  void checkBucket(size_type bucket) const;
  void checkMaximum(size_type bucket, size_type maximum) const;
};

}  // namespace ActsExamples

#include "ActsExamples/EventData/detail/CudaMuonHoughMaximum.ipp"
