// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/Definitions/Algebra.hpp"
#include "Acts/Utilities/detail/ContainerIterator.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include <cuda_runtime.h>

namespace ActsExamples {

using SegmentSeedCoordType = double;
using SegmentSeedCountType = double;

/// Non-owning device view of a grouped segment-seed SoA.
struct CudaMuonSegmentSeedArrays {
  SegmentSeedCoordType* tanBeta = nullptr;
  SegmentSeedCoordType* interceptY = nullptr;
  SegmentSeedCoordType* tanAlpha = nullptr;
  SegmentSeedCoordType* interceptX = nullptr;
  SegmentSeedCountType* counts = nullptr;

  std::uint32_t* parentBucket = nullptr;
  std::uint8_t* hasPhiExtension = nullptr;
  std::uint32_t* xBin = nullptr;
  std::uint32_t* yBin = nullptr;

  std::uint32_t* nSeeds = nullptr;
  std::uint32_t* nAssociatedHits = nullptr;
  std::uint32_t* associatedHitOffsets = nullptr;
  std::uint32_t* associatedHitIndices = nullptr;

  std::uint32_t nGroups = 0u;
  std::uint32_t capacityPerGroup = 0u;
  std::uint32_t totalAssociatedHits = 0u;

  __host__ __device__ std::uint32_t index(
      std::uint32_t group, std::uint32_t seed) const noexcept {
    return group * capacityPerGroup + seed;
  }
};

/// Host columns mirroring the segment-seed device SoA.
struct CudaMuonSegmentSeedHostData {
  std::vector<SegmentSeedCoordType> tanBeta;
  std::vector<SegmentSeedCoordType> interceptY;
  std::vector<SegmentSeedCoordType> tanAlpha;
  std::vector<SegmentSeedCoordType> interceptX;
  std::vector<SegmentSeedCountType> counts;

  std::vector<std::uint32_t> parentBucket;
  std::vector<std::uint8_t> hasPhiExtension;

  std::vector<std::uint32_t> nSeeds;
  std::vector<std::uint32_t> nAssociatedHits;
  std::vector<std::uint32_t> associatedHitOffsets;
  std::vector<std::uint32_t> associatedHitIndices;
};

class CudaMuonSegmentSeedContainer;

/// Host proxy for one segment seed stored in a CudaMuonSegmentSeedContainer.
class CudaMuonSegmentSeedProxy {
 public:
  /// Parameter order used by parameters().
  enum ParameterIndex : std::size_t {
    TanBeta = 0u,
    InterceptY = 1u,
    TanAlpha = 2u,
    InterceptX = 3u,
  };

  using Parameters = std::array<SegmentSeedCoordType, 4u>;
  using HitType = CudaMuonSpacePointPtr;
  using HitVec = std::vector<HitType>;

  CudaMuonSegmentSeedProxy() = default;
  CudaMuonSegmentSeedProxy(const CudaMuonSegmentSeedContainer& container,
                           std::size_t slot) noexcept;

  /// Line slope and intercept in the non-precision projection.
  SegmentSeedCoordType tanAlpha() const;
  SegmentSeedCoordType interceptX() const;
  /// Line slope and intercept in the precision projection.
  SegmentSeedCoordType tanBeta() const;
  SegmentSeedCoordType interceptY() const;
  /// Return {tanBeta, interceptY, tanAlpha, interceptX}.
  const Parameters& parameters() const;

  /// Effective Hough count and associated measurements.
  SegmentSeedCountType getCounts() const;
  const HitVec& getHitsInMax() const;
  std::span<const std::uint32_t> associatedHitIndices() const;

  /// Flat index of the parent space-point bucket.
  std::size_t parentBucket() const;
  /// Whether tanAlpha and interceptX were determined by a Phi transform.
  bool hasPhiExtension() const;

  /// Position and direction in the local sector frame.
  const Acts::Vector3& localPosition() const;
  const Acts::Vector3& localDirection() const;

 private:
  const CudaMuonSegmentSeedContainer* m_container = nullptr;
  std::size_t m_slot = 0u;

  mutable Parameters m_parametersCache{};
  mutable HitVec m_hitsCache{};
  mutable Acts::Vector3 m_positionCache{Acts::Vector3::Zero()};
  mutable Acts::Vector3 m_directionCache{Acts::Vector3::UnitZ()};
};

/// Pointer-like segment-seed wrapper, analogous to CudaMuonSpacePointPtr.
class CudaMuonSegmentSeedPtr {
 public:
  using element_type = CudaMuonSegmentSeedProxy;

  CudaMuonSegmentSeedPtr() = default;
  CudaMuonSegmentSeedPtr(const CudaMuonSegmentSeedContainer& container,
                         std::size_t slot) noexcept;

  element_type* operator->() const noexcept { return &m_proxy; }
  element_type& operator*() const noexcept { return m_proxy; }
  explicit operator bool() const noexcept { return m_valid; }

 private:
  mutable element_type m_proxy{};
  bool m_valid = false;
};

/// CUDA-backed segment-seed container.
///
/// Device storage is grouped by parent Eta-maximum slot. Host iteration is
/// flattened over occupied slots only. Associated measurements are stored as
/// CSR indices into the referenced CudaMuonSpacePointContainer.
class CudaMuonSegmentSeedContainer {
 public:
  using value_type = CudaMuonSegmentSeedPtr;
  using size_type = std::size_t;

  template <bool read_only>
  using Iterator = Acts::detail::ContainerIterator<
      CudaMuonSegmentSeedContainer, value_type, size_type, read_only>;
  using iterator = Iterator<false>;
  using const_iterator = Iterator<true>;

  CudaMuonSegmentSeedContainer(size_type nGroups, size_type capacityPerGroup,
                               const CudaMuonSpacePointContainer& spacePoints);
  CudaMuonSegmentSeedContainer(const CudaMuonSegmentSeedContainer&) = delete;
  CudaMuonSegmentSeedContainer& operator=(
      const CudaMuonSegmentSeedContainer&) = delete;
  CudaMuonSegmentSeedContainer(CudaMuonSegmentSeedContainer&& other) noexcept;
  CudaMuonSegmentSeedContainer& operator=(
      CudaMuonSegmentSeedContainer&& other) noexcept;
  ~CudaMuonSegmentSeedContainer() noexcept;

  size_type size() const noexcept { return m_activeSlots.size(); }
  bool empty() const noexcept { return size() == 0u; }
  size_type groupCount() const noexcept { return m_nGroups; }
  size_type capacityPerGroup() const noexcept { return m_capacityPerGroup; }
  size_type totalCapacity() const noexcept {
    return groupCount() * capacityPerGroup();
  }

  size_type nSeeds(size_type group) const;
  value_type at(size_type group, size_type seed) const;
  value_type operator[](size_type index) const;
  value_type operator[](size_type index) { return std::as_const(*this)[index]; }

  iterator begin() noexcept { return {*this, 0u}; }
  iterator end() noexcept { return {*this, size()}; }
  const_iterator begin() const noexcept { return {*this, 0u}; }
  const_iterator end() const noexcept { return {*this, size()}; }

  void moveToDevice(cudaStream_t stream);
  void copyAssociationMetadataToHost(cudaStream_t stream);
  void allocateAssociationStorage(cudaStream_t stream);
  void copyAssociatedHitIndicesToHost(cudaStream_t stream);
  void moveToHost(cudaStream_t stream);
  void clearDevice() noexcept;

  bool isOnDevice() const noexcept { return m_onDevice; }
  CudaMuonSegmentSeedArrays deviceArrays() const noexcept { return m_device; }

 private:
  friend class CudaMuonSegmentSeedProxy;

  size_type m_nGroups = 0u;
  size_type m_capacityPerGroup = 0u;
  const CudaMuonSpacePointContainer* m_spacePoints = nullptr;
  CudaMuonSegmentSeedHostData m_host{};
  std::vector<std::uint32_t> m_activeSlots{};
  CudaMuonSegmentSeedArrays m_device{};
  bool m_onDevice = false;
  bool m_metadataOnHost = false;
  bool m_associationStorageAllocated = false;
  bool m_associatedHitIndicesOnHost = false;

  size_type slotIndex(size_type group, size_type seed) const noexcept {
    return group * capacityPerGroup() + seed;
  }
  void rebuildActiveSlots();
  void prepareAssociationStorageHost();
  void clearAssociationStorage() noexcept;
  void checkGroup(size_type group) const;
  void checkSeed(size_type group, size_type seed) const;
};

}  // namespace ActsExamples
