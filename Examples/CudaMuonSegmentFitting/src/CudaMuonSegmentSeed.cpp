// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ActsExamples/EventData/CudaMuonSegmentSeed.hpp"

#include "ActsExamples/Utilities/CudaUtilities.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace {

using namespace ActsExamples;

void resizeHostData(CudaMuonSegmentSeedHostData& host, std::size_t nSlots,
                    std::size_t nGroups) {
  host.tanBeta.resize(nSlots);
  host.interceptY.resize(nSlots);
  host.tanAlpha.resize(nSlots);
  host.interceptX.resize(nSlots);
  host.counts.resize(nSlots);
  host.parentBucket.resize(nSlots);
  host.hasPhiExtension.resize(nSlots);
  host.nSeeds.resize(nGroups);
  host.nAssociatedHits.resize(nSlots);
}

void allocateDeviceData(CudaMuonSegmentSeedArrays& device,
                        std::size_t nSlots, std::size_t nGroups) {
  allocateDeviceColumn(device.tanBeta, nSlots);
  allocateDeviceColumn(device.interceptY, nSlots);
  allocateDeviceColumn(device.tanAlpha, nSlots);
  allocateDeviceColumn(device.interceptX, nSlots);
  allocateDeviceColumn(device.counts, nSlots);
  allocateDeviceColumn(device.parentBucket, nSlots);
  allocateDeviceColumn(device.hasPhiExtension, nSlots);
  allocateDeviceColumn(device.xBin, nSlots);
  allocateDeviceColumn(device.yBin, nSlots);
  allocateDeviceColumn(device.nSeeds, nGroups);
  allocateDeviceColumn(device.nAssociatedHits, nSlots);
}

void freeDeviceData(CudaMuonSegmentSeedArrays& device) noexcept {
  freeDeviceColumn(device.tanBeta);
  freeDeviceColumn(device.interceptY);
  freeDeviceColumn(device.tanAlpha);
  freeDeviceColumn(device.interceptX);
  freeDeviceColumn(device.counts);
  freeDeviceColumn(device.parentBucket);
  freeDeviceColumn(device.hasPhiExtension);
  freeDeviceColumn(device.xBin);
  freeDeviceColumn(device.yBin);
  freeDeviceColumn(device.nSeeds);
  freeDeviceColumn(device.nAssociatedHits);
  freeDeviceColumn(device.associatedHitOffsets);
  freeDeviceColumn(device.associatedHitIndices);
}

}  // namespace

namespace ActsExamples {

CudaMuonSegmentSeedProxy::CudaMuonSegmentSeedProxy(
    const CudaMuonSegmentSeedContainer& container, std::size_t slot) noexcept
    : m_container{&container}, m_slot{slot} {}

SegmentSeedCoordType CudaMuonSegmentSeedProxy::tanAlpha() const {
  return m_container->m_host.tanAlpha[m_slot];
}

SegmentSeedCoordType CudaMuonSegmentSeedProxy::interceptX() const {
  return m_container->m_host.interceptX[m_slot];
}

SegmentSeedCoordType CudaMuonSegmentSeedProxy::tanBeta() const {
  return m_container->m_host.tanBeta[m_slot];
}

SegmentSeedCoordType CudaMuonSegmentSeedProxy::interceptY() const {
  return m_container->m_host.interceptY[m_slot];
}

const CudaMuonSegmentSeedProxy::Parameters&
CudaMuonSegmentSeedProxy::parameters() const {
  m_parametersCache = {tanBeta(), interceptY(), tanAlpha(), interceptX()};
  return m_parametersCache;
}

SegmentSeedCountType CudaMuonSegmentSeedProxy::getCounts() const {
  return m_container->m_host.counts[m_slot];
}

std::span<const std::uint32_t>
CudaMuonSegmentSeedProxy::associatedHitIndices() const {
  if (!m_container->m_associatedHitIndicesOnHost) {
    throw std::logic_error(
        "Segment-seed hit indices are not available on the host");
  }
  const std::size_t begin =
      m_container->m_host.associatedHitOffsets[m_slot];
  const std::size_t end =
      m_container->m_host.associatedHitOffsets[m_slot + 1u];
  return {m_container->m_host.associatedHitIndices.data() + begin,
          end - begin};
}

const CudaMuonSegmentSeedProxy::HitVec&
CudaMuonSegmentSeedProxy::getHitsInMax() const {
  m_hitsCache.clear();
  const auto indices = associatedHitIndices();
  m_hitsCache.reserve(indices.size());
  for (const std::uint32_t index : indices) {
    m_hitsCache.emplace_back(*m_container->m_spacePoints, index);
  }
  return m_hitsCache;
}

std::size_t CudaMuonSegmentSeedProxy::parentBucket() const {
  return m_container->m_host.parentBucket[m_slot];
}

bool CudaMuonSegmentSeedProxy::hasPhiExtension() const {
  return m_container->m_host.hasPhiExtension[m_slot] != 0u;
}

const Acts::Vector3& CudaMuonSegmentSeedProxy::localPosition() const {
  m_positionCache = Acts::Vector3{interceptX(), interceptY(), 0.0};
  return m_positionCache;
}

const Acts::Vector3& CudaMuonSegmentSeedProxy::localDirection() const {
  m_directionCache = Acts::Vector3{tanAlpha(), tanBeta(), 1.0}.normalized();
  return m_directionCache;
}

CudaMuonSegmentSeedPtr::CudaMuonSegmentSeedPtr(
    const CudaMuonSegmentSeedContainer& container, std::size_t slot) noexcept
    : m_proxy{container, slot}, m_valid{true} {}

CudaMuonSegmentSeedContainer::CudaMuonSegmentSeedContainer(
    size_type nGroups, size_type capacityPerGroup,
    const CudaMuonSpacePointContainer& spacePoints)
    : m_nGroups{nGroups},
      m_capacityPerGroup{capacityPerGroup},
      m_spacePoints{&spacePoints} {
  if (nGroups == 0u) {
    throw std::invalid_argument("Segment-seed group count must be non-zero");
  }
  if (capacityPerGroup == 0u) {
    throw std::invalid_argument("Segment-seed group capacity must be non-zero");
  }
  if (nGroups > std::numeric_limits<size_type>::max() / capacityPerGroup) {
    throw std::overflow_error("Segment-seed capacity overflows std::size_t");
  }
  const size_type capacity = totalCapacity();
  if (capacity > std::numeric_limits<std::uint32_t>::max()) {
    throw std::overflow_error(
        "Segment-seed capacity must fit into std::uint32_t");
  }
  resizeHostData(m_host, capacity, groupCount());
}

CudaMuonSegmentSeedContainer::CudaMuonSegmentSeedContainer(
    CudaMuonSegmentSeedContainer&& other) noexcept
    : m_nGroups{std::exchange(other.m_nGroups, 0u)},
      m_capacityPerGroup{std::exchange(other.m_capacityPerGroup, 0u)},
      m_spacePoints{std::exchange(other.m_spacePoints, nullptr)},
      m_host{std::move(other.m_host)},
      m_activeSlots{std::move(other.m_activeSlots)},
      m_device{std::exchange(other.m_device, CudaMuonSegmentSeedArrays{})},
      m_onDevice{std::exchange(other.m_onDevice, false)},
      m_metadataOnHost{std::exchange(other.m_metadataOnHost, false)},
      m_associationStorageAllocated{
          std::exchange(other.m_associationStorageAllocated, false)},
      m_associatedHitIndicesOnHost{
          std::exchange(other.m_associatedHitIndicesOnHost, false)} {}

CudaMuonSegmentSeedContainer& CudaMuonSegmentSeedContainer::operator=(
    CudaMuonSegmentSeedContainer&& other) noexcept {
  if (this == &other) {
    return *this;
  }
  clearDevice();
  m_nGroups = std::exchange(other.m_nGroups, 0u);
  m_capacityPerGroup = std::exchange(other.m_capacityPerGroup, 0u);
  m_spacePoints = std::exchange(other.m_spacePoints, nullptr);
  m_host = std::move(other.m_host);
  m_activeSlots = std::move(other.m_activeSlots);
  m_device = std::exchange(other.m_device, CudaMuonSegmentSeedArrays{});
  m_onDevice = std::exchange(other.m_onDevice, false);
  m_metadataOnHost = std::exchange(other.m_metadataOnHost, false);
  m_associationStorageAllocated =
      std::exchange(other.m_associationStorageAllocated, false);
  m_associatedHitIndicesOnHost =
      std::exchange(other.m_associatedHitIndicesOnHost, false);
  return *this;
}

CudaMuonSegmentSeedContainer::~CudaMuonSegmentSeedContainer() noexcept {
  clearDevice();
}

CudaMuonSegmentSeedContainer::size_type
CudaMuonSegmentSeedContainer::nSeeds(size_type group) const {
  if (!m_metadataOnHost) {
    throw std::logic_error("Segment-seed metadata is not available on host");
  }
  checkGroup(group);
  const size_type count = m_host.nSeeds[group];
  if (count > capacityPerGroup()) {
    throw std::runtime_error("Invalid segment-seed group count");
  }
  return count;
}

CudaMuonSegmentSeedContainer::value_type CudaMuonSegmentSeedContainer::at(
    size_type group, size_type seed) const {
  checkSeed(group, seed);
  return value_type{*this, slotIndex(group, seed)};
}

CudaMuonSegmentSeedContainer::value_type
CudaMuonSegmentSeedContainer::operator[](size_type index) const {
  if (index >= size()) {
    throw std::out_of_range("Segment-seed index out of range");
  }
  return value_type{*this, m_activeSlots[index]};
}

void CudaMuonSegmentSeedContainer::moveToDevice(cudaStream_t /*stream*/) {
  clearDevice();
  m_activeSlots.clear();
  m_host.associatedHitOffsets.clear();
  m_host.associatedHitIndices.clear();
  m_metadataOnHost = false;
  m_associationStorageAllocated = false;
  m_associatedHitIndicesOnHost = false;

  m_device.nGroups = static_cast<std::uint32_t>(groupCount());
  m_device.capacityPerGroup =
      static_cast<std::uint32_t>(capacityPerGroup());
  try {
    allocateDeviceData(m_device, totalCapacity(), groupCount());
  } catch (...) {
    clearDevice();
    throw;
  }
  m_onDevice = true;
}

void CudaMuonSegmentSeedContainer::copyAssociationMetadataToHost(
    cudaStream_t stream) {
  if (!m_onDevice) {
    throw std::logic_error("Segment-seed container is not on the device");
  }
  copyColumnToHost(m_host.nSeeds, m_device.nSeeds, stream);
  copyColumnToHost(m_host.nAssociatedHits, m_device.nAssociatedHits, stream);
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
  m_metadataOnHost = true;
  rebuildActiveSlots();
}

void CudaMuonSegmentSeedContainer::prepareAssociationStorageHost() {
  clearAssociationStorage();
  m_host.associatedHitOffsets.assign(totalCapacity() + 1u, 0u);
  std::uint64_t totalHits = 0u;

  for (size_type group = 0u; group < groupCount(); ++group) {
    const size_type seeds = nSeeds(group);
    for (size_type seed = 0u; seed < capacityPerGroup(); ++seed) {
      const size_type slot = slotIndex(group, seed);
      const std::uint32_t count = m_host.nAssociatedHits[slot];
      if (seed >= seeds && count != 0u) {
        throw std::runtime_error(
            "Unoccupied segment-seed slot contains associated hits");
      }
      if (seed < seeds) {
        totalHits += count;
      }
      if (totalHits > std::numeric_limits<std::uint32_t>::max()) {
        throw std::overflow_error(
            "Total segment-seed hit count must fit in std::uint32_t");
      }
      m_host.associatedHitOffsets[slot + 1u] =
          static_cast<std::uint32_t>(totalHits);
    }
  }
  m_host.associatedHitIndices.resize(static_cast<size_type>(totalHits));
}

void CudaMuonSegmentSeedContainer::allocateAssociationStorage(
    cudaStream_t stream) {
  if (!m_onDevice || !m_metadataOnHost) {
    throw std::logic_error(
        "Segment-seed metadata must be copied before association allocation");
  }
  prepareAssociationStorageHost();
  try {
    allocateDeviceColumn(m_device.associatedHitOffsets,
                         m_host.associatedHitOffsets.size());
    copyColumnToDevice(m_device.associatedHitOffsets,
                       m_host.associatedHitOffsets, stream);
    ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
    allocateDeviceColumn(m_device.associatedHitIndices,
                         m_host.associatedHitIndices.size());
    m_device.totalAssociatedHits =
        static_cast<std::uint32_t>(m_host.associatedHitIndices.size());
    m_associationStorageAllocated = true;
    m_associatedHitIndicesOnHost = false;
  } catch (...) {
    clearAssociationStorage();
    throw;
  }
}

void CudaMuonSegmentSeedContainer::copyAssociatedHitIndicesToHost(
    cudaStream_t stream) {
  if (!m_associationStorageAllocated) {
    throw std::logic_error(
        "Segment-seed association storage has not been allocated");
  }
  copyColumnToHost(m_host.associatedHitIndices,
                   m_device.associatedHitIndices, stream);
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
  m_associatedHitIndicesOnHost = true;
}

void CudaMuonSegmentSeedContainer::moveToHost(cudaStream_t stream) {
  if (!m_onDevice) {
    return;
  }
  copyColumnToHost(m_host.tanBeta, m_device.tanBeta, stream);
  copyColumnToHost(m_host.interceptY, m_device.interceptY, stream);
  copyColumnToHost(m_host.tanAlpha, m_device.tanAlpha, stream);
  copyColumnToHost(m_host.interceptX, m_device.interceptX, stream);
  copyColumnToHost(m_host.counts, m_device.counts, stream);
  copyColumnToHost(m_host.parentBucket, m_device.parentBucket, stream);
  copyColumnToHost(m_host.hasPhiExtension, m_device.hasPhiExtension, stream);
  copyColumnToHost(m_host.nSeeds, m_device.nSeeds, stream);
  copyColumnToHost(m_host.nAssociatedHits, m_device.nAssociatedHits, stream);
  ACTS_CUDA_CHECK(cudaStreamSynchronize(stream));
  m_metadataOnHost = true;
  rebuildActiveSlots();
}

void CudaMuonSegmentSeedContainer::rebuildActiveSlots() {
  m_activeSlots.clear();
  for (size_type group = 0u; group < groupCount(); ++group) {
    const size_type count = nSeeds(group);
    for (size_type seed = 0u; seed < count; ++seed) {
      m_activeSlots.push_back(
          static_cast<std::uint32_t>(slotIndex(group, seed)));
    }
  }
}

void CudaMuonSegmentSeedContainer::clearAssociationStorage() noexcept {
  freeDeviceColumn(m_device.associatedHitOffsets);
  freeDeviceColumn(m_device.associatedHitIndices);
  m_device.totalAssociatedHits = 0u;
  m_associationStorageAllocated = false;
}

void CudaMuonSegmentSeedContainer::clearDevice() noexcept {
  freeDeviceData(m_device);
  m_device = {};
  m_onDevice = false;
  m_associationStorageAllocated = false;
}

void CudaMuonSegmentSeedContainer::checkGroup(size_type group) const {
  if (group >= groupCount()) {
    throw std::out_of_range("Segment-seed group index out of range");
  }
}

void CudaMuonSegmentSeedContainer::checkSeed(size_type group,
                                             size_type seed) const {
  if (seed >= nSeeds(group)) {
    throw std::out_of_range("Segment-seed slot index out of range");
  }
}

}  // namespace ActsExamples
