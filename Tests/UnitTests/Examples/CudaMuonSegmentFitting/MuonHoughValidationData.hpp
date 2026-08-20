// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/Utilities/Logger.hpp"
#include "ActsExamples/EventData/CudaMuonSpacePoint.hpp"
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsExamples/Framework/WhiteBoard.hpp"
#include "ActsExamples/Io/Root/RootMuonSpacePointReader.hpp"
#include "ActsTests/CommonHelpers/WhiteBoardUtilities.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ActsTests {

struct EtaValidationTruth {
  std::uint32_t validationBucketId = 0u;
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint32_t segmentIndex = 0u;
  double tanBeta = 0.0;
  double y0 = 0.0;
  std::vector<std::uint32_t> truthHitIndices{};
  double tanAlpha = 0.0;
  double x0 = 0.0;
  std::vector<std::uint32_t> phiTruthHitIndices{};
};

struct EtaValidationBucket {
  std::uint32_t eventId = 0u;
  std::uint16_t sourceBucketId = 0u;
  std::uint16_t nTruthSegments = 0u;
};

struct EtaValidationBatch {
  ActsExamples::CudaMuonSpacePointContainer spacePoints;
  std::vector<EtaValidationBucket> buckets;
  std::vector<EtaValidationTruth> truth;
};

namespace detail {

inline ActsExamples::MuonSpacePointContainer readSpacePoints(
    const std::filesystem::path& inputPath,
    std::vector<EtaValidationBucket>& bucketMetadata) {
  ActsExamples::RootMuonSpacePointReader::Config config{};
  config.filePath = inputPath.string();
  config.treeName = "MuonSpacePoints";
  config.outputSpacePoints = "ValidationSpacePoints";
  ActsExamples::RootMuonSpacePointReader reader{
      config, Acts::Logging::Level::ERROR};

  const auto [firstEvent, lastEvent] = reader.availableEvents();
  if (firstEvent != 0u) {
    throw std::runtime_error("Unexpected first space-point event");
  }
  ActsExamples::MuonSpacePointContainer batched;
  for (std::size_t eventNumber = firstEvent; eventNumber < lastEvent;
       ++eventNumber) {
    ActsExamples::WhiteBoard board{Acts::getDefaultLogger(
        "MuonHoughValidationInput", Acts::Logging::Level::ERROR)};
    ActsExamples::AlgorithmContext context{0u, eventNumber, board, 0u};
    if (reader.read(context) != ActsExamples::ProcessCode::SUCCESS) {
      throw std::runtime_error("RootMuonSpacePointReader failed");
    }
    auto eventSpacePoints =
        ActsTests::getFromWhiteBoard<ActsExamples::MuonSpacePointContainer>(
            config.outputSpacePoints, board);

    for (std::size_t sourceBucket = 0u;
         sourceBucket < eventSpacePoints.size(); ++sourceBucket) {
      auto& bucket = eventSpacePoints[sourceBucket];
      if (eventNumber > std::numeric_limits<std::uint32_t>::max() ||
          sourceBucket > std::numeric_limits<std::uint16_t>::max()) {
        throw std::runtime_error("Space-point event or bucket ID overflow");
      }
      bucketMetadata.push_back(
          {static_cast<std::uint32_t>(eventNumber),
           static_cast<std::uint16_t>(sourceBucket), 0u});
      batched.push_back(std::move(bucket));
    }
  }
  return batched;
}

}  // namespace detail

/// Load and batch production space points without reading validation truth.
inline EtaValidationBatch fileReadEtaValidation(
    const std::filesystem::path& spacePointPath) {
  std::vector<EtaValidationBucket> buckets;
  auto input = detail::readSpacePoints(spacePointPath, buckets);
  return {ActsExamples::CudaMuonSpacePointContainer{input},
          std::move(buckets), {}};
}

}  // namespace ActsTests
