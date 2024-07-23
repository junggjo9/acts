// This file is part of the Acts project.
//
// Copyright (C) 2023 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "ActsExamples/Io/Csv/CsvMuonSpacePointReader.hpp"

#include "Acts/Definitions/Units.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"
#include "ActsExamples/EventData/MuonSpacePoint.hpp"
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsExamples/Utilities/Paths.hpp"
#include "ActsFatras/EventData/Barcode.hpp"
#include "ActsFatras/EventData/Hit.hpp"

#include <array>
#include <stdexcept>

#include <dfe/dfe_io_dsv.hpp>

#include "CsvOutputData.hpp"

namespace {}



namespace ActsExamples {
CsvMuonSpacePointReader::CsvMuonSpacePointReader(
    const CsvMuonSpacePointReader::Config& config,
    Acts::Logging::Level level)
    : m_cfg(config),
      // TODO check that all files (hits,cells,truth) exists
      m_eventsRange(
          determineEventFilesRange(m_cfg.inputDir, m_cfg.inputStem + ".csv")),
      m_logger(Acts::getDefaultLogger("CsvMuonSpacePointReader", level)) {
  if (m_cfg.inputStem.empty()) {
    throw std::invalid_argument("Missing input filename stem");
  }
  if (m_cfg.outputSpacePoints.empty()) {
    throw std::invalid_argument("Missing simulated hits output collection");
  }

  m_outputSpacePoints.initialize(m_cfg.outputSpacePoints);
}

std::string CsvMuonSpacePointReader::CsvMuonSpacePointReader::name()
    const {
  return "CsvMuonSpacePointReader";
}

std::pair<std::size_t, std::size_t>
CsvMuonSpacePointReader::availableEvents() const {
  return m_eventsRange;
}

ProcessCode CsvMuonSpacePointReader::read(
    const AlgorithmContext& ctx) {
  auto path = perEventFilepath(m_cfg.inputDir, m_cfg.inputStem + ".csv",
                               ctx.eventNumber);

  dfe::NamedTupleCsvReader<MuonSpacePointData> reader(path);

  MuonSpacePointData data;
  MuonSpacePointContainer outContainer{};

  int lastBucket{0};
  while (reader.read(data)) {
      if (lastBucket != data.bucketId){
          outContainer.emplace_back();
          lastBucket = data.bucketId;
      }
      MuonSpacePointBucket& bucket{outContainer.back()};

      MuonSpacePoint::Vector3 position{data.localPositionX, data.localPositionY, data.localPositionZ};
      MuonSpacePoint::CovMat cov{MuonSpacePoint::CovMat::Zero()};
      cov(Acts::eX, Acts::eX) = data.covX;
      cov(Acts::eY, Acts::eX) = data.covYX;
      cov(Acts::eX, Acts::eY) = data.covXY;
      cov(Acts::eY, Acts::eY) = data.covY;
      
      MuonSpacePoint spacePoint{std::move(position), std::move(cov), data.driftR,
                                data.stationName, data.stationEta, data.stationPhi,
                                data.gasGap, data.primaryCh, data.secondaryCh,
                                static_cast<bool>(data.measuresEta), 
                                static_cast<bool>(data.measuresPhi)};

      bucket.emplace_back(std::move(spacePoint));
  }
  for (MuonSpacePointBucket& bucket : outContainer) {
    std::sort(bucket.begin(), bucket.end());
  }
  m_outputSpacePoints(ctx, std::move(outContainer));

  return ProcessCode::SUCCESS;
}
}