// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Acts/Propagator/detail/CovarianceEngine.hpp"

#include "Acts/Definitions/Common.hpp"
#include "Acts/Definitions/TrackParametrization.hpp"
#include "Acts/EventData/GenericBoundTrackParameters.hpp"
#include "Acts/EventData/TrackParameters.hpp"
#include "Acts/EventData/TransformationHelpers.hpp"
#include "Acts/EventData/detail/CorrectedTransformationFreeToBound.hpp"
#include "Acts/Propagator/detail/JacobianEngine.hpp"
#include "Acts/Utilities/Result.hpp"

#include <optional>
#include <system_error>
#include <utility>

namespace Acts {

/// Some type defs
using Jacobian = BoundMatrix;
using BoundState = std::tuple<BoundTrackParameters, Jacobian, double>;

Result<BoundState> detail::boundState(
    const GeometryContext& geoContext, const Surface& surface,
    BoundSquareMatrix& boundCovariance, BoundMatrix& fullTransportJacobian,
    FreeMatrix& freeTransportJacobian, FreeVector& freeToPathDerivatives,
    BoundToFreeMatrix& boundToFreeJacobian,
    const std::optional<FreeMatrix>& additionalFreeCovariance,
    FreeVector& freeParameters, const ParticleHypothesis& particleHypothesis,
    bool covTransport, double accumulatedPath,
    const FreeToBoundCorrection& freeToBoundCorrection) {
  // Create the bound parameters
  Result<BoundVector> bv =
      transformFreeToBoundParameters(freeParameters, surface, geoContext);
  if (!bv.ok()) {
    return bv.error();
  }

  // Covariance transport
  std::optional<BoundSquareMatrix> cov = std::nullopt;
  if (covTransport) {
    // Calculate the jacobian and transport the covarianceMatrix to final local.
    // Then reinitialize the transportJacobian, derivatives and the
    // boundToFreeJacobian
    transportCovarianceToBound(
        geoContext, surface, boundCovariance, fullTransportJacobian,
        freeTransportJacobian, freeToPathDerivatives, boundToFreeJacobian,
        additionalFreeCovariance, freeParameters, freeToBoundCorrection);
    cov = boundCovariance;
  }

  // Create the bound state
  return std::make_tuple(
      BoundTrackParameters(surface.getSharedPtr(), *bv, std::move(cov),
                           particleHypothesis),
      fullTransportJacobian, accumulatedPath);
}

BoundState detail::curvilinearState(
    BoundSquareMatrix& boundCovariance, BoundMatrix& fullTransportJacobian,
    FreeMatrix& freeTransportJacobian, FreeVector& freeToPathDerivatives,
    BoundToFreeMatrix& boundToFreeJacobian,
    const std::optional<FreeMatrix>& additionalFreeCovariance,
    const FreeVector& freeParameters,
    const ParticleHypothesis& particleHypothesis, bool covTransport,
    double accumulatedPath) {
  const Vector3& direction = freeParameters.segment<3>(eFreeDir0);

  // Covariance transport
  std::optional<BoundSquareMatrix> cov = std::nullopt;
  if (covTransport) {
    // Calculate the jacobian and transport the covarianceMatrix to final local.
    // Then reinitialize the transportJacobian, derivatives and the
    // boundToFreeJacobian
    transportCovarianceToCurvilinear(boundCovariance, fullTransportJacobian,
                                     freeTransportJacobian,
                                     freeToPathDerivatives, boundToFreeJacobian,
                                     additionalFreeCovariance, direction);
    cov = boundCovariance;
  }

  // Create the curvilinear parameters
  Vector4 pos4 = Vector4::Zero();
  pos4[ePos0] = freeParameters[eFreePos0];
  pos4[ePos1] = freeParameters[eFreePos1];
  pos4[ePos2] = freeParameters[eFreePos2];
  pos4[eTime] = freeParameters[eFreeTime];
  BoundTrackParameters curvilinearParams =
      BoundTrackParameters::createCurvilinear(
          pos4, direction, freeParameters[eFreeQOverP], std::move(cov),
          particleHypothesis);
  // Create the curvilinear state
  return {std::move(curvilinearParams), fullTransportJacobian, accumulatedPath};
}

void detail::transportCovarianceToBound(
    const GeometryContext& geoContext, const Surface& surface,
    BoundSquareMatrix& boundCovariance, BoundMatrix& fullTransportJacobian,
    FreeMatrix& freeTransportJacobian, FreeVector& freeToPathDerivatives,
    BoundToFreeMatrix& boundToFreeJacobian,
    const std::optional<FreeMatrix>& additionalFreeCovariance,
    FreeVector& freeParameters,
    const FreeToBoundCorrection& freeToBoundCorrection) {
  FreeToBoundMatrix freeToBoundJacobian;

  // Calculate the full jacobian from local parameters at the start surface to
  // current bound parameters
  boundToBoundTransportJacobian(geoContext, surface, freeParameters,
                                boundToFreeJacobian, freeTransportJacobian,
                                freeToBoundJacobian, freeToPathDerivatives,
                                fullTransportJacobian);

  bool correction = false;
  if (freeToBoundCorrection) {
    BoundToFreeMatrix startBoundToFinalFreeJacobian =
        freeTransportJacobian * boundToFreeJacobian;
    FreeSquareMatrix freeCovariance = startBoundToFinalFreeJacobian *
                                      boundCovariance *
                                      startBoundToFinalFreeJacobian.transpose();

    auto transformer =
        detail::CorrectedFreeToBoundTransformer(freeToBoundCorrection);
    auto correctedRes =
        transformer(freeParameters, freeCovariance, surface, geoContext);

    if (correctedRes.has_value()) {
      auto correctedValue = correctedRes.value();
      BoundVector boundParams = std::get<BoundVector>(correctedValue);
      // 1. Update the free parameters with the corrected bound parameters
      freeParameters =
          transformBoundToFreeParameters(surface, geoContext, boundParams);

      // 2. Update the bound covariance
      boundCovariance = std::get<BoundSquareMatrix>(correctedValue);

      correction = true;
    }
  }

  if (!correction) {
    // Apply the actual covariance transport to get covariance of the current
    // bound parameters
    boundCovariance = fullTransportJacobian * boundCovariance *
                      fullTransportJacobian.transpose();
  }

  if (additionalFreeCovariance) {
    boundCovariance += freeToBoundJacobian * (*additionalFreeCovariance) *
                       freeToBoundJacobian.transpose();
  }

  // Reinitialize jacobian components:
  // ->The transportJacobian is reinitialized to Identity
  // ->The derivatives is reinitialized to Zero
  // ->The boundToFreeJacobian is initialized to that at the current surface
  reinitializeJacobians(geoContext, surface, freeTransportJacobian,
                        freeToPathDerivatives, boundToFreeJacobian,
                        freeParameters);
}

void detail::transportCovarianceToCurvilinear(
    BoundSquareMatrix& boundCovariance, BoundMatrix& fullTransportJacobian,
    FreeMatrix& freeTransportJacobian, FreeVector& freeToPathDerivatives,
    BoundToFreeMatrix& boundToFreeJacobian,
    const std::optional<FreeMatrix>& additionalFreeCovariance,
    const Vector3& direction) {
  FreeToBoundMatrix freeToBoundJacobian;

  // Calculate the full jacobian from local parameters at the start surface to
  // current curvilinear parameters
  boundToCurvilinearTransportJacobian(
      direction, boundToFreeJacobian, freeTransportJacobian,
      freeToBoundJacobian, freeToPathDerivatives, fullTransportJacobian);

  // Apply the actual covariance transport to get covariance of the current
  // curvilinear parameters
  boundCovariance = fullTransportJacobian * boundCovariance *
                    fullTransportJacobian.transpose();

  if (additionalFreeCovariance) {
    boundCovariance += freeToBoundJacobian * (*additionalFreeCovariance) *
                       freeToBoundJacobian.transpose();
  }

  // Reinitialize jacobian components:
  // ->The free transportJacobian is reinitialized to Identity
  // ->The path derivatives is reinitialized to Zero
  // ->The boundToFreeJacobian is reinitialized to that at the current
  // curvilinear surface
  reinitializeJacobians(freeTransportJacobian, freeToPathDerivatives,
                        boundToFreeJacobian, direction);
}

Result<BoundTrackParameters> detail::boundToBoundConversion(
    const GeometryContext& gctx, const BoundTrackParameters& boundParameters,
    const Surface& targetSurface, const Vector3& bField) {
  const auto& sourceSurface = boundParameters.referenceSurface();

  FreeVector freePars = transformBoundToFreeParameters(
      sourceSurface, gctx, boundParameters.parameters());

  auto res = transformFreeToBoundParameters(freePars, targetSurface, gctx);

  if (!res.ok()) {
    return res.error();
  }
  BoundVector parOut = *res;

  std::optional<BoundMatrix> covOut = std::nullopt;

  if (boundParameters.covariance().has_value()) {
    BoundToFreeMatrix boundToFreeJacobian = sourceSurface.boundToFreeJacobian(
        gctx, freePars.segment<3>(eFreePos0), freePars.segment<3>(eFreeDir0));

    FreeMatrix freeTransportJacobian = FreeMatrix::Identity();

    FreeVector freeToPathDerivatives = FreeVector::Zero();
    freeToPathDerivatives.head<3>() = freePars.segment<3>(eFreeDir0);

    freeToPathDerivatives.segment<3>(eFreeDir0) =
        bField.cross(freePars.segment<3>(eFreeDir0));

    BoundMatrix boundToBoundJac;
    FreeToBoundMatrix freeToBoundJacobian;
    detail::boundToBoundTransportJacobian(
        gctx, targetSurface, freePars, boundToFreeJacobian,
        freeTransportJacobian, freeToBoundJacobian, freeToPathDerivatives,
        boundToBoundJac);

    covOut = boundToBoundJac * (*boundParameters.covariance()) *
             boundToBoundJac.transpose();
  }

  return BoundTrackParameters{targetSurface.getSharedPtr(), parOut, covOut,
                              boundParameters.particleHypothesis()};
}

}  // namespace Acts
