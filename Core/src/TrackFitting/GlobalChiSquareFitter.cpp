// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Acts/TrackFitting/GlobalChiSquareFitter.hpp"

#include "Acts/Definitions/TrackParametrization.hpp"

void Acts::Experimental::updateGx2fParams(
    BoundTrackParameters& params, const Eigen::VectorXd& deltaParamsExtended,
    const std::size_t nMaterialSurfaces,
    std::unordered_map<GeometryIdentifier, MaterialProperties>& materialMap,
    const std::vector<GeometryIdentifier>& geoIdVector, const bool energyLoss) {
  // update params
  params.parameters() +=
      deltaParamsExtended.topLeftCorner<eBoundSize, 1>().eval();

  // update the scattering angles.
  for (std::size_t matSurface = 0; matSurface < nMaterialSurfaces;
       matSurface++) {
    const std::size_t deltaPosition =
        energyLoss ? eBoundSize + 3 * matSurface : eBoundSize + 2 * matSurface;
    const GeometryIdentifier geoId = geoIdVector[matSurface];
    const auto materialMapId = materialMap.find(geoId);
    assert(materialMapId != materialMap.end() &&
           "No material properties found for material surface.");
    materialMapId->second.scatteringAngles().block<2, 1>(2, 0) +=
        deltaParamsExtended.block<2, 1>(deltaPosition, 0).eval();
    // energy loss update for delta(q/p)
    if (energyLoss) {
      materialMapId->second.deltaQOverP() +=
          deltaParamsExtended.block<1, 1>(deltaPosition + 2, 0).value();
    }
  }

  return;
}

void Acts::Experimental::updateGx2fCovarianceParams(
    BoundMatrix& fullCovariancePredicted, Gx2fSystem& extendedSystem) {
  // make invertible
  for (std::size_t i = 0; i < extendedSystem.nDims(); ++i) {
    if (extendedSystem.aMatrix()(i, i) == 0.) {
      extendedSystem.aMatrix()(i, i) = 1.;
    }
  }

  visit_measurement(extendedSystem.findRequiredNdf(), [&](auto N) {
    fullCovariancePredicted.topLeftCorner<N, N>() =
        extendedSystem.aMatrix().inverse().topLeftCorner<N, N>();
  });

  return;
}

void Acts::Experimental::addMeasurementToGx2fSumsBackend(
    Gx2fSystem& extendedSystem,
    const std::vector<BoundMatrix>& jacobianFromStart,
    const Eigen::MatrixXd& covarianceMeasurement, const BoundVector& predicted,
    const Eigen::VectorXd& measurement, const Eigen::MatrixXd& projector,
    bool energyLoss, const Logger& logger) {
  // First, w try to invert the covariance matrix. If the inversion fails, we
  // can already abort.
  const auto safeInvCovMeasurement = safeInverse(covarianceMeasurement);
  if (!safeInvCovMeasurement) {
    ACTS_WARNING("addMeasurementToGx2fSums: safeInvCovMeasurement failed.");
    ACTS_VERBOSE("    covarianceMeasurement:\n" << covarianceMeasurement);
    return;
  }

  // Create an extended Jacobian. This one contains only eBoundSize rows,
  // because the rest is irrelevant. We fill it in the next steps.
  // TODO make dimsExtendedParams template with unrolling
  Eigen::MatrixXd extendedJacobian =
      Eigen::MatrixXd::Zero(eBoundSize, extendedSystem.nDims());

  // This part of the Jacobian comes from the material-less propagation
  extendedJacobian.topLeftCorner<eBoundSize, eBoundSize>() =
      jacobianFromStart[0];

  // If we have material, loop here over all Jacobians. We add extra columns for
  // their phi-theta projections. These parts account for the propagation of the
  // scattering angles. if energy loss is enabled, we also add the q/p
  // projection.
  for (std::size_t matSurface = 1; matSurface < jacobianFromStart.size();
       matSurface++) {
    const BoundMatrix jac = jacobianFromStart[matSurface];

    // const Matrix<eBoundSize, 2> jacPhiTheta =
    //     jac * Gx2fConstants::phiThetaProjector;

    // The position, where we need to insert the values in the extended Jacobian
    // accounting for material effects
    const std::size_t deltaPosition = energyLoss
                                          ? eBoundSize + 3 * (matSurface - 1)
                                          : eBoundSize + 2 * (matSurface - 1);
    if (energyLoss) {
      extendedJacobian.block<eBoundSize, 3>(0, deltaPosition) =
          jac * Gx2fConstants::phiThetaQOverPProjector;
    } else {
      extendedJacobian.block<eBoundSize, 2>(0, deltaPosition) =
          jac * Gx2fConstants::phiThetaProjector;
    }
  }

  const Eigen::MatrixXd projJacobian = projector * extendedJacobian;

  const Eigen::VectorXd projPredicted = projector * predicted;

  const Eigen::VectorXd residual = measurement - projPredicted;

  // Finally contribute to chi2sum, aMatrix, and bVector
  extendedSystem.chi2() +=
      (residual.transpose() * (*safeInvCovMeasurement) * residual)(0, 0);

  extendedSystem.aMatrix() +=
      (projJacobian.transpose() * (*safeInvCovMeasurement) * projJacobian)
          .eval();

  extendedSystem.bVector() +=
      (residual.transpose() * (*safeInvCovMeasurement) * projJacobian)
          .eval()
          .transpose();

  ACTS_VERBOSE(
      "Contributions in addMeasurementToGx2fSums:\n"
      << "    predicted:   " << predicted.transpose() << "\n"
      << "    measurement: " << measurement.transpose() << "\n"
      << "    covarianceMeasurement:\n"
      << covarianceMeasurement << "\n"
      << "    projector:\n"
      << projector.eval() << "\n"
      << "    projJacobian:\n"
      << projJacobian.eval() << "\n"
      << "    projPredicted: " << (projPredicted.transpose()).eval() << "\n"
      << "    residual: " << (residual.transpose()).eval() << "\n"
      << "    extendedJacobian:\n"
      << extendedJacobian << "\n"
      << "    aMatrix contribution:\n"
      << (projJacobian.transpose() * (*safeInvCovMeasurement) * projJacobian)
             .eval()
      << "\n"
      << "    bVector contribution: "
      << (residual.transpose() * (*safeInvCovMeasurement) * projJacobian).eval()
      << "\n"
      << "    chi2sum contribution: "
      << (residual.transpose() * (*safeInvCovMeasurement) * residual)(0, 0)
      << "\n"
      << "    safeInvCovMeasurement:\n"
      << (*safeInvCovMeasurement));
}

Eigen::VectorXd Acts::Experimental::computeGx2fDeltaParams(
    const Acts::Experimental::Gx2fSystem& extendedSystem) {
  return extendedSystem.aMatrix().colPivHouseholderQr().solve(
      extendedSystem.bVector());
}

double Acts::Experimental::computeDeltaQOverPFromEnergyLoss(
    const MaterialSlab& slab, const double eLoss,
    const ParticleHypothesis& particleHypothesis, const double qOverP,
    const Direction direction) {
  const PdgParticle absPdg = particleHypothesis.absolutePdg();
  const double mass = particleHypothesis.mass();
  const double absQ = particleHypothesis.absoluteCharge();

  // Nothing to do for vacuum, when the momentum is externally fixed, or for
  // neutral particles, which do not lose energy by ionisation and for which the
  // material functions are not defined
  if (slab.isVacuum() || particleHypothesis.hasMomentumHypothesis() ||
      absQ <= 0.) {
    return 0.;
  }

  const double momentum = particleHypothesis.extractMomentum(qOverP);

  // in forward(backward) propagation, energy decreases(increases)
  const double nextE = fastHypot(mass, momentum) - eLoss * direction;
  // put the particle at rest if the energy loss is too large
  double nextP = (mass < nextE) ? fastCathetus(nextE, mass) : 0.;

  // minimum momentum below which we will not push particles via material update
  static constexpr double minP = 10 * UnitConstants::MeV;
  nextP = std::max(minP, nextP);

  const double nextQOverP =
      particleHypothesis.qOverP(nextP, std::copysign(absQ, qOverP));

  return nextQOverP - qOverP;
}
