// This file is part of the Acts project.
//
// Copyright (C) 2023 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "Acts/Definitions/Algebra.hpp"

namespace ActsExamples {

    /// Representation of a muon space point used for track seeding and fitting.
    /// Space point abstract the various muon detector technologies to a common 
    /// class interface. Further two 1D measurements, the first providing information
    /// about the precision eta-coordinate and the second providing information about the
    /// complementary phi-coordinate may be combined into a single muon space point
    class MuonSpacePoint {
        public:
            /// Abrivations of the Eigen types used in this class
            using Scalar = Acts::ActsScalar;
            using CovMat = Acts::ActsSquareMatrix<2>;
            using Vector3 = Acts::Vector3;
            
            /// Default constructor.
            MuonSpacePoint() = default;
            
            /** @brief Full constructor taking all information needed to construct a space point
                @param pos: Position of the space point within the local chamber frame.
                            The convention of the chamber frame is as follows:
                                x-axis: Points along the global phi direction
                                y-axis: Points along the global eta direction
                                z-axis: Points outwards the chamber / detector
                @param covariance: Covariance estimate of the space point.
                @param driftRadius: Measured drift radius if the space point is built up out of 
                                    tube measurements
                @param stationName: Identify the station of the chamber
                @param stationEta:  Placement of the chamber along the eta direction
                @param stationPhi:  Placement of the chamber along the phi ring
                @param gasGap:      In which gasGap of the chamber is the hit recorded
                @param primaryChannel: Number of the fired strip of the primary measurement
                @param secondaryChannel: If the space point is a combination of 2 1D measurements,
                                         the fired strip of the complementary phi measurement is passed
                @param measuresEta: Does the space point measure the eta coordinate
                @param measuresPhi: Does the space point measure the phi coordinate **/
            MuonSpacePoint(Vector3&& pos,
                           CovMat&& covariance,
                           Scalar driftRadius,
                           int stationName,
                           int stationEta,
                           int stationPhi,
                           int gasGap,
                           int primaryChannel,
                           int secondaryChannel,
                           bool measuresEta,
                           bool measuresPhi);

            /** @brief Returns the space point position inside the chamber */
            const Vector3& position() const;
            /** @brief Sets the space point position inside the chamber */
            void setPosition(Vector3&& pos);

            /** @brief Returns the covariance estimate of the space point */
            const CovMat& covariance() const;
            /** @brief Sets the covariance estimate of the space point */
            void setCovariance(CovMat&& cov);

            /** @brief Returns the stationName of the associated chamber */
            int stationName() const;
            /** @brief Sets the statioName of the associated chamber */
            void setStationName(int stName);
            
            /** @brief Returns the stationEta position of the associated chamber */
            int stationEta() const;
            /** @brief Sets the stationEta position of the associated chamber */
            void setStationEta(int stEta);
            
            /** @brief Returns the sationPhi position of the associated chamber */
            int stationPhi() const;
            /** @brief Sets the stationPhi position of the associated chamber */
            void setStationPhi(int stPhi);

            /** @brief Returns the gasGap number in which the space point was produced */
            int gasGap() const;
            /** @brief Sets the gasGap number in which the space point was produced */
            void setGasGap(int gap);
            
            int primaryChannel() const;
            void setPrimaryChannel(int channel);

            int secondaryChannel() const;
            void setSecondaryChannel(int channel);

            bool measuresEta() const;
            void setMeasuresEta(bool measEta);

            bool measuresPhi() const;
            void setMeasuresPhi(bool measPhi);


            bool operator<(const MuonSpacePoint& other) const;
            bool operator==(const MuonSpacePoint& other) const;
            

        private:
            Vector3 m_pos{Vector3::Zero()};
            CovMat m_cov{CovMat::Identity()};

            Scalar m_driftR{0.};

            int m_stationName{-1};
            int m_stationEta{0};
            int m_stationPhi{-1};
            int m_gasGap{-1};
            int m_primCh{-1};
            int m_secondCh{-1};

            bool m_measEta{false};
            bool m_measPhi{false};
    };

    /// Class describing a collection of space points within a chamber
    class MuonSpacePointBucket: public std::vector<MuonSpacePoint>{
        public:
            // Copy all vector constructors
            using std::vector<MuonSpacePoint>::vector;
            // Set the identifier of the MuonSpace point bucket
            void setBucketId(int id);
            /// Returns the identifier of the space point bucket
            int bucketId() const;
            /// Sorting of the buckets by bucket Id
            bool operator<(const MuonSpacePointBucket& other) const;
        private:
            int m_bucketId{0};


    };

    /// Container of space points.
    using MuonSpacePointContainer = std::vector<MuonSpacePointBucket>;

}