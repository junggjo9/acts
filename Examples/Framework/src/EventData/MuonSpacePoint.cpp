// This file is part of the Acts project.
//
// Copyright (C) 2024 CERN for the benefit of the Acts project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <ActsExamples/EventData/MuonSpacePoint.hpp>


namespace ActsExamples {
    MuonSpacePoint::MuonSpacePoint(Vector3&& pos, CovMat&& covariance, Scalar driftRadius,
                                    int stationName, int stationEta, int stationPhi,
                                    int gasGap, int primaryChannel, int secondaryChannel,
                                    bool measuresEta, bool measuresPhi):
            m_pos{std::move(pos)},
            m_cov{std::move(covariance)},
            m_driftR{driftRadius},
            m_stationName{stationName},
            m_stationEta{stationEta},
            m_stationPhi{stationPhi},
            m_gasGap{gasGap},
            m_primCh{primaryChannel},
            m_secondCh{secondaryChannel},
            m_measEta{measuresEta},
            m_measPhi{measuresPhi}{}

    const MuonSpacePoint::Vector3& MuonSpacePoint::position() const{
        return m_pos;
    }
    void MuonSpacePoint::setPosition(Vector3&& pos){
        m_pos = std::move(pos);
    }
    const MuonSpacePoint::CovMat& MuonSpacePoint::covariance() const{
        return m_cov;
    }
    void MuonSpacePoint::setCovariance(CovMat&& cov) {
        m_cov = std::move(cov);
    }
    int MuonSpacePoint::stationName() const{
        return m_stationName;
    }
    void MuonSpacePoint::setStationName(int stName) {
        m_stationName = stName;
    }
    int MuonSpacePoint::stationEta() const{
        return m_stationEta;
    }
    void MuonSpacePoint::setStationEta(int stEta){
        m_stationEta = stEta;
    }
    int MuonSpacePoint::stationPhi() const{
        return m_stationPhi;
    }
    void MuonSpacePoint::setStationPhi(int stPhi){
        m_stationPhi = stPhi;
    }

    int MuonSpacePoint::gasGap() const{
        return m_gasGap;
    }
    void MuonSpacePoint::setGasGap(int gap) {
        m_gasGap = gap;
    }
    int MuonSpacePoint::primaryChannel() const{
        return m_primCh;
    }
    void MuonSpacePoint::setPrimaryChannel(int channel) {
        m_primCh = channel;
    }
    int MuonSpacePoint::secondaryChannel() const{
        return m_secondCh;
    }
    void MuonSpacePoint::setSecondaryChannel(int channel) {
        m_secondCh = channel;
    }
    bool MuonSpacePoint::measuresEta() const{
        return m_measEta;
    }
    void MuonSpacePoint::setMeasuresEta(bool measEta){
        m_measEta = measEta;
    }
    bool MuonSpacePoint::measuresPhi() const{
        return m_measPhi;
    }
    void MuonSpacePoint::setMeasuresPhi(bool measPhi){
        m_measPhi = measPhi;
    }
    bool MuonSpacePoint::operator==(const MuonSpacePoint& other) const {
        return ! ((*this) < other  || other < (*this));
    }
    bool MuonSpacePoint::operator<(const MuonSpacePoint& other) const {
        if (stationName() != other.stationName()){
            return stationName() < other.stationName();
        }
        if (stationEta() != other.stationEta()) {
            return stationEta() < other.stationEta();
        }
        if (stationPhi() != other.stationPhi()) {
            return stationPhi() < other.stationPhi();
        }
        if (gasGap() != other.gasGap()) {
            return gasGap() < other.gasGap();
        }
        if (primaryChannel() != other.primaryChannel()) {
            return primaryChannel() < other.primaryChannel();
        }
        if (secondaryChannel() != other.secondaryChannel()) {
            return secondaryChannel() < other.secondaryChannel();
        }
        if (measuresEta() != other.measuresEta()) {
            return measuresEta();
        }
        return measuresPhi() < other.measuresPhi();
    }
    int MuonSpacePointBucket::bucketId() const{
        return m_bucketId;
    }
    bool MuonSpacePointBucket::operator<(const MuonSpacePointBucket& other) const {
        return bucketId() < other.bucketId();
    }
}