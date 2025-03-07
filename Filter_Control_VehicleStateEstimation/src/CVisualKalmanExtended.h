// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVISUALKALMANEXTENDED_H
#define CVISUALKALMANEXTENDED_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <os/CTimer.h>

class CVisualKalmanExtended
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const Eigen::Index STATE_SIZE = 6;
    static const Eigen::Index OBSERVATION_SIZE = 6;

    CVisualKalmanExtended() = default;
    CVisualKalmanExtended(const CVisualKalmanExtended&) = delete;
    CVisualKalmanExtended(CVisualKalmanExtended&&) = delete;
    CVisualKalmanExtended& operator=(const CVisualKalmanExtended&) = delete;
    CVisualKalmanExtended& operator=(CVisualKalmanExtended&&) = delete;
    ~CVisualKalmanExtended() = default;

    void init(const Eigen::MatrixXf& x);
    void predict();
    void update(const Eigen::MatrixXf& z);

    const Eigen::MatrixXf& state() { return m_x; }

private:
    Eigen::MatrixXf buildStateTransitionMatrix();
    Eigen::MatrixXf buildObservationMatrix();

    Eigen::MatrixXf m_x = Eigen::MatrixXf::Zero(STATE_SIZE, 1);
    Eigen::MatrixXf m_P = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);

    Eigen::MatrixXf m_Q = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXf m_R = Eigen::MatrixXf::Identity(OBSERVATION_SIZE, OBSERVATION_SIZE);
};

#endif // CVISUALKALMANEXTENDED_H
