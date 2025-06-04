// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVISUALKALMAN_H
#define CVISUALKALMAN_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <os/CTimer.h>

class CVisualKalman
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const Eigen::Index STATE_SIZE = 13;
    static const Eigen::Index OBSERVATION_SIZE = 12;

    CVisualKalman() = default;
    CVisualKalman(const CVisualKalman&) = delete;
    CVisualKalman(CVisualKalman&&) = delete;
    CVisualKalman& operator=(const CVisualKalman&) = delete;
    CVisualKalman& operator=(CVisualKalman&&) = delete;
    ~CVisualKalman() = default;

    void init(const Eigen::MatrixXf& x);
    void predict();
    void update(const Eigen::MatrixXf& z);

    const Eigen::MatrixXf& state() { return m_x; }

private:
    Eigen::MatrixXf buildStateTransitionMatrix(float dt);
    Eigen::MatrixXf buildObservationMatrix(const Eigen::MatrixXf& z, float dt);

    Eigen::MatrixXf m_x = Eigen::MatrixXf::Zero(STATE_SIZE, 1);
    Eigen::MatrixXf m_P = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);

    Eigen::MatrixXf m_Q = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXf m_R = Eigen::MatrixXf::Identity(OBSERVATION_SIZE, OBSERVATION_SIZE);

    CTimer m_timer;
};

#endif // CVisualKalman_VisualSLAM_H
