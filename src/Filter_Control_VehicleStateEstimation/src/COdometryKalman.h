// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CODOMETRYKALMAN_H
#define CODOMETRYKALMAN_H

#include <Eigen/Core>
#include <os/CTimer.h>

class COdometryKalman
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const Eigen::Index STATE_SIZE = 6;
    static const Eigen::Index OBSERVATION_SIZE = 4;

    COdometryKalman() = default;
    COdometryKalman(const COdometryKalman&) = delete;
    COdometryKalman(COdometryKalman&&) = delete;
    COdometryKalman& operator=(const COdometryKalman&) = delete;
    COdometryKalman& operator=(COdometryKalman&&) = delete;
    ~COdometryKalman() = default;

    void init(const Eigen::MatrixXf& x, float vehicleLength);
    void predict();
    void update(const Eigen::MatrixXf& z);

    const Eigen::MatrixXf& state() { return m_x; }

private:
    Eigen::MatrixXf buildStateTransitionMatrix(float dt);
    Eigen::MatrixXf buildObservationMatrix();

    float m_fVehicleLength = 1.F;

    Eigen::MatrixXf m_x = Eigen::MatrixXf::Zero(STATE_SIZE, 1);
    Eigen::MatrixXf m_P = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);

    Eigen::MatrixXf m_Q = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXf m_R = Eigen::MatrixXf::Identity(OBSERVATION_SIZE, OBSERVATION_SIZE);

    CTimer m_timer;
    CTimer m_timer2;
};

#endif // CODOMETRYKALMAN_H
