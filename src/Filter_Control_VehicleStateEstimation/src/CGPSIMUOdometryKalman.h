// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CGPSIMUODOMETRYKALMAN_H
#define CGPSIMUODOMETRYKALMAN_H

#include <Eigen/Core>
#include <os/CTimer.h>

class CGPSIMUOdometryKalman
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static const Eigen::Index STATE_SIZE = 4;
    static const Eigen::Index OBSERVATION_SIZE = 7;
    static const Eigen::Index INPUT_SIZE = 3;

    CGPSIMUOdometryKalman() = default;
    CGPSIMUOdometryKalman(const CGPSIMUOdometryKalman&) = delete;
    CGPSIMUOdometryKalman(CGPSIMUOdometryKalman&&) = delete;
    CGPSIMUOdometryKalman& operator=(const CGPSIMUOdometryKalman&) = delete;
    CGPSIMUOdometryKalman& operator=(CGPSIMUOdometryKalman&&) = delete;
    ~CGPSIMUOdometryKalman() = default;

    void init(const Eigen::MatrixXf& x);
    void predict(Eigen::MatrixXf u);
    void update(Eigen::MatrixXf z, bool gpsAvailable);

    const Eigen::MatrixXf& state() { return m_x; }

private:
    Eigen::MatrixXf buildStateTransitionMatrix(float dt);
    Eigen::MatrixXf buildObservationMatrix(bool gpsAvailable);

    Eigen::MatrixXf m_x = Eigen::MatrixXf::Zero(STATE_SIZE, 1);
    Eigen::MatrixXf m_P = Eigen::MatrixXf::Identity(STATE_SIZE + INPUT_SIZE, STATE_SIZE + INPUT_SIZE);

    Eigen::MatrixXf m_delta_u = Eigen::MatrixXf::Zero(INPUT_SIZE, 1);

    CTimer m_timer;
    CTimer m_timer2;
};

#endif // CGPSIMUODOMETRYKALMAN_H
