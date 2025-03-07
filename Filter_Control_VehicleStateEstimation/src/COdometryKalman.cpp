// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "COdometryKalman.h"
#include <control/CKalmanOps.h>

void COdometryKalman::init(const Eigen::MatrixXf& x, float vehicleLength)
{
    m_fVehicleLength = vehicleLength;

    // x = [x, y, v, h, s, a]
    m_Q.diagonal() << 0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F;

    // z = [v, h, s, a]
    m_R.diagonal() << 0.5F, 0.9F, 0.5F, 0.5F;

    assert(x.size() == STATE_SIZE);
    m_x = x;

    m_timer.start();
}

void COdometryKalman::predict()
{
    // Compute elapsed time in a (more) precise way:
    // 1. Use microseconds, not milliseconds
    // 2. Use a second timer to compute the time it takes for the first 
    //    timer to restart, and add it to the final time used in Kalman
    m_timer2.restart();
    const float dt = m_timer.getElapsedTimeMicroseconds() * MSEC2SEC * MSEC2SEC;
    m_timer.restart();
    const float dt2 = m_timer2.getElapsedTimeMicroseconds() * MSEC2SEC * MSEC2SEC;

    const Eigen::MatrixXf F = buildStateTransitionMatrix(dt + dt2);

    // x = [x, y, v, h, s, a]

    // x = x + v * cos(h) * dt;
    // y = y + v * sin(h) * dt;
    // v = v + a * dt
    // h = h - tanf(steering) * v / m_VehicleModel.m_fVehicleLength * dt;
    // s = s
    // a = a

    float& x = m_x(0, 0);
    float& y = m_x(1, 0);
    float& v = m_x(2, 0);
    float& h = m_x(3, 0);
    const float& s = m_x(4, 0);
    const float& a = m_x(5, 0);

    x = x + v * cosf(h) * dt;
    y = y + v * sinf(h) * dt;
    v = v + a * dt;
    h = h - tanf(s) * v / m_fVehicleLength * dt;

    m_P = F * m_P * F.transpose() + m_Q;
}

void COdometryKalman::update(const Eigen::MatrixXf& z)
{
    const Eigen::MatrixXf H = buildObservationMatrix();
    CKalmanOps::updateStateWithoutFilter(m_x, m_P, z, H , m_R);
}

Eigen::MatrixXf COdometryKalman::buildStateTransitionMatrix(float dt)
{
    // x = [x, y, v, h, s, a]

    // x = x + v * cos(h) * dt;
    // y = y + v * sin(h) * dt;
    // v = v + a * dt
    // h = h - tanf(steering) * v / m_VehicleModel.m_fVehicleLength * dt;
    // s = s
    // a = a

    //
    // [[1, 0, cos(h)*dt, -sin(h)*v*dt, 0, 0],
    //  [0, 1, sin(h)*dt,  cos(h)*v*dt, 0, 0],
    //  [0, 0, 1, 0, 0, dt],
    //  [0, 0, -tan(s)*t/L, 1, -sec^2(s)*v*dt/L, 0],
    //  [0, 0, 0, 0, 1, 0],
    //  [0, 0, 0, 0, 0, 1]]
    //
    // sec(a) = 1/cos(a)
    //

    const float x = m_x(0);
    const float y = m_x(1);
    const float v = m_x(2);
    const float h = m_x(3);
    const float s = m_x(4);
    const float a = m_x(5);

    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    F(0, 2) = cosf(h) * dt;
    F(0, 3) = -sinf(h) * v * dt;
    F(1, 2) = sinf(h) * dt;
    F(1, 3) = cosf(h) * v * dt;
    F(2, 5) = dt;
    F(3, 2) = -tanf(s) * dt / m_fVehicleLength;
    F(3, 4) = powf(1.F / cosf(s), 2.F) * v * dt / m_fVehicleLength;

    return F;
}

Eigen::MatrixXf COdometryKalman::buildObservationMatrix()
{
    // x = [x, y, v, h, s, a]
    // z = [v, h, s, a]

    // 
    // [[0, 0, 1, 0, 0, 0],
    //  [0, 0, 0, 1, 0, 0],
    //  [0, 0, 0, 0, 1, 0],
    //  [0, 0, 0, 0, 0, 1]]
    //

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(OBSERVATION_SIZE, STATE_SIZE);
    H.block(0, 2, 4, 4) = Eigen::MatrixXf::Identity(4, 4);

    return H;
}
