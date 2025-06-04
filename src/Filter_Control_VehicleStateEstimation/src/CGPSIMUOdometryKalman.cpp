// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CGPSIMUOdometryKalman.h"
#include <control/CKalmanOps.h>

// Get gravity value based on
// geographical coordinates
float gravity(float lat, float alt)
{
    const auto lambda = PI / 180.F * lat;
    const auto gamma = 9.780327 * (1.F + 0.0053024 * pow(sin(lambda), 2) - 0.0000058 * pow(sin(2.F * lambda), 2));
    const auto g = gamma - ((3.0877e-6) - (0.004e-6) * pow(sin(lambda), 2)) * alt + (0.072e-12) * pow(alt, 2);

    return g;
}

void CGPSIMUOdometryKalman::init(const Eigen::MatrixXf& x)
{
    assert(x.size() == STATE_SIZE);
    m_x = x;
    m_P *= 100.F;

    m_timer.start();
}

void CGPSIMUOdometryKalman::predict(Eigen::MatrixXf u)
{
    // Compute elapsed time in a (more) precise way:
    // 1. Use microseconds, not milliseconds
    // 2. Use a second timer to compute the time it takes for the first
    //    timer to restart, and add it to the final time used in Kalman
    m_timer2.restart();
    const float dt = m_timer.getElapsedTimeMicroseconds() * MSEC2SEC * MSEC2SEC;
    m_timer.restart();
    const float dt2 = m_timer2.getElapsedTimeMicroseconds() * MSEC2SEC * MSEC2SEC;

    const auto delta_t = dt + dt2;
    const Eigen::MatrixXf F = buildStateTransitionMatrix(dt + dt2);

    // Correct input with estimated biases
    //u += m_delta_u;

    // x = [x, y, v, h]

    // x = x + v * cos(h) * dt + 0.5*ax*dt^2;
    // y = y + v * sin(h) * dt + 0.5*ay*dt^2;
    // v = v + ax * dt;
    // h = h + rz * dt;
    // ax = ax;
    // ay = ay;

    float& x = m_x(0, 0);
    float& y = m_x(1, 0);
    float& v = m_x(2, 0);
    float& h = m_x(3, 0);
    const float& ax = u(0, 0);
    const float& ay = u(1, 0);
    const float& rz = u(2, 0);

    x = x + v * cosf(h) * delta_t;// +0.5F * ax * delta_t * delta_t;
    y = y + v * sinf(h) * delta_t;// +0.5F * ay * delta_t * delta_t;
    v = v;// +ax * delta_t;
    h = h;// +rz * delta_t;

    // x = [x, y, v, h]
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(STATE_SIZE + INPUT_SIZE, STATE_SIZE + INPUT_SIZE);
    Q.diagonal() << 0.1F, 0.1F, 0.1F, 0.375F, 0.01F, 0.01F, 0.01F;
    Q *= delta_t;

    m_P = F * m_P * F.transpose() + Q;
}

void CGPSIMUOdometryKalman::update(Eigen::MatrixXf z_full, bool gpsAvailable)
{
    // gpsAvailable == True: z = [x, y, v, h, ax, ay, rz]
    // gpsAvailable == False: z = [0, 0, v, h, ax, ay, rz]

    const Eigen::MatrixXf H = buildObservationMatrix(gpsAvailable);

    // z = [x, y, v, h, ax, ay, rz]
    Eigen::MatrixXf R_full = Eigen::MatrixXf::Identity(OBSERVATION_SIZE, OBSERVATION_SIZE);
    R_full.diagonal() << 6.42F, 6.42F, 0.1F, 0.375F, 0.00325F, 0.00325F, 0.045F;

    // Remove the first two measurements if gps is not available
    Eigen::MatrixXf R = gpsAvailable
        ? R_full
        : R_full.bottomRightCorner(R_full.rows() - 2, R_full.cols() - 2);
    // Remove the first two measurements if gps is not available
    Eigen::MatrixXf z = gpsAvailable
        ? z_full
        : z_full.bottomRows(z_full.rows() - 2);

    // Kalman gain
    const Eigen::MatrixXf S = H * m_P * H.transpose() + R;

    const Eigen::MatrixXf cho_b = m_P * H.transpose();
    const auto cho_factor = S.ldlt();
    const Eigen::MatrixXf K = cho_factor.solve(cho_b.transpose()).transpose();

    Eigen::MatrixXf delta_y = Eigen::MatrixXf::Zero(STATE_SIZE + INPUT_SIZE, 1);
    delta_y.bottomRows(delta_y.rows() - STATE_SIZE) = m_delta_u;

    const Eigen::MatrixXf y = z - H.block(0, 0, H.rows(), STATE_SIZE) * m_x;
    const Eigen::MatrixXf i = (K * y) + delta_y; // innovation

    m_x = m_x + i.topRows(STATE_SIZE);
    m_P = m_P - K * S * K.transpose();

    m_delta_u = i.bottomRows(delta_y.rows() - STATE_SIZE);
}

Eigen::MatrixXf CGPSIMUOdometryKalman::buildStateTransitionMatrix(float dt)
{
    // x = [x, y, v, h]

    // x = x + v * cos(h) * dt;
    // y = y + v * sin(h) * dt;
    // v = v;
    // h = h;

    //
    // [[1, 0, cos(h)*dt, -sin(h)*v*dt],
    //  [0, 1, sin(h)*dt,  cos(h)*v*dt],
    //  [0, 0, 1, 0],
    //  [0, 0, 0, 1],
    //

    const float x = m_x(0);
    const float y = m_x(1);
    const float v = m_x(2);
    const float h = m_x(3);

    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(STATE_SIZE + INPUT_SIZE, STATE_SIZE + INPUT_SIZE);
    F(0, 2) = cosf(h) * dt;
    F(0, 3) = -sinf(h) * v * dt;

    F(1, 2) = sinf(h) * dt;
    F(1, 3) = cosf(h) * v * dt;

    return F;
}

Eigen::MatrixXf CGPSIMUOdometryKalman::buildObservationMatrix(bool gpsAvailable)
{
    // gpsAvailable == True: z = [x, y, v, h, ax, ay, rz]
    // gpsAvailable == False: z = [0, 0, v, h, ax, ay, rz]

    Eigen::MatrixXf H = Eigen::MatrixXf::Identity(OBSERVATION_SIZE, STATE_SIZE + INPUT_SIZE);

    // Remove the first two measurements if the gps is not available
    return gpsAvailable
        ? H
        : H.bottomRightCorner(H.rows() - 2, H.cols());
}
