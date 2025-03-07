// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVisualKalman.h"
#include <control/CKalmanOps.h>

//#define _DEBUG

#ifdef _DEBUG
#include <csv_writer.h>
#endif

void CVisualKalman::init(const Eigen::MatrixXf& x)
{
    //x = [r, p, y, dr, dp, dy, x, y, z, vx, vy, vz, scale]
    m_Q.diagonal() <<
        1.F, 1.F, 1.F,
        1.F, 1.F, 1.F,
        1.F, 1.F, 1.F,
        0.01F, 0.01F, 0.01F,
        0.1F;

    // z = [r_imu, p_imu, y_imu, ax, ay, az, dr_cam, dp_cam, dy_cam, dx_cam, dy_cam, dz_cam]
    m_R.diagonal() <<
        5.134e-01F, 2.79e-01F, 3.31e-01F,
        2.4969e-01F, 1.82e-01F, 4.81e-01F,
        2.449088F, 2.070458F, 2.850814F,
        0.204703F, 0.248347F, 0.29955F;

    assert(x.size() == STATE_SIZE);
    // start with scale at 1, should never be 0
    m_x(STATE_SIZE - 1, 0) = 1.F;

    m_timer.start();
}

void CVisualKalman::predict()
{
    const float dt = m_timer.elapsedMilliseconds() / 1000.F;
    m_timer.restart();

    const Eigen::MatrixXf F = buildStateTransitionMatrix(dt);
    CKalmanOps::transitionState(m_x, m_P, F, m_Q);

#ifdef _DEBUG
    static csv::writer w;
    if (!w.is_open())
    {
        w.open("state.csv");
        w.set_column_names("r", "p", "y", "dr", "dp", "dy", "x", "y", "z", "vx", "vy", "vz", "scale");
    }

    w.write_row(m_x(0, 0), m_x(1, 0), m_x(2, 0), m_x(3, 0), m_x(4, 0), m_x(5, 0), m_x(6, 0), m_x(7, 0), m_x(8, 0), m_x(9, 0), m_x(10, 0), m_x(11, 0), m_x(12, 0));
#endif
}

void CVisualKalman::update(const Eigen::MatrixXf& z)
{
    assert(z.cols() == OBSERVATION_SIZE);

#ifdef _DEBUG
    static csv::writer w_filtered, w_nonfiltered;
    if (!w_filtered.is_open())
    {
        w_filtered.open("data_filtered.csv");
        w_nonfiltered.open("data_nonfiltered.csv");

        w_filtered.set_column_names("r_imu", "p_imu", "y_imu", "ax", "ay", "az", "dr_cam", "dp_cam", "dy_cam", "dx_cam", "dy_cam", "dz_cam");
        w_nonfiltered.set_column_names("r_imu", "p_imu", "y_imu", "ax", "ay", "az", "dr_cam", "dp_cam", "dy_cam", "dx_cam", "dy_cam", "dz_cam");
    }
#endif

    const Eigen::MatrixXf H = buildObservationMatrix(z, 0.035F);
    if (CKalmanOps::updateStateWithFilter(m_x, m_P, z, H, m_R))
    {
#ifdef _DEBUG
        spdlog::info("Measurement has been used.");
        spdlog::info("{} {} {}", z(6, 0), z(7, 0), z(8, 0));
        w_filtered.write_row(z(0, 0), z(1, 0), z(2, 0), z(3, 0), z(4, 0), z(5, 0), z(6, 0), z(7, 0), z(8, 0), z(9, 0), z(10, 0), z(11, 0));
#endif
    }
    else
    {
#ifdef _DEBUG
        w_nonfiltered.write_row(z(0, 0), z(1, 0), z(2, 0), z(3, 0), z(4, 0), z(5, 0), z(6, 0), z(7, 0), z(8, 0), z(9, 0), z(10, 0), z(11, 0));
        spdlog::info("Measurement has been discarded.");
        spdlog::info("{} {} {}", z(6, 0), z(7, 0), z(8, 0));
#endif
    }
}

Eigen::MatrixXf CVisualKalman::buildStateTransitionMatrix(float dt)
{
    //x = [r, p, y, dr, dp, dy, x, y, z, vx, vy, vz, scale]

    //r = r + dr
    //p = p + dp
    //y = y + dy
    //dr = dr
    //dp = dp
    //dy = dy
    //x = x + vx * dt
    //y = y + vy * dt
    //z = z + vz * dt
    //vx = vx
    //vy = vy
    //vz = vz
    //scale = scale

    //F = [[1., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 1., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 1., 0., 0., dt, 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., dt, 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., dt, 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.]]

    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    F.block(0, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3);
    F.block(6, 9, 3, 3) = Eigen::MatrixXf::Constant(3, 3, dt);

    return F;
}

Eigen::MatrixXf CVisualKalman::buildObservationMatrix(const Eigen::MatrixXf& z, float dt)
{
    // z = [r_imu, p_imu, y_imu, ax, ay, az, dr_cam, dp_cam, dy_cam, dx_cam, dy_cam, dz_cam]
    // x = [r, p, y, dr, dp, dy, x, y, z, vx, vy, vz, scale]

    //r_imu = r
    //p_imu = p
    //y_imu = y
    //ax = vx / t
    //ay = vy / t
    //az = vz / t
    //dr_cam = dr
    //dp_cam = dp
    //dy_cam = dy
    //dx_cam = (1 / scale) * vx * dt
    //dy_cam = (1 / scale) * vy * dt
    //dz_cam = (1 / scale) * vz * dt

    //vx = x[9]
    //vy = x[10]
    //vz = x[11]
    //scale = x[12]
    //H = [[1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 1. / dt, 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1. / dt, 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1. / dt, 0.],
    //     [0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., (1. / scale) * dt, 0., 0., vx * dt],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., (1. / scale) * dt, 0., vy * dt],
    //     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., (1. / scale) * dt, vz * dt]]

    const float vx = m_x(9, 0);
    const float vy = m_x(10, 0);
    const float vz = m_x(11, 0);
    const float scale = m_x(12, 0);

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(OBSERVATION_SIZE, STATE_SIZE);
    H.block(0, 0, 3, 3) = Eigen::MatrixXf::Identity(3, 3);
    H.block(3, 9, 3, 3) = Eigen::MatrixXf::Constant(3, 3, 1.F / dt);
    H.block(6, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3);
    H.block(9, 9, 3, 3) = Eigen::MatrixXf::Constant(3, 3, 1.F / scale * dt);
    H.block(9, 12, 3, 1) << vx * dt, vy * dt, vz * dt;

    return H;
}


