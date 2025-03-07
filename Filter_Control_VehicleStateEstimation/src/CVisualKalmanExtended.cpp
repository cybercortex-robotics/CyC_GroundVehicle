// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVisualKalmanExtended.h"
#include <control/CKalmanOps.h>

void CVisualKalmanExtended::init(const Eigen::MatrixXf& x)
{
    //x = [x, y, h, dx, dy, dyaw]
    m_Q.diagonal() <<
        1.F, 1.F, 1.F,
        1.F, 1.F, 1.F;

    // z = [x, y, h, dx, dy, dyaw]
    m_R.diagonal() <<
        5.134e-01F, 2.79e-01F, 3.31e-01F,
        2.4969e-01F, 1.82e-01F, 4.81e-01F;

    assert(x.size() == STATE_SIZE);
}

void CVisualKalmanExtended::predict()
{
    const Eigen::MatrixXf F = buildStateTransitionMatrix();
    CKalmanOps::transitionState(m_x, m_P, F, m_Q);
}

void CVisualKalmanExtended::update(const Eigen::MatrixXf& z)
{
    assert(z.cols() == OBSERVATION_SIZE);

    const Eigen::MatrixXf H = buildObservationMatrix();
    if (CKalmanOps::updateStateWithFilter(m_x, m_P, z, H, m_R))
    {
    }
    else
    {
    }
}

Eigen::MatrixXf CVisualKalmanExtended::buildStateTransitionMatrix()
{
    //x = [x, y, h, dx, dy, dyaw]

    //x = x + dx
    //y = y + dy
    //h = h + dyaw
    //dx = dx
    //dy = dy
    //dyaw = dyaw

    //F = [[1., 0., 0., 1., 0., 0.],
    //     [0., 1., 0., 0., 1., 0.],
    //     [0., 0., 1., 0., 0., 1.],
    //     [0., 0., 0., 1., 0., 0.],
    //     [0., 0., 0., 0., 1., 0.],
    //     [0., 0., 0., 0., 0., 1.]]

    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(STATE_SIZE, STATE_SIZE);
    F.block(0, 3, 3, 3) = Eigen::MatrixXf::Identity(3, 3);

    return F;
}

Eigen::MatrixXf CVisualKalmanExtended::buildObservationMatrix()
{
    return Eigen::MatrixXf::Identity(OBSERVATION_SIZE, OBSERVATION_SIZE);
}


