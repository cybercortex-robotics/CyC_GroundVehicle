// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CControllerPurePursuite.h"

CControllerPurePursuite::CControllerPurePursuite(const CModelVehicle& _state_space_model,
    const float& _dt,
    const float& _target_speed,
    const float& _lookahead_dist,
    const float& _kp,
    const float& _k) :
    CBaseController(_state_space_model, _dt),
    m_TargetSpeed(_target_speed),
    m_Lfc(_lookahead_dist),
    m_Kp(_kp),
    m_k(_k)
{}

bool CControllerPurePursuite::update(const Eigen::VectorXf& _y)
{
    const CModelVehicle* pModel = static_cast<const CModelVehicle*>(this->getModel());

    // Set target speed to zero if the vehicle position is below 1m from target
    float Ed = std::abs(sqrtf(powf(m_r(0) - _y(0), 2) + powf(m_r(1) - _y(1), 2)));

    if (Ed < 1.f)
        m_TargetSpeed = 0.f;
    //else
    //    m_TargetSpeed = m_r[2];

    // Calculate velocity control input (r(2) - reference target speed in m/s) [m/s] = [km/h] / 3.6
    float throttle = m_Kp * (m_TargetSpeed - _y(2));
    throttle = std::clamp(throttle, -pModel->m_fMaxReverseSpeed, pModel->m_fMaxForwardSpeed);

    // Calculate steering angle
    float alpha = atan2f(m_r(1) - _y(1), m_r(0) - _y(0)) - _y(3);
    float Lf = m_k * _y(2) + m_Lfc;
    float steer = atan2f(2.f * pModel->m_fLongDistWheels * sinf(alpha) / Lf, 1.f);
    steer = std::clamp(steer, -pModel->m_fMaxSteeringAngleRad, pModel->m_fMaxSteeringAngleRad);
    
    m_u(0) = throttle;
    m_u(1) = steer;
    

    return true;
}
