// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CControllerPurePursuite_H_
#define CControllerPurePursuite_H_

#include "CyC_TYPES.h"
#include "control/CBaseController.h"
#include "control/CModelVehicle.h"

class qq
{
public:
    int a = 2;
};

class CControllerPurePursuite : public CBaseController
{
public:
    CControllerPurePursuite(const CModelVehicle& _state_space_model, 
        const float& _dt, 
        const float& _target_speed = 4.f,
        const float& _lookahead_dist = 2.7f,
        const float& _kp = 0.2f,
        const float& _k = 0.0001f);
    CControllerPurePursuite(const CControllerPurePursuite&) = default;
    CControllerPurePursuite(CControllerPurePursuite&&) = default;
    CControllerPurePursuite& operator=(const CControllerPurePursuite&) = default;
    CControllerPurePursuite& operator=(CControllerPurePursuite&&) = default;
    ~CControllerPurePursuite() = default;

    virtual bool update(const Eigen::VectorXf& _y);

private:
    // Pure pursuite controller parameters
    float   m_TargetSpeed;    // Desired vehicle speed in [m/s] ([m/s] = [km/h] / 3.6)
    float   m_Lfc;            // look-ahead distance
    float   m_Kp;             // speed proportional gain
    float   m_k;              // look forward gain
};

#endif /* CControllerPurePursuite_H_ */
