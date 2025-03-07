// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef _MPC_WRAPPER_H_
#define _MPC_WRAPPER_H_
#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "CControllerBase.h"

class CMPCWrapper : public CControllerBase
{
private:
    CycControlInput previous_input;
    VehicleModel m_VehicleModel;

public:
    ~CMPCWrapper() 
    {
    };

    CMPCWrapper(float dt) :
        CMPCWrapper(dt, VehicleModel(0.25f))
    {};

    CMPCWrapper(float dt, VehicleModel vehicleModel) :
        CControllerBase(dt),
        m_VehicleModel(vehicleModel)
    {
        previous_input.u = Eigen::VectorXf::Zero(2);
    };
    void Initialize(float ref_speed);
    void SetReferenceVelocity(float v_ref);
    
    CycState GetState();

    bool           Solve(const CycState& x0, const Eigen::VectorXf& poly);
    CycControlInput    GetControlSolution();
    Eigen::VectorXf      SolveAndCommand(const CycReferenceSetPoints& ref_points, const CycState& vehicle_state);
};
#endif //_MPC_WRAPPER_H_
