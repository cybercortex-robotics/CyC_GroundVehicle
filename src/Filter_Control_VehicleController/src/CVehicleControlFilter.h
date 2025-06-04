// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVEHICLECONTROLFILTER_H_
#define CVEHICLECONTROLFILTER_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include <string>
#include <csv_reader.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "os/CTimer.h"
#include "math/CMathUtils.h"
#include "control/CModelVehicle.h"
#include "CControllerPurePursuite.h"
#include "plan/CDwa.h"

class CVehicleControlFilter: public CCycFilterBase
{
    enum ControllerType
    {
        CONTROL_MANUAL = 0,
        CONTROL_PURE_PURSUITE = 1,
        CONTROL_MPC = 2,
        CONTROL_LQR = 3,
        CONTROL_SKID = 4,
        CONTROL_DWA = 5
    };

public:
	explicit CVehicleControlFilter(CycDatablockKey key);
	explicit CVehicleControlFilter(const ConfigFilterParameters& params);
	~CVehicleControlFilter() override;

    bool enable() override;
	bool disable() override;

private:
	bool    process() override;
    void    loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

    CyC_INT StringToEnumType(const std::string& str_ctrl_name);
    void    manual_ctrl_thread();

private:
    // Control variables
    CyC_INT                 m_ControllerType = CONTROL_MANUAL;
    CycState                m_VehicleState;
    CycReferenceSetPoints   m_VehicleReferenceSetPoints;
    CycControlInput         m_OutputCmd; // Control input command (velocity u[0] and steering u[1])

    // Input filters
    CycInputSource*	m_pFilterVehicleState = nullptr;
    CyC_TIME_UNIT   m_lastReadTSState = 0;
    CycInputSource*	m_pFilterPathPlanner = nullptr;
    CyC_TIME_UNIT   m_lastReadTSPathPlanner = 0;

    // DWA requirements
    CycInputSource*	                m_pFilterMissionPlanner = nullptr;
    CyC_TIME_UNIT                   m_lastReadTSMissionPlan = 0;
    CycInputSource*                 m_pFilterMap = nullptr;
    CyC_TIME_UNIT                   m_lastReadTSMap = 0;
    std::vector<Eigen::Vector4f>    m_MissionPlan;
    CycEnvironment                  m_EnvironmentSnapshot = CycEnvironment(1.f);
    std::vector<CyC_INT>            m_TraversableClassIDs;
    
    // Vehicle model
    std::unique_ptr<CModelVehicle> m_pVehicleModel;

    // Manual controller
    std::thread m_ManualCtrlThread;

    // Controllers
    std::unique_ptr<CControllerPurePursuite>    m_pControllerPurePursuite;
    std::unique_ptr<CDwa>                       m_pControllerDwa;

    float m_RefSpeed = 0.f;
};

#endif /* CVEHICLECONTROLFILTER_H_ */
