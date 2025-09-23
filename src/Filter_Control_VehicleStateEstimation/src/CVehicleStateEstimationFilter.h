// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// Inputs: Visual SLAM, IMU, wheels encoder

#ifndef CVehicleStateEstimationFilter_H_
#define CVehicleStateEstimationFilter_H_

#include "CyC_TYPES.h"
#include <CCycFilterBase.h>
#include "CVehicleStateEstimation.h"
#include <os/CCsvReader.h>
#include "api/unity/CStateEstimationUnityApi.h"
#include "CVisualKalman.h"

class CVehicleStateEstimationFilter : public CCycFilterBase
{
public:
    enum VehStateEstApiType
    {
        VehicleStateEstimationApi_SIM   = 0,    // Copied from a vehicle simulation filter
        VehicleStateEstimationApi_UNITY = 1,    // Read from unity
        VehicleStateEstimationApi_REAL  = 2     // Calculated with own state estimation algorithm
    };

public:
	explicit CVehicleStateEstimationFilter(CycDatablockKey key);
	explicit CVehicleStateEstimationFilter(const ConfigFilterParameters& params);

	~CVehicleStateEstimationFilter() override;

	bool enable() override;
	bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

    static VehStateEstApiType StringToEnumType(const std::string& str_type);

private:
    VehStateEstApiType m_InterfaceType = VehicleStateEstimationApi_SIM;

    CycState    m_InitialState;
    CycState    m_FilterOutput;
    
    // Vehicle state estimation algorithm
    std::unique_ptr<CVehicleStateEstimation> m_pVehicleStateEstimation;

    // Unity simulation: just copies the simulated state in unity to vehicle state estimation
    CVehStateEstUnityApi m_UnityApi;

    bool m_bStateInitialized = false;

    // Vehicle simulation filter: just copies the simulated state to vehicle state estimation
    CCycFilterBase* m_pInputFilterVehicleSim = nullptr;
    CyC_TIME_UNIT   m_lastTsVehicleSim = 0;

    // Actuator filter: can be either a control or an actuator filter
    CCycFilterBase* m_pInputFilterActuators = nullptr;
    CyC_TIME_UNIT   m_lastTsActuators = 0;
    bool        m_bUseActuators = false;

    // Rotary wheels encoder
    CCycFilterBase* m_pInputFilterWheelsEncoder = nullptr;
    CyC_TIME_UNIT   m_lastTsWheelsEncoder = 0;
    bool        m_bUseWheelsEncoder = false;

    // Inertial sensor
    CCycFilterBase* m_pInputFilterIMU = nullptr;
    CyC_TIME_UNIT   m_lastTsIMU = 0;
    bool        m_bUseIMU = false;

    // Compass sensor
    CCycFilterBase*        m_pInputFilterCompass = nullptr;
    CyC_TIME_UNIT          m_lastTsCompass = 0;
    bool               m_bUseCompass = false;
    std::vector<float> m_initialHeadingCompass{ 0.F };

    // GPS sensor
    CCycFilterBase* m_pInputFilterGPS = nullptr;
    CyC_TIME_UNIT   m_lastTsGPS = 0;
    bool        m_bUseGPS = false;

    // Visual slam
    CCycFilterBase* m_pInputFilterVisualSlam = nullptr;
    CyC_TIME_UNIT   m_lastTsVisualSlam = 0;
    bool        m_bUseVisualSlam = false;
    CVisualKalman*  m_pVisualKalman = nullptr;

    // Mission planner as origin provider
    CCycFilterBase* m_pMissionPlanner = nullptr;
    bool        m_bUseMissionPlanner = false;
};

#endif /* CVehicleStateEstimationFilter_H_ */
