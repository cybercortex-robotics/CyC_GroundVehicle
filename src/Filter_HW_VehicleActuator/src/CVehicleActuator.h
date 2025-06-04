// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVEHICLE_ACTUATOR_H_
#define CVEHICLE_ACTUATOR_H_

#include "CCycFilterBase.h"
#include "control/CModelVehicle.h"
#include "api/unity/CVehicleActuatorUnityApi.h"
#ifdef RASPBERRY_PI
#include "api/raspi/CVehicleActuatorRaspiApi.h"
#endif

class CVehicleActuator: public CCycFilterBase
{
public:
    enum VehicleActuatorApiType
    {
        VehicleActuatorApi_SIM = 0,
        VehicleActuatorApi_UNITY = 1,
        VehicleActuatorApi_MODEL_CAR = 2,
        VehicleActuatorApi_RASPI = 3,
        VehicleActuatorApi_SCOUT = 4
    };

public:
	explicit CVehicleActuator(CycDatablockKey key);
	explicit CVehicleActuator(const ConfigFilterParameters& params);

	~CVehicleActuator() override;

	bool enable() override;
	bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string&, const std::string&) override;

    static VehicleActuatorApiType StringToEnumType(const std::string& str_type);

private:
    CyC_INT         m_ApiType = 0;
    CyC_TIME_UNIT   m_lastTs = 0;
    CCycFilterBase* m_pControlFilter = nullptr;
    bool            m_bControlInputFound = false;

#ifndef RASPBERRY_PI
    CVehicleActuatorUnityApi m_UnityApi;
#endif

    CycControlInput m_OutputCmd;  // ackermann like control input (command) cmd.u[0]: acceleration, cmd.u[1]: steering angle

    // Terminal filter variables
    CyC_TIME_UNIT   m_lastTsTerminalVehicle = 0;
    CCycFilterBase* m_pTerminalVehicleFilter = nullptr;
    bool            m_bTerminalVehicleFound = false;

    CyC_INT         m_vehicleCommand = 0;

    CyC_TIME_UNIT   m_ControlWaitTime = 500;   // Amount of time waiting for a control input, until 0 (stop command) is sent

    // Scout variables
    std::string m_scoutDeviceName;
    int32_t     m_scoutBaudRate;
};

#endif /* CVEHICLE_ACTUATOR_H_ */
