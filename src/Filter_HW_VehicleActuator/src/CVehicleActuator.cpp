// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleActuator.h"
#include "../../Filter_Terminal_Vehicle/src/CTerminalVehicle.h"

#ifdef Build_Robot_AgileScout
#include <CAgileScout.h>
#endif

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CVehicleActuator(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CVehicleActuator(_params);
}

CVehicleActuator::CVehicleActuator(CycDatablockKey key) :
	CCycFilterBase(key)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_VEHICLE_ACTUATOR_FILTER_TYPE");
    m_OutputDataType = CyC_CONTROL_INPUT;
}

CVehicleActuator::CVehicleActuator(const ConfigFilterParameters& params) :
    CCycFilterBase(params)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_VEHICLE_ACTUATOR_FILTER_TYPE");
    m_OutputDataType = CyC_CONTROL_INPUT;

    m_scoutBaudRate = 115200; // default
    if (params.CustomParameters.find("device_name") != params.CustomParameters.end())
        m_scoutDeviceName = params.CustomParameters.at("device_name");
    if (params.CustomParameters.find("baud_rate") != params.CustomParameters.end())
        m_scoutBaudRate = std::stoi(params.CustomParameters.at("baud_rate"));
}

CVehicleActuator::~CVehicleActuator()
{
	if (m_bIsEnabled)
		disable();

#ifdef Build_Robot_AgileScout
    if (CAgileScout::instance().isConnected())
    {
        CAgileScout::instance().setMoveCommand(0.F, 0.F);
    }
#endif // Build_Robot_AgileScout
}

bool CVehicleActuator::enable()
{
    if (m_CustomParameters.find("interface") == m_CustomParameters.end())
    {
        spdlog::warn("Filter [{}-{}]: CVehicleActuator: interface parameter not found in configuration file. Simulation will be used.", getFilterKey().nCoreID, getFilterKey().nFilterID);
        m_ApiType = VehicleActuatorApi_SIM;
    }

    for (const CycInputSource& src : getInputSources())
    {
        if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_TERMINAL_VEHICLE_FILTER_TYPE"))
        {
            m_pTerminalVehicleFilter = src.pCycFilter;
            m_bTerminalVehicleFound = true;
        }
        else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_CONTROL_FILTER_TYPE"))
        {
            m_pControlFilter = src.pCycFilter;
            m_bControlInputFound = true;
        }
        else
        {
            spdlog::error("Filter [{}-{}]: ERROR CVehicleActuators: Expected CyC_VEHICLE_CONTROL_FILTER_TYPE as input", getFilterKey().nCoreID, getFilterKey().nFilterID);
        }
    }

    /*if (!m_bControlInputFound)
    {
        spdlog::warn("Filter [{}-{}]: CVehicleActuators: No control input filter given. Actuation commands will be zero", getFilterKey().nCoreID, getFilterKey().nFilterID);
        m_bIsEnabled = false;
        return m_bIsEnabled;
    }

    if (!m_bTerminalVehicleFound)
    {
        spdlog::error("Filter[{}-{}]: ERROR CVehicleActuator: Expected CyC_TERMINAL_VEHICLE_FILTER_TYPE as input.", getFilterKey().nCoreID, getFilterKey().nFilterID);
        m_bIsEnabled = false;
        return m_bIsEnabled;
    }*/

    m_ApiType = StringToEnumType(m_CustomParameters["interface"]);

    if (m_ApiType == VehicleActuatorApi_UNITY)
    {
        if (m_CustomParameters.find("ip") == m_CustomParameters.end() || m_CustomParameters.find("port") == m_CustomParameters.end())
        {
            spdlog::error("Filter [{}-{}]: CVehicleActuator: ip and/or port parameters not found in configuration file", getFilterKey().nCoreID, getFilterKey().nFilterID);
            return false;
        }
#ifndef RASPBERRY_PI
        m_bIsEnabled = m_UnityApi.enable(m_CustomParameters["ip"], m_CustomParameters["port"]);
#endif
    }
    else if(m_ApiType == VehicleActuatorApi_RASPI)
    {
#ifdef RASPBERRY_PI
        //m_bIsEnabled = m_RasPiInterface.enable();
        // The enabling is done in the singleton constructor
        m_bIsEnabled = true;
        spdlog::info("Filter [{}-{}]: CVehicleActuators::enable() RasPi Interface enabled 2", getFilterKey().nCoreID, getFilterKey().nFilterID);
#endif
    }
    else if (m_ApiType == VehicleActuatorApi_SCOUT)
    {
#ifdef Build_Robot_AgileScout
        if (!m_scoutDeviceName.empty() && (m_scoutBaudRate != 0))
        {
            CAgileScout::instance().connect(m_scoutDeviceName, m_scoutBaudRate);
            auto state = CAgileScout::instance().getState();
            log_info("Battery: {}", state.battery_voltage);
            m_bIsEnabled = true;
        }
        else
        {
            spdlog::error("Filter[{}-{}]: Device name or baud rate not set for Agile Scout robot.", getFilterKey().nCoreID, getFilterKey().nFilterID);
        }
#endif // Build_Robot_AgileScout
    }
    else
    {
        m_bIsEnabled = true;
    }

    // Initial actuator output command
    m_OutputCmd.u = Eigen::VectorXf::Zero(ModelVehicle_NumInputs);

    if (m_bIsEnabled)
        spdlog::info("Filter [{}-{}]: CVehicleActuators::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);
    else
        spdlog::error("Filter [{}-{}]: CVehicleActuator::enable() failed", getFilterKey().nCoreID, getFilterKey().nFilterID);
    
	return m_bIsEnabled;
}


bool CVehicleActuator::disable()
{	
	if (isRunning())
        stop();

	m_bIsEnabled = false;
	return true;
}

float low_pass_filter(float old_value, float new_value, float c)
{
    return (c * new_value) + ((1.F - c) * old_value);
}

bool CVehicleActuator::process()
{
    bool bReturn(false);
    
    CyC_TIME_UNIT ts = 0;
    if (m_pControlFilter != nullptr)
        ts = m_pControlFilter->getTimestampStop();

    CycTerminalCommand term_cmd;
    if (m_pTerminalVehicleFilter != nullptr)
        m_pTerminalVehicleFilter->getData(term_cmd);
    m_vehicleCommand = term_cmd.cmd.size() ? term_cmd.cmd[0] : CTerminalVehicle::Vehicle_STOP;

#ifndef RASPBERRY_PI
    if (m_ApiType == VehicleActuatorApi_UNITY)
    {
        m_UnityApi.process();
    }
#endif

    // Send stop signal if a control signals has not been received for a period longer than m_ControlWaitTime
    if ((CTimer::now() - m_lastTs) > m_ControlWaitTime)
        m_OutputCmd.u(0) = 0.f;
    
    // Iterate over vehicle commands (stop and drive)
    switch (m_vehicleCommand)
    {
        case CTerminalVehicle::Vehicle_DRIVE:
        {
            if (m_ApiType == VehicleActuatorApi_SIM)
            {
                if (ts > m_lastTs)
                {
                    m_lastTs = ts;
                    if (m_pControlFilter->getData(m_OutputCmd))
                    {
                        bReturn = true;
                    }
                }
            }
            else if (m_ApiType == VehicleActuatorApi_UNITY)
            {
#ifndef RASPBERRY_PI
                if (ts > m_lastTs)
                {
                    m_lastTs = ts;
                    if (m_pControlFilter->getData(m_OutputCmd))
                    {
                        m_UnityApi.send_control(m_OutputCmd);
                        bReturn = true;
                    }
                }
#endif
                bReturn = true;
            }
            else if (m_ApiType == VehicleActuatorApi_RASPI)
            {
                if (ts > m_lastTs)
                {
                    m_lastTs = ts;
                    if (m_pControlFilter->getData(m_OutputCmd))
                    {
#ifdef RASPBERRY_PI
                        if (CVehicleActuatorRaspiInterface::getInstance().isInitialized())
                        {
                            CVehicleActuatorRaspiInterface::getInstance().send_control(cmd);
                        }
#endif
                        bReturn = true;
                    }
                }
            }
            else if (m_ApiType == VehicleActuatorApi_SCOUT)
            {
                if (ts > m_lastTs)
                {
                    m_lastTs = ts;
                    if (m_pControlFilter->getData(m_OutputCmd))
                    {
#ifdef Build_Robot_AgileScout
                        if (CAgileScout::instance().isConnected())
                        {
                            CAgileScout::instance().setMoveCommand(m_OutputCmd.u[0], m_OutputCmd.u[1]);
                        }
#endif // Build_Robot_AgileScout
                        bReturn = true;
                    }
                }
            }
        }
        break;
        case CTerminalVehicle::Vehicle_STOP:
        default:
        {
            m_OutputCmd.u.setZero();
            if (m_ApiType == VehicleActuatorApi_SIM)
            {
                bReturn = true;
            }
            else if (m_ApiType == VehicleActuatorApi_UNITY)
            {
#ifndef RASPBERRY_PI
                m_UnityApi.send_control(m_OutputCmd);
#endif
                bReturn = true;
            }
            else if (m_ApiType == VehicleActuatorApi_SCOUT)
            {
#ifdef Build_Robot_AgileScout
                if (CAgileScout::instance().isConnected())
                {
                    CAgileScout::instance().setMoveCommand(0.F, 0.F);
                }
#endif // Build_Robot_AgileScout
                bReturn = true;
            }
        }
        break;
    }
    
    if (bReturn)
    {
        updateData(m_OutputCmd);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    
    return bReturn;
}

void CVehicleActuator::loadFromDatastream(const std::string&, const std::string&)
{
	// TBD
}

CVehicleActuator::VehicleActuatorApiType CVehicleActuator::StringToEnumType(const std::string& str_type)
{
    if (str_type.compare("sim") == 0)
        return VehicleActuatorApi_SIM;
    else if (str_type.compare("unity") == 0)
        return VehicleActuatorApi_UNITY;
    else if (str_type.compare("model_car") == 0)
        return VehicleActuatorApi_MODEL_CAR;
    else if (str_type.compare("raspi") == 0)
        return VehicleActuatorApi_RASPI;
    else if (str_type.compare("scout") == 0)
        return VehicleActuatorApi_SCOUT;

    return VehicleActuatorApi_SIM;
}
