// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleSim.h"

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CVehicleSim(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CVehicleSim(_params);
}

CVehicleSim::CVehicleSim(CycDatablockKey key) : CCycFilterBase(key)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_VEHICLE_SIMULATION_FILTER_TYPE");
    m_OutputDataType = CyC_STATE;
}

CVehicleSim::CVehicleSim(const ConfigFilterParameters& params) : CCycFilterBase(params)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_VEHICLE_SIMULATION_FILTER_TYPE");
    m_OutputDataType = CyC_STATE;
}

CVehicleSim::~CVehicleSim()
{
	if (m_bIsEnabled)
		disable();
}

bool CVehicleSim::enable()
{
    m_bIsEnabled = false;

    if (!this->isReplayFilter() && !this->isNetworkFilter())
    {
        if (this->getInputSources().size() == 0)
            spdlog::info("Filter [{}-{}]: CVehicleSim::enable(): No input actuator sources. Outputing static state.", getFilterKey().nCoreID, getFilterKey().nFilterID);

        for (const CycInputSource& src : this->getInputSources())
        {
            if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_ACTUATOR_FILTER_TYPE") ||
                src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_CONTROL_FILTER_TYPE"))
            {
                m_pActuatorFilter = src.pCycFilter;
                m_bActuatorFound = true;
            }
        }
        if (m_pActuatorFilter == nullptr)
            spdlog::info("Filter [{}-{}]: CVehicleSim::enable(): No input actuator sources. Outputing static state.", getFilterKey().nCoreID, getFilterKey().nFilterID);

        std::string vehicle_model_file;
        if (!m_CustomParameters["vehicle_model"].empty())
        {
            vehicle_model_file = fs::path(getGlobalBasePath()) / fs::path(m_CustomParameters.at("vehicle_model"));

            if (!fs::exists(vehicle_model_file))
            {
                spdlog::error("Filter [{}-{}]: CVehicleSim: No vehicle model file defined. CVehicleLocalPlannerFilter disabled.", getFilterKey().nCoreID, getFilterKey().nFilterID);
                return false;
            }
        }

        m_pVehicleModel = std::make_unique<CModelVehicle>(vehicle_model_file);

        // Initial zero state
        m_State.x_hat = Eigen::VectorXf::Zero(m_pVehicleModel->getNumStates());

        // Read the initial state
        if (!m_pVehicleModel->str2state(m_CustomParameters["initial_state"], m_State))
            spdlog::error("Filter [{}-{}]: CVehicleSim: Initial conditions could not be read.", getFilterKey().nCoreID, getFilterKey().nFilterID);
        
        m_pVehicleModel->set_x(m_State.x_hat);
        m_pVehicleModel->set_y(m_State.x_hat);

        log_info("Initial vehicle position: ({}, {})", m_State.x_hat.x(), m_State.x_hat.y());

        //updateData(m_State);

        // Enable filter
        m_bIsEnabled = true;
    }
    else
    {
        m_bIsEnabled = true;
    }
    
    if (m_bIsEnabled)
	    spdlog::info("Filter [{}-{}]: CVehicleSim::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);
    else
        spdlog::info("Filter [{}-{}]: CVehicleSim::enable() failed", getFilterKey().nCoreID, getFilterKey().nFilterID);

	return true;
}

bool CVehicleSim::disable()
{	
	if (isRunning())
        stop();

	m_bIsEnabled = false;
	return true;
}

bool CVehicleSim::process()
{
    bool bReturn(false);

    if (!m_bActuatorFound)
    {
        bReturn = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    else
    {
        const CyC_TIME_UNIT tsActuator = m_pActuatorFilter->getTimestampStop();

        if (tsActuator > m_prevTsActuator)
        {
            CycControlInput cmd;
            if (m_pActuatorFilter->getData(cmd))
            {
                float dt = 0.01f;
                
                if (m_prevTsActuator != 0)  
                    dt = (float)(tsActuator - m_prevTsActuator) * MSEC2SEC;

                m_pVehicleModel->step(dt, cmd.u);
                m_State.x_hat = m_pVehicleModel->x();
                m_prevTsActuator = tsActuator;
            }
        }

        bReturn = true;
    }

    if (bReturn)
    {
        updateData(m_State);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    return bReturn;
}

void CVehicleSim::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // Set the processing flag
    m_bIsProcessing = true;
    
    csv::reader::row row;
    row.parse_line(datastream_entry, ',');

    enum { TS_STOP, SAMPLING_TIME, NUM };
    if (row.size() <= NUM)
    {
        spdlog::error("CVehicleSim: Wrong number of columns. {} provided, but expected {} or more.", row.size(), NUM + 1);
        return;
    }

    CycState m_StateMeasurement;
    m_StateMeasurement.x_hat.resize(row.size() - NUM);
    for (size_t i = NUM; i < row.size(); ++i)
    {
        m_StateMeasurement.x_hat[i - NUM] = row.get<float>(i);
    }

    const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;
    
    updateData(m_StateMeasurement, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);
    
    // Unset the processing flag
    m_bIsProcessing = false;
}
