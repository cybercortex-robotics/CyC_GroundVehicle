// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleStateEstimationFilter.h"
#include <env/CGeolocation.h>

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CVehicleStateEstimationFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CVehicleStateEstimationFilter(_params);
}

CVehicleStateEstimationFilter::CVehicleStateEstimationFilter(CycDatablockKey key) :
    CCycFilterBase(key)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE");
    m_OutputDataType = CyC_STATE;
    m_initialHeadingCompass.push_back(0);
}

CVehicleStateEstimationFilter::CVehicleStateEstimationFilter(const ConfigFilterParameters& params) :
    CCycFilterBase(params)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE");
    m_OutputDataType = CyC_STATE;
    m_initialHeadingCompass.push_back(0);
}

CVehicleStateEstimationFilter::~CVehicleStateEstimationFilter()
{
	if (m_bIsEnabled)
		disable();
}

bool CVehicleStateEstimationFilter::enable()
{
    m_bIsEnabled = false;
    m_InitialState.x_hat = Eigen::VectorXf::Zero(ModelVehicle_NumStates);

    // Define input filters
    if (!isReplayFilter() && !isNetworkFilter())
    {
        if (m_CustomParameters.find("interface") == m_CustomParameters.end())
            spdlog::warn("Filter [{}-{}]: CVehicleStateEstimationFilter: interface parameter not found in configuration file. Sim will be used.", getFilterKey().nCoreID, getFilterKey().nFilterID);

        m_InterfaceType = StringToEnumType(m_CustomParameters["interface"]);

        // Set the initial state
        if (m_CustomParameters.find("initial_state") != m_CustomParameters.end())
        {
            // Initial state from configuration file
            CBaseStateSpaceModel::str2state(m_CustomParameters["initial_state"], ModelVehicle_NumStates, m_InitialState);
        }
        else if (m_bUseGPS)
        {
            // initial state from GPS is taken in process()
        }
        else
        {
            spdlog::warn("Filter [{}-{}]: CVehicleStateEstimationFilter: No initial state set. Using default [0, 0, 0, 0].", getFilterKey().nCoreID, getFilterKey().nFilterID);
        }

        if (m_InterfaceType == VehicleStateEstimationApi_SIM)
        {
            bool bFoundSimulationFilter = false;
            for (const CycInputSource& src : this->getInputSources())
            {
                if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE") ||
                    src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE") ||
                    src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_INERTIAL_ODOMETRY_FILTER_TYPE"))
                {
                    m_pInputFilterVehicleSim = src.pCycFilter;
                    bFoundSimulationFilter = true;
                }
            }

            if (!bFoundSimulationFilter)
            {
                spdlog::error("Filter [{}-{}]: CVehicleStateEstimationFilter::enable(): No vehicle state input source found. Cannot copy state estimation. Disabling CVehicleStateEstimationFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID);
                return false;
            }
            else
            {
                m_bIsEnabled = true;
            }
        }
        else if (m_InterfaceType == VehicleStateEstimationApi_REAL)
        {
            for (const CycInputSource& src : getInputSources())
            {
                if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_ACTUATOR_FILTER_TYPE"))
                {
                    m_pInputFilterActuators = src.pCycFilter;
                    m_bUseActuators = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_CONTROL_FILTER_TYPE"))
                {
                    m_pInputFilterActuators = src.pCycFilter;
                    m_bUseActuators = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_ROTARY_ENCODER_FILTER_TYPE"))
                {
                    m_pInputFilterWheelsEncoder = src.pCycFilter;
                    m_bUseWheelsEncoder = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_IMU_FILTER_TYPE"))
                {
                    m_pInputFilterIMU = src.pCycFilter;
                    m_bUseIMU = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_COMPASS_FILTER_TYPE"))
                {
                    m_pInputFilterCompass = src.pCycFilter;
                    m_bUseCompass = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_GPS_FILTER_TYPE"))
                {
                    m_pInputFilterGPS = src.pCycFilter;
                    m_bUseGPS = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VISUAL_SLAM_FILTER_TYPE"))
                {
                    m_pInputFilterVisualSlam = src.pCycFilter;
                    m_bUseVisualSlam = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_MISSION_PLANNER_FILTER_TYPE"))
                {
                    m_pMissionPlanner = src.pCycFilter;
                    m_bUseMissionPlanner = true;
                }
                else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE"))
                {
                    spdlog::warn("Filter [{}-{}]: CVehicleStateEstimationFilter: Simulation filter specified, but the filter does not use SIM interface. Ignoring.", getFilterKey().nCoreID, getFilterKey().nFilterID);
                }
                else
                {
                    spdlog::warn("Filter [{}-{}]: CVehicleStateEstimationFilter: Unknown input specified {}", getFilterKey().nCoreID, getFilterKey().nFilterID, src.pCycFilter->getFilterKey().nFilterID);

                }
            }
            
            // Vehicle model
            if (!m_CustomParameters["vehicle_model"].empty())
            {
                std::string vehicle_model_file = fs::path(getGlobalBasePath()) / fs::path(m_CustomParameters.at("vehicle_model"));

                if (!fs::exists(vehicle_model_file))
                {
                    spdlog::error("Filter [{}-{}]: CVehicleStateEstimationFilter: No vehicle model file defined. CVehicleLocalPlannerFilter disabled.", getFilterKey().nCoreID, getFilterKey().nFilterID);
                    return false;
                }

                m_pVehicleStateEstimation = std::make_unique<CVehicleStateEstimation>(vehicle_model_file);
                m_pVehicleStateEstimation->setAvailableInput(m_bUseActuators, m_bUseWheelsEncoder, m_bUseIMU, m_bUseCompass, m_bUseGPS, m_bUseVisualSlam);
                m_pVehicleStateEstimation->setInitialState(m_InitialState);
            }

            m_bIsEnabled = true;
        }
        else if (m_InterfaceType == VehicleStateEstimationApi_UNITY)
        {
            if ((m_CustomParameters.find("ip") == m_CustomParameters.end()) ||
                (m_CustomParameters.find("port") == m_CustomParameters.end()))
            {
                spdlog::error("Filter [{}-{}]: CVehicleStateEstimationFilter: ip and/or port parameters not found in configuration file", getFilterKey().nCoreID, getFilterKey().nFilterID);
                return false;
            }

            m_bIsEnabled = m_UnityApi.enable(m_CustomParameters["ip"], m_CustomParameters["port"]);
        }

        // Set initial state
        updateData(m_InitialState);
    }
    else
    {
        m_bIsEnabled = true;
    }

    if (m_bIsEnabled)
        spdlog::info("Filter [{}-{}]: CVehicleStateEstimationFilter::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);
    else
        spdlog::error("Filter [{}-{}]: CVehicleStateEstimationFilter::enable() failed", getFilterKey().nCoreID, getFilterKey().nFilterID);

    return m_bIsEnabled;
}


bool CVehicleStateEstimationFilter::disable()
{	
	if (isRunning())
        stop();

	m_bIsEnabled = false;
	return true;
}


bool CVehicleStateEstimationFilter::process()
{
    bool bReturn = false;
    if (m_InterfaceType == VehicleStateEstimationApi_SIM)
    {
        const CyC_TIME_UNIT ts = m_pInputFilterVehicleSim->getTimestampStop();

        if (ts > m_lastTsVehicleSim)
        {
            CycState state;
            if (m_pInputFilterVehicleSim->getData(state))
            {
                m_lastTsVehicleSim = ts;
                m_FilterOutput = state;
                bReturn = true;
            }
        }
    }
    else if (m_InterfaceType == VehicleStateEstimationApi_UNITY)
    {
        m_UnityApi.process();
        CycState state = m_UnityApi.get_state();
        
        if (state.x_hat.size() > 0)
        {
            m_FilterOutput = state;
            bReturn = true;
        }
    }
    else if (m_InterfaceType == VehicleStateEstimationApi_REAL)
    {
        if (!m_bStateInitialized)
        {
            spdlog::warn("CVehicleStateEstimation: Initial state not set in conf file. Initializing state estimation with position from GPS, Mission Planner, IMU and/or Compass.");

            // Initial position given by GPS
            if (m_bUseMissionPlanner && m_bUseGPS)
            {
                spdlog::info("CVehicleStateEstimation: Position will be initialized from GPS and Mission Planner.");

                bool readGPS = false;
                bool readMission = false;

                CycGps gps;
                CycPoints mission;

                do {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    readGPS = m_pInputFilterGPS->getData(gps);
                    readMission = m_pMissionPlanner->getData(mission);
                } while (!readGPS || !readMission);

                const auto gps_pos = CGeolocation::gps2utm(gps.lat, gps.lng, gps.alt);
                const auto mission_pos = CGeolocation::gps2utm(mission.front().pt2d[0], mission.front().pt2d[1], mission.front().pt2d[2]);

                m_InitialState.x_hat[0] = gps_pos.x() - mission_pos.x();
                m_InitialState.x_hat[1] = gps_pos.y() - mission_pos.y();

                const CycGps mission_gps{ mission.front().pt2d[0], mission.front().pt2d[1], mission.front().pt2d[2] };
                m_pVehicleStateEstimation->setMissionOrigin(mission_gps);

                spdlog::info("CVehicleStateEstimation: Initial position calculated is ({}, {}).", m_InitialState.x_hat[0], m_InitialState.x_hat[1]);
            }
            else if (m_bUseMissionPlanner && !m_bUseGPS)
            {
                spdlog::warn("CVehicleStateEstimation: Can't initialize position just from Mission Planner. Filter also needs the GPS.");
            }
            else if (!m_bUseMissionPlanner && m_bUseGPS)
            {
                //spdlog::warn("CVehicleStateEstimation: Can't initialize position just from GPS. Filter also needs the mission planner.");
            }

            if (m_bUseIMU && !m_bUseCompass)
            {
                spdlog::info("CVehicleStateEstimation: Orientation will be initialized from IMU.");

                CycImu imu;
                while (true)
                {
                    if (m_pInputFilterIMU->getData(imu))
                    {
                        Eigen::Vector3f euler_rpy(CQuaternion(imu.quat).to_euler_ZYX());
                        m_InitialState.x_hat[3] = euler_rpy.z();
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                spdlog::info("CVehicleStateEstimation: Initial orientation is {} deg.", m_InitialState.x_hat[3] * RAD2DEG);
            }

            if (m_bUseCompass)
            {
                spdlog::info("CVehicleStateEstimation: Orientation will be initialized from Compass.");

                while (true)
                {
                    if (m_pInputFilterCompass->getData(m_initialHeadingCompass))
                    {
                        m_InitialState.x_hat[3] = m_initialHeadingCompass.at(0) * DEG2RAD;
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                spdlog::info("CVehicleStateEstimation: Initial orientation is {} deg.", m_InitialState.x_hat[3] * RAD2DEG);
            }

            m_pVehicleStateEstimation->setInitialState(m_InitialState);
            m_bStateInitialized = true;
        }

        bool bUpdateStateEstimate = false;

        // Update actuators
        if (m_bUseActuators)
        {
            const auto ts = m_pInputFilterActuators->getTimestampStop();
            if (ts != m_lastTsActuators)
            {
                CycControlInput cmd;
                if (m_pInputFilterActuators->getData(cmd))
                {
                    m_lastTsActuators = ts;
                    m_pVehicleStateEstimation->updateActuators(cmd, ts);
                    bUpdateStateEstimate = true;
                }
            }
        }

        // Update wheels encoder
        if (m_bUseWheelsEncoder)
        {
            const auto ts = m_pInputFilterWheelsEncoder->getTimestampStop();
            if (ts > m_lastTsWheelsEncoder)
            {
                std::vector<CyC_INT> wheels;
                if (m_pInputFilterWheelsEncoder->getData(wheels))
                {
                    m_lastTsWheelsEncoder = ts;
                    m_pVehicleStateEstimation->updateWheelsEncoder(wheels, ts);
                    bUpdateStateEstimate = true;
                }
            }
        }

        // Update IMU
        if (m_bUseIMU)
        {
            const auto ts = m_pInputFilterIMU->getTimestampStop();
            if (ts > m_lastTsIMU)
            {
                CycImu imu;
                if (m_pInputFilterIMU->getData(imu))
                {
                    CQuaternion imu_quat(imu.quat);

                    Eigen::Vector3f imu_euler_rpy = imu_quat.to_euler_ZYX();
                    
                    // Offset IMU with the initial compass value
                    imu_euler_rpy.x() = imu_euler_rpy.x() + m_initialHeadingCompass.at(0);

                    // Clip the value to the [0, 360] interval (6.28319 rad = 360 deg)
                    if (imu_euler_rpy.x() >= 6.28319f)
                        imu_euler_rpy.x() = imu_euler_rpy.x() - 6.28319f;

                    imu_quat.update(imu_euler_rpy.x(), imu_euler_rpy.y(), imu_euler_rpy.z());
                    imu.quat = imu_quat.to_vector();
                    
                    m_lastTsIMU = ts;
                    m_pVehicleStateEstimation->updateIMU(imu, ts);
                    bUpdateStateEstimate = true;
                }
            }
        }

        // Update Compass
        if (m_bUseCompass)
        {
            const auto ts = m_pInputFilterCompass->getTimestampStop();
            if (ts > m_lastTsCompass)
            {
                std::vector<float> heading;
                if (m_pInputFilterCompass->getData(heading))
                {
                    m_lastTsCompass = ts;
                    m_pVehicleStateEstimation->updateCompass(heading, ts);
                    bUpdateStateEstimate = true;
                }
            }
        }

        // Update GPS
        if (m_bUseGPS)
        {
            const auto ts = m_pInputFilterGPS->getTimestampStop();
            if (ts > m_lastTsGPS)
            {
                CycGps gps;
                if (m_pInputFilterGPS->getData(gps))
                {
                    m_lastTsGPS = ts;
                    m_pVehicleStateEstimation->updateGPS(gps, ts);
                    bUpdateStateEstimate = true;
                }
            }
        }

        // Update visual slam
        if (m_bUseVisualSlam)
        {
            const auto ts = m_pInputFilterVisualSlam->getTimestampStop();
            if (ts > m_lastTsVisualSlam)
            {
                CycSlam slam;
                if (m_pInputFilterVisualSlam->getData(slam))
                {
                    m_lastTsVisualSlam = ts;
                    m_pVehicleStateEstimation->updateVisualSlam(slam, ts);
                    bUpdateStateEstimate = true;
                }
            }
        }

        // Calculate state estimation and update filter output
        if (bUpdateStateEstimate)
        {
            m_pVehicleStateEstimation->update();
            m_pVehicleStateEstimation->getStateEstimate(m_FilterOutput);
            bReturn = true;
        }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (bReturn)
    {
        // Save state estimation output into data cache
        updateData(m_FilterOutput);
    }

    return bReturn;
}

void CVehicleStateEstimationFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // Set the processing flag
    m_bIsProcessing = true;

    csv::reader::row row;
    row.parse_line(datastream_entry, ',');

    enum { TS_STOP, SAMPLING_TIME, NUM };
    if (row.size() <= NUM)
    {
        spdlog::error("Filter [{}-{}]: Wrong number of columns. {} provided, but expected {} or more.", getFilterKey().nCoreID, getFilterKey().nFilterID, row.size(), NUM + 1);
        return;
    }

    CycState m_State;
    m_State.x_hat.resize(row.size() - NUM);
    for (size_t i = NUM; i < row.size(); ++i)
    {
        m_State.x_hat[i - NUM] = row.get<float>(i);
    }

    const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;
    
    updateData(m_State, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);

    // Unset the processing flag
    m_bIsProcessing = false;
}

CVehicleStateEstimationFilter::VehStateEstApiType CVehicleStateEstimationFilter::StringToEnumType(const std::string& str_type)
{
    if (str_type.compare("sim") == 0)
        return VehicleStateEstimationApi_SIM;
    else if (str_type.compare("unity") == 0)
        return VehicleStateEstimationApi_UNITY;
    else if (str_type.compare("real") == 0)
        return VehicleStateEstimationApi_REAL;

    return VehicleStateEstimationApi_SIM;
}