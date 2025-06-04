// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleControlFilter.h"

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CVehicleControlFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CVehicleControlFilter(_params);
}

CVehicleControlFilter::CVehicleControlFilter(CycDatablockKey key) :
    CCycFilterBase(key)
{
    // Assign the output data type
    setFilterType("CyC_VEHICLE_CONTROL_FILTER_TYPE");
    m_OutputDataType = CyC_CONTROL_INPUT;
}

CVehicleControlFilter::CVehicleControlFilter(const ConfigFilterParameters& params) :
    CCycFilterBase(params)
{
    if (!m_CustomParameters["ControllerType"].empty())
    {
        m_ControllerType = StringToEnumType(m_CustomParameters["ControllerType"]);
    }
    else
    {
        spdlog::warn("Filter [{}-{}]: CVehicleStateSpaceControlFilter: No controller type defined. Switching to manual.", getFilterKey().nCoreID, getFilterKey().nFilterID);
        m_ControllerType = 0;
    }

    // Assign the output data type
    setFilterType("CyC_VEHICLE_CONTROL_FILTER_TYPE");
    m_OutputDataType = CyC_CONTROL_INPUT;
}

CVehicleControlFilter::~CVehicleControlFilter()
{
	if (m_bIsEnabled)
		disable();
}

bool CVehicleControlFilter::enable()
{
    if(!isNetworkFilter() && !isReplayFilter())
    {
        for (CycInputSource& src : getInputSources())
        {
            if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE") ||
                src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_SIMULATION_FILTER_TYPE"))
                m_pFilterVehicleState = &src;
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_LOCAL_PLANNER_FILTER_TYPE"))
                m_pFilterPathPlanner = &src;
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_SENSOR_FUSION_MODEL_FILTER_TYPE") ||
                src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_MAP_FILTER_TYPE"))
                m_pFilterMap = &src;
            else if (src.pCycFilter->getFilterType() == CStringUtils::CyC_HashFunc("CyC_VEHICLE_MISSION_PLANNER_FILTER_TYPE"))
                m_pFilterMissionPlanner = &src;
        }

        if (m_pFilterVehicleState == nullptr && (m_ControllerType != CONTROL_MANUAL))
        {
            spdlog::error("Filter [{}-{}]: {}: No state measurement filter given as input. Disabling the CVehicleControlFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
            return false;
        }

        if (m_ControllerType == CONTROL_PURE_PURSUITE)
        {
            if (m_pFilterPathPlanner == nullptr)
            {
                spdlog::error("Filter [{}-{}]: {}: Pure pursuite control requires a path planner filter as input. Disabling the CVehicleControlFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                return false;
            }
        }
        else if (m_ControllerType == CONTROL_DWA)
        {
            if (m_pFilterMissionPlanner == nullptr)
            {
                spdlog::error("Filter [{}-{}]: {}: DWA control requires a mission path filter as input. Disabling the CVehicleControlFilter.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                return false;
            }
        }


        if (!m_CustomParameters["ReferenceSpeed"].empty())
            m_RefSpeed = std::stof(m_CustomParameters["ReferenceSpeed"]);

        std::string vehicle_model_file;
        if (!m_CustomParameters["vehicle_model"].empty())
        {
            vehicle_model_file = fs::path(getGlobalBasePath()) / fs::path(m_CustomParameters.at("vehicle_model"));

            if (!fs::exists(vehicle_model_file))
            {
                spdlog::error("Filter [{}-{}]: CVehicleControlFilter: No vehicle model file defined. CVehicleLocalPlannerFilter disabled.", getFilterKey().nCoreID, getFilterKey().nFilterID);
                return false;
            }
        }

        float lookahead_distance = 5.7f; // lookahead distance
        if (!this->m_CustomParameters["LookaheadDistance"].empty())
            lookahead_distance = std::stof(this->m_CustomParameters["LookaheadDistance"]);
        else
            spdlog::warn("Filter [{}-{}]: {}: Lookahead distance parameter not found in config file. Defaulting to 5.7m...", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());

        float goal_distance = 1.f;
        if (!m_CustomParameters["goal_distance"].empty())
            goal_distance = std::stof(m_CustomParameters["goal_distance"]);
        else
            spdlog::warn("Filter [{}-{}]: {}: goal_distance not found inside control filter parameters. Defaulting to: {}", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name(), goal_distance);

        // Set the traversable class IDs used for projecting the segmented traversable into the octree
        if (!m_CustomParameters["traversable_class_ids"].empty())
        {
            std::vector<std::string> tokens;
            CStringUtils::splitstring(m_CustomParameters["traversable_class_ids"], ",", tokens);

            for (const auto& token : tokens)
                if (CStringUtils::is_positive_int(token))
                    m_TraversableClassIDs.emplace_back(std::stoi(token));
                else
                    spdlog::warn("CSensorFusionFilter::init(): Could not convert '{}' to traversable class id.", token);
        }

        m_pVehicleModel = std::make_unique<CModelVehicle>(vehicle_model_file);

        if (m_ControllerType == CONTROL_PURE_PURSUITE)
            m_pControllerPurePursuite = std::make_unique<CControllerPurePursuite>(*m_pVehicleModel, this->getDt() * MSEC2SEC, m_RefSpeed, lookahead_distance);
        else if (m_ControllerType == CONTROL_DWA)
            m_pControllerDwa = std::make_unique<CDwa>(float(this->getDt() * MSEC2SEC), vehicle_model_file, lookahead_distance, goal_distance, m_TraversableClassIDs);

        if (m_ControllerType == CONTROL_MANUAL)
            m_ManualCtrlThread = std::thread(&CVehicleControlFilter::manual_ctrl_thread, this);
    }

    m_OutputCmd.u = Eigen::VectorXf::Zero(ModelVehicle_NumInputs);
    updateData(m_OutputCmd);

    spdlog::info("Filter [{}-{}]: CVehicleControlFilter::enable() successful", getFilterKey().nCoreID, getFilterKey().nFilterID);
    
    // Enable filter
    m_bIsEnabled = true;

	return m_bIsEnabled;
}

bool CVehicleControlFilter::disable()
{
	if (isRunning())
		stop();

    // Stop the manual control thread
    if (m_ManualCtrlThread.joinable())
        m_ManualCtrlThread.join();

    m_bIsEnabled = false;
    return true;
}

CyC_INT CVehicleControlFilter::StringToEnumType(const std::string& str_ctrl_name)
{
    if (str_ctrl_name.compare("manual") == 0)
        return CONTROL_MANUAL;
    else if (str_ctrl_name.compare("PP") == 0)
        return CONTROL_PURE_PURSUITE;
    else if (str_ctrl_name.compare("MPC") == 0)
        return CONTROL_MPC;
    else if (str_ctrl_name.compare("LQR") == 0)
        return CONTROL_LQR;
    else if (str_ctrl_name.compare("skid") == 0)
        return CONTROL_SKID;
    else if (str_ctrl_name.compare("dwa") == 0)
        return CONTROL_DWA;
    return CONTROL_MANUAL;
}

void CVehicleControlFilter::manual_ctrl_thread()
{
    bool bWindowOn(true);
    char key;
    while (bWindowOn)
    {
        cv::Mat disp = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);
        key = cv::waitKey(5);

        switch (key)
        {
            // Forward
            case 'w':
                m_OutputCmd.u[0] += 0.05f;
                break;
            // Backward
            case 's':
                m_OutputCmd.u[0] -= 0.05f;
                break;
            // Left
            case 'a':
                m_OutputCmd.u[1] += 1.f * DEG2RAD;
                break;
            // Right
            case 'd':
                m_OutputCmd.u[1] -= 1.f * DEG2RAD;
                break;
            // Stop
            case 'x':
                m_OutputCmd.u[0] = 0.f;
                m_OutputCmd.u[1] = 0.f;
                break;
            // Exit manual control
            case 27: // ESC
                bWindowOn = false;
            default:
                break;
        }

        // Acceleration constraints
        /*if (m_VehicleStateCommand.u[0] >= m_pVehicleModel->m_fMaxAcceleration)
            m_VehicleStateCommand.u[0] = m_pVehicleModel->m_fMaxAcceleration;
        else if (m_VehicleStateCommand.u[0] <= -m_pVehicleModel->m_fMaxAcceleration)
            m_VehicleStateCommand.u[0] = -m_pVehicleModel->m_fMaxAcceleration;
        */
        // Steering constraints
        /*
        if (m_VehicleStateCommand.u[1] >= m_pVehicleModel->m_fMaxSteeringAngleRad)
            m_VehicleStateCommand.u[1] = m_pVehicleModel->m_fMaxSteeringAngleRad;
        else if (m_VehicleStateCommand.u[1] <= -m_pVehicleModel->m_fMaxSteeringAngleRad)
            m_VehicleStateCommand.u[1] = -m_pVehicleModel->m_fMaxSteeringAngleRad;
        */

        char str[128];
        snprintf(str, sizeof(str) - 1, "Acceleration: %.2f m/ss", m_OutputCmd.u(0));
        cv::putText(disp, str, cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        snprintf(str, sizeof(str) - 1, "Steering: %.2f deg", m_OutputCmd.u(1) * RAD2DEG);
        cv::putText(disp, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "Commands:", cv::Point(20, 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'w' - Forward", cv::Point(20, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'s' - Backward", cv::Point(20, 110), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'a' - Left", cv::Point(20, 130), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'d' - Right", cv::Point(20, 150), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);
        cv::putText(disp, "'x' - Stop", cv::Point(20, 170), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);

        cv::imshow(getFilterName(), disp);
    }
}

bool CVehicleControlFilter::process()
{
    bool bReturn(false);

    if (m_ControllerType == CONTROL_DWA)
    {
        // Get the global reference path from the vehicle mission planner
        CyC_TIME_UNIT ts_mission_plan = m_pFilterMissionPlanner->pCycFilter->getTimestampStart();

        while (m_MissionPlan.empty() || ts_mission_plan > m_lastReadTSMissionPlan)
        {
            if (m_pFilterMissionPlanner->pCycFilter->getData(m_MissionPlan) &&
                (m_pFilterVehicleState->pCycFilter->getTimestampStop() > 0) &&
                m_pFilterVehicleState->pCycFilter->getData(m_VehicleState))
            {
                m_lastReadTSMissionPlan = ts_mission_plan;
                m_pControllerDwa->setMissionPath(m_MissionPlan, m_VehicleState);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (m_ControllerType == CONTROL_MANUAL)
    {
        // Implements manual control
        bReturn = true;
    }
    else
    {
        // Implements automatic control
        bool bStateRead = false;
        const CyC_TIME_UNIT ts_state = m_pFilterVehicleState->pCycFilter->getTimestampStop();
        if (ts_state > m_lastReadTSState)
        {
            m_lastReadTSState = ts_state;
            bStateRead = m_pFilterVehicleState->pCycFilter->getData(m_VehicleState);
        }

        if (m_ControllerType == CONTROL_PURE_PURSUITE)
        {
            const CyC_TIME_UNIT ts_path_planner = m_pFilterPathPlanner->pCycFilter->getTimestampStop();
            if (ts_path_planner > m_lastReadTSPathPlanner)
            {
                m_lastReadTSPathPlanner = ts_path_planner;
                m_pFilterPathPlanner->pCycFilter->getData(m_VehicleReferenceSetPoints);
            }

            if (bStateRead)
            {
                if (m_VehicleReferenceSetPoints.ref.size() > 0)
                {
                    m_pControllerPurePursuite->set_r(m_VehicleReferenceSetPoints.ref.back());
                    m_pControllerPurePursuite->update(m_VehicleState.x_hat);
                    m_OutputCmd.u = m_pControllerPurePursuite->u();
                    bReturn = true;
                }
            }
        }
        else if (m_ControllerType == CONTROL_DWA)
        {
            if (m_pFilterMap != nullptr)
            {
                const CyC_TIME_UNIT ts_map = m_pFilterMap->pCycFilter->getTimestampStop();
                if (ts_map > m_lastReadTSMap)
                {
                    m_lastReadTSMap = ts_map;
                    m_pFilterMap->pCycFilter->getData(m_EnvironmentSnapshot);
                }
            }

            if (bStateRead)
            {
                m_OutputCmd = m_pControllerDwa->dwaControl(m_VehicleState, m_EnvironmentSnapshot);
                bReturn = true;
            }
        }
    }

    if (bReturn)
    {
        // Set target speed to zero if the vehicle position is below 1m from target
        if (m_MissionPlan.size() > 0)
        {
            const Eigen::VectorXf y = m_VehicleState.x_hat;
            const Eigen::Vector4f r = m_MissionPlan.back();
            float Ed = std::abs(sqrtf(powf(r(0) - y(0), 2) + powf(r(1) - y(1), 2)));
            if (Ed < 1.f)
                m_OutputCmd.u[0] = 0.f;
        }

        updateData(m_OutputCmd);
    }
    return bReturn;
}

void CVehicleControlFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
	// Set the processing flag
	m_bIsProcessing = true;

	csv::reader::row row;
	row.parse_line(datastream_entry, ',');
	enum { TS_STOP, SAMPLING_TIME, NUM};
	if (row.size() < NUM)
	{
		spdlog::error("CVehicleControlFilter: Wrong number of columns. {} provided, but expected at least {}.", row.size(), NUM + 1);
		return;
	}

	const auto sizeCtrl = row.size() - NUM + 1;

	m_OutputCmd.u.resize(sizeCtrl);
	for (auto idx = 0; idx < sizeCtrl; ++idx)
	{
		m_OutputCmd.u[idx] = row.get<float>(NUM + idx);
	}

    const auto tTimestampStop  = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime   = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;

	updateData(m_OutputCmd, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);

	// Unset the processing flag
	m_bIsProcessing = false;
}
