// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CTerminalVehicle.h"
#include "env/CObjectClasses.h"

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CTerminalVehicle(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CTerminalVehicle(_params);
}

CTerminalVehicle::CTerminalVehicle(CycDatablockKey key) : CCycFilterBase(key)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_TERMINAL_VEHICLE_FILTER_TYPE");
    m_OutputDataType = CyC_TERMINAL_DATA;
}

CTerminalVehicle::CTerminalVehicle(const ConfigFilterParameters& _params) : CCycFilterBase(_params)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_TERMINAL_VEHICLE_FILTER_TYPE");
    m_OutputDataType = CyC_TERMINAL_DATA;

    // Check if the QT singleton is already registered
    if (_params.pSingletonRegistry != nullptr)
    {
        if (_params.pSingletonRegistry->get<CCycQTSkeleton>() == nullptr)
            _params.pSingletonRegistry->registerInstance<CCycQTSkeleton>(_params.pSingletonRegistry);
        m_pCcrQTSkeleton = _params.pSingletonRegistry->get<CCycQTSkeleton>();
        m_qt_image = std::make_unique<CCycQTImage>(m_pCcrQTSkeleton.get());
    }
}

CTerminalVehicle::~CTerminalVehicle()
{
    if (m_bIsEnabled)
        disable();
}

bool CTerminalVehicle::enable()
{
    if (!this->isReplayFilter() && !this->isNetworkFilter())
    {
        m_HmiThread = std::thread(&CTerminalVehicle::state_machine_thread, this);
    }

    // Read default command
    if (!m_CustomParameters["default_cmd"].empty())
    {
        if (m_CustomParameters["default_cmd"].compare("drive") == 0)
            m_VehicleCommand = Vehicle_DRIVE;
        else if (m_CustomParameters["default_cmd"].compare("explore") == 0)
            m_VehicleCommand = Vehicle_EXPLORE;
        else
            m_VehicleCommand = Vehicle_STOP;
    }

    // Read destinations
    if (!m_CustomParameters["destinations"].empty())
        str2destinations(m_CustomParameters["destinations"]);

    spdlog::info("Filter [{}-{}]: {} enabled successfully.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());

    // Enable filter
    m_bIsEnabled = true;

    return true;
}

bool CTerminalVehicle::disable()
{
    if (isRunning())
        stop();

    // Stop the threads
    if (m_HmiThread.joinable())
        m_HmiThread.join();

    m_bIsEnabled = false;
    return true;
}

bool CTerminalVehicle::process()
{
    bool bReturn = m_bUpdate;

    if (bReturn)
    {
        CycTerminalCommand term;
        term.cmd = { m_VehicleCommand, m_VehicleControlMode };
        if (m_IdxDestination >= 0 && m_IdxDestination < m_Destinations.size())
            term.destination = m_Destinations[m_IdxDestination].first;

        m_bUpdate = false;
        updateData(term);
    }

    return bReturn;
}

void CTerminalVehicle::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
    // TODO
}

void CTerminalVehicle::state_machine_thread()
{
    CyC_INT key = 0;
    char str_state[128] = {};
    static char str_msg[128] = {};

    char title[32];
    snprintf(title, sizeof(title) - 1, "[%d - %d] %s API", getFilterKey().nCoreID, getFilterKey().nFilterID, getFilterName().c_str());
    
    while (true)
    {
        Vehicle_Command last_cmd = m_VehicleCommand;

        key = m_qt_image->get_last_key(title);

        cv::Mat disp = cv::Mat::zeros(cv::Size(500, 300), CV_8UC3);
        cv::putText(disp, str_state, cv::Point(20, 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 200, 255), 1);
        cv::putText(disp, "Commands:", cv::Point(20, 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 255, 255), 1);

        cv::Scalar color_cmd;
        if (m_VehicleCommand == Vehicle_STOP)
            color_cmd = color::red;
        else
            color_cmd = color::white;
        cv::putText(disp, "'s' - Stop", cv::Point(20, 100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color_cmd, 1);
        
        if (m_VehicleCommand == Vehicle_DRIVE)
            color_cmd = color::green;
        else
            color_cmd = color::white;
        cv::putText(disp, "'d' - Drive", cv::Point(170, 100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color_cmd, 1);

        if (m_VehicleCommand == Vehicle_EXPLORE)
            color_cmd = color::green;
        else
            color_cmd = color::white;
        cv::putText(disp, "'e' - Explore", cv::Point(330, 100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color_cmd, 1);
        cv::putText(disp, str_msg, cv::Point(20, disp.rows - 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.75, CV_RGB(200, 255, 100), 1);

        // Display destinations
        for (size_t i = 0; i < m_Destinations.size(); ++i)
        {
            char s[128]; 
            cv::Scalar color_dst = color::white;
            if (i == m_IdxDestination)
                color_dst = CV_RGB(0, 200, 255);
            const auto& dst = m_Destinations[i];
            const Eigen::Vector3f& dst_pos = dst.first.translation_3x1();
            snprintf(s, sizeof(s) - 1, "[%d] %s (%.1f, %.1f, %.1f)", i, dst.second, dst_pos.x(), dst_pos.y(), dst_pos.z());
            cv::putText(disp, s, cv::Point(20, 140 + i * 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, color_dst, 1);
        }

        cv::Mat cvt;
        cv::cvtColor(disp, cvt, cv::COLOR_BGR2RGBA);
        m_qt_image->display_rgba(cvt.data, cvt.cols, cvt.rows, title);

        //cv::imshow("qqq", disp);
        //cv::waitKey(1);

        // Get key input
        switch (key)
        {
            case 's':
            {
                if (m_VehicleCommand != Vehicle_STOP)
                {
                    m_VehicleCommand = Vehicle_STOP;
                    snprintf(str_msg, sizeof(str_msg) - 1, "");
                }
                else
                {
                    snprintf(str_msg, sizeof(str_msg) - 1, "Stop command already sent");
                }
                break;
            }
            case 'd':
            {
                if (m_VehicleCommand != Vehicle_DRIVE)
                {
                    m_VehicleCommand = Vehicle_DRIVE;
                    snprintf(str_msg, sizeof(str_msg) - 1, "");
                }
                else
                {
                    snprintf(str_msg, sizeof(str_msg) - 1, "Drive command already sent");
                }
                break;
            }
            case 'e':
            {
                if (m_VehicleCommand != Vehicle_EXPLORE)
                {
                    m_VehicleCommand = Vehicle_EXPLORE;
                    snprintf(str_msg, sizeof(str_msg) - 1, "");
                }
                else
                {
                    snprintf(str_msg, sizeof(str_msg) - 1, "Explore command already sent");
                }
                break;
            }
            
            default:
                break;
        }

        // Check destination
        for (size_t i = 0; i < m_Destinations.size(); ++i)
        {
            if (key - '0' == i)
            {
                m_IdxDestination = i;
                m_VehicleCommand = Vehicle_DRIVE;
            }
        }

        // Update state
        switch (m_VehicleCommand)
        {
            case Vehicle_STOP:
                snprintf(str_state, sizeof(str_state) - 1, "Last sent command: Stop");
                break;
            case Vehicle_DRIVE:
                snprintf(str_state, sizeof(str_state) - 1, "Last sent command: Drive");
                break;
            case Vehicle_EXPLORE:
                snprintf(str_state, sizeof(str_state) - 1, "Last sent command: Explore");
                break;
            default:
                break;
        }

        if (m_VehicleCommand != last_cmd)
            m_bUpdate = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CTerminalVehicle::str2destinations(const std::string& _str)
{
    m_Destinations.clear();

    std::vector<std::string> dsts; CStringUtils::splitstring(_str, ";", dsts);
    for (const auto& dst : dsts)
    {
        std::vector<std::string> s; CStringUtils::splitstring(dst, ",", s);
        if (s.size() != 4)
        {
            spdlog::error("Filter [{}-{}]: {}: Bad destination format. Exiting.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
            exit(EXIT_FAILURE);
        }

        s[0] = CStringUtils::remove_spaces(s[0]);
        s[1] = CStringUtils::remove_spaces(s[1]);
        s[2] = CStringUtils::remove_spaces(s[2]);

        s[0] = s[0].substr(2);
        s[2] = s[2].substr(0, s[2].size() - 1);
        s[3] = s[3].substr(0, s[3].size() - 1);

        if (!CStringUtils::is_float(s[0]) || !CStringUtils::is_float(s[1]) || !CStringUtils::is_float(s[2]))
        {
            spdlog::error("Filter [{}-{}]: {}: Destination position is not a number. Exiting.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
            exit(EXIT_FAILURE);
        }

        CPose pose(std::stof(s[0]), std::stof(s[1]), std::stof(s[2]), 0.f, 0.f, 0.f);
        m_Destinations.emplace_back(std::make_pair(pose, s[3]));
    }
}
