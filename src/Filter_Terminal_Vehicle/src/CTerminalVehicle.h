// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu
 
#ifndef CTerminalVehicle_H_
#define CTerminalVehicle_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "os/qtplot/qtplot.h"
#include "os/qtplot/qtimage.h"
#include <csv_reader.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class CTerminalVehicle : public CCycFilterBase
{
public:
    enum Vehicle_Command
    {
        Vehicle_STOP = 0,
        Vehicle_DRIVE = 1,
        Vehicle_EXPLORE = 2
    };

    enum Vehicle_ControlMode
    {
        Vehicle_AUTOMATIC
    };

public:
    explicit CTerminalVehicle(CycDatablockKey key);
    explicit CTerminalVehicle(const ConfigFilterParameters& params);
    ~CTerminalVehicle() override;

    bool enable() override;
    bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

    void state_machine_thread();
    void str2destinations(const std::string& _str);

private:
    Vehicle_Command     m_VehicleCommand = Vehicle_STOP;
    Vehicle_ControlMode m_VehicleControlMode = Vehicle_AUTOMATIC;
    CyC_INT             m_IdxDestination = -1;
    CyC_ATOMIC_BOOL     m_bUpdate = false;

    std::shared_ptr<CCycQTSkeleton> m_pCcrQTSkeleton = nullptr;
    std::unique_ptr<CCycQTImage>    m_qt_image = nullptr;
    std::thread                     m_HmiThread;

    std::vector<std::pair<CPose, std::string>> m_Destinations;
};

#endif /* CTerminalVehicle_H_ */
