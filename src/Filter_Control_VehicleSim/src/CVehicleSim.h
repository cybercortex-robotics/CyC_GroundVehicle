// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVEHICLE_SIM_H_
#define CVEHICLE_SIM_H_

#include "CyC_TYPES.h"
#include "CCycFilterBase.h"
#include "control/CModelVehicle.h"
#include <os/CCsvReader.h>

class CVehicleSim : public CCycFilterBase
{
public:
	explicit CVehicleSim(CycDatablockKey key);
	explicit CVehicleSim(const ConfigFilterParameters& params);

	~CVehicleSim() override;

	bool enable() override;
	bool disable() override;

private:
    bool process() override;
    void loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path) override;

private:
    bool            m_bActuatorFound = false;
    CyC_TIME_UNIT   m_prevTsActuator = 0;
    CycState        m_State;
    CCycFilterBase* m_pActuatorFilter = nullptr;

    // Vehicle parameters
    std::unique_ptr<CModelVehicle> m_pVehicleModel; // Vehicle state space model
};

#endif /* CVEHICLE_SIM_H_ */
