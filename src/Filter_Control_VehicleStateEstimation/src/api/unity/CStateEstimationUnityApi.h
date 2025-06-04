// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef VEH_STATE_EST_UNITY_INTERFACE_H
#define VEH_STATE_EST_UNITY_INTERFACE_H
#include "CyC_TYPES.h"
#include "enet/enet.h"
#include "control/CModelVehicle.h"

class CVehStateEstUnityApi
{
public:
    CVehStateEstUnityApi() :
        m_IpAddress(""),
        m_PortNumber(""),
        m_WaitTime(0),
        m_IsInitialized(false)
    {}
    ~CVehStateEstUnityApi();

    bool enable(std::string ip, std::string port);
    void process();
    CycState get_state() { return m_State; }

private:
    CycState m_State;
    std::string m_IpAddress;
    std::string m_PortNumber;
    ENetAddress m_Address;
    ENetHost* m_Client = nullptr;
    ENetPeer* m_Peer = nullptr;
    CyC_INT m_WaitTime;
    bool m_IsInitialized;

    void EventHandler(ENetEvent event);
    void ExtractStateData(enet_uint8* packet, size_t packetSize);
    std::vector<std::string> SplitString(std::string str, const std::string delimiter);
    std::vector<std::vector<float>> ConvertStringToFloat(std::vector<std::string> str);
};
#endif // VEH_STATE_EST_UNITY_INTERFACE_H