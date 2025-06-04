// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVehicleActuatorUnityApi_H_
#define CVehicleActuatorUnityApi_H_
#include "CyC_TYPES.h"
#include "enet/enet.h"
#include "os/CTimer.h"

class CVehicleActuatorUnityApi
{
public:
    CVehicleActuatorUnityApi() :
        m_IpAddress(""),
        m_PortNumber(""),
        m_WaitTime(0),
        m_IsInitialized(false)
    {}
    ~CVehicleActuatorUnityApi();

    bool    enable(std::string ip, std::string port);
    void        process();
    void        send_control(const CycControlInput& control);

private:
    std::string m_IpAddress;
    std::string m_PortNumber;
    ENetAddress m_Address;
    ENetHost*   m_Client = nullptr;
    ENetPeer*   m_Peer = nullptr;
    CyC_INT     m_WaitTime;
    bool        m_IsInitialized;
    ENetPacket* m_PacketToSend = nullptr;

    void EventHandler(ENetEvent event);
};
#endif // CVehicleActuatorUnityApi_H_