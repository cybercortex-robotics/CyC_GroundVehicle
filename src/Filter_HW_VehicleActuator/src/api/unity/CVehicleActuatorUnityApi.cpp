// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleActuatorUnityApi.h"
#include <json.hpp>

bool CVehicleActuatorUnityApi::enable(std::string ip, std::string port)
{
    m_IpAddress = ip;
    m_PortNumber = port;
    if (enet_initialize() != 0) {
        spdlog::error("{}: An error occurred while initializing ENet.", typeid(*this).name());
    }

    enet_address_set_host(&m_Address, m_IpAddress.c_str());
    m_Address.port = (ushort)std::atoi(m_PortNumber.c_str());

    m_Client = enet_host_create(NULL, 32, 1, 0, 0);

    if (m_Client == NULL) {
        spdlog::error("{}: An error occured while initializing Ultrasonics Interface client", typeid(*this).name());
        return false;
    }


    m_Peer = enet_host_connect(m_Client, &m_Address, 5, 0);
    if (m_Peer == NULL) {
        spdlog::error("{}: No available peers for initiating an ENet connection", typeid(*this).name());
        return false;
    }

    spdlog::info("{}: ENet connection established.", typeid(*this).name());

    m_IsInitialized = true;
    return true;
}

CVehicleActuatorUnityApi::~CVehicleActuatorUnityApi()
{
    if (m_Client != nullptr)
        enet_host_destroy(m_Client);
}

void CVehicleActuatorUnityApi::process()
{
    if (m_IsInitialized)
    {
        ENetEvent event;
        if (enet_host_service(m_Client, &event, m_WaitTime) > 0) {
            EventHandler(event);
        }
        enet_host_flush(m_Client);
    }
}

void CVehicleActuatorUnityApi::EventHandler(ENetEvent event)
{
    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        spdlog::info("{}: Connected to {}:{}", typeid(*this).name(), event.peer->address.host, event.peer->address.port);
        break;
    case ENET_EVENT_TYPE_RECEIVE:
        spdlog::info("{}: Received packet", typeid(*this).name());
        break;
    case ENET_EVENT_TYPE_DISCONNECT:
        spdlog::info("{}: '{}' disconnected", typeid(*this).name(), event.peer->data);
        /* Reset the peer's client information. */
        event.peer->data = NULL;
        break;
    case ENET_EVENT_TYPE_NONE:
        break;
    }
}

void CVehicleActuatorUnityApi::send_control(const CycControlInput& control)
{
    char j_c[1000];

    //CycControlInput tmp_u;
    //tmp_u.u = Eigen::Vector2f::Zero();
    //tmp_u.u[0] = 1.f;
    //tmp_u.u[1] = 2.f;
    //spdlog::info("{}", tmp_u.u[0]);

    sprintf(j_c, "{\"acc\":%f, \"delta\":%f}", control.u[0], control.u[1]);
    m_PacketToSend = enet_packet_create(j_c, strlen(j_c) + 1, ENET_PACKET_FLAG_RELIABLE);

    if (m_PacketToSend)
        enet_peer_send(m_Peer, 0, m_PacketToSend);
}
