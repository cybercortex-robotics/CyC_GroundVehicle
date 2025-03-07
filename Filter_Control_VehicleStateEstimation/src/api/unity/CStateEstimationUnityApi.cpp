// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CStateEstimationUnityApi.h"
#include <json.hpp>

bool CVehStateEstUnityApi::enable(std::string ip, std::string port)
{
    m_State.x_hat.resize(ModelVehicle_NumStates);

    m_IpAddress = ip;
    m_PortNumber = port;
    if (enet_initialize() != 0) {
        spdlog::error("CVehStateEstUnityInterface: An error occurred while initializing ENet.");
    }

    enet_address_set_host(&m_Address, m_IpAddress.c_str());
    m_Address.port = (ushort)std::atoi(m_PortNumber.c_str());

    m_Client = enet_host_create(NULL, 32, 1, 0, 0);

    if (m_Client == NULL) {
        spdlog::error("CVehStateEstUnityInterface: An error occured while initializing State Estimation Interface client.");
        return false;
    }

    m_Peer = enet_host_connect(m_Client, &m_Address, 5, 0);
    if (m_Peer == NULL) {
        spdlog::error("CVehStateEstUnityInterface: No available peers for initiating an ENet connection");
        return false;
    }

    m_IsInitialized = true;
    return true;
}

CVehStateEstUnityApi::~CVehStateEstUnityApi()
{
    if (m_Client != nullptr)
        enet_host_destroy(m_Client);
}

void CVehStateEstUnityApi::process()
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

void CVehStateEstUnityApi::EventHandler(ENetEvent event)
{
    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        spdlog::info("{}: Connected to {}:{}", typeid(*this).name(), event.peer->address.host, event.peer->address.port);
        break;
    case ENET_EVENT_TYPE_RECEIVE:
        ExtractStateData(event.packet->data, event.packet->dataLength);
        /* Clean up the packet now that we're done using it. */
        enet_packet_destroy(event.packet);
        break;
    case ENET_EVENT_TYPE_DISCONNECT:
        spdlog::info("{}: {} disconnected", typeid(*this).name(), event.peer->data);
        /* Reset the peer's client information. */
        event.peer->data = NULL;
        break;
    case ENET_EVENT_TYPE_NONE:
        break;
    }
}

void CVehStateEstUnityApi::ExtractStateData(enet_uint8* packet, size_t packetSize)
{
    std::string packetAllData((char*)packet, packetSize);

    nlohmann::json dataJson = nlohmann::json::parse(packetAllData);

    std::vector<std::string> packetSplitData;

    if (dataJson["CarData"].get<std::string>() != "")
    {
        packetSplitData = SplitString(dataJson["CarData"].get<std::string>(), "&");
        std::vector<std::string> allCarsData = this->SplitString(packetSplitData[0], "/");
        std::vector<float> state = ConvertStringToFloat(allCarsData)[0];

        if (state.size() != ModelVehicle_NumStates)
            spdlog::info("{}: ERROR: Received state size '{}' is different from the required state size ''{}", typeid(*this).name(), state.size(), ModelVehicle_NumStates);
        else
            for (size_t i = 0; i < state.size(); ++i)
                m_State.x_hat[i] = state[i];
    }
}

std::vector<std::string> CVehStateEstUnityApi::SplitString(std::string str, const std::string delimiter)
{
    std::vector<std::string> result;
    size_t pos = 0;
    std::string token;

    while ((pos = str.find(delimiter)) != std::string::npos) {
        token = str.substr(0, pos);
        result.push_back(token);
        str.erase(0, pos + delimiter.length());
    }
    return result;
}

std::vector<std::vector<float>> CVehStateEstUnityApi::ConvertStringToFloat(std::vector<std::string> str)
{
    std::vector<std::vector<float>> floats;

    for (CyC_INT idx = 0; idx < str.size(); idx++)
    {
        std::vector<float> localFloats;
        // for some reason i have to add another ;
        str[idx].push_back(';');
        std::vector<std::string> carData = SplitString(str[idx], ";");
        for (CyC_INT j = 0; j < carData.size(); j++)
        {
            if (carData[j] != "")
            {
                localFloats.push_back(std::stof(carData[j]));
            }
        }
        floats.push_back(localFloats);
    }

    return floats;
}
