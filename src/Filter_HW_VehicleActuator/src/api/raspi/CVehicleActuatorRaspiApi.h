// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CVEHICLE_ACTUATOR_RASPI_INTERFACE_H
#define CVEHICLE_ACTUATOR_RASPI_INTERFACE_H
#include "CyC_TYPES.h"
#include "enet/enet.h"
#include "os/CTimer.h"
#include "CRotaryEncoder.h"

typedef struct
{
    CyC_INT gpio;
    CyC_INT minPulse;
    CyC_INT maxPulse;
    CyC_INT pw;
    CyC_INT pwInc;
} servoInf_t;

typedef struct
{
    CyC_INT left_motor_A;
    CyC_INT left_motor_B;
    CyC_INT left_motor_E;
    CyC_INT right_motor_A;
    CyC_INT right_motor_B;
    CyC_INT right_motor_E;
} motorInf_t;

typedef struct
{
    CyC_INT left_motor_encoder_white_wire;
    CyC_INT left_motor_encoder_green_wire;
    CyC_INT right_motor_encoder_white_wire;
    CyC_INT right_motor_encoder_green_wire;
} encoderInf_t;



class CVehicleActuatorRaspiInterface
{
public:
    static CVehicleActuatorRaspiInterface& getInstance()
    {
        static CVehicleActuatorRaspiInterface instance;
        return instance;
    }

    ~CVehicleActuatorRaspiInterface();

    CyC_INT getNrOfPulsesLeftEnc();
    CyC_INT getNrOfPulsesRightEnc();
    
    bool isInitialized() { return m_IsInitialized; }

    void send_control(const CycControlInput& control);
    void moveMotorsForward(int pwm_fill);
    void moveMotorsBackward(int pwm_fill);

    CVehicleActuatorRaspiInterface(CVehicleActuatorRaspiInterface const&) = delete;
    void operator=(CVehicleActuatorRaspiInterface const&) = delete;

private:
    CVehicleActuatorRaspiInterface() :
        m_IsInitialized(false),
        m_nrOfPulsesLeftEnc(0),
        m_nrOfPulsesRightEnc(0)
    {
        spdlog::info("CVehicleActuatorRaspiInterface constructor has been called");
        enable();
    }

    bool enable();

    bool m_IsInitialized;
    servoInf_t m_servoInf;
    motorInf_t m_motorInf;
	encoderInf_t m_encoderInf;
	re_decoder m_leftEnc;
	re_decoder m_rightEnc;
    CyC_INT m_motorPwmFill;

    CyC_INT m_nrOfPulsesLeftEnc;
    CyC_INT m_nrOfPulsesRightEnc;
};
#endif //CVEHICLE_ACTUATOR_RASPI_INTERFACE_H
