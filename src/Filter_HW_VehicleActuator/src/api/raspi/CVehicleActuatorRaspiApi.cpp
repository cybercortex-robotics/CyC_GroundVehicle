// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu
 
#include "CVehicleActuatorRaspiInterface.h"
#include <pigpio.h>
#include <json.hpp>

bool CVehicleActuatorRaspiInterface::enable()
{
    if (m_IsInitialized)
    {
        spdlog::error("RaspiInterface already initialized");
        return false;
    }
    
     // Init GPIO ports
    if (gpioInitialise() < 0)
    {
        spdlog::error("An error occurred while initializing GPIO ports.");
        return false;
    }

	// Initialize motor information
    m_motorInf.left_motor_A = 13; // Forward
    m_motorInf.left_motor_B = 19; // Reverse
    m_motorInf.left_motor_E = 24; // Enable
    m_motorInf.right_motor_A = 20;
    m_motorInf.right_motor_B = 21;
    m_motorInf.right_motor_E = 23;

    // Set pin directions
    gpioSetMode(m_motorInf.left_motor_A, PI_OUTPUT);
    gpioSetMode(m_motorInf.left_motor_B, PI_OUTPUT);
    gpioSetMode(m_motorInf.left_motor_E, PI_OUTPUT);
    gpioSetMode(m_motorInf.right_motor_A, PI_OUTPUT);
    gpioSetMode(m_motorInf.right_motor_B, PI_OUTPUT);
    gpioSetMode(m_motorInf.right_motor_E, PI_OUTPUT);

    m_motorPwmFill = 0;

    // Initialize servo information
    m_servoInf.gpio = 18;
    m_servoInf.minPulse = 1000;
    m_servoInf.maxPulse = 2000;
    m_servoInf.pw = 1500;
    m_servoInf.pwInc = 30;
	
	// Initialize encoder information
	m_encoderInf.left_motor_encoder_white_wire = 22;
    m_encoderInf.left_motor_encoder_green_wire = 27;
    m_encoderInf.right_motor_encoder_white_wire = 6;
    m_encoderInf.right_motor_encoder_green_wire = 5;

    //Encoders
	m_leftEnc.re_init_left(m_encoderInf.left_motor_encoder_white_wire, m_encoderInf.left_motor_encoder_green_wire);
	m_rightEnc.re_init_right(m_encoderInf.right_motor_encoder_white_wire, m_encoderInf.right_motor_encoder_green_wire);
    spdlog::info("Encoders are enabled");

    m_IsInitialized = true;
    return m_IsInitialized;
}

CVehicleActuatorRaspiInterface::~CVehicleActuatorRaspiInterface()
{
    m_leftEnc.re_cancel();
    m_rightEnc.re_cancel();
    gpioTerminate();
}

CyC_INT CVehicleActuatorRaspiInterface::getNrOfPulsesLeftEnc()
{
    return m_leftEnc.getNrOfPulsesLeftEnc();
}

CyC_INT CVehicleActuatorRaspiInterface::getNrOfPulsesRightEnc()
{
    return m_rightEnc.getNrOfPulsesRightEnc();
}

void CVehicleActuatorRaspiInterface::send_control(const CycControlInput& control)
{
    // -------------------
    // Motors
    m_motorPwmFill = (CyC_INT)(control.u[0] * 500);
    
    /*if(control.u[0] < 0)
    {
        m_motorPwmFill = 0;
    }
    
	
	// m_motorPwmFill is 255 = maximum speed forward
	// m_motorPwmFill is -255 = maximum speed backward 
	// Both moveMotorsBackward and moveMotorsForward have positive PWM fill as parameter
	// The rotation direction is commanded via left_motor_A, B and right_motor_A, B pin values
	// control.u[0] = linear acceleration
	// control.u[1] = steering angle (in radians)
	
    if(m_motorPwmFill > 0 && m_motorPwmFill < 120)
    {
        m_motorPwmFill = 120;
    }
    
    if(m_motorPwmFill < 0 && m_motorPwmFill < -120)
    {
        m_motorPwmFill = -120;
    }*/

    // Check for motor limits
    if(m_motorPwmFill > 255)
    {
        m_motorPwmFill = 255;
    }
    if(m_motorPwmFill < -255)
    {
        m_motorPwmFill = -255;
    }

    if(m_motorPwmFill > 0)
    {
        moveMotorsForward(m_motorPwmFill);
    }
    else
    {
        moveMotorsBackward(-m_motorPwmFill);
    }

    // ---------------
    // Steering
    float steering_angle_degrees = control.u[1] * RADTODEG;

    // pulse width 1000 = -35 degrees,
    // pulse width 1500 = 0 degrees
    // pulse width 2000 = +35 degrees
    float st_angle_pwinc = 500.F / 35.F; // pw_inc for each degree
    m_servoInf.pw = 1500 + CyC_INT(steering_angle_degrees * st_angle_pwinc);

    // Check for servo limits
    if(m_servoInf.pw < m_servoInf.minPulse)
    {
        m_servoInf.pw = m_servoInf.minPulse;
    }
    if(m_servoInf.pw > m_servoInf.maxPulse)
    {
        m_servoInf.pw = m_servoInf.maxPulse;
    }

    // Move the servo with the steering pw value
    gpioServo(m_servoInf.gpio, m_servoInf.pw);
}

void CVehicleActuatorRaspiInterface::moveMotorsForward(int pwm_fill)
{
    gpioWrite(m_motorInf.left_motor_A, 1);
    gpioWrite(m_motorInf.left_motor_B, 0);
    gpioWrite(m_motorInf.right_motor_A, 1);
    gpioWrite(m_motorInf.right_motor_B , 0);
    gpioPWM(m_motorInf.left_motor_E, pwm_fill);
    gpioPWM(m_motorInf.right_motor_E , pwm_fill);
}

void CVehicleActuatorRaspiInterface::moveMotorsBackward(int pwm_fill)
{
    gpioWrite(m_motorInf.left_motor_A, 0);
    gpioWrite(m_motorInf.left_motor_B, 1);
    gpioWrite(m_motorInf.right_motor_A, 0);
    gpioWrite(m_motorInf.right_motor_B , 1);
    gpioPWM(m_motorInf.left_motor_E, pwm_fill);
    gpioPWM(m_motorInf.right_motor_E , pwm_fill);
}
