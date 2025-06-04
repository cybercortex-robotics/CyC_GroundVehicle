// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

/*
 * Differential drive odometry: https://robotics.stackexchange.com/questions/1861/question-about-car-like-robot-localization-based-on-dead-reckoning
 */

#ifndef CVehicleStateEstimation_H_
#define CVehicleStateEstimation_H_

#include "CyC_TYPES.h"
#include "control/CModelVehicle.h"
#include "CVisualKalman.h"
#include "COdometryKalman.h"
#include "CVisualKalmanExtended.h"
#include "CGPSIMUOdometryKalman.h"

struct sAngleTransform
{
    sAngleTransform(float lowerBound = 0.F, float upperBound = 2 * PI)
        : m_lowerBound(lowerBound),
        m_upperBound(upperBound)
    {
    }

    void setInitialAngle(float initialAngle)
    {
        m_wraps = static_cast<int32_t>(floor(initialAngle / (m_upperBound - m_lowerBound)));
        m_lastWrappedAngle = initialAngle;
    }

    float toContinuous(float wrappedAngle)
    {
        const float diffAngle = abs(m_lastWrappedAngle - wrappedAngle);

        const auto mid = (m_upperBound - m_lowerBound) * 0.5F;
        if (diffAngle > mid)
        {
            if ((m_lastWrappedAngle > mid) && (wrappedAngle < mid))
            {
                m_wraps += 1;
            }
            else if ((m_lastWrappedAngle < mid) && (wrappedAngle > mid))
            {
                m_wraps -= 1;
            }
        }

        m_lastWrappedAngle = wrappedAngle;
        wrappedAngle += m_wraps * (m_upperBound - m_lowerBound);

        return wrappedAngle;
    }

    float fromContinuous(float continuousAngle)
    {
        const auto wraps = static_cast<int32_t>(floor(continuousAngle / (m_upperBound - m_lowerBound)));
        continuousAngle -= wraps * (m_upperBound - m_lowerBound);

        return continuousAngle;
    }

private:
    int32_t m_wraps = 0;
    float m_lastWrappedAngle = 0.F;

    float m_lowerBound;
    float m_upperBound;
};

class CVehicleStateEstimation
{
public:
    CVehicleStateEstimation(const std::string& _vehicle_model_file);
    virtual ~CVehicleStateEstimation();

    // Updates estimated state
    void update();

    // Returns the current state estimate
    void getStateEstimate(CycState& _state) { _state = m_CurrentState; };

    // Sets the initial state
    void setInitialState(const CycState& _init_state);

    void setMissionOrigin(const CycGps& gps);

    // Data update functions
    void updateActuators(const CycControlInput& _actuators, const CyC_TIME_UNIT& _ts)
    {
        m_Actuators = _actuators;
        m_tsActuators = _ts;
    }

    void updateWheelsEncoder(const std::vector<CyC_INT>& _wheels_encoder, const CyC_TIME_UNIT& _ts)
    {
        m_lastWheelsEncoder = m_WheelsEncoder;
        if (m_lastWheelsEncoder.empty() && !_wheels_encoder.empty())
        {
            m_lastWheelsEncoder.resize(_wheels_encoder.size());
            std::fill(m_lastWheelsEncoder.begin(), m_lastWheelsEncoder.end(), 0);
        }

        m_WheelsEncoder = _wheels_encoder;
        m_tsPrevWheelsEncoder = m_tsWheelsEncoder;
        m_tsWheelsEncoder = _ts;
    }

    void updateIMU(const CycImu& _imu, const CyC_TIME_UNIT& _ts)
    {
        m_prevIMU = m_IMU;
        m_IMU = _imu;
        m_tsIMU = _ts;
    }

    void updateCompass(const std::vector<float>& _compass, const CyC_TIME_UNIT& _ts)
    {
        m_prevCompass = m_Compass;
        m_Compass = _compass;
        m_tsCompass = _ts;
    }

    void updateGPS(const CycGps& _gps, const CyC_TIME_UNIT& _ts)
    {
        m_GPS = _gps;
        m_tsGPS = _ts;
    }

    void updateVisualSlam(const CycSlam& _pose, const CyC_TIME_UNIT& _ts)
    {
        m_VisualSlam = _pose;
        m_tsVisualSlam = _ts;
    }

    void setAvailableInput(bool useActuators, bool useWheelsEncoders, bool useIMU, bool useCompass, bool useGPS, bool useVisualSlam)
    {
        m_bUseActuators = useActuators;
        m_bUseWheelEncoders = useWheelsEncoders;
        m_bUseIMU = useIMU;
        m_bUseCompass = useCompass;
        m_bUseGPS = useGPS;
        m_bUseVisualSlam = useVisualSlam;
    }

private:
    bool isCurrentInputEnough() const;

    CycState m_CurrentState;

    // Sensory input data
    // Actuator filter: can be either a control or an actuator filter
    CycControlInput m_Actuators;
    CyC_TIME_UNIT   m_tsActuators = 0;
    CyC_TIME_UNIT   m_tsPrevActuators = 0;
    bool        m_bUseActuators = false;

    // Rotary wheels encoder
    std::vector<CyC_INT> m_WheelsEncoder;
    std::vector<CyC_INT> m_lastWheelsEncoder;
    CyC_TIME_UNIT        m_tsWheelsEncoder = 0;
    CyC_TIME_UNIT        m_tsPrevWheelsEncoder = 0;
    bool             m_bUseWheelEncoders = false;

    // Inertial sensor
    CycImu        m_IMU;
    CycImu        m_prevIMU;
    CyC_TIME_UNIT m_tsIMU = 0;
    CyC_TIME_UNIT m_tsPrevIMU = 0;
    bool      m_bUseIMU = false;
    sAngleTransform m_angleTransform;

    // Compass sensor
    std::vector<float> m_Compass;
    std::vector<float> m_prevCompass;
    CyC_TIME_UNIT          m_tsCompass = 0;
    CyC_TIME_UNIT          m_tsPrevCompass = 0;
    bool               m_bUseCompass = false;

    // GPS sensor
    CycGps        m_GPS;
    CyC_TIME_UNIT m_tsGPS = 0;
    CyC_TIME_UNIT m_tsPrevGPS = 0;
    bool      m_bUseGPS = false;

    // Visual slam
    CycSlam       m_VisualSlam;
    CyC_TIME_UNIT m_tsVisualSlam = 0;
    CyC_TIME_UNIT m_tsPrevVisualSlam = 0;
    bool      m_bUseVisualSlam = false;

    // Localization origin
    CycGps        m_MissionOrigin;
    bool      m_bUseMissionOrigin = false;

    // State estimation variables
    std::unique_ptr<CModelVehicle> m_pVehicleModel;
    float m_DistLeft = 0;
    float m_DistRight = 0;

    COdometryKalman m_OdometryKalman;
    CVisualKalman m_VisualKalman;
    CVisualKalmanExtended m_VisualKalmanExtended;
    CGPSIMUOdometryKalman m_GPSIMUOdoKalman;
};

#endif // CVehicleStateEstimation_H_ 
