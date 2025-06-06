// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CVehicleStateEstimation.h"
#include <env/CGeolocation.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

CVehicleStateEstimation::CVehicleStateEstimation(const std::string& _vehicle_model_file)
{
    m_pVehicleModel = std::make_unique<CModelVehicle>(_vehicle_model_file);

    m_CurrentState.x_hat = Eigen::VectorXf::Zero(ModelVehicle_NumStates);
    m_Actuators.u = Eigen::VectorXf::Zero(ModelVehicle_NumInputs);

    m_VisualKalman.init(Eigen::MatrixXf::Zero(CVisualKalman::STATE_SIZE, 1));
    m_VisualKalmanExtended.init(Eigen::MatrixXf::Zero(CVisualKalmanExtended::STATE_SIZE, 1));
    m_OdometryKalman.init(Eigen::MatrixXf::Zero(COdometryKalman::STATE_SIZE, 1), m_pVehicleModel->m_fLongDistWheels);
}

CVehicleStateEstimation::~CVehicleStateEstimation()
{}

void CVehicleStateEstimation::setInitialState(const CycState& _init_state)
{
    m_CurrentState = _init_state;
    m_angleTransform.setInitialAngle(_init_state.x_hat[3]);

    // Init Kalman filters
    Eigen::MatrixXf state_visual_kalman;
    state_visual_kalman = Eigen::MatrixXf::Zero(CVisualKalman::STATE_SIZE, 1);
    state_visual_kalman(6) = _init_state.x_hat[0];
    state_visual_kalman(7) = _init_state.x_hat[1];
    state_visual_kalman(2) = _init_state.x_hat[3];
    m_VisualKalman.init(state_visual_kalman);
    
    Eigen::MatrixXf state_visual_kalman_extended;
    state_visual_kalman_extended = Eigen::MatrixXf::Zero(CVisualKalmanExtended::STATE_SIZE, 1);
    state_visual_kalman_extended(0) = _init_state.x_hat[0];
    state_visual_kalman_extended(1) = _init_state.x_hat[1];
    state_visual_kalman_extended(2) = _init_state.x_hat[3];
    m_VisualKalmanExtended.init(state_visual_kalman_extended);

    Eigen::MatrixXf state_odometry_kalman;
    state_odometry_kalman = Eigen::MatrixXf::Zero(COdometryKalman::STATE_SIZE, 1);
    state_odometry_kalman(0) = _init_state.x_hat[0];
    state_odometry_kalman(1) = _init_state.x_hat[1];
    state_odometry_kalman(2) = _init_state.x_hat[2];
    state_odometry_kalman(3) = _init_state.x_hat[3];
    m_OdometryKalman.init(state_odometry_kalman, m_pVehicleModel->m_fLongDistWheels);

    Eigen::MatrixXf state_gps_kalman;
    state_gps_kalman = Eigen::MatrixXf::Zero(CGPSIMUOdometryKalman::STATE_SIZE, 1);
    state_gps_kalman(0) = _init_state.x_hat[0];
    state_gps_kalman(1) = _init_state.x_hat[1];
    state_gps_kalman(2) = _init_state.x_hat[2];
    state_gps_kalman(3) = _init_state.x_hat[3];
    m_GPSIMUOdoKalman.init(state_gps_kalman);
};

void CVehicleStateEstimation::setMissionOrigin(const CycGps& gps)
{
    m_MissionOrigin = gps;
    m_bUseMissionOrigin = true;
}

bool CVehicleStateEstimation::isCurrentInputEnough() const
{
    // IMU, compass or GPS should be enough by themselves, in theory,
    // but their precision might be pretty bad, so it not recommended
    if (m_bUseIMU || m_bUseCompass || m_bUseGPS)
    {
        if ((m_tsIMU > m_tsPrevIMU) || (m_tsCompass > m_tsPrevCompass) || (m_tsGPS > m_tsPrevGPS))
            return true;
    }

    // Since SLAM can calculate both rotation and translation, it should be enough.
    if (m_bUseVisualSlam && m_bUseIMU)
    {
        if ((m_tsVisualSlam > m_tsPrevVisualSlam) && (m_tsIMU > m_tsPrevIMU))
            return true;
    }

    // Using only state data from the visual SLAM filter
    if (m_bUseVisualSlam && !m_bUseIMU)
    {
        if (m_tsVisualSlam > m_tsPrevVisualSlam)
            return true;
    }

    // Wheels encorders are only enough if there's also command available.
    // TODO: This is currently problematic because we also need to know the model.
    //       Currently, the model is not specified, but it's assumed to be
    //       the bicycle model. This should be possible to use with differential drive too.
    if (m_bUseWheelEncoders && m_bUseActuators)
    {
        // Does not need both at the same time, they can have different sampling times
        if ((m_tsWheelsEncoder > m_tsPrevWheelsEncoder) || (m_tsActuators > m_tsPrevActuators))
            return true;
    }

    return false;
}

void CVehicleStateEstimation::update()
{
    if (!isCurrentInputEnough())
    {
        //spdlog::info("Returned from iscurrentinputenough()");
        return;
    }

    float x = m_CurrentState.x_hat[0];
    float y = m_CurrentState.x_hat[1];
    float v = m_CurrentState.x_hat[2];
    float h = m_CurrentState.x_hat[3];

    // Convert imu quaternion output to euler RPY
    Eigen::Vector3f imu_euler_rpy = Eigen::Vector3f::Zero();
    // TODO: calculate quaternions from imu data
    //if (m_bUseIMU)
    //    imu_euler_rpy = CQuaternion(m_IMU.quat).to_euler_ZYX();

    if (m_bUseGPS && m_bUseWheelEncoders && m_bUseIMU)
    {
        if ((m_tsWheelsEncoder > m_tsPrevWheelsEncoder))
        {
            float delta_t = (m_tsWheelsEncoder - m_tsPrevWheelsEncoder) * MSEC2SEC;

            // Velocity
            const CyC_INT deltaNumOfPulsesLeft = m_WheelsEncoder[0] - m_lastWheelsEncoder[0];
            const CyC_INT deltaNumOfPulsesRight = m_WheelsEncoder[1] - m_lastWheelsEncoder[1];

            const float numOfRotationsLeft = deltaNumOfPulsesLeft / (float)m_pVehicleModel->m_iPulsesPerRotationLeft;
            const float numOfRotationsRight = deltaNumOfPulsesRight / (float)m_pVehicleModel->m_iPulsesPerRotationRight;

            const float distanceLeft = numOfRotationsLeft * m_pVehicleModel->m_fWheelCircumference;
            const float distanceRight = numOfRotationsRight * m_pVehicleModel->m_fWheelCircumference;

            v = (distanceLeft + distanceRight) / (2.F * delta_t);

            h = m_angleTransform.toContinuous(imu_euler_rpy.x());

            Eigen::MatrixXf u = Eigen::MatrixXf::Zero(CGPSIMUOdometryKalman::INPUT_SIZE, 1);
            u << m_IMU.acc.x(), m_IMU.acc.y(), 0.F; // TODO: use gyroZ
            m_GPSIMUOdoKalman.predict(std::move(u));

            auto local_x = 0.F;
            auto local_y = 0.F;
            const bool gpsAvailable = m_tsGPS > m_tsPrevGPS;

            if (gpsAvailable)
            {
                const auto orig = m_bUseMissionOrigin
                    ? CGeolocation::gps2utm(m_MissionOrigin.lat, m_MissionOrigin.lng, m_MissionOrigin.alt) // use mission origin
                    : CGeolocation::gps2utm(m_GPS.baseLat, m_GPS.baseLng, m_GPS.baseAlt); // use first measurement of the gps

                const auto current = CGeolocation::gps2utm(m_GPS.lat, m_GPS.lng, m_GPS.alt);
                const Eigen::Vector2f local = current - orig;
                local_x = local.x();
                local_y = local.y();

                m_tsPrevGPS = m_tsGPS;
            }

            // z = [x, y, v, h, ax, ay, rz]
            Eigen::MatrixXf z = Eigen::MatrixXf::Zero(CGPSIMUOdometryKalman::OBSERVATION_SIZE, 1);
            z << local_x, local_y, v, h, m_IMU.acc.x(), m_IMU.acc.y(), 0.F; // TODO: use gyroZ

            m_GPSIMUOdoKalman.update(std::move(z), gpsAvailable);

            // x = [x, y, v, h]
            m_CurrentState.x_hat[0] = m_GPSIMUOdoKalman.state()(0);
            m_CurrentState.x_hat[1] = m_GPSIMUOdoKalman.state()(1);
            m_CurrentState.x_hat[2] = m_GPSIMUOdoKalman.state()(2);
            m_CurrentState.x_hat[3] = m_GPSIMUOdoKalman.state()(3);

            m_tsPrevWheelsEncoder = m_tsWheelsEncoder;
            m_tsPrevIMU = m_tsIMU;
        }
    }
    // Wheel Encoders and IMU
    else if (m_bUseWheelEncoders && m_bUseIMU)
    {
        //spdlog::info("Wheel Encoders and IMU, without Compass");
        if ((m_tsWheelsEncoder > m_tsPrevWheelsEncoder) && (m_tsIMU > m_tsPrevIMU))
        {
            float delta_t = (m_tsWheelsEncoder - m_tsPrevWheelsEncoder) * MSEC2SEC;

            // Velocity
            const CyC_INT deltaNumOfPulsesLeft = m_WheelsEncoder[0] - m_lastWheelsEncoder[0];
            const CyC_INT deltaNumOfPulsesRight = m_WheelsEncoder[1] - m_lastWheelsEncoder[1];

            const float numOfRotationsLeft = deltaNumOfPulsesLeft / (float)m_pVehicleModel->m_iPulsesPerRotationLeft;
            const float numOfRotationsRight = deltaNumOfPulsesRight / (float)m_pVehicleModel->m_iPulsesPerRotationRight;

            const float distanceLeft = numOfRotationsLeft * m_pVehicleModel->m_fWheelCircumference;
            const float distanceRight = numOfRotationsRight * m_pVehicleModel->m_fWheelCircumference;

            v = (distanceLeft + distanceRight) / (2.F * delta_t);

            h = m_angleTransform.toContinuous(imu_euler_rpy.x());

            // Kalman Filter
            Eigen::MatrixXf z = Eigen::MatrixXf::Zero(COdometryKalman::OBSERVATION_SIZE, 1);
            z << v, h, 0.F, 0.F; // TODO: add steering and acceleration?

            m_OdometryKalman.predict();
            m_OdometryKalman.update(z);

            m_CurrentState.x_hat[0] = m_OdometryKalman.state()(0);
            m_CurrentState.x_hat[1] = m_OdometryKalman.state()(1);
            m_CurrentState.x_hat[2] = m_OdometryKalman.state()(2);
            m_CurrentState.x_hat[3] = m_OdometryKalman.state()(3);

            m_tsPrevWheelsEncoder = m_tsWheelsEncoder;
            m_tsPrevIMU = m_tsIMU;
        }

        if (m_bUseVisualSlam && (m_tsVisualSlam > m_tsPrevVisualSlam))
        {
            // TODO: m_VisualSlam.relative_pose is not available anymore (now is m_VisualSlam.pose); to check
            //z = [x, y, h, dx, dy, dyaw]
            Eigen::MatrixXf z = Eigen::MatrixXf::Zero(CVisualKalmanExtended::OBSERVATION_SIZE, 1);
            z << x, y, h,
                m_VisualSlam.Absolute_Body_W.translation_3x1()[0],
                m_VisualSlam.Absolute_Body_W.translation_3x1()[1],
                m_VisualSlam.Absolute_Body_W.rotation_euler()[2];

            m_VisualKalmanExtended.predict();
            m_VisualKalmanExtended.update(z);

            // speed is not included in this kalman,
            // but since it is already calculated by the other one,
            // we can keep it as it is
            m_CurrentState.x_hat[0] = m_OdometryKalman.state()(0);
            m_CurrentState.x_hat[1] = m_OdometryKalman.state()(1);
            m_CurrentState.x_hat[3] = m_OdometryKalman.state()(2);

            m_tsPrevVisualSlam = m_tsVisualSlam;
        }
    }
    // Wheel Encoders and Compass, without IMU
    else if (m_bUseWheelEncoders && m_bUseCompass && !m_bUseIMU)
    {
        //spdlog::info("Wheel Encoders and Compass, without IMU");
        if ((m_tsWheelsEncoder > m_tsPrevWheelsEncoder) && (m_tsCompass > m_tsPrevCompass))
        {
            float delta_t = (m_tsWheelsEncoder - m_tsPrevWheelsEncoder) * MSEC2SEC;

            // Velocity
            const CyC_INT deltaNumOfPulsesLeft = m_WheelsEncoder[0] - m_lastWheelsEncoder[0];
            const CyC_INT deltaNumOfPulsesRight = m_WheelsEncoder[1] - m_lastWheelsEncoder[1];

            const float numOfRotationsLeft = deltaNumOfPulsesLeft / (float)m_pVehicleModel->m_iPulsesPerRotationLeft;
            const float numOfRotationsRight = deltaNumOfPulsesRight / (float)m_pVehicleModel->m_iPulsesPerRotationRight;

            const float distanceLeft = numOfRotationsLeft * m_pVehicleModel->m_fWheelCircumference;
            const float distanceRight = numOfRotationsRight * m_pVehicleModel->m_fWheelCircumference;

            v = (distanceLeft + distanceRight) / (2.F * delta_t);

            h = m_angleTransform.toContinuous(m_Compass.at(0));

            // Kalman Filter
            Eigen::MatrixXf z = Eigen::MatrixXf::Zero(COdometryKalman::OBSERVATION_SIZE, 1);
            z << v, h, 0.F, 0.F; // TODO: add steering and acceleration?

            m_OdometryKalman.predict();
            m_OdometryKalman.update(z);

            m_CurrentState.x_hat[0] = m_OdometryKalman.state()(0);
            m_CurrentState.x_hat[1] = m_OdometryKalman.state()(1);
            m_CurrentState.x_hat[2] = m_OdometryKalman.state()(2);
            m_CurrentState.x_hat[3] = m_OdometryKalman.state()(3);

            m_tsPrevWheelsEncoder = m_tsWheelsEncoder;
            m_tsPrevCompass = m_tsCompass;
        }
    }
    else if (m_bUseVisualSlam && m_bUseIMU)
    {
        if ((m_tsVisualSlam > m_tsPrevVisualSlam) &&
            (m_tsIMU > m_tsPrevIMU))
        {
            // TODO: m_VisualSlam.relative_pose is not available anymore (now is m_VisualSlam.pose); to check
            //[r_imu, p_imu, y_imu, ax, ay, az, dr_cam, dp_cam, dy_cam, dx_cam, dy_cam, dz_cam]
            Eigen::MatrixXf z = Eigen::MatrixXf::Zero(CVisualKalman::OBSERVATION_SIZE, 1);
            z << imu_euler_rpy.z(), imu_euler_rpy.y(), imu_euler_rpy.x(),
                m_IMU.acc.x(), m_IMU.acc.y(), m_IMU.acc.z(),
                m_VisualSlam.Absolute_Body_W.rotation_euler()[0],
                m_VisualSlam.Absolute_Body_W.rotation_euler()[1],
                m_VisualSlam.Absolute_Body_W.rotation_euler()[2],
                m_VisualSlam.Absolute_Body_W.translation_3x1()[0],
                m_VisualSlam.Absolute_Body_W.translation_3x1()[1],
                m_VisualSlam.Absolute_Body_W.translation_3x1()[2];

            m_VisualKalman.predict();
            m_VisualKalman.update(z);

            //[r, p, y, dr, dp, dy, x, y, z, vx, vy, vz, scale]
            m_CurrentState.x_hat[0] = m_VisualKalman.state()(6);
            m_CurrentState.x_hat[1] = m_VisualKalman.state()(7);
            m_CurrentState.x_hat[2] = m_VisualKalman.state()(9);
            m_CurrentState.x_hat[3] = m_VisualKalman.state()(2);

            m_tsPrevVisualSlam = m_tsVisualSlam;
            m_tsPrevIMU = m_tsIMU;
        }
    }
    // Copies the state estimate received from the visual SLAM filter
    else if (m_bUseVisualSlam && !m_bUseIMU)
    {
        //spdlog::info("qqqq: {}, {}, {}", m_VisualSlam.pose.translation_3x1()[0],
        //    m_VisualSlam.pose.translation_3x1()[1],
        //    m_VisualSlam.pose.translation_3x1()[2]);

        m_CurrentState.x_hat[0] = m_VisualSlam.Absolute_Body_W.translation_3x1().x();
        m_CurrentState.x_hat[1] = m_VisualSlam.Absolute_Body_W.translation_3x1().y();
        m_CurrentState.x_hat[2] = 0.f;
        m_CurrentState.x_hat[3] = m_VisualSlam.Absolute_Body_W.rotation_euler().z();
    }
}
