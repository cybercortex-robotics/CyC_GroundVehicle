// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu
/*
 * x = [cart_position, cart_velocity, stick_angle, stick_angular_velocity] ^ T
 * y = [cart_position, stick_angular_velocity] ^ T
 * u = force applied to the cart
 */

#include <iostream>
#include "CyC_TYPES.h"
#include "os/CFileUtils.h"
#include "control/CModelCartPole.h"
#include "control/CStateSpaceController.h"
#include <opencv2/opencv.hpp>

void showUsage()
{
    printf("\nUsage:\n"
        "tu_ModelCartPole.cpp [options]\n\n"
        "Options:\n"
        "eg: tu_ModelCartPole.cpp\n");
    exit(1);
}

int main(int argc, char ** argv)
{
    /*
    // Do not use scientific notation
    std::cout << std::fixed;

    const float dt = 0.1f;

    // Cart pole model
    CModelCartPole cart_pole(0.5f, 0.2f, 0.1f, 0.3f, 0.006f);

    // State estimation is enable, since X != Y (no full state feedback control)
    CStateSpaceController controller(cart_pole, dt, true, false, false, false);

    // Initialize controller gains
    Eigen::MatrixXf K = Eigen::MatrixXf(cart_pole.getNumInputs(), cart_pole.getNumStates());
    Eigen::MatrixXf L = Eigen::MatrixXf(cart_pole.getNumStates(), cart_pole.getNumOutputs());
    
    K << -70.7107f, -37.8345f, 105.5298f, 20.9238f;
    L << 12.61f, 0.02f,
        29.51f, 2.34f,
        0.02f, 19.30f,
        -1.67f, 135.98f;

    controller.setGains(K, L);

    // Initial conditions
    cart_pole.set_x(Eigen::Vector4f{ -1.f, 2.f, 0.2f, -0.1f });

    // Set reference: Try to bring the cart to rest at 3.5m from it's home position
    controller.set_r(Eigen::Vector2f{ 3.5f, 0.0f });

    // Start the control loop
    while (1)
    {
        // Firstly generate some measurements from the simulator. Since this model assumes that only part of the state can be can be directly observed,
        // y contains two elements containing cart position and stick angle. If we were controlling an actual cart pole, these observations would come, for example,
        // from a set of encoders attached to the cart's wheels and stick joint.
        cart_pole.step(dt, controller.u());
        Eigen::VectorXf y = cart_pole.y();

        // Now update the state space controller, which causes it to update its u member (the control input). When controlling an actual system, the updated control input
        // would be used to command the cart's motor driver or similar.
        controller.update(y);

        // Print the current system output to serial. If the controller is doing its job properly then y should settle at whatever r was set to after a short transient.
        std::cout << "u: " << controller.u()(0) << ";\tCart position = " << cart_pole.getPosition() << ";\tStick angle = " << cart_pole.getPoleAngle() << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(CyC_INT(dt * SEC2MSEC)));
    }
    */
  
    return EXIT_SUCCESS;
}
