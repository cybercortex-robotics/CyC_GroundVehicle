// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <iostream>
#include <libconfig.h++>
#include "CyC_TYPES.h"
#include "os/CCsvReader.h"
#include "os/CFileUtils.h"
#include "vision/CImageDisplayUtils.h"
#include "control/CStateSpaceModelVehicle.h"
#include "control/CStateSpaceController.h"
#include "helpers.h"

// Vehicle parameters
float vehicle_length = 0.25f;

std::vector<cv::Point2f>::value_type min_pt, max_pt;
std::vector<Eigen::Vector4f> global_path;
cv::Mat cvGridImage = cv::Mat(480, 640, CV_8UC3);


void MovePose(const float& dt);
void VehiclePurePursuiteControl(const float& dt, const std::string& vehicle_model_file);
void VehicleMPCControl(const float dt);
void VehicleLQRControl(const float& dt, const std::string& vehicle_model_file);
void CartPoleExample(const float& dt);

void showUsage()
{
    printf("\nUsage:\n"
        "tu_VehicleController [options] reference_path.csv\n\n"
        "Options:\n"
        "eg: tu_VehicleController ../../etc/test_db/sensor_data_descripton.csv\n");
    exit(1);
}

void loadReferencePath(std::string strFilePath)
{
    csv::reader csv_reader;
    if (!csv_reader.open(strFilePath))
    {
        std::cout << "failed to open csv" << std::endl;
    }
    else
    {
        csv_reader.select_cols("state_variable_0", "state_variable_1", "state_variable_2", "state_variable_3");

        float x, y, v, yaw;
        int i = 0;
        while (csv_reader.read_row(x, y, v, yaw))
        {
            ++i;
            global_path.emplace_back(x, y, v, yaw);
        }

        std::cout << "No of setpoints on global path: " << global_path.size() << std::endl;
    }
}

int main(int argc, char ** argv)
{
    const float dt = 0.01f;

    if (argc < 2)
    {
        showUsage();
        return EXIT_SUCCESS;
    }

    // Load the vehicle model
    std::string sVehicleModelFile = "../etc/vehicle_models/vehicle_modelcar.conf";

    // Load the reference path file
    std::string refPathFile = argv[argc - 1];
    bool bFileExists = CFileUtils::FileExist(refPathFile.c_str());

    if (bFileExists)
    {
        loadReferencePath(refPathFile);
    }
    else
    {
        std::cout << "ERROR: Reference path file not found. Exiting." << std::endl;
        return EXIT_FAILURE;
    }

    if (global_path.size() > 0)
    {
        // Open-loop system test
        //MovePose(dt);

        // Run the unicycle vehicle example
        VehiclePurePursuiteControl(dt, sVehicleModelFile);
        //VehicleMPCControl(dt);
        //VehicleLQRControl(dt);

        // Run the car pole example
        //CartPoleExample(dt);
    }
    return EXIT_SUCCESS;
}

void VehicleMPCControl(const float dt)
{
    /*
    std::vector<Eigen::Vector2f> ref_path;
	
    // Vehicle state space model
    VehicleModel vehicle_model(vehicle_length);

    CMPCWrapper MPCController(dt, vehicle_length);
    // Simulation class
    VehicleSimulation<vehicle_model.states, vehicle_model.inputs, vehicle_model.outputs> sim(vehicle_model);
    
    // Get display dimensions
    CImageDisplayUtils::find_minmax_vehiclegrid(global_path, min_pt, max_pt);
    sim.x << static_cast<float>(global_path[0].x()), static_cast<float>(global_path[0].y()), 0.0f, 3.14f;
    
    // Measurements from the vehicle
    Eigen::VectorXf y = Eigen::VectorXf::Zero(vehicle_model.outputs);
    
    CycState state;
    state.x_hat = y;
    
    MPCController.Initialize(0.5f);
    //MPCController.SetReferenceVelocity(0.5f);
    
    ref_path.push_back(global_path[10]);
    y = sim.y = sim.x;

    // Start the control loop
    for (CyC_INT i = 0;; ++i)
    {
        // Measure the state of the vehicle
        y = sim.y;
        state.x_hat[0] = sim.y[0];
        state.x_hat[1] = sim.y[1];
        state.x_hat[2] = sim.y[3];
        state.x_hat[3] = sim.y[2];
        Eigen::VectorXf poly = get_poly_coefficients(global_path, state);
        state.x_hat[0] = 0;
        state.x_hat[1] = 0;
        state.x_hat[2] = 0;
        bool solved = MPCController.Solve(state, poly);
		if (solved)
		{
			//state = MPCController.GetState();
			// Execute control
			Eigen::VectorXf u = MPCController.GetControlSolution().u;
			sim.step(u, dt);

			float data = MPCController.GetControlSolution().u[1];
			//mpc_plot.PlotData(data, 25, 1000);

		}
        else
        {
            std::cout << "MPC could not solve" << std::endl;
        }
		

        // Plot vehicle state
        cvGridImage.setTo(cv::Scalar(0, 0, 0));
        

        CImageDisplayUtils::vehicle2grid(cvGridImage,
            global_path, ref_path,
            min_pt, max_pt,
            cv::Point2d{ sim.y(0), sim.y(1) }, sim.y(3));

        // Print vehicle state information
        int y_offset = 25;
        char str[128];
        snprintf(str, sizeof(str) - 1, "x: %.1fm; y: %.1fm; heading: %.0fdeg; velocity: %.2fm/s", sim.x(0), sim.x(1), sim.x(3) * RAD2DEG, sim.x(2));
        cv::putText(cvGridImage, str, cv::Point(5, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(200, 200, 200), 1);
        snprintf(str, sizeof(str) - 1, "rx: %.1fm; ry: %.1fm", ref_path[0].x(), ref_path[0].y());
        cv::putText(cvGridImage, str, cv::Point(5, y_offset * 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(200, 200, 200), 1);

        cv::imshow("Vehicle simulation", cvGridImage);

        if (cv::waitKey(10) == 27) // ESC
        {
            std::cout << "Exited" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
    */
}

void VehiclePurePursuiteControl(const float& dt, const std::string& vehicle_model_file)
{
    /*
    // Controller parameters
    double Kp = 1.f;                   // Speed proportional gain
    float target_speed = 10.f / 3.6f; // [m/s] [m/s] = [km/h] / 3.6
    CycReferenceSetPoints ref_path;

    //CVehicleVisualization vehicle_viz;

    // Vehicle state space model
    VehicleModel vehicle_model(vehicle_model_file);

    // Reference state
    Eigen::VectorXf r = Eigen::VectorXf(vehicle_model.outputs);
    
    // Measurements from the vehicle
    Eigen::VectorXf y = Eigen::VectorXf::Zero(vehicle_model.outputs);
    
    // Pure pursuite controller
    CPurePursuiteController PPController(dt, vehicle_model_file);
    
    // Simulation class
    VehicleSimulationAckermann<vehicle_model.states, vehicle_model.inputs, vehicle_model.outputs> sim(vehicle_model);
    
    r << global_path[20].x(), global_path[20].y(), target_speed, 0.0f;    // [x, y, v, theta]
    PPController.set_r(r);
    ref_path.ref.push_back(global_path[20]);
    
    sim.x << global_path[0].x(), global_path[0].y(), 0.f, 0.f;    // [x, y, v, theta]
    y = sim.y = sim.x;

    // Calculate control reference
    //r = PPController.reference(global_path, y);
    r = Eigen::Vector2f(5.f, 2.f);
    PPController.set_r(r);
    ref_path.ref.clear();
    ref_path.ref.push_back(Eigen::Vector2f(r[0], r[1]));
    
    // Get display dimensions
    CImageDisplayUtils::find_minmax_vehiclegrid(global_path, min_pt, max_pt);
    
    // Start the control loop
    for (CyC_INT i = 0;; ++i)
    {
        // Measure the state of the vehicle
        y = sim.y;

        // Calculate reference coordinates
        //r = PPController.reference(global_path, y);
        //PPController.set_r(r);
        //ref_path.ref.clear();
        //ref_path.ref.push_back(Eigen::Vector2f(r[0], r[1]));

        PPController.update(y);
        Eigen::VectorXf u = PPController.get_u();

        // Actuator constraints
        if (u(1) >= vehicle_model.m_fMaxSteeringAngleRad)
            u(1) = vehicle_model.m_fMaxSteeringAngleRad;
        else if (u(1) <= -vehicle_model.m_fMaxSteeringAngleRad)
            u(1) = -vehicle_model.m_fMaxSteeringAngleRad;

        // Execute control
        sim.step(u, dt);
        
        // Plot vehicle state
        cvGridImage.setTo(cv::Scalar(0, 0, 0));

        CImageDisplayUtils::vehicle2globalgrid(cvGridImage,
            global_path, global_path, ref_path,
            min_pt, max_pt,
            cv::Point2f{ sim.y(0), sim.y(1) }, sim.y(3),
            color::vehicle);

        // Print vehicle state information
        int y_offset = 25;
        char str[128];
        snprintf(str, sizeof(str) - 1, "x: %.1fm; y: %.1fm; heading: %.0fdeg; velocity: %.2fm/s", sim.x(0), sim.x(1), sim.x(3) * RAD2DEG, sim.x(2));
        cv::putText(cvGridImage, str, cv::Point(5, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(200, 200, 200), 1);
        snprintf(str, sizeof(str) - 1, "rx: %.1fm; ry: %.1fm", ref_path.ref[0].x(), ref_path.ref[0].y());
        cv::putText(cvGridImage, str, cv::Point(5, y_offset * 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(200, 200, 200), 1);
        snprintf(str, sizeof(str) - 1, "cmd acc: %.2fm; cmd steer: %.2fm", u(0), u(1));
        cv::putText(cvGridImage, str, cv::Point(5, y_offset * 3), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(200, 200, 200), 1);

        cv::imshow("Vehicle simulation", cvGridImage);

        if (cv::waitKey(10) == 27) // ESC
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
    */
}

void VehicleLQRControl(const float& dt, const std::string& vehicle_model_file)
{
    /*
    //CVehicleVisualization vehicle_viz;
    
    // Vehicle state space model
    VehicleModel model(vehicle_model_file);

    // Measurements from the vehicle
    Eigen::VectorXf y = Eigen::VectorXf::Zero(model.outputs);

    // State space controller
    CStateSpaceController<model.states, 
        model.inputs, 
        model.outputs, 
        false,                        // State estimation
        false,                        // Integral control
        false,                        // Reference tracking
        true> controller(model);      // LQR approximation

    // Simulation class
    VehicleSimulationAckermann<model.states, model.inputs, model.outputs> sim(model);

    // Once the system and gain matrices are filled in, the controller needs to be initialised so that it can precalculate Nbar
    controller.initialise();
    
    // Reference state
    //controller.r << 0., 0., 0., 0.;    // [x, y, v, theta]
    controller.r << 40., 40., 0., 0.;    // [x, y, v, theta]

    // Initial state
    sim.x << -20., 60., 0., 0.;    // [x, y, v, theta]
    sim.y = sim.x;
    
    // Start the control loop
    for (CyC_INT i = 0;; ++i)
    {
        // Measure the state of the vehicle
        y = sim.y;

        // Linearize the model at the current operating point
        model.linearizeMethod1(y(2), y(3), controller.u(1), dt);

        controller.update(y, dt);

        // Actuator constraints
        if (controller.u(1) >= model.m_fMaxSteeringAngleRad)
            controller.u(1) = model.m_fMaxSteeringAngleRad;
        else if (controller.u(1) <= -model.m_fMaxSteeringAngleRad)
            controller.u(1) = -model.m_fMaxSteeringAngleRad;

        // Execute control
        sim.step(controller.u, dt);

        // Plot data
        if (i % 5 == 0)
        {
            CycControlInput state_command;
            state_command.u = controller.u;
            //vehicle_viz.plot_state(state_command);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
    */
}

void MovePose(const float& dt, const std::string& vehicle_model_file)
{
    /*
    //CVehicleVisualization vehicle_viz;

    // Vehicle state space model
    VehicleModel model(vehicle_model_file);

    // Reference state
    Eigen::VectorXf r = Eigen::VectorXf::Zero(model.outputs);
    r << 20, 20., 0., 0.;    // [x, y, v, theta]

    // Control input
    Eigen::VectorXf u = Eigen::VectorXf(model.inputs);

    // Simulation class
    VehicleSimulationAckermann<model.states, model.inputs, model.outputs> sim(model);

    // Initial state
    sim.x << 0.f, 0.f, 0.f, 0.f;    // [x, y, v, theta]
    
    // Start the control loop
    for (CyC_INT i = 0;; ++i)
    {
        // Acceleration
        u(0) = 1.2f;

        // Steering
        u(1) = sinf(2.f * PI * i / 360.f);

        Eigen::VectorXf y = sim.y;
        sim.step(u, dt);

        // Plot data
        if (i % 10 == 0)
        {
            CycControlInput state_command;
            state_command.u = u;
            //vehicle_viz.plot_state(state_command);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
    */
}

void CartPoleExample(const float& dt)
{
    /*
    // x = [cart_position, cart_velocity, stick_angle, stick_angular_rate] ^ T
    // y = [cart_position, stick_angle] ^ T
    // u = force applied to the cart

    // Measurements from the plant
    Eigen::VectorXf y = Eigen::VectorXf::Zero(2);

    // Start by defining a state space model
    CartPoleModel model(0.5f, 0.2f, 0.1f, 0.3f, 0.006f);

    // Next define a state space controller. 
    // The cart pole model uses 4 states, 1 input and 2 outputs so we'll need to specify these inside the <> brackets when declaring the controller.
    // The controller also uses state estimation so to enable this, supply true as the fourth template parameter, like so:
    CStateSpaceController<4, 1, 2, true> controller(model);

    // Lastly, since the controller isn't controlling a cart pole, we'll need to simulate one to show how the controller works.
    // The Simulation class handles this by accepting the control inputs generated by the controller (u) and returning observations from the motor (y).
    Simulation<4, 1, 2> sim(model);

    // To parameterise the controller, we'll need to fill out the control law matrix K, and the estimator matrix L.
    // K defines feedback gains which are loosely similar to PID gains while L is equivilent to the Kalman Gain of a Kalman Filter.
    // If you're wondering where these numbers came from, head over to TuneThoseGains.ipynb
    controller.K << -70.7107f, -37.8345f, 105.5298f, 20.9238f;
    controller.L << 12.61f, 0.02f,
        29.51f, 2.34f,
        0.02f, 19.30f,
        -1.67f, 135.98f;

    // Once the system and gain matrices are filled in, the controller needs to be initialised so that it can precalculate Nbar
    controller.initialise();

    // Try to bring the cart to rest at 3.5m from it's home position
    controller.r << 3.5f, 0.0f;

    // Let's also set some initial conditions on the simulation so that the controller's estimator has to work for it
    sim.x << -1.f, 2.f, 0.2f, -0.1f;

    // Start the control loop
    while (1)
    {
        // Firstly generate some measurements from the simulator. Since this model assumes that only part of the state can be can be directly observed,
        // y contains two elements containing cart position and stick angle. If we were controlling an actual cart pole, these observations would come, for example,
        // from a set of encoders attached to the cart's wheels and stick joint.
        y = sim.step(controller.u, dt);

        // Now update the state space controller, which causes it to update its u member (the control input). When controlling an actual system, the updated control input
        // would be used to command the cart's motor driver or similar.
        controller.update(y, dt);

        // Print the current system output to serial. If the controller is doing its job properly then y should settle at whatever r was set to after a short transient.
        std::cout << "Cart position = " << y(0) << ";\tStick angle = " << y(1) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
    */
}
