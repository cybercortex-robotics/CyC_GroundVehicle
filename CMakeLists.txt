# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

cmake_minimum_required(VERSION 3.10)
project(CyC_GroundVehicle)
set(CMAKE_CXX_STANDARD 17)

######## USER DEFINED VARIABLES ########
# Folder for building the filters DLLs
set(CyC_FILTERS_PATH "filters")

# Folder with 3rdParty libraries
set(CyC_THIRD_PARTY "/home/sorin/dev/src/CyberCortex.AI/core/3rdParty")

# Define the OpenCV version
set(CyC_OPENCV_VERSION "4.9.0")

# Define the CUDA Toolkit SDK version
set(CyC_CUDA_VERSION "11.8")

# Define the libtorch and torchvision versions
set(CyC_TORCH_VERSION "2.1.2")
set(CyC_ONNXRUNTIME_VERSION "1.16.3")
######## !USER DEFINED VARIABLES ########

if (NOT ANDROID)
    #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}/bin>)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif (NOT ANDROID)

# Add CyberCortex inference engine
add_subdirectory(CyC_inference)

if (NOT ANDROID)
    #set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}/bin>)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif (NOT ANDROID)

# Include 3rdParty directories
include_directories(${CyC_THIRD_PARTY_INCLUDE_DIRECTORIES})

include_directories(CyC_inference/include)
include_directories(CyC_inference/include/core)

# Include 3rdParty library directories
link_directories(${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

# Build Robots
add_subdirectory(CyC_Robots/src)

# Filters
add_cyc_filter(Filter_Comm_DataChannel CyC_Comm/src/Filter_Comm_DataChannel)
add_cyc_filter(Filter_HW_MonoCamera CyC_Sensors/src/Filter_HW_MonoCamera)
add_cyc_filter(Filter_HW_RgbdCamera CyC_Sensors/src/Filter_HW_RgbdCamera)
add_cyc_filter(Filter_HW_Imu CyC_Sensors/src/Filter_HW_Imu)
add_cyc_filter(Filter_HW_Lidar CyC_Sensors/src/Filter_HW_Lidar)
add_cyc_filter(Filter_HW_Gps CyC_Sensors/src/Filter_HW_Gps)
add_cyc_filter(Filter_Vision_VisualSlam CyC_Slam/src/Filter_Vision_VisualSlam)
add_cyc_filter(Filter_Environment_Map CyC_Perception/src/Filter_Environment_Map)
add_cyc_filter(Filter_Environment_SensorFusion CyC_Perception/src/Filter_Environment_SensorFusion)
add_cyc_filter(Filter_Planning_VehicleMission CyC_Planning/src/Filter_Planning_VehicleMission)
add_cyc_filter(Filter_Control_VehicleController src/Filter_Control_VehicleController)
add_cyc_filter(Filter_Control_VehicleSim src/Filter_Control_VehicleSim)
add_cyc_filter(Filter_Control_VehicleStateEstimation src/Filter_Control_VehicleStateEstimation)
add_cyc_filter(Filter_HW_VehicleActuator src/Filter_HW_VehicleActuator)
add_cyc_filter(Filter_Terminal_Vehicle src/Filter_Terminal_Vehicle)
add_cyc_filter(Filter_Visualization_Vehicle CyC_Visualization/src/Filter_Visualization_Vehicle)
add_cyc_filter(Filter_Visualization_Sensing CyC_Visualization/src/Filter_Visualization_Sensing)
add_cyc_filter(Filter_Visualization_Octoviz CyC_Visualization/src/Filter_Visualization_Octoviz)
add_cyc_filter(Filter_Daemon_CycCore CyC_Daemon/src/Filter_Daemon_CycCore)
add_cyc_filter(Filter_Terminal_CycCoreDaemon CyC_Daemon/src/Filter_Terminal_CycCoreDaemon)
