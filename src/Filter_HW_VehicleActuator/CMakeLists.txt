# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_HW_VehicleActuator)

find_package(Qt5 REQUIRED COMPONENTS Core)

if(RASPBERRY_PI)
    add_library(${FILTER_NAME} SHARED
        src/CVehicleActuator.h
        src/CVehicleActuator.cpp
        src/api/raspi/CRotaryEncoder.cpp
        src/api/raspi/CRotaryEncoder.h
        src/api/raspi/CVehicleActuatorRaspiApi.cpp
        src/api/raspi/CVehicleActuatorRaspiApi.h)
else(RASPBERRY_PI)
    add_library(${FILTER_NAME} SHARED
        src/CVehicleActuator.h
        src/CVehicleActuator.cpp
        src/api/unity/CVehicleActuatorUnityApi.cpp
        src/api/unity/CVehicleActuatorUnityApi.h)
endif(RASPBERRY_PI)

target_link_directories(${FILTER_NAME}
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_libraries(${FILTER_NAME}
    PRIVATE
        CycCore
        CycToolkit
        enet
        libcsv
        Qt5::Core
        ${CyC_OPENCV_WORLD})

if (WIN32)
    target_link_libraries(${FILTER_NAME} PUBLIC libconfig++ winmm ws2_32)
else ()
    target_link_libraries(${FILTER_NAME} PUBLIC config++)
endif()

if (NOT WIN32)
    target_link_libraries(${FILTER_NAME}
        PUBLIC
            Robot_AgileScout)
    target_compile_definitions(${FILTER_NAME}
        PRIVATE
            -DBuild_Robot_AgileScout)
endif ()

# Build test unit only on Raspberry Pi Robot
if(RASPBERRY_PI)
    # Test unit
    add_executable(tu_RaspberryPiRobotManualControl
        src/tu_RaspberryPiRobotManualControl.cpp)

    target_link_directories(tu_RaspberryPiRobotManualControl
        PUBLIC 
            ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

    target_link_libraries(tu_RaspberryPiRobotManualControl
        PUBLIC
            ${CyC_OPENCV_WORLD}
            pthread
            pigpio)
endif(RASPBERRY_PI)

# Build the filter into the filters folder
set_target_properties(${FILTER_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
)
# Override config-specific paths to keep everything in the same folder
set_target_properties(${FILTER_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"

    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"

    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CyC_FILTERS_PATH}"
)
