# Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
# Author: Sorin Mihai Grigorescu

set(FILTER_NAME Filter_Control_VehicleController)

add_library(${FILTER_NAME} SHARED
    src/helpers.h
    src/helpers.cpp
    src/CVehicleControlFilter.h
    src/CVehicleControlFilter.cpp
    src/CControllerPurePursuite.h
    src/CControllerPurePursuite.cpp
    #src/CMPCWrapper.h
    #src/CMPCWrapper.cpp
    #src/AcadoCodegen/acado_auxiliary_functions.c
    #src/AcadoCodegen/acado_solver.c
    #src/AcadoCodegen/acado_integrator.c
    #src/AcadoCodegen/acado_qpoases_interface.cpp
    #src/AcadoCodegen/qpoases/SRC/MessageHandling.cpp
    #src/AcadoCodegen/qpoases/SRC/QProblem.cpp
    #src/AcadoCodegen/qpoases/SRC/QProblemB.cpp
    #src/AcadoCodegen/qpoases/SRC/Utils.cpp
    #src/AcadoCodegen/qpoases/SRC/Bounds.cpp
    #src/AcadoCodegen/qpoases/SRC/SubjectTo.cpp
    #src/AcadoCodegen/qpoases/SRC/Indexlist.cpp
    #src/AcadoCodegen/qpoases/SRC/Constraints.cpp
    #src/AcadoCodegen/qpoases/SRC/CyclingManager.cpp
    )

target_include_directories(${FILTER_NAME}
    PRIVATE
        #${PYTHON_INCLUDE_DIRS}
        #${PYTHON_INCLUDE_DIRS}/../Lib/site-packages/numpy/core/include
        #${PYTHONHOME}/include
        #${PYTHONHOME}/Lib/site-packages/numpy/core/include
        src/AcadoCodegen
        src/AcadoCodegen/qpoases
        src/AcadoCodegen/qpoases/INCLUDE
        src/AcadoCodegen/qpoases/SRC)

target_link_directories(${FILTER_NAME}
    PUBLIC 
        ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

target_link_librarieS(${FILTER_NAME}
    PRIVATE
        #${PYTHON_LIBRARIES}
        CycCore
        CycToolkit
        libconfig++
        libcsv
        octomap
        octomath
        ${CyC_OPENCV_WORLD})

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

if (NOT GENIUS_BOARD)
    # Test unit
    add_executable(tu_VehicleController
            src/tu_VehicleController.cpp)

    target_link_directories(tu_VehicleController
        PUBLIC 
            ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

    target_link_libraries(tu_VehicleController
        PRIVATE
            #${FILTER_NAME}
            CycToolkit
            libcsv
            ${CyC_OPENCV_WORLD})
            
    add_executable(tu_CartPoleController
		src/tu_CartPoleController.cpp)

    target_include_directories(tu_CartPoleController
        PUBLIC
            ${CMAKE_SOURCE_DIR}/src/cyc_filters/Filters_Visualization/Filter_Visualization_Octoviz/src/octoviz/include
            ${CMAKE_SOURCE_DIR}/src/cyc_filters/Filters_Visualization/Filter_Visualization_Octoviz/src/octoviz/src/extern/QGLViewer
            ${CMAKE_SOURCE_DIR}/3rdParty/qt-5.12.2/win-msvc141-x64/include)

    target_link_directories(tu_CartPoleController
        PUBLIC 
            ${CyC_THIRD_PARTY_LIBRARY_DIRECTORIES})

    target_link_libraries(tu_CartPoleController
        PUBLIC
            CycToolkit
            ${CyC_OPENCV_WORLD})
endif (NOT GENIUS_BOARD)