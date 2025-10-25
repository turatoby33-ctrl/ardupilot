cmake_minimum_required(VERSION 3.10)
project(EduCopter VERSION 1.0)

# C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Libraries
)

# Library sources
set(LIBRARY_SOURCES
    # Math
    Libraries/AP_Math/quaternion.h
    Libraries/AP_Math/vector3.h
    Libraries/AP_Math/AP_Math.h

    # HAL
    Libraries/AP_HAL/AP_HAL.h

    # Parameters
    Libraries/AP_Param/AP_Param.h
    Libraries/AP_Param/AP_Param.cpp

    # Scheduler
    Libraries/AP_Scheduler/AP_Scheduler.h
    Libraries/AP_Scheduler/AP_Scheduler.cpp

    # Sensors
    Libraries/AP_InertialSensor/AP_InertialSensor.h
    Libraries/AP_InertialSensor/AP_InertialSensor.cpp
    Libraries/AP_Baro/AP_Baro.h
    Libraries/AP_Baro/AP_Baro.cpp
    Libraries/AP_GPS/AP_GPS.h
    Libraries/AP_GPS/AP_GPS.cpp
    Libraries/AP_Compass/AP_Compass.h
    Libraries/AP_Compass/AP_Compass.cpp

    # AHRS
    Libraries/AP_AHRS/AP_AHRS.h
    Libraries/AP_AHRS/AP_AHRS.cpp

    # PID
    Libraries/AC_PID/AC_PID.h
    Libraries/AC_PID/AC_PID.cpp

    # Attitude Control
    Libraries/AC_AttitudeControl/AC_AttitudeControl.h
    Libraries/AC_AttitudeControl/AC_AttitudeControl.cpp

    # Motors
    Libraries/AP_Motors/AP_Motors.h
    Libraries/AP_Motors/AP_Motors.cpp

    # Arming
    Libraries/AP_Arming/AP_Arming.h
    Libraries/AP_Arming/AP_Arming.cpp

    # MAVLink
    Libraries/GCS_MAVLink/GCS_MAVLink.h
    Libraries/GCS_MAVLink/GCS_MAVLink.cpp

    # Logger
    Libraries/AP_Logger/AP_Logger.h
    Libraries/AP_Logger/AP_Logger.cpp

    # SITL
    Libraries/SITL/SITL_Physics.h
    Libraries/SITL/SITL_Physics.cpp
    Libraries/AP_HAL_SITL/AP_HAL_SITL.h
    Libraries/AP_HAL_SITL/AP_HAL_SITL.cpp
)

# Main vehicle sources
set(VEHICLE_SOURCES
    mode.h
    mode.cpp
    mode_stabilize.h
    mode_stabilize.cpp
    mode_althold.h
    mode_althold.cpp
    mode_land.h
    mode_land.cpp
    EduCopter.h
    EduCopter.cpp
    config.h
)

# SITL executable
add_executable(educopter_sitl
    main_sitl.cpp
    ${LIBRARY_SOURCES}
    ${VEHICLE_SOURCES}
)

# Link libraries
target_link_libraries(educopter_sitl
    pthread
    m
)

# Installation
install(TARGETS educopter_sitl
    RUNTIME DESTINATION bin
)

# Print build info
message(STATUS "")
message(STATUS "========================================")
message(STATUS "EduCopter Build Configuration")
message(STATUS "========================================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "========================================")
message(STATUS "")
