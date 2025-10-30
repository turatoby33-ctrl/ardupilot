cmake_minimum_required(VERSION 3.15)
project(CustomCopter VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

# Source files
set(SOURCES
    # Core
    src/core/Copter.cpp
    src/core/Scheduler.cpp
    src/core/DataTypes.cpp

    # HAL (Hardware Abstraction Layer)
    src/hal/HAL_Linux.cpp
    src/hal/GPIO.cpp
    src/hal/PWM.cpp
    src/hal/UART.cpp
    src/hal/I2C.cpp
    src/hal/SPI.cpp

    # Sensors
    src/sensors/IMU.cpp
    src/sensors/Barometer.cpp
    src/sensors/GPS.cpp
    src/sensors/Compass.cpp

    # Control
    src/control/AttitudeControl.cpp
    src/control/PositionControl.cpp
    src/control/PID.cpp
    src/control/Motors.cpp

    # Flight Modes
    src/modes/Mode.cpp
    src/modes/ModeStabilize.cpp
    src/modes/ModeAltHold.cpp
    src/modes/ModeLoiter.cpp

    # Safety
    src/safety/Arming.cpp
    src/safety/Failsafe.cpp
    src/safety/LandingDetector.cpp

    # Main
    src/main.cpp
)

# Create executable
add_executable(customcopter ${SOURCES})

# Link libraries
target_link_libraries(customcopter
    pthread
    m
)

# Installation
install(TARGETS customcopter DESTINATION bin)

# Enable testing
enable_testing()
add_subdirectory(tests)

# Print configuration
message(STATUS "")
message(STATUS "CustomCopter Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
