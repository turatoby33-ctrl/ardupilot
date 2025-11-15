# GCS_MAVLink_Pro Integration Guide

## Complete Guide to Integrating ArduPilot Libraries

This comprehensive guide shows you how to integrate all analyzed ArduPilot libraries into your GCS_MAVLink_Pro project, with step-by-step instructions, code examples, and best practices.

---

## Table of Contents

1. [Project Setup](#1-project-setup)
2. [Minimal Integration (Quick Start)](#2-minimal-integration-quick-start)
3. [Full Integration (All Libraries)](#3-full-integration-all-libraries)
4. [Platform Selection](#4-platform-selection)
5. [Build System Setup](#5-build-system-setup)
6. [Initialization Patterns](#6-initialization-patterns)
7. [Common Integration Patterns](#7-common-integration-patterns)
8. [Custom Library Implementation](#8-custom-library-implementation)
9. [Testing Strategies](#9-testing-strategies)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Project Setup

### 1.1 Directory Structure

Recommended project structure for GCS_MAVLink_Pro:

```
GCS_MAVLink_Pro/
├── libraries/              # ArduPilot libraries (submodule or copy)
│   ├── AP_HAL/
│   ├── AP_HAL_Linux/      # Or your chosen platform
│   ├── AP_Common/
│   ├── AP_Math/
│   ├── AP_Param/
│   ├── AP_SerialManager/
│   ├── AP_Scheduler/
│   ├── AP_Arming/
│   ├── RC_Channel/
│   ├── AP_Mount/
│   ├── AP_Vehicle/
│   └── SITL/              # For testing
├── src/                   # Your application code
│   ├── main.cpp
│   ├── GCS_MAVLink_Pro.h
│   └── GCS_MAVLink_Pro.cpp
├── include/               # Your headers
├── config/                # Configuration files
│   └── hwdef.dat         # Hardware definition
├── build/                 # Build output
├── CMakeLists.txt        # Or Makefile
└── README.md
```

### 1.2 Obtaining ArduPilot Libraries

**Option 1: Git Submodule (Recommended)**
```bash
cd GCS_MAVLink_Pro
git submodule add https://github.com/ArduPilot/ardupilot.git libraries/ardupilot
git submodule update --init --recursive

# Create symlinks to needed libraries
cd libraries
ln -s ardupilot/libraries/AP_HAL AP_HAL
ln -s ardupilot/libraries/AP_Common AP_Common
# ... repeat for other libraries
```

**Option 2: Selective Copy**
```bash
# Copy only needed libraries
cp -r /path/to/ardupilot/libraries/AP_HAL libraries/
cp -r /path/to/ardupilot/libraries/AP_Common libraries/
cp -r /path/to/ardupilot/libraries/AP_Math libraries/
# ... etc
```

---

## 2. Minimal Integration (Quick Start)

### 2.1 Minimum Required Libraries

For a basic GCS_MAVLink_Pro that communicates via MAVLink:

```
Required:
├── AP_HAL (interface)
├── AP_HAL_Linux (or your platform)
├── AP_Common (utilities)
├── AP_Math (basic math)
└── AP_Param (configuration)

Optional but Recommended:
└── AP_SerialManager (serial port management)
```

### 2.2 Minimal main.cpp

```cpp
// File: src/main.cpp
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

// Select your HAL platform
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include <AP_HAL_Linux/HAL_Linux_Class.h>
    const AP_HAL::HAL& hal = AP_HAL_Linux::get_HAL();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #include <AP_HAL_SITL/HAL_SITL_Class.h>
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
#endif

// Your application class
class GCS_MAVLink_Pro {
public:
    void setup();
    void loop();

private:
    AP_Float version;
    AP_Int32 baudrate;

    static const struct AP_Param::GroupInfo var_info[];
};

const struct AP_Param::GroupInfo GCS_MAVLink_Pro::var_info[] = {
    AP_GROUPINFO("VERSION", 0, GCS_MAVLink_Pro, version, 1.0f),
    AP_GROUPINFO("BAUD", 1, GCS_MAVLink_Pro, baudrate, 57600),
    AP_GROUPEND
};

// Global instance
static GCS_MAVLink_Pro gcs_mavlink_pro;

void GCS_MAVLink_Pro::setup() {
    // Initialize HAL
    hal.console->printf("GCS_MAVLink_Pro starting...\n");

    // Load parameters
    AP_Param::setup_sketch_defaults();
    if (!AP_Param::check_var_info()) {
        hal.console->printf("ERROR: parameter table error\n");
        return;
    }
    AP_Param::load_all();

    hal.console->printf("Setup complete. Version: %.2f\n", version.get());
}

void GCS_MAVLink_Pro::loop() {
    // Main loop - 10 Hz
    hal.scheduler->delay(100);

    // Your GCS logic here
    hal.console->printf("Loop running...\n");
}

// HAL setup and main loop
void setup() {
    gcs_mavlink_pro.setup();
}

void loop() {
    gcs_mavlink_pro.loop();
}

// Entry point
AP_HAL_MAIN();
```

### 2.3 Minimal CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(GCS_MAVLink_Pro)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define HAL board
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_LINUX)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/libraries/AP_HAL
    ${CMAKE_SOURCE_DIR}/libraries/AP_HAL_Linux
    ${CMAKE_SOURCE_DIR}/libraries/AP_Common
    ${CMAKE_SOURCE_DIR}/libraries/AP_Math
    ${CMAKE_SOURCE_DIR}/libraries/AP_Param
)

# Source files
file(GLOB_RECURSE AP_HAL_SOURCES
    libraries/AP_HAL/*.cpp
    libraries/AP_HAL_Linux/*.cpp
    libraries/AP_Common/*.cpp
    libraries/AP_Math/*.cpp
    libraries/AP_Param/*.cpp
)

add_executable(gcs_mavlink_pro
    src/main.cpp
    ${AP_HAL_SOURCES}
)

target_link_libraries(gcs_mavlink_pro
    pthread
    rt
    m
)
```

### 2.4 Build and Run

```bash
mkdir build && cd build
cmake ..
make

# Run
./gcs_mavlink_pro
```

---

## 3. Full Integration (All Libraries)

### 3.1 Complete Application Structure

```cpp
// File: src/GCS_MAVLink_Pro.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Arming/AP_Arming.h>

class GCS_MAVLink_Pro {
public:
    friend class GCS_MAVLink_Pro_Arming;

    // Constructor
    GCS_MAVLink_Pro();

    // Initialization
    void setup();
    void loop();

    // Main loop tasks
    void fast_loop();           // 100 Hz
    void update_telemetry();    // 10 Hz
    void update_sensors();      // 50 Hz
    void check_arming();        // 1 Hz

private:
    // Core systems
    AP_SerialManager serial_manager;
    AP_Scheduler scheduler;
    RC_Channels_MAVLink rc;

    // Parameters
    AP_Float version;
    AP_Int32 telemetry_baud;
    AP_Int8 system_id;

    // State
    bool armed;
    uint32_t loop_count;

    // Parameter table
    static const struct AP_Param::GroupInfo var_info[];

    // Scheduler tasks
    static const AP_Scheduler::Task scheduler_tasks[];
};

// Custom arming checks
class GCS_MAVLink_Pro_Arming : public AP_Arming {
public:
    GCS_MAVLink_Pro_Arming(GCS_MAVLink_Pro& _gcs)
        : AP_Arming(), gcs(_gcs) {}

    bool pre_arm_checks(bool report) override;

private:
    GCS_MAVLink_Pro& gcs;
};
```

```cpp
// File: src/GCS_MAVLink_Pro.cpp
#include "GCS_MAVLink_Pro.h"

extern const AP_HAL::HAL& hal;

// Parameter table
const struct AP_Param::GroupInfo GCS_MAVLink_Pro::var_info[] = {
    AP_GROUPINFO("VERSION", 0, GCS_MAVLink_Pro, version, 1.0f),
    AP_GROUPINFO("TELEM_BAUD", 1, GCS_MAVLink_Pro, telemetry_baud, 57600),
    AP_GROUPINFO("SYSID", 2, GCS_MAVLink_Pro, system_id, 1),

    // Sub-objects
    AP_SUBGROUPINFO(serial_manager, "SERIAL", 3, GCS_MAVLink_Pro, AP_SerialManager),
    AP_SUBGROUPINFO(rc, "RC", 4, GCS_MAVLink_Pro, RC_Channels_MAVLink),

    AP_GROUPEND
};

// Scheduler tasks (rate, max_time_us, priority)
const AP_Scheduler::Task GCS_MAVLink_Pro::scheduler_tasks[] = {
    SCHED_TASK(fast_loop,         100,  200,  3),  // 100 Hz
    SCHED_TASK(update_sensors,     50,  400,  6),  // 50 Hz
    SCHED_TASK(update_telemetry,   10,  500, 12),  // 10 Hz
    SCHED_TASK(check_arming,        1,  100, 15),  // 1 Hz
};

GCS_MAVLink_Pro::GCS_MAVLink_Pro()
    : armed(false)
    , loop_count(0)
{
}

void GCS_MAVLink_Pro::setup() {
    hal.console->printf("\n\nGCS_MAVLink_Pro Initializing...\n");

    // 1. Board-specific setup
    hal.scheduler->init();

    // 2. Load parameters
    AP_Param::setup_sketch_defaults();
    if (!AP_Param::check_var_info()) {
        hal.console->printf("ERROR: Bad parameter table\n");
        return;
    }
    AP_Param::load_all();

    hal.console->printf("Version: %.2f\n", version.get());
    hal.console->printf("System ID: %d\n", (int)system_id.get());

    // 3. Initialize serial manager
    serial_manager.init();

    // 4. Initialize RC channels
    rc.init();

    // 5. Initialize scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), 0);

    hal.console->printf("Setup complete!\n\n");
}

void GCS_MAVLink_Pro::loop() {
    // Run scheduler
    scheduler.loop();

    // Update loop counter
    loop_count++;
}

void GCS_MAVLink_Pro::fast_loop() {
    // Read RC inputs
    rc.read_input();

    // Update state
    // ...
}

void GCS_MAVLink_Pro::update_sensors() {
    // Read sensors
    // ...
}

void GCS_MAVLink_Pro::update_telemetry() {
    // Send MAVLink messages
    hal.console->printf("Loop: %lu, Armed: %d\n",
                        (unsigned long)loop_count, armed);
}

void GCS_MAVLink_Pro::check_arming() {
    // Check arming conditions
    // ...
}

// Custom arming checks
bool GCS_MAVLink_Pro_Arming::pre_arm_checks(bool report) {
    // Call base class checks
    if (!AP_Arming::pre_arm_checks(report)) {
        return false;
    }

    // Custom checks for GCS_MAVLink_Pro
    if (gcs.telemetry_baud < 9600) {
        check_failed(report, "Baud rate too low");
        return false;
    }

    return true;
}

// Entry point
static GCS_MAVLink_Pro gcs_app;

void setup() {
    gcs_app.setup();
}

void loop() {
    gcs_app.loop();
}

AP_HAL_MAIN();
```

---

## 4. Platform Selection

### 4.1 Linux Platform (Raspberry Pi, BeagleBone)

```cpp
// In your build configuration
#define CONFIG_HAL_BOARD HAL_BOARD_LINUX
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_LINUX_NONE

// Include
#include <AP_HAL_Linux/HAL_Linux_Class.h>

// Main file
const AP_HAL::HAL& hal = AP_HAL_Linux::get_HAL();
```

**CMake configuration**:
```cmake
add_definitions(
    -DCONFIG_HAL_BOARD=HAL_BOARD_LINUX
    -DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_NONE
)

include_directories(libraries/AP_HAL_Linux)

file(GLOB_RECURSE HAL_LINUX_SOURCES
    libraries/AP_HAL_Linux/*.cpp
)
```

### 4.2 ESP32 Platform

```cpp
// In sdkconfig or build config
#define CONFIG_HAL_BOARD HAL_BOARD_ESP32

// Include
#include <AP_HAL_ESP32/HAL_ESP32_Class.h>

// Main file
const AP_HAL::HAL& hal = AP_HAL_ESP32::get_HAL();
```

**Build with ESP-IDF**:
```bash
# In your ESP32 project
idf.py build flash monitor
```

### 4.3 SITL (Testing/Simulation)

```cpp
// In build configuration
#define CONFIG_HAL_BOARD HAL_BOARD_SITL

// Include
#include <AP_HAL_SITL/HAL_SITL_Class.h>
#include <SITL/SITL.h>

// Main file
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
```

**Running SITL**:
```bash
# Build for SITL
./waf configure --board sitl
./waf build --target bin/arducopter

# Or use sim_vehicle
./Tools/autotest/sim_vehicle.py -v Copter --console --map
```

---

## 5. Build System Setup

### 5.1 CMake Build System (Recommended)

Full `CMakeLists.txt` for all libraries:

```cmake
cmake_minimum_required(VERSION 3.10)
project(GCS_MAVLink_Pro CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Platform selection
option(BUILD_FOR_LINUX "Build for Linux" ON)
option(BUILD_FOR_ESP32 "Build for ESP32" OFF)
option(BUILD_FOR_SITL "Build for SITL" OFF)

if(BUILD_FOR_LINUX)
    add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_LINUX)
    set(HAL_PLATFORM "Linux")
elseif(BUILD_FOR_SITL)
    add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_SITL)
    set(HAL_PLATFORM "SITL")
endif()

# Feature flags
add_definitions(
    -DAP_PARAM_ENABLED=1
    -DAP_SCHEDULER_ENABLED=1
    -DHAL_LOGGING_ENABLED=0
    -DHAL_WITH_DSP=0
)

# Include directories
set(LIBRARY_DIR ${CMAKE_SOURCE_DIR}/libraries)

include_directories(
    ${LIBRARY_DIR}/AP_HAL
    ${LIBRARY_DIR}/AP_HAL_${HAL_PLATFORM}
    ${LIBRARY_DIR}/AP_Common
    ${LIBRARY_DIR}/AP_Math
    ${LIBRARY_DIR}/AP_Param
    ${LIBRARY_DIR}/AP_SerialManager
    ${LIBRARY_DIR}/AP_Scheduler
    ${LIBRARY_DIR}/AP_Arming
    ${LIBRARY_DIR}/RC_Channel
    ${LIBRARY_DIR}/AP_Mount
    ${LIBRARY_DIR}/AP_Vehicle
    ${CMAKE_SOURCE_DIR}/src
)

# Collect source files
file(GLOB_RECURSE SOURCES
    ${LIBRARY_DIR}/AP_HAL/*.cpp
    ${LIBRARY_DIR}/AP_HAL_${HAL_PLATFORM}/*.cpp
    ${LIBRARY_DIR}/AP_Common/*.cpp
    ${LIBRARY_DIR}/AP_Math/*.cpp
    ${LIBRARY_DIR}/AP_Param/*.cpp
    ${LIBRARY_DIR}/AP_SerialManager/*.cpp
    ${LIBRARY_DIR}/AP_Scheduler/*.cpp
    ${LIBRARY_DIR}/AP_Arming/*.cpp
    ${LIBRARY_DIR}/RC_Channel/*.cpp
    ${LIBRARY_DIR}/AP_Mount/*.cpp
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    pthread
    rt
    m
)

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
```

### 5.2 Makefile Build System

```makefile
# Makefile for GCS_MAVLink_Pro

# Platform selection
HAL_BOARD ?= LINUX
# HAL_BOARD = SITL
# HAL_BOARD = ESP32

# Compiler
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2

# Definitions
CXXFLAGS += -DCONFIG_HAL_BOARD=HAL_BOARD_$(HAL_BOARD)
CXXFLAGS += -DAP_PARAM_ENABLED=1
CXXFLAGS += -DAP_SCHEDULER_ENABLED=1

# Directories
LIB_DIR = libraries
SRC_DIR = src
BUILD_DIR = build

# Include paths
INCLUDES = -I$(LIB_DIR)/AP_HAL \
           -I$(LIB_DIR)/AP_HAL_$(HAL_BOARD) \
           -I$(LIB_DIR)/AP_Common \
           -I$(LIB_DIR)/AP_Math \
           -I$(LIB_DIR)/AP_Param \
           -I$(LIB_DIR)/AP_SerialManager \
           -I$(LIB_DIR)/AP_Scheduler \
           -I$(LIB_DIR)/AP_Arming \
           -I$(LIB_DIR)/RC_Channel \
           -I$(SRC_DIR)

# Source files
SOURCES = $(shell find $(LIB_DIR)/AP_HAL -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_HAL_$(HAL_BOARD) -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_Common -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_Math -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_Param -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_SerialManager -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_Scheduler -name '*.cpp') \
          $(shell find $(LIB_DIR)/AP_Arming -name '*.cpp') \
          $(shell find $(LIB_DIR)/RC_Channel -name '*.cpp') \
          $(shell find $(SRC_DIR) -name '*.cpp')

# Objects
OBJECTS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(notdir $(SOURCES)))

# Libraries
LIBS = -lpthread -lrt -lm

# Target
TARGET = gcs_mavlink_pro

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) $(TARGET)
```

---

## 6. Initialization Patterns

### 6.1 Standard Initialization Sequence

Follow this order in your `setup()` function:

```cpp
void GCS_MAVLink_Pro::setup() {
    // 1. HAL initialization (automatic via AP_HAL_MAIN)
    // Already done before setup() is called

    // 2. Console output for debugging
    hal.console->printf("Starting GCS_MAVLink_Pro...\n");

    // 3. Board-specific initialization
    hal.scheduler->init();
    hal.gpio->init();

    // 4. Parameter system
    AP_Param::setup_sketch_defaults();
    if (!AP_Param::check_var_info()) {
        hal.console->printf("ERROR: Parameter table\n");
        return;
    }
    AP_Param::load_all();

    // 5. Serial manager (before devices that need serial)
    serial_manager.init();

    // 6. Initialize subsystems
    rc.init();
    // ... other subsystems

    // 7. Arming checks initialization
    arming.init(AP_Arming::ArmingMethod::RUDDER);

    // 8. Scheduler initialization (last)
    scheduler.init(&scheduler_tasks[0],
                   ARRAY_SIZE(scheduler_tasks),
                   0);  // log_bit_mask

    hal.console->printf("Initialization complete!\n");
}
```

### 6.2 Parameter Setup

```cpp
// Define default parameters
void setup_defaults() {
    // Set default parameters if not already set
    if (!version.load()) {
        version.set_and_save(1.0f);
    }

    if (!telemetry_baud.load()) {
        telemetry_baud.set_and_save(57600);
    }
}

// In setup():
AP_Param::setup_sketch_defaults();
setup_defaults();
AP_Param::load_all();
```

### 6.3 Scheduler Task Registration

```cpp
// Define tasks with rates and priorities
const AP_Scheduler::Task GCS_MAVLink_Pro::scheduler_tasks[] = {
    // Function          Rate(Hz)  MaxTime(μs)  Priority
    SCHED_TASK(fast_loop,      100,      200,       3),
    SCHED_TASK(medium_loop,     50,      400,       6),
    SCHED_TASK(slow_loop,       10,      500,      12),
    SCHED_TASK(very_slow_loop,   1,      100,      15),
};

// In setup():
scheduler.init(&scheduler_tasks[0],
               ARRAY_SIZE(scheduler_tasks),
               0);  // log bitmask

// In loop():
void loop() {
    scheduler.loop();  // Calls tasks at their rates
}
```

---

## 7. Common Integration Patterns

### 7.1 Using Serial Ports (AP_SerialManager)

```cpp
class GCS_MAVLink_Pro {
private:
    AP_SerialManager serial_manager;
    AP_HAL::UARTDriver *telemetry_port;
    AP_HAL::UARTDriver *gps_port;
};

void GCS_MAVLink_Pro::setup() {
    // Initialize serial manager
    serial_manager.init();

    // Find telemetry port (MAVLink protocol, instance 0)
    telemetry_port = serial_manager.find_serial(
        AP_SerialManager::SerialProtocol_MAVLink, 0);

    if (telemetry_port != nullptr) {
        hal.console->printf("Telemetry on port %d at %d baud\n",
            serial_manager.find_portnum(
                AP_SerialManager::SerialProtocol_MAVLink, 0),
            serial_manager.find_baudrate(
                AP_SerialManager::SerialProtocol_MAVLink, 0)
        );
    }

    // Find GPS port
    gps_port = serial_manager.find_serial(
        AP_SerialManager::SerialProtocol_GPS, 0);
}

void GCS_MAVLink_Pro::update_telemetry() {
    if (telemetry_port && telemetry_port->available()) {
        // Read incoming MAVLink
        uint8_t byte = telemetry_port->read();
        // Process...
    }

    // Send MAVLink
    if (telemetry_port) {
        telemetry_port->write(data, length);
    }
}
```

### 7.2 Using RC Channels

```cpp
#include <RC_Channel/RC_Channel.h>

class GCS_MAVLink_Pro {
private:
    RC_Channels_MAVLink rc;
};

void GCS_MAVLink_Pro::setup() {
    // Initialize RC
    rc.init();

    // Set channel mapping
    // RC1 = Roll, RC2 = Pitch, RC3 = Throttle, RC4 = Yaw
}

void GCS_MAVLink_Pro::fast_loop() {
    // Read RC inputs (call every loop)
    rc.read_input();

    // Get channel values
    RC_Channel *roll_ch = rc.channel(0);  // RC1
    if (roll_ch) {
        float roll_input = roll_ch->norm_input();  // -1.0 to 1.0
        uint16_t roll_pwm = roll_ch->get_radio_in();  // PWM value

        hal.console->printf("Roll: %.2f (%d PWM)\n",
                            roll_input, roll_pwm);
    }

    // Check for failsafe
    if (rc.has_new_overrides()) {
        hal.console->printf("RC override active\n");
    }
}

// Handle MAVLink RC override
void handle_rc_override(mavlink_message_t *msg) {
    mavlink_rc_channels_override_t override;
    mavlink_msg_rc_channels_override_decode(msg, &override);

    rc.set_override(0, override.chan1_raw, 0);  // Channel 1
    rc.set_override(1, override.chan2_raw, 0);  // Channel 2
    // ... etc
}
```

### 7.3 Using Math Libraries

```cpp
#include <AP_Math/AP_Math.h>

void calculate_navigation() {
    // Vector operations
    Vector3f position(10.0f, 20.0f, -5.0f);
    Vector3f target(50.0f, 60.0f, -10.0f);

    Vector3f error = target - position;
    float distance = error.length();
    error.normalize();

    // Quaternion for attitude
    Quaternion attitude;
    attitude.from_euler(roll_rad, pitch_rad, yaw_rad);

    // Rotation matrix
    Matrix3f dcm;
    dcm.from_euler(roll_rad, pitch_rad, yaw_rad);

    // Transform vector from earth to body frame
    Vector3f error_body = dcm.transposed() * error;

    // Constrain values
    float limited_speed = constrain_float(speed, 0.0f, max_speed);

    // Angle wrapping
    float heading = wrap_360(yaw_deg);

    // Safe math
    float asin_value = safe_asin(ratio);
}

// Location operations
void navigate_to_waypoint() {
    Location current_pos = gps.location();
    Location target_pos(lat, lon, alt, Location::AltFrame::ABSOLUTE);

    // Distance and bearing
    float distance = current_pos.get_distance(target_pos);
    float bearing = current_pos.get_bearing_to(target_pos);

    hal.console->printf("Target: %.1fm at %.1f°\n",
                        distance, degrees(bearing));
}
```

### 7.4 Using Parameters

```cpp
class MyModule {
public:
    AP_Float gain;
    AP_Int16 mode;
    AP_Vector3f offset;

    static const struct AP_Param::GroupInfo var_info[];
};

const struct AP_Param::GroupInfo MyModule::var_info[] = {
    // Name, Index, Object, Member, Default
    AP_GROUPINFO("GAIN", 0, MyModule, gain, 1.5f),
    AP_GROUPINFO("MODE", 1, MyModule, mode, 0),
    AP_GROUPINFO("OFFSET", 2, MyModule, offset, 0),
    AP_GROUPEND
};

void use_parameters() {
    // Read parameter
    float current_gain = gain.get();

    // Set parameter
    gain.set(2.0f);

    // Set and save to EEPROM
    gain.set_and_save(2.5f);

    // Load from EEPROM
    gain.load();

    // Check if parameter has been set by user
    if (!gain.configured()) {
        gain.set_and_save(1.0f);  // Set default
    }
}
```

### 7.5 Implementing Arming Checks

```cpp
#include <AP_Arming/AP_Arming.h>

class GCS_Arming : public AP_Arming {
public:
    GCS_Arming(GCS_MAVLink_Pro& _gcs)
        : AP_Arming()
        , gcs(_gcs)
    {}

    // Override base class checks
    bool pre_arm_checks(bool report) override {
        // Call base implementation
        if (!AP_Arming::pre_arm_checks(report)) {
            return false;
        }

        // Custom checks
        if (!check_telemetry(report)) {
            return false;
        }

        if (!check_connections(report)) {
            return false;
        }

        return true;
    }

private:
    GCS_MAVLink_Pro& gcs;

    bool check_telemetry(bool report) {
        if (gcs.telemetry_port == nullptr) {
            check_failed(report, "No telemetry port");
            return false;
        }
        return true;
    }

    bool check_connections(bool report) {
        if (!gcs.gcs_connected) {
            check_failed(report, "GCS not connected");
            return false;
        }
        return true;
    }
};

// In main application:
class GCS_MAVLink_Pro {
private:
    GCS_Arming arming;
    bool armed;
};

GCS_MAVLink_Pro::GCS_MAVLink_Pro()
    : arming(*this)
    , armed(false)
{}

void GCS_MAVLink_Pro::setup() {
    arming.init(AP_Arming::ArmingMethod::MAVLINK);
}

void GCS_MAVLink_Pro::handle_arm_command() {
    if (arming.arm(AP_Arming::Method::MAVLINK)) {
        armed = true;
        hal.console->printf("Armed!\n");
    } else {
        hal.console->printf("Arm failed\n");
    }
}
```

---

## 8. Custom Library Implementation

### 8.1 Creating a Custom Library

If you need to create your own library that follows ArduPilot patterns:

```cpp
// File: libraries/GCS_Custom/GCS_Custom.h
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class GCS_Custom {
public:
    GCS_Custom();

    // Initialization
    void init();

    // Update (call periodically)
    void update();

    // Parameters
    AP_Float update_rate;
    AP_Int8 enabled;

    static const struct AP_Param::GroupInfo var_info[];

private:
    uint32_t last_update_ms;
    bool initialized;
};
```

```cpp
// File: libraries/GCS_Custom/GCS_Custom.cpp
#include "GCS_Custom.h"

extern const AP_HAL::HAL& hal;

const struct AP_Param::GroupInfo GCS_Custom::var_info[] = {
    AP_GROUPINFO("RATE", 0, GCS_Custom, update_rate, 10.0f),
    AP_GROUPINFO("ENABLE", 1, GCS_Custom, enabled, 1),
    AP_GROUPEND
};

GCS_Custom::GCS_Custom()
    : last_update_ms(0)
    , initialized(false)
{}

void GCS_Custom::init() {
    hal.console->printf("GCS_Custom initializing...\n");
    initialized = true;
}

void GCS_Custom::update() {
    if (!initialized || !enabled) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    uint32_t dt_ms = now - last_update_ms;

    if (dt_ms < (1000.0f / update_rate)) {
        return;  // Not time yet
    }

    last_update_ms = now;

    // Your update logic here
    hal.console->printf("GCS_Custom update\n");
}
```

---

## 9. Testing Strategies

### 9.1 Unit Testing with SITL

```bash
# Build for SITL
cd ardupilot
./waf configure --board sitl
./waf build --target bin/arducopter

# Run with your custom parameters
sim_vehicle.py -v Copter \
    --add-param-file=/path/to/your/params.txt \
    --console --map
```

### 9.2 Hardware-in-the-Loop (HIL) Testing

```cpp
// Enable HIL mode
#define HIL_MODE HIL_MODE_SENSORS

// In your code:
if (AP::sitl() != nullptr) {
    // Running in simulation
    hal.console->printf("SITL mode active\n");
}
```

### 9.3 Integration Testing

Create test suite:

```cpp
// File: tests/test_integration.cpp
#include <AP_HAL/AP_HAL.h>
#include "GCS_MAVLink_Pro.h"

extern const AP_HAL::HAL& hal;

void test_parameter_system() {
    AP_Param::erase_all();
    AP_Param::load_all();

    // Test parameter read/write
    GCS_MAVLink_Pro gcs;
    gcs.version.set(2.0f);
    assert(gcs.version.get() == 2.0f);

    hal.console->printf("PASS: Parameter system\n");
}

void test_serial_manager() {
    AP_SerialManager serial_mgr;
    serial_mgr.init();

    // Test port discovery
    auto *port = serial_mgr.find_serial(
        AP_SerialManager::SerialProtocol_MAVLink, 0);
    assert(port != nullptr);

    hal.console->printf("PASS: Serial manager\n");
}

void setup() {
    hal.console->printf("Running integration tests...\n");
    test_parameter_system();
    test_serial_manager();
    hal.console->printf("All tests passed!\n");
}

void loop() {
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
```

---

## 10. Troubleshooting

### 10.1 Common Compilation Errors

**Error: `AP_HAL::HAL' has not been declared**
```cpp
// Solution: Include AP_HAL header and define hal
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;
```

**Error: Multiple definition of `hal'**
```cpp
// Solution: Use extern in headers, define in one .cpp file
// Header file (.h):
extern const AP_HAL::HAL& hal;

// Source file (.cpp):
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
```

**Error: undefined reference to `AP_Param::setup()'**
```cpp
// Solution: Link AP_Param library in build system
# CMake:
file(GLOB_RECURSE AP_PARAM_SOURCES libraries/AP_Param/*.cpp)
# Makefile:
SOURCES += $(wildcard libraries/AP_Param/*.cpp)
```

### 10.2 Runtime Issues

**Parameters not saving**
```cpp
// Check: Is storage initialized?
if (!hal.storage->is_initialized()) {
    hal.console->printf("ERROR: Storage not initialized\n");
}

// Check: Is parameter table valid?
if (!AP_Param::check_var_info()) {
    hal.console->printf("ERROR: Bad parameter table\n");
}

// Force save all parameters
AP_Param::save_all();
```

**Serial port not found**
```cpp
// Check available ports
for (uint8_t i = 0; i < 10; i++) {
    auto *uart = hal.serial(i);
    if (uart != nullptr) {
        hal.console->printf("UART%d available\n", i);
    }
}

// Check serial manager configuration
auto *port = serial_manager.find_serial(
    AP_SerialManager::SerialProtocol_MAVLink, 0);
if (port == nullptr) {
    hal.console->printf("No MAVLink port configured\n");
    hal.console->printf("Set SERIAL1_PROTOCOL = 2\n");
}
```

**Scheduler tasks not running**
```cpp
// Check: Is scheduler initialized?
scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), 0);

// Check: Is loop() calling scheduler?
void loop() {
    scheduler.loop();  // Must call this!
}

// Enable debug output
scheduler.debug_flags = AP_Scheduler::DEBUG_TIMING;
```

### 10.3 Memory Issues

```cpp
// Check available memory
uint32_t free_mem = hal.util->available_memory();
hal.console->printf("Free memory: %u bytes\n",
                    (unsigned)free_mem);

// Reduce buffer sizes if low on memory
// In your UARTDriver begin():
uart->begin(57600,
            128,  // RX buffer (reduced from 256)
            128); // TX buffer (reduced from 256)
```

---

## 11. Example: Complete MAVLink Telemetry Application

Here's a complete working example:

```cpp
// File: src/MAVLink_Telemetry.cpp
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

// Select HAL
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include <AP_HAL_Linux/HAL_Linux_Class.h>
    const AP_HAL::HAL& hal = AP_HAL_Linux::get_HAL();
#endif

class MAVLink_Telemetry {
public:
    MAVLink_Telemetry();
    void setup();
    void loop();

private:
    // Serial manager
    AP_SerialManager serial_manager;
    AP_HAL::UARTDriver *mavlink_port;

    // Parameters
    AP_Int8 system_id;
    AP_Int8 component_id;
    AP_Int32 telemetry_baud;

    // State
    uint32_t heartbeat_timer_ms;
    uint32_t loop_count;

    // Methods
    void send_heartbeat();
    void send_sys_status();
    void handle_mavlink();

    static const struct AP_Param::GroupInfo var_info[];
};

const struct AP_Param::GroupInfo MAVLink_Telemetry::var_info[] = {
    AP_GROUPINFO("SYSID", 0, MAVLink_Telemetry, system_id, 1),
    AP_GROUPINFO("COMPID", 1, MAVLink_Telemetry, component_id, 1),
    AP_GROUPINFO("BAUD", 2, MAVLink_Telemetry, telemetry_baud, 57600),
    AP_SUBGROUPINFO(serial_manager, "SERIAL", 3, MAVLink_Telemetry, AP_SerialManager),
    AP_GROUPEND
};

MAVLink_Telemetry::MAVLink_Telemetry()
    : mavlink_port(nullptr)
    , heartbeat_timer_ms(0)
    , loop_count(0)
{}

void MAVLink_Telemetry::setup() {
    hal.console->printf("\nMAVLink Telemetry Starting...\n");

    // Initialize parameters
    AP_Param::setup_sketch_defaults();
    AP_Param::check_var_info();
    AP_Param::load_all();

    // Initialize serial manager
    serial_manager.init();

    // Find MAVLink port
    mavlink_port = serial_manager.find_serial(
        AP_SerialManager::SerialProtocol_MAVLink, 0);

    if (mavlink_port) {
        hal.console->printf("MAVLink on port at %d baud\n",
                            (int)telemetry_baud.get());
    } else {
        hal.console->printf("No MAVLink port configured!\n");
        hal.console->printf("Set SERIAL1_PROTOCOL = 2\n");
    }

    hal.console->printf("System ID: %d\n", (int)system_id.get());
    hal.console->printf("Setup complete!\n\n");
}

void MAVLink_Telemetry::loop() {
    uint32_t now = AP_HAL::millis();

    // Send heartbeat every 1 second
    if (now - heartbeat_timer_ms >= 1000) {
        send_heartbeat();
        send_sys_status();
        heartbeat_timer_ms = now;
    }

    // Handle incoming MAVLink
    handle_mavlink();

    // Update counter
    loop_count++;

    // Run at ~100 Hz
    hal.scheduler->delay(10);
}

void MAVLink_Telemetry::send_heartbeat() {
    if (!mavlink_port) return;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        system_id,
        component_id,
        &msg,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,  // custom_mode
        MAV_STATE_ACTIVE
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_port->write(buf, len);

    hal.console->printf("Heartbeat sent (loop: %lu)\n",
                        (unsigned long)loop_count);
}

void MAVLink_Telemetry::send_sys_status() {
    if (!mavlink_port) return;

    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        system_id,
        component_id,
        &msg,
        0,      // onboard_control_sensors_present
        0,      // onboard_control_sensors_enabled
        0,      // onboard_control_sensors_health
        500,    // load (50.0%)
        11000,  // voltage_battery (11.0V)
        -1,     // current_battery
        50,     // battery_remaining
        0, 0, 0, 0, 0, 0, 0
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_port->write(buf, len);
}

void MAVLink_Telemetry::handle_mavlink() {
    if (!mavlink_port) return;

    while (mavlink_port->available()) {
        uint8_t byte = mavlink_port->read();
        // Parse MAVLink (simplified)
        // In real code, use mavlink_parse_char()
    }
}

// Global instance
static MAVLink_Telemetry app;

void setup() {
    app.setup();
}

void loop() {
    app.loop();
}

AP_HAL_MAIN();
```

---

## 12. Best Practices Summary

### 12.1 Code Organization
- ✅ Use one class per library
- ✅ Follow ArduPilot naming conventions (AP_ClassName)
- ✅ Put parameters in var_info[] table
- ✅ Use AP_HAL for all hardware access
- ✅ Document your code

### 12.2 Performance
- ✅ Use scheduler for periodic tasks
- ✅ Minimize allocations in loop()
- ✅ Use const& for large objects
- ✅ Profile with scheduler timing debug
- ✅ Check available memory regularly

### 12.3 Safety
- ✅ Implement comprehensive arming checks
- ✅ Handle RC failsafe
- ✅ Validate all inputs
- ✅ Log important events
- ✅ Test thoroughly in SITL first

### 12.4 Maintainability
- ✅ Keep libraries decoupled
- ✅ Use dependency injection
- ✅ Write unit tests
- ✅ Version your parameters
- ✅ Document integration points

---

## 13. Quick Reference

### 13.1 Essential Includes

```cpp
#include <AP_HAL/AP_HAL.h>              // Hardware abstraction
#include <AP_Common/AP_Common.h>        // Common utilities
#include <AP_Math/AP_Math.h>            // Math libraries
#include <AP_Param/AP_Param.h>          // Parameters
#include <AP_SerialManager/AP_SerialManager.h>  // Serial ports
#include <AP_Scheduler/AP_Scheduler.h>  // Task scheduling
#include <RC_Channel/RC_Channel.h>      // RC input/output
#include <AP_Arming/AP_Arming.h>        // Arming checks
```

### 13.2 Common Singletons

```cpp
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
AP::scheduler()
AP::serialmanager()
AP::gps()
AP::compass()
AP::ins()
```

### 13.3 Typical Loop Rates

```
Fast loop:        100-400 Hz (IMU, control)
Medium loop:      50 Hz (sensors, GPS)
Slow loop:        10 Hz (telemetry, logging)
Very slow loop:   1 Hz (arming checks, status)
```

---

## Conclusion

This integration guide provides everything you need to integrate ArduPilot libraries into your GCS_MAVLink_Pro project. Start with the minimal integration, test thoroughly, then add features incrementally.

For questions and support:
- ArduPilot Forums: https://discuss.ardupilot.org/
- Discord: https://ardupilot.org/discord
- GitHub: https://github.com/ArduPilot/ardupilot

Happy coding!
