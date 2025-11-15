# ArduPilot Library Interconnections and Dependencies

## Complete Dependency Map for GCS_MAVLink_Pro Project

This document maps the relationships, dependencies, and interconnections between all analyzed ArduPilot libraries.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Dependency Graph](#dependency-graph)
3. [Layer-by-Layer Analysis](#layer-by-layer-analysis)
4. [Data Flow Diagrams](#data-flow-diagrams)
5. [Library Interaction Patterns](#library-interaction-patterns)
6. [Initialization Order](#initialization-order)
7. [Runtime Communication Paths](#runtime-communication-paths)

---

## 1. Architecture Overview

### 1.1 Five-Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  LAYER 5: VEHICLE APPLICATIONS                              │
│  ├─ ArduCopter, ArduPlane, ArduRover, ArduSub              │
│  └─ Vehicle-specific logic and modes                        │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  LAYER 4: VEHICLE BASE & COMMON FUNCTIONALITY               │
│  ├─ AP_Vehicle (base class)                                │
│  ├─ AP_Arming (safety checks)                              │
│  ├─ RC_Channel (radio control)                             │
│  └─ AP_Mount (camera gimbals)                              │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  LAYER 3: CORE LIBRARIES & SUBSYSTEMS                       │
│  ├─ AP_Scheduler (task management)                         │
│  ├─ AP_Param (configuration storage)                       │
│  ├─ AP_SerialManager (port allocation)                     │
│  ├─ AP_Math (mathematics & algorithms)                     │
│  └─ AP_Common (utilities & helpers)                        │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  LAYER 2: HARDWARE ABSTRACTION LAYER (HAL)                 │
│  ├─ AP_HAL (interface definitions)                         │
│  ├─ AP_HAL_ChibiOS (STM32 implementation)                  │
│  ├─ AP_HAL_Linux (companion computer)                      │
│  ├─ AP_HAL_ESP32 (ESP32 microcontroller)                   │
│  └─ AP_HAL_SITL (simulation)                               │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  LAYER 1: HARDWARE & SIMULATION                             │
│  ├─ Physical Hardware (STM32, Linux, ESP32)                │
│  └─ SITL (Software-in-the-Loop simulation)                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Dependency Graph

### 2.1 Direct Dependencies

```
AP_Vehicle
├── depends on: AP_HAL
├── depends on: AP_Scheduler
├── depends on: AP_Param
├── depends on: AP_SerialManager
├── depends on: AP_Math
├── depends on: AP_Common
├── depends on: AP_Arming
├── depends on: RC_Channel
└── depends on: AP_Mount

AP_Arming
├── depends on: AP_HAL
├── depends on: AP_Param
├── depends on: AP_Math
├── depends on: AP_Common
└── depends on: AP_SerialManager (indirect via sensors)

RC_Channel
├── depends on: AP_HAL (RCInput/RCOutput)
├── depends on: AP_Param
├── depends on: AP_Math
└── depends on: AP_Common

AP_Mount
├── depends on: AP_HAL (UARTDriver, I2C, SPI)
├── depends on: AP_Param
├── depends on: AP_Math
├── depends on: AP_Common
└── depends on: AP_SerialManager

AP_Scheduler
├── depends on: AP_HAL (Scheduler, Semaphores)
├── depends on: AP_Param
├── depends on: AP_Math
└── depends on: AP_Common

AP_SerialManager
├── depends on: AP_HAL (UARTDriver)
├── depends on: AP_Param
└── depends on: AP_Common

AP_Param
├── depends on: AP_HAL (Storage)
├── depends on: AP_Math
└── depends on: AP_Common

AP_Math
├── depends on: AP_HAL (basic types)
└── depends on: AP_Common

AP_Common
└── depends on: AP_HAL (minimal)

AP_HAL_SITL
├── depends on: AP_HAL (interface)
├── depends on: SITL (physics engine)
├── depends on: AP_Math
└── depends on: AP_Common

AP_HAL_ESP32
├── depends on: AP_HAL (interface)
├── depends on: ESP-IDF
└── depends on: AP_Common

SITL
├── depends on: AP_HAL
├── depends on: AP_Math
├── depends on: AP_Common
└── depends on: AP_Param
```

### 2.2 Dependency Matrix

| Library | HAL | Common | Math | Param | SerialMgr | Scheduler | Arming | RC | Mount | Vehicle | SITL |
|---------|-----|--------|------|-------|-----------|-----------|--------|----|----|---------|------|
| **AP_HAL** | - | ✓ | - | - | - | - | - | - | - | - | - |
| **AP_Common** | ✓ | - | - | - | - | - | - | - | - | - | - |
| **AP_Math** | ✓ | ✓ | - | - | - | - | - | - | - | - | - |
| **AP_Param** | ✓ | ✓ | ✓ | - | - | - | - | - | - | - | - |
| **AP_SerialManager** | ✓ | ✓ | - | ✓ | - | - | - | - | - | - | - |
| **AP_Scheduler** | ✓ | ✓ | ✓ | ✓ | - | - | - | - | - | - | - |
| **AP_Arming** | ✓ | ✓ | ✓ | ✓ | ✓ | - | - | - | - | - | - |
| **RC_Channel** | ✓ | ✓ | ✓ | ✓ | - | - | - | - | - | - | - |
| **AP_Mount** | ✓ | ✓ | ✓ | ✓ | ✓ | - | - | - | - | - | - |
| **AP_Vehicle** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | - | - |
| **SITL** | ✓ | ✓ | ✓ | ✓ | - | - | - | - | - | - | - |
| **AP_HAL_SITL** | ✓ | ✓ | ✓ | - | - | - | - | - | - | - | ✓ |
| **AP_HAL_ESP32** | ✓ | ✓ | - | - | - | - | - | - | - | - | - |

---

## 3. Layer-by-Layer Analysis

### 3.1 Layer 1: Hardware & Platform (AP_HAL, AP_HAL_SITL, AP_HAL_ESP32)

**Purpose**: Provide uniform hardware access across platforms

**Key Interfaces**:
- GPIO, UART, SPI, I2C, CAN
- Scheduler (threading)
- Storage (EEPROM/Flash)
- RC Input/Output
- Analog sensors

**Platform Implementations**:
```
AP_HAL (Interface)
├── AP_HAL_ChibiOS → STM32 (Pixhawk, Cube, etc.)
├── AP_HAL_Linux → Raspberry Pi, BeagleBone
├── AP_HAL_ESP32 → ESP32 microcontrollers
├── AP_HAL_SITL → Software simulation
└── AP_HAL_QURT → Qualcomm Snapdragon
```

**Global Singleton**:
```cpp
extern const AP_HAL::HAL& hal;  // Global hardware access
```

**Interconnections**:
- Used by ALL higher layers
- Platform-specific at compile time
- Zero runtime overhead (compile-time polymorphism)

---

### 3.2 Layer 2: Utilities & Math (AP_Common, AP_Math)

**AP_Common Provides**:
- Location class (GPS coordinates)
- Bitmask template
- NMEA checksum helpers
- Memory allocation wrappers
- Compiler macros (PACKED, WEAK, etc.)

**AP_Math Provides**:
- Vector2/3, Matrix3, Quaternion
- Rotation transformations
- Control algorithms (sqrt_controller, shape_accel)
- S-Curve motion planning
- Polygon/circle geometry
- CRC/checksum functions

**Usage Pattern**:
```cpp
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

// Used everywhere for basic operations
Vector3f position;
Quaternion attitude;
Location home;
```

**Interconnections**:
- AP_Common: Used by every library
- AP_Math: Used for navigation, control, sensor fusion
- Both are foundational dependencies

---

### 3.3 Layer 3: Core Services (AP_Param, AP_SerialManager, AP_Scheduler)

#### 3.3.1 AP_Param - Configuration Storage

**Provides**:
- Parameter persistence (EEPROM/Flash)
- MAVLink parameter protocol
- Parameter groups and nesting
- Type-safe parameter access

**Storage Backend**:
```cpp
AP_Param
    ↓
AP_HAL::Storage
    ↓
Platform-specific flash/EEPROM
```

**Usage Example**:
```cpp
class MyLibrary {
    AP_Float my_gain;
    AP_Int16 my_mode;

    static const struct AP_Param::GroupInfo var_info[];
};
```

**Used By**: Every library with configuration

---

#### 3.3.2 AP_SerialManager - Port Allocation

**Provides**:
- Serial port → protocol mapping
- Multi-instance device support
- Baud rate management
- 50+ protocol types

**Connection Chain**:
```cpp
GPS Library
    ↓
AP_SerialManager::find_serial(SerialProtocol_GPS, 0)
    ↓
Returns: AP_HAL::UARTDriver*
    ↓
Platform UART (hardware or simulation)
```

**Supported Protocols**:
- MAVLink (telemetry)
- GPS (multiple types)
- Rangefinders, LiDAR
- Gimbals, ESCs
- Custom/scripting

**Used By**: All libraries needing serial communication

---

#### 3.3.3 AP_Scheduler - Task Management

**Provides**:
- Periodic task scheduling
- Priority management
- Loop rate control
- Performance monitoring

**Task Registration**:
```cpp
static const AP_Scheduler::Task scheduler_tasks[] = {
    SCHED_TASK(read_sensors,     400, 100, 3),
    SCHED_TASK(update_control,   400, 200, 6),
    SCHED_TASK(send_telemetry,    10,  50, 12),
};
```

**Integration with HAL**:
```cpp
AP_Scheduler
    ↓
AP_HAL::Scheduler (threading primitives)
    ↓
FreeRTOS/ChibiOS/Linux threads
```

**Used By**: AP_Vehicle (main loop organization)

---

### 3.4 Layer 4: Vehicle Services (AP_Arming, RC_Channel, AP_Mount)

#### 3.4.1 AP_Arming - Safety System

**Provides**:
- 70+ pre-arm checks
- Arming/disarming logic
- Safety state management
- Vehicle-specific customization

**Dependencies**:
```cpp
AP_Arming
├── AP_HAL (safety switch GPIO)
├── AP_Param (configuration)
├── Sensor libraries (via AP_Vehicle)
│   ├── AP_GPS
│   ├── AP_InertialSensor
│   ├── AP_Compass
│   └── AP_Baro
└── AP_SerialManager (sensor discovery)
```

**Interconnection Flow**:
```
User arm command
    ↓
AP_Arming::arm()
    ↓
├── pre_arm_checks() → Check all sensors
├── arm_checks() → Final validation
└── Vehicle::arm() → Vehicle-specific arming
```

**Used By**: AP_Vehicle (arming state machine)

---

#### 3.4.2 RC_Channel - Radio Control

**Provides**:
- 16 RC channels
- 317+ auxiliary functions
- RC protocol support (21 types)
- Failsafe detection
- GCS override

**Data Flow**:
```
Physical RC Receiver
    ↓
AP_HAL::RCInput (PWM/PPM/SBUS capture)
    ↓
AP_RCProtocol (decoding)
    ↓
RC_Channels (normalization, mapping)
    ↓
Vehicle control (roll, pitch, yaw, throttle)
```

**Channel Mapping**:
```cpp
RC_Channel::RC1 → Roll
RC_Channel::RC2 → Pitch
RC_Channel::RC3 → Throttle
RC_Channel::RC4 → Yaw
RC_Channel::RC5-16 → Auxiliary functions
```

**GCS Override**:
```cpp
MAVLink RC_CHANNELS_OVERRIDE
    ↓
RC_Channels::set_override()
    ↓
Replaces physical RC input
```

**Used By**: All vehicle types for manual control

---

#### 3.4.3 AP_Mount - Gimbal Control

**Provides**:
- 14+ gimbal backends
- ROI (Region of Interest) tracking
- Stabilization algorithms
- MAVLink gimbal protocol

**Serial Gimbal Connection**:
```cpp
AP_Mount
    ↓
AP_SerialManager::find_serial(SerialProtocol_Gimbal, 0)
    ↓
Backend (Siyi, SToRM32, etc.)
    ↓
Serial protocol commands
```

**Control Modes**:
- NEUTRAL (safe position)
- MAVLINK_TARGETING (remote control)
- RC_TARGETING (RC stick control)
- GPS_POINT (track GPS coordinate)
- SYSID_TARGET (track vehicle)

**Used By**: Plane, Copter (camera control)

---

### 3.5 Layer 5: Vehicle Base (AP_Vehicle)

**Provides**:
- Common vehicle initialization
- Scheduler task setup
- GCS interface
- Sensor initialization
- Mode management base

**Initialization Sequence**:
```cpp
AP_Vehicle::init_ardupilot()
├── 1. HAL initialization
├── 2. Board-specific setup
├── 3. AP_Param::load_all()
├── 4. AP_SerialManager::init()
├── 5. GCS initialization
├── 6. Sensor initialization
│   ├── AP_GPS
│   ├── AP_InertialSensor
│   ├── AP_Compass
│   ├── AP_Baro
│   └── AP_RangeFinder
├── 7. Vehicle::init_ardupilot() [override]
├── 8. AP_Scheduler::init()
└── 9. Main loop start
```

**Inheritance Hierarchy**:
```
AP_Vehicle (base class)
├── Copter
├── Plane
├── Rover
├── Sub
├── Tracker
└── Blimp
```

**Common Tasks**:
```cpp
// Shared across all vehicles
fast_loop()           // 400 Hz
update_GPS()          // 50 Hz
update_compass()      // 10 Hz
update_mount()        // 50 Hz
ten_hz_logging_loop() // 10 Hz
```

---

## 4. Data Flow Diagrams

### 4.1 Sensor Data Flow

```
Physical Sensors (IMU, GPS, Compass, Baro)
    ↓
AP_HAL (I2C/SPI/UART drivers)
    ↓
Sensor Libraries (AP_InertialSensor, AP_GPS, etc.)
    ↓
AP_AHRS (attitude estimation, sensor fusion)
    ↓
AP_Vehicle (navigation, control)
    ↓
Vehicle Mode (Auto, Stabilize, etc.)
    ↓
Control Output
```

### 4.2 RC Input Flow

```
RC Transmitter → RC Receiver
    ↓
AP_HAL::RCInput (PWM capture)
    ↓
AP_RCProtocol (decode SBUS/CRSF/etc.)
    ↓
RC_Channels (calibration, mapping)
    ↓
Vehicle Control Axes
    ├── Roll
    ├── Pitch
    ├── Yaw
    ├── Throttle
    └── Auxiliary Functions (mode switch, etc.)
```

### 4.3 Motor Output Flow

```
Vehicle Control Logic
    ↓
Motor Mixing (frame-specific)
    ↓
AP_Motors library
    ↓
RC_Channels::calc_pwm() (convert to PWM)
    ↓
AP_HAL::RCOutput (PWM/DShot generation)
    ↓
ESCs → Physical Motors
```

### 4.4 Parameter Flow

```
Ground Control Station (MAVLink)
    ↓
GCS_MAVLink (PARAM_SET command)
    ↓
AP_Param::set()
    ↓
In-memory parameter value
    ↓
AP_Param::save() (on change)
    ↓
AP_HAL::Storage (EEPROM/Flash)
    ↓
Persistent storage
```

### 4.5 Serial Communication Flow

```
External Device (GPS, Gimbal, Telemetry)
    ↓
Physical UART pins
    ↓
AP_HAL::UARTDriver (buffered I/O)
    ↓
AP_SerialManager (protocol routing)
    ↓
Device Driver (AP_GPS, AP_Mount, GCS_MAVLink)
    ↓
Application Logic
```

---

## 5. Library Interaction Patterns

### 5.1 Singleton Pattern (Global Access)

Most libraries use singleton for global access:

```cpp
// HAL singleton
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Namespace singletons (AP namespace)
AP::scheduler()      → AP_Scheduler
AP::serialmanager()  → AP_SerialManager
AP::gps()            → AP_GPS
AP::compass()        → AP_Compass
AP::ins()            → AP_InertialSensor
AP::baro()           → AP_Barometer
```

**Usage**:
```cpp
// Access GPS from anywhere
const AP_GPS& gps = AP::gps();
if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
    Location loc = gps.location();
}
```

### 5.2 Callback Registration Pattern

Libraries register callbacks with scheduler:

```cpp
// In AP_Vehicle::init_ardupilot()
scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);

// Scheduler calls tasks periodically
void Copter::fast_loop() {
    // Called at 400 Hz
}
```

### 5.3 HAL Abstraction Pattern

All hardware access goes through HAL:

```cpp
// Platform-independent code
hal.console->printf("Initializing...\n");
hal.scheduler->delay(100);
hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
hal.i2c_mgr->get_device(bus, addr);
```

### 5.4 Parameter Group Pattern

Libraries expose parameters via AP_Param groups:

```cpp
class MyLibrary {
    AP_Float gain;
    AP_Int16 mode;

    static const struct AP_Param::GroupInfo var_info[];
};

const struct AP_Param::GroupInfo MyLibrary::var_info[] = {
    AP_GROUPINFO("GAIN", 0, MyLibrary, gain, 1.0f),
    AP_GROUPINFO("MODE", 1, MyLibrary, mode, 0),
    AP_GROUPEND
};
```

---

## 6. Initialization Order

### 6.1 Boot Sequence

```
1. Hardware Reset
    ↓
2. Bootloader (if present)
    ↓
3. main() entry [AP_HAL_MAIN macro]
    ↓
4. HAL initialization
   ├── hal.init()
   ├── Console UART
   ├── Scheduler
   └── GPIO/SPI/I2C/Storage
    ↓
5. AP_Vehicle::init_ardupilot()
   ├── Load parameters (AP_Param::load_all)
   ├── Initialize serial manager
   ├── Initialize GCS
   ├── Initialize sensors
   │   ├── InertialSensor
   │   ├── Barometer
   │   ├── Compass
   │   ├── GPS
   │   └── RangeFinder
   ├── Vehicle-specific init
   ├── Arming checks initialization
   ├── RC_Channels initialization
   └── Mount initialization
    ↓
6. Scheduler::init(tasks)
    ↓
7. Main Loop (scheduler.loop())
   ├── Fast loop (400 Hz)
   ├── Medium tasks (50-100 Hz)
   └── Slow tasks (1-10 Hz)
```

### 6.2 Critical Dependencies During Boot

**Phase 1 - HAL Only**:
- No parameters available
- No serial ports configured
- Only hal.console for debug

**Phase 2 - Parameters Loaded**:
- AP_Param::load_all() completes
- Can read configuration
- Serial manager can configure ports

**Phase 3 - Sensors Online**:
- All sensor libraries initialized
- Calibration data loaded
- Ready for arming checks

**Phase 4 - Vehicle Ready**:
- Scheduler running
- All systems operational
- Can arm and fly

---

## 7. Runtime Communication Paths

### 7.1 MAVLink Command Path

```
Ground Control Station
    ↓ (MAVLink over serial/WiFi/UDP)
AP_HAL::UARTDriver or Socket
    ↓
GCS_MAVLink::handle_message()
    ↓
Command handlers:
├── handle_command_long()
│   ├── MAV_CMD_COMPONENT_ARM_DISARM → AP_Arming::arm()
│   ├── MAV_CMD_NAV_TAKEOFF → set_mode(AUTO)
│   └── MAV_CMD_DO_SET_SERVO → RC_Channels::set_output_pwm()
├── handle_rc_channels_override() → RC_Channels::set_override()
├── handle_param_set() → AP_Param::set()
└── handle_mission_item() → AP_Mission::add_cmd()
```

### 7.2 Sensor Fusion Path

```
Raw Sensor Data
├── AP_InertialSensor (gyro, accel at 1000 Hz)
├── AP_Compass (magnetometer at 100 Hz)
├── AP_Baro (pressure at 50 Hz)
└── AP_GPS (position/velocity at 5-10 Hz)
    ↓
AP_AHRS (Extended Kalman Filter)
├── Fuses all sensor data
├── Estimates: attitude, position, velocity
└── Provides: get_position(), get_rotation_body_to_ned()
    ↓
AP_Navigation
├── Waypoint following
├── Path planning
└── Obstacle avoidance
    ↓
Vehicle Control
├── Attitude controller
├── Position controller
└── Velocity controller
    ↓
Motor Outputs
```

### 7.3 Arming Path

```
User Arming Request
├── RC stick sequence (rudder right)
├── MAVLink command (MAV_CMD_COMPONENT_ARM_DISARM)
└── GCS button click
    ↓
AP_Arming::arm(method)
├── Pre-arm checks
│   ├── GPS lock? (AP_GPS::status())
│   ├── Compass healthy? (AP_Compass::healthy())
│   ├── IMU calibrated? (AP_InertialSensor::calibrated())
│   ├── RC calibrated? (RC_Channels::min_max_configured())
│   ├── Battery voltage OK?
│   ├── Safety switch? (hal.util->safety_switch_state())
│   └── 20+ more checks...
├── Arm checks (side effects)
│   ├── Log arm event
│   ├── Set GPIO armed pin
│   ├── Enable terrain database
│   └── Enable geofence
└── Vehicle::arm() [vehicle-specific]
    ├── Copter: Enable motors
    ├── Plane: Enable throttle
    └── Rover: Enable steering
    ↓
Armed State (motors can spin)
```

### 7.4 Logging Path

```
Data to Log
├── IMU readings
├── GPS position
├── Attitude estimate
├── RC inputs
├── Motor outputs
└── System status
    ↓
AP_Logger (formerly DataFlash)
├── Format definitions (FMT messages)
├── Buffering (ring buffer)
└── Storage selection
    ├── SD card (SPI/SDIO)
    ├── onboard flash
    └── MAVLink streaming
    ↓
AP_HAL::Storage or SD card
    ↓
Binary log files (.BIN)
```

---

## 8. Key Singleton Access Points

### 8.1 Commonly Used Singletons

```cpp
// Hardware access
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Core systems
AP::scheduler()           // Task scheduling
AP::serialmanager()       // Serial port allocation
AP::param()              // Parameter system (rare direct use)

// Sensors
AP::gps()                // GPS receivers
AP::compass()            // Magnetometers
AP::ins()                // Inertial sensors (gyro/accel)
AP::baro()               // Barometers
AP::rangefinder()        // Distance sensors
AP::opticalflow()        // Optical flow sensors

// Navigation
AP::ahrs()               // Attitude & heading reference
AP::mission()            // Waypoint missions
AP::terrain()            // Terrain database

// Safety & Control
AP::arming()             // Arming checks (if available)
RC_Channels::instance()  // RC input/output

// Communication
GCS_MAVLINK::instance()  // MAVLink telemetry

// Storage & Logging
AP::logger()             // Data logging
AP::fs()                 // Filesystem (if available)
```

---

## 9. Inter-Library Communication Examples

### 9.1 GPS → AHRS → Navigation

```cpp
// GPS provides position
Location current_pos = AP::gps().location();

// AHRS provides attitude
Matrix3f dcm = AP::ahrs().get_rotation_body_to_ned();

// Navigation uses both
Vector3f target_vel = navigate_to_waypoint(current_pos, waypoint, dcm);
```

### 9.2 RC → Flight Mode → Motors

```cpp
// RC provides pilot input
RC_Channel* roll_channel = RC_Channels::rc_channel(RC_CHANNEL_ROLL);
float roll_input = roll_channel->norm_input();

// Flight mode interprets input
Copter::Mode::stabilize.run();  // Uses roll_input

// Motor mixing
motors->set_roll(roll_output);
motors->output();  // → AP_HAL::RCOutput
```

### 9.3 SerialManager → GPS → Navigation

```cpp
// SerialManager provides UART
AP_HAL::UARTDriver* gps_uart =
    AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_GPS, 0);

// GPS uses UART
AP_GPS gps;
gps.init(gps_uart);
gps.update();  // Reads serial data

// Navigation uses GPS data
if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
    navigate_to(target);
}
```

---

## 10. Platform-Specific Adaptations

### 10.1 ChibiOS (Pixhawk)

```
AP_HAL_ChibiOS
├── Real-time threads (ChibiOS/RT)
├── DMA for UART/SPI/I2C
├── Hardware PWM timers
├── Flash storage (wear leveling)
└── CAN bus (FDCAN peripheral)
```

### 10.2 ESP32

```
AP_HAL_ESP32
├── FreeRTOS threads
├── Dual-core task pinning
├── WiFi networking (TCP/UDP)
├── Flash storage (NVS)
└── RMT for RC input (unique to ESP32)
```

### 10.3 SITL (Simulation)

```
AP_HAL_SITL
├── POSIX threads
├── In-memory "sensors"
├── UDP for multi-vehicle
├── File-based storage
└── SITL physics engine
    ├── Aircraft dynamics
    ├── Sensor simulation
    └── External simulator connection
```

---

## 11. Summary: Critical Integration Points

### For GCS_MAVLink_Pro Project:

**You MUST integrate**:
1. **AP_HAL** - Hardware abstraction (always required)
2. **AP_Param** - Configuration management
3. **AP_SerialManager** - Serial port allocation
4. **AP_Math** - Mathematical operations
5. **AP_Common** - Basic utilities

**You SHOULD integrate**:
6. **AP_Scheduler** - Task management (if real-time control needed)
7. **RC_Channel** - If manual control required
8. **AP_Arming** - If safety checks required

**You MAY integrate**:
9. **AP_Mount** - If gimbal control needed
10. **SITL** - For testing without hardware
11. **AP_HAL_ESP32** - For ESP32 platform
12. **AP_Vehicle** - As reference for initialization patterns

**Dependencies you inherit**:
- AP_Math depends on AP_HAL, AP_Common
- AP_Param depends on AP_HAL, AP_Math, AP_Common
- AP_SerialManager depends on AP_HAL, AP_Param, AP_Common
- Everything depends on AP_HAL and AP_Common

**Minimum viable integration**:
```
Your Project
├── AP_HAL (select platform: ChibiOS/Linux/ESP32/SITL)
├── AP_Common
├── AP_Math
├── AP_Param
└── AP_SerialManager
```

---

## End of Interconnection Documentation

This document provides a complete map of library relationships. Use this as a reference when integrating ArduPilot libraries into your GCS_MAVLink_Pro project.
