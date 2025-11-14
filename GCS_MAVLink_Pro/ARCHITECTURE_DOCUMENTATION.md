# ArduPilot GCS MAVLink Architecture - Complete Analysis

## Table of Contents
1. [Overview](#overview)
2. [Component Analysis](#component-analysis)
3. [Class Hierarchy & Relationships](#class-hierarchy--relationships)
4. [Message Flow](#message-flow)
5. [Integration Guide](#integration-guide)
6. [Implementation Guide](#implementation-guide)

---

## Overview

The ArduPilot GCS (Ground Control Station) MAVLink system provides a comprehensive framework for bidirectional communication between flight controllers and ground control stations using the MAVLink protocol. The architecture is designed with a multi-layer approach that separates:

- **Protocol handling** (MAVLink serialization/deserialization)
- **Common functionality** (shared across all vehicles)
- **Vehicle-specific implementation** (e.g., Copter, Plane, Rover)

### Key Design Principles

1. **Inheritance-based architecture**: Vehicle-specific classes inherit from common base classes
2. **Channel abstraction**: Support for multiple simultaneous MAVLink connections
3. **Message streaming**: Configurable message rates per channel
4. **Protocol versioning**: Support for MAVLink1 and MAVLink2
5. **Modular design**: Easy to extend for new vehicles or custom implementations

---

## Component Analysis

### 1. GCS_MAVLink.h

**Location**: `libraries/GCS_MAVLink/GCS_MAVLink.h`

**Purpose**: Low-level MAVLink protocol integration header

**Key Elements**:
```cpp
// Channel configuration
#define MAVLINK_COMM_NUM_BUFFERS 8  // or 5 depending on board size

// Global communication arrays
extern AP_HAL::UARTDriver *mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
extern bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];
extern mavlink_system_t mavlink_system;
```

**What it does**:
- Defines the number of simultaneous MAVLink channels supported
- Provides macros for MAVLink helper functions
- Includes the generated MAVLink protocol headers (v2.0)
- Declares global communication port arrays
- Provides helper functions: `comm_get_txspace()`, `comm_send_buffer()`, `comm_send_lock()`

**Integration Notes**:
- This is the lowest level - you rarely interact with it directly
- Primarily included by other GCS headers
- Controls MAVLink 1 vs 2 protocol selection

---

### 2. GCS.h

**Location**: `libraries/GCS_MAVLink/GCS.h`

**Purpose**: Defines the main GCS class hierarchy and base classes

**Key Classes**:

#### **GCS_MAVLINK** (Base Channel Class)
Lines 174-1085

**What it does**:
- Manages a single MAVLink communication channel
- Handles message parsing and routing
- Implements message streaming with configurable rates
- Provides command handling infrastructure

**Key Members**:
```cpp
class GCS_MAVLINK {
protected:
    mavlink_channel_t chan;                    // Channel ID (COMM_0, COMM_1, etc)
    AP_HAL::UARTDriver *_port;                 // UART port for this channel
    mavlink_message_t _channel_buffer;         // Message buffer
    mavlink_status_t _channel_status;          // Protocol status
    AP_Int16 streamRates[NUM_STREAMS];         // Stream rate configuration

public:
    // Core functions
    void update_receive(uint32_t max_time_us=1000);  // Parse incoming messages
    void update_send();                               // Send queued messages
    void send_message(enum ap_message id);            // Queue a message
    void send_text(MAV_SEVERITY severity, const char *fmt, ...);

    // Virtual functions (vehicle must implement)
    virtual void send_nav_controller_output() const = 0;
    virtual void send_pid_tuning() = 0;
    virtual uint8_t base_mode() const = 0;
    virtual MAV_STATE vehicle_system_status() const = 0;
};
```

**Message Streaming System** (Lines 273-285):
```cpp
enum streams : uint8_t {
    STREAM_RAW_SENSORS,      // IMU, baro, etc
    STREAM_EXTENDED_STATUS,  // SYS_STATUS, battery, etc
    STREAM_RC_CHANNELS,      // RC input
    STREAM_RAW_CONTROLLER,   // Servo outputs
    STREAM_POSITION,         // GPS, position
    STREAM_EXTRA1,           // Attitude
    STREAM_EXTRA2,           // VFR_HUD
    STREAM_EXTRA3,           // AHRS, simstate, etc
    STREAM_PARAMS,           // Parameter messages
    STREAM_ADSB,             // ADS-B traffic
    NUM_STREAMS
};
```

Each stream has a configurable rate (Hz) that determines how often messages in that stream are sent.

#### **GCS** (Global Manager Class)
Lines 1089-1376

**What it does**:
- Manages all GCS_MAVLINK channel instances
- Provides global functions (send_text to all channels)
- Handles statustext queuing and distribution
- Manages sensor status flags

**Key Members**:
```cpp
class GCS {
protected:
    GCS_MAVLINK *_chan[MAVLINK_COMM_NUM_BUFFERS];  // Array of channel objects
    uint8_t _num_gcs;                               // Number of active channels

public:
    virtual GCS_MAVLINK *chan(const uint8_t ofs) = 0;  // Get channel by index

    // Global operations
    void send_text(MAV_SEVERITY severity, const char *fmt, ...);
    void send_message(enum ap_message id);
    void update_send();   // Update all channels
    void update_receive(); // Receive on all channels

    // Vehicle-specific pure virtuals
    virtual uint32_t custom_mode() const = 0;
    virtual MAV_TYPE frame_type() const = 0;
};
```

---

### 3. GCS.cpp

**Location**: `libraries/GCS_MAVLink/GCS.cpp`

**Purpose**: Implementation of GCS global manager

**Key Functions** (598 lines total):

#### Parameter Configuration (Lines 37-129)
```cpp
const AP_Param::GroupInfo GCS::var_info[] {
    AP_GROUPINFO("_SYSID", 1, GCS, sysid, MAV_SYSID_DEFAULT),
    AP_GROUPINFO("_GCS_SYSID", 2, GCS, mav_gcs_sysid, 255),
    // ... channel-specific parameter groups
};
```

Defines saveable parameters like system ID, GCS ID range, options, etc.

#### Sensor Status Updates (Lines 313-516)
```cpp
void GCS::update_sensor_status_flags() {
    // Updates bitmasks:
    // - control_sensors_present (what hardware exists)
    // - control_sensors_enabled (what's turned on)
    // - control_sensors_health  (what's working)

    // Examples:
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    if (gps.status() >= min_status_for_gps_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
}
```

These flags are sent in the SYS_STATUS message to inform the GCS about vehicle sensor health.

#### Text Message Distribution (Lines 172-190)
```cpp
void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...) {
    // Determines which channels should receive the message
    uint8_t mask = statustext_send_channel_mask();

    // Formats and queues message for sending
    send_textv(severity, fmt, arg_list, mask);
}
```

**What it does**:
- Formats text messages
- Queues them in `_statustext_queue`
- Distributes to all active, non-private channels
- Thread-safe via semaphore protection

---

### 4. GCS_MAVLink.cpp

**Location**: `libraries/GCS_MAVLink/GCS_MAVLink.cpp`

**Purpose**: Low-level MAVLink communication helpers (195 lines)

**Key Functions**:

#### Buffer Access (Lines 44-66)
```cpp
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
    GCS_MAVLINK *link = gcs().chan(chan);
    return link->channel_buffer();
}

mavlink_status_t* mavlink_get_channel_status(uint8_t chan) {
    GCS_MAVLINK *link = gcs().chan(chan);
    return link->channel_status();
}
```

Used by MAVLink library to access per-channel parsing buffers.

#### Communication Primitives (Lines 117-193)
```cpp
uint16_t comm_get_txspace(mavlink_channel_t chan);  // Get available TX buffer space
void comm_send_buffer(mavlink_channel_t chan, ...); // Send raw bytes
void comm_send_lock(mavlink_channel_t chan, ...);   // Lock channel for atomic send
void comm_send_unlock(mavlink_channel_t chan);      // Release lock
```

**What these do**:
- Provide thread-safe access to UART ports
- Implement locking to prevent partial message transmission
- Handle high-latency link filtering
- Manage alternative protocol switching

---

### 5. GCS_Common.cpp

**Location**: `libraries/GCS_MAVLink/GCS_Common.cpp`

**Purpose**: Common message handlers and senders shared by all vehicles

**Size**: ~83,660 tokens (very large file with hundreds of functions)

**Structure**:
- Constructor and initialization (Lines 134-242)
- Message sending functions (~300 different message types)
- Message handling functions (~200 different incoming messages)
- Command handling (MAV_CMD_* commands)
- Mission protocol implementation
- Parameter protocol implementation

**Sample Functions**:

#### Initialization (Lines 141-242)
```cpp
bool GCS_MAVLINK::init(uint8_t instance) {
    // Get channel ID
    chan = (mavlink_channel_t)(MAVLINK_COMM_0 + instance);

    // Find serial port configuration
    uartstate = AP::serialmanager().find_protocol_instance(
        AP_SerialManager::SerialProtocol_MAVLink, instance);

    // Configure port
    _port->begin(uartstate->baudrate());

    // Load signing keys (if MAVLink2)
    load_signing_key();

    // Register channel globally
    mavlink_comm_port[chan] = _port;
}
```

#### Battery Status (Lines 302-443)
```cpp
void GCS_MAVLINK::send_battery_status(const uint8_t instance) const {
    const AP_BattMonitor &battery = AP::battery();

    // Get battery data
    float temp, current, consumed_mah, consumed_wh;
    battery.get_temperature(temp, instance);
    battery.current_amps(current, instance);

    // Prepare cell voltages (up to 14 cells)
    uint16_t cell_mvolts[10];
    uint16_t cell_mvolts_ext[4];
    const AP_BattMonitor::cells& batt_cells = battery.get_cell_voltages(instance);

    // Send MAVLink message
    mavlink_msg_battery_status_send(chan, instance, ...);
}
```

#### Command Handling Framework
```cpp
void handle_command_int(const mavlink_message_t &msg);
MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, ...);
MAV_RESULT handle_command_do_set_home(const mavlink_command_int_t &packet);
// ... hundreds more command handlers
```

**What it does**:
- Receives COMMAND_INT and COMMAND_LONG messages
- Routes to appropriate handler based on command ID
- Sends COMMAND_ACK with result
- Implements arming, mode changes, calibration, etc.

---

### 6. GCS_Copter.h

**Location**: `ArduCopter/GCS_Copter.h`

**Purpose**: Copter-specific GCS manager class

**Code** (47 lines):
```cpp
class GCS_Copter : public GCS {
    friend class Copter;

public:
    // Required macro to generate chan() methods that return GCS_MAVLINK_Copter*
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Copter);

    // Vehicle identification
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;
    const char* frame_string() const override;

    // Vehicle state
    bool vehicle_initialised() const override;
    bool simple_input_active() const override;
    bool supersimple_input_active() const override;

    // Sensor status
    void update_vehicle_sensor_status_flags(void) override;

protected:
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;  // Copter needs 250us minimum
    }

    GCS_MAVLINK_Copter *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Copter(uart);
    }
};
```

**What it does**:
- Inherits from base GCS class
- Creates GCS_MAVLINK_Copter instances for each channel
- Provides copter-specific mode and type information
- Adds copter-specific sensor status flags (position control, terrain, etc.)

---

### 7. GCS_Copter.cpp

**Location**: `ArduCopter/GCS_Copter.cpp`

**Purpose**: Implementation of copter GCS manager (120 lines)

**Key Functions**:

```cpp
const char* GCS_Copter::frame_string() const {
    return copter.motors->get_frame_string();  // "Quad", "Hexa", "Octa", etc
}

void GCS_Copter::update_vehicle_sensor_status_flags(void) {
    // Always present on copter
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;

    // Position controller flags
    if (copter.pos_control->is_active_NE()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    }

    // Optional sensors
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }

    // Terrain
    if (copter.terrain.status() == AP_Terrain::TerrainStatusOK) {
        control_sensors_health |= MAV_SYS_STATUS_TERRAIN;
    }
}
```

---

### 8. GCS_MAVLink_Copter.h

**Location**: `ArduCopter/GCS_MAVLink_Copter.h`

**Purpose**: Copter-specific MAVLink channel class declaration

**Code** (131 lines):
```cpp
class GCS_MAVLINK_Copter : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;  // Inherit constructor

protected:
    // Message sending (vehicle-specific data)
    void send_attitude_target() override;
    void send_position_target_global_int() override;
    void send_position_target_local_ned() override;
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;
    void send_winch_status() const override;

    // Message handling
    void handle_message(const mavlink_message_t &msg) override;
    void handle_message_set_attitude_target(const mavlink_message_t &msg);
    void handle_message_set_position_target_global_int(const mavlink_message_t &msg);
    void handle_landing_target(...) override;
    void handle_manual_control_axes(...) override;

    // Command handling
    MAV_RESULT handle_command_int_packet(...) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_CONDITION_YAW(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_DO_MOTOR_TEST(const mavlink_command_int_t &packet);
    MAV_RESULT handle_MAV_CMD_NAV_TAKEOFF(const mavlink_command_int_t &packet);

    // State
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    MAV_LANDED_STATE landed_state() const override;

    // VFR_HUD data
    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;
};
```

**What it does**:
- Inherits from GCS_MAVLINK base class
- Overrides virtual functions with copter-specific implementations
- Adds copter-specific command handlers
- Provides copter-specific state information

---

### 9. GCS_MAVLink_Copter.cpp

**Location**: `ArduCopter/GCS_MAVLink_Copter.cpp`

**Purpose**: Implementation of copter channel class

**Sample Implementations**:

#### Vehicle Type (Lines 7-27)
```cpp
MAV_TYPE GCS_Copter::frame_type() const {
    #if FRAME_CONFIG == HELI_FRAME
        const MAV_TYPE mav_type_default = MAV_TYPE_HELICOPTER;
    #else
        const MAV_TYPE mav_type_default = MAV_TYPE_QUADROTOR;
    #endif

    MAV_TYPE mav_type = copter.motors->get_frame_mav_type();
    return (mav_type == MAV_TYPE_GENERIC) ? mav_type_default : mav_type;
}
```

#### Base Mode Flags (Lines 29-60)
```cpp
uint8_t GCS_MAVLINK_Copter::base_mode() const {
    uint8_t mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

    if (copter.pos_control->is_active_NE()) {
        mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

    mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (copter.motors->armed()) {
        mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    return mode;
}
```

#### Sending Target Attitude (Lines 86-107)
```cpp
void GCS_MAVLINK_Copter::send_attitude_target() {
    const Quaternion quat = copter.attitude_control->get_attitude_target_quat();
    const Vector3f ang_vel = copter.attitude_control->get_attitude_target_ang_vel();
    const float thrust = copter.attitude_control->get_throttle_in();

    mavlink_msg_attitude_target_send(
        chan,
        AP_HAL::millis(),
        0,  // typemask (send all)
        quat_out,
        ang_vel.x, ang_vel.y, ang_vel.z,
        thrust
    );
}
```

#### PID Tuning (Lines 257-302)
```cpp
void GCS_MAVLINK_Copter::send_pid_tuning() {
    static const PID_TUNING_AXIS axes[] = {
        PID_TUNING_ROLL, PID_TUNING_PITCH, PID_TUNING_YAW, PID_TUNING_ACCZ
    };

    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        if (!(copter.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;  // Skip if not enabled in bitmask
        }

        const AP_PIDInfo *pid_info = nullptr;
        switch (axes[i]) {
            case PID_TUNING_ROLL:
                pid_info = &copter.attitude_control->get_rate_roll_pid().get_pid_info();
                break;
            // ... other axes
        }

        mavlink_msg_pid_tuning_send(chan, axes[i],
            pid_info->target, pid_info->actual,
            pid_info->FF, pid_info->P, pid_info->I, pid_info->D,
            pid_info->slew_rate, pid_info->Dmod);
    }
}
```

---

## Class Hierarchy & Relationships

### Inheritance Diagram

```
                    ┌─────────────┐
                    │     GCS     │ (Global Manager - libraries/GCS_MAVLink/GCS.h)
                    └──────┬──────┘
                           │
                     ┌─────▼──────┐
                     │ GCS_Copter │ (Copter Manager - ArduCopter/GCS_Copter.h)
                     └─────┬──────┘
                           │ owns array of
                           │
                    ┌──────▼────────────┐
                    │  GCS_MAVLINK      │ (Base Channel - libraries/GCS_MAVLink/GCS.h)
                    └──────┬────────────┘
                           │
                ┌──────────▼─────────────┐
                │ GCS_MAVLINK_Copter     │ (Copter Channel - ArduCopter/GCS_MAVLink_Copter.h)
                └────────────────────────┘
```

### Object Relationships

```
┌─────────────────────────────────────────────────────────────┐
│                        Copter                                │
│                     (Main Vehicle)                           │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │              gcs (GCS_Copter instance)                 │ │
│  │                                                        │ │
│  │  _chan[0]: GCS_MAVLINK_Copter  (Serial1, USB)        │ │
│  │  _chan[1]: GCS_MAVLINK_Copter  (Serial2, Telemetry)  │ │
│  │  _chan[2]: GCS_MAVLINK_Copter  (Serial3, Optional)   │ │
│  │  ...                                                   │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                              │
│  Each GCS_MAVLINK_Copter has:                               │
│  - AP_HAL::UARTDriver *_port                                │
│  - mavlink_channel_t chan (COMM_0, COMM_1, etc)            │
│  - Message queues and buffers                               │
└─────────────────────────────────────────────────────────────┘
```

### Call Flow Example: Sending ATTITUDE

```
1. Scheduler calls: copter.gcs().update_send()
   ↓
2. GCS::update_send() loops through all channels
   ↓
3. For each channel: chan[i]->update_send()
   ↓
4. GCS_MAVLINK::update_send() checks message intervals
   ↓
5. If ATTITUDE due: try_send_message(MSG_ATTITUDE)
   ↓
6. GCS_MAVLINK::send_attitude() [can be overridden]
   ↓
7. Gets data from: AP::ahrs().get_attitude()
   ↓
8. Formats MAVLink packet
   ↓
9. mavlink_msg_attitude_send(chan, ...)
   ↓
10. comm_send_buffer(chan, packet_buffer, packet_len)
    ↓
11. _port->write(buffer, len)  // Actual UART transmission
```

---

## Message Flow

### Outgoing Messages (Vehicle → GCS)

#### Stream-Based Messages
Messages are grouped into streams with configurable rates:

**STREAM_EXTRA1** (Attitude data):
- ATTITUDE
- ATTITUDE_QUATERNION
- LOCAL_POSITION_NED

**STREAM_EXTRA2** (VFR data):
- VFR_HUD
- SIMSTATE (if in SITL)

**STREAM_POSITION** (Position):
- GLOBAL_POSITION_INT
- GPS_RAW_INT
- GPS_GLOBAL_ORIGIN

**STREAM_RAW_SENSORS** (Sensors):
- RAW_IMU
- SCALED_PRESSURE
- SCALED_IMU2, SCALED_IMU3

**STREAM_EXTENDED_STATUS** (Status):
- SYS_STATUS
- POWER_STATUS
- MCU_STATUS
- MEMINFO
- MISSION_CURRENT
- GPS_RAW_INT
- NAV_CONTROLLER_OUTPUT
- FENCE_STATUS

**Configuration**:
```cpp
// Set stream rates via parameters
// MAVn_RATE (master rate multiplier)
// SRn_* (individual stream rates)

// Example:
MAV1_RATE = 10       // Base rate: 10 Hz
SR1_EXTRA1 = 10      // ATTITUDE at 10 Hz
SR1_POSITION = 2     // GPS at 2 Hz
SR1_RAW_SENS = 1     // IMU at 1 Hz
```

#### Event-Based Messages
Some messages sent on events:
- STATUSTEXT (when send_text() called)
- COMMAND_ACK (after command execution)
- MISSION_ITEM_REACHED (waypoint events)
- HEARTBEAT (1 Hz always)

### Incoming Messages (GCS → Vehicle)

#### Receive Pipeline
```
UART Bytes → parse_char() → mavlink_parse_char()
           → mavlink_message_t complete
           → handle_message(msg)
           → route to specific handler
```

#### Common Handlers (in GCS_Common.cpp)
- **PARAM_REQUEST_LIST** → send all parameters
- **PARAM_REQUEST_READ** → send specific parameter
- **PARAM_SET** → set parameter value
- **MISSION_REQUEST_LIST** → start mission download
- **MISSION_COUNT** → start mission upload
- **MISSION_ITEM_INT** → receive mission item
- **COMMAND_INT** / **COMMAND_LONG** → execute command
- **SET_MODE** → change flight mode
- **RC_CHANNELS_OVERRIDE** → manual control
- **MANUAL_CONTROL** → joystick input
- **HEARTBEAT** → GCS presence

#### Copter-Specific Handlers (in GCS_MAVLink_Copter.cpp)
- **SET_ATTITUDE_TARGET** → guided mode attitude control
- **SET_POSITION_TARGET_LOCAL_NED** → guided mode position control
- **SET_POSITION_TARGET_GLOBAL_INT** → guided mode waypoint
- **LANDING_TARGET** → precision landing
- **MAV_CMD_NAV_TAKEOFF** → initiate takeoff
- **MAV_CMD_CONDITION_YAW** → set yaw target
- **MAV_CMD_DO_MOTOR_TEST** → test individual motors

---

## Integration Guide

### Integrating with GCS_MAVLink_Pro

If you're building a custom project that needs to interface with the existing GCS system:

#### Option 1: Use Existing GCS Channels

```cpp
// In your custom code
#include <GCS_MAVLink/GCS.h>

void your_function() {
    // Send text to all GCS connections
    gcs().send_text(MAV_SEVERITY_INFO, "Custom message from %s", "MyModule");

    // Send to a specific channel
    GCS_MAVLINK *chan0 = gcs().chan(0);
    if (chan0 != nullptr) {
        chan0->send_text(MAV_SEVERITY_WARNING, "Channel 0 specific");
    }

    // Send a named value
    gcs().send_named_float("my_value", 123.45f);
}
```

#### Option 2: Create Custom Message Handler

```cpp
// In GCS_MAVLink_Copter.cpp (or your vehicle)

void GCS_MAVLINK_Copter::handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_YOUR_CUSTOM_MESSAGE: {
        mavlink_your_custom_message_t packet;
        mavlink_msg_your_custom_message_decode(&msg, &packet);

        // Handle your custom message
        handle_your_custom_message(packet);
        break;
    }
    default:
        // Pass to base class
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}
```

#### Option 3: Add Custom Command

```cpp
// In GCS_MAVLink_Copter.cpp

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_YOUR_CUSTOM_COMMAND:
        return handle_your_custom_command(packet);

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

MAV_RESULT GCS_MAVLINK_Copter::handle_your_custom_command(
    const mavlink_command_int_t &packet)
{
    // param1, param2, etc. are command parameters
    float value1 = packet.param1;

    // Do your custom action
    bool success = copter.do_custom_thing(value1);

    return success ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
}
```

#### Option 4: Custom Periodic Message

```cpp
// Add to your vehicle's try_send_message()

bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id) {
    switch(id) {
    case MSG_YOUR_CUSTOM_MESSAGE:
        CHECK_PAYLOAD_SIZE(YOUR_CUSTOM_MESSAGE);
        send_your_custom_message();
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

void GCS_MAVLINK_Copter::send_your_custom_message() {
    mavlink_msg_your_custom_message_send(
        chan,
        AP_HAL::millis(),
        your_data_field1,
        your_data_field2
    );
}

// Then set message interval via MAVLink:
// MAV_CMD_SET_MESSAGE_INTERVAL with msgid=YOUR_CUSTOM_MESSAGE_ID, interval_us=100000 (10Hz)
```

---

## Implementation Guide

### Creating Your Own Vehicle-Specific GCS

If you want to create a completely new vehicle type (like ArduSub, ArduRover, etc.):

#### Step 1: Create Your GCS Manager Class

**MyVehicle/GCS_MyVehicle.h**:
```cpp
#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_MyVehicle.h"

class GCS_MyVehicle : public GCS {
    friend class MyVehicle;

public:
    // This macro creates chan() methods that return GCS_MAVLINK_MyVehicle*
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_MyVehicle);

    // Vehicle type identification
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;
    const char* frame_string() const override { return "MyVehicle"; }

    // Initialization state
    bool vehicle_initialised() const override;

    // Sensor status updates
    void update_vehicle_sensor_status_flags(void) override;

protected:
    // Timing constraint for sending messages
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 200;  // Adjust based on your control loop timing
    }

    // Factory method for creating channel objects
    GCS_MAVLINK_MyVehicle *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_MyVehicle(uart);
    }
};
```

**MyVehicle/GCS_MyVehicle.cpp**:
```cpp
#include "GCS_MyVehicle.h"
#include "MyVehicle.h"

uint32_t GCS_MyVehicle::custom_mode() const {
    return (uint32_t)myvehicle.current_mode;
}

MAV_TYPE GCS_MyVehicle::frame_type() const {
    return MAV_TYPE_GROUND_ROVER;  // Or whatever type fits
}

bool GCS_MyVehicle::vehicle_initialised() const {
    return myvehicle.initialised;
}

void GCS_MyVehicle::update_vehicle_sensor_status_flags(void) {
    // Add your vehicle-specific sensors
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    if (myvehicle.motors_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // Add more as needed
}
```

#### Step 2: Create Your Channel Class

**MyVehicle/GCS_MAVLink_MyVehicle.h**:
```cpp
#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_MyVehicle : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;  // Inherit base constructor

protected:
    // REQUIRED OVERRIDES - these are pure virtual in base class
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    // OPTIONAL OVERRIDES - customize as needed
    void send_attitude_target() override;
    void send_position_target_global_int() override;

    // VFR_HUD data
    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    // Message handling
    void handle_message(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;

    // Command handling
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;

private:
    // Your custom handlers
    MAV_RESULT handle_my_custom_command(const mavlink_command_int_t &packet);
    void handle_my_custom_message(const mavlink_message_t &msg);
};
```

**MyVehicle/GCS_MAVLink_MyVehicle.cpp**:
```cpp
#include "MyVehicle.h"
#include "GCS_MAVLink_MyVehicle.h"

// Required overrides

uint8_t GCS_MAVLINK_MyVehicle::base_mode() const {
    uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    if (myvehicle.is_armed()) {
        mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    if (myvehicle.mode_supports_manual_input()) {
        mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    if (myvehicle.in_auto_mode()) {
        mode |= MAV_MODE_FLAG_AUTO_ENABLED;
    }

    return mode;
}

MAV_STATE GCS_MAVLINK_MyVehicle::vehicle_system_status() const {
    if (!myvehicle.initialised) {
        return MAV_STATE_BOOT;
    }

    if (myvehicle.any_failsafe_triggered()) {
        return MAV_STATE_CRITICAL;
    }

    if (myvehicle.is_armed()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_MyVehicle::send_nav_controller_output() const {
    if (!myvehicle.initialised) {
        return;
    }

    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // roll target (degrees)
        0,  // pitch target
        myvehicle.nav_heading(),  // heading target
        myvehicle.bearing_to_target(),  // bearing to target
        myvehicle.distance_to_target(),  // distance to target (m)
        0,  // altitude error
        0,  // airspeed error
        0   // crosstrack error
    );
}

void GCS_MAVLINK_MyVehicle::send_pid_tuning() {
    // If you have PIDs to tune, send their data
    if (myvehicle.steering_controller != nullptr) {
        const AP_PIDInfo &pid_info = myvehicle.steering_controller->get_pid_info();

        mavlink_msg_pid_tuning_send(
            chan,
            PID_TUNING_STEERING,  // axis
            pid_info.target,
            pid_info.actual,
            pid_info.FF,
            pid_info.P,
            pid_info.I,
            pid_info.D,
            pid_info.slew_rate,
            pid_info.Dmod
        );
    }
}

// Optional overrides

float GCS_MAVLINK_MyVehicle::vfr_hud_airspeed() const {
    return myvehicle.get_speed();
}

int16_t GCS_MAVLINK_MyVehicle::vfr_hud_throttle() const {
    return myvehicle.get_throttle_percentage();
}

float GCS_MAVLINK_MyVehicle::vfr_hud_alt() const {
    return myvehicle.get_altitude();
}

// Message handling

void GCS_MAVLINK_MyVehicle::handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MY_CUSTOM_MESSAGE:
        handle_my_custom_message(msg);
        break;

    default:
        // Always call base class for unhandled messages
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

bool GCS_MAVLINK_MyVehicle::try_send_message(enum ap_message id) {
    switch(id) {
    case MSG_MY_CUSTOM_DATA:
        CHECK_PAYLOAD_SIZE(MY_CUSTOM_DATA);
        send_my_custom_data();
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

// Command handling

MAV_RESULT GCS_MAVLINK_MyVehicle::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {
    case MAV_CMD_MY_CUSTOM_COMMAND:
        return handle_my_custom_command(packet);

    case MAV_CMD_NAV_WAYPOINT:
        // Custom waypoint handling
        return handle_nav_waypoint(packet);

    default:
        // Always call base class for unhandled commands
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}
```

#### Step 3: Instantiate in Your Vehicle Class

**MyVehicle/MyVehicle.h**:
```cpp
#pragma once

#include "GCS_MyVehicle.h"

class MyVehicle {
public:
    MyVehicle();

    // ... other vehicle members ...

    // GCS instance
    GCS_MyVehicle gcs_instance;
    GCS_MyVehicle &gcs() { return gcs_instance; }

    // Control loops
    void fast_loop();  // ~400Hz
    void main_loop();  // ~50Hz
    void slow_loop();  // ~10Hz

private:
    bool initialised;
    // ... other state ...
};

// Global accessor
extern MyVehicle myvehicle;
inline GCS &gcs() { return myvehicle.gcs(); }
```

**MyVehicle/MyVehicle.cpp**:
```cpp
#include "MyVehicle.h"

MyVehicle myvehicle;

void MyVehicle::main_loop() {
    // Update sensors, run control loops, etc.
    // ...

    // Update GCS communication
    gcs().update_receive();  // Parse incoming messages
    gcs().update_send();     // Send outgoing messages
}
```

#### Step 4: Initialize in Setup

**MyVehicle/system.cpp**:
```cpp
void MyVehicle::init_ardupilot() {
    // Initialize serial ports
    serial_manager.init();

    // Initialize GCS
    gcs().init();
    gcs().setup_console();  // Setup console/USB port
    gcs().setup_uarts();    // Setup telemetry ports

    // ... rest of initialization ...

    initialised = true;
}
```

### Step 5: Add Build Configuration

**MyVehicle/wscript**:
```python
def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'GCS_MAVLink',
            # ... other libraries ...
        ],
    )
```

---

## Advanced Topics

### Custom Streams

To add a custom stream with your own messages:

1. **Define stream in GCS.h** (modify enum streams)
2. **Add stream rate parameter** (modify var_info in GCS_MAVLINK_Parameters.cpp)
3. **Map messages to stream** (modify all_stream_entries[])
4. **Implement message sender** (in your GCS_MAVLINK_MyVehicle class)

### Message Intervals from Files

You can configure message intervals via SD card files:

**@SYS/message-intervals-chan0.txt**:
```
# Message intervals for channel 0 (milliseconds)
ATTITUDE 100        # 10 Hz
GPS_RAW_INT 200     # 5 Hz
GLOBAL_POSITION_INT 200
VFR_HUD 200
```

The system will read these on boot and configure intervals accordingly.

### MAVLink Signing

For secure communication:

```cpp
// Enabled automatically if using SerialProtocol_MAVLink2
// Key is stored in @SYS/signing_key.dat
// Timestamp stored in @SYS/signing_timestamp

// To require signing:
// Set MAVn_OPTIONS bit 0 = 0 (enable signing)

// The base class handles all signing automatically
```

### High Latency Links

For satellite or very slow links:

```cpp
// Use SerialProtocol_MAVLinkHL
// Enables HIGH_LATENCY2 message instead of regular telemetry
// Compresses all telemetry into one message every few seconds
```

### Private Channels

To prevent message forwarding on a channel:

```cpp
// Set MAVn_OPTIONS bit 1 = 1 (NO_FORWARD)
// This channel won't forward messages to/from other channels
// Useful for one-to-one point-to-point protocols
```

---

## Debugging Tips

### Enable MAVLink Debugging

```cpp
// In GCS.h, set:
#define GCS_DEBUG_SEND_MESSAGE_TIMINGS 1

// Tracks:
// - Longest message send time
// - Messages that didn't fit in buffer
// - Stream timing statistics
```

### Check Channel Status

```cpp
GCS_MAVLINK *link = gcs().chan(0);
if (link != nullptr) {
    bool is_active = link->is_active();      // Received heartbeat?
    bool is_streaming = link->is_streaming(); // Sending stream messages?
    uint16_t tx_space = link->txspace();     // Bytes available in buffer
    uint32_t last_hb = link->get_last_heartbeat_time();
}
```

### Monitor Message Counts

```cpp
// In GCS_Common.cpp:
uint16_t send_packet_count;  // Total packets sent
uint16_t out_of_space_to_send_count;  // Packets dropped due to full buffer
```

### Parameter Debugging

```cpp
// Get parameter values:
gcs().sysid_this_mav()       // Our MAVLink system ID
gcs().telem_delay()          // Startup delay
link->get_stream_slowdown_ms()  // Additional delay per message
```

---

## Summary

The ArduPilot GCS MAVLink system is a sophisticated multi-layer architecture:

1. **GCS_MAVLink.h/.cpp**: Low-level protocol and communication primitives
2. **GCS.h/.cpp**: Base classes for global manager and channels
3. **GCS_Common.cpp**: Shared message handlers for all vehicles
4. **GCS_Copter.h/.cpp**: Copter-specific global manager
5. **GCS_MAVLink_Copter.h/.cpp**: Copter-specific channel implementation

The design allows:
- Multiple simultaneous telemetry connections
- Vehicle-specific customization via inheritance
- Efficient message streaming with configurable rates
- Extensible command and message handling
- Protocol versioning (MAVLink 1/2)
- Security (signing) and special link types (high latency)

To integrate or extend:
- Use existing GCS instances for simple tasks
- Override message handlers for custom messages
- Add command handlers for custom commands
- Create full vehicle-specific subclasses for new vehicle types

All communication flows through the update_send()/update_receive() loop called from the main scheduler, ensuring predictable timing and allowing prioritization of flight control over communications.
