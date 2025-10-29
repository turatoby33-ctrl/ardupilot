# EduCopter FC_GCS_MAVLink Integration Guide

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Integration Steps](#integration-steps)
4. [Required External Functions](#required-external-functions)
5. [Parameter System](#parameter-system)
6. [Mission/Fence/Rally System](#missionfencerally-system)
7. [Custom GCS Channel](#custom-gcs-channel)
8. [Main Loop Integration](#main-loop-integration)
9. [Configuration](#configuration)
10. [Testing](#testing)
11. [Troubleshooting](#troubleshooting)

---

## Overview

The EduCopter FC_GCS_MAVLink system provides complete MAVLink 2.0 communication for your flight controller. It includes:

- ✅ **27 custom files** (10 headers + 17 implementations)
- ✅ **Full MAVLink 2.0 protocol** support
- ✅ **Parameter management** (read/write/save/load)
- ✅ **Mission protocol** (waypoints, fence, rally)
- ✅ **File transfer** (MAVLink FTP)
- ✅ **Message signing** (SHA-256 HMAC security)
- ✅ **Stream rate configuration**
- ✅ **Servo/relay control**
- ✅ **Serial passthrough**
- ✅ **Device operations** (I2C/SPI access)

**Total Code**: ~6,500 lines of custom EduCopter implementation

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Flight Controller                    │
│                                                               │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐            │
│  │  Sensors   │  │   Motors   │  │  Battery   │            │
│  └─────┬──────┘  └──────┬─────┘  └──────┬─────┘            │
│        │                │                │                   │
│        └────────────────┴────────────────┘                   │
│                         │                                    │
│        ┌────────────────▼────────────────┐                  │
│        │   Vehicle State Manager         │                  │
│        │  (attitude, position, battery)  │                  │
│        └────────────────┬────────────────┘                  │
│                         │                                    │
│        ┌────────────────▼────────────────┐                  │
│        │   EduCopter GCS System          │                  │
│        │   (FC_GCS_MAVLink)              │                  │
│        │                                  │                  │
│        │  ┌──────────────────────────┐   │                  │
│        │  │  GCS Singleton           │   │                  │
│        │  │  - Manages channels      │   │                  │
│        │  │  - Routes messages       │   │                  │
│        │  └────────┬─────────────────┘   │                  │
│        │           │                      │                  │
│        │  ┌────────▼─────────┐           │                  │
│        │  │  GCS Channel(s)  │           │                  │
│        │  │  - USB/UART0/... │           │                  │
│        │  │  - Send/Receive  │           │                  │
│        │  │  - Scheduling    │           │                  │
│        │  └────────┬─────────┘           │                  │
│        │           │                      │                  │
│        │  ┌────────▼─────────┐           │                  │
│        │  │  Protocol        │           │                  │
│        │  │  - Parameters    │           │                  │
│        │  │  - Mission       │           │                  │
│        │  │  - Fence/Rally   │           │                  │
│        │  │  - FTP           │           │                  │
│        │  └──────────────────┘           │                  │
│        └─────────────────────────────────┘                  │
│                         │                                    │
└─────────────────────────┼────────────────────────────────────┘
                          │
                    ┌─────▼──────┐
                    │  UART/USB  │
                    └─────┬──────┘
                          │
                    ┌─────▼──────┐
                    │    GCS     │
                    │ (Mission   │
                    │  Planner)  │
                    └────────────┘
```

---

## Integration Steps

### Step 1: Add Files to Your Project

Copy all 27 files from `FC_GCS_MAVLink/` to your project:

```
EduCopter/FC_GCS_MAVLink/
├── ap_message.h                         (Message IDs)
├── GCS_config.h                         (Configuration)
├── GCS_MAVLink.h/cpp                    (MAVLink protocol)
├── MAVLink_routing.h/cpp                (Message routing)
├── GCS.h/cpp                            (Main GCS manager)
├── GCS_Common.cpp                       (Common handlers)
├── GCS_Param.cpp                        (Parameters)
├── GCS_MAVLink_Parameters.cpp           (Stream rates)
├── MissionItemProtocol.h/cpp            (Mission base)
├── MissionItemProtocol_Waypoints.h/cpp  (Waypoints)
├── MissionItemProtocol_Fence.h/cpp      (Geofence)
├── MissionItemProtocol_Rally.h/cpp      (Rally points)
├── GCS_FTP.h/cpp                        (File transfer)
├── GCS_Fence.cpp                        (Fence handlers)
├── GCS_Rally.cpp                        (Rally handlers)
├── GCS_ServoRelay.cpp                   (Servo/relay)
├── GCS_Signing.cpp                      (Message signing)
├── GCS_serial_control.cpp               (Serial passthrough)
└── GCS_DeviceOp.cpp                     (Device operations)
```

### Step 2: Configure Build System

Add to your CMakeLists.txt or Makefile:

```cmake
# Add GCS source files
set(GCS_SOURCES
    FC_GCS_MAVLink/GCS_MAVLink.cpp
    FC_GCS_MAVLink/MAVLink_routing.cpp
    FC_GCS_MAVLink/GCS.cpp
    FC_GCS_MAVLink/GCS_Common.cpp
    FC_GCS_MAVLink/GCS_Param.cpp
    FC_GCS_MAVLink/GCS_MAVLink_Parameters.cpp
    FC_GCS_MAVLink/MissionItemProtocol.cpp
    FC_GCS_MAVLink/MissionItemProtocol_Waypoints.cpp
    FC_GCS_MAVLink/MissionItemProtocol_Fence.cpp
    FC_GCS_MAVLink/MissionItemProtocol_Rally.cpp
    FC_GCS_MAVLink/GCS_FTP.cpp
    FC_GCS_MAVLink/GCS_Fence.cpp
    FC_GCS_MAVLink/GCS_Rally.cpp
    FC_GCS_MAVLink/GCS_ServoRelay.cpp
    FC_GCS_MAVLink/GCS_Signing.cpp
    FC_GCS_MAVLink/GCS_serial_control.cpp
    FC_GCS_MAVLink/GCS_DeviceOp.cpp
)

# Add to your executable
add_executable(EduCopter
    ${YOUR_SOURCES}
    ${GCS_SOURCES}
)

# Add include directory
target_include_directories(EduCopter PRIVATE
    FC_GCS_MAVLink
    path/to/mavlink/include/mavlink/v2.0
)
```

### Step 3: Install MAVLink Headers

Download MAVLink C library:

```bash
git clone https://github.com/mavlink/c_library_v2.git
# Or use: https://github.com/mavlink/mavlink/releases
```

Add to include path:
```
-I/path/to/mavlink/include/mavlink/v2.0
```

---

## Required External Functions

You must implement these functions in your vehicle code:

### Time Functions
```cpp
uint32_t millis();           // Milliseconds since boot
uint16_t millis16();         // Milliseconds (16-bit)
```

### Attitude Functions
```cpp
float getAttitudeRoll();     // Roll in radians
float getAttitudePitch();    // Pitch in radians
float getAttitudeYaw();      // Yaw in radians
float getAttitudeRollRate(); // Roll rate in rad/s
float getAttitudePitchRate();// Pitch rate in rad/s
float getAttitudeYawRate();  // Yaw rate in rad/s
```

### Position Functions
```cpp
int32_t getLatitude();       // Latitude in degE7
int32_t getLongitude();      // Longitude in degE7
float getAltitude();         // Altitude MSL in meters
float getRelativeAltitude(); // Altitude AGL in meters
int16_t getVelocityX();      // North velocity in cm/s
int16_t getVelocityY();      // East velocity in cm/s
int16_t getVelocityZ();      // Down velocity in cm/s
uint16_t getHeading();       // Heading in centidegrees
```

### Battery Functions
```cpp
float getBatteryVoltage();   // Voltage in volts
float getBatteryCurrent();   // Current in amperes
int8_t getBatteryRemaining();// Percentage 0-100
```

### System Functions
```cpp
uint16_t getCPULoad();       // CPU load in d% (0-1000)
uint32_t getSensorsPresentMask();  // Sensor bitmask
uint32_t getSensorsEnabledMask();  // Enabled sensors
uint32_t getSensorsHealthMask();   // Healthy sensors
```

See `FC_GCS_Integration.cpp` for complete implementation examples.

---

## Parameter System

### Define Your Parameters

```cpp
struct Parameter {
    const char* name;
    float value;
    float min;
    float max;
    float defaultValue;
};

static Parameter s_params[] = {
    {"PID_ROLL_P", 0.15f, 0.0f, 1.0f, 0.15f},
    {"PID_ROLL_I", 0.05f, 0.0f, 0.5f, 0.05f},
    // ... add more parameters ...
};
```

### Implement Parameter Interface

```cpp
uint16_t getParameterCount() {
    return sizeof(s_params) / sizeof(s_params[0]);
}

const char* getParameterName(uint16_t index) {
    return s_params[index].name;
}

float getParameterValue(uint16_t index) {
    return s_params[index].value;
}

bool setParameterValue(uint16_t index, float value) {
    if (value < s_params[index].min ||
        value > s_params[index].max) {
        return false;
    }
    s_params[index].value = value;
    return true;
}

int16_t findParameterIndex(const char* name) {
    for (uint16_t i = 0; i < getParameterCount(); i++) {
        if (strcmp(s_params[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

MAV_PARAM_TYPE getParameterType(uint16_t index) {
    return MAV_PARAM_TYPE_REAL32;
}
```

### Parameter Persistence (Optional)

```cpp
bool saveParameters() {
    // Save to EEPROM/Flash
    return true;
}

bool loadParameters() {
    // Load from EEPROM/Flash
    return true;
}

bool resetParameters() {
    // Reset to defaults
    for (uint16_t i = 0; i < getParameterCount(); i++) {
        s_params[i].value = s_params[i].defaultValue;
    }
    return true;
}
```

---

## Mission/Fence/Rally System

### Implement Mission Storage

```cpp
// Waypoint storage
static mavlink_mission_item_int_t s_waypoints[255];
static uint16_t s_waypointCount = 0;

uint16_t getWaypointCount() {
    return s_waypointCount;
}

bool getWaypoint(uint16_t index, mavlink_mission_item_int_t& outItem) {
    if (index >= s_waypointCount) return false;
    outItem = s_waypoints[index];
    return true;
}

bool appendWaypoint(const mavlink_mission_item_int_t& item) {
    if (s_waypointCount >= 255) return false;
    s_waypoints[s_waypointCount++] = item;
    return true;
}

bool clearWaypoints() {
    s_waypointCount = 0;
    return true;
}
```

Repeat similar implementations for fence points and rally points.

---

## Custom GCS Channel

Create your vehicle-specific GCS channel:

```cpp
#include "GCS.h"

namespace EduCopter {
namespace GCS {

class EduCopterGCSChannel : public GCSChannel {
public:
    explicit EduCopterGCSChannel(uint8_t channelID)
        : GCSChannel(channelID) {}

    // Override pure virtual methods

    void sendNavControllerOutput() override {
        // Send your nav data
    }

    void sendPIDTuning() override {
        // Send PID tuning info
    }

    uint8_t getBaseMode() const override {
        uint8_t mode = 0;
        if (isArmed()) mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        return mode;
    }

    MAV_STATE getSystemStatus() const override {
        if (isCalibrating()) return MAV_STATE_CALIBRATING;
        if (isArmed()) return MAV_STATE_ACTIVE;
        return MAV_STATE_STANDBY;
    }
};

} // namespace GCS
} // namespace EduCopter
```

---

## Main Loop Integration

### Initialization (in setup())

```cpp
#include "GCS.h"

void setup() {
    // ... your initialization ...

    // Initialize GCS
    EduCopter::GCS::GCS& gcs = EduCopter::GCS::GCS::getInstance();
    gcs.initialize(1); // System ID = 1

    // GCS is ready!
}
```

### Main Loop (in loop())

```cpp
void loop() {
    // Update flight controller
    updateSensors();
    updateEstimator();
    updateController();
    updateMotors();

    // Update GCS (add these two lines)
    EduCopter::GCS::GCS& gcs = EduCopter::GCS::GCS::getInstance();
    gcs.updateReceive();  // Process incoming messages
    gcs.updateSend();     // Send outgoing messages

    // Continue with rest of loop
}
```

That's it! Only 2 lines needed in your main loop.

---

## Configuration

Edit `GCS_config.h` to enable/disable features:

```cpp
// Enable/disable entire GCS system
#define EDUCOPTER_GCS_ENABLED 1

// Number of MAVLink channels (USB + UARTs)
#define EDUCOPTER_MAX_MAVLINK_CHANNELS 4

// Enable MAVLink 2.0
#define EDUCOPTER_MAVLINK2_ENABLED 1

// Enable mission planning
#define EDUCOPTER_MISSION_ENABLED 1

// Enable geofence
#define EDUCOPTER_FENCE_ENABLED 1

// Enable rally points
#define EDUCOPTER_RALLY_ENABLED 1

// Enable file transfer (FTP)
#define EDUCOPTER_FTP_ENABLED 1

// Enable message signing (security)
#define EDUCOPTER_SIGNING_ENABLED 1

// Enable servo/relay control
#define EDUCOPTER_SERVO_RELAY_ENABLED 1

// Enable serial passthrough
#define EDUCOPTER_SERIAL_CONTROL_ENABLED 1

// Enable device operations
#define EDUCOPTER_DEVICE_OP_ENABLED 1
```

---

## Testing

### Test with Mission Planner

1. **Connect via USB/UART**
   - Set baudrate to 57600 or 115200
   - Vehicle should appear in Mission Planner

2. **Verify Heartbeat**
   - Should see heartbeat messages
   - System status should update

3. **Test Parameters**
   - Go to CONFIG > Full Parameter List
   - Should see all your parameters
   - Try changing a value

4. **Test Mission Upload**
   - Go to FLIGHT PLAN tab
   - Right-click map, add waypoints
   - Click "Write WPs"
   - Should upload successfully

5. **Test Telemetry**
   - Go to FLIGHT DATA tab
   - Should see attitude, position, battery
   - Check HUD displays correctly

### Test with QGroundControl

1. **Application Settings > Comm Links**
   - Add serial link
   - Connect

2. **Verify Vehicle Type**
   - Should show as quadrotor

3. **Test Parameters**
   - Vehicle Setup > Parameters
   - Browse and modify

4. **Test Mission**
   - Plan View
   - Create mission
   - Upload

---

## Troubleshooting

### No Connection

**Problem**: GCS doesn't connect

**Solutions**:
- Check UART pins are correct
- Verify baudrate matches (57600/115200)
- Check `EDUCOPTER_GCS_ENABLED` is 1
- Verify `updateReceive()` is being called
- Add debug print in `handleHeartbeat()`

### Parameters Not Showing

**Problem**: Parameter list is empty

**Solutions**:
- Verify `getParameterCount()` returns correct count
- Check `getParameterName()` returns valid strings
- Ensure `handleParamRequestList()` is called
- Add debug print in `sendParameter()`

### Mission Upload Fails

**Problem**: Mission upload times out

**Solutions**:
- Check `EDUCOPTER_MISSION_ENABLED` is 1
- Verify waypoint storage functions work
- Ensure `handleMissionCount()` is being called
- Check `appendWaypoint()` returns true

### Telemetry Not Updating

**Problem**: Attitude/position not updating

**Solutions**:
- Verify external functions return valid data
- Check stream rates are configured
- Ensure `updateSend()` is being called
- Verify `hasPayloadSpace()` returns true

### High CPU Usage

**Problem**: GCS uses too much CPU

**Solutions**:
- Reduce stream rates
- Increase message bucket intervals
- Disable unused features in `GCS_config.h`
- Use lower baudrate if needed

---

## Performance Tips

1. **Optimize Stream Rates**
   - Attitude: 10-50Hz (high priority)
   - Position: 5-10Hz (medium priority)
   - Status: 1-2Hz (low priority)

2. **Buffer Sizes**
   - Increase TX buffer if messages are dropped
   - Adjust `EDUCOPTER_TXBUF_SIZE` in config

3. **Message Priorities**
   - Heartbeat and attitude are highest priority
   - Mission items are lowest priority

4. **Disable Unused Features**
   - Turn off FTP if not needed
   - Disable signing for better performance
   - Reduce number of channels if possible

---

## Next Steps

1. ✅ Complete integration guide (this document)
2. ✅ Study use case examples (`FC_GCS_UseCases.cpp`)
3. ✅ Review integration example (`FC_GCS_Integration.cpp`)
4. 🔄 Implement external functions for your vehicle
5. 🔄 Create your parameter table
6. 🔄 Test with Mission Planner/QGC
7. 🔄 Fine-tune stream rates and performance

---

## Support & Resources

- **Example Files**:
  - `FC_GCS_Integration.cpp` - Complete integration example
  - `FC_GCS_UseCases.cpp` - 10 detailed use cases

- **MAVLink Resources**:
  - https://mavlink.io/en/
  - https://mavlink.io/en/messages/common.html

- **Ground Station Software**:
  - Mission Planner: https://ardupilot.org/planner/
  - QGroundControl: http://qgroundcontrol.com/

---

**Happy Flying! 🚁**

*EduCopter Development Team*
