# FC_GCS_MAVLink - EduCopter Ground Control Station MAVLink Implementation

## Overview

This directory contains a complete copy of the ArduPilot GCS_MAVLink communication system, adapted for the EduCopter project. These files implement the MAVLink protocol for bidirectional communication between the flight controller and ground control stations (GCS).

**Created:** October 28, 2025
**Source:** ArduPilot libraries/GCS_MAVLink
**Purpose:** Educational multirotor platform with full MAVLink telemetry and control

---

## Directory Structure

### Configuration Files (1 file)
- `GCS_config.h` - Feature flags and conditional compilation settings

### Core Header Files (9 files)
- `GCS.h` - Main GCS class definitions and interfaces
- `GCS_MAVLink.h` - MAVLink protocol integration header
- `GCS_FTP.h` - File Transfer Protocol over MAVLink
- `MAVLink_routing.h` - Message routing between channels
- `MissionItemProtocol.h` - Base mission item transfer protocol
- `MissionItemProtocol_Fence.h` - Geofence upload/download
- `MissionItemProtocol_Rally.h` - Rally point management
- `MissionItemProtocol_Waypoints.h` - Waypoint/mission management
- `ap_message.h` - Message ID enumeration

### Core Implementation Files (17 files)
- `GCS.cpp` - GCS singleton and initialization
- `GCS_Common.cpp` - Common GCS message handling (7,428 lines)
- `GCS_MAVLink.cpp` - MAVLink channel setup and management
- `GCS_MAVLink_Parameters.cpp` - Stream rate configuration
- `GCS_Param.cpp` - Parameter get/set protocol
- `GCS_DeviceOp.cpp` - Device operation (I2C/SPI) via MAVLink
- `GCS_FTP.cpp` - FTP session management
- `GCS_Fence.cpp` - Fence parameter handling
- `GCS_Rally.cpp` - Rally point message handling
- `GCS_ServoRelay.cpp` - Servo and relay control
- `GCS_Signing.cpp` - Message signing for security
- `GCS_serial_control.cpp` - Serial port passthrough
- `MAVLink_routing.cpp` - Routing table and forwarding
- `MissionItemProtocol.cpp` - Base mission transfer logic
- `MissionItemProtocol_Fence.cpp` - Fence-specific protocol
- `MissionItemProtocol_Rally.cpp` - Rally-specific protocol
- `MissionItemProtocol_Waypoints.cpp` - Waypoint-specific protocol

---

## File-by-File Analysis

### 1. GCS_config.h (143 lines)
**Purpose:** Configuration and feature flags for GCS system

**Key Definitions:**
- `HAL_GCS_ENABLED` - Master enable for GCS system
- `AP_MAVLINK_SIGNING_ENABLED` - Message authentication
- `HAL_HIGH_LATENCY2_ENABLED` - High latency link support
- `AP_MAVLINK_FTP_ENABLED` - File transfer protocol
- `AP_MAVLINK_COMMAND_LONG_ENABLED` - Legacy command support

**Function:** Controls which MAVLink features are compiled into the firmware based on vehicle configuration and flash space availability.

---

### 2. ap_message.h (118 lines)
**Purpose:** Enumeration of all message types that can be sent

**Key Content:**
```cpp
enum ap_message : uint8_t {
    MSG_HEARTBEAT = 0,
    MSG_ATTITUDE = 3,
    MSG_LOCATION = 5,
    MSG_SYS_STATUS = 7,
    MSG_GPS_RAW = 22,
    MSG_BATTERY_STATUS = 65,
    // ... ~100 message types total
};
```

**Function:** Provides unified message IDs used internally to queue and prioritize outgoing MAVLink messages.

---

### 3. GCS.h (1,429 lines)
**Purpose:** Core class definitions for GCS system

**Major Classes:**

#### 3.1 `GCS_MAVLINK` Class
The per-channel MAVLink handler. Each telemetry port gets one instance.

**Key Methods:**
- `update_receive()` - Process incoming MAVLink packets
- `update_send()` - Send queued messages
- `send_message(ap_message id)` - Queue a message to send
- `send_text()` - Send text messages to GCS
- `handle_message()` - Process received messages
- `packetReceived()` - Called when complete packet received

**Key Members:**
- `chan` - MAVLink channel number
- `_port` - UART driver pointer
- `streamRates[]` - Message stream rate configuration
- `deferred_message_bucket[]` - Message scheduling system

**Function:** Manages one MAVLink communication channel, handling message I/O, routing, and protocol state.

#### 3.2 `GCS` Class
Global singleton managing all GCS channels.

**Key Methods:**
- `send_text()` - Broadcast text to all channels
- `send_message()` - Send to all active channels
- `update_send()` - Update all channels
- `update_receive()` - Receive on all channels

**Function:** Coordinates multiple MAVLink channels and provides vehicle-wide GCS services.

---

### 4. GCS_MAVLink.h (88 lines)
**Purpose:** MAVLink library integration

**Key Content:**
- Includes MAVLink v2.0 protocol definitions
- Defines channel buffer management
- Provides convenience macros for packet handling
- Sets up multi-threading support

**Important Macros:**
- `MAVLINK_COMM_NUM_BUFFERS` - Number of channels (5-8)
- `PAYLOAD_SIZE(chan, id)` - Calculate message size
- `HAVE_PAYLOAD_SPACE(chan, id)` - Check if message fits
- `CHECK_PAYLOAD_SIZE(id)` - Guard against buffer overflow

---

### 5. MAVLink_routing.h & .cpp (80 + 476 lines)
**Purpose:** Intelligent message routing and forwarding

**Class:** `MAVLink_routing`

**Key Features:**
- Learns routes automatically from heartbeats
- Forwards messages between channels
- Supports up to 20 simultaneous routes
- Component discovery (e.g., find gimbal)

**Key Methods:**
- `check_and_forward()` - Route incoming message
- `learn_route()` - Add new route from heartbeat
- `send_to_components()` - Multicast to onboard components
- `find_by_mavtype()` - Locate specific component type

**Use Case:** Allows GCS on one channel to control a gimbal on another channel, or forward messages between multiple GCS.

---

### 6. GCS.cpp (604 lines)
**Purpose:** GCS singleton implementation

**Key Functions:**

#### `init()` - Initialize GCS system
- Registers parameter groups
- Creates MissionItemProtocol objects
- Sets up message routing

#### `setup_uarts()` - Configure telemetry ports
- Scans serial manager for GCS ports
- Creates GCS_MAVLINK backend for each
- Starts alternative protocols (FrSky, LTM, DEVO)

#### `send_text()` - Broadcast text messages
- Queues statustext for all channels
- Thread-safe with semaphore
- Handles message severity levels

#### `update_send() / update_receive()`
- Round-robin through all channels
- Prevents starvation of later channels
- Checks timing constraints

**Routing Setup:**
Creates protocol handlers for:
- Waypoint missions
- Rally points
- Geofences

---

### 7. GCS_Common.cpp (7,428 lines) ⭐ **LARGEST FILE**
**Purpose:** Core message handling and sending

This is the heart of the GCS system. Major sections:

#### 7.1 Message Sending Functions
Hundreds of functions to encode and send MAVLink messages:

- `send_heartbeat()` - Vehicle heartbeat
- `send_attitude()` - Roll/pitch/yaw
- `send_global_position_int()` - GPS position
- `send_sys_status()` - System health
- `send_battery_status()` - Power status
- `send_vfr_hud()` - HUD data
- `send_rc_channels()` - RC input
- `send_raw_imu()` - IMU data
- ... 100+ send functions

#### 7.2 Message Receiving/Handling
Processes incoming commands and data:

- `handle_command_int()` - Execute MAV commands
- `handle_param_request()` - Parameter protocol
- `handle_mission_item()` - Mission upload
- `handle_rc_channels_override()` - RC control
- `handle_manual_control()` - Joystick input
- `handle_set_mode()` - Mode changes
- ... 80+ handle functions

#### 7.3 Command Handlers
Specific command implementations:

- `handle_command_preflight_calibration()` - Sensor cal
- `handle_command_do_set_home()` - Home position
- `handle_command_component_arm_disarm()` - Arm/disarm
- `handle_command_do_fence_enable()` - Geofence control
- `handle_command_set_message_interval()` - Stream config
- ... 50+ command handlers

#### 7.4 Message Scheduling
Intelligent message prioritization:

- `deferred_message[]` - High priority queue
- `deferred_message_bucket[]` - Stream-based scheduling
- `find_next_bucket_to_send()` - Round-robin scheduler
- `get_reschedule_interval_ms()` - Adaptive timing

**Key Innovation:** Messages are grouped into "buckets" by stream rate, ensuring fair distribution of telemetry bandwidth.

---

### 8. GCS_MAVLink.cpp (164 lines)
**Purpose:** Channel initialization and packet reception

**Key Functions:**

#### `init()` - Initialize channel
- Set up mavlink system/component ID
- Configure signing if enabled
- Initialize message intervals
- Set channel active

#### `packetReceived()` - Process complete packet
- Check packet validity
- Learn routing
- Forward if needed
- Dispatch to handler
- Log statistics

#### `update_receive()` - Poll for data
- Read bytes from UART
- Parse MAVLink frames
- Handle partial packets
- Timeout detection

#### `update_send()` - Transmit queued messages
- Check deferred messages
- Process buckets
- Send parameters
- Send mission items
- Handle flow control

---

### 9. GCS_MAVLink_Parameters.cpp (438 lines)
**Purpose:** Configure message stream rates

**Streams Defined:**
1. `STREAM_RAW_SENSORS` - IMU, baro, mag
2. `STREAM_EXTENDED_STATUS` - Status, mode, battery
3. `STREAM_RC_CHANNELS` - RC input
4. `STREAM_RAW_CONTROLLER` - PID outputs
5. `STREAM_POSITION` - GPS, attitude
6. `STREAM_EXTRA1` - Attitude, simulation
7. `STREAM_EXTRA2` - VFR_HUD
8. `STREAM_EXTRA3` - AHRS, position, wind
9. `STREAM_PARAMS` - Parameters
10. `STREAM_ADSB` - Traffic

**Key Functions:**
- `handle_request_data_stream()` - Legacy rate setting
- `set_message_interval()` - Modern message intervals
- `initialise_message_intervals_from_streamrates()` - Apply rates
- `cap_message_interval()` - Prevent excessive rates

**Configuration:** Rates stored as AP_Int16 parameters (SR0_*, SR1_*, etc.)

---

### 10. GCS_Param.cpp (453 lines)
**Purpose:** Parameter get/set protocol implementation

**Protocol Support:**
- `PARAM_REQUEST_LIST` - Download all parameters
- `PARAM_REQUEST_READ` - Get single parameter
- `PARAM_SET` - Change parameter value
- `PARAM_VALUE` - Parameter response

**Key Features:**
- Asynchronous parameter sending
- Request/reply queuing
- Parameter name lookup
- Type conversion
- Change logging

**Key Functions:**
- `handle_param_request_list()` - Start parameter dump
- `handle_param_request_read()` - Single param query
- `handle_param_set()` - Validate and set parameter
- `queued_param_send()` - Background parameter sending
- `send_parameter_value()` - Encode and transmit

**Safety:** Parameters are validated before being set, and some require reboot to take effect.

---

### 11. GCS_Fence.cpp (105 lines)
**Purpose:** Geofence parameter message handling

**Messages Handled:**
- `FENCE_STATUS` - Send fence breach status
- `FENCE_POINT` - Upload fence vertex (deprecated)
- `FENCE_FETCH_POINT` - Download fence vertex (deprecated)

**Note:** Modern fence handling uses MissionItemProtocol_Fence. These are legacy handlers.

**Key Function:**
- `handle_fence_message()` - Route fence-related messages

---

### 12. GCS_Rally.cpp (105 lines)
**Purpose:** Rally point message handling

**Messages Handled:**
- `RALLY_POINT` - Upload rally point (deprecated)
- `RALLY_FETCH_POINT` - Download rally point (deprecated)

**Note:** Modern rally handling uses MissionItemProtocol_Rally. Legacy protocol being phased out.

**Key Functions:**
- `handle_common_rally_message()` - Route rally messages
- `handle_rally_point()` - Process uploaded rally
- `handle_rally_fetch_point()` - Send rally to GCS

---

### 13. GCS_ServoRelay.cpp (41 lines)
**Purpose:** Direct servo and relay control via MAVLink

**Commands Handled:**
- `MAV_CMD_DO_SET_SERVO` - Set servo PWM
- `MAV_CMD_DO_SET_RELAY` - Set relay on/off
- `MAV_CMD_DO_REPEAT_SERVO` - Pulse servo
- `MAV_CMD_DO_REPEAT_RELAY` - Pulse relay

**Use Case:** Camera triggers, sprayers, lights, auxiliary actuators.

---

### 14. GCS_Signing.cpp (250 lines)
**Purpose:** MAVLink 2.0 message authentication

**Features:**
- SHA-256 message signatures
- Timestamp-based replay protection
- Key storage in EEPROM
- Setup via `SETUP_SIGNING` message

**Security Model:**
- Shared secret key (256-bit)
- Incrementing timestamp
- Accept window for clock drift
- Optional enforcement

**Key Functions:**
- `signing_key_save()` - Store key in EEPROM
- `signing_key_load()` - Retrieve key
- `signing_enabled()` - Check if active
- `update_signing_timestamp()` - Sync time from GPS
- `handle_setup_signing()` - Configure signing

---

### 15. GCS_serial_control.cpp (185 lines)
**Purpose:** Serial port passthrough over MAVLink

**Use Cases:**
- Configure GPS via GCS
- Talk to companion computer
- Debug serial devices
- Upload firmware to peripherals

**Protocol:**
- `SERIAL_CONTROL` message
- Lock/unlock port
- Send/receive data
- Multiple ports supported

**Safety:** Locks out MAVLink while passthrough active to prevent conflicts.

---

### 16. GCS_DeviceOp.cpp (126 lines)
**Purpose:** I2C/SPI device operations via MAVLink

**Operations:**
- Read from I2C device
- Write to I2C device
- Read from SPI device
- Write to SPI device

**Use Cases:**
- Configure sensors remotely
- Debug hardware issues
- Test new peripherals

**Messages:**
- `DEVICE_OP_READ` - Read from bus
- `DEVICE_OP_WRITE` - Write to bus
- `DEVICE_OP_READ_REPLY` - Response

**Security:** Should be disabled on production vehicles.

---

### 17. GCS_FTP.cpp (810 lines)
**Purpose:** File Transfer Protocol implementation

**Operations Supported:**
- `ListDirectory` - Browse filesystem
- `OpenFileRO` - Open for reading
- `ReadFile` - Read data
- `CreateFile` - Create new file
- `WriteFile` - Write data
- `RemoveFile` - Delete file
- `CreateDirectory` - Make directory
- `RemoveDirectory` - Delete directory
- `Rename` - Rename file
- `CalcFileCRC32` - Verify file
- `BurstReadFile` - High-speed read

**Session Management:**
- Up to 5 concurrent FTP sessions
- Per-session file handles
- Timeout handling
- Request/response queuing

**Security:**
- Read-only mode supported
- Protected files/directories
- Parameter sets can be uploaded
- Logs can be downloaded

**Key Classes:**
- `GCS_FTP` - Main FTP handler
- `Session` - Per-connection state
- `Transaction` - Request/response packet

---

### 18. MissionItemProtocol.h & .cpp (146 + 435 lines)
**Purpose:** Base class for mission item transfer

**Protocol Flow:**
1. GCS sends `MISSION_COUNT`
2. FC requests items with `MISSION_REQUEST_INT`
3. GCS sends `MISSION_ITEM_INT`
4. Repeat until all items received
5. FC sends `MISSION_ACK`

**Key Methods:**
- `handle_mission_count()` - Start upload
- `handle_mission_item()` - Receive item
- `handle_mission_request_list()` - Start download
- `handle_mission_request_int()` - Send item to GCS
- `handle_mission_clear_all()` - Delete all items
- `handle_mission_write_partial_list()` - Partial update

**Safety Features:**
- Timeout detection (8 seconds)
- Duplicate request handling
- Mission type validation
- MAVLink 2 requirements

**Derived Classes:** Fence, Rally, Waypoints

---

### 19. MissionItemProtocol_Waypoints.cpp (127 lines)
**Purpose:** Waypoint/mission upload/download

**Integration:** Works with `AP_Mission` library

**Key Methods:**
- `get_item()` - Read waypoint from mission
- `append_item()` - Add waypoint to mission
- `replace_item()` - Update existing waypoint
- `truncate()` - Resize mission
- `clear_all_items()` - Delete mission
- `complete()` - Finalize upload

**Special Handling:**
- Validates command types
- Checks mission size limits
- Handles DO commands
- Updates mission state

---

### 20. MissionItemProtocol_Fence.cpp (257 lines)
**Purpose:** Geofence upload/download

**Integration:** Works with `AC_Fence` library

**Fence Types Supported:**
- Polygon inclusion/exclusion zones
- Cylinder zones
- Altitude limits

**Key Features:**
- Temporary storage during upload
- Atomic fence update
- Validation before activation
- Partial updates supported

**Key Methods:**
- `allocate_receive_resources()` - Allocate temp storage
- `free_upload_resources()` - Free temp storage
- `complete()` - Commit fence to storage
- `timeout()` - Abort upload

---

### 21. MissionItemProtocol_Rally.cpp (176 lines)
**Purpose:** Rally point upload/download

**Integration:** Works with `AP_Rally` library

**Rally Point Features:**
- Emergency landing points
- RTL alternate destinations
- Height above ground
- Max distance validation

**Key Methods:**
- `get_item()` - Read rally point
- `append_item()` - Add rally point
- `replace_item()` - Update rally point
- `truncate()` - Resize rally list
- `complete()` - Finalize rally upload

---

## Architecture Overview

### Class Hierarchy

```
GCS (singleton)
├── GCS_MAVLINK (per channel)
│   ├── update_receive()
│   ├── update_send()
│   └── handle_message()
│
├── MAVLink_routing (singleton)
│   └── Routes messages between channels
│
└── MissionItemProtocol objects (3)
    ├── MissionItemProtocol_Waypoints
    ├── MissionItemProtocol_Fence
    └── MissionItemProtocol_Rally
```

### Message Flow

#### Receiving:
1. UART bytes arrive
2. `GCS_MAVLINK::update_receive()` parses
3. `packetReceived()` called
4. `MAVLink_routing::check_and_forward()` routes
5. `handle_message()` processes
6. Specific handler executes

#### Sending:
1. Code calls `send_message(MSG_*)` or `send_text()`
2. Message queued in bucket or deferred queue
3. `update_send()` checks timing
4. `try_send_message()` called
5. Specific send function executes
6. Packet written to UART

### Scheduling System

Messages are organized into **buckets** by stream rate:

```
Bucket 0: 50 Hz messages (attitude, position)
Bucket 1: 10 Hz messages (GPS, battery)
Bucket 2: 4 Hz messages (status, mode)
Bucket 3: 2 Hz messages (misc sensors)
...
Bucket 9: 0.2 Hz messages (autopilot version)
```

Each bucket has an interval. The scheduler round-robins through buckets, sending one message per bucket per cycle.

**Deferred Messages** (heartbeat, parameters, high-latency) bypass buckets for guaranteed delivery.

---

## Integration with EduCopter

### Required Vehicle Implementation

EduCopter must implement these vehicle-specific methods:

```cpp
// In GCS_EduCopter class:
uint32_t custom_mode() const;              // Current flight mode
MAV_TYPE frame_type() const;               // MAV_TYPE_QUADROTOR
uint8_t send_available_mode(uint8_t);     // Mode list

// In GCS_MAVLINK_EduCopter class:
void send_nav_controller_output() const;   // Navigation status
void send_pid_tuning();                    // PID debug
uint8_t base_mode() const;                 // Base mode bits
MAV_STATE vehicle_system_status() const;   // Vehicle state
void handle_message(mavlink_message_t&);   // Custom messages
bool try_send_message(ap_message id);      // Custom sends
```

### Initialization

In `EduCopter::setup()`:

```cpp
gcs().init();              // Initialize GCS singleton
gcs().setup_uarts();       // Configure telemetry ports
```

### Main Loop

In `EduCopter::loop()`:

```cpp
gcs().update_receive();    // Process incoming
gcs().update_send();       // Send queued messages
```

### Sending Messages

```cpp
// Text to GCS
gcs().send_text(MAV_SEVERITY_INFO, "EduCopter initialized");

// Queue message
gcs().send_message(MSG_ATTITUDE);

// Send on specific channel
GCS_MAVLINK *chan = gcs().chan(0);
if (chan) {
    chan->send_message(MSG_HEARTBEAT);
}
```

---

## MAVLink Protocol Details

### Message Structure

All MAVLink 2 messages have:
- Magic byte (0xFD)
- Payload length
- Incompatibility flags
- Compatibility flags
- Sequence number
- System ID
- Component ID
- Message ID (24-bit)
- Payload (0-255 bytes)
- Checksum (CRC-16)
- Optional signature (13 bytes)

### System/Component IDs

EduCopter typically uses:
- System ID: 1 (configurable via SYSID_THISMAV)
- Component ID: 1 (MAV_COMP_ID_AUTOPILOT1)

GCS typically uses:
- System ID: 255 (MAV_COMP_ID_MISSIONPLANNER)
- Component ID: 190 (or 0 for broadcast)

### Channels

ArduPilot supports up to 8 MAVLink channels:
- MAVLINK_COMM_0: Console (USB)
- MAVLINK_COMM_1: Telem1
- MAVLINK_COMM_2: Telem2
- MAVLINK_COMM_3: Telem3 / WiFi
- ... up to COMM_7

Each channel is independent with its own:
- Stream rates
- Routing table
- Signing state
- High/low latency mode

---

## Key Concepts

### 1. Streams vs Messages

**Stream:** A group of related messages sent at the same rate
- Example: `STREAM_POSITION` contains GPS, attitude, local position

**Message:** Individual MAVLink packet
- Example: `GLOBAL_POSITION_INT`

Legacy GCS clients set stream rates. Modern approach sets individual message intervals.

### 2. Deferred Sending

Not all messages are sent immediately. Most are queued and sent when bandwidth available:

- **Immediate:** Commands, ACKs
- **High priority:** Heartbeat, parameters, mission items
- **Scheduled:** Telemetry streams
- **Opportunistic:** Statustext, low-priority data

### 3. Flow Control

Multiple mechanisms prevent overwhelming slow links:

- **Payload space checking:** Before sending, verify UART buffer space
- **Stream slowdown:** Increase intervals on slow links
- **Priority:** Critical messages bypass streams
- **Burst limiting:** Max messages per update cycle

### 4. Routing

Messages can be:
- **Consumed:** Processed by this vehicle
- **Forwarded:** Sent to another system
- **Broadcast:** Sent to all channels

Routing table learns destinations automatically from heartbeats.

---

## Common Message Sequences

### 1. Connection Establishment

```
GCS                          EduCopter
 |                               |
 |--- HEARTBEAT ---------------→|
 |                               |
 |←-- HEARTBEAT -----------------|
 |←-- AUTOPILOT_VERSION ---------|
 |←-- PROTOCOL_VERSION ----------|
 |                               |
 |--- PARAM_REQUEST_LIST ------→|
 |                               |
 |←-- PARAM_VALUE (1/500) -------|
 |←-- PARAM_VALUE (2/500) -------|
 |       ... (498 more) ...      |
```

### 2. Arming

```
GCS                          EduCopter
 |                               |
 |--- COMMAND_INT ------------→ |
 |    (MAV_CMD_COMPONENT_ARM)    |
 |                               |
 |←-- COMMAND_ACK ---------------|
 |    (MAV_RESULT_ACCEPTED)      |
 |                               |
 |←-- HEARTBEAT -----------------|
 |    (armed bit set)            |
```

### 3. Mission Upload

```
GCS                          EduCopter
 |                               |
 |--- MISSION_COUNT ----------→ |
 |    (type=MISSION, count=10)   |
 |                               |
 |←-- MISSION_REQUEST_INT -------|
 |    (seq=0)                    |
 |                               |
 |--- MISSION_ITEM_INT --------→|
 |    (seq=0, CMD_NAV_TAKEOFF)   |
 |                               |
 |←-- MISSION_REQUEST_INT -------|
 |    (seq=1)                    |
 |                               |
 |       ... (8 more) ...        |
 |                               |
 |←-- MISSION_ACK ---------------|
 |    (ACCEPTED)                 |
```

### 4. File Download (FTP)

```
GCS                          EduCopter
 |                               |
 |--- FILE_TRANSFER_PROTOCOL -→ |
 |    (op=ListDirectory)         |
 |                               |
 |←-- FILE_TRANSFER_PROTOCOL ----|
 |    (ACK, directory listing)   |
 |                               |
 |--- FILE_TRANSFER_PROTOCOL -→ |
 |    (op=OpenFileRO)            |
 |                               |
 |←-- FILE_TRANSFER_PROTOCOL ----|
 |    (ACK, session=1)           |
 |                               |
 |--- FILE_TRANSFER_PROTOCOL -→ |
 |    (op=BurstReadFile)         |
 |                               |
 |←-- FILE_TRANSFER_PROTOCOL ----|
 |    (ACK, data chunk 1)        |
 |←-- FILE_TRANSFER_PROTOCOL ----|
 |    (ACK, data chunk 2)        |
 |       ... (more chunks) ...   |
```

---

## Performance Characteristics

### Message Rates

Typical stream rates on 57600 baud link:
- Heartbeat: 1 Hz
- Attitude: 10 Hz
- Position: 3 Hz
- GPS: 2 Hz
- Status: 2 Hz
- RC channels: 2 Hz

### Bandwidth Usage

At 57600 baud (5760 bytes/sec):
- Heartbeat (9 bytes): 0.15% @ 1 Hz
- Attitude (28 bytes): 4.8% @ 10 Hz
- Position (28 bytes): 1.5% @ 3 Hz
- Parameter dump (500 params): ~15 seconds

High-speed links (921600 baud) support faster rates.

### CPU Usage

MAVLink processing typically uses:
- 1-3% CPU on STM32H7 @ 480 MHz
- Most time in send functions (formatting)
- Receive is fast (parsing pre-optimized)

### Memory Usage

Static allocation:
- GCS_MAVLINK object: ~1 KB each
- Routing table: ~500 bytes
- Message buffers: ~300 bytes per channel
- Statustext queue: ~2 KB
- Parameter queue: ~500 bytes

Dynamic allocation:
- Mission upload: Variable (mission size)
- Fence upload: Variable (fence points)
- FTP sessions: ~1 KB per session

---

## Debugging and Development

### Enable MAVLink Debug Output

In `GCS.h`, set:
```cpp
#define GCS_DEBUG_SEND_MESSAGE_TIMINGS 1
```

Logs:
- Longest send time
- Messages dropped
- Buffer overruns

### Common Issues

#### 1. No Heartbeat
**Symptom:** GCS shows "No Heartbeat"
**Causes:**
- UART not initialized
- Wrong baud rate
- TX pin not connected
- System ID mismatch

**Debug:**
```cpp
gcs().send_text(MAV_SEVERITY_INFO, "GCS init");
hal.console->printf("Chan active: %d\n", gcs().chan(0)->is_active());
```

#### 2. Parameters Not Loading
**Symptom:** Parameter list incomplete
**Causes:**
- EEPROM corruption
- Insufficient bandwidth
- Parameter request timeout

**Debug:**
- Check `_queued_parameter_index`
- Verify `queued_param_send()` called
- Increase parameter stream rate

#### 3. Mission Upload Fails
**Symptom:** MISSION_ACK returns error
**Causes:**
- Invalid command type
- Mission too large
- MAVLink 1 used (requires MAVLink 2)
- Timeout

**Debug:**
- Enable `MISSION_REQUEST_WARNING`
- Check `MissionItemProtocol::receiving` flag
- Verify mission storage available

#### 4. High CPU Usage
**Symptom:** GCS uses excessive CPU
**Causes:**
- Stream rates too high
- Slow UART (buffer fills)
- Inefficient custom messages

**Debug:**
- Reduce stream rates
- Increase UART baud rate
- Profile `try_send_message()` times

---

## Future Enhancements

Potential improvements for EduCopter:

1. **Custom Messages**
   - Add EduCopter-specific telemetry
   - Educational debugging data
   - Student performance metrics

2. **Mission Extensions**
   - Educational mission commands
   - Autonomous challenges
   - Competition modes

3. **Parameter Groups**
   - Tuning parameter sets
   - Quick configuration profiles
   - Safety presets

4. **Logging Integration**
   - Stream logs over MAVLink
   - Real-time log analysis
   - Graph data in GCS

5. **Security**
   - Enable message signing
   - Command authentication
   - Secure firmware upload

---

## Compliance and Standards

### MAVLink Protocol Version
- **Protocol:** MAVLink 2.0
- **Dialect:** Common + ArduPilotMega
- **Specification:** https://mavlink.io/en/

### Message IDs
Uses official MAVLink message IDs from:
- Common: 0-999
- ArduPilotMega: 10000-10999

### Compliance
Tested with:
- Mission Planner
- QGroundControl
- MAVProxy
- APM Planner 2

---

## References

### Documentation
- MAVLink Protocol: https://mavlink.io
- ArduPilot MAVLink: https://ardupilot.org/dev/docs/mavlink-basics.html
- Message Definitions: https://github.com/ArduPilot/mavlink

### Related Libraries
- `AP_Param` - Parameter storage
- `AP_Mission` - Mission management
- `AC_Fence` - Geofence management
- `AP_Rally` - Rally point management
- `AP_Logger` - Data logging

### Tools
- MAVExplorer - Log analysis
- MAVProxy - Command line GCS
- pymavlink - Python library
- MAVSDK - C++ library

---

## Revision History

| Date       | Version | Changes                                    |
|------------|---------|-------------------------------------------|
| 2025-10-28 | 1.0     | Initial import from ArduPilot GCS_MAVLink |
|            |         | All 27 files copied for EduCopter         |
|            |         | Comprehensive documentation created        |

---

## License

All files in this directory are licensed under **GNU General Public License v3.0** as part of the ArduPilot project.

Copyright (C) ArduPilot Dev Team
Copyright (C) 2025 EduCopter Project

---

**End of Documentation**

*For questions or contributions, please refer to the EduCopter project repository.*
