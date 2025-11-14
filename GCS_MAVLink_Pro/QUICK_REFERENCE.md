# GCS MAVLink Quick Reference Guide

## File Locations Summary

```
libraries/GCS_MAVLink/
├── GCS_MAVLink.h          - Low-level protocol headers
├── GCS_MAVLink.cpp        - Communication primitives (195 lines)
├── GCS.h                  - Base classes (1429 lines)
├── GCS.cpp                - Global manager implementation (598 lines)
└── GCS_Common.cpp         - Shared handlers (83,660 tokens)

ArduCopter/
├── GCS_Copter.h           - Copter manager class (47 lines)
├── GCS_Copter.cpp         - Copter manager implementation (120 lines)
├── GCS_MAVLink_Copter.h   - Copter channel class (131 lines)
└── GCS_MAVLink_Copter.cpp - Copter channel implementation (varies)
```

## Class Quick Reference

### GCS (Global Manager)
```cpp
Location: libraries/GCS_MAVLink/GCS.h (lines 1089-1376)
Purpose: Manages all MAVLink channels, global messaging

Key Methods:
- void init()                              // Initialize GCS system
- void setup_uarts()                       // Setup all telemetry ports
- void update_send()                       // Send on all channels
- void update_receive()                    // Receive on all channels
- void send_text(severity, fmt, ...)       // Send to all channels
- GCS_MAVLINK *chan(uint8_t ofs)          // Get channel by index
- uint8_t num_gcs()                        // Number of channels

Pure Virtual (must implement):
- uint32_t custom_mode()                   // Current flight mode
- MAV_TYPE frame_type()                    // Vehicle type
- GCS_MAVLINK *new_gcs_mavlink_backend()  // Channel factory
```

### GCS_MAVLINK (Base Channel)
```cpp
Location: libraries/GCS_MAVLink/GCS.h (lines 174-1085)
Purpose: Single MAVLink communication channel

Key Methods:
- bool init(uint8_t instance)              // Initialize channel
- void update_send()                       // Send queued messages
- void update_receive(uint32_t max_time_us)// Parse incoming bytes
- void send_message(ap_message id)         // Queue message for sending
- void send_text(severity, fmt, ...)       // Send text on this channel
- uint16_t txspace()                       // Available TX buffer space
- bool is_active()                         // Has heartbeat been received?
- bool is_streaming()                      // Actively sending streams?
- mavlink_channel_t get_chan()             // Get channel ID

Pure Virtual (must implement):
- void send_nav_controller_output()        // NAV_CONTROLLER_OUTPUT msg
- void send_pid_tuning()                   // PID_TUNING msg
- uint8_t base_mode()                      // MAVLink base mode flags
- MAV_STATE vehicle_system_status()        // System state

Virtual (can override):
- bool try_send_message(ap_message id)     // Send specific message
- void handle_message(mavlink_message_t)   // Handle incoming message
- MAV_RESULT handle_command_int_packet()   // Handle commands
- float vfr_hud_airspeed()                 // VFR_HUD airspeed
- int16_t vfr_hud_throttle()               // VFR_HUD throttle
```

## Common Tasks

### Task 1: Send Text Message to GCS

```cpp
// To all channels
gcs().send_text(MAV_SEVERITY_INFO, "Hello from %s", "MyModule");

// To specific channel
GCS_MAVLINK *chan0 = gcs().chan(0);
if (chan0) {
    chan0->send_text(MAV_SEVERITY_WARNING, "Channel 0 warning");
}

// Severity levels:
// MAV_SEVERITY_EMERGENCY   - System unusable
// MAV_SEVERITY_ALERT       - Action must be taken
// MAV_SEVERITY_CRITICAL    - Critical conditions
// MAV_SEVERITY_ERROR       - Error conditions
// MAV_SEVERITY_WARNING     - Warning conditions
// MAV_SEVERITY_NOTICE      - Normal but significant
// MAV_SEVERITY_INFO        - Informational
// MAV_SEVERITY_DEBUG       - Debug messages
```

### Task 2: Send Named Value (for debugging)

```cpp
// Sends NAMED_VALUE_FLOAT message
gcs().send_named_float("my_debug_value", 123.45f);

// Sends NAMED_VALUE_STRING message (custom, see GCS.cpp:246)
gcs().send_named_string("my_debug_string", "test");

// These appear in GCS as real-time data
```

### Task 3: Check if GCS is Connected

```cpp
GCS_MAVLINK *chan = gcs().chan(0);  // Check primary channel

if (chan && chan->is_active()) {
    // GCS is connected (received heartbeat in last 2.5 seconds)
    uint32_t last_hb_ms = chan->get_last_heartbeat_time();
    uint32_t time_since_hb = AP_HAL::millis() - last_hb_ms;

    if (time_since_hb > 5000) {
        // Connection may be degraded
    }
}
```

### Task 4: Handle Custom MAVLink Message

```cpp
// In GCS_MAVLink_Copter.cpp

void GCS_MAVLINK_Copter::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_YOUR_MESSAGE: {
        mavlink_your_message_t packet;
        mavlink_msg_your_message_decode(&msg, &packet);

        // Process the message
        float value = packet.field1;
        uint32_t flags = packet.field2;

        // Do something with it
        copter.handle_gcs_custom_message(value, flags);

        break;
    }

    default:
        // ALWAYS call base class for unhandled messages
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}
```

### Task 5: Handle Custom MAVLink Command

```cpp
// In GCS_MAVLink_Copter.cpp

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {

    case MAV_CMD_YOUR_CUSTOM_COMMAND: {
        // Extract parameters
        float param1 = packet.param1;
        float param2 = packet.param2;
        int32_t param5 = packet.x;  // latitude (if location-based)
        int32_t param6 = packet.y;  // longitude
        float param7 = packet.z;    // altitude

        // Validate
        if (param1 < 0) {
            return MAV_RESULT_DENIED;
        }

        // Execute
        bool success = copter.do_custom_action(param1, param2);

        // Return result (will send COMMAND_ACK automatically)
        return success ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
    }

    default:
        // ALWAYS call base class for unhandled commands
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}
```

### Task 6: Send Custom MAVLink Message Periodically

```cpp
// Step 1: Add message ID to ap_message enum
// In libraries/GCS_MAVLink/ap_message.h, add:
enum ap_message : uint8_t {
    // ... existing messages ...
    MSG_YOUR_CUSTOM,
    MSG_LAST
};

// Step 2: Implement sender
// In GCS_MAVLink_Copter.cpp

bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id)
{
    switch(id) {

    case MSG_YOUR_CUSTOM:
        CHECK_PAYLOAD_SIZE(YOUR_CUSTOM);  // YOUR_CUSTOM is MAVLink msg name
        send_your_custom();
        break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

void GCS_MAVLINK_Copter::send_your_custom()
{
    // Get data from vehicle
    float value1 = copter.get_some_value();
    uint16_t value2 = copter.get_another_value();

    // Send MAVLink message
    mavlink_msg_your_custom_send(
        chan,
        AP_HAL::millis(),  // timestamp
        value1,
        value2
    );
}

// Step 3: Set message interval via MAVLink command:
// MAV_CMD_SET_MESSAGE_INTERVAL
// param1 = MAVLINK_MSG_ID_YOUR_CUSTOM
// param2 = interval in microseconds (e.g., 100000 = 10 Hz)
```

### Task 7: Create Mode Change Handler

```cpp
// Mode changes come via SET_MODE message or MAV_CMD_DO_SET_MODE command

// The base class already handles this, but you can customize:

void GCS_MAVLink_Copter::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SET_MODE: {
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(&msg, &packet);

        // Base class will call vehicle set_mode()
        // But you can add pre/post processing here

        // Log the mode change request
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                     "Mode change requested: %u", packet.custom_mode);

        // Let base class handle it
        GCS_MAVLINK::handle_message(msg);
        break;
    }

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}
```

### Task 8: Add Parameter Validation

```cpp
// Parameters are handled automatically, but you can add validation:

void GCS_MAVLINK_Copter::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t packet;
        mavlink_msg_param_set_decode(&msg, &packet);

        // Check if we want to allow this parameter change
        if (strncmp(packet.param_id, "CRITICAL_PARAM", 16) == 0) {
            if (!copter.allow_param_change()) {
                send_text(MAV_SEVERITY_WARNING,
                         "Parameter change denied: %s", packet.param_id);
                return;  // Don't call base class, ignore the request
            }
        }

        // Let base class handle it normally
        GCS_MAVLINK::handle_message(msg);
        break;
    }

    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}
```

## Message Streaming Configuration

### Stream Types and Default Contents

```cpp
STREAM_RAW_SENSORS (SR*_RAW_SENS):
├── RAW_IMU
├── SCALED_IMU2
├── SCALED_IMU3
├── SCALED_PRESSURE
├── SCALED_PRESSURE2
└── SCALED_PRESSURE3

STREAM_EXTENDED_STATUS (SR*_EXT_STAT):
├── SYS_STATUS
├── POWER_STATUS
├── MCU_STATUS
├── MEMINFO
├── MISSION_CURRENT
├── GPS_RAW_INT
├── GPS_RTK
├── GPS2_RTK
├── NAV_CONTROLLER_OUTPUT
├── FENCE_STATUS
└── EKF_STATUS_REPORT

STREAM_RC_CHANNELS (SR*_RC_CHAN):
├── SERVO_OUTPUT_RAW
└── RC_CHANNELS

STREAM_RAW_CONTROLLER (SR*_RAW_CTRL):
└── SERVO_OUTPUT_RAW

STREAM_POSITION (SR*_POSITION):
├── GLOBAL_POSITION_INT
├── LOCAL_POSITION_NED
└── (GPS already in EXTENDED_STATUS)

STREAM_EXTRA1 (SR*_EXTRA1):
├── ATTITUDE
├── ATTITUDE_QUATERNION
├── ATTITUDE_TARGET
├── PID_TUNING
└── LOCAL_POSITION_NED

STREAM_EXTRA2 (SR*_EXTRA2):
├── VFR_HUD
└── SIMSTATE (if SITL)

STREAM_EXTRA3 (SR*_EXTRA3):
├── AHRS
├── HWSTATUS
├── SYSTEM_TIME
├── WIND
├── RANGEFINDER
├── DISTANCE_SENSOR
├── TERRAIN_REPORT
├── BATTERY_STATUS
├── GIMBAL_DEVICE_ATTITUDE_STATUS
└── OPTICAL_FLOW

STREAM_PARAMS (SR*_PARAMS):
└── PARAM_VALUE (queued parameters)

STREAM_ADSB (SR*_ADSB):
└── ADSB_VEHICLE
```

### Setting Stream Rates

#### Via Parameters (persistent):
```
MAV1_RATE = 10        # Master rate multiplier (Hz)
SR1_EXTRA1 = 10       # ATTITUDE at 10 Hz
SR1_EXTRA2 = 10       # VFR_HUD at 10 Hz
SR1_POSITION = 3      # GPS at 3 Hz
SR1_RAW_SENS = 2      # IMU at 2 Hz
SR1_EXT_STAT = 2      # SYS_STATUS at 2 Hz
SR1_RC_CHAN = 2       # RC inputs at 2 Hz
SR1_RAW_CTRL = 0      # Servo outputs disabled
SR1_PARAMS = 10       # Parameters at 10 Hz (when queued)
SR1_ADSB = 5          # ADS-B at 5 Hz

# Channels: 1=Primary telemetry, 2=Secondary, etc.
```

#### Via MAVLink (runtime):
```cpp
// Use REQUEST_DATA_STREAM message (deprecated but still works)
// Or use MAV_CMD_SET_MESSAGE_INTERVAL for individual messages

// Example: Set ATTITUDE to 20 Hz
mavlink_command_int_t cmd;
cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
cmd.param1 = MAVLINK_MSG_ID_ATTITUDE;
cmd.param2 = 50000;  // 50ms = 20 Hz (in microseconds)
```

## Important Macros

### Payload Size Checking

```cpp
// In message sender functions:

CHECK_PAYLOAD_SIZE(MESSAGE_NAME)
// Expands to:
// if (!check_payload_size(MAVLINK_MSG_ID_MESSAGE_NAME_LEN)) return false

// Example:
void GCS_MAVLINK::send_heartbeat() {
    CHECK_PAYLOAD_SIZE(HEARTBEAT);
    mavlink_msg_heartbeat_send(chan, ...);
}
```

### Checking TX Space

```cpp
// Check if message will fit in buffer:
if (!HAVE_PAYLOAD_SPACE(chan, MESSAGE_NAME)) {
    return;  // Not enough space, try again later
}

// Example:
if (!HAVE_PAYLOAD_SPACE(chan, GLOBAL_POSITION_INT)) {
    return false;
}
mavlink_msg_global_position_int_send(chan, ...);
```

## Timing and Scheduling

### Message Send Timing

The GCS system sends messages based on:

1. **Stream intervals**: Configured via SR parameters
2. **Message intervals**: Individual message rates via SET_MESSAGE_INTERVAL
3. **Event-based**: Immediate sending (COMMAND_ACK, STATUSTEXT, etc.)
4. **Deferred messages**: Special high-priority (HEARTBEAT, PARAM_VALUE)

### Loop Integration

```cpp
// Typical vehicle main loop (50 Hz):

void Copter::fast_loop() {
    // Flight control - highest priority
    read_AHRS();
    run_nav_updates();
    update_flight_mode();
    motors_output();
}

void Copter::main_loop() {
    // Called at 50 Hz or similar

    // Update GCS after control loop
    gcs().update_receive();  // Parse incoming (max 1ms)
    gcs().update_send();     // Send outgoing messages

    // GCS respects min_loop_time_remaining_for_message_send_us()
    // Won't send if insufficient time remaining in loop
}
```

### Time Budget

```cpp
// GCS checks time remaining before sending:
// From GCS.cpp:

bool GCS::out_of_time() const {
    if (hal.scheduler->in_delay_callback()) {
        return false;  // Always allow during delay
    }

    if (min_loop_time_remaining_for_message_send_us() <=
        AP::scheduler().time_available_usec()) {
        return false;  // Enough time
    }

    return true;  // Not enough time, skip sending
}
```

## Debugging

### Enable Debug Output

```cpp
// In GCS.h, change:
#define GCS_DEBUG_SEND_MESSAGE_TIMINGS 1

// This enables tracking of:
struct try_send_message_stats {
    uint32_t longest_time_us;        // Longest send time
    ap_message longest_id;           // Which message took longest
    uint32_t no_space_for_message;   // Count of buffer full events
    uint32_t behind;                 // Stream timing behind
    uint32_t out_of_time;            // Skipped due to time
    // ... more stats
};
```

### Check Channel Status

```cpp
void check_channel_status() {
    for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
        GCS_MAVLINK *link = gcs().chan(i);
        if (link == nullptr) continue;

        gcs().send_text(MAV_SEVERITY_INFO,
            "Chan %u: active=%u stream=%u txspace=%u",
            i,
            link->is_active(),
            link->is_streaming(),
            link->txspace()
        );
    }
}
```

### Monitor Message Rates

```cpp
// Access stream configuration:
GCS_MAVLINK *link = gcs().chan(0);

for (uint8_t i = 0; i < GCS_MAVLINK::NUM_STREAMS; i++) {
    uint16_t rate = link->streamRates[i];
    uint16_t interval = link->get_interval_for_stream((GCS_MAVLINK::streams)i);

    gcs().send_text(MAV_SEVERITY_INFO,
        "Stream %u: rate=%u interval=%u ms",
        i, rate, interval);
}
```

## Common Gotchas

### 1. Always Call Base Class

```cpp
// WRONG:
void handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MY_MESSAGE:
        handle_my_message(msg);
        break;
    }
    // Missing base class call!
}

// CORRECT:
void handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MY_MESSAGE:
        handle_my_message(msg);
        break;
    default:
        GCS_MAVLINK::handle_message(msg);  // <-- Important!
        break;
    }
}
```

### 2. Check for nullptr

```cpp
// WRONG:
void my_function() {
    gcs().chan(0)->send_text(MAV_SEVERITY_INFO, "Test");  // May crash!
}

// CORRECT:
void my_function() {
    GCS_MAVLINK *chan0 = gcs().chan(0);
    if (chan0 != nullptr) {
        chan0->send_text(MAV_SEVERITY_INFO, "Test");
    }
}
```

### 3. Use Correct Severity

```cpp
// WRONG - Everything is INFO:
send_text(MAV_SEVERITY_INFO, "Critical failure!");

// CORRECT - Use appropriate severity:
send_text(MAV_SEVERITY_CRITICAL, "Critical failure!");
send_text(MAV_SEVERITY_WARNING, "Sensor degraded");
send_text(MAV_SEVERITY_INFO, "Mode changed to AUTO");
send_text(MAV_SEVERITY_DEBUG, "Debug: value=%f", val);
```

### 4. Don't Block in Handlers

```cpp
// WRONG:
void handle_message(const mavlink_message_t &msg) {
    hal.scheduler->delay(1000);  // NEVER do this!
    // Blocks the entire system
}

// CORRECT:
void handle_message(const mavlink_message_t &msg) {
    // Set a flag, state machine will handle it
    pending_action = true;
    pending_action_time = AP_HAL::millis();
}

void slow_loop() {
    if (pending_action && AP_HAL::millis() - pending_action_time > 1000) {
        // Do slow operation here
        pending_action = false;
    }
}
```

### 5. Thread Safety with send_text

```cpp
// send_text() is thread-safe (uses semaphore)
// But don't format large strings on stack from ISR

// SAFER in ISR:
gcs().send_text(MAV_SEVERITY_CRITICAL, "Fault!");

// AVOID in ISR:
char buf[256];
snprintf(buf, sizeof(buf), "Fault at time %lu with code %d", time, code);
gcs().send_text(MAV_SEVERITY_CRITICAL, "%s", buf);  // Large stack usage
```

## Performance Tips

### 1. Batch Related Operations

```cpp
// LESS EFFICIENT:
for (int i = 0; i < 10; i++) {
    gcs().send_named_float("value", i);
    hal.scheduler->delay(10);
}

// MORE EFFICIENT:
// Use a single custom message with array
```

### 2. Respect Stream Rates

```cpp
// Don't manually send stream messages
// Let the stream system handle it

// WRONG:
void fast_loop() {
    gcs().chan(0)->send_attitude();  // Every loop!
}

// CORRECT:
// Set SR1_EXTRA1 parameter, stream system sends it automatically
```

### 3. Use Message Intervals

```cpp
// For custom messages, use the interval system
// instead of manually tracking time

// Add to try_send_message(), then:
// MAV_CMD_SET_MESSAGE_INTERVAL to configure rate
```

## Quick Setup Checklist

For a new vehicle type:

- [ ] Create GCS_MyVehicle class inheriting from GCS
- [ ] Implement custom_mode(), frame_type(), frame_string()
- [ ] Implement update_vehicle_sensor_status_flags()
- [ ] Create GCS_MAVLINK_MyVehicle class inheriting from GCS_MAVLINK
- [ ] Implement required pure virtuals (base_mode, system_status, nav_output, pid_tuning)
- [ ] Override VFR_HUD methods (airspeed, throttle, alt)
- [ ] Add vehicle-specific message handlers
- [ ] Add vehicle-specific command handlers
- [ ] Instantiate GCS in vehicle class
- [ ] Call gcs().init(), setup_console(), setup_uarts() in setup
- [ ] Call gcs().update_receive() and update_send() in main loop
- [ ] Add to build system (wscript)
- [ ] Test with Mission Planner or QGroundControl

## Resources

- MAVLink Documentation: https://mavlink.io
- ArduPilot Dev Wiki: https://ardupilot.org/dev/
- MAVLink Message Definitions: `modules/mavlink/message_definitions/v1.0/common.xml`
- Example Vehicles: ArduCopter, ArduPlane, ArduRover, ArduSub
