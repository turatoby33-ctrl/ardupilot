# Version 1: Standalone Minimal GCS with HEARTBEAT

This is a **completely standalone** implementation of a minimal GCS (Ground Control Station) MAVLink system. It demonstrates the fundamental concepts without requiring the full ArduPilot infrastructure.

## Purpose

This version is designed to:
- **Teach** the basics of GCS/MAVLink communication
- **Demonstrate** HEARTBEAT message send/receive
- **Show** the core patterns used throughout ArduPilot
- **Run independently** without dependencies on ArduPilot libraries

## What's Included

### Files

1. **minimal_GCS.h** - Header file with class definitions
   - `GCS_MAVLINK` - Channel class (handles one MAVLink connection)
   - `GCS` - Global manager class (manages all channels)
   - MAVLink type definitions (simplified)
   - Helper function declarations

2. **minimal_GCS.cpp** - Basic GCS infrastructure implementation
   - Channel initialization
   - Message sending framework
   - Update loops (send/receive)
   - Connection monitoring
   - Timing functions

3. **minimal_GCS_Common.cpp** - HEARTBEAT-specific functionality
   - `send_heartbeat()` - Creates and sends HEARTBEAT message
   - `handle_heartbeat()` - Receives and parses HEARTBEAT message
   - Detailed comments explaining each field
   - Connection status updates

4. **test_heartbeat.cpp** - Test program demonstrating usage
   - Initializes GCS system
   - Creates simulated GCS
   - Runs bidirectional HEARTBEAT exchange
   - Monitors connection status
   - Provides detailed output

5. **Makefile** - Build system
   - Compiles all files
   - Links executable
   - Provides clean target

## Quick Start

### Compile

```bash
make
```

Or manually:
```bash
g++ -o test_heartbeat test_heartbeat.cpp minimal_GCS.cpp minimal_GCS_Common.cpp -std=c++11
```

### Run

```bash
./test_heartbeat
```

Or:
```bash
make run
```

### Clean

```bash
make clean
```

## What the Test Program Does

### Sequence

1. **Initialize GCS System**
   - Creates GCS singleton
   - Initializes channel 0
   - Sets up system/component IDs

2. **Create Simulated GCS**
   - Acts as a ground control station
   - Sends HEARTBEAT messages at 1 Hz

3. **Run Communication Loop** (10 seconds)
   - Vehicle sends HEARTBEAT every 1 second
   - GCS sends HEARTBEAT every 1 second
   - Monitor and display connection status
   - Parse and display message details

4. **Display Summary**
   - Final connection status
   - Statistics and timing info
   - What was demonstrated

### Expected Output

```
╔═══════════════════════════════════════════════════════╗
║   Minimal GCS HEARTBEAT Test Program (Version 1)     ║
║   Standalone Implementation                           ║
╚═══════════════════════════════════════════════════════╝

>>> STEP 1: Initialize GCS System
[timestamp] GCS: Initializing
[timestamp] GCS_MAVLINK: Channel 0 initialized
[timestamp] GCS: Initialized with 1 channel(s)

>>> STEP 2: Get MAVLink Channel
Channel 0 ready

>>> STEP 3: Create Simulated GCS
Simulated GCS created

>>> STEP 4: Running Test Loop (10 seconds)
Will demonstrate:
  - Vehicle sending HEARTBEAT every 1 second
  - GCS sending HEARTBEAT every 1 second
  - Connection status monitoring

=== SENDING HEARTBEAT ===
  Type: 0
  Autopilot: ArduPilot (3)
  Base Mode: 0x51
  Custom Mode: 0
  System Status: 3
  >>> HEARTBEAT SENT <<<

╔═══════════════════════════════════════╗
║  SIMULATED GCS SENDING HEARTBEAT      ║
╚═══════════════════════════════════════╝

=== HEARTBEAT RECEIVED ===
  From: sysid=255 compid=1
  Type: 0 (0=Generic, 2=Quad, 4=Heli, 6+=GCS)
  Autopilot: 0 (0=Generic, 3=ArduPilot)
  Base Mode: 0x00
    - Custom Mode Enabled: No
    - Test Enabled: No
    - Auto Enabled: No
    - Guided Enabled: No
    - Stabilize Enabled: No
    - Manual Input Enabled: No
    - Safety Armed: No
  Custom Mode: 0
  System Status: 4 (0=Uninit, 1=Boot, 3=Standby, 4=Active, 5=Critical)
  MAVLink Version: 3
  >>> This is a GROUND CONTROL STATION <<<
*** GCS CONNECTED FOR THE FIRST TIME ***

┌─────────────────────────────────────┐
│  CONNECTION STATUS                  │
├─────────────────────────────────────┤
│  Channel Active: YES ✓
│  Last GCS Heartbeat: 123 ms ago
└─────────────────────────────────────┘

[... continues for 10 seconds ...]

╔═══════════════════════════════════════╗
║  TEST COMPLETE                        ║
╚═══════════════════════════════════════╝

Summary:
  - Duration: 10 seconds
  - Final Connection Status: ACTIVE ✓
  - Time since last GCS heartbeat: 456 ms

What we demonstrated:
  ✓ GCS initialization
  ✓ Channel creation and setup
  ✓ Sending HEARTBEAT messages (vehicle → GCS)
  ✓ Receiving HEARTBEAT messages (GCS → vehicle)
  ✓ Parsing HEARTBEAT payload
  ✓ Connection status monitoring
  ✓ Message timing and intervals
```

## Code Structure

### GCS_MAVLINK Class (Channel)

```cpp
class GCS_MAVLINK {
public:
    // Initialization
    bool init(uint8_t instance);

    // Update loops
    void update_receive();  // Parse incoming messages
    void update_send();     // Send periodic messages

    // HEARTBEAT specific
    void send_heartbeat();
    void handle_heartbeat(const mavlink_message_t &msg);

    // Status
    bool is_active() const;
    uint32_t get_last_heartbeat_time() const;

protected:
    // Virtual functions for customization
    virtual uint8_t base_mode() const;
    virtual MAV_STATE system_status() const;
    virtual uint32_t custom_mode() const;
    virtual MAV_TYPE frame_type() const;
};
```

### GCS Class (Manager)

```cpp
class GCS {
public:
    void init();                      // Initialize system
    GCS_MAVLINK* chan(uint8_t ofs);  // Get channel by index
    void update_receive();           // Update all channels (receive)
    void update_send();              // Update all channels (send)
};
```

### Message Structure

```cpp
typedef struct {
    uint8_t msgid;         // Message ID (0 = HEARTBEAT)
    uint8_t sysid;         // System ID (who sent it)
    uint8_t compid;        // Component ID
    uint8_t len;           // Payload length
    uint8_t seq;           // Sequence number
    uint8_t payload[255];  // Message data
} mavlink_message_t;
```

### HEARTBEAT Payload

```cpp
typedef struct {
    uint32_t custom_mode;     // Vehicle-specific mode
    uint8_t type;             // MAV_TYPE (quad, plane, GCS, etc.)
    uint8_t autopilot;        // MAV_AUTOPILOT (ArduPilot, PX4, etc.)
    uint8_t base_mode;        // Base mode flags (armed, enabled, etc.)
    uint8_t system_status;    // MAV_STATE (boot, active, critical, etc.)
    uint8_t mavlink_version;  // MAVLink version (always 3)
} mavlink_heartbeat_t;
```

## Understanding HEARTBEAT

### Purpose

The HEARTBEAT message (ID 0) is the most fundamental MAVLink message:

1. **Keep-Alive** - Indicates system is operational
2. **Connection Monitoring** - Both sides track when last HEARTBEAT received
3. **State Information** - Armed status, flight mode, health
4. **System Identification** - Vehicle type, autopilot type
5. **Failsafe Trigger** - Connection lost if >2.5 seconds without HEARTBEAT

### Fields Explained

#### type (MAV_TYPE)
- `0` = Generic (usually GCS)
- `2` = Quadrotor
- `4` = Helicopter
- `10` = Ground rover
- `6+` = Various GCS types

#### autopilot (MAV_AUTOPILOT)
- `0` = Generic
- `3` = ArduPilot
- `12` = PX4

#### base_mode (Bitfield)
```
Bit 0: Custom mode enabled
Bit 1: Test enabled
Bit 2: Auto enabled
Bit 3: Guided enabled
Bit 4: Stabilize enabled
Bit 5: HIL (hardware-in-loop) enabled
Bit 6: Manual input enabled
Bit 7: Safety armed
```

#### system_status (MAV_STATE)
- `0` = Uninitialized
- `1` = Boot
- `2` = Calibrating
- `3` = Standby
- `4` = Active
- `5` = Critical
- `6` = Emergency
- `7` = Poweroff

#### custom_mode
Vehicle-specific mode number. For example, Copter:
- `0` = Stabilize
- `1` = Acro
- `2` = Alt Hold
- `3` = Auto
- etc.

### Timing Requirements

- **Send Rate**: Minimum 1 Hz (once per second)
- **Typical Rate**: 1 Hz is standard
- **Connection Timeout**: 2.5 seconds without HEARTBEAT
- **Recommended**: Send every 1000ms exactly

## Key Concepts Demonstrated

### 1. Message Sending Pattern

```cpp
void send_heartbeat() {
    // 1. Create message payload
    mavlink_heartbeat_t heartbeat;
    heartbeat.type = frame_type();
    heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    // ... fill other fields ...

    // 2. Pack into MAVLink message
    mavlink_message_t msg;
    msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    msg.sysid = _system_id;
    msg.compid = _component_id;
    msg.len = sizeof(heartbeat);
    memcpy(msg.payload, &heartbeat, sizeof(heartbeat));

    // 3. Send it
    send_message(msg);
}
```

This pattern applies to **all** MAVLink messages!

### 2. Message Receiving Pattern

```cpp
void handle_heartbeat(const mavlink_message_t &msg) {
    // 1. Decode payload
    mavlink_heartbeat_t heartbeat;
    memcpy(&heartbeat, msg.payload, sizeof(heartbeat));

    // 2. Process the data
    print_message("Type: %u", heartbeat.type);
    print_message("Status: %u", heartbeat.system_status);
    // ... process other fields ...

    // 3. Update state
    _last_gcs_heartbeat_ms = millis();

    // 4. Take action
    if (is_gcs) {
        // Handle GCS connection
    }
}
```

This pattern applies to **all** MAVLink messages!

### 3. Periodic Sending

```cpp
void update_send() {
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now = millis();

    // Check if it's time to send
    if (now - last_heartbeat_ms >= 1000) {
        send_heartbeat();
        last_heartbeat_ms = now;
    }
}
```

This pattern is used for all periodic messages.

### 4. Connection Monitoring

```cpp
bool is_active() const {
    uint32_t time_since_heartbeat = millis() - _last_gcs_heartbeat_ms;
    return time_since_heartbeat < 2500;  // 2.5 seconds
}
```

Used to detect lost connections.

## Differences from Real ArduPilot

This standalone version simplifies:

1. **No UART/Serial Communication** - Uses simulated messages in memory
2. **No MAVLink Library** - Simplified message structures and parsing
3. **Single Channel** - Real system supports 5-8 channels
4. **No Streams** - Real system has configurable message streams
5. **No Parameters** - Real system has parameter protocol
6. **No Missions** - Real system has mission protocol
7. **No Commands** - Real system handles 200+ commands
8. **Simplified Parsing** - Real system has full MAVLink parser

### What's the Same

1. **Class Structure** - Same GCS/GCS_MAVLINK pattern
2. **Message Pattern** - Same send/receive/handle pattern
3. **HEARTBEAT Logic** - Identical timing and requirements
4. **Connection Monitoring** - Same 2.5 second timeout
5. **Virtual Functions** - Same customization approach

## Next Steps

### Add More Messages

Follow the HEARTBEAT pattern to add:

1. **SYS_STATUS** - System health and sensors
2. **ATTITUDE** - Roll, pitch, yaw
3. **GLOBAL_POSITION_INT** - GPS position
4. **VFR_HUD** - Airspeed, altitude, heading

Example template:
```cpp
void send_sys_status() {
    mavlink_sys_status_t status;
    // Fill fields...

    mavlink_message_t msg;
    msg.msgid = MAVLINK_MSG_ID_SYS_STATUS;
    // Pack and send...
}

void handle_sys_status(const mavlink_message_t &msg) {
    mavlink_sys_status_t status;
    memcpy(&status, msg.payload, sizeof(status));
    // Process...
}
```

### Add Real Communication

Replace simulated messages with:

1. **Serial Port** - Read/write to `/dev/ttyUSB0` or similar
2. **Network Socket** - UDP or TCP communication
3. **File** - Read/write to file for logging/playback

### Add Message Routing

```cpp
void handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat(msg);
        break;
    case MAVLINK_MSG_ID_SYS_STATUS:
        handle_sys_status(msg);
        break
    // ... more messages ...
    }
}
```

### Add Configuration

```cpp
// In GCS_MAVLINK constructor
void set_system_id(uint8_t sysid) { _system_id = sysid; }
void set_heartbeat_interval(uint32_t ms) { _heartbeat_interval_ms = ms; }
void enable_message(uint8_t msgid, uint32_t interval_ms);
```

## Comparison to Version 2

**Version 1 (This)**:
- ✓ Standalone, runs independently
- ✓ Simplified, easier to understand
- ✓ Self-contained, no external dependencies
- ✗ Not integrated with ArduPilot
- ✗ Simulated communication only
- ✗ Single channel only

**Version 2 (Next)**:
- ✓ Fully integrated with ArduPilot
- ✓ Uses real GCS.h, GCS_MAVLink.h/cpp
- ✓ Real UART/serial communication
- ✓ Multiple channels
- ✗ More complex
- ✗ Requires ArduPilot build environment

## Learning Path

1. **Understand Version 1** (This)
   - Run and study the output
   - Read the code comments
   - Modify and experiment

2. **Study the Differences**
   - Compare minimal_GCS.h with real GCS.h
   - Compare minimal_GCS_Common.cpp with real GCS_Common.cpp
   - Understand what was simplified

3. **Move to Version 2**
   - See how to integrate with real ArduPilot
   - Use real MAVLink library
   - Work with real hardware

4. **Extend Both**
   - Add more messages to Version 1
   - Add vehicle-specific code to Version 2
   - Implement custom features

## Troubleshooting

### Compilation Errors

**Error**: `'usleep' was not declared in this scope`
**Fix**: Add `#include <unistd.h>` at top of file

**Error**: `'memcpy' was not declared in this scope`
**Fix**: Add `#include <string.h>` at top of file

**Error**: `'printf' was not declared in this scope`
**Fix**: Add `#include <stdio.h>` at top of file

### Runtime Issues

**Problem**: No output appears
**Fix**: Ensure stdout is not buffered - add `fflush(stdout);` after prints

**Problem**: Timestamps all the same
**Fix**: Check that `get_time_ms()` is working - add debug prints

**Problem**: Connection never becomes active
**Fix**: Check that `handle_heartbeat()` is being called and updating timestamp

## Resources

- **MAVLink Protocol**: https://mavlink.io/en/
- **Message Definitions**: https://mavlink.io/en/messages/common.html
- **HEARTBEAT**: https://mavlink.io/en/messages/common.html#HEARTBEAT
- **ArduPilot Dev Docs**: https://ardupilot.org/dev/

## Summary

This standalone version demonstrates the core concepts of GCS/MAVLink communication in a simple, understandable way. The patterns shown here (send, receive, handle, monitor) apply to all MAVLink messages in the full ArduPilot system.

Key takeaways:
- HEARTBEAT is the foundation of all MAVLink communication
- Sending and receiving follows the same pattern for all messages
- Connection monitoring uses timing and timeouts
- Virtual functions allow vehicle-specific customization
- The GCS/GCS_MAVLINK class structure scales to the full system

Use this as a learning tool and reference when working with the full ArduPilot GCS system!
