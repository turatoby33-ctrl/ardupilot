# Version 2: ArduPilot-Integrated Minimal GCS with HEARTBEAT

This version integrates with the **real ArduPilot GCS infrastructure**. It uses the actual GCS.h, GCS.cpp, GCS_MAVLink.h, and GCS_MAVLink.cpp files, but with a minimal GCS_Common.cpp that implements only HEARTBEAT functionality.

## Purpose

This version demonstrates:
- **Integration** with real ArduPilot GCS classes
- **HEARTBEAT** send/receive using actual MAVLink library
- **Real serial communication** (UART/USB)
- **Vehicle-specific** customization patterns
- **Production-ready** architecture (scales to full system)

## What's Included

### Files

1. **minimal_GCS_Common.cpp** - Minimal common functionality
   - Works with real GCS.h and GCS_MAVLink.h
   - Implements HEARTBEAT send/receive
   - Real MAVLink library integration
   - Actual UART communication
   - Production code patterns

2. **integration_example.cpp** - Complete vehicle example
   - GCS_MyVehicle class (manager)
   - GCS_MAVLINK_MyVehicle class (channel)
   - MyVehicle class (main vehicle)
   - Build configuration
   - Usage instructions

## Key Differences from Version 1

| Feature | Version 1 (Standalone) | Version 2 (Integrated) |
|---------|----------------------|------------------------|
| **Dependencies** | None (self-contained) | Requires ArduPilot GCS |
| **MAVLink** | Simplified structures | Full MAVLink library |
| **Communication** | Simulated | Real UART/serial |
| **Channels** | Single | Multiple (5-8) |
| **Integration** | Standalone demo | Production-ready |
| **Complexity** | Simple | More complex |
| **Build** | g++ | waf build system |

## Integration Steps

### Step 1: File Placement

Place files in ArduPilot directory structure:

```
ardupilot/
├── libraries/GCS_MAVLink/
│   └── GCS_Common_Minimal.cpp   (copy minimal_GCS_Common.cpp here)
│
└── MyVehicle/                    (your vehicle directory)
    ├── GCS_MyVehicle.h
    ├── GCS_MyVehicle.cpp
    ├── GCS_MAVLink_MyVehicle.h
    ├── GCS_MAVLink_MyVehicle.cpp
    ├── MyVehicle.h
    ├── MyVehicle.cpp
    └── wscript
```

### Step 2: Modify GCS Build

Edit `libraries/GCS_MAVLink/wscript`:

```python
def build(bld):
    # ... existing code ...

    # Comment out full GCS_Common:
    # 'GCS_Common.cpp',

    # Add minimal version:
    'GCS_Common_Minimal.cpp',

    # ... rest of files ...
```

### Step 3: Build

```bash
./waf configure --board=<your_board>
./waf myvehicle
```

### Step 4: Upload and Test

```bash
./waf --upload myvehicle
```

Connect with MAVProxy:
```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

You should see:
```
HEARTBEAT {type : GROUND_ROVER, autopilot : ARDUPILOTMEGA, ...}
```

## Architecture

### Class Hierarchy

```
ArduPilot GCS Infrastructure:
├── GCS (base)                     ← From libraries/GCS_MAVLink/GCS.h
│   └── GCS_MyVehicle (vehicle)    ← You implement this
│
└── GCS_MAVLINK (base)             ← From libraries/GCS_MAVLink/GCS.h
    └── GCS_MAVLINK_MyVehicle      ← You implement this

Common Functionality:
└── GCS_Common_Minimal.cpp         ← Minimal HEARTBEAT implementation
    (replaces full GCS_Common.cpp)
```

### Message Flow

```
Vehicle Main Loop
    ↓
gcs().update_receive()
    ↓
GCS::update_receive()
    ↓
GCS_MAVLINK::update_receive()
    ↓
Read from UART → parse bytes
    ↓
Complete message → handle_message()
    ↓
Route to handler (HEARTBEAT, etc.)
    ↓
Update state, take action

---

Vehicle Main Loop
    ↓
gcs().update_send()
    ↓
GCS::update_send()
    ↓
GCS_MAVLINK::update_send()
    ↓
Check message intervals
    ↓
send_heartbeat() (at 1 Hz)
    ↓
Pack message → Write to UART
```

## Required Implementations

### In GCS_MyVehicle

```cpp
// Vehicle identification
uint32_t custom_mode() const override;
MAV_TYPE frame_type() const override;
bool vehicle_initialised() const override;

// Sensor status
void update_vehicle_sensor_status_flags(void) override;

// Factory method
GCS_MAVLINK_MyVehicle *new_gcs_mavlink_backend(...) override;
```

### In GCS_MAVLINK_MyVehicle

```cpp
// REQUIRED (pure virtual):
uint8_t base_mode() const override;
MAV_STATE vehicle_system_status() const override;
void send_nav_controller_output() const override;
void send_pid_tuning() override;

// OPTIONAL (but recommended):
float vfr_hud_airspeed() const override;
int16_t vfr_hud_throttle() const override;
void handle_message(...) override;
MAV_RESULT handle_command_int_packet(...) override;
```

## Main Loop Integration

### Essential Pattern

```cpp
void MyVehicle::loop() {
    // Your control loops
    read_sensors();
    run_control();
    output_motors();

    // GCS update - CRITICAL!
    // Must be called regularly (50 Hz typical)
    gcs().update_receive();  // Parse incoming
    gcs().update_send();     // Send outgoing
}
```

**Important**:
- Call both `update_receive()` and `update_send()`
- Call at regular intervals (20-50 Hz typical)
- Don't block - these return quickly
- update_receive() has timeout (~1ms default)

## Adding More Messages

To extend beyond HEARTBEAT, add to `minimal_GCS_Common.cpp`:

### 1. Add Send Function

```cpp
void GCS_MAVLINK::send_attitude() const
{
    CHECK_PAYLOAD_SIZE(ATTITUDE);

    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        roll, pitch, yaw,
        rollspeed, pitchspeed, yawspeed
    );
}
```

### 2. Add to update_send()

```cpp
void GCS_MAVLINK::update_send()
{
    // ... existing heartbeat code ...

    // Add ATTITUDE at 10 Hz
    static uint32_t last_attitude_ms = 0;
    uint32_t now = AP_HAL::millis();

    if (now - last_attitude_ms >= 100) {  // 10 Hz
        send_attitude();
        last_attitude_ms = now;
    }
}
```

### 3. Add Receive Handler

```cpp
void GCS_MAVLINK::handle_param_request_list(const mavlink_message_t &msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(&msg, &packet);

    // Handle parameter list request
    // ...
}
```

### 4. Add to Router

```cpp
void GCS_MAVLINK::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat(msg);
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  // NEW!
        handle_param_request_list(msg);
        break;

    // ... more cases ...
    }
}
```

## Transition to Full GCS

Once your minimal version works, switch to full functionality:

### Step 1: Update Build

In `libraries/GCS_MAVLink/wscript`:
```python
# Uncomment:
'GCS_Common.cpp',

# Remove:
# 'GCS_Common_Minimal.cpp',
```

### Step 2: Rebuild

```bash
./waf clean
./waf configure --board=<your_board>
./waf myvehicle
```

### Step 3: You Now Have

✓ Parameters (get/set/list)
✓ Missions (upload/download)
✓ Commands (200+ supported)
✓ All standard telemetry
✓ File transfer (FTP)
✓ Logging
✓ And much more!

**Your vehicle-specific code doesn't change!**

The beauty of this architecture is that your GCS_MyVehicle and GCS_MAVLINK_MyVehicle classes work with both minimal and full GCS_Common.cpp.

## Debugging

### Enable Debug Output

```cpp
// In your code, add:
hal.console->printf("Debug: value=%f\n", value);
```

Connect serial console:
```bash
minicom -D /dev/ttyACM0 -b 115200
```

### Common Issues

**Problem**: HEARTBEAT not received
- Check `update_send()` is being called
- Verify serial port is configured correctly
- Check baudrate matches GCS
- Ensure minimal_GCS_Common.cpp is being compiled

**Problem**: Messages not parsed
- Check `update_receive()` is being called
- Verify MAVLink library is linked
- Check message ID is in handle_message()

**Problem**: Connection drops
- HEARTBEAT must be sent every ~1 second
- Check main loop isn't blocking
- Verify update_send() timing

### MAVProxy Debug

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200 --debug
```

Shows all MAVLink traffic:
```
>>>MAVPKT: HEARTBEAT(...)
<<<MAVPKT: HEARTBEAT(...)
```

## Comparison with Real GCS_Common.cpp

### Minimal Version (This)

- ~300 lines
- HEARTBEAT only
- Educational
- Easy to understand
- Starting point

### Full GCS_Common.cpp

- ~30,000 lines (~83,000 tokens)
- 300+ message handlers
- 200+ message senders
- Parameter protocol
- Mission protocol
- Command handling
- Stream management
- FTP support
- Logging
- And much more...

### What's the Same

✓ Class structure (GCS/GCS_MAVLINK)
✓ Inheritance pattern
✓ Virtual function overrides
✓ Message send/receive pattern
✓ HEARTBEAT implementation
✓ Integration with vehicle

### What's Different

✗ Number of messages (1 vs 300+)
✗ Complexity (simple vs extensive)
✗ Features (basic vs complete)
✗ Size (small vs large)

## Example Vehicles

See these for reference implementations:

**ArduCopter**:
```
ArduCopter/
├── GCS_Copter.h/cpp
└── GCS_MAVLink_Copter.h/cpp
```

**ArduRover**:
```
Rover/
├── GCS_Rover.h/cpp
└── GCS_MAVLink_Rover.h/cpp
```

**ArduPlane**:
```
ArduPlane/
├── GCS_Plane.h/cpp
└── GCS_MAVLink_Plane.h/cpp
```

These use the full GCS_Common.cpp but follow the same patterns shown in this minimal version.

## Testing Checklist

- [ ] Code compiles without errors
- [ ] Uploads to hardware successfully
- [ ] Serial port opens at correct baudrate
- [ ] HEARTBEAT visible in MAVProxy/Mission Planner
- [ ] HEARTBEAT sent every ~1 second
- [ ] GCS HEARTBEAT received and parsed
- [ ] Connection status updates correctly
- [ ] No crashes or hangs
- [ ] Console output shows correct messages
- [ ] Ready to add more messages!

## Next Steps

1. **Test** the minimal version
   - Verify HEARTBEAT working
   - Check connection status
   - Monitor serial traffic

2. **Add More Messages**
   - ATTITUDE
   - GPS_RAW_INT
   - SYS_STATUS
   - Follow HEARTBEAT pattern

3. **Add Vehicle Features**
   - Custom commands
   - Vehicle-specific messages
   - Mode handling
   - Parameter support

4. **Transition to Full**
   - Switch to GCS_Common.cpp
   - Get complete MAVLink
   - Production-ready system

## Resources

- **Integration Example**: `integration_example.cpp` in this directory
- **Full Documentation**: `../ARCHITECTURE_DOCUMENTATION.md`
- **Quick Reference**: `../QUICK_REFERENCE.md`
- **Version 1**: `../Version1_Standalone/` for comparison
- **MAVLink Docs**: https://mavlink.io
- **ArduPilot Dev**: https://ardupilot.org/dev/

## Summary

Version 2 bridges the gap between learning (Version 1) and production (full GCS_Common.cpp). It uses real ArduPilot infrastructure with minimal complexity, providing a solid foundation for building vehicle-specific GCS functionality.

Key advantages:
✓ Real MAVLink library
✓ Real serial communication
✓ Production architecture
✓ Easy to extend
✓ Scales to full system
✓ No vehicle code changes needed

Perfect for:
- Learning ArduPilot GCS integration
- Building new vehicle types
- Understanding message patterns
- Incremental development
- Testing and debugging

Start here, then grow to full GCS_Common.cpp when needed!
