# MiniVehicle - Build and Run Instructions

This guide shows you how to build, upload, and test MiniVehicle with MAVLink/GCS integration.

## What You've Built

**MiniVehicle** is a minimal ArduPilot vehicle that demonstrates the **complete GCS integration** as described in `Version2_ArduPilot/minimal_GCS_Common.cpp`.

It implements all 4 integration steps:

### ✓ Step 1: Created Vehicle GCS Classes
- `GCS_MiniVehicle` (inherits from GCS)
- `GCS_MAVLINK_MiniVehicle` (inherits from GCS_MAVLINK)

### ✓ Step 2: Implemented Required Pure Virtuals
- `base_mode()` - Returns MAVLink mode flags
- `vehicle_system_status()` - Returns vehicle state
- `send_nav_controller_output()` - Sends navigation data
- `send_pid_tuning()` - Sends PID tuning data

### ✓ Step 3: Called GCS Functions in Main Loop
- `gcs().update_receive()` - Parse incoming messages
- `gcs().update_send()` - Send outgoing messages

### ✓ Step 4: Initialized GCS in Setup
- `gcs().init()` - Initialize GCS system
- `gcs().setup_console()` - Setup console/USB
- `gcs().setup_uarts()` - Setup telemetry ports

## File Structure

```
GCS_MAVLink_Pro/MiniVehicle/
├── main.cpp                        # Entry point (setup/loop)
├── MiniVehicle.h                   # Main vehicle class definition
├── MiniVehicle.cpp                 # Vehicle implementation
├── system.cpp                      # System functions (arm/mode/etc)
├── GCS_MiniVehicle.h              # GCS manager class
├── GCS_MiniVehicle.cpp            # GCS manager implementation
├── GCS_MAVLink_MiniVehicle.h      # MAVLink channel class
├── GCS_MAVLink_MiniVehicle.cpp    # MAVLink channel implementation
├── wscript                         # Build configuration
└── BUILD_AND_RUN.md               # This file
```

## Prerequisites

### Software Requirements

1. **ArduPilot Build Environment**
   - Git
   - Python 3.6+
   - gcc/g++ (for SITL) or arm-none-eabi-gcc (for hardware)
   - waf build system (included in ArduPilot)

2. **Ground Control Station**
   - MAVProxy (recommended for testing)
   - OR Mission Planner (Windows)
   - OR QGroundControl (cross-platform)

### Hardware Requirements

**For SITL (Simulation):**
- Any Linux/Mac/WSL computer
- No hardware needed!

**For Hardware:**
- Pixhawk, CubeOrange, or compatible flight controller
- USB cable for connection
- Telemetry radio (optional)

## Building

### Option 1: Build for SITL (Recommended for Testing)

SITL (Software In The Loop) runs on your PC - no hardware needed!

```bash
# Navigate to ArduPilot root
cd ~/ardupilot

# Configure for SITL
./waf configure --board=sitl

# Build MiniVehicle
./waf minivehicle
```

**Expected output:**
```
Build commands will be stored in build/sitl/compile_commands.json
'configure' finished successfully (1.234s)
Waf: Entering directory `/home/user/ardupilot/build/sitl'
[1/245] Compiling GCS_MAVLink_Pro/MiniVehicle/main.cpp
[2/245] Compiling GCS_MAVLink_Pro/MiniVehicle/MiniVehicle.cpp
[3/245] Compiling GCS_MAVLink_Pro/MiniVehicle/system.cpp
[4/245] Compiling GCS_MAVLink_Pro/MiniVehicle/GCS_MiniVehicle.cpp
[5/245] Compiling GCS_MAVLink_Pro/MiniVehicle/GCS_MAVLink_MiniVehicle.cpp
...
[245/245] Linking build/sitl/bin/minivehicle
'build' finished successfully (45.678s)
```

**Binary location:** `build/sitl/bin/minivehicle`

### Option 2: Build for Hardware (Pixhawk/Cube/etc)

```bash
# Configure for your board
# Common boards: CubeOrange, Pixhawk1, Pixhawk4, CubeBlack, etc.
./waf configure --board=CubeOrange

# Build MiniVehicle
./waf minivehicle

# Upload to hardware (connect via USB first)
./waf --upload minivehicle
```

**Expected output:**
```
Uploading to board...
Waiting for bootloader...
Found board: CubeOrange
Erasing...
Programming...
Verifying...
Success!
```

## Running

### Option 1: Run SITL

#### Basic Run
```bash
# Run the vehicle
./build/sitl/bin/minivehicle

# In another terminal, connect with MAVProxy
mavproxy.py --master=tcp:127.0.0.1:5760 --baudrate=115200
```

#### Run with Console Output
```bash
# Run with detailed console output
./build/sitl/bin/minivehicle --uartA=tcp:0

# Connect MAVProxy
mavproxy.py --master=tcp:127.0.0.1:5760
```

#### Run with Custom Parameters
```bash
# Run on different port
./build/sitl/bin/minivehicle --uartA=tcp:5762

# Connect
mavproxy.py --master=tcp:127.0.0.1:5762
```

### Option 2: Run on Hardware

#### Via USB
```bash
# No need to run manually - it auto-starts on boot!
# Just connect MAVProxy:
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

#### Via Telemetry Radio
```bash
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600
```

#### Via Network (WiFi/Ethernet)
```bash
mavproxy.py --master=udp:192.168.1.100:14550
```

## Testing MAVLink Communication

### Expected Behavior

When you connect with MAVProxy, you should see:

```
Connect tcp:127.0.0.1:5760 source_system=255
online system 1
AP: MiniVehicle ready
HEARTBEAT {type : GROUND_ROVER, autopilot : ARDUPILOTMEGA, base_mode : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode : 0, system_status : STANDBY}
AP: MiniVehicle 1.0.0 ready
Received 245 parameters
```

### Verify HEARTBEAT Messages

In MAVProxy, enable message display:

```
MAV> set heartbeat 1
```

You should see HEARTBEAT messages every second:

```
HEARTBEAT {type : GROUND_ROVER, autopilot : ARDUPILOTMEGA, base_mode : 81, custom_mode : 0, system_status : 3, mavlink_version : 3}
```

### Test Commands

Try these MAVProxy commands to test the integration:

#### Check Status
```
MAV> status
```

**Expected output:**
```
System ID: 1
Mode: MANUAL
Armed: False
GPS: No GPS
Heading: 0
Altitude: 0.0m
```

#### Arm the Vehicle
```
MAV> arm throttle
```

**Expected output:**
```
AP: Armed
HEARTBEAT {... base_mode : 209 ...}  ← Note: bit 7 set (armed)
```

#### Disarm the Vehicle
```
MAV> disarm
```

**Expected output:**
```
AP: Disarmed
HEARTBEAT {... base_mode : 81 ...}  ← Note: bit 7 clear (disarmed)
```

#### Change Mode
```
MAV> mode AUTO
```

**Expected output:**
```
AP: Mode: AUTO
HEARTBEAT {... custom_mode : 1 ...}
```

#### Request Parameters
```
MAV> param show *
```

**Expected output:**
```
(List of parameters - minimal set in this version)
```

### Debugging MAVLink Traffic

Enable verbose MAVLink debugging:

```bash
# In MAVProxy
MAV> set shownoise 1
MAV> set moddebug 3
```

You'll see all MAVLink messages:

```
>>>MAVPKT: HEARTBEAT(sysid=1, compid=1)
<<<MAVPKT: PARAM_REQUEST_LIST(sysid=255, compid=1)
>>>MAVPKT: PARAM_VALUE(sysid=1, compid=1, param_id='SYSID_THISMAV')
...
```

## What You Should See

### Console Output (Vehicle Side)

When MiniVehicle starts, you should see:

```
========================================
   MiniVehicle Starting Up
========================================

>>> Initializing ArduPilot Core
  - Initializing serial ports
  - Initializing GCS
GCS: Initialized with sysid=1
GCS_MAVLINK: Channel 0 initialized at 115200 baud
  - GCS initialized
  - Initializing AHRS
<<< ArduPilot Core initialized

>>> Ground Startup
Mode: MANUAL -> MANUAL (reason: AUTO)
<<< Ground Startup complete

========================================
   MiniVehicle Ready!
========================================

GCS_MAVLINK: HEARTBEAT sent - base_mode=0x51, custom_mode=0, status=3
```

Then every second:

```
GCS_MAVLINK: HEARTBEAT sent - base_mode=0x51, custom_mode=0, status=3
```

### GCS Side (MAVProxy)

When you connect MAVProxy:

```
Connecting to tcp:127.0.0.1:5760
MAV> online system 1
AP: MiniVehicle 1.0.0 ready
Received 0 parameters (0 left)
```

Then every second:

```
HEARTBEAT {type : GROUND_ROVER, autopilot : ARDUPILOTMEGA, base_mode : 81, custom_mode : 0, system_status : 3}
```

## Troubleshooting

### Problem: Build Errors

**Error**: `Cannot find GCS.h`

**Solution**: Make sure you're building from ArduPilot root:
```bash
cd ~/ardupilot
./waf minivehicle
```

**Error**: `undefined reference to GCS_MAVLINK::send_heartbeat()`

**Solution**: Make sure GCS_MAVLink library is in wscript. It should already be included.

**Error**: `Pure virtual function called`

**Solution**: Make sure all 4 required pure virtuals are implemented in GCS_MAVLink_MiniVehicle.cpp:
- `base_mode()`
- `vehicle_system_status()`
- `send_nav_controller_output()`
- `send_pid_tuning()`

### Problem: No HEARTBEAT Received

**Check 1**: Is `update_send()` being called?

Add debug print in `MiniVehicle::update_gcs()`:
```cpp
hal.console->printf("update_gcs() called\n");
```

**Check 2**: Is serial port configured correctly?

Check serial manager parameters:
```
MAV> param show SERIAL*
```

**Check 3**: Are you connected to the right port?

Try different MAVProxy connections:
```bash
# USB
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200

# TCP (SITL)
mavproxy.py --master=tcp:127.0.0.1:5760

# UDP
mavproxy.py --master=udp:127.0.0.1:14550
```

### Problem: Connection Drops

**Symptom**: MAVProxy shows "No HEARTBEAT" after a few seconds

**Check**: Is `update_send()` being called every loop?

HEARTBEAT must be sent every ~1 second. If your loop is blocked or running too slow, connection will drop.

**Solution**: Make sure `minivehicle.loop()` is called repeatedly and quickly.

### Problem: Messages Not Parsed

**Symptom**: HEARTBEAT sent but commands not working

**Check**: Is `update_receive()` being called?

Add debug print:
```cpp
void MiniVehicle::update_gcs() {
    hal.console->printf("Calling update_receive/send\n");
    gcs().update_receive();
    gcs().update_send();
}
```

**Check**: Are messages being routed correctly?

In `handle_message()`, make sure you call base class:
```cpp
default:
    GCS_MAVLINK::handle_message(msg);  // CRITICAL!
    break;
```

### Problem: Compile Warnings

**Warning**: `unused variable 'minivehicle'`

**Solution**: This is normal if you're not using all vehicle features. Ignore or add `(void)minivehicle;` to silence.

## Next Steps

### 1. Test Basic Integration ✓

You've now verified:
- ✓ HEARTBEAT sends at 1 Hz
- ✓ HEARTBEAT received by GCS
- ✓ Connection status updates
- ✓ Basic commands work (arm/disarm)

### 2. Add More Messages

Follow the HEARTBEAT pattern to add:

#### ATTITUDE (Roll/Pitch/Yaw)
```cpp
void GCS_MAVLINK_MiniVehicle::send_attitude() const {
    Vector3f omega;
    minivehicle.ahrs.get_gyro(omega);

    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        minivehicle.ahrs.roll,
        minivehicle.ahrs.pitch,
        minivehicle.ahrs.yaw,
        omega.x, omega.y, omega.z
    );
}
```

#### GPS Position
```cpp
void GCS_MAVLINK_MiniVehicle::send_gps_raw_int() const {
    const Location &loc = minivehicle.ahrs.get_location();

    mavlink_msg_gps_raw_int_send(
        chan,
        AP_HAL::micros64(),
        3,  // fix type (3 = 3D fix)
        loc.lat,
        loc.lng,
        loc.alt * 10,  // mm
        UINT16_MAX,    // eph
        UINT16_MAX,    // epv
        UINT16_MAX,    // vel
        UINT16_MAX,    // cog
        255            // satellites_visible
    );
}
```

#### System Status
```cpp
void GCS_MAVLINK_MiniVehicle::send_sys_status() const {
    mavlink_msg_sys_status_send(
        chan,
        gcs().control_sensors_present,
        gcs().control_sensors_enabled,
        gcs().control_sensors_health,
        500,    // load (‰)
        12600,  // voltage (mV)
        1000,   // current (cA)
        80,     // battery remaining (%)
        0, 0, 0, 0, 0, 0
    );
}
```

### 3. Add Vehicle Features

Implement real vehicle functionality:

- **Sensors**: Read GPS, compass, barometer
- **Control**: Implement steering, throttle controllers
- **Navigation**: Add waypoint following
- **Missions**: Support mission upload/download
- **Failsafes**: Add radio/GPS/battery failsafes
- **Logging**: Enable DataFlash logging

### 4. Customize for Your Application

Use this as a template for:

- **Ground Rover**: Drive control, waypoint following
- **Boat**: Marine navigation, anchor mode
- **Robot**: Manipulator control, vision processing
- **Custom Vehicle**: Whatever you need!

## Summary

You've successfully:

✓ Created a minimal ArduPilot vehicle
✓ Integrated GCS/MAVLink communication
✓ Implemented all 4 required integration steps
✓ Built and tested with real GCS tools
✓ Verified HEARTBEAT send/receive

This demonstrates the **complete pattern** used by all ArduPilot vehicles (Copter, Plane, Rover, Sub, etc.). The same architecture scales from this minimal example to full production systems with hundreds of features.

**Key Takeaways:**

1. **GCS Integration is straightforward** - Just 4 steps
2. **HEARTBEAT is the foundation** - Everything else builds on it
3. **Pattern is consistent** - Same for all messages and commands
4. **Base class does heavy lifting** - You just implement vehicle-specific parts
5. **Architecture scales** - Minimal → Full system with no vehicle code changes

## Resources

- **Full Documentation**: `../ARCHITECTURE_DOCUMENTATION.md`
- **Quick Reference**: `../QUICK_REFERENCE.md`
- **Version Comparison**: `../VERSIONS_COMPARISON.md`
- **MAVLink Docs**: https://mavlink.io/en/
- **ArduPilot Dev Guide**: https://ardupilot.org/dev/

## Support

If you encounter issues:

1. **Check this guide** - Most issues covered in Troubleshooting
2. **Review documentation** - See files in `GCS_MAVLink_Pro/`
3. **Enable debugging** - Use `hal.console->printf()` liberally
4. **Test with MAVProxy** - Use `--debug` flag for verbose output
5. **Compare with examples** - See Rover/Copter/Plane for reference

Good luck with your ArduPilot GCS integration!
