# Rover Quick Reference Guide

Quick reference for common Rover development tasks, file locations, and code patterns.

---

## File Locations Quick Map

```
Rover/
├── Rover.h/cpp              → Main vehicle class
├── mode*.cpp                → Mode implementations
├── GCS_MAVLink_Rover.cpp    → MAVLink message handling
├── GCS_Rover.cpp            → GCS manager
├── system.cpp               → Init & mode switching
├── failsafe.cpp             → Failsafe logic
├── Parameters.h/cpp         → Parameter definitions
├── commands.cpp             → Mission commands
└── sensors.cpp              → Sensor reading

libraries/
├── AR_WPNav/                → Waypoint navigation
├── AR_Motors/               → Motor control
├── AC_AttitudeControl/      → Control loops
├── AP_SmartRTL/             → Smart return
└── GCS_MAVLink/             → MAVLink base
```

---

## Common Code Patterns

### 1. Adding a New Mode

**Step 1:** Declare mode class in `mode.h`
```cpp
class ModeMyMode : public Mode {
public:
    Number mode_number() const override { return Number::MY_MODE; }
    const char *name4() const override { return "MYMD"; }
    void update() override;

protected:
    bool _enter() override;
    void _exit() override;
};
```

**Step 2:** Add mode number to enum in `mode.h`
```cpp
enum class Number : uint8_t {
    MANUAL = 0,
    // ... existing modes
    MY_MODE = 20,  // Choose unused number
};
```

**Step 3:** Create implementation file `mode_mymode.cpp`
```cpp
bool ModeMyMode::_enter() {
    // Initialize mode
    return true;
}

void ModeMyMode::update() {
    // Called at 400 Hz
    // Implement mode behavior here
}

void ModeMyMode::_exit() {
    // Cleanup
}
```

**Step 4:** Add mode instance to `Rover.h`
```cpp
class Rover {
    ModeMyMode mode_mymode;
};
```

**Step 5:** Add to mode lookup in `mode.cpp`
```cpp
Mode *Rover::mode_from_mode_num(const Mode::Number num) {
    switch (num) {
        // ... existing cases
        case Mode::Number::MY_MODE:
            return &mode_mymode;
    }
}
```

### 2. Handling a New MAVLink Message

**In `GCS_MAVLink_Rover.h`:**
```cpp
class GCS_MAVLINK_Rover : public GCS_MAVLINK {
    void handle_my_message(const mavlink_message_t &msg);
};
```

**In `GCS_MAVLink_Rover.cpp`:**
```cpp
void GCS_MAVLINK_Rover::handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MY_MESSAGE:
        handle_my_message(msg);
        break;

    default:
        GCS_MAVLINK::handle_message(msg);  // CRITICAL!
        break;
    }
}

void GCS_MAVLINK_Rover::handle_my_message(const mavlink_message_t &msg) {
    mavlink_my_message_t packet;
    mavlink_msg_my_message_decode(&msg, &packet);

    // Process message
    // ...
}
```

### 3. Sending a Custom MAVLink Message

**In `GCS_MAVLink_Rover.cpp`:**
```cpp
void GCS_MAVLINK_Rover::send_my_message() const {
    // Check payload space
    if (!HAVE_PAYLOAD_SPACE(chan, MY_MESSAGE)) {
        return;
    }

    // Send message
    mavlink_msg_my_message_send(
        chan,
        field1,
        field2,
        // ... more fields
    );
}

// Add to try_send_message()
bool GCS_MAVLINK_Rover::try_send_message(enum ap_message id) {
    switch (id) {
    case MSG_MY_MESSAGE:
        send_my_message();
        return true;
    // ... existing cases
    }
}
```

### 4. Adding a New Parameter

**In `Parameters.h`:**
```cpp
class Parameters {
    enum {
        // ... existing params
        k_param_my_param = 250,  // Choose unused key
    };

    AP_Int16 my_param;  // or AP_Float, AP_Int8, etc.
};

// In var_info array:
const AP_Param::GroupInfo Parameters::var_info[] = {
    // @Param: MY_PARAM
    // @DisplayName: My Parameter
    // @Description: Description of what it does
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("MY_PARAM", k_param_my_param, Parameters, my_param, 50),  // default=50
};
```

**Usage:**
```cpp
int16_t value = g.my_param;  // Read
g.my_param.set(75);          // Write
```

### 5. Adding a Failsafe

**In `failsafe.cpp`:**
```cpp
void Rover::check_my_failsafe() {
    bool failsafe_triggered = /* condition */;

    if (failsafe_triggered) {
        if (!(failsafe.bits & FAILSAFE_EVENT_MY)) {
            // First detection
            failsafe_trigger(FAILSAFE_EVENT_MY, "My Failsafe", true);
        }
    } else {
        if (failsafe.bits & FAILSAFE_EVENT_MY) {
            // Condition cleared
            failsafe_trigger(FAILSAFE_EVENT_MY, "My Failsafe", false);
        }
    }
}
```

**In `defines.h`:**
```cpp
#define FAILSAFE_EVENT_MY (1<<7)  // Choose unused bit
```

**Add to scheduler in `Rover.cpp`:**
```cpp
SCHED_TASK(check_my_failsafe, 10, 200, 150),  // 10 Hz
```

### 6. Adding a Scheduled Task

**In `Rover.cpp`:**
```cpp
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    // ... existing tasks
    SCHED_TASK(my_task_func, 50, 200, 200),
    //         function     Hz   µs   priority
};
```

**Implement function in Rover.cpp:**
```cpp
void Rover::my_task_func() {
    // Task implementation
}
```

### 7. Navigation to Waypoint

**In any mode:**
```cpp
void ModeMyMode::update() {
    // Set destination
    Location dest(lat, lon, alt);
    g2.wp_nav.set_desired_location(dest);

    // Navigate (updates controllers)
    navigate_to_waypoint();

    // Check arrival
    if (reached_destination()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Arrived!");
    }
}
```

### 8. Control Vehicle Directly

**Steering + Throttle:**
```cpp
void ModeMyMode::update() {
    // Direct motor control
    float steering = 1000;    // -4500 to +4500
    float throttle = 50;      // -100 to +100

    g2.motors.set_steering(steering);
    g2.motors.set_throttle(throttle);
}
```

**Using Control Loops:**
```cpp
void ModeMyMode::update() {
    // Heading control
    float desired_heading_cd = 18000;  // 180 degrees
    calc_steering_to_heading(desired_heading_cd);

    // Speed control
    float target_speed = 5.0;  // m/s
    calc_throttle(target_speed, true);  // true = enable avoidance
}
```

---

## Important Functions

### Mode Base Class Helpers

```cpp
// Read pilot input
void get_pilot_desired_steering_and_throttle(float &steer, float &thr);
void get_pilot_desired_steering_and_speed(float &steer, float &speed);
void get_pilot_desired_heading_and_speed(float &heading, float &speed);

// Navigation
void navigate_to_waypoint();  // Uses WPNav to follow path
bool stop_vehicle();           // Controlled stop

// Steering control
void calc_steering_to_heading(float heading_cd, float rate_max=0);
void calc_steering_from_turn_rate(float turn_rate_rads);
void calc_steering_from_lateral_acceleration(float lat_accel);

// Throttle control
void calc_throttle(float target_speed, bool avoidance_enabled);

// Speed helpers
float get_speed_default(bool rtl=false);  // Get mode's default speed
float calc_speed_nudge(float target_speed, bool reversed);
```

### Rover Main Functions

```cpp
// Mode management
bool set_mode(Mode &mode, ModeReason reason);
Mode *mode_from_mode_num(Mode::Number num);

// State queries
bool is_boat() const;
bool is_balancebot() const;

// Waypoint info
bool get_wp_distance_m(float &distance);
bool get_wp_bearing_deg(float &bearing);
bool get_wp_crosstrack_error_m(float &error);

// Home management
bool set_home_to_current_location(bool lock);
bool set_home(const Location &loc, bool lock);
```

### GCS Functions

```cpp
// Text messages
gcs().send_text(MAV_SEVERITY_INFO, "Message");
gcs().send_text(MAV_SEVERITY_WARNING, "Warning");
gcs().send_text(MAV_SEVERITY_ERROR, "Error");
gcs().send_text(MAV_SEVERITY_CRITICAL, "Critical!");

// Parameter access
AP_Param::set_by_name("PARAM_NAME", value);
AP_Param::get_by_name("PARAM_NAME", value);
```

---

## Key Enums & Constants

### Mode Numbers
```cpp
MANUAL       = 0
ACRO         = 1
STEERING     = 3
HOLD         = 4
LOITER       = 5
FOLLOW       = 6
SIMPLE       = 7
DOCK         = 8
CIRCLE       = 9
AUTO         = 10
RTL          = 11
SMART_RTL    = 12
GUIDED       = 15
INITIALISING = 16
```

### Mode Reason
```cpp
enum class ModeReason : uint8_t {
    UNKNOWN,
    RC_COMMAND,          // Mode switch from RC
    GCS_COMMAND,         // Mode change from GCS
    RADIO_FAILSAFE,      // RC loss failsafe
    BATTERY_FAILSAFE,    // Battery failsafe
    GCS_FAILSAFE,        // GCS loss failsafe
    EKF_FAILSAFE,        // EKF error
    GPS_GLITCH,          // GPS glitch
    MISSION_END,         // Mission complete
    THROTTLE_LAND_ESCAPE,
    FENCE_BREACHED,      // Geofence breach
    CRASH_FAILSAFE,      // Crash detected
    INITIALISED,         // Initial mode at boot
};
```

### Failsafe Actions
```cpp
enum class FailsafeAction : int8_t {
    None = 0,
    RTL = 1,
    Hold = 2,
    SmartRTL = 3,
    SmartRTL_Hold = 4,
    Terminate = 5,
    Loiter_Hold = 6,
};
```

### Frame Types
```cpp
enum frame_class {
    FRAME_UNDEFINED = 0,
    FRAME_ROVER = 1,      // Ground vehicle
    FRAME_BOAT = 2,       // Water vehicle
    FRAME_BALANCEBOT = 3, // Self-balancing
};
```

---

## Parameter Access Patterns

### Common Parameters

```cpp
g.speed_cruise         // Cruise speed (m/s)
g.throttle_cruise      // Cruise throttle (%)
g.fs_action            // Failsafe action
g.fs_timeout           // Failsafe timeout (s)
g.initial_mode         // Boot mode
g.mode1-6              // Mode switch positions

g2.motors              // Motor configuration
g2.attitude_control    // Attitude controller
g2.wp_nav              // Waypoint navigation
g2.pos_control         // Position controller
g2.frame_type          // Vehicle frame type
```

### Parameter Groups

**G (main):** Original parameters
**G2 (extended):** Newer parameters and objects

---

## Debugging Tips

### 1. Console Output
```cpp
hal.console->printf("Debug: value=%d\n", value);
```

### 2. GCS Text Messages
```cpp
gcs().send_text(MAV_SEVERITY_INFO, "Debug: %s", str);
```

### 3. Logging
```cpp
logger.Write("MYLOG", "TimeUS,Field1,Field2", "QHH",
             AP_HAL::micros64(), field1, field2);
```

### 4. Check Mode
```cpp
if (control_mode == &mode_auto) {
    // Do auto-specific stuff
}
```

### 5. Check Vehicle State
```cpp
if (!rover.initialised) return;
if (!arming.is_armed()) return;
if (!have_position) return;
```

---

## Common Pitfalls

### ❌ DON'T: Forget to call base class
```cpp
void handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MY_MSG:
        // handle
        break;
    }
    // ❌ Missing: GCS_MAVLINK::handle_message(msg);
}
```

### ✅ DO: Always call base class
```cpp
void handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MY_MSG:
        handle_my_msg(msg);
        break;
    default:
        GCS_MAVLINK::handle_message(msg);  // ✅
        break;
    }
}
```

### ❌ DON'T: Block in scheduler tasks
```cpp
SCHED_TASK(my_task, 10, 200, 100),

void Rover::my_task() {
    hal.scheduler->delay(100);  // ❌ Blocks scheduler!
}
```

### ✅ DO: Use state machines for delays
```cpp
void Rover::my_task() {
    static uint32_t start_ms = 0;

    if (start_ms == 0) {
        start_ms = AP_HAL::millis();
        return;
    }

    if (AP_HAL::millis() - start_ms < 100) {
        return;  // Not ready yet
    }

    // Do work
    start_ms = 0;  // Reset
}
```

### ❌ DON'T: Modify mode outside mode class
```cpp
// In Rover::some_function()
mode_auto._submode = ModeAuto::SubMode::RTL;  // ❌ Bad!
```

### ✅ DO: Use mode's public interface
```cpp
if (control_mode == &mode_auto) {
    mode_auto.start_RTL();  // ✅ Use public method
}
```

---

## Build & Test Commands

### Build for SITL
```bash
./waf configure --board=sitl
./waf rover
```

### Run SITL
```bash
./build/sitl/bin/ardurover --model rover

# With custom home location
./build/sitl/bin/ardurover --home=LAT,LON,ALT,YAW
```

### Connect MAVProxy
```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --map --console
```

### Build for Hardware
```bash
./waf configure --board=CubeOrange
./waf rover
./waf --upload rover
```

### Useful MAVProxy Commands
```
mode auto          # Switch to AUTO mode
mode guided        # Switch to GUIDED
wp list            # List waypoints
wp load file.txt   # Load mission
param show SPEED*  # Show speed parameters
param set SPEED_CRUISE 5  # Set cruise speed to 5 m/s
arm throttle       # Arm vehicle
disarm             # Disarm vehicle
```

---

## Useful Code Snippets

### Get Current Location
```cpp
Location loc;
if (ahrs.get_location(loc)) {
    // loc.lat, loc.lng, loc.alt available
}
```

### Get Current Heading
```cpp
float heading_cd = ahrs.yaw_sensor;  // Centidegrees
float heading_rad = ahrs.yaw;         // Radians
```

### Get Ground Speed
```cpp
float speed_ms = ahrs.groundspeed();  // m/s
```

### Calculate Bearing to Location
```cpp
Location target(...);
float bearing_cd = current_loc.get_bearing_to(target);
```

### Calculate Distance to Location
```cpp
Location target(...);
float distance_m = current_loc.get_distance(target);
```

### Check GPS Lock
```cpp
if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
    // Have 3D fix
}
```

### Check EKF Health
```cpp
if (!ekf_position_ok()) {
    // EKF unhealthy
}
```

### Limit Value
```cpp
float limited = constrain_float(value, min, max);
int16_t limited = constrain_int16(value, min, max);
```

### Wrap Angle
```cpp
float wrapped = wrap_360_cd(angle_cd);   // 0-36000
float wrapped = wrap_180_cd(angle_cd);   // -18000 to +18000
float wrapped = wrap_PI(angle_rad);      // -PI to +PI
```

---

## File Templates

### New Mode Template
```cpp
// mode_mymode.cpp
#include "Rover.h"

bool ModeMyMode::_enter() {
    // Initialize mode-specific state
    return true;
}

void ModeMyMode::update() {
    // Read pilot input (if manual mode)
    float steering, throttle;
    get_pilot_desired_steering_and_throttle(steering, throttle);

    // Or navigate autonomously
    // navigate_to_waypoint();

    // Set motor output
    g2.motors.set_steering(steering);
    g2.motors.set_throttle(throttle);
}

void ModeMyMode::_exit() {
    // Cleanup
}

float ModeMyMode::get_distance_to_destination() const {
    return _distance_to_destination;
}
```

### New Scheduler Task Template
```cpp
// In Rover.h
void my_new_task();

// In Rover.cpp
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    // ... existing tasks
    SCHED_TASK(my_new_task, 10, 200, 200),
};

void Rover::my_new_task() {
    // Task runs at 10 Hz
    // Should complete in ~200 microseconds
}
```

---

## Quick Comparison: MiniVehicle vs Rover

| Aspect | MiniVehicle | Rover |
|--------|-------------|-------|
| **Setup** | `gcs().init()` | Same |
| **Loop** | `update_gcs()` | Same (via scheduler) |
| **Modes** | 1 simple loop | 13+ full modes |
| **Motors** | Direct output | AR_Motors (6+ types) |
| **Navigation** | None | AR_WPNav (full stack) |
| **Control** | None | AR_AttitudeControl (10+ PIDs) |
| **Failsafe** | None | 8+ types with actions |
| **Messages** | HEARTBEAT | 50+ messages |
| **Commands** | 4 basic | 30+ commands |

**Key insight:** Same foundation (AP_Vehicle, GCS integration), different complexity levels!

---

## Resources

- **ArduPilot Dev Wiki:** https://ardupilot.org/dev/
- **Rover Docs:** https://ardupilot.org/rover/
- **MAVLink Docs:** https://mavlink.io/en/
- **Your Code:** `/home/user/ardupilot/Rover/`
- **MiniVehicle:** `/home/user/ardupilot/GCS_MAVLink_Pro/MiniVehicle/`
- **Full Analysis:** `ROVER_COMPLETE_ANALYSIS.md`

---

**This quick reference covers the most common development tasks for Rover!**
