# ArduPilot Rover - Complete Vehicle Analysis

This document provides a comprehensive analysis of the ArduPilot Rover vehicle, covering architecture, all dependencies, modes, GCS integration, and how it compares to MiniVehicle.

---

## Table of Contents

1. [Overview](#overview)
2. [File Structure (50+ Files)](#file-structure)
3. [Architecture](#architecture)
4. [All Dependencies](#all-dependencies)
5. [Mode System (13+ Modes)](#mode-system)
6. [GCS/MAVLink Integration](#gcs-mavlink-integration)
7. [Key Systems](#key-systems)
8. [MiniVehicle vs Rover Comparison](#minivehicle-vs-rover-comparison)
9. [Learning Path](#learning-path)

---

## Overview

**ArduPilot Rover** is a production-grade autonomous ground vehicle firmware supporting:

- **13+ Flight Modes** - Manual, Acro, Steering, Hold, Loiter, Follow, Simple, Dock, Circle, Auto, RTL, SmartRTL, Guided
- **Multiple Vehicle Types** - Ground rovers, boats, balance bots, omni-drives, walking robots, sailboats
- **Full Autonomy** - Mission planning, waypoint navigation, obstacle avoidance
- **Complete Safety** - Failsafes, geofencing, crash detection, arming checks
- **Professional GCS** - 50+ MAVLink messages, full telemetry, parameter management

**Code Stats:**
- **50+ source files**
- **~25,000 lines of code**
- **24+ library dependencies**
- **30+ scheduler tasks**
- **8+ failsafe types**

---

## File Structure

### Core Files (Main Vehicle Logic)

```
Rover/
├── Rover.h                  # Main vehicle class declaration (459 lines)
├── Rover.cpp                # Main implementation & scheduler (565 lines)
├── mode.h                   # Mode base class & all mode declarations (927 lines)
├── mode.cpp                 # Mode base class implementation (570 lines)
├── defines.h                # Constants, enums, logging masks (102 lines)
├── config.h                 # Configuration settings (78 lines)
├── version.h                # Version information
└── wscript                  # Build configuration (32 lines)
```

### Mode Implementation Files (13 Modes)

```
├── mode_manual.cpp          # Direct pilot control (39 lines)
├── mode_acro.cpp            # Turn rate + speed control (65 lines)
├── mode_steering.cpp        # Lateral acceleration control (55 lines)
├── mode_hold.cpp            # Stop/hold position (19 lines)
├── mode_loiter.cpp          # Loiter at location (81 lines)
├── mode_guided.cpp          # External navigation control (448 lines)
├── mode_auto.cpp            # Mission execution (1058 lines)
├── mode_rtl.cpp             # Return to launch (85 lines)
├── mode_smart_rtl.cpp       # Smart return via recorded path
├── mode_circle.cpp          # Circle navigation
├── mode_simple.cpp          # Simplified heading control
├── mode_follow.cpp          # Follow another vehicle
└── mode_dock.cpp            # Precision docking
```

### GCS/MAVLink Files

```
├── GCS_Rover.h              # GCS interface declaration (36 lines)
├── GCS_Rover.cpp            # GCS interface implementation (68 lines)
├── GCS_MAVLink_Rover.h      # MAVLink protocol declaration (85 lines)
└── GCS_MAVLink_Rover.cpp    # MAVLink handler (1074 lines)
```

### Parameter System

```
├── Parameters.h             # Parameter declarations (434 lines)
└── Parameters.cpp           # Parameter definitions (893 lines)
```

### System Files

```
├── system.cpp               # Initialization & mode switching (331 lines)
├── failsafe.cpp             # Failsafe handling (167 lines)
├── commands.cpp             # Mission command execution (66 lines)
├── sensors.cpp              # Sensor reading
├── radio.cpp                # RC input processing
├── Steering.cpp             # Servo output
└── Log.cpp                  # DataFlash logging
```

### Specialized Systems

```
├── AP_Arming_Rover.h/cpp    # Arming pre-checks (248 lines)
├── AP_Rally.h/cpp           # Rally point management
├── AP_ExternalControl_Rover.h/cpp  # External control API
├── RC_Channel_Rover.h/cpp   # RC channel mapping
├── balance_bot.cpp          # Balance bot pitch control
├── sailboat.h/cpp           # Sailboat-specific control
├── cruise_learn.cpp         # Cruise speed learning
├── crash_check.cpp          # Crash detection
├── ekf_check.cpp            # EKF health monitoring
├── fence.cpp                # Geofence handling
├── motor_test.cpp           # Motor testing
├── precision_landing.cpp    # Precision landing for docking
└── afs_rover.h/cpp          # Advanced failsafe
```

**Total: 50+ files, ~25,000 lines of code**

---

## Architecture

### Class Hierarchy

```
AP_Vehicle (base class in libraries/AP_Vehicle)
    ↓
Rover (main vehicle class)
    ↓
    ├── Mode (base class for all modes)
    │    ├── ModeInitializing
    │    ├── ModeManual
    │    ├── ModeAcro
    │    ├── ModeSteering
    │    ├── ModeHold
    │    ├── ModeLoiter
    │    ├── ModeGuided (6 submodes)
    │    ├── ModeAuto (8 submodes)
    │    ├── ModeRTL
    │    ├── ModeSmartRTL
    │    ├── ModeCircle
    │    ├── ModeSimple
    │    ├── ModeFollow
    │    └── ModeDock
    │
    ├── GCS_Rover (GCS manager)
    │    └── GCS_MAVLINK_Rover (MAVLink channel)
    │
    ├── AP_Arming_Rover (arming checks)
    ├── Sailboat (sailboat control)
    └── Parameters/ParametersG2 (configuration)
```

### Main Rover Class Structure

```cpp
class Rover : public AP_Vehicle {
public:
    Rover();

    // Main loop scheduler
    void loop();

private:
    // ========================================
    // PARAMETERS
    // ========================================
    AP_Param param_loader;
    Parameters g;              // Main parameters
    ParametersG2 g2;           // Extended parameters (G2)

    // ========================================
    // CORE SENSORS & STATE ESTIMATION
    // ========================================
    AP_AHRS ahrs;              // Attitude/heading reference
    AP_InertialSensor ins;     // IMU (gyros/accels)
    Compass compass;           // Magnetometer
    AP_GPS gps;                // GPS
    AP_Baro barometer;         // Barometer/altimeter
    RangeFinder rangefinder;   // Distance sensors
    AP_OpticalFlow optflow;    // Optical flow
    AP_WheelEncoder wheel_encoder; // Wheel speed sensors

    // ========================================
    // CONTROL SYSTEMS
    // ========================================
    AR_AttitudeControl attitude_control;  // Low-level controllers
    AR_WPNav_OA wp_nav;        // Waypoint navigation + obstacle avoidance
    AR_PosControl pos_control; // Position controller
    AP_MotorsUGV motors;       // Motor mixing & output

    // ========================================
    // NAVIGATION & MISSIONS
    // ========================================
    AP_Mission mission;        // Mission storage & execution
    AP_SmartRTL smart_rtl;     // Smart return-to-launch
    AP_Rally rally;            // Rally points

    // ========================================
    // MODES
    // ========================================
    Mode *control_mode;        // Pointer to current mode

    // Mode instances (all modes instantiated)
    ModeInitializing mode_initializing;
    ModeManual mode_manual;
    ModeAcro mode_acro;
    ModeSteering mode_steering;
    ModeHold mode_hold;
    ModeLoiter mode_loiter;
    ModeGuided mode_guided;
    ModeAuto mode_auto;
    ModeRTL mode_rtl;
    ModeSmartRTL mode_smartrtl;
    ModeCircle mode_circle;
    ModeSimple mode_simple;
    ModeFollow mode_follow;  // if enabled
    ModeDock mode_dock;      // if enabled

    // ========================================
    // GCS & TELEMETRY
    // ========================================
    GCS_Rover _gcs;
    GCS_Rover &gcs() { return _gcs; }

    // ========================================
    // RC INPUT
    // ========================================
    RC_Channel *channel_steer;
    RC_Channel *channel_throttle;
    RC_Channel *channel_lateral;     // For omni drives
    RC_Channel *channel_roll;        // For walking robots
    RC_Channel *channel_pitch;       // For walking robots
    RC_Channel *channel_walking_height;

    // ========================================
    // VEHICLE STATE
    // ========================================
    Location current_loc;      // Current position
    float ground_speed;        // Current speed (m/s)
    bool have_position;        // Valid position estimate
    bool initialised;          // Init complete

    // Failsafe state
    struct {
        uint8_t bits;          // Active failsafes
        uint32_t start_time;   // Failsafe start time
        uint8_t triggered;     // Triggered actions
        uint32_t last_valid_rc_ms; // Last RC input
        bool ekf;              // EKF failsafe
    } failsafe;

    // ========================================
    // SPECIALIZED SYSTEMS
    // ========================================
    AP_BattMonitor battery;    // Battery monitoring
    AP_Arming_Rover arming;    // Arming checks
    Sailboat sailboat;         // Sailboat control
    AP_WindVane windvane;      // Wind sensor
    AC_PrecLand precland;      // Precision landing
    AC_Fence fence;            // Geofence
    AP_Proximity proximity;    // Proximity sensors
    AP_Beacon beacon;          // Position beacons
    AP_Follow follow;          // Follow library
};

// Global instance
extern Rover rover;
```

### Initialization Sequence

**Rover startup flow (system.cpp):**

```cpp
void Rover::init_ardupilot() {
    // 1. Board initialization
    hal.board_config.init();

    // 2. Initialize notify (LEDs, buzzer)
    notify.init();

    // 3. Load parameters from EEPROM
    load_parameters();

    // 4. Initialize battery monitoring
    battery.init();

    // 5. Initialize serial manager & GCS
    AP::serialmanager().init();
    gcs().init();
    gcs().setup_console();
    gcs().setup_uarts();

    // 6. Initialize sensors
    compass.init();
    barometer.init();
    ins.init(scheduler.get_loop_rate_hz());
    ahrs.init();

    // 7. Initialize GPS
    gps.init();

    // 8. Initialize rangefinder
    rangefinder.init(ROTATION_NONE);

    // 9. Initialize proximity sensors
    proximity.init();

    // 10. Initialize optical flow
    optflow.init();

    // 11. Initialize wheel encoders
    wheel_encoder.init();

    // 12. Initialize RC input
    init_rc_in();

    // 13. Initialize motors & servos
    motors.init(get_frame_type());

    // 14. Initialize attitude control
    attitude_control.init();

    // 15. Initialize position control
    pos_control.init();

    // 16. Initialize waypoint navigation
    wp_nav.init();

    // 17. Initialize mission library
    mission.init();

    // 18. Initialize smart RTL
    smart_rtl.init();

    // 19. Initialize logger
    logger.init();

    // 20. Calibrate gyros
    startup_INS();

    // 21. Set initial mode
    Mode *initial_mode = mode_from_mode_num((Mode::Number)g.initial_mode.get());
    set_mode(*initial_mode, ModeReason::INITIALISED);

    // Mark as initialized
    initialised = true;
}
```

### Main Loop & Scheduler

**Rover uses a task scheduler running at 400 Hz:**

```cpp
// Main loop (Rover.cpp)
void Rover::loop() {
    scheduler.loop();  // Runs all scheduled tasks
}

// Scheduler tasks (runs at different rates)
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    // Task                     Rate(Hz)  Cost(µs)  Prio
    SCHED_TASK(read_radio,           50,     200,    3),  // RC input
    SCHED_TASK(ahrs_update,         400,     400,    6),  // AHRS/EKF
    SCHED_TASK(read_rangefinders,    50,     200,    9),  // Rangefinder
    SCHED_TASK(update_current_mode, 400,     200,   12),  // MODE UPDATE ← CRITICAL
    SCHED_TASK(set_servos,          400,     200,   15),  // Motor output
    SCHED_TASK(update_wheel_encoder, 50,     200,   18),  // Wheel encoders
    SCHED_TASK(update_compass,       10,     200,   39),  // Compass
    SCHED_TASK(update_logging1,      10,     200,   45),  // Logging
    SCHED_TASK(update_logging2,      10,     200,   48),  // Logging
    SCHED_TASK(gcs_retry_deferred,   50,     500,   51),  // GCS messages
    SCHED_TASK(gcs_update,          400,     750,   54),  // GCS send/receive
    SCHED_TASK(gcs_data_stream_send, 50,     750,   57),  // GCS streams
    SCHED_TASK(update_mount,         50,     200,   66),  // Camera mount
    SCHED_TASK(update_trigger,       50,     200,   69),  // Camera trigger
    SCHED_TASK(gcs_failsafe_check,   10,     200,   81),  // GCS failsafe
    SCHED_TASK(fence_check,          10,     200,   84),  // Geofence
    SCHED_TASK(ekf_check,            10,     100,   87),  // EKF health
    SCHED_TASK(crash_check,          10,     200,   90),  // Crash detection
    SCHED_TASK(cruise_learn_update,  50,     200,   93),  // Cruise learning
    SCHED_TASK(one_second_loop,       1,    1500,   96),  // 1Hz tasks
    // ... 30+ total tasks
};
```

**Key:**
- **Rate**: How often task runs (Hz)
- **Cost**: Typical execution time (microseconds)
- **Priority**: Lower number = higher priority

**Most critical tasks:**
1. `ahrs_update()` - 400 Hz - Sensor fusion
2. `update_current_mode()` - 400 Hz - Mode-specific control
3. `set_servos()` - 400 Hz - Motor/servo output
4. `gcs_update()` - 400 Hz - MAVLink communication

---

## All Dependencies

### From wscript

```python
ap_libraries = bld.ap_common_vehicle_libraries() + [
    # Rover-specific libraries
    'APM_Control',           # PID controllers
    'AP_Mount',              # Camera/antenna mount control
    'AP_Navigation',         # Navigation algorithms
    'AR_WPNav',              # Rover waypoint navigation
    'AP_AdvancedFailsafe',   # Advanced failsafe system
    'AP_WheelEncoder',       # Wheel speed sensors
    'AP_SmartRTL',           # Smart return-to-launch
    'AC_AttitudeControl',    # Attitude/rate controllers
    'AP_LTM_Telem',          # LTM telemetry protocol
    'AP_Devo_Telem',         # Devo telemetry protocol
    'AP_WindVane',           # Wind direction/speed sensor
    'AR_Motors',             # Rover motor control
    'AP_Torqeedo',           # Torqeedo motor driver
    'AC_PrecLand',           # Precision landing
    'AP_IRLock',             # IR beacon tracking
]
```

### Common Vehicle Libraries (inherited)

```python
ap_common_vehicle_libraries = [
    'AP_AHRS',               # Attitude/Heading Reference System
    'AP_Baro',               # Barometer
    'AP_BattMonitor',        # Battery monitoring
    'AP_BoardConfig',        # Board configuration
    'AP_Camera',             # Camera control
    'AP_Common',             # Common utilities
    'AP_Compass',            # Magnetometer
    'AP_GPS',                # GPS
    'AP_InertialSensor',     # IMU (gyros/accels)
    'AP_Logger',             # DataFlash logging
    'AP_Mission',            # Mission storage/execution
    'AP_Notify',             # LEDs, buzzer, display
    'AP_OpticalFlow',        # Optical flow sensor
    'AP_Param',              # Parameter system
    'AP_RangeFinder',        # Distance sensors
    'AP_RCMapper',           # RC input mapping
    'AP_RPM',                # RPM sensors
    'AP_Scheduler',          # Task scheduler
    'AP_SerialManager',      # Serial port management
    'AP_Vehicle',            # Vehicle base class
    'GCS_MAVLink',           # Ground station communication ← CRITICAL
    'StorageManager',        # EEPROM/parameter storage
    'AP_Arming',             # Arming checks base class
    'AC_Fence',              # Geofence
    'AP_Proximity',          # Proximity sensors
    'AP_Beacon',             # Position beacons
    'AP_Follow',             # Follow another vehicle
    'AP_OSD',                # On-screen display
    'AP_Frsky_Telem',        // FrSky telemetry
    // ... and many more
]
```

### Key Library Purposes

#### AR_WPNav (Rover Waypoint Navigation)
**Location:** `libraries/AR_WPNav/`

**Purpose:** S-curve path planning with obstacle avoidance

```cpp
class AR_WPNav_OA {
    // Set destination waypoint
    bool set_desired_location(const Location &destination,
                               Location next_destination = Location());

    // Update path planning (call at 50Hz+)
    void update(float dt);

    // Get outputs
    float get_turn_rate_rads();      // Desired turn rate
    float get_speed_target();         // Desired speed
    float get_distance_to_destination();
    float get_bearing_cd();           // Bearing to target
    float get_lat_accel();            // Lateral acceleration

    // Status
    bool reached_destination();
    bool is_fast_waypoint();         // High-speed waypoint
};
```

**Features:**
- S-curve path smoothing for comfort
- Lookahead distance calculation
- Turn radius management
- Speed reduction in curves
- Object avoidance integration
- Pivot turn detection

#### AR_Motors (Rover Motor Control)
**Location:** `libraries/AR_Motors/`

**Purpose:** Motor mixing for all vehicle types

```cpp
class AP_MotorsUGV {
    // Supported vehicle types
    enum motor_frame_type {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_ROVER = 1,       // Steering + throttle
        MOTOR_FRAME_BOAT = 2,         // Boats
        MOTOR_FRAME_BALANCEBOT = 3,   // Balance bots
        MOTOR_FRAME_OMNI3 = 4,        // 3-wheel omni
        MOTOR_FRAME_OMNIX = 5,        // X-configuration omni
        MOTOR_FRAME_OMNIPLUS = 6,     // Plus-configuration omni
        MOTOR_FRAME_SKID = 7,         // Skid steering (differential)
        MOTOR_FRAME_MECANUM = 8,      // Mecanum wheels
    };

    // Set control outputs
    void set_steering(float steering);    // -4500 to +4500
    void set_throttle(float throttle);    // -100 to +100
    void set_lateral(float lateral);      // For omni/mecanum
    void set_roll(float roll);            // For walking robots
    void set_pitch(float pitch);          // For walking robots
    void set_walking_height(float height);// For walking robots
    void set_mainsail(float angle);       // For sailboats
    void set_wingsail(float angle);       // For sailboats

    // Output to servos/ESCs
    void output();
};
```

#### AR_AttitudeControl (Rover Attitude Control)
**Location:** `libraries/AC_AttitudeControl/`

**Purpose:** Low-level control loops

```cpp
class AR_AttitudeControl {
    // Steering rate controller
    float get_steering_out_rate(
        float desired_rate_rads,  // Desired turn rate (rad/s)
        bool limit_left,
        bool limit_right,
        float dt
    );

    // Heading controller
    float get_steering_out_heading(
        float desired_heading_cd,  // Desired heading (centidegrees)
        float desired_rate,        // Optional rate limit
        bool limit_left,
        bool limit_right,
        float dt
    );

    // Speed controller
    float get_throttle_out_speed(
        float desired_speed,       // Desired speed (m/s)
        bool limit_low,
        bool limit_high,
        float cruise_speed,
        float cruise_throttle,
        float dt
    );

    // Lateral acceleration controller (for steering mode)
    float get_steering_out_lat_accel(
        float desired_lat_accel,   // Desired lateral accel (m/s²)
        bool limit_left,
        bool limit_right,
        float dt
    );

    // Balance bot pitch controller
    float get_pitch_to_throttle(
        float desired_pitch,       // Desired pitch (radians)
        float dt
    );

    // Sailboat heel angle controller
    float get_sailboat_heel_pid(
        float desired_heel,        // Desired heel angle
        float dt
    );
};
```

#### AP_SmartRTL (Smart Return-to-Launch)
**Location:** `libraries/AP_SmartRTL/`

**Purpose:** Record path and return along it

```cpp
class AP_SmartRTL {
    // Save current position to path
    void update(bool position_ok, const Location &current_loc);

    // Request return path
    bool request_thorough_cleanup();

    // Get next point on return path
    bool pop_point(Location &point);

    // Path statistics
    uint16_t get_num_points() const;
    float get_max_path_distance_m() const;
};
```

**How it works:**
1. Continuously saves positions as vehicle moves
2. Removes redundant points (simplification)
3. On RTL, provides path in reverse
4. Safer than direct return (avoids obstacles already navigated)

---

## Mode System

### All 13+ Modes

| Mode | Number | Description | Auto | Pos Req | Vel Req |
|------|--------|-------------|------|---------|---------|
| **MANUAL** | 0 | Direct RC control, no stabilization | No | No | No |
| **ACRO** | 1 | Turn rate + speed control | No | No | Yes* |
| **STEERING** | 3 | Lateral acceleration control | No | No | Yes |
| **HOLD** | 4 | Stop vehicle / loiter for boats | No | No | No |
| **LOITER** | 5 | Stay near current position | No | Yes | Yes |
| **FOLLOW** | 6 | Follow another vehicle | No | Yes | Yes |
| **SIMPLE** | 7 | Simplified heading-based control | No | Yes | Yes |
| **DOCK** | 8 | Precision docking using beacon | Yes | Yes | Yes |
| **CIRCLE** | 9 | Circle around a point | Yes | Yes | Yes |
| **AUTO** | 10 | Execute mission waypoints | Yes | Yes | Yes |
| **RTL** | 11 | Return to launch point | Yes | Yes | Yes |
| **SMART_RTL** | 12 | Return via recorded path | Yes | Yes | Yes |
| **GUIDED** | 15 | External control (GCS/companion) | Yes | Yes | Yes |
| **INITIALISING** | 16 | Boot/calibration only | No | No | No |

*Acro requires velocity for non-skid-steering vehicles

### Mode Base Class

```cpp
class Mode {
public:
    // Mode identification
    virtual Number mode_number() const = 0;
    virtual const char *name4() const = 0;  // 4-character name

    // Main update function (called at 400 Hz)
    virtual void update() = 0;

    // Entry/exit
    bool enter();                    // Public entry point
    void exit();                     // Public exit point

    // Capabilities
    virtual bool is_autopilot_mode() const { return false; }
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }
    virtual bool allows_arming() const { return true; }
    virtual bool has_manual_input() const { return false; }
    virtual bool attitude_stabilized() const { return true; }

    // Navigation status (for NAV_CONTROLLER_OUTPUT)
    virtual float wp_bearing() const;
    virtual float nav_bearing() const;
    virtual float crosstrack_error() const;
    virtual float get_desired_lat_accel() const;

    // Destination management
    virtual float get_distance_to_destination() const { return 0.0f; }
    virtual bool get_desired_location(Location &dest) const { return false; }
    virtual bool set_desired_location(const Location &dest) { return false; }
    virtual bool reached_destination() const { return true; }
    virtual bool set_desired_speed(float speed) { return false; }

protected:
    // Subclass hooks
    virtual bool _enter() { return true; }   // Override for init
    virtual void _exit() { return; }          // Override for cleanup

    // Helper functions for modes
    void get_pilot_desired_steering_and_throttle(float &steer, float &thr);
    void get_pilot_desired_steering_and_speed(float &steer, float &speed);
    void navigate_to_waypoint();
    void calc_steering_to_heading(float heading_cd, float rate_max = 0);
    void calc_throttle(float target_speed, bool avoidance_enabled);
    bool stop_vehicle();

    // References to vehicle systems (convenience)
    AP_AHRS &ahrs;
    Parameters &g;
    ParametersG2 &g2;
    RC_Channel *&channel_steer;
    RC_Channel *&channel_throttle;
    AR_AttitudeControl &attitude_control;
};
```

### Mode Descriptions

#### MANUAL Mode
**Direct pass-through of pilot inputs, no stabilization**

```cpp
void ModeManual::update() {
    // Read pilot input
    float steering, throttle;
    get_pilot_input(steering, throttle);

    // Direct output to motors (no processing)
    g2.motors.set_steering(steering);
    g2.motors.set_throttle(throttle);
}
```

**Use case:** Basic RC vehicle control, testing, manual override

#### ACRO Mode
**Turn rate and speed control**

```cpp
void ModeAcro::update() {
    float steering, throttle;
    get_pilot_input(steering, throttle);

    // Convert steering to turn rate
    float desired_turn_rate = steering * g.acro_turn_rate;

    // Use rate controller
    float steering_out = attitude_control.get_steering_out_rate(
        desired_turn_rate, true, true, dt);

    // Use speed controller
    float speed = throttle * g.speed_cruise;
    float throttle_out = attitude_control.get_throttle_out_speed(
        speed, true, true, g.speed_cruise, g.throttle_cruise, dt);

    g2.motors.set_steering(steering_out);
    g2.motors.set_throttle(throttle_out);
}
```

**Use case:** Sport driving, precise turn rate control

#### AUTO Mode
**Mission execution with full navigation**

```cpp
void ModeAuto::update() {
    switch (_submode) {
    case SubMode::WP:
        // Navigate to waypoint using WPNav
        navigate_to_waypoint();
        break;

    case SubMode::RTL:
        // Return to launch (within Auto)
        do_RTL();
        break;

    case SubMode::Loiter:
        // Loiter at current location
        stop_vehicle();
        break;

    case SubMode::Guided:
        // External control within Auto
        // Controlled by companion computer
        break;

    case SubMode::NavScriptTime:
        // Lua script control
        // Scripts provide targets
        break;
    }

    // Check mission completion
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        if (verify_command(mission.get_current_nav_cmd())) {
            mission.advance_current_nav_cmd();
        }
    }
}
```

**Submodes:**
- `WP` - Navigate to waypoint
- `HeadingAndSpeed` - Drive at heading/speed
- `RTL` - Return within Auto
- `Loiter` - Loiter within Auto
- `Guided` - External control within Auto
- `Stop` - Stop vehicle
- `NavScriptTime` - Lua script control
- `Circle` - Circle navigation

**Use case:** Autonomous missions, waypoint following, surveys

#### GUIDED Mode
**External navigation control (GCS or companion computer)**

```cpp
void ModeGuided::update() {
    switch (_guided_mode) {
    case SubMode::WP:
        // Navigate to commanded waypoint
        navigate_to_waypoint();
        break;

    case SubMode::HeadingAndSpeed:
        // Drive at commanded heading and speed
        calc_steering_to_heading(_desired_yaw_cd);
        calc_throttle(_desired_speed, true);
        break;

    case SubMode::TurnRateAndSpeed:
        // Turn at rate while driving at speed
        float steering = attitude_control.get_steering_out_rate(
            _desired_yaw_rate_cds * 0.01f, true, true, dt);
        calc_throttle(_desired_speed, true);
        break;

    case SubMode::SteeringAndThrottle:
        // Direct steering/throttle (for scripts)
        g2.motors.set_steering(_strthr_steering * 4500);
        g2.motors.set_throttle(_strthr_throttle * 100);
        break;

    case SubMode::Loiter:
        // Loiter at location
        navigate_to_waypoint();
        break;

    case SubMode::Stop:
        // Stop vehicle
        stop_vehicle();
        break;
    }

    // Check guided limits (timeout, distance)
    if (limit_breached()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Guided limit breached");
        set_mode(rover.mode_hold, ModeReason::GUIDED_TIMEOUT);
    }
}
```

**Submodes:**
- `WP` - Go to waypoint
- `HeadingAndSpeed` - Drive at heading + speed
- `TurnRateAndSpeed` - Turn rate + speed
- `Loiter` - Loiter at position
- `SteeringAndThrottle` - Direct control (scripts)
- `Stop` - Stop

**Use case:** Companion computer control, dynamic missions, teleoperation

#### RTL Mode
**Return to launch point**

```cpp
void ModeRTL::update() {
    if (!_loitering) {
        // Navigate back to home
        navigate_to_waypoint();

        // Check if reached home
        if (reached_destination()) {
            _loitering = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Reached home");
        }
    } else {
        // Loiter at home
        stop_vehicle();
    }
}
```

**Use case:** Return home on low battery, failsafe, or command

#### SMART_RTL Mode
**Return via recorded path**

```cpp
void ModeSmartRTL::update() {
    switch (smart_rtl_state) {
    case SmartRTLState::PathFollow:
        // Follow recorded path in reverse
        navigate_to_waypoint();

        if (reached_destination()) {
            // Load next point
            Location next_point;
            if (!g2.smart_rtl.pop_point(next_point)) {
                // Path complete, switch to regular RTL
                smart_rtl_state = SmartRTLState::StopAtHome;
            } else {
                set_desired_location(next_point);
            }
        }
        break;

    case SmartRTLState::StopAtHome:
        stop_vehicle();
        break;

    case SmartRTLState::Failure:
        // Fall back to regular RTL
        set_mode(rover.mode_rtl, ModeReason::SMARTRTL_FAILURE);
        break;
    }
}
```

**Use case:** Safer return (follows known-good path)

---

## GCS/MAVLink Integration

### GCS Classes

```cpp
class GCS_Rover : public GCS {
    // Channel access
    GCS_MAVLINK_Rover *chan(uint8_t ofs) override;

    // Vehicle info
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;
    bool vehicle_initialised() const override;

    // Sensor status
    void update_vehicle_sensor_status_flags() override;

    // Simple mode (for Copter compatibility)
    bool simple_input_active() const override;

protected:
    // Create channel instance
    GCS_MAVLINK_Rover *new_gcs_mavlink_backend(
        AP_HAL::UARTDriver &uart) override;
};
```

```cpp
class GCS_MAVLINK_Rover : public GCS_MAVLINK {
public:
    using GCS_MAVLINK::GCS_MAVLINK;

protected:
    // Required pure virtuals
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

    // Message handling
    void handle_message(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;

    // Command handling
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;

    // Rover-specific messages
    void send_rangefinder() const override;
    void send_water_depth();
    void send_position_target_global_int() override;

    // Mode reporting
    uint8_send_available_mode(uint8_t index) const override;
};
```

### MAVLink Messages Sent

**Periodic telemetry (sent automatically):**

```cpp
bool GCS_MAVLINK_Rover::try_send_message(enum ap_message id) {
    switch (id) {
    case MSG_HEARTBEAT:
        send_heartbeat();
        return true;

    case MSG_SYS_STATUS:
        send_sys_status();
        return true;

    case MSG_ATTITUDE:
        send_attitude();
        return true;

    case MSG_LOCATION:
        send_location();
        return true;

    case MSG_NAV_CONTROLLER_OUTPUT:
        send_nav_controller_output();
        return true;

    case MSG_POSITION_TARGET_GLOBAL_INT:
        send_position_target_global_int();
        return true;

    case MSG_RANGEFINDER:
        send_rangefinder();
        return true;

    case MSG_RC_CHANNELS:
        send_rc_channels();
        return true;

    case MSG_SERVO_OUTPUT_RAW:
        send_servo_output_raw();
        return true;

    case MSG_VFR_HUD:
        send_vfr_hud();
        return true;

    case MSG_WIND:
        send_wind();  // For sailboats
        return true;

    case MSG_PID_TUNING:
        send_pid_tuning();
        return true;

    // ... many more
    }
}
```

**Key messages:**

1. **HEARTBEAT** - Vehicle status
```cpp
void GCS_MAVLINK_Rover::send_heartbeat() {
    mavlink_msg_heartbeat_send(
        chan,
        gcs().frame_type(),      // MAV_TYPE_GROUND_ROVER or SURFACE_BOAT
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode(),
        rover.control_mode->mode_number(),
        vehicle_system_status()
    );
}
```

2. **NAV_CONTROLLER_OUTPUT** - Navigation status
```cpp
void GCS_MAVLINK_Rover::send_nav_controller_output() const {
    const Mode *mode = rover.control_mode;

    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // roll (not used for rovers)
        0,  // pitch (not used for rovers)
        mode->nav_bearing(),
        mode->wp_bearing(),
        MIN(mode->get_distance_to_destination(), UINT16_MAX),
        0,  // alt error (not used)
        mode->speed_error(),
        mode->crosstrack_error()
    );
}
```

3. **RC_CHANNELS_SCALED** - Servo outputs (steering/throttle)
```cpp
void GCS_MAVLINK_Rover::send_servo_out() {
    mavlink_msg_rc_channels_scaled_send(
        chan,
        AP_HAL::millis(),
        0,  // port (always 0)
        g2.motors.get_steering(),  // channel 1
        g2.motors.get_throttle(),  // channel 2
        g2.motors.get_lateral(),   // channel 3 (omni)
        0, 0, 0, 0, 0,  // unused channels
        rover.get_rssi()
    );
}
```

4. **WHEEL_DISTANCE** - Wheel encoder data
```cpp
void Rover::send_wheel_encoder_distance(mavlink_channel_t chan) {
    mavlink_msg_wheel_distance_send(
        chan,
        AP_HAL::micros64(),
        g2.wheel_encoder.num_sensors(),
        {wheel_encoder_last_distance_m[0],
         wheel_encoder_last_distance_m[1], ...}
    );
}
```

5. **WATER_DEPTH** - For boats with downward rangefinder
```cpp
void GCS_MAVLINK_Rover::send_water_depth() {
    RangeFinder *rf = AP::rangefinder();
    float distance = rf->distance_orient(ROTATION_PITCH_270);
    float temperature = rf->temperature_orient(ROTATION_PITCH_270);

    mavlink_msg_water_depth_send(
        chan,
        AP_HAL::micros64(),
        0,  // id
        distance,
        temperature
    );
}
```

### MAVLink Messages Received

```cpp
void GCS_MAVLINK_Rover::handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_position_target_local_ned(msg);
        break;

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
        handle_set_position_target_global_int(msg);
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_set_attitude_target(msg);
        break;

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        handle_manual_control(msg);
        break;

    case MAVLINK_MSG_ID_RADIO:
        handle_radio(msg);
        break;

    case MAVLINK_MSG_ID_LANDING_TARGET:
        handle_landing_target(msg);
        break;

    default:
        GCS_MAVLINK::handle_message(msg);  // CRITICAL: Call base
        break;
    }
}
```

**Key received messages:**

1. **SET_POSITION_TARGET_GLOBAL_INT** - Set waypoint
```cpp
void GCS_MAVLINK_Rover::handle_set_position_target_global_int(
    const mavlink_message_t &msg) {

    mavlink_set_position_target_global_int_t packet;
    mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

    // Check if position is valid
    if (!(packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE)) {
        Location target(packet.lat_int, packet.lon_int, packet.alt,
                       Location::AltFrame::ABSOLUTE);

        // Switch to GUIDED if not already
        if (rover.control_mode != &rover.mode_guided) {
            rover.set_mode(rover.mode_guided, ModeReason::GCS_COMMAND);
        }

        // Send to guided mode
        rover.mode_guided.set_desired_location(target);
    }
}
```

2. **SET_ATTITUDE_TARGET** - Set heading/speed
```cpp
void GCS_MAVLINK_Rover::handle_set_attitude_target(
    const mavlink_message_t &msg) {

    mavlink_set_attitude_target_t packet;
    mavlink_msg_set_attitude_target_decode(&msg, &packet);

    // Extract yaw and speed
    float yaw = degrees(packet.q[0]);  // Simplified
    float speed = packet.thrust * g.speed_cruise;

    // Switch to GUIDED
    if (rover.control_mode != &rover.mode_guided) {
        rover.set_mode(rover.mode_guided, ModeReason::GCS_COMMAND);
    }

    // Command heading and speed
    rover.mode_guided.set_desired_heading_and_speed(yaw * 100, speed);
}
```

### MAVLink Commands

```cpp
MAV_RESULT GCS_MAVLINK_Rover::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg) {

    switch (packet.command) {
    case MAV_CMD_DO_CHANGE_SPEED:
        // Change target speed
        rover.control_mode->set_desired_speed(packet.param2);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        rover.set_mode(rover.mode_rtl, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_SET_REVERSE:
        rover.control_mode->set_reversed(packet.param1 == 1);
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_DO_MOTOR_TEST:
        // Motor test implementation
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        return handle_command_nav_set_yaw_speed(packet, msg);

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}
```

---

## Key Systems

### Navigation System (AR_WPNav)

**S-curve path planning with lookahead:**

```
Current Position
      │
      ├─────► Lookahead Point (calculated based on speed)
      │              │
      │              ├─────► Path to Destination
      │              │              │
      │              │              │
      ▼              ▼              ▼
    ●──────────────●──────────────●
  (Now)      (Lookahead)     (Destination)
```

**Speed management in turns:**
- Reduces speed based on turn radius
- Prevents skidding/tipping
- Configurable via `WP_SPEED` and `WP_RADIUS`

### Motor Control (AR_Motors)

**Skid Steering (differential):**
```
Left Motor:  throttle - steering
Right Motor: throttle + steering
```

**Regular (steering servo):**
```
Steering Servo: steering angle
Throttle ESC:   throttle
```

**Omni Drives:**
```
3-Wheel Omni:
  Motor 1: forward + rotation
  Motor 2: (forward - lateral) + rotation
  Motor 3: (forward + lateral) + rotation
```

### Attitude Control Loops

**Steering → Rate → Motors:**
```
Desired Heading
      ↓
 Heading Controller (P)
      ↓
  Desired Turn Rate
      ↓
Rate Controller (PID)
      ↓
  Steering Output
      ↓
    Motors
```

**Speed Control:**
```
Desired Speed
      ↓
Speed Controller (PID)
      ↓
  Throttle Output
      ↓
    Motors
```

### Failsafe System

**Failsafe Priority (highest to lowest):**
1. Terminate
2. Hold
3. RTL
4. SmartRTL+Hold
5. SmartRTL
6. None

**Failsafe Flow:**
```
Condition Detected
      ↓
 Wait FS_TIMEOUT (1.5s default)
      ↓
Still Active?
   ↓ Yes       No ↓
Execute        Clear
FS_ACTION    Failsafe
```

**Types:**
- RC Loss (`FAILSAFE_EVENT_THROTTLE`)
- GCS Loss (`FAILSAFE_EVENT_GCS`)
- EKF Error
- Battery Low/Critical
- Fence Breach
- Crash Detection

---

## MiniVehicle vs Rover Comparison

### Feature Matrix

| Feature | MiniVehicle | Rover |
|---------|-------------|-------|
| **Modes** | 1 (passthrough) | 13+ (full autonomy) |
| **Navigation** | None | AR_WPNav (S-curves, OA) |
| **Mission Support** | None | Full (20+ commands) |
| **Motor Control** | Direct | AR_Motors (6+ vehicle types) |
| **Control Loops** | None | 10+ PID controllers |
| **Failsafes** | Basic | 8+ types with actions |
| **GCS Messages** | 1 (HEARTBEAT) | 50+ messages |
| **Logging** | None | Comprehensive DataFlash |
| **Parameters** | Minimal | 200+ parameters |
| **Arming Checks** | None | 20+ pre-arm checks |
| **Sensors** | IMU only | GPS, compass, rangefinder, encoders, etc. |
| **Source Files** | 5 | 50+ |
| **Lines of Code** | ~500 | ~25,000+ |

### Code Complexity Comparison

**MiniVehicle Loop:**
```cpp
void MiniVehicle::loop() {
    uint32_t now = AP_HAL::millis();

    if (now - _fast_loop_timer >= 20) {  // 50 Hz
        fast_loop();
        _fast_loop_timer = now;
    }

    if (now - _slow_loop_timer >= 100) {  // 10 Hz
        slow_loop();
        _slow_loop_timer = now;
    }

    update_gcs();  // GCS updates
}

void MiniVehicle::fast_loop() {
    ahrs.update();  // That's it!
}
```

**Rover Loop:**
```cpp
void Rover::loop() {
    scheduler.loop();  // Runs 30+ tasks at various rates
}

// Sample of tasks:
// - read_radio() @ 50 Hz
// - ahrs_update() @ 400 Hz
// - update_current_mode() @ 400 Hz  ← Calls mode's update()
// - set_servos() @ 400 Hz
// - gcs_update() @ 400 Hz
// - fence_check() @ 10 Hz
// - ekf_check() @ 10 Hz
// - crash_check() @ 10 Hz
// - ... 25+ more tasks
```

### Architecture Comparison

**MiniVehicle:**
```
MiniVehicle
  ├── AHRS
  ├── GCS (minimal)
  └── Simple loops
```

**Rover:**
```
Rover
  ├── Core Sensors
  │     ├── AHRS (IMU + GPS + Compass fusion)
  │     ├── GPS
  │     ├── Compass
  │     ├── Barometer
  │     ├── RangeFinder
  │     ├── Optical Flow
  │     ├── Wheel Encoders
  │     └── Proximity Sensors
  │
  ├── Navigation Stack
  │     ├── AR_WPNav (S-curve planning)
  │     ├── AR_PosControl (position control)
  │     ├── AP_SmartRTL (path recording)
  │     └── Object Avoidance
  │
  ├── Control Stack
  │     ├── AR_AttitudeControl
  │     │     ├── Steering rate PID
  │     │     ├── Heading PID
  │     │     ├── Speed PID
  │     │     ├── Lateral accel PID
  │     │     ├── Pitch PID (balance bots)
  │     │     └── Heel PID (sailboats)
  │     └── AR_Motors
  │           ├── Regular (steering + throttle)
  │           ├── Skid steering
  │           ├── Omni drives (3, X, +, mecanum)
  │           ├── Balance bots
  │           ├── Walking robots
  │           └── Sailboats
  │
  ├── Mode System (13+ modes)
  │     ├── Manual modes (Manual, Acro, Steering, Hold)
  │     ├── Assisted modes (Loiter, Simple, Follow)
  │     ├── Autonomous modes (Auto, Guided, RTL, SmartRTL)
  │     └── Special modes (Circle, Dock)
  │
  ├── Mission System
  │     ├── AP_Mission (storage)
  │     ├── Command execution
  │     └── Conditional logic
  │
  ├── Safety Systems
  │     ├── Failsafe (8+ types)
  │     ├── Geofence
  │     ├── Crash detection
  │     ├── EKF monitoring
  │     └── Arming checks (20+)
  │
  ├── GCS Integration
  │     ├── GCS_Rover (manager)
  │     ├── GCS_MAVLINK_Rover (protocol)
  │     ├── 50+ message types
  │     ├── Parameter management (200+)
  │     └── Telemetry streams
  │
  ├── Logging System
  │     ├── DataFlash logger
  │     └── 20+ message types
  │
  └── Specialized Features
        ├── Sailboat control (WindVane, sail control)
        ├── Balance bot (pitch stabilization)
        ├── Precision landing (IR beacon)
        ├── Follow mode (ADSB/MAVLink)
        └── Scripting interface (Lua)
```

---

## Learning Path

### From MiniVehicle to Rover

**You've already learned** (via MiniVehicle):
1. ✅ Basic vehicle structure
2. ✅ Inheriting from AP_Vehicle
3. ✅ GCS initialization (init, setup_console, setup_uarts)
4. ✅ GCS update loop (update_receive, update_send)
5. ✅ Simple scheduler pattern
6. ✅ HEARTBEAT implementation

**Next steps to understand Rover:**

#### Step 1: Study Mode System
**Files:** `mode.h`, `mode.cpp`, `mode_manual.cpp`

**Focus:**
- How Mode base class works
- Entry/exit hooks (`_enter()`, `_exit()`)
- Update pattern (400 Hz)
- Mode switching in `system.cpp`

#### Step 2: Add Navigation
**Files:** `mode_auto.cpp`, study AR_WPNav library

**Focus:**
- How waypoints are followed
- S-curve path planning
- Lookahead calculation
- `navigate_to_waypoint()` helper

#### Step 3: Understand Control Loops
**Files:** `mode_acro.cpp`, `mode_steering.cpp`, study AR_AttitudeControl

**Focus:**
- Steering rate controller
- Heading controller
- Speed controller
- PID tuning

#### Step 4: Explore Motor Control
**Files:** Study AR_Motors library, `Steering.cpp`

**Focus:**
- Different drive types
- Motor mixing
- Servo output timing

#### Step 5: Master GCS Integration
**Files:** `GCS_MAVLink_Rover.cpp`

**Focus:**
- All message types (compare to MiniVehicle's HEARTBEAT)
- Command handling
- Parameter management
- Telemetry streams

#### Step 6: Safety Systems
**Files:** `failsafe.cpp`, `crash_check.cpp`, `fence.cpp`, `ekf_check.cpp`

**Focus:**
- Failsafe triggers
- Failsafe actions
- Priority system
- Recovery logic

#### Step 7: Mission System
**Files:** `mode_auto.cpp`, `commands.cpp`, study AP_Mission library

**Focus:**
- Mission storage
- Command types
- Conditional commands
- Mission flow control

### Recommended Reading Order

1. **Start:** MiniVehicle (you've done this!)
2. **Next:** Rover/mode.h - Understand mode architecture
3. **Then:** Rover/mode_manual.cpp - Simplest mode
4. **Then:** Rover/mode_acro.cpp - Add control loops
5. **Then:** Rover/mode_guided.cpp - External control
6. **Then:** Rover/mode_auto.cpp - Full autonomy
7. **Then:** Rover/GCS_MAVLink_Rover.cpp - Complete GCS
8. **Finally:** Libraries (AR_WPNav, AR_Motors, AR_AttitudeControl)

### Key Concepts to Master

1. **Mode-Based Architecture**
   - Each mode is a state machine
   - Modes have entry/exit hooks
   - Modes update at 400 Hz
   - Mode switching is controlled

2. **Control Loop Hierarchy**
   - High-level (modes) → Mid-level (navigation) → Low-level (PIDs) → Motors
   - Each layer has specific responsibility
   - Clear separation of concerns

3. **Scheduler Pattern**
   - Tasks run at different rates
   - Priority-based execution
   - Budget monitoring (execution time)

4. **Safety First**
   - Multiple failsafe layers
   - Pre-arm checks
   - Runtime monitoring
   - Graceful degradation

5. **GCS as Partner**
   - Bidirectional communication
   - Parameter management
   - Mission upload/download
   - Real-time telemetry

---

## Summary

**Rover is a production-grade autonomous vehicle firmware** with:

✅ **13+ modes** from manual to full autonomy
✅ **Complete navigation stack** with S-curves and obstacle avoidance
✅ **Advanced control** supporting 6+ vehicle types
✅ **Comprehensive safety** with 8+ failsafe types
✅ **Professional GCS** with 50+ MAVLink messages
✅ **Mission support** with 20+ command types
✅ **Extensive logging** for analysis and tuning
✅ **200+ parameters** for complete customization

**From MiniVehicle to Rover** you go from:
- 500 lines → 25,000 lines
- 1 mode → 13+ modes
- 1 message → 50+ messages
- Basic structure → Production system

**But the architecture is the same!** Both:
- Inherit from AP_Vehicle
- Use GCS integration (init, setup, update)
- Follow ArduPilot patterns
- Scale with the same design

MiniVehicle teaches the foundation. Rover shows what you can build on it!

---

**For more details, see:**
- ArduPilot Dev Wiki: https://ardupilot.org/dev/
- Rover code: `/Rover/` directory
- Libraries: `/libraries/AR_*` and `/libraries/AP_*`
- Your MiniVehicle: `/GCS_MAVLink_Pro/MiniVehicle/`
