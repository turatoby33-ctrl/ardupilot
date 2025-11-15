# AP_Arming Library - Comprehensive Analysis Report

## Executive Summary

The AP_Arming library is a critical safety system in ArduPilot that manages vehicle arming/disarming state and enforces comprehensive pre-arm checks to ensure the vehicle is in a safe state before flight. The system uses a modular check architecture with 21 different check categories, supports multiple arming methods, and allows vehicle-specific implementations through inheritance.

---

## 1. Main Arming Classes and Architecture

### 1.1 Base Class: AP_Arming

**Location:** `/home/user/ardupilot/libraries/AP_Arming/`

**Key Files:**
- `AP_Arming.h` - Header file with class definition
- `AP_Arming.cpp` - Implementation with 70+ arming check functions
- `AP_Arming_config.h` - Build configuration

**Core Class Structure:**
```cpp
class AP_Arming {
public:
    // Singleton instance management
    static AP_Arming *get_singleton();
    
    // Main arming operations
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks=true);
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks=true);
    virtual bool arm_force(AP_Arming::Method method) { return arm(method, false); }
    
    // Check functions
    virtual bool pre_arm_checks(bool report);
    virtual bool arm_checks(AP_Arming::Method method);
    
    // State queries
    bool is_armed() const;
    bool is_armed_and_safety_off() const;
    uint64_t arm_time_us() const;  // Returns 0 if disarmed
    
    // Configuration and parameters
    AP_Enum<Required> require;
    AP_Int32 checks_to_perform;
    AP_Int32 _arming_options;
};
```

### 1.2 Arming Check Categories (21 Total)

The library defines 21 distinct check categories using an enum:

```cpp
enum class Check {
    ALL         = (1U << 0),    // Check all items
    BARO        = (1U << 1),    // Barometer checks
    COMPASS     = (1U << 2),    // Compass/Magnetometer checks
    GPS         = (1U << 3),    // GPS positioning checks
    INS         = (1U << 4),    // Inertial Measurement Unit (IMU) checks
    PARAMETERS  = (1U << 5),    // Parameter validation
    RC          = (1U << 6),    // Radio Control checks
    VOLTAGE     = (1U << 7),    // Board voltage checks
    BATTERY     = (1U << 8),    // Battery/Power checks
    AIRSPEED    = (1U << 9),    // Airspeed sensor checks
    LOGGING     = (1U << 10),   // SD card logging availability
    SWITCH      = (1U << 11),   // Hardware safety switch
    GPS_CONFIG  = (1U << 12),   // GPS configuration
    SYSTEM      = (1U << 13),   // System health (storage, loop rate, etc.)
    MISSION     = (1U << 14),   // Mission feasibility
    RANGEFINDER = (1U << 15),   // Rangefinder sensor checks
    CAMERA      = (1U << 16),   // Camera/RunCam checks
    AUX_AUTH    = (1U << 17),   // Auxiliary authorization (scripting)
    VISION      = (1U << 18),   // Visual odometry checks
    FFT         = (1U << 19),   // Gyro FFT vibration analysis
    OSD         = (1U << 20),   // On-Screen Display checks
};
```

### 1.3 Arming Methods (40+ Methods)

The library supports multiple arming methods for different triggering mechanisms:

```cpp
enum class Method {
    RUDDER = 0,                     // Rudder stick arming (Plane)
    MAVLINK = 1,                    // GCS/MAVLink arming
    AUXSWITCH = 2,                  // Auxiliary switch arming
    MOTORTEST = 3,                  // Motor test mode
    SCRIPTING = 4,                  // Scripting-initiated arming
    
    // Disarm-only methods (automatic disarming)
    TERMINATION = 5,                // Safety termination
    CPUFAILSAFE = 6,                // CPU failsafe triggered
    BATTERYFAILSAFE = 7,            // Battery failsafe
    SOLOPAUSEWHENLANDED = 8,        // Solo pause when landed
    AFS = 9,                         // Advanced Failsafe System
    ADSBCOLLISIONACTION = 10,        // ADSB collision avoidance
    PARACHUTE_RELEASE = 11,          // Parachute deployment
    CRASH = 12,                      // Crash detection
    LANDED = 13,                     // Auto-disarm on landing
    MISSIONEXIT = 14,                // Mission exit
    FENCEBREACH = 15,                // Geofence breach
    RADIOFAILSAFE = 16,              // Radio/RC failsafe
    DISARMDELAY = 17,                // Disarm delay timer
    GCSFAILSAFE = 18,                // GCS failsafe
    TERRAINFAILSAFE = 19,            // Terrain failsafe
    FAILSAFE_ACTION_TERMINATE = 20,  // Generic failsafe action
    TERRAINFAILSAFE = 21,            // Terrain following failsafe
    MOTORDETECTDONE = 22,            // Motor detection complete
    BADFLOWOFCONTROL = 23,           // Safety check - bad control flow
    EKFFAILSAFE = 24,                // EKF navigation failsafe
    GCS_FAILSAFE_SURFACEFAILED = 25, // GCS surface command failed
    GCS_FAILSAFE_HOLDFAILED = 26,    // GCS hold command failed
    TAKEOFFTIMEOUT = 27,             // Takeoff timeout
    AUTOLANDED = 28,                 // Auto-landing completion
    PILOT_INPUT_FAILSAFE = 29,       // Pilot input loss
    TOYMODELANDTHROTTLE = 30,        // Toy mode throttle
    TOYMODELANDFORCE = 31,           // Toy mode force
    LANDING = 32,                    // Landing mode disarm
    DEADRECKON_FAILSAFE = 33,        // Dead reckoning failsafe
    BLACKBOX = 34,                   // Blackbox logging
    DDS = 35,                        // DDS interface
    AUTO_ARM_ONCE = 36,              // Auto-arm once
    TURTLE_MODE = 37,                // Turtle mode
    TOYMODE = 38,                    // Toy mode
    UNKNOWN = 100,                   // Unknown method
};
```

### 1.4 Arming Required Enum

Controls how strict arming requirements are:

```cpp
enum class Required {
    NO              = 0,    // No arming required (immediate flight)
    YES_MIN_PWM     = 1,    // Send minimum throttle PWM when disarmed
    YES_ZERO_PWM    = 2,    // Send 0 PWM (no signal) when disarmed
    YES_AUTO_ARM_MIN_PWM = 3,  // Auto-arm once with min PWM
    YES_AUTO_ARM_ZERO_PWM = 4, // Auto-arm once with zero PWM
};
```

### 1.5 Vehicle-Specific Implementations

The library uses inheritance to allow vehicle-specific arming logic:

| Vehicle Type | Class | Location |
|---|---|---|
| **Copter** (Quadcopter, Hexacopter, etc.) | `AP_Arming_Copter` | `ArduCopter/AP_Arming_Copter.h` |
| **Plane** (Fixed-wing) | `AP_Arming_Plane` | `ArduPlane/AP_Arming_Plane.h` |
| **Rover** (Ground vehicle) | `AP_Arming_Rover` | `Rover/AP_Arming_Rover.h` |
| **Sub** (Underwater) | `AP_Arming_Sub` | `ArduSub/AP_Arming_Sub.h` |
| **Blimp** | `AP_Arming_Blimp` | `Blimp/AP_Arming_Blimp.h` |
| **Tracker** (Antenna tracker) | `AP_Arming_Tracker` | `AntennaTracker/AP_Arming_Tracker.h` |

---

## 2. Arming/Disarming State Machine

### 2.1 State Variables

```cpp
private:
    bool armed;                      // Current armed state
    uint64_t last_arm_time_us;      // Timestamp of last arm
    Method _last_arm_method;         // Last method used to arm
    Method _last_disarm_method;      // Last method used to disarm
    bool running_arming_checks;      // True when arm() is executing
    bool last_prearm_checks_result;  // Cache of last pre-arm result
```

### 2.2 Arm State Flow

```
[DISARMED] 
    |
    |--> arm() called with Method & checks parameter
         |
         |--> Check 1: Is already armed? 
         |    YES --> Return false
         |    NO  --> Continue
         |
         |--> Check 2: Validate arming method
         |    (e.g., rudder arming disabled?)
         |    FAIL --> Return false
         |    PASS --> Continue
         |
         |--> Check 3: Perform pre_arm_checks() 
         |    (conditional on do_arming_checks flag)
         |    - Hardware safety switch
         |    - IMU/INS consistency
         |    - Compass health and consistency
         |    - GPS fix and health
         |    - Battery voltage
         |    - RC calibration
         |    - ... and 14 more checks
         |    FAIL --> armed = false, log failure, return false
         |    PASS --> Continue
         |
         |--> Check 4: Perform arm_checks()
         |    (side-effect checks that shouldn't run pre-arm)
         |    - RC arm-specific checks
         |    - GPS driver prep
         |    - Logger PrepForArming()
         |    FAIL --> armed = false, log failure, return false
         |    PASS --> Continue
         |
         |--> SUCCESS: 
         |    - Set armed = true
         |    - Record last_arm_time_us = current time
         |    - Record _last_arm_method = method
         |    - Log ARM event
         |    - Update terrain reference
         |    - Auto-enable geofence
         |    - Update GPIO pin
         |    - Return true
         |    
         |    --> [ARMED]
```

### 2.3 Disarm State Flow

```
[ARMED]
    |
    |--> disarm() called with Method & checks parameter
         |
         |--> Check 1: Is armed?
         |    NO  --> Return false (already disarmed)
         |    YES --> Continue
         |
         |--> Check 2: Validate disarm method
         |    (e.g., rudder disarm enabled? throttle down?)
         |    FAIL --> Return false
         |    PASS --> Continue
         |
         |--> SUCCESS:
         |    - Set armed = false
         |    - Record _last_disarm_method = method
         |    - Log DISARM event
         |    - Check forced logging requirement
         |    - Auto-enable safety switch (if configured)
         |    - Save FFT parameters
         |    - Auto-disable geofence
         |    - Update GPIO pin
         |    - Return true
         |    
         |    --> [DISARMED]
```

### 2.4 Critical State Functions

**is_armed():**
```cpp
bool AP_Arming::is_armed() const
{
    return armed || arming_required() == Required::NO;
}
```
Returns true if either the internal armed flag is set OR arming is not required.

**is_armed_and_safety_off():**
```cpp
bool AP_Arming::is_armed_and_safety_off() const
{
    return is_armed() && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}
```
Returns true only if BOTH armed AND safety switch is disengaged.

---

## 3. Pre-Arm Checks System

### 3.1 Pre-Arm Check Flow

The `pre_arm_checks()` function executes 22+ checks in sequence:

```cpp
bool AP_Arming::pre_arm_checks(bool report)
{
    // Sequence of checks - all must pass (bitwise AND)
    bool checks_result = 
        hardware_safety_check(report)          // Safety switch check
        & heater_min_temperature_checks()      // IMU heater (if enabled)
        & barometer_checks(report)             // Barometer health
        & ins_checks(report)                   // IMU consistency
        & compass_checks(report)               // Compass health & consistency
        & gps_checks(report)                   // GPS lock quality
        & battery_checks(report)               // Battery voltage
        & logging_checks(report)               // SD card status
        & manual_transmitter_checks(report)    // RC calibration
        & mission_checks(report)               // Mission validity
        & rangefinder_checks(report)           // Rangefinder health
        & servo_checks(report)                 // Servo configuration
        & board_voltage_checks(report)         // Board power supply
        & system_checks(report)                // System health
        & terrain_checks(report)               // Terrain database
        & can_checks(report)                   // CAN bus
        & generator_checks(report)             // Generator (if enabled)
        & proximity_checks(report)             // Proximity sensors
        & camera_checks(report)                // Camera/RunCam
        & osd_checks(report)                   // On-screen display
        & mount_checks(report)                 // Camera mount
        & fettec_checks(report)                // FETtec OneWire ESCs
        & visodom_checks(report)               // Visual odometry
        & aux_auth_checks(report)              // Auxiliary auth (scripting)
        & disarm_switch_checks(report)         // Disarm switch config
        & fence_checks(report)                 // Geofence validity
        & opendroneid_checks(report)           // OpenDroneID
        & crashdump_checks(report)             // Crash dump acknowledgment
        & serial_protocol_checks(report)       // Serial protocol config
        & estop_checks(report);                // Emergency stop config
    
    // Track state changes for immediate reporting
    if (!checks_result && last_prearm_checks_result) {
        report_immediately = true;  // Trigger immediate status text
    }
    last_prearm_checks_result = checks_result;
    
    return checks_result;
}
```

### 3.2 Critical Safety Checks

#### IMU (Inertial Sensor) Consistency Checks
```cpp
bool AP_Arming::ins_checks(bool report)
{
    // Gyro health and calibration
    if (!ins.get_gyro_health_all())
        return false;  // "Gyros not healthy"
    if (!ins.gyro_calibrated_ok_all())
        return false;  // "Gyros not calibrated"
    
    // Accelerometer health and calibration
    if (!ins.get_accel_health_all())
        return false;  // "Accels not healthy"
    if (!ins.accel_calibrated_ok_all())
        return false;  // "3D Accel calibration needed"
    
    // Consistency checks with 10-second confirmation
    if (!ins_accels_consistent(ins))
        return false;  // "Accels inconsistent"
    if (!ins_gyros_consistent(ins))
        return false;  // "Gyros inconsistent"
    
    // Temperature calibration
    if (ins.temperature_cal_running())
        return false;  // "temperature cal running"
    
    // Update rate check
    if (!ins.pre_arm_check_gyro_backend_rate_hz(fail_msg))
        return false;
}

bool AP_Arming::ins_accels_consistent(const AP_InertialSensor &ins)
{
    // Requires consistency threshold (ARMING_ACCTHRESH parameter)
    if (!ins.accels_consistent(accel_error_threshold))
        return false;  // Threshold exceeded
    
    // Must maintain consistency for 10 seconds minimum
    if (ins.get_accel_count() > 1 && now - last_accel_pass_ms < 10000)
        return false;
    
    return true;
}
```

#### Compass Health Checks
```cpp
bool AP_Arming::compass_checks(bool report)
{
    // Calibration state
    if (compass.is_calibrating())
        return false;  // "Compass calibration running"
    if (compass.compass_cal_requires_reboot())
        return false;  // "Compass calibrated requires reboot"
    
    // Health check
    if (!compass.healthy())
        return false;  // "Compass not healthy"
    
    // Offset validation
    if (!compass.configured(failure_msg))
        return false;  // Configuration error
    if (offsets.length() > compass.get_offsets_max())
        return false;  // "Compass offsets too high"
    
    // Magnetic field strength (185-875 milligauss)
    float mag_field = compass.get_field().length();
    if (mag_field > AP_ARMING_COMPASS_MAGFIELD_MAX || 
        mag_field < AP_ARMING_COMPASS_MAGFIELD_MIN)
        return false;  // "Check mag field: X, max Y, min Z"
    
    // Compass consistency
    if (!compass.consistent())
        return false;  // "Compasses inconsistent"
    
    // Earth magnetic model comparison (optional)
    Vector3f diff_mgauss = vehicle_mag_field - earth_field_mgauss;
    if (MAX(fabsf(diff_mgauss.x), fabsf(diff_mgauss.y)) > threshold)
        return false;  // Magnetic anomaly detected
}
```

#### GPS Validation Checks
```cpp
bool AP_Arming::gps_checks(bool report)
{
    // GPS driver pre-arm checks
    if (!gps.pre_arm_checks(failure_msg))
        return false;
    
    // For each GPS sensor
    for (uint8_t i = 0; i < gps.num_sensors(); i++) {
        // Fix quality (requires 3D fix)
        if (gps.status(i) < AP_GPS::GPS_OK_FIX_3D)
            return false;  // "GPS X: Bad fix"
        
        // Update rate health
        if (!gps.is_healthy(i))
            return false;  // "GPS X: not healthy"
    }
    
    // Home position set
    if (!ahrs.home_is_set())
        return false;  // "AHRS: waiting for home"
    
    // GPS consistency (within 50m)
    if (!gps.all_consistent(distance_m))
        return false;  // "GPS positions differ by Xm"
    
    // AHRS/GPS alignment (within 10m)
    float distance = gps.location().get_distance(ahrs.location());
    if (distance > 10.0f)
        return false;  // "GPS and AHRS differ by Xm"
    
    // GPS configuration ready
    if (!gps.prepare_for_arming())
        return false;
}
```

#### RC Calibration and Input Checks
```cpp
bool AP_Arming::manual_transmitter_checks(bool report)
{
    // Radio failsafe state
    if (AP_Notify::flags.failsafe_radio)
        return false;  // "Radio failsafe on"
    
    // RC calibration
    for (each RC channel) {
        if (rc_min > rc_trim || rc_max < rc_trim)
            return false;  // "RCX_MIN/MAX vs TRIM mismatch"
    }
    
    // RC calibration in progress
    if (rc().calibrating())
        return false;  // "RC calibrating"
    
    return true;
}

bool AP_Arming::rc_arm_checks(AP_Arming::Method method)
{
    // No valid RC input?
    if (!rc().has_valid_input())
        return true;  // Skip if in failsafe
    
    // Recent RC input
    if (AP_HAL::millis() - rc().last_input_ms() > 1000)
        return true;  // No recent input received
    
    // Duplicate aux function checks
    if (rc().duplicate_options_exist())
        return false;  // "Duplicate Aux Switch Options"
    
    // Flight mode conflicts
    if (rc().flight_mode_channel_conflicts_with_rc_option())
        return false;  // "Mode channel and RCX_OPTION conflict"
    
    // Roll/Pitch/Yaw neutral (unless skipped)
    for each critical_channel {
        if (channel.get_control_in() != 0)
            return false;  // "Roll/Pitch/Yaw is not neutral"
    }
    
    // Throttle check
    if (rc().arming_check_throttle()) {
        if (throttle.get_control_in() != 0)
            return false;  // "Throttle is not neutral"
    }
    
    return true;
}
```

#### Battery Voltage Check
```cpp
bool AP_Arming::battery_checks(bool report)
{
    // Delegates to AP_BattMonitor
    if (!battery.arming_checks(sizeof(buffer), buffer))
        return false;  // Battery-specific failure message
    return true;
}
```

#### System Health Checks
```cpp
bool AP_Arming::system_checks(bool report)
{
    // Parameter storage health
    if (!hal.storage->healthy())
        return false;  // "Param storage failed"
    
    // Parameter storage full
    if (AP_Param::get_eeprom_full())
        return false;  // "parameter storage full"
    
    // Main loop rate (must be >= 90% of expected)
    float loop_rate_pct = actual_loop_rate / expected_loop_rate;
    if (loop_rate_pct < 0.90)
        return false;  // "Main loop slow (XHz < YHz)"
    
    // Terrain system memory
    if (terrain->init_failed())
        return false;  // "Terrain out of memory"
    
    // ADSB system memory
    if (adsb->init_failed())
        return false;  // "ADSB out of memory"
    
    // Internal error flags
    if (AP::internalerror().errors() != 0)
        return false;  // "Internal errors 0xX line:Y"
    
    // GPIO arming checks
    if (!hal.gpio->arming_checks(buffer))
        return false;
    
    // GPS blending configuration
    if (gps.get_auto_switch_type() == 2 && !blending_supported)
        return false;  // "GPS_AUTO_SWITCH==2 but no blending"
}
```

### 3.3 Arm-Time Only Checks

The `arm_checks()` function runs checks with side-effects that should only occur when arming:

```cpp
bool AP_Arming::arm_checks(AP_Arming::Method method)
{
    // RC arm checks (trigger modes, etc.)
    if (!rc_arm_checks(method))
        return false;
    
    // GPS driver final preparation
    if (!gps.prepare_for_arming())
        return false;
    
    // Logger initialization (side-effect!)
    // Must be last to be cleaned up properly
    if (logger->logging_present()) {
        logger->PrepForArming();  // Opens log files
        if (!logger->logging_started())
            return false;  // "Logging not started"
    }
    
    return true;
}
```

### 3.4 Update Loop (1 Hz Check Display)

```cpp
void AP_Arming::update(void)
{
    // Run pre-arm checks at ~1 Hz with state-change detection
    
    const uint32_t now_ms = AP_HAL::millis();
    
    // Display failures every 30 seconds (or immediately if state changed)
    bool display_fail = false;
    if ((report_immediately && (now_ms - last_prearm_display_ms > 4000)) ||
        (now_ms - last_prearm_display_ms > PREARM_DISPLAY_PERIOD*1000)) {
        report_immediately = false;
        display_fail = true;
        last_prearm_display_ms = now_ms;
    }
    
    // User can disable display
    if (option_enabled(Option::DISABLE_PREARM_DISPLAY)) {
        display_fail = false;
    }
    
    // Run checks
    pre_arm_checks(display_fail);
}
```

---

## 4. Mandatory Checks System

Checks that CANNOT be bypassed even with arming checks disabled:

```cpp
bool AP_Arming::mandatory_checks(bool report)
{
    bool ret = true;
#if AP_OPENDRONEID_ENABLED
    // OpenDroneID cannot be skipped
    ret &= opendroneid_checks(report);
#endif
    // RC input calibration cannot be skipped
    ret &= rc_in_calibration_check(report);
    // Serial protocol configuration cannot be skipped
    ret &= serial_protocol_checks(report);
    return ret;
}
```

---

## 5. Vehicle-Specific Arming Implementations

### 5.1 ArduCopter Implementation

**Key Features:**
- Motor status integration (can be armed via motors library)
- Lean angle check (must be within angle_max)
- Mode-specific arming validation
- EKF attitude health requirement
- Proximity sensor integration
- Throttle position validation (prevents hot-start)
- ADSB threat detection
- Altitude disparity check vs barometer
- Motor interlock check
- Disarm switch validation
- Custom parameter checks (failsafe, tuning, etc.)

**Copter Arm Flow:**
```cpp
bool AP_Arming_Copter::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    // Prevent reentrancy
    if (in_arm_motors) return false;
    in_arm_motors = true;
    
    // Return if already armed (motors library integration)
    if (copter.motors->armed()) {
        in_arm_motors = false;
        return true;
    }
    
    // Call base class arm()
    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }
    
    // Copter-specific post-arm initialization:
    AP::logger().set_vehicle_armed(true);
    copter.failsafe_disable();
    AP_Notify::flags.armed = true;
    AP::notify().update();  // Multiple times for notification
    
    // Remember orientation and bearing
    copter.init_simple_bearing();
    copter.initial_armed_bearing_rad = ahrs.get_yaw_rad();
    
    // Reset EKF altitude reference
    if (!ahrs.home_is_set()) {
        ahrs.resetHeightDatum();
        copter.arming_altitude_m = 0;
    }
    
    // Reset home position if not locked
    if (!ahrs.home_is_locked()) {
        copter.set_home_to_current_location(false);
        float pos_d_m = 0;
        ahrs.get_relative_position_D_origin_float(pos_d_m);
        copter.arming_altitude_m = -pos_d_m;
    }
    
    copter.update_super_simple_bearing(false);
    
    // SmartRTL setup
    copter.g2.smart_rtl.set_home(copter.position_ok());
    
    // Hardware soft-armed flag
    hal.util->set_soft_armed(true);
    
    // Disable test functions
    copter.sprayer.test_pump(false);
    
    in_arm_motors = false;
    return true;
}

bool AP_Arming_Copter::arm_checks(AP_Arming::Method method)
{
    // AHRS health
    if (!ahrs.healthy())
        return false;  // "AHRS not healthy"
    
    // Compass health (unless non-compass yaw source)
    if (!ahrs.using_noncompass_for_yaw()) {
        if (!compass.healthy())
            return false;  // "Compass not healthy"
    }
    
    // Mode allows arming
    if (!copter.flightmode->allows_arming(method))
        return false;  // "MODE_NAME mode not armable"
    
    // Lean angle check
    if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max)
        return false;  // "Leaning"
    
    // ADSB threat check
    if (copter.failsafe.adsb)
        return false;  // "ADSB threat detected"
    
    // Throttle position (prevent hot-start)
    if (copter.get_pilot_desired_climb_rate_ms() > 0.0f)
        return false;  // "Throttle too high"
    
    // Manual modes require zero throttle
    if (copter.flightmode->has_manual_throttle() && copter.channel_throttle->get_control_in() > 0)
        return false;  // "Throttle too high"
    
    // Safety switch
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED)
        return false;  // "Safety Switch"
    
    // Call base class (side-effect checks)
    return AP_Arming::arm_checks(method);
}
```

**Copter Pre-Arm Checks (beyond base class):**
- Motor configuration validation
- Autorotation checks (Helicopter)
- Parameter consistency (PID gains, etc.)
- Obstacle avoidance readiness
- GCS failsafe state
- Winch configuration
- Altitude constraints (if landing is mandatory)
- RC throttle failsafe values

### 5.2 ArduPlane Implementation

**Key Features:**
- QuadPlane-specific checks
- Delayed arming support (spoolup delay)
- Terrain database requirements
- Mission feasibility
- RC received validation
- Soft-armed state tracking

**Plane-Specific Methods:**
```cpp
class AP_Arming_Plane : public AP_Arming {
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;
    bool quadplane_checks(bool display_failure);
    bool rc_received_if_enabled_check(bool display_failure);
    void update_soft_armed();
    void change_arm_state(void);
    
private:
    bool delay_arming;  // For QuadPlane spoolup delay
};
```

### 5.3 Rover Implementation

**Key Features:**
- Simple mode checks (no throttle-dependent arming)
- Obstacle avoidance checks
- Motor configuration
- Mode-specific validation

**Rover-Specific Methods:**
```cpp
class AP_Arming_Rover : public AP_Arming {
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;
    bool rc_calibration_checks(const bool display_failure) override;
    bool gps_checks(bool display_failure) override;
    void update_soft_armed();
    
protected:
    bool oa_check(bool report);
    bool parameter_checks(bool report);
    bool mode_checks(bool report);
    bool motor_checks(bool report);
};
```

---

## 6. Arming Parameters

### 6.1 Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| **ARMING_REQUIRE** | Enum | YES_MIN_PWM | Whether arming is required (0=No, 1=Min PWM, 2=Zero PWM, 3=Auto once) |
| **ARMING_CHECK** | Bitmask | ALL | Which checks to perform before arming |
| **ARMING_ACCTHRESH** | Float | 0.75 | Accelerometer consistency threshold (m/s/s) |
| **ARMING_MAGTHRESH** | Int | 100 | Magnetic field error threshold (mGauss) |
| **ARMING_RUDDER** | Enum | 0 (Plane)/1 (Others) | Rudder arming mode (0=Disabled, 1=Arm only, 2=Arm/Disarm) |
| **ARMING_MIS_ITEMS** | Bitmask | 0 | Required mission items for arming |
| **ARMING_OPTIONS** | Bitmask | 0 | Arming behavior options |
| **ARMING_NEED_LOC** | Bool | 0 (Copter/Rover) | Require home location before arming |

### 6.2 Arming Options

```cpp
enum class Option : int32_t {
    DISABLE_PREARM_DISPLAY = (1U << 0),      // Don't show pre-arm check failures
    DISABLE_STATUSTEXT_ON_STATE_CHANGE = (1U << 1),  // No status message on arm/disarm
    SKIP_IMU_CONSISTENCY_ICE_RUNNING = (1U << 2),    // Skip IMU checks when ICE engine running
};
```

---

## 7. Rudder Arming System

Special arming mode for fixed-wing (Plane) vehicles:

```cpp
enum class RudderArming {
    IS_DISABLED = 0,   // Rudder arming disabled
    ARMONLY = 1,       // Right rudder arms only
    ARMDISARM = 2      // Right rudder arms, left rudder disarms
};

// Arm via rudder
bool AP_Arming::arm(AP_Arming::Method method, const bool do_arming_checks)
{
    if (method == Method::RUDDER) {
        switch (get_rudder_arming_type()) {
        case RudderArming::IS_DISABLED:
            return false;  // Not allowed
        case RudderArming::ARMONLY:
        case RudderArming::ARMDISARM:
            break;  // Allowed
        }
    }
    // ... continue with normal arm flow
}

// Disarm via rudder
bool AP_Arming::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (method == Method::RUDDER) {
        // Throttle must be down
        if (rc().get_throttle_channel().get_control_in() > 0)
            return false;
        // Rudder disarm must be enabled
        if (get_rudder_arming_type() != RudderArming::ARMDISARM) {
            gcs().send_text(MAV_SEVERITY_INFO, "Disarm: rudder disarm disabled");
            return false;
        }
    }
    // ... continue with normal disarm flow
}
```

---

## 8. Logging and Telemetry

### 8.1 Logging Events

```cpp
void AP_Arming::Log_Write_Arm(const bool forced, const AP_Arming::Method method)
{
    // Logs ARM event to dataflash/SD card
    // Records: forced_arm flag, arming method, timestamp
}

void AP_Arming::Log_Write_Disarm(const bool forced, const AP_Arming::Method method)
{
    // Logs DISARM event to dataflash/SD card
    // Records: forced_disarm flag, disarm method, timestamp
}

void AP_Arming::check_forced_logging(const AP_Arming::Method method)
{
    // Determines if logging should continue after disarm
    // Some disarm methods (landing, failsafe) keep logging active
}
```

### 8.2 Status Text Messages

Pre-arm check failures are sent as status text via GCS:

```cpp
void AP_Arming::check_failed(const AP_Arming::Check check, bool report, const char *fmt, ...) const
{
    if (!report) return;
    
    // Prepend "PreArm: " or "Arm: " based on running_arming_checks
    const char *metafmt = running_arming_checks ? "Arm: %s" : "PreArm: %s";
    
    // Send to GCS with severity:
    // - CRITICAL if check is enabled
    // - DEBUG if check is disabled
    MAV_SEVERITY severity = check_enabled(check) ? MAV_SEVERITY_CRITICAL : MAV_SEVERITY_DEBUG;
    gcs().send_textv(severity, taggedfmt, arg_list);
}
```

Display periods:
- Every 30 seconds normally
- Immediately when a check transitions from pass to fail

---

## 9. Safety Mechanisms

### 9.1 Hardware Safety Switch

Integrated check that the physical safety switch is in the armed position:

```cpp
bool AP_Arming::hardware_safety_check(bool report)
{
    if (check_enabled(Check::SWITCH)) {
        if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
            check_failed(Check::SWITCH, report, "Hardware safety switch");
            return false;
        }
    }
    return true;
}
```

The vehicle cannot arm if the physical safety switch is in DISARMED position.

### 9.2 Auxiliary Authorization System

Scripting interface for custom arming authorization:

```cpp
#if AP_ARMING_AUX_AUTH_ENABLED
bool AP_Arming::get_aux_auth_id(uint8_t& auth_id);
void AP_Arming::set_aux_auth_passed(uint8_t auth_id);
void AP_Arming::set_aux_auth_failed(uint8_t auth_id, const char* fail_msg);
void AP_Arming::reset_all_aux_auths();
```

Allows custom scripts to block arming with custom failure messages.

### 9.3 Crash Dump Acknowledgment

```cpp
bool AP_Arming::crashdump_checks(bool report)
{
    if (hal.util->last_crash_dump_size() == 0)
        return true;  // No crash dump
    
    if (crashdump_ack.acked)
        return true;  // User acknowledged
    
    // Crash dump present and NOT acknowledged
    check_failed(Check::PARAMETERS, true, "CrashDump data detected");
    return false;
}
```

Prevents arming if firmware has previously crashed, requiring user acknowledgment.

### 9.4 Reboot Requirements

Certain conditions require a reboot before arming can be attempted:

```cpp
// Compass requires reboot after calibration
if (compass.compass_cal_requires_reboot())
    return false;  // "Compass calibrated requires reboot"

// Accelerometer requires reboot after calibration
if (ins.accel_cal_requires_reboot())
    return false;  // "Accels calibrated requires reboot"

// Batch sampling requires reboot
if (ins.batchsampler.enabled() && !ins.batchsampler.is_initialised())
    return false;  // "Batch sampling requires reboot"
```

---

## 10. Automatic Disarming Mechanisms

The library supports automatic disarming triggered by various failsafes:

| Method | Trigger | Behavior |
|--------|---------|----------|
| `LANDED` | Landing detection | Auto-disarm when vehicle lands |
| `CRASH` | Crash detection | Immediate disarm on impact |
| `CPUFAILSAFE` | CPU watchdog | Emergency disarm on CPU failure |
| `BATTERYFAILSAFE` | Low battery | Disarm when battery critically low |
| `RADIOFAILSAFE` | Radio loss | Disarm after RC loss timeout |
| `FENCEBREACH` | Geofence breach | Disarm if boundary violated |
| `EKFFAILSAFE` | EKF divergence | Disarm if navigation fails |
| `TAKEOFFTIMEOUT` | Takeoff duration | Disarm if takeoff takes too long |
| `MISSIONEXIT` | Mission completion | Disarm when mission ends |
| `PARACHUTE_RELEASE` | Parachute deploy | Disarm when parachute fires |

---

## 11. State Persistence and Recovery

### 11.1 Arming State Storage

```cpp
// State information available after events
Method last_disarm_method() const { return _last_disarm_method; }
Method last_arm_method() const { return _last_arm_method; }
uint64_t arm_time_us() const { return is_armed() ? last_arm_time_us : 0; }
bool get_last_prearm_checks_result() const { return last_prearm_checks_result; }
```

### 11.2 Check Result Caching

```cpp
// Pre-arm check results are cached and tracked for state changes
bool last_prearm_checks_result;   // Result of last check
bool report_immediately;          // Flag to report on next failure
uint32_t last_prearm_display_ms;  // Time of last status display
```

---

## 12. Build Configuration

### 12.1 Feature Flags

```cpp
// AP_Arming_config.h
#ifndef AP_ARMING_ENABLED
#define AP_ARMING_ENABLED 1  // Master enable/disable
#endif

#ifndef AP_ARMING_AUX_AUTH_ENABLED
#define AP_ARMING_AUX_AUTH_ENABLED AP_SCRIPTING_ENABLED  // Aux auth requires scripting
#endif

#ifndef AP_ARMING_CRASHDUMP_ACK_ENABLED
#define AP_ARMING_CRASHDUMP_ACK_ENABLED AP_CRASHDUMP_ENABLED  // Only if crashes supported
#endif
```

### 12.2 Vehicle-Specific Configuration

Different vehicles have different arming requirements:

- **Plane**: `ARMING_RUDDER_DEFAULT = ARMONLY` (rudder arm only)
- **Copter/Rover/Heli**: `ARMING_RUDDER_DEFAULT = ARMDISARM` (full control)
- **Plane**: No ARMING_NEED_LOC parameter (location not required)
- **Copter/Rover**: ARMING_NEED_LOC parameter (location optional)

---

## 13. Summary of Key Design Principles

### 1. **Safety First**
- All checks must pass before arming (AND logic)
- Critical failures block arming, warnings display for convenience only
- Mandatory checks cannot be disabled

### 2. **Modularity**
- 21 independent check categories with enable/disable bitmask
- Each check is a separate function, easy to modify or override
- Vehicle-specific implementations via inheritance

### 3. **Flexibility**
- Multiple arming methods (GCS, RC, Rudder, Scripting, etc.)
- Multiple disarming methods for various failsafes
- Configurable check severity and behavior

### 4. **User Feedback**
- Real-time status text messages for each failure
- Periodic display of persistent failures (30 second interval)
- Immediate notification when status changes

### 5. **Extensibility**
- Scripting interface for custom checks (AUX_AUTH)
- Vehicle subclasses can add custom checks
- Library components can register their own checks

### 6. **Robustness**
- Check result caching to prevent rapid state flapping
- 10-second confirmation periods for sensor consistency
- Reentry protection in arm function

### 7. **Logging**
- All arm/disarm events logged with method and timestamp
- Crash dumps trigger acknowledgment requirement
- Failures logged as internal events

---

## 14. Typical Arming Sequence Timeline

```
T+0ms:     User initiates arm request (GCS, RC, Rudder, or API)
           arm(method, true) called

T+1ms:     Check if already armed (return false if yes)
           Check if method is allowed for this vehicle
           
T+10ms:    Execute pre_arm_checks():
           - Safety switch status
           - Hardware health (IMU, compass, barometer)
           - Sensor calibration state
           - Sensor consistency (may block if < 10 seconds)
           - GPS lock and accuracy
           - Battery voltage
           - RC calibration
           - SD card logging
           - System health (storage, loop rate)
           - ... and 12+ more checks

T+1000ms:  If all pre_arm_checks pass, execute arm_checks():
           - RC arm-time specific checks
           - GPS driver preparation
           - Logger PrepForArming() - opens log file
           
T+1100ms:  Set armed = true
           Record arm timestamp and method
           Log ARM event
           Initialize vehicle-specific systems:
           - Terrain reference altitude
           - Geofence auto-enable
           - GPIO arm signal
           - Vehicle-specific initialization
           
T+1200ms:  Send status message "Vehicle armed"
           Return true to caller

[ARMED STATE - Vehicle is flying]

T+flight:  Motors running, vehicle under control
           Continuous failsafe monitoring
           Automatic disarm triggers monitoring
           
T+flight:  One of several disarm triggers occurs:
           - User command
           - Landing detected
           - Failsafe (low battery, RC loss, EKF failure, etc.)
           - Crash detected
           - Mission end
           
T+landing: disarm(method, false) called
           Check if armed (return false if not)
           Set armed = false
           Record disarm method
           Log DISARM event
           Update vehicle systems:
           - Safety switch engage (optional)
           - Geofence auto-disable
           - FFT parameter save
           - Forced logging check
           
T+1ms:     Send status message
           Return true
```

---

## Conclusion

The AP_Arming library provides a comprehensive, modular, and vehicle-agnostic arming system with:

- **21 independent safety checks** covering sensors, configuration, and system state
- **Multiple arming/disarming methods** for different use cases and failsafe triggers
- **Inheritance-based architecture** allowing vehicle-specific customization
- **Real-time feedback** to users via status text messages
- **Extensive logging** for diagnostics and post-flight analysis
- **Safety-first design** ensuring critical checks cannot be bypassed

The system is production-proven across multiple vehicle types (Copter, Plane, Rover, Sub, Blimp, Tracker) and has enabled safe autonomous operation of thousands of aircraft worldwide.

