# ARDCOPTER STAGE 2 DEEP DIVE ANALYSIS
## Part 2: Navigation & Safety Systems Technical Report

---

## EXECUTIVE SUMMARY

ArduCopter implements a multi-layered safety architecture with sophisticated failsafe handling, EKF-based state estimation monitoring, crash detection, and automated landing. The system prioritizes safety through redundant checks, hysteretic filtering, and escalating failsafe responses.

---

## 1. EXTENDED KALMAN FILTER (EKF) HEALTH MONITORING

### File: ekf_check.cpp

#### 1.1 System Architecture

**EKF Check State Machine:**
```
EKF Health Monitoring Loop (Runs at 10Hz)
├── Check if EKF has origin (prerequisite)
├── Evaluate variance thresholds
├── Track fail_count with hysteresis
├── Trigger failsafe at EKF_CHECK_ITERATIONS_MAX (10 iterations = 1 second)
└── Handle recovery/clearance with decrementing counter
```

#### 1.2 Critical Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| **EKF_CHECK_ITERATIONS_MAX** | 10 | Consecutive bad EKF readings to declare failure (1 second at 10Hz) |
| **EKF_CHECK_WARNING_TIME** | 30 seconds | Throttle GCS warning messages to prevent spam |
| **FS_EKF_THRESH** | 0.8 (default) | Variance threshold in meters² |
| **FS_EKF_FILT** | 5.0 Hz | Low-pass filter cutoff for variance smoothing |

#### 1.3 Variance Detection Algorithm

**ekf_over_threshold() Logic:**

```cpp
// Evaluate three variance types
1. COMPASS VARIANCE (mag_variance - x,y,z components)
   - Get max of 3-axis compass variance
   - Count if >= FS_EKF_THRESH

2. VELOCITY VARIANCE (vel_var)
   - CRITICAL: If no optical flow AND vel_var >= 2*FS_EKF_THRESH → count as 2
   - Otherwise: if vel_var >= FS_EKF_THRESH → count as 1

3. POSITION VARIANCE (position_var)
   - Count if >= FS_EKF_THRESH

// Failsafe triggers if:
// (position_var_high AND over_thresh_count >= 1) OR over_thresh_count >= 2
```

**Key Insight:** Velocity variance has special handling - without optical flow, it doubles in weight because it's the only position estimate available.

#### 1.4 EKF Recovery Procedures

**Two-stage recovery when approaching failure (at iterations 8 and 9 of 10):**

```
Iteration 8 (2 steps before failure):
  └─> ahrs.request_yaw_reset()
      • Attempts to correct compass/yaw issues
      • Common source of variance spikes

Iteration 9 (1 step before failure):
  └─> ahrs.check_lane_switch()
      • Switch to alternate EKF core if available
      • Allows multiple EKF instances to operate in parallel
      • Critical for redundancy
```

#### 1.5 Vibration Detection System

**Secondary check running in parallel:**

```
Vibration Detection Criteria:
├── Vertical velocity innovation > 0 (upward bias)
├── Vertical position innovation > 0 (height estimate rising)
├── AND Velocity variance > 1.0 m²/s²
└── OR EKF reports vibration affected

Actions if vibration detected for > 1 second:
├── Set vibration compensation flag
├── Switch position controller to "resistant gains"
└── GCS alert: "Vibration compensation ON"

Recovery:
└── Must be clean for 15 seconds to disable compensation
```

#### 1.6 EKF Failsafe Response (failsafe_ekf_event)

**Response based on FS_EKF_ACTION parameter:**

| Action | Response |
|--------|----------|
| **FS_EKF_ACTION_REPORT_ONLY** (0) | GCS alert only, no mode change |
| **FS_EKF_ACTION_ALTHOLD** (2) | Switch to ALT_HOLD, or LAND if RTL lost |
| **FS_EKF_ACTION_LAND** (1) | Switch to LAND mode (uses no GPS) |
| **FS_EKF_ACTION_LAND_EVEN_STABILIZE** (3) | Force LAND even from manual modes |

**Special Case - Landing with Position:**
```cpp
// If already landing with GPS, disable GPS for new land attempt
if (landing_with_GPS() && !report_only) {
    mode_land.do_not_use_GPS();
}
```

---

## 2. POSITION & NAVIGATION CONTROL

### Files: navigation.cpp, autoyaw.cpp, takeoff.cpp

#### 2.1 Navigation Structure

**Core Navigation Functions:**

```
run_nav_updates() [called at main loop rate]
├── update_super_simple_bearing(false)
└── Calculates bearing from vehicle to home
    (used for simple/super-simple mode corrections)

position_ok() [prerequisite for GPS-dependent operations]
├── Check EKF position estimate health
├── Verify GPS/visual odometry is available
└── Gate all GPS-dependent flight modes
```

#### 2.2 Position Estimation

```cpp
// Home Distance Calculation
float home_distance_m() {
    if (position_ok()) {
        _home_distance_m = current_loc.get_distance(ahrs.get_home());
    }
    return _home_distance_m;  // cached value if no position
}

// Home Bearing (used for RTL and loiter)
float home_bearing_rad() {
    if (position_ok()) {
        _home_bearing_rad = current_loc.get_bearing(ahrs.get_home());
    }
    return _home_bearing_rad;  // cached value if no position
}
```

**Critical Point:** Position estimates are cached - if GPS is lost, the last known position continues to be used for calculations.

#### 2.3 AutoYaw System (Heading Control)

**AutoYaw Modes:**

| Mode | Algorithm | Used For |
|------|-----------|----------|
| **HOLD** | Lock current heading | Hover, standby |
| **LOOK_AT_NEXT_WP** | Face toward waypoint | Auto missions |
| **LOOK_AHEAD** | Face direction of motion | High-speed flight, forward momentum |
| **ROI** | Face Region of Interest | Camera gimbal tracking |
| **FIXED** | Slew to fixed heading with rate limit | Commanded yaw changes |
| **RATE** | Follow yaw rate command | Manual yaw input or guided |
| **ANGLE_RATE** | Rotate at fixed angular rate | Sweeping operations |
| **CIRCLE** | Follow circle path | Orbit operations |
| **WEATHERVANE** | Turn into wind (if enabled) | Stability in wind |

**Yaw Rate Slew Limiting:**

```cpp
// Smooth yaw target changes to prevent jerky motion
const uint32_t now_ms = millis();
float dt = (now_ms - _last_update_ms) * 0.001;
float yaw_angle_step_rad = constrain_float(
    _fixed_yaw_offset_rad,
    -dt * _fixed_yaw_slewrate_rads,  // limit negative change
    dt * _fixed_yaw_slewrate_rads    // limit positive change
);

// Apply step to yaw target
_fixed_yaw_offset_rad -= yaw_angle_step_rad;
_yaw_angle_rad += yaw_angle_step_rad;
```

**Look-Ahead Yaw Calculation:**

```cpp
// Automatically orient toward direction of travel
Vector3f vel_ned_ms;
if (position_ok() && ahrs.get_velocity_NED(vel_ned_ms)) {
    const float speed_ms_sq = vel_ned_ms.xy().length_squared();
    if (speed_ms_sq > (YAW_LOOK_AHEAD_MIN_SPEED_MS * YAW_LOOK_AHEAD_MIN_SPEED_MS)) {
        _look_ahead_yaw_rad = atan2f(vel_ned_ms.y, vel_ned_ms.x);
        // Minimum 1 m/s speed required to activate
    }
}
```

---

## 3. SAFETY & FAILSAFE SYSTEMS

### 3.1 Core Failsafe Architecture

#### File: failsafe.cpp

**Failsafe Execution Levels:**

```
Level 1: CPU/Watchdog Failsafe (1kHz timer interrupt)
├── Monitor main loop tick counter
├── Trigger if main loop stalled > 2 seconds
├── Action: Set motors to minimum output
└── Force disarm if lock persists > 1 second

Level 2: Radio Failsafe (Main loop)
├── Check throttle input against failsafe threshold
├── Debounce with 3-frame counter (requires 3 frames below threshold)
├── Gate: Only trigger if armed OR RC ever seen
└── Recovery: 3 good frames to clear

Level 3: GCS Failsafe (Telem timeout)
├── Monitor heartbeat from ground station
├── Default: No action if GCS lost
├── Configurable: RTL, Land, Continue mission, etc.

Level 4: EKF/Position Failsafe
├── Handled by ekf_check.cpp (covered above)
├── Gate on position estimate requirements

Level 5: Battery Failsafe
├── Monitored by battery monitor library
├── Low voltage or cell under-voltage detection
├── Actions: Warn, RTL, Land, or Continue
```

#### 3.2 Radio Failsafe Details

**Throttle Failsafe Mechanism:**

```cpp
#define FS_COUNTER 3  // 3 consecutive low throttle readings

set_throttle_and_failsafe(throttle_pwm):
├── Check: throttle_pwm < failsafe_throttle_value (default: 975 PWM)
│
├── If LOW throttle:
│   ├── Guard: Skip if already in failsafe or not armed
│   └── Increment: failsafe.radio_counter++
│       └── Trigger when counter >= 3
│
└── If NORMAL throttle:
    ├── Decrement: failsafe.radio_counter--
    └── Clear failsafe when counter <= 0 (recovery)
```

**Detection Logic:**

```cpp
read_radio():
├── If RC input received:
│   ├── Update last_radio_update_ms timestamp
│   └── Process throttle (includes failsafe check)
│
└── If NO RC input:
    ├── Check: elapsed_ms > RC_FS_TIMEOUT_MS (default: 1000ms)
    ├── Guard: Skip if failsafe throttle disabled
    ├── Guard: Skip unless armed OR receiver ever seen
    └── Trigger: set_failsafe_radio(true)
        LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, RADIO_LATE_FRAME)
```

**Failsafe Response Actions (g.failsafe_throttle parameter):**

| Value | Name | Response |
|-------|------|----------|
| 0 | DISABLED | No failsafe |
| 1 | RTL | Return to launch |
| 2 | CONTINUE | Keep flying mission (obsolete in 4.0+) |
| 3 | LAND | Land immediately |
| 4 | SMART_RTL_OR_RTL | Smart return or fall back to RTL |
| 5 | SMART_RTL_OR_LAND | Smart return or fall back to land |
| 6 | AUTO_RTL_OR_RTL | Auto RTL if in auto, else RTL |
| 7 | BRAKE_OR_LAND | Brake mode or land |

---

### 3.3 Crash Detection System

#### File: crash_check.cpp

**Crash Detection Algorithm:**

```
Trigger Criteria (ALL must be true for 2 seconds):
├── 1. Lean angle >= CRASH_CHECK_ANGLE_MIN_DEG (15°)
│   └─> Calculated: lean_angle = acos(cos_roll * cos_pitch)
│
├── 2. Angle error >= CRASH_CHECK_ANGLE_DEVIATION_DEG (30°)
│   └─> abs(desired_angle - actual_angle) > 30°
│
├── 3. Velocity < CRASH_CHECK_SPEED_MAX (10 m/s)
│   └─> Prevents false triggers during fast flight
│
├── 4. Filtered acceleration < CRASH_CHECK_ACCEL_MAX (3 m/s²)
│   └─> 1Hz low-pass filtered, magnitude only
│       (gravity subtracted in NED frame)
│
└── 5. Flight mode has crash check enabled
    ├─ Enabled: Stabilize, Alt Hold, Loiter, Pos Hold, Sport, Guided
    └─ Disabled: Acro, Manual, Throw, Flip (intentional inverted allowed)

Trigger Time: CRASH_CHECK_TRIGGER_SEC = 2 seconds
Counter Rate: MAIN_LOOP_RATE (100 Hz default)
Trigger Count: 200 iterations

Action on Trigger:
├── Log error: CRASH_CHECK_CRASH
├── GCS alert: "Crash: Disarming: AngErr=X.X>30, Accel=X.X<3"
└── Force disarm via: arming.disarm(AP_Arming::Method::CRASH)
```

---

### 3.4 Landing Detection System

#### File: land_detector.cpp

**Landing Detection Algorithm:**

```
Requirements for Landing Declaration (ALL must be true):
├── 1. Motor Output Lower Limit
│   ├─ Multi: throttle_lower limit flag set
│   └─ Heli: below_land_min_collective flag
│
├── 2. Throttle Mix at Minimum
│   └─> attitude_control->is_throttle_mix_min() = true
│       (Prevents altitude control from overriding)
│
├── 3. NO Large Angle Commands
│   └─> desired_roll_pitch_angle < 15° both axes
│       (If pilot demanding tilt, not landed)
│
├── 4. NO Large Angle Error
│   └─> abs(actual - desired) < 30° both axes
│       (Uncontrolled tilt = not landed)
│
├── 5. Acceleration Stationary
│   └─> 1Hz filtered ||accel|| < LAND_DETECTOR_ACCEL_MAX (1.0 m/s²)
│
├── 6. Vertical Velocity Low
│   └─> |vertical_velocity| < LAND_DETECTOR_VEL_Z_MAX (1.0 m/s)
│       (Applied with land_detector_scalar multiplier)
│
├── 7. Rangefinder Check
│   └─> If rangefinder healthy: altitude < 2.0m
│       └─> Prevents false landing at altitude
│
└── 8. Weight-on-Wheels (if equipped)
    └─> Must show contact or unknown (never "no contact")

Duration: Trigger after accumulating time:
├─ Normal mode: LAND_DETECTOR_TRIGGER_SEC (1.0 seconds)
├─ Air Mode: LAND_AIRMODE_DETECTOR_TRIGGER_SEC (3.0 seconds)
│   (Longer in air mode due to motor hover characteristics)
└─ "Maybe" landing: 0.2 seconds (used to prevent tip-over)

Counter Decrement:
└─> If ANY condition fails → reset counter to 0
    (Hysteresis requires continuous satisfaction)
```

---

## 4. ARMING & DISARMING SYSTEM

### File: AP_Arming_Copter.cpp

#### 4.1 Pre-Arm Check Sequence

**Mandatory Checks (always run):**

```cpp
pre_arm_checks():
├── 1. System Initialization
│   └─> hal.scheduler->is_system_initialized()
│
├── 2. Motor Interlock Conflict
│   ├─ Cannot use interlock AND emergency stop simultaneously
│   └─ Check: !find_channel(MOTOR_INTERLOCK && MOTOR_ESTOP)
│
├── 3. Motor Interlock State
│   └─> If using interlock: motor_interlock_switch must be disabled
│
├── 4. Disarm Switch Check
│   └─> Safety switch NOT in disarm position
│
├── 5. Motor Arming Checks
│   └─> motors->arming_checks() for ESC/motor health
│
└── 6. AutoRotate Config (helicopters)
    └─> Check autorotation parameters if mode enabled

Optional Checks (skipped if check_enabled = 0):
├── Parameter Validation
├── Object Avoidance
├── GCS Failsafe
├── Winch Configuration
├── RC Calibration (throttle, roll, pitch, yaw)
├── Altitude Estimates
├── GPS Health
├── Barometer Health
├── Compass Health
├── Lean Angle
└── And more...
```

#### 4.2 GPS Pre-Arm Requirements

**Mandatory GPS Checks:**

```cpp
mandatory_gps_checks():
├── 1. AHRS Pre-Arm
│   └─> AHRS ready, EKF healthy, origin set
│
├── 2. Position Estimate
│   └─> If mode requires GPS:
│       position_ok() must return true
│
├── 3. GPS Glitch Detection
│   └─> EKF reports gps_glitching = false
│       (EKF internal health assessment)
│
└── 4. EKF Variance Check
    └─> If FS_EKF_THRESH > 0:
        All variances must be < FS_EKF_THRESH
        ├─ Compass variance
        ├─ Position variance
        ├─ Velocity variance
        └─ Height variance
```

---

## 5. RADIO CONTROL (RC) INPUT PROCESSING

### Files: radio.cpp, RC_Channel_Copter.cpp

#### 5.1 RC Input System Initialization

**Radio Input Setup (init_rc_in):**

```cpp
init_rc_in():
├── Get channel references (guaranteed non-nullptr):
│   ├─ channel_roll = &rc().get_roll_channel()
│   ├─ channel_pitch = &rc().get_pitch_channel()
│   ├─ channel_throttle = &rc().get_throttle_channel()
│   └─ channel_yaw = &rc().get_yaw_channel()
│
├── Set input ranges:
│   ├─ Roll, Pitch, Yaw: set_angle(4500) → ±45°
│   └─ Throttle: set_range(1000) → 0-1000 control range
│
├── Dead Zone Configuration:
│   ├─ Multi: Roll=20, Pitch=20, Throttle=30, Yaw=20
│   └─ Heli: Roll=20, Pitch=20, Throttle=10, Yaw=15
│       (Heli tighter on throttle for precision)
│
└── Transmitter Tuning Setup
    ├─ rc_tuning = find_channel(TRANSMITTER_TUNING)
    └─ rc_tuning2 = find_channel(TRANSMITTER_TUNING2)
```

#### 5.2 Radio Data Reading (read_radio)

**Main RC Input Loop (runs every main loop iteration):**

```cpp
read_radio():
├── Check: rc().read_input()
│   ├── YES (new frame received):
│   │   ├─ ap.new_radio_frame = true
│   │   ├─ set_throttle_and_failsafe(throttle_pwm)
│   │   ├─ set_throttle_zero_flag(throttle_control)
│   │   ├─ radio_passthrough_to_motors() [for wiggle while disarmed]
│   │   └─ rc_throttle_control_in_filter.apply(dt)  [1Hz LPF]
│   │
│   └── NO (no RC input this frame):
│       ├─ Skip if already in radio failsafe
│       ├─ Check elapsed time since last valid frame
│       ├─ If > RC_FS_TIMEOUT_MS (1000ms default):
│       │   └─ Guard: Skip if RC failsafe disabled
│       │   └─ Guard: Skip unless armed OR receiver ever seen
│       │   └─ Trigger: set_failsafe_radio(true)
│       │       LOGGER_WRITE_ERROR(RADIO, RADIO_LATE_FRAME)
│       └─ Return
```

---

## 6. CRITICAL THRESHOLDS & PARAMETERS SUMMARY

### Table: Key Safety Thresholds

| System | Parameter | Value | Impact |
|--------|-----------|-------|--------|
| **EKF** | FS_EKF_THRESH | 0.8 m² | Variance trigger |
| **EKF** | EKF_CHECK_ITERATIONS_MAX | 10 | 1 second to failsafe |
| **Crash** | CRASH_CHECK_ANGLE_MIN | 15° | Minimum lean detection |
| **Crash** | CRASH_CHECK_ANGLE_DEV | 30° | Angle error threshold |
| **Crash** | CRASH_CHECK_ACCEL_MAX | 3.0 m/s² | Acceleration threshold |
| **Crash** | CRASH_CHECK_SPEED_MAX | 10 m/s | Max speed for crash |
| **Crash** | CRASH_CHECK_TRIGGER | 2 sec | Time before disarm |
| **Land** | LAND_DETECTOR_ACCEL | 1.0 m/s² | Accel threshold |
| **Land** | LAND_DETECTOR_VEL | 1.0 m/s | Velocity threshold |
| **Land** | LAND_DETECTOR_TRIGGER | 1.0 sec | Time to land detect |
| **RC** | RC_FS_TIMEOUT | 1000 ms | RC loss detection |
| **RC** | FS_THR_VALUE | 975 PWM | Failsafe throttle |

---

## CONCLUSION

ArduCopter's navigation and safety systems represent mature, production-proven design. Key strengths:

- **Redundant monitoring** across multiple sensor streams
- **Intelligent hysteresis** prevents false triggers
- **Graceful degradation** maintains safety over mission completion
- **Comprehensive logging** enables post-flight failure analysis
- **Escalating failsafe responses** from warning to disarm
- **Multi-core EKF** allows seamless core switching
- **Extensive pre-arm checks** prevent unsafe configurations

The system prioritizes safety first, then reliability, then capability - reflected in the multiple layers of checks, filters, and failsafes throughout the codebase.
