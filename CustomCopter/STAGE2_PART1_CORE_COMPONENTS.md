# ARDCOPTER STAGE 2 DEEP DIVE ANALYSIS
## Part 1: Core Components Technical Report

---

## EXECUTIVE SUMMARY

ArduCopter's architecture is organized around a scheduler-based main loop that synchronizes sensor reading, attitude control, motor mixing, and output. The system follows a modular design with clear separation between:
- **Sensor Acquisition Layer** (sensors.cpp, system.cpp)
- **Attitude Control Layer** (AC_AttitudeControl libraries)
- **Motor Control Layer** (AP_Motors)
- **Execution Layer** (Scheduler, rate thread)

---

## 1. SENSORS SYSTEM

### 1.1 Sensor Types and Initialization

**Primary Sensors Initialized:**

```cpp
// system.cpp: init_ardupilot() - Line 16-201
void Copter::init_ardupilot()
{
    // Barometer - reads altitude
    barometer.init();  // Line 35
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();  // Line 138

    // GPS - provides position and velocity
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init();  // Line 91

    // Compass - provides heading
    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();  // Line 94

    // Optical Flow - provides relative motion
    #if AP_OPTICALFLOW_ENABLED
    optflow.init(MASK_LOG_OPTFLOW);  // Line 108
    #endif

    // Rangefinder - distance below vehicle
    #if AP_RANGEFINDER_ENABLED
    init_rangefinder();  // Line 142
    #endif

    // Proximity sensor
    #if HAL_PROXIMITY_ENABLED
    g2.proximity.init();  // Line 147
    #endif

    // IMU/Inertial Measurement Unit
    ins.init(scheduler.get_loop_rate_hz());  // Line 214 (startup_INS_ground)
    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);   // Line 186
}
```

### 1.2 Sensor Reading Rates (from scheduler_tasks in Copter.cpp)

```cpp
// Copter.cpp: scheduler_tasks[] - Lines 113-265

// FAST_TASK (highest priority, runs at loop rate ~400Hz or 1000Hz)
FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),      // IMU
FAST_TASK(run_rate_controller_main),                          // Rate control
FAST_TASK(motors_output_main),                                // Motor output

// Scheduled Tasks:
SCHED_TASK(rc_loop,              250,    130,  3),   // 250 Hz - RC input
SCHED_TASK(throttle_loop,         50,     75,  6),   // 50 Hz
SCHED_TASK_CLASS(AP_GPS, &copter.gps, update, 50, 200, 9),  // 50 Hz
SCHED_TASK_CLASS(AP_OpticalFlow, &copter.optflow, update, 200, 160, 12),  // 200 Hz
SCHED_TASK(update_batt_compass,   10,    120, 15),  // 10 Hz - Battery & Compass
SCHED_TASK(read_rangefinder,      20,    100, 33),  // 20 Hz
SCHED_TASK(update_altitude,       10,    100, 42),  // 10 Hz - Barometer
```

### 1.3 Sensor Data Reading Implementation

**Barometer:**
```cpp
// sensors.cpp: Lines 3-9
void Copter::read_barometer(void)
{
    barometer.update();  // Update barometer library
    baro_alt_m = barometer.get_altitude();  // Get altitude in meters
}
```

**Rangefinder:**
```cpp
// sensors.cpp: Lines 25-37
void Copter::read_rangefinder(void)
{
    rangefinder.update();  // Update all rangefinder instances
    rangefinder_state.update();  // Update downward facing
    rangefinder_up_state.update();  // Update upward facing

    // Connect to proximity system if enabled
    if (rangefinder_state.enabled_and_healthy()) {
        g2.proximity.set_rangefinder_alt(...);
    }
}
```

### 1.4 Sensor Data Validation and Filtering

**Position Validation:**
```cpp
// system.cpp: Lines 220-230
bool Copter::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }
    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// system.cpp: Lines 232-256
bool Copter::ekf_has_absolute_position() const
{
    if (!ahrs.have_inertial_nav()) {
        return false;  // DCM position not allowed
    }
    if (!motors->armed()) {
        if (ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS) ||
            ahrs.has_status(AP_AHRS::Status::PRED_HORIZ_POS_ABS)) {
            return true;
        }
        return false;
    }
    if (ahrs.has_status(AP_AHRS::Status::CONST_POS_MODE)) {
        return false;  // EKF in constant position mode - bad
    }
    return ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS);
}
```

**Altitude Validation:**
```cpp
// system.cpp: Lines 299-315
bool Copter::ekf_alt_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        return false;
    }
    // require both vertical velocity and position
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }
    if (!ahrs.has_status(AP_AHRS::Status::VERT_VEL)) {
        return false;
    }
    return true;
}
```

**Rangefinder Filtering:**
```cpp
// sensors.cpp: Lines 11-22
void Copter::init_rangefinder(void)
{
    rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
    rangefinder.init(ROTATION_PITCH_270);
    // Apply low-pass filter to rangefinder data
    rangefinder_state.alt_m_filt.set_cutoff_frequency(g2.rangefinder_filt);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
}
```

---

## 2. MOTOR CONTROL SYSTEM

### 2.1 Motor Initialization and Frame Configuration

**Motor Allocation (system.cpp: Lines 363-528):**

```cpp
void Copter::allocate_motors(void)
{
    // Select motor class based on frame configuration
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        // ... etc
        default:
            // Create matrix-based motor controller for generic frames
            motors = NEW_NOTHROW AP_MotorsMatrix(
                copter.scheduler.get_loop_rate_hz());
            break;

        case AP_Motors::MOTOR_FRAME_TRI:
            motors = NEW_NOTHROW AP_MotorsTri(
                copter.scheduler.get_loop_rate_hz());
            break;

        case AP_Motors::MOTOR_FRAME_HELI:
            motors = NEW_NOTHROW AP_MotorsHeli_Single(
                copter.scheduler.get_loop_rate_hz());
            break;
    }

    // Load motor parameters from EEPROM
    AP_Param::load_object_from_eeprom(motors, motors_var_info);
}
```

### 2.2 Motor Mixing Algorithm

The motor mixing uses a **matrix-based approach** where each motor has roll, pitch, yaw, and throttle factors.

**Motor Output Equation (AP_MotorsMatrix.cpp: Lines 213-400):**

```cpp
void AP_MotorsMatrix::output_armed_stabilizing()
{
    // Apply compensation for battery voltage and altitude
    const float compensation_gain = thr_lin.get_compensation_gain();

    // Get roll, pitch, yaw control inputs (normalized -1 to +1)
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle_thrust = get_throttle() * compensation_gain;

    // For each motor, calculate thrust output
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // CORE MIXING EQUATION:
            // thrust = roll*roll_factor + pitch*pitch_factor
            _thrust_rpyt_out[i] =
                roll_thrust * _roll_factor[i] +
                pitch_thrust * _pitch_factor[i];
        }
    }

    // Calculate yaw control headroom (how much yaw we can fit)
    float yaw_allowed = 1.0f;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            const float thrust_rp_best_throttle =
                throttle_thrust_best_rpy + _thrust_rpyt_out[i];
            float motor_room;
            if (is_positive(yaw_thrust * _yaw_factor[i])) {
                motor_room = 1.0 - thrust_rp_best_throttle;  // room to upper limit
            } else {
                motor_room = thrust_rp_best_throttle;  // room to lower limit
            }
            const float motor_yaw_allowed =
                MAX(motor_room, 0.0) / fabsf(_yaw_factor[i]);
            yaw_allowed = MIN(yaw_allowed, motor_yaw_allowed);
        }
    }

    // Constrain yaw to what's available
    yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);

    // Add yaw control to each motor
    float rpy_low = 1.0f;   // lowest thrust
    float rpy_high = -1.0f; // highest thrust
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // FINAL MIXING EQUATION:
            // thrust = roll*roll_factor + pitch*pitch_factor + yaw*yaw_factor
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] +
                yaw_thrust * _yaw_factor[i];

            rpy_low = MIN(rpy_low, _thrust_rpyt_out[i]);
            rpy_high = MAX(rpy_high, _thrust_rpyt_out[i]);
        }
    }

    // Calculate scaling to fit all outputs in 0-1 range
    float rpy_scale = 1.0f;
    if (rpy_high - rpy_low > 1.0f) {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }

    // Scale and add throttle to each motor
    const float throttle_thrust_best_plus_adj =
        throttle_thrust_best_rpy + thr_adj;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // COMPLETE MOTOR EQUATION:
            _thrust_rpyt_out[i] =
                (throttle_thrust_best_plus_adj * _throttle_factor[i]) +
                (rpy_scale * _thrust_rpyt_out[i]);
        }
    }
}
```

### 2.3 Motor Output Conversion

```cpp
// AP_MotorsMatrix.cpp: Lines 143-183
void AP_MotorsMatrix::output_to_motors()
{
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // Convert normalized thrust (0-1) to actuator value with slew limiting
            set_actuator_with_slew(_actuator[i],
                thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]));
        }
    }

    // Convert actuator to PWM and send to ESC
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }
}
```

### 2.4 Example: Quadcopter X Configuration

For a **Quadcopter X** frame, the mixing factors are:

```
Motor 0 (front-right):  roll_factor = +0.5,  pitch_factor = +0.5,  yaw_factor = -1.0
Motor 1 (rear-left):    roll_factor = -0.5,  pitch_factor = -0.5,  yaw_factor = -1.0
Motor 2 (front-left):   roll_factor = -0.5,  pitch_factor = +0.5,  yaw_factor = +1.0
Motor 3 (rear-right):   roll_factor = +0.5,  pitch_factor = -0.5,  yaw_factor = +1.0
```

**Motor Equations for Quad X:**
```
M0_thrust = throttle + roll*0.5 + pitch*0.5 - yaw
M1_thrust = throttle - roll*0.5 - pitch*0.5 - yaw
M2_thrust = throttle - roll*0.5 + pitch*0.5 + yaw
M3_thrust = throttle + roll*0.5 - pitch*0.5 + yaw
```

### 2.5 ESC Calibration

**Calibration Process (esc_calibration.cpp):**

```cpp
// Check if ESC calibration should be entered
void Copter::esc_calibration_startup_check()  // Lines 10-75
{
    #define ESC_CALIBRATION_HIGH_THROTTLE   950

    // Check if throttle is high on startup
    if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
        g.esc_calibrate.set_and_save(
            ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
        // Request restart
        while(1) { hal.scheduler->delay(5); }
    }
}

// Passthrough calibration mode
void Copter::esc_calibration_passthrough()  // Lines 78-104
{
    while(1) {
        read_radio();  // Read pilot throttle input

        // Pass pilot throttle directly to motors
        motors->set_throttle_passthrough_for_esc_calibration(
            channel_throttle->get_control_in() * 0.001f);

        hal.scheduler->delay(3);  // 3ms delay for 333Hz update rate
    }
}

// Automatic calibration
void Copter::esc_calibration_auto()  // Lines 107-140
{
    // Set throttle to maximum
    motors->set_throttle_passthrough_for_esc_calibration(1.0f);

    // Hold maximum for 5 seconds
    uint32_t tstart = millis();
    while (millis() - tstart < 5000) {
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        hal.scheduler->delay(3);
    }

    // Set throttle to minimum (blocks until restart)
    while(1) {
        motors->set_throttle_passthrough_for_esc_calibration(0.0f);
        hal.scheduler->delay(3);
    }
}
```

### 2.6 Motor Testing

```cpp
// motor_test.cpp: Lines 19-96
void Copter::motor_test_output()
{
    if (!ap.motor_test) return;

    uint32_t now = AP_HAL::millis();
    if ((now - motor_test_start_ms) >= motor_test_timeout_ms) {
        motor_test_stop();
        return;
    }

    int16_t pwm = 0;

    // Calculate PWM based on throttle type
    switch (motor_test_throttle_type) {
        case MOTOR_TEST_THROTTLE_PERCENT:
            // Convert percentage to PWM
            pwm = pwm_min + (pwm_max - pwm_min) *
                  motor_test_throttle_value * 1e-2f;
            break;

        case MOTOR_TEST_THROTTLE_PWM:
            // Direct PWM value
            pwm = (int16_t)motor_test_throttle_value;
            break;

        case MOTOR_TEST_THROTTLE_PILOT:
            // Pass through pilot throttle
            pwm = channel_throttle->get_radio_in();
            break;
    }

    // Validate PWM range
    if (pwm < RC_Channel::RC_MIN_LIMIT_PWM ||
        pwm > RC_Channel::RC_MAX_LIMIT_PWM) {
        motor_test_stop();
        return;
    }

    // Output to specific motor sequence
    if (!motors->output_test_seq(motor_test_seq, pwm)) {
        motor_test_stop();
    }
}
```

### 2.7 Critical Motor Safety Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| MOT_PWM_MIN | 1000 µs | Minimum PWM output |
| MOT_PWM_MAX | 2000 µs | Maximum PWM output |
| MOT_SPIN_ARM | 0.1 | PWM point where motors start spinning |
| MOT_SPIN_MIN | 0.15 | PWM minimum spinning threshold |
| MOT_SPIN_MAX | 0.95 | PWM point where thrust saturates |
| MOT_THST_HOVER | ~0.5 | Throttle needed to hover (learned) |
| MOT_YAW_HEADROOM | 0-500 | PWM headroom for yaw control |
| MOT_THST_EXPO | 0.0 | Thrust curve (0=linear, 1=2nd order) |

---

## 3. ATTITUDE CONTROL SYSTEM

### 3.1 Attitude Control Architecture

**Class Hierarchy:**
```
AC_AttitudeControl (base class)
    ├── AC_AttitudeControl_Multi (multicopters)
    │   └── AC_AttitudeControl_Multi_6DoF (6DOF)
    └── AC_AttitudeControl_Heli (traditional helicopters)
```

### 3.2 Three-Level Control Cascade

```
Level 1: Angle Control (input desired attitude)
         ↓
Level 2: Rate Control (input desired angular rate)
         ↓
Level 3: Motor Commands (output motor thrust)
```

### 3.3 Rate Controller PID Gains (AC_AttitudeControl_Multi.h)

**Roll/Pitch Rate Control:**
```cpp
AC_PID _pid_rate_roll {
    AC_PID::Defaults{
        .p         = AC_ATC_MULTI_RATE_RP_P       (0.135f),   // Proportional
        .i         = AC_ATC_MULTI_RATE_RP_I       (0.135f),   // Integral
        .d         = AC_ATC_MULTI_RATE_RP_D       (0.0036f),  // Derivative
        .ff        = 0.0f,                                     // Feed-forward
        .imax      = AC_ATC_MULTI_RATE_RP_IMAX   (0.5f),     // Integral max
        .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ(20.0f),   // Target filter
        .filt_E_hz = 0.0f,                                     // Error filter
        .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ(20.0f),   // Derivative filter
        .srmax     = 0,                                        // Slew rate max
        .srtau     = 1.0                                       // Slew rate tau
    }
};
```

**Yaw Rate Control:**
```cpp
AC_PID _pid_rate_yaw {
    AC_PID::Defaults{
        .p         = AC_ATC_MULTI_RATE_YAW_P      (0.180f),
        .i         = AC_ATC_MULTI_RATE_YAW_I      (0.018f),
        .d         = AC_ATC_MULTI_RATE_YAW_D      (0.0f),
        .ff        = 0.0f,
        .imax      = AC_ATC_MULTI_RATE_YAW_IMAX  (0.5f),
        .filt_T_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ(20.0f),
        .filt_E_hz = AC_ATC_MULTI_RATE_YAW_FILT_HZ(2.5f),   // Lower for yaw
        .filt_D_hz = AC_ATC_MULTI_RATE_RPY_FILT_HZ(20.0f),
        .srmax     = 0,
        .srtau     = 1.0
    }
};
```

### 3.4 Attitude Control Loop Execution (Attitude.cpp)

```cpp
// Attitude.cpp: Lines 10-24
void Copter::run_rate_controller_main()
{
    // Set loop time for controllers
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt_s(last_loop_time_s);
    attitude_control->set_dt_s(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt_s(last_loop_time_s);
        // Run PID rate controller for roll/pitch/yaw
        attitude_control->rate_controller_run();
    }

    // Reset temporary inputs (auto-levels, system ID inputs)
    attitude_control->rate_controller_target_reset();
}
```

### 3.5 Throttle Management

**Hover Throttle Learning:**
```cpp
// Attitude.cpp: Lines 30-67
void Copter::update_throttle_hover()
{
    if (!motors->armed() || ap.land_complete) return;
    if (flightmode->has_manual_throttle()) return;
    if (!is_zero(pos_control->get_vel_desired_NEU_ms().z)) return;

    float vel_d_ms;
    if (!AP::ahrs().get_velocity_D(vel_d_ms, vibration_check.high_vibes)) {
        return;
    }

    // If hovering level: update hover throttle estimate
    if ((throttle > 0.0f) && (fabsf(vel_d_ms) < 0.6) &&
        (fabsf(ahrs.get_roll_rad() - attitude_control->get_roll_trim_rad())
            < radians(5)) &&
        (labs(ahrs.get_pitch_rad()) < radians(5))) {

        // Update hover throttle with 1% time constant
        motors->update_throttle_hover(0.01f);
    }
}
```

**Pilot Climb Rate Conversion:**
```cpp
// Attitude.cpp: Lines 71-112
float Copter::get_pilot_desired_climb_rate_ms()
{
    // Return zero if no RC input
    if (!rc().has_valid_input()) {
        return 0.0f;
    }

    float throttle_control = channel_throttle->get_control_in();

    // Ensure reasonable throttle value (0-1000)
    throttle_control = constrain_float(throttle_control, 0.0f, 1000.0f);

    // Apply deadzone around mid-stick
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    float desired_rate_ms = 0.0f;

    // Map throttle to climb rate
    if (throttle_control < deadband_bottom) {
        // Below deadband: descend proportionally
        desired_rate_ms = get_pilot_speed_dn() * 0.01 *
            (throttle_control - deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // Above deadband: ascend proportionally
        desired_rate_ms = g.pilot_speed_up_cms * 0.01 *
            (throttle_control - deadband_top) / (1000.0 - deadband_top);
    } else {
        // In deadband: zero climb rate
        desired_rate_ms = 0.0f;
    }

    return desired_rate_ms;
}
```

### 3.6 Throttle-Attitude Mix Control

```cpp
// AC_AttitudeControl_Multi.h: Lines 69-77
// Throttle vs attitude control prioritisation parameters
AP_Float _thr_mix_man;     // Manual mode (default ~0.5)
AP_Float _thr_mix_min;     // Landing mode (default ~0.1-0.25)
AP_Float _thr_mix_max;     // Flight mode (default ~0.5-0.9)

// Usage in AC_AttitudeControl_Multi.cpp:
void set_throttle_mix_min() override {
    _throttle_rpy_mix_desired = _thr_mix_min;  // Landing
}
void set_throttle_mix_man() override {
    _throttle_rpy_mix_desired = _thr_mix_man;  // Manual
}
void set_throttle_mix_max(float ratio) override;  // Automatic
```

---

## 4. MAIN LOOP & SCHEDULER

### 4.1 Main Loop Structure (Copter.cpp: Lines 113-265)

**FAST_TASK (Runs at loop rate ~400Hz)**
```
Priority 1: INS Update (gyro/accel reading)
Priority 2: Rate Controller (PID loop for attitude)
Priority 3: Motor Output
Priority 4: AHRS Update (EKF fusion)
Priority 5: Inertial Navigation
Priority 6: EKF Reset Check
Priority 7: Flight Mode Update
Priority 8: Home Update
Priority 9: Land/Crash Detection
Priority 10: Rangefinder Update
```

**Scheduled Tasks (Various rates)**
```
250 Hz: RC Input Reading (rc_loop)
200 Hz: Optical Flow
50 Hz: GPS, Throttle Control, Navigation Updates
25 Hz: Fence Check
20 Hz: Rangefinder, Compass
10 Hz: Battery, Compass, Altitude, Auto-Disarm, Auto-Trim
3 Hz: General tasks
1 Hz: System-level tasks (parameter updates, logging)
```

### 4.2 Task Scheduler Table (scheduler_tasks[])

```cpp
// Copter.cpp: Lines 113-265
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // ====== FAST_TASK (highest priority, runs at loop rate) ======
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),

    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller_main),

    // send outputs to the motors library immediately
    FAST_TASK(motors_output_main),

    // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),

    // Inertial Nav
    FAST_TASK(read_inertia),

    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),

    // run the attitude controllers
    FAST_TASK(update_flight_mode),

    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),

    // check if we've landed or crashed
    FAST_TASK(update_land_and_crash_detectors),

    // surface tracking update
    FAST_TASK(update_rangefinder_terrain_offset),

    // ====== Regular Scheduled Tasks ======
    SCHED_TASK(rc_loop,              250,    130,  3),   // RC input
    SCHED_TASK(throttle_loop,         50,     75,  6),   // Throttle control

    SCHED_TASK_CLASS(AP_GPS, &copter.gps, update, 50, 200, 9),  // GPS
    SCHED_TASK_CLASS(AP_OpticalFlow, &copter.optflow, update, 200, 160, 12),

    SCHED_TASK(update_batt_compass,   10,    120, 15),   // Battery & Compass
    SCHED_TASK(read_rangefinder,      20,    100, 33),   // Rangefinder

    SCHED_TASK(update_altitude,       10,    100, 42),   // Barometer
    SCHED_TASK(run_nav_updates,       50,    100, 45),   // Navigation
    SCHED_TASK(update_throttle_hover, 100,    90, 48),   // Hover throttle

    // Logging and monitoring
    SCHED_TASK(one_hz_loop,           1,     100, 81),   // 1 Hz tasks
    SCHED_TASK(ekf_check,             10,     75, 84),   // EKF validation

    // ... more tasks at various rates ...
};
```

### 4.3 Task Execution Flow

```
Main Loop Cycle (every 2.5ms at 400Hz):
│
├─ INS Update [FAST_TASK]
│  └─ Update gyro & accel from IMU
│
├─ Rate Controller [FAST_TASK]
│  └─ PID calculation for roll/pitch/yaw rates
│  └─ Output motor commands
│
├─ Motors Output [FAST_TASK]
│  └─ Convert thrust to PWM
│  └─ Send to ESCs
│
├─ AHRS Update [FAST_TASK]
│  └─ EKF fusion of all sensors
│  └─ Calculate attitude & position
│
├─ Scheduled Task Check
│  └─ Check if any scheduled task is due
│  └─ Execute if time has elapsed
│
└─ Repeat
```

### 4.4 RC Input Loop (250 Hz)

```cpp
// Copter.cpp: Lines 573-579
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    read_radio();  // Read RC input from receiver
    rc().read_mode_switch();  // Check for mode changes
}
```

### 4.5 Throttle Loop (50 Hz)

```cpp
// Copter.cpp: Lines 583-602
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of
    // throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();

    // Helicopter-specific updates
    #if FRAME_CONFIG == HELI_FRAME
    heli_update_rotor_speed_targets();
    heli_update_landing_swash();
    #endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}
```

### 4.6 1 Hz Loop (System-Level Tasks)

```cpp
// Copter.cpp: Lines 763-818
void Copter::one_hz_loop()
{
    if (!motors->armed()) {
        // Only when disarmed:
        update_using_interlock();
        motors->set_frame_class_and_type(...);  // Update frame config
        motors->update_throttle_range();  // Update throttle calibration
    }

    // Update auxiliary servos
    AP::srv().enable_aux_servos();

    // Update PID notch filters with filtered loop rate
    if (!using_rate_thread) {
        attitude_control->set_notch_sample_rate(
            AP::scheduler().get_filtered_loop_rate_hz());
    }

    // Check if we should start rate thread
    #if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    if (!started_rate_thread && get_fast_rate_type() !=
        FastRateType::FAST_RATE_DISABLED) {
        hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&Copter::rate_controller_thread, void),
            "rate",
            1536, AP_HAL::Scheduler::PRIORITY_RCOUT, 1);
    }
    #endif
}
```

### 4.7 Rate Thread (Optional High-Speed Rate Control)

For high-performance systems, ArduCopter can run the rate controller in a **separate thread** at maximum rate:

```cpp
// rate_thread.cpp
void Copter::rate_controller_thread()
{
    // This thread runs at maximum rate (typically 1000 Hz)
    // without being limited by other tasks

    while (true) {
        // Wait for new IMU data
        while (!ins.wait_for_sample()) {
            hal.scheduler->delay_microseconds(10);
        }

        // Run the rate controller immediately on new IMU data
        attitude_control->rate_controller_run();

        // Output to motors
        hal.rcout->cork();
        // ... output to motors ...
        hal.rcout->push();
    }
}
```

### 4.8 Update Flight Mode (Attitude Control Entry Point)

```cpp
// Called from main loop FAST_TASK
void Copter::update_flight_mode()
{
    // Call the current flight mode's run function
    // This processes RC input and updates target attitude/rates
    flightmode->run();
}
```

---

## 5. DATA FLOW DIAGRAMS

### 5.1 Complete System Data Flow

```
SENSORS INPUT
├─ IMU (Gyro + Accel)
│  └─→ [INS Update @ loop rate]
│      └─→ AHRS/EKF
│
├─ GPS
│  └─→ [50 Hz Update]
│      └─→ EKF Position/Velocity
│
├─ Compass
│  └─→ [10 Hz Update]
│      └─→ EKF Heading
│
├─ Barometer
│  └─→ [10 Hz Update]
│      └─→ Altitude Estimate
│
└─ RC Receiver
   └─→ [250 Hz Reading]
       └─→ Pilot Command Processing

        ↓

ATTITUDE CONTROL
├─ Desired Attitude (from flight mode)
│  └─→ Angle Error = Desired - Current
│      └─→ [Angle Controller]
│          └─→ Desired Rate
│
└─ Desired Rate (from above or direct input)
   └─→ Rate Error = Desired - Current (from gyro)
       └─→ [Rate Controller PID]
           └─→ Motor Control Signal (-1 to +1)

        ↓

MOTOR MIXING
├─ Roll Signal × Roll Factor
├─ Pitch Signal × Pitch Factor
├─ Yaw Signal × Yaw Factor
├─ Throttle Signal × Throttle Factor
└─→ [Sum for each motor]
    └─→ Motor Thrust Output [0-1]

        ↓

ESC & MOTOR
├─ Thrust [0-1]
│  └─→ [Convert to PWM 1000-2000 µs]
│      └─→ PWM output to ESC
│
└─ ESC/Motor Power Output
   └─→ Propeller thrust
       └─→ Gyro senses angular acceleration
           └─→ Back to INS Update
```

### 5.2 Control Loop Hierarchy

```
Outer Loop (Slower, ~10-50 Hz)
  Position/Navigation Control
    │
    ├─→ Desired Position/Velocity
    └─→ Desired Attitude/Climb Rate
         │
    Inner Loop (Fast, ~400 Hz)
      Attitude Control
        │
        ├─→ Desired Attitude
        └─→ Rate Controller (PID)
             │
             ├─→ Gyro Reading
             ├─→ Rate Error Calculation
             ├─→ PID Processing
             └─→ Motor Command
                 │
                 Motor Mixing (400 Hz)
                   │
                   ├─→ Mix roll/pitch/yaw/throttle
                   ├─→ Apply thrust to motors
                   └─→ PWM output (1000-2000 µs)
```

---

## 6. CRITICAL CONSTANTS AND PARAMETERS

### 6.1 Loop Rates

| Component | Default Rate | Purpose |
|-----------|--------------|---------|
| INS/Gyro | 1000 Hz | Inertial measurement unit update |
| Rate Controller | 400 Hz | Attitude rate feedback control |
| Motor Output | 400 Hz | Motor command sending |
| AHRS/EKF | 400 Hz | State estimation |
| RC Input | 250 Hz | Pilot command reading |
| Throttle Control | 50 Hz | Throttle mixing & hover learning |
| GPS | 50 Hz | Position/velocity update |
| Compass | 10 Hz | Heading measurement |
| Barometer | 10 Hz | Altitude measurement |
| Battery | 10 Hz | Voltage/current monitoring |
| Rangefinder | 20 Hz | Distance measurement |
| Optical Flow | 200 Hz | Relative motion |

### 6.2 PID Tuning Defaults (Multicopter)

**Roll/Pitch Rate Controller:**
```
ATC_RAT_RLL_P = 0.135    (Proportional gain)
ATC_RAT_RLL_I = 0.135    (Integral gain)
ATC_RAT_RLL_D = 0.0036   (Derivative gain)
ATC_RAT_RLL_IMAX = 0.5   (Integral limit)
ATC_RAT_RLL_FILT_HZ = 20 (Filter cutoff)
```

**Yaw Rate Controller:**
```
ATC_RAT_YAW_P = 0.180
ATC_RAT_YAW_I = 0.018
ATC_RAT_YAW_D = 0.0
ATC_RAT_YAW_IMAX = 0.5
ATC_RAT_YAW_FLTE_HZ = 2.5  (Lower for yaw damping)
```

### 6.3 Motor Output Parameters

```
MOT_PWM_MIN = 1000         (Minimum PWM in µs)
MOT_PWM_MAX = 2000         (Maximum PWM in µs)
MOT_SPIN_ARM = 0.1         (PWM point where motors arm)
MOT_SPIN_MIN = 0.15        (Minimum thrust point)
MOT_SPIN_MAX = 0.95        (Maximum useful thrust)
MOT_THST_HOVER = varies    (Learned via hover detection)
MOT_THST_EXPO = 0.0        (0=linear, 1=quadratic thrust)
MOT_YAW_HEADROOM = 0-500   (PWM reserved for yaw control)
MOT_BAT_VOLT_MAX = 4.2×cells
MOT_BAT_VOLT_MIN = 3.3×cells
```

---

## CONCLUSION

ArduCopter's core components demonstrate a modular, scheduler-based approach with clear separation of concerns:

1. **Sensors** → Data Collection & Validation
2. **State Estimation** → EKF Fusion
3. **Control Logic** → Flight Modes + Attitude Control
4. **Motor Mixing** → Convert attitude to thrust
5. **Output** → PWM to ESCs

The **400 Hz rate control loop** runs independently from slower navigation/mission tasks, ensuring responsive attitude stabilization. The **extensible motor mixing system** supports any multi-rotor configuration through factor matrices, while **safety systems** with multiple layers of failsafes ensure safe operation.
