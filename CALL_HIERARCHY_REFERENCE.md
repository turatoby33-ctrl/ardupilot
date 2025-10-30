# ARDUPILOT COPTER - CALL HIERARCHY QUICK REFERENCE
## File Locations and Key Functions

---

## 1. STARTUP & INITIALIZATION

**AP_Vehicle Setup Sequence:**
- `libraries/AP_Vehicle/AP_Vehicle.cpp:305` - `AP_Vehicle::setup()`
  - Main initialization entry point
  - Load parameters, setup scheduler, initialize hardware

- `ArduCopter/system.cpp:16` - `Copter::init_ardupilot()`
  - Flight-control specific initialization
  - Initialize sensors, motors, failsafe

**Key Init Functions:**
- `ArduCopter/radio.cpp:20` - `Copter::init_rc_in()` - RC channel setup
- `ArduCopter/radio.cpp:47` - `Copter::init_rc_out()` - Motor output setup

---

## 2. MAIN LOOP & SCHEDULER

**Main Loop Entry:**
- `libraries/AP_Vehicle/AP_Vehicle.cpp:551` - `AP_Vehicle::loop()` - Main loop
- `libraries/AP_Scheduler/AP_Scheduler.cpp` - `AP_Scheduler::loop()` - Task scheduler

**Copter Scheduler Tasks:**
- `ArduCopter/Copter.cpp:113` - `scheduler_tasks[]` - All scheduler tasks defined
  - FAST_TASKS listed with frequencies
  - Scheduled tasks with priorities 0-255

---

## 3. SENSOR READING PATHS

**IMU/Gyroscope:**
- `libraries/AP_InertialSensor/AP_InertialSensor.cpp` - `AP_InertialSensor::update()`
  - Raw accel/gyro data at 400Hz (FAST_TASK)

**AHRS/EKF State Estimation:**
- `ArduCopter/Copter.cpp:899` - `Copter::read_AHRS()`
  - Calls `ahrs.update()` for EKF state estimation
  - Position, velocity, attitude output

- `libraries/AP_AHRS/AP_AHRS.cpp` - `AP_AHRS::update()`
  - EKF2/EKF3 implementation
  - Sensor fusion: IMU + GPS + Compass + Barometer + OptFlow

**Inertial Navigation:**
- `ArduCopter/inertia.cpp:4` - `Copter::read_inertia()`
  - Extract position from EKF
  - Update position controller estimates

**Individual Sensor Updates:**
- `libraries/AP_GPS/AP_GPS.cpp` - `AP_GPS::update()` [50Hz]
- `libraries/AP_Compass/AP_Compass.cpp` - `AP_Compass::read()` [10Hz]
- `ArduCopter/Copter.cpp:906` - `Copter::update_altitude()` [10Hz]
- `ArduCopter/sensors.cpp:25` - `Copter::read_rangefinder()` [20Hz]
- `libraries/AP_OpticalFlow/AP_OpticalFlow.cpp` - `AP_OpticalFlow::update()` [200Hz]

**EKF Health & Resets:**
- `ArduCopter/ekf_check.cpp:248` - `Copter::check_ekf_reset()`
  - Detect EKF yaw/primary core resets
  - Adjust attitude controller if needed
- `ArduCopter/ekf_check.cpp` - `Copter::ekf_check()` [10Hz]
  - Monitor EKF health, trigger failsafes

---

## 4. CONTROL LOOPS

**Flight Mode Selection:**
- `ArduCopter/mode.cpp:233` - `Copter::set_mode()`
  - Primary mode switching function
  - Extensive pre-flight checks

**Flight Mode Execution:**
- `ArduCopter/mode.cpp:410` - `Copter::update_flight_mode()` [~400Hz FAST_TASK]
  - Calls `flightmode->run()`

**Flight Mode Implementations:**
- `ArduCopter/mode_stabilize.cpp:9` - `ModeStabilize::run()`
  - Direct pilot control of attitude
  - Pilot throttle pass-through
  
- `ArduCopter/mode_althold.cpp` - `ModeAltHold::run()`
  - Automatic altitude hold
  - Pilot pitch/roll control
  
- `ArduCopter/mode_loiter.cpp:80` - `ModeLoiter::run()`
  - Position hold using GPS
  - Navigation controller + position controller
  
- `ArduCopter/mode_auto.cpp` - `ModeAuto::run()`
  - Waypoint following
  - Mission command execution
  
- `ArduCopter/mode_rtl.cpp` - `ModeRTL::run()`
  - Return to launch
  - Waypoint navigation to home

**Attitude Control:**
- `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp` - Attitude controller PID loops
  - `input_euler_angle_roll_pitch_euler_rate_yaw_rad()` - Angle + rate control
  - `input_attitude_roll_pitch_yaw()` - Direct attitude control
  - `rate_controller_run()` - Low-level gyro rate control

**Position Control:**
- `libraries/AC_AttitudeControl/AC_PosControl.cpp` - Position controller
  - `update_xy_controller()` - Horizontal position control
  - `update_altitude_controller()` - Vertical position control

**Navigation:**
- `libraries/AC_WPNav/AC_WPNav.cpp` - Waypoint navigation
- `libraries/AC_WPNav/AC_Loiter.cpp` - Loiter (position hold) controller

**Motor Mixing & Output:**
- `ArduCopter/Attitude.cpp:10` - `Copter::run_rate_controller_main()` [~400Hz FAST_TASK]
  - Update dt, execute rate controller
  
- `ArduCopter/motors.cpp:120` - `Copter::motors_output_main()` [~400Hz FAST_TASK]
  - Send motor commands to hardware
  
- `ArduCopter/motors.cpp:60` - `Copter::motors_output()`
  - Main motor output function
  - Mixes attitude commands to motor commands
  
- `libraries/AP_Motors/AP_Motors.cpp` - Motor mixing library
  - Frame-specific motor mixing
  - PWM output to ESCs

**Throttle & Hover Estimation:**
- `ArduCopter/Attitude.cpp:32` - `Copter::update_throttle_hover()` [100Hz]
  - Estimate throttle needed for level hover

---

## 5. FAILSAFE SYSTEMS

**Failsafe Main Entry:**
- `ArduCopter/failsafe.cpp:35` - `Copter::failsafe_check()` [1kHz interrupt]
  - Main loop watchdog (called from timer interrupt)
  - Detects main loop hang

**Failsafe Event Handlers:**
- `ArduCopter/events.cpp:13` - `Copter::failsafe_radio_on_event()`
  - RC link loss handling
  
- `ArduCopter/events.cpp:82` - `Copter::failsafe_radio_off_event()`
  - RC link recovery
  
- `ArduCopter/events.cpp:99` - `Copter::handle_battery_failsafe()`
  - Battery voltage/capacity failsafe
  
- `ArduCopter/events.cpp:126` - `Copter::failsafe_gcs_check()` [10Hz]
  - GCS link monitoring
  
- `ArduCopter/events.cpp:163` - `Copter::failsafe_gcs_on_event()`
  - GCS link loss handling
  
- `ArduCopter/events.cpp:299` - `Copter::gpsglitch_check()` [10Hz]
  - GPS glitch detection
  
- `ArduCopter/events.cpp:243` - `Copter::failsafe_terrain_check()` [10Hz]
  - Terrain data availability check
  
- `ArduCopter/events.cpp:474` - `Copter::do_failsafe_action()`
  - Execute failsafe action (LAND, RTL, SMARTRTL, etc.)

**EKF Failsafe:**
- `ArduCopter/ekf_check.cpp` - `Copter::ekf_check()` [10Hz]
  - Detect EKF failures
  - Trigger mode change if configured

**Vibration Check:**
- `ArduCopter/ekf_check.cpp:269` - `Copter::check_vibration()` [10Hz]
  - Excessive vibration detection
  - Enable/disable vibration compensation

---

## 6. MODE SWITCHING

**Mode Change Entry:**
- `ArduCopter/mode.cpp:233` - `Copter::set_mode()` - Primary mode switch function
  - Pre-flight checks
  - Init new mode
  - Exit old mode
  - Update system state

**Mode Initialization & Exit:**
- `ArduCopter/mode.cpp:421` - `Copter::exit_mode()` - Cleanup from old mode
  - Throttle smoothing transition
  - Takeoff cancellation
  - Mode-specific cleanup

**Mode Objects:**
- `ArduCopter/mode.h` - Mode class definitions
- All mode implementations in `ArduCopter/mode_*.cpp`

**Mode Retrieval:**
- `ArduCopter/mode.cpp:32` - `Copter::mode_from_mode_num()` - Get mode instance from number

---

## 7. ARMING SYSTEM

**Pre-Arm Checks:**
- `ArduCopter/AP_Arming_Copter.cpp:8` - `AP_Arming_Copter::pre_arm_checks()`
  - Comprehensive pre-flight checks
  - Sensor calibration verification
  - Parameter validation
  
- `ArduCopter/AP_Arming_Copter.cpp:17` - `AP_Arming_Copter::run_pre_arm_checks()`
  - Execute all individual checks
  
- `ArduCopter/AP_Arming_Copter.cpp:410` - `AP_Arming_Copter::pre_arm_ekf_attitude_check()`
  - EKF attitude agreement check

**Arming Sequence:**
- `ArduCopter/AP_Arming_Copter.cpp:675` - `AP_Arming_Copter::arm()`
  - Main arming function
  - Bearing initialization
  - Home position setup
  - Motor initialization
  - Failsafe re-enable
  
- `ArduCopter/AP_Arming_Copter.cpp:567` - `AP_Arming_Copter::arm_checks()`
  - Final safety checks before armed

**Disarming:**
- `ArduCopter/AP_Arming_Copter.cpp:790` - `AP_Arming_Copter::disarm()`
  - Disarm motors
  - Save compass offsets learned by EKF
  - Clear failsafe state

**Auto-Disarm:**
- `ArduCopter/motors.cpp:10` - `Copter::auto_disarm_check()` [10Hz]
  - Automatic disarm when landed with low throttle

---

## 8. RC INPUT & PILOT COMMANDS

**RC Reading:**
- `ArduCopter/Copter.cpp:573` - `Copter::rc_loop()` [250Hz]
  - Read RC channels
  - Detect mode switch changes
  
- `ArduCopter/radio.cpp:85` - `Copter::read_radio()`
  - Raw RC receiver PWM reading
  - Deadzone application

**RC Channel Access:**
- `libraries/RC_Channel/RC_Channel.cpp` - RC channel class
  - Stores PWM values
  - Scaling to usable ranges

**Pilot Input Functions (Mode-specific):**
- `ArduCopter/mode.cpp` - Various `get_pilot_desired_*()` functions
  - `get_pilot_desired_lean_angles_rad()` - Convert stick to lean angle
  - `get_pilot_desired_yaw_rate_rads()` - Convert stick to yaw rate
  - `get_pilot_desired_throttle()` - Get throttle value
  - `get_pilot_desired_climb_rate_ms()` - Get vertical velocity command

---

## 9. GCS COMMUNICATION

**Telemetry Reception:**
- `ArduCopter/GCS_Copter.cpp` - GCS_Copter class
- `libraries/GCS_MAVLink/GCS_MAVLink.cpp` - MAVLink message handling

**GCS Task Updates:**
- `ArduCopter/Copter.cpp:210-211` - Scheduler tasks for GCS
  - `GCS::update_receive()` [400Hz, priority 102]
  - `GCS::update_send()` [400Hz, priority 105]

---

## 10. LOGGING & DATA RECORDING

**Logging System:**
- `libraries/AP_Logger/AP_Logger.cpp` - Dataflash logging

**Logging Tasks:**
- `ArduCopter/Log.cpp` - Copter-specific logging functions
- `ArduCopter/Copter.cpp` - Scheduler logging tasks [10Hz, 25Hz, 400Hz]

---

## KEY TIMING INFORMATION

**Critical Rates:**
- 400Hz (FAST_TASK): IMU update, EKF update, Rate controller, Flight mode, Motor output
- 250Hz: RC input reading
- 200Hz: OpticalFlow, Proximity sensors
- 100Hz: Throttle hover estimation
- 50Hz: GPS, Navigation, GCS send, Barometer(alt), Throttle loop
- 25Hz: Logging, Fence check
- 20Hz: Rangefinder
- 10Hz: Compass, Battery, EKF check, Vibration check, Failsafe checks, Logging
- 3Hz: SmartRTL position save, Sprayer, General 3Hz loop
- 1Hz: General 1Hz checks, Logging

**Total Loop Time:** ~2.5ms for all FAST_TASKS + one scheduled task

---

## EXECUTION FLOW SUMMARY

```
START: AP_HAL main()
  → AP_Vehicle::setup()
    → Copter::init_ardupilot()
  → AP_Vehicle::loop() [continuous]
    → AP_Scheduler::loop()
      → Execute all FAST_TASKS (400Hz)
        1. Read IMU
        2. Run rate controller
        3. Output motors
        4. Update EKF/AHRS
        5. Read inertial position
        6. Check EKF reset
        7. Run flight mode
        [... ~10 more FAST_TASKS]
      → Execute one scheduled task per loop
        - RC reading (250Hz)
        - GPS update (50Hz)
        - Failsafe checks (10Hz)
        - GCS comms (400Hz interleaved)
        - etc...
      → Continue loop (~2.5ms per iteration)
```

---

## DOCUMENT LOCATION

**Full Call Hierarchy Document:** `/home/user/ardupilot/ARDUPILOT_CALL_HIERARCHIES.md`

Contains complete function call trees for:
1. Startup/Initialization path
2. Main loop execution with all FAST_TASK and scheduled task flows
3. Sensor reading path (INS → AHRS → EKF → Flight mode)
4. Control loop paths (STABILIZE and LOITER modes)
5. Failsafe trigger paths (RC loss, EKF failure, Battery failsafe, etc.)
6. Mode switch path
7. Arming path

Each section includes:
- Complete function call chains
- File locations and line numbers
- Input/output parameters
- Brief descriptions of what each function does
- Execution rates and priorities
- Timing diagrams showing latency

