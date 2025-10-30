# ARDUPILOT COPTER - COMPLETE FUNCTION CALL HIERARCHIES
## Stage 4 Architecture Mapping

---

## 1. STARTUP/INITIALIZATION PATH
### From main() to armed and ready

```
AP_HAL main()
  └─> AP_Vehicle::setup()  [libraries/AP_Vehicle/AP_Vehicle.cpp:305]
       │   Brief: Initialize system, load parameters, setup scheduler
       │   
       ├─> AP_Param::setup_sketch_defaults()
       │    └─ Load default parameter values
       │
       ├─> AP_Param::check_var_info() + load_parameters()
       │    └─ Validate and load persistent parameters from storage
       │
       ├─> AP_Scheduler::init(tasks, task_count)
       │    └─ Initialize task scheduler with Copter and Vehicle task tables
       │
       ├─> set_control_channels()
       │    └─ Set up RC channel mappings
       │
       ├─> gcs().init() + gcs().setup_console()
       │    └─ Initialize GCS (Ground Control Station) communication
       │
       ├─> BoardConfig.init()
       │    └─ Initialize board-specific hardware configuration
       │
       ├─> logger.init()
       │    └─ Initialize data logging system
       │
       ├─> Copter::init_ardupilot()  [ArduCopter/system.cpp:16]
       │    Brief: Flight-control specific initialization
       │    │
       │    ├─> notify.init()
       │    │    └─ Initialize notification system (LEDs, buzzers)
       │    │
       │    ├─> battery.init()
       │    │    └─ Initialize battery monitor
       │    │
       │    ├─> barometer.init()
       │    │    └─ Initialize barometric pressure sensor
       │    │
       │    ├─> gcs().setup_uarts()
       │    │    └─ Setup telemetry serial ports
       │    │
       │    ├─> update_using_interlock()
       │    │    └─ Setup motor interlock configuration
       │    │
       │    ├─> init_rc_in()  [ArduCopter/radio.cpp:20]
       │    │    Brief: Initialize RC input channels
       │    │    │
       │    │    ├─> rc().get_roll_channel() / get_pitch_channel() / etc.
       │    │    │    └─ Get references to RC channels
       │    │    │
       │    │    ├─> channel_*->set_angle(ROLL_PITCH_YAW_INPUT_MAX)
       │    │    │    └─ Set angle limits for roll/pitch/yaw
       │    │    │
       │    │    ├─> channel_throttle->set_range(1000)
       │    │    │    └─ Set throttle range
       │    │    │
       │    │    └─> default_dead_zones()
       │    │         └─ Set default RC dead zones
       │    │
       │    ├─> allocate_motors()
       │    │    └─ Allocate motor class (Multicopter or Heli)
       │    │
       │    ├─> rc().convert_options() + rc().init()
       │    │    └─ Convert RC options and initialize RC system
       │    │
       │    ├─> init_rc_out()  [ArduCopter/radio.cpp:47]
       │    │    Brief: Initialize motor outputs
       │    │    │
       │    │    ├─> motors->init(frame_class, frame_type)
       │    │    │    └─ Initialize motor library with frame config
       │    │    │
       │    │    ├─> AP::srv().enable_aux_servos()
       │    │    │    └─ Enable auxiliary servo channels
       │    │    │
       │    │    ├─> motors->set_update_rate(rc_speed)
       │    │    │    └─ Set motor update rate from parameter
       │    │    │
       │    │    ├─> motors->convert_pwm_min_max_param()
       │    │    │    └─ Convert throttle range to motor PWM range
       │    │    │
       │    │    └─> SRV_Channels::update_aux_servo_function()
       │    │         └─ Update auxiliary channel functions
       │    │
       │    ├─> esc_calibration_startup_check()
       │    │    └─ Check if ESC calibration mode should be entered
       │    │
       │    ├─> gps.init()
       │    │    └─ Initialize GPS sensor
       │    │
       │    ├─> AP::compass().init()
       │    │    └─ Initialize compass/magnetometer
       │    │
       │    ├─> optflow.init()
       │    │    └─ Initialize optical flow sensor
       │    │
       │    ├─> camera_mount.init()
       │    │    └─ Initialize camera gimbal mount
       │    │
       │    ├─> camera.init()
       │    │    └─ Initialize camera system
       │    │
       │    ├─> barometer.calibrate()
       │    │    └─ Perform initial barometer calibration
       │    │
       │    ├─> init_rangefinder()
       │    │    └─ Initialize range finder sensors
       │    │
       │    ├─> g2.proximity.init()
       │    │    └─ Initialize proximity sensors
       │    │
       │    └─> init_precland()
       │         └─ Initialize precision landing
       │
       └─> check_firmware()
            └─ Verify firmware integrity

AP_HAL loop() [Runs continuously]
  └─> AP_Vehicle::loop()  [libraries/AP_Vehicle/AP_Vehicle.cpp:551]
       │   Brief: Main scheduler loop
       │
       └─> AP_Scheduler::loop()
            Brief: Execute scheduler tasks
```

---

## 2. MAIN LOOP EXECUTION PATH
### Scheduler.tick() flow with all FAST_TASK and scheduled task frequencies

```
AP_Vehicle::loop()  [libraries/AP_Vehicle/AP_Vehicle.cpp:551]
  └─> AP_Scheduler::loop()
       │
       └─> Execute FAST_TASKS (highest priority, called at main loop rate ~400Hz)
            │
            ├─> FAST_TASK: AP_InertialSensor::update()  [400Hz]
            │    Brief: Read IMU (accelerometer/gyroscope) data
            │    │
            │    └─> AP_InertialSensor internal update
            │         └─ Fetches raw accel/gyro from hardware
            │
            ├─> FAST_TASK: Copter::run_rate_controller_main()  [400Hz]
            │    Brief: Low-level rate controller (gyro stabilization)
            │    File: ArduCopter/Attitude.cpp:10
            │    │
            │    ├─> AP::scheduler().get_last_loop_time_s()
            │    │    └─ Get dt for this loop iteration
            │    │
            │    ├─> pos_control->set_dt_s(dt)
            │    │    └─ Update position controller dt
            │    │
            │    ├─> attitude_control->set_dt_s(dt)
            │    │    └─ Update attitude controller dt
            │    │
            │    ├─> motors->set_dt_s(dt)  [if not using rate thread]
            │    │    └─ Update motor controller dt
            │    │
            │    ├─> attitude_control->rate_controller_run()
            │    │    Brief: Execute gyro stabilization control loop
            │    │    └─ Calculates motor command from gyro error
            │    │
            │    └─> attitude_control->rate_controller_target_reset()
            │         └─ Reset target inputs for next iteration
            │
            ├─> FAST_TASK: Copter::motors_output_main()  [400Hz]
            │    File: ArduCopter/motors.cpp:120
            │    Brief: Send motor outputs
            │    │
            │    └─> Copter::motors_output(full_push=false)
            │         Brief: Format and send motor commands
            │         │
            │         ├─> SRV_Channels::calc_pwm()
            │         │    └─ Calculate all PWM outputs
            │         │
            │         ├─> AP::srv().cork()
            │         │    └─ Begin atomic servo update
            │         │
            │         ├─> SRV_Channels::output_ch_all()
            │         │    └─ Output all auxiliary channels
            │         │
            │         ├─> Update motor interlock state
            │         │    └─ Enable/disable motors based on conditions
            │         │
            │         ├─> flightmode->output_to_motors()  [if not motor test]
            │         │    Brief: Flight mode specific motor output
            │         │    Parameters: attitude_control outputs
            │         │    └─ Sends commands to AP_Motors library
            │         │
            │         └─> hal.rcout->push()
            │              └─ Send PWM to ESCs (1000-2000μs)
            │
            ├─> FAST_TASK: Copter::read_AHRS()  [400Hz]
            │    File: ArduCopter/Copter.cpp:899
            │    Brief: EKF state estimation update
            │    │
            │    └─> ahrs.update(skip_ins=true)
            │         Brief: Update AHRS (Attitude/Heading Reference System)
            │         │
            │         ├─> EKF2/EKF3 predict step
            │         │    └─ Propagate state with latest IMU data
            │         │
            │         ├─> EKF2/EKF3 update with available sensors
            │         │    ├─ GPS position/velocity updates
            │         │    ├─ Barometer altitude update
            │         │    ├─ Compass heading update
            │         │    ├─ Optical flow velocity update
            │         │    └─ Rangefinder altitude update
            │         │
            │         └─> Output:
            │              ├─ attitude (roll, pitch, yaw)
            │              ├─ position estimates
            │              └─ velocity estimates
            │
            ├─> FAST_TASK: Copter::read_inertia()  [400Hz]
            │    File: ArduCopter/inertia.cpp:4
            │    Brief: Inertial navigation - update vehicle position
            │    │
            │    ├─> pos_control->update_estimates()
            │    │    └─ Update altitude estimates (using barometer if high vibes)
            │    │
            │    ├─> ahrs.get_location(loc)
            │    │    └─ Get current lat/lng from EKF
            │    │
            │    └─> ahrs.get_relative_position_D_origin_float()
            │         └─ Get altitude above EKF origin
            │
            ├─> FAST_TASK: Copter::check_ekf_reset()  [400Hz]
            │    File: ArduCopter/ekf_check.cpp:248
            │    Brief: Check for EKF resets and adjust controls
            │    │
            │    ├─> ahrs.getLastYawResetAngle()
            │    │    └─ Detect if EKF reset yaw
            │    │
            │    └─> attitude_control->inertial_frame_reset()  [if reset detected]
            │         └─ Reset attitude controller targets after EKF reset
            │
            ├─> FAST_TASK: Copter::update_flight_mode()  [400Hz]
            │    File: ArduCopter/mode.cpp:410
            │    Brief: Execute flight mode logic
            │    │
            │    ├─> attitude_control->landed_gain_reduction()
            │    │    └─ Adjust PID gains when landed
            │    │
            │    └─> flightmode->run()
            │         Brief: Flight mode specific control algorithm
            │         Parameters: Current vehicle state, pilot input
            │         Returns: Attitude/throttle targets
            │         │
            │         ├─ For STABILIZE mode:
            │         │  ├─> get_pilot_desired_lean_angles()
            │         │  │    └─ Convert stick to lean angle targets
            │         │  │
            │         │  ├─> get_pilot_desired_yaw_rate()
            │         │  │    └─ Convert stick to yaw rate target
            │         │  │
            │         │  ├─> get_pilot_desired_throttle()
            │         │  │    └─ Get throttle from stick
            │         │  │
            │         │  ├─> attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw()
            │         │  │    └─ Command attitude controller
            │         │  │
            │         │  └─> attitude_control->set_throttle_out()
            │         │       └─ Set motor throttle
            │         │
            │         └─ For LOITER mode:
            │            ├─> loiter_nav->update()
            │            │    └─ Navigation controller to maintain position
            │            │
            │            ├─> pos_control->update_xy_controller()
            │            │    └─ Calculate position correction
            │            │
            │            ├─> attitude_control->input_attitude_roll_pitch_yaw()
            │            │    └─ Command attitude from position error
            │            │
            │            └─> attitude_control->set_throttle_out()
            │                 └─ Set throttle for altitude hold
            │
            ├─> FAST_TASK: Copter::update_home_from_EKF()  [400Hz]
            │    └─ Update home position if GPS lock obtained
            │
            ├─> FAST_TASK: Copter::update_land_and_crash_detectors()  [400Hz]
            │    └─ Detect landing and crash conditions
            │
            ├─> FAST_TASK: Copter::update_rangefinder_terrain_offset()  [400Hz]
            │    └─ Update terrain/surface tracking offset
            │
            └─> FAST_TASK: AP_Mount::update_fast()  [400Hz]
                 └─ Update camera gimbal at fast rate
            
            [End FAST_TASKS, now execute scheduled tasks in priority order]

       └─> Execute SCHEDULED_TASKS (variable rates, priorities 0-255)
            │
            ├─> SCHED_TASK: Copter::rc_loop()  [250Hz, priority 3]
            │    File: ArduCopter/Copter.cpp:573
            │    Brief: Read RC input from radio receiver
            │    │
            │    ├─> read_radio()
            │    │    Brief: Read RC channel values from receiver
            │    │    │
            │    │    └─ For each RC channel (roll, pitch, throttle, yaw):
            │    │         ├─ Read PWM value from receiver (1000-2000μs)
            │    │         ├─ Apply deadzone
            │    │         └─ Store in control_in variable
            │    │
            │    └─> rc().read_mode_switch()
            │         └─ Read flight mode switch position
            │
            ├─> SCHED_TASK: Copter::throttle_loop()  [50Hz, priority 6]
            │    File: ArduCopter/Copter.cpp:583
            │    Brief: Throttle-related updates
            │    │
            │    ├─> update_throttle_mix()
            │    │    └─ Update throttle priority vs attitude control
            │    │
            │    ├─> update_auto_armed()
            │    │    └─ Update auto-arming state
            │    │
            │    ├─> heli_update_rotor_speed_targets()  [if HELI_FRAME]
            │    │    └─ Update helicopter rotor RPM
            │    │
            │    ├─> heli_update_landing_swash()  [if HELI_FRAME]
            │    │    └─ Adjust swashplate during landing
            │    │
            │    └─> compensate_for_ground_effect()
            │         └─ Adjust throttle for ground effect
            │
            ├─> SCHED_TASK: AP_Fence::check()  [25Hz, priority 7]
            │    └─ Check geofence violations
            │
            ├─> SCHED_TASK: AP_GPS::update()  [50Hz, priority 9]
            │    Brief: Update GPS sensor
            │    │
            │    ├─> GPS device read()
            │    │    └─ Read raw data from GPS hardware
            │    │
            │    ├─> Parse GPS message
            │    │    └─ Extract lat, lng, altitude, velocity
            │    │
            │    └─ Update GPS instance variables
            │         └─ Available for EKF and navigation
            │
            ├─> SCHED_TASK: AP_OpticalFlow::update()  [200Hz, priority 12]
            │    └─ Update optical flow sensor
            │
            ├─> SCHED_TASK: Copter::update_batt_compass()  [10Hz, priority 15]
            │    Brief: Update battery and compass
            │    │
            │    ├─> battery.read()
            │    │    └─ Read battery voltage/current
            │    │
            │    └─> compass.read()
            │         └─ Read compass heading
            │
            ├─> SCHED_TASK: RC_Channels::read_aux_all()  [10Hz, priority 18]
            │    └─ Read all auxiliary RC channels
            │
            ├─> SCHED_TASK: Copter::auto_disarm_check()  [10Hz, priority 27]
            │    Brief: Auto-disarm timer check
            │    │
            │    └─> Check if vehicle should auto-disarm (landed, throttle low)
            │
            ├─> SCHED_TASK: Copter::read_rangefinder()  [20Hz, priority 33]
            │    └─ Update rangefinder/sonar altitude
            │
            ├─> SCHED_TASK: AP_Proximity::update()  [200Hz, priority 36]
            │    └─ Update proximity/obstacle avoidance sensors
            │
            ├─> SCHED_TASK: Copter::update_altitude()  [10Hz, priority 42]
            │    File: ArduCopter/Copter.cpp:906
            │    Brief: Update altitude from barometer
            │    │
            │    ├─> read_barometer()
            │    │    └─ Read barometer pressure and convert to altitude
            │    │
            │    └─ Write control tuning log
            │         └─ Log PID values for tuning
            │
            ├─> SCHED_TASK: Copter::run_nav_updates()  [50Hz, priority 45]
            │    └─ Update waypoint navigation
            │
            ├─> SCHED_TASK: Copter::update_throttle_hover()  [100Hz, priority 48]
            │    File: ArduCopter/Attitude.cpp:32
            │    Brief: Update estimated hover throttle
            │    │
            │    └─ Continuously estimate throttle needed for level hover
            │
            ├─> SCHED_TASK: ModeSmartRTL::save_position()  [3Hz, priority 51]
            │    └─ Save position history for Smart RTL
            │
            ├─> SCHED_TASK: AC_Sprayer::update()  [3Hz, priority 54]
            │    └─ Update sprayer system
            │
            ├─> SCHED_TASK: Copter::three_hz_loop()  [3Hz, priority 57]
            │    └─ 3Hz general updates
            │
            ├─> SCHED_TASK: Copter::update_precland()  [400Hz, priority 69]
            │    └─ Precision landing update
            │
            ├─> SCHED_TASK: Copter::one_hz_loop()  [1Hz, priority 81]
            │    Brief: 1Hz general updates
            │    │
            │    ├─> Check compass health
            │    ├─> Update system health
            │    └─ Other low-frequency checks
            │
            ├─> SCHED_TASK: Copter::ekf_check()  [10Hz, priority 84]
            │    File: ArduCopter/ekf_check.cpp
            │    Brief: Check EKF health and trigger failsafes
            │    │
            │    └─ Trigger EKF failsafe if needed
            │
            ├─> SCHED_TASK: Copter::check_vibration()  [10Hz, priority 87]
            │    File: ArduCopter/ekf_check.cpp:269
            │    Brief: Check for excessive vibration
            │    │
            │    └─ Enable vibration compensation if needed
            │
            ├─> SCHED_TASK: Copter::gpsglitch_check()  [10Hz, priority 90]
            │    File: ArduCopter/events.cpp:299
            │    └─ Detect GPS glitches
            │
            ├─> SCHED_TASK: Copter::takeoff_check()  [50Hz, priority 91]
            │    └─ Check takeoff conditions
            │
            ├─> SCHED_TASK: Copter::landinggear_update()  [10Hz, priority 93]
            │    └─ Update landing gear state
            │
            ├─> SCHED_TASK: GCS::update_receive()  [400Hz, priority 102]
            │    Brief: Receive MAVLink messages from GCS
            │    │
            │    ├─> Handle SET_MODE message
            │    ├─ Handle ARM/DISARM command
            │    ├─> Handle waypoint uploads
            │    └─ Handle various parameter updates
            │
            ├─> SCHED_TASK: GCS::update_send()  [400Hz, priority 105]
            │    Brief: Send telemetry to GCS
            │    │
            │    ├─> Send HEARTBEAT (1Hz)
            │    ├─> Send SYS_STATUS (1Hz)
            │    ├─> Send ATTITUDE (10Hz)
            │    ├─> Send GLOBAL_POSITION_INT (10Hz)
            │    ├─> Send NAV_CONTROLLER_OUTPUT (10Hz)
            │    └─ Send other telemetry packets
            │
            ├─> SCHED_TASK: AP_Mount::update()  [50Hz, priority 108]
            │    └─ Update camera gimbal stabilization
            │
            ├─> SCHED_TASK: AP_Camera::update()  [50Hz, priority 111]
            │    └─ Update camera triggers
            │
            ├─> SCHED_TASK: Copter::ten_hz_logging_loop()  [10Hz, priority 114]
            │    └─ Periodic data logging
            │
            ├─> SCHED_TASK: Copter::twentyfive_hz_logging()  [25Hz, priority 117]
            │    └─ Higher frequency logging
            │
            ├─> SCHED_TASK: AP_Logger::periodic_tasks()  [400Hz, priority 120]
            │    └─ Logger maintenance tasks
            │
            ├─> SCHED_TASK: AP_InertialSensor::periodic()  [400Hz, priority 123]
            │    └─ IMU maintenance tasks
            │
            ├─> SCHED_TASK: AP_Scheduler::update_logging()  [0.1Hz, priority 126]
            │    └─ Log scheduler performance data
            │
            └─> [Additional lower priority tasks...]
                 └─ User hooks, button updates, etc.
```

**Key Execution Order:**
- FAST_TASKS run in order listed above at ~400Hz
- Scheduled tasks execute in priority order (0=highest)
- One scheduled task per loop iteration (time-permitting)
- All sensors read, state estimated, control computed, outputs sent within ~2.5ms

---

## 3. SENSOR READING PATH
### INS update → AHRS → EKF → Flight mode

```
[START of every loop iteration]

1. AP_InertialSensor::update()  [FAST_TASK, ~400Hz]
   ├─ Read raw accelerometer data from MPU6000/ICM20689
   │  ├─ Read X, Y, Z acceleration values
   │  ├─ Apply calibration offsets
   │  └─ Apply scaling to get m/s²
   │
   ├─ Read raw gyroscope data from IMU
   │  ├─ Read roll, pitch, yaw rotation rates
   │  ├─ Apply temperature compensation
   │  └─ Apply scaling to get rad/s
   │
   └─ Store in AP_InertialSensor internal buffers
      └─ Available for DCM/AHRS processing

2. Copter::run_rate_controller_main()  [FAST_TASK, ~400Hz]
   └─ Uses latest IMU data for gyro stabilization

3. Copter::read_AHRS()  [FAST_TASK, ~400Hz]
   File: ArduCopter/Copter.cpp:899
   │
   └─> ahrs.update(skip_ins=true)
       │   Brief: EKF2 or EKF3 state estimation
       │   Inputs: Latest IMU, GPS, compass, barometer, optical flow
       │
       ├─ EKF Predict Step (every iteration):
       │  ├─> Integrate IMU accelerations
       │  │   └─ Update velocity: v_new = v_old + accel * dt
       │  │
       │  ├─> Integrate velocity
       │  │   └─ Update position: p_new = p_old + vel * dt
       │  │
       │  └─> Integrate gyro rates
       │      └─ Update attitude: q_new = q_old + angvel * dt
       │
       ├─ EKF Update Step (when sensor data available):
       │  │
       │  ├─> If GPS available (50Hz):
       │  │    ├─ GPS position innovation: z = gps_pos - ekf_pos
       │  │    ├─ Kalman gain calculation
       │  │    ├─ State correction: x = x + K * z
       │  │    └─ Covariance update: P = (I - K*H)*P
       │  │
       │  ├─> If barometer available (10Hz):
       │  │    ├─ Altitude innovation: z = baro_alt - ekf_alt
       │  │    ├─ Kalman gain for altitude channel
       │  │    └─ Update altitude and vertical velocity
       │  │
       │  ├─> If compass available (10Hz):
       │  │    ├─ Compass heading innovation
       │  │    └─ Update yaw estimate
       │  │
       │  ├─> If optical flow available (200Hz):
       │  │    ├─ Use flow for position correction when GPS denied
       │  │    └─ Better horizontal position accuracy
       │  │
       │  └─> If rangefinder available (20Hz):
       │       └─ Update altitude when near ground
       │
       └─ Output state vector:
          ├─ Position: (lat, lng, alt)
          ├─ Velocity: (vx, vy, vz)
          ├─ Attitude: (roll, pitch, yaw) via quaternion
          ├─ Gyro bias estimates
          ├─ Accel bias estimates
          └─ Covariance matrix (uncertainty estimates)

4. Copter::read_inertia()  [FAST_TASK, ~400Hz]
   File: ArduCopter/inertia.cpp:4
   │
   ├─> pos_control->update_estimates()
   │    └─ Update altitude estimate (use barometer if high vibes, else EKF)
   │
   ├─> ahrs.get_location()
   │    └─ Extract lat/lng from EKF state vector
   │
   └─> ahrs.get_relative_position_D_origin()
        └─ Extract vertical position (altitude above origin)

5. Sensor Reading Sequence (Throughout loop):
   │
   ├─ AP_GPS::update()  [SCHED_TASK, 50Hz]
   │  ├─> Read UART from GPS receiver
   │  ├─> Parse NMEA/UBlox protocol
   │  ├─> Extract: lat, lng, altitude, hdop, vdop, velocity
   │  └─> Mark as available for EKF
   │
   ├─ AP_Compass::read()  [10Hz]
   │  ├─> Read magnetometer I2C/SPI
   │  ├─> Apply calibration matrix (from compass learning)
   │  ├─> Get heading
   │  └─> Provide to EKF for heading estimate
   │
   ├─ Barometer reading  [10Hz in update_altitude()]
   │  ├─> Read pressure sensor
   │  ├─> Convert pressure to altitude using standard atmosphere
   │  └─ Provide altitude for EKF
   │
   ├─ AP_OpticalFlow::update()  [200Hz]
   │  ├─> Read optical flow sensor (PX4Flow, Dualsense, etc)
   │  ├─> Get optical flow rates (rad/s)
   │  └─> Use for position hold in GPS-denied flight
   │
   ├─ Rangefinder reading  [20Hz in read_rangefinder()]
   │  ├─> Read ultrasonic/lidar distance
   │  ├─> Convert to altitude above ground
   │  └─ Use for landing and low-altitude control
   │
   └─ Battery monitoring  [10Hz in update_batt_compass()]
      ├─> Read battery voltage
      ├─> Read current sensor
      └─ Calculate consumed energy and time-to-depleted

6. Data Flow Through EKF:
   
   Physical Sensors
      │
      ├─> Gyroscope (raw)
      │   └─> IMU measurement model
      │       └─> Angular velocity estimate
      │
      ├─> Accelerometer (raw)
      │   └─> IMU measurement model  
      │       └─> Specific force (accel - gravity)
      │
      ├─> GPS (position, velocity)
      │   └─> GPS measurement model
      │       └─> Position/velocity estimate
      │
      ├─> Barometer (pressure)
      │   └─> Altitude measurement model
      │       └─> Altitude/vertical velocity estimate
      │
      ├─> Compass (magnetic field)
      │   └─> Magnetometer measurement model
      │       └─> Heading estimate
      │
      ├─> Optical Flow (pixel motion)
      │   └─> Optical flow measurement model
      │       └─> Horizontal velocity estimate
      │
      └─> Rangefinder (distance)
          └─> Altitude measurement model
              └─> Low-altitude estimate

   [All inputs fed into Extended Kalman Filter]
   
   EKF2/EKF3 State Vector:
   ├─ Position (lat, lng, alt) - WGS84 coordinates
   ├─ Velocity (vx, vy, vz) - m/s in NED frame
   ├─ Attitude quaternion - orientation
   ├─ Gyro bias estimates - account for IMU drift
   ├─ Accel bias estimates
   ├─ Wind estimates - for airspeed correction
   └─ Covariance matrix - measure of uncertainty

7. Output to Flight Mode:
   
   Copter::update_flight_mode()  [FAST_TASK, ~400Hz]
   │
   └─> flightmode->run()
       │
       ├─ Access EKF outputs via:
       │  ├─> ahrs.get_roll_rad() / get_pitch_rad() / get_yaw_rad()
       │  ├─> ahrs.get_position() (lat/lng)
       │  ├─> ahrs.get_velocity_NED()
       │  └─> ahrs.get_gyro()
       │
       └─> For STABILIZE mode: converts stick input to attitude targets
          For LOITER mode: uses position error to calculate attitude targets
          For ALT_HOLD: uses velocity to maintain altitude
```

---

## 4. CONTROL LOOP PATH
### Pilot input → Flight mode → Attitude control → Motor mixing → PWM output

### PATH A: STABILIZE MODE (Simple Pilot-Direct Control)

```
INPUT STAGE:
├─> Copter::rc_loop()  [250Hz]
│   File: ArduCopter/Copter.cpp:573
│   │
│   ├─> read_radio()
│   │    Brief: Read RC receiver PWM values
│   │    │
│   │    └─ For each channel:
│   │         ├─ Read raw PWM (1000-2000 μs)
│   │         ├─ Apply deadzone
│   │         ├─ Constrain to range
│   │         └─ Store in RC channel object
│   │
│   └─> rc().read_mode_switch()
│        └─ Read flight mode selection switch

PILOT DESIRED CALCULATIONS:
├─> ModeStabilize::run()  [~400Hz]
│   File: ArduCopter/mode_stabilize.cpp:9
│   │
│   ├─> update_simple_mode()
│   │    └─ If SIMPLE/SUPER_SIMPLE mode: rotate stick inputs to absolute reference frame
│   │
│   ├─> get_pilot_desired_lean_angles_rad()
│   │    Brief: Convert stick to lean angle target (RATE mode behavior)
│   │    Inputs: RC roll/pitch values (−4500 to +4500 = −45° to +45°)
│   │    │
│   │    ├─ Roll: stick_in * max_lean_angle / max_stick = target_roll_rad
│   │    ├─ Pitch: stick_in * max_lean_angle / max_stick = target_pitch_rad
│   │    │
│   │    └─ Output: lean angle targets (−radians(45) to +radians(45))
│   │
│   ├─> get_pilot_desired_yaw_rate_rads()
│   │    Brief: Convert yaw stick to rotation rate target
│   │    Inputs: RC yaw value
│   │    │
│   │    └─ Output: yaw_rate rad/s (±radians(360) per second based on param)
│   │
│   └─> get_pilot_desired_throttle()
│        Brief: Get raw throttle stick value
│        Inputs: RC throttle (0-1000)
│        └─ Output: throttle 0.0 to 1.0

ATTITUDE CONTROL STAGE:
├─> attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad()
│   Brief: Attitude controller: converts attitude targets to motor commands
│   File: AC_AttitudeControl library
│   Inputs:
│   ├─ target_roll_rad (from pilot, −45° to +45°)
│   ├─ target_pitch_rad (from pilot, −45° to +45°)
│   ├─ target_yaw_rate_rads (from pilot spin rate)
│   └─ dt from scheduler
│   
│   Operation [at 400Hz]:
│   ├─> Read current attitude from EKF:
│   │    ├─ ahrs.get_roll_rad()
│   │    ├─ ahrs.get_pitch_rad()
│   │    ├─ ahrs.get_yaw_rad()
│   │    └─ ahrs.get_gyro() (current rotation rates)
│   │
│   ├─ ANGLE CONTROLLER (Roll & Pitch) [PID]:
│   │    │
│   │    ├─ Roll error: e_roll = target_roll − actual_roll
│   │    ├─ Pitch error: e_pitch = target_pitch − actual_pitch
│   │    │
│   │    ├─ P term: K_p * error
│   │    │    └─ Proportional correction to error
│   │    │
│   │    ├─ I term: accumulated error * K_i
│   │    │    └─ Removes steady-state error (level trim)
│   │    │
│   │    ├─ D term: -K_d * gyro_rate
│   │    │    └─ Damping to slow down rotation
│   │    │
│   │    └─ Output: roll_rate_target, pitch_rate_target (rad/s)
│   │         └─ Pass to rate controller
│   │
│   ├─ RATE CONTROLLER (Roll, Pitch, Yaw) [PID]:
│   │    [Executes in attitude_control->rate_controller_run()]
│   │    │
│   │    ├─ For each axis (roll, pitch, yaw):
│   │    │    │
│   │    │    ├─ Rate error: e = target_rate − actual_rate (from gyro)
│   │    │    │
│   │    │    ├─ P term: K_p_rate * error
│   │    │    ├─ I term: accumulated_error * K_i_rate (max-limited)
│   │    │    ├─ D term: -K_d_rate * rate_derivative
│   │    │    └─ F term: feedforward of target rate
│   │    │
│   │    └─ Output: axis_command [±4500]
│   │         └─ Motor command magnitude
│   │
│   └─ Store rate controller outputs (roll_cmd, pitch_cmd, yaw_cmd)

THROTTLE CONTROL STAGE:
├─> attitude_control->set_throttle_out()
│   Brief: Convert throttle stick to motor throttle command
│   Inputs:
│   ├─ pilot_desired_throttle (0.0 to 1.0 from stick)
│   └─ true/false (pass-through throttle)
│   │
│   ├─ For STABILIZE (manual throttle):
│   │    └─ throttle_out = pilot_desired_throttle
│   │
│   └─ Store throttle command for mixing

MOTOR MIXING STAGE:
├─> attitude_control->get_motor_output_stabilizing_roll() / pitch() / yaw()
│   Brief: Attitude control outputs for motor mixing
│   │
│   └─ Returns:
│       ├─ roll_control [−4500 to +4500]
│       ├─ pitch_control [−4500 to +4500]
│       ├─ yaw_control [−4500 to +4500]
│       └─ throttle_out [0.0 to 1.0]

├─> AP_Motors::update_motor_mix()
│   Brief: Mix attitude commands and throttle into motor signals
│   Inputs:
│   ├─ roll_control, pitch_control, yaw_control (attitude commands)
│   ├─ throttle_out (0.0 to 1.0)
│   └─ motor configuration (quad, hexa, octa, etc)
│   
│   For QuadCopter X configuration:
│   ├─ Motor 1 (front-right):
│   │    output = throttle + roll − pitch + yaw
│   │
│   ├─ Motor 2 (rear-left):
│   │    output = throttle + roll + pitch − yaw
│   │
│   ├─ Motor 3 (front-left):
│   │    output = throttle − roll − pitch − yaw
│   │
│   └─ Motor 4 (rear-right):
│        output = throttle − roll + pitch + yaw
│   
│   Motor mixing algorithm:
│   │
│   ├─ Mix based on frame geometry:
│   │    └─ Moment arm relationships between motors and desired torques
│   │
│   ├─ Apply limits:
│   │    ├─ Minimum throttle (ground idle): 1000 PWM
│   │    ├─ Maximum throttle (100%): 2000 PWM
│   │    └─ Constrain each motor independently
│   │
│   ├─ Apply slew limiting (smoothing):
│   │    └─ Limit rate of change to avoid mechanical shock
│   │
│   └─ Output: 4 motor commands [1000-2000 PWM] or [0-1000 range]

PWM OUTPUT STAGE:
├─> Copter::motors_output_main()  [~400Hz]
│   File: ArduCopter/motors.cpp:120
│   │
│   └─> Copter::motors_output(full_push)
│        │
│        ├─> SRV_Channels::calc_pwm()
│        │    └─ Calculate PWM values for all channels
│        │
│        ├─> motors->set_interlock() / armed state check
│        │    └─ Enable motor output if armed and interlock active
│        │
│        ├─> flightmode->output_to_motors()
│        │    └─ For STABILIZE: calls motors->set_desired_spool_state() and outputs
│        │
│        └─> hal.rcout->push()
│             Brief: Send PWM to hardware
│             │
│             ├─ For each motor (1-4):
│             │  └─ Send PWM pulse width to ESC via pin
│             │
│             └─ ESCs interpret PWM and:
│                  ├─ Convert 1000-2000 μs to throttle command (0-100%)
│                  ├─ Commutate BLDC motor to desired speed
│                  └─ Feed back motor voltage/current

COMPLETE CONTROL LOOP TIMING (STABILIZE MODE):
┌──────────────────────────────────────────────────────────────┐
│ t=0ms:    rc_loop() reads stick input                        │
│ t=0-1ms:  read_AHRS() updates attitude estimate             │
│ t=1-2ms:  run_rate_controller_main() computes motor commands │
│ t=2-2.5ms: motors_output_main() sends PWM to ESCs            │
│ t=2.5ms:  ESC receives PWM command                           │
│ t=2.5-5ms: Motor spins up/down to commanded throttle        │
│ t=5ms:    Next control loop iteration begins                │
└──────────────────────────────────────────────────────────────┘
Latency: pilot stick → motor response ≈ 5-10ms
```

### PATH B: LOITER MODE (Autonomous Position Hold)

```
[Note: LOITER mode builds on STABILIZE control loop, adds position control]

POSITION ESTIMATION:
├─ read_AHRS()  [400Hz]
│  └─ EKF provides: position (lat/lng/alt), velocity (vx/vy/vz)
│
└─ read_inertia()  [400Hz]
   └─ Update current_loc with latest position from EKF

NAVIGATION STAGE:
├─> Copter::run_nav_updates()  [50Hz]
│   Brief: Update waypoint navigation
│   │
│   └─ For LOITER mode, loiter_nav manages target position

├─> AC_Loiter::update()
│   Brief: Loiter navigation controller
│   Inputs:
│   ├─ Current position (lat/lng/alt)
│   ├─ Target position (loiter center)
│   └─ Maximum speed/acceleration parameters
│   │
│   ├─ Calculate position error:
│   │    ├─ error_north = target_lat − current_lat (meters)
│   │    └─ error_east = target_lng − current_lng (meters)
│   │
│   ├─ Velocity feedforward from pilot input:
│   │    ├─ If pilot moving stick, add to desired velocity
│   │    └─ Allow pilot to command within loiter circle
│   │
│   └─ Output:
│        └─ Desired acceleration in North and East directions (m/s²)

POSITION CONTROL STAGE:
├─> AC_PosControl::update_xy_controller()
│   Brief: XY position controller (PID for horizontal position)
│   Inputs:
│   ├─ Desired acceleration from loiter_nav (m/s²)
│   ├─ Current velocity from EKF (vx/vy m/s)
│   ├─ dt from scheduler
│   └─ Position control gains (K_p, K_i, K_d)
│   │
│   ├─ Velocity error:
│   │    ├─ North velocity error = desired_vel_north − actual_vel_north
│   │    └─ East velocity error = desired_vel_east − actual_vel_east
│   │
│   ├─ Velocity controller (PID):
│   │    ├─ P: K_p * velocity_error
│   │    ├─ I: accumulated_error * K_i (limits in m/s²)
│   │    └─ D: -K_d * acceleration (optional)
│   │
│   └─ Output: desired_accel (m/s²) → convert to lean angles

ATTITUDE GENERATION FROM POSITION:
├─> attitudes_from_accel()
│   Brief: Calculate attitude targets from desired acceleration
│   Inputs:
│   ├─ desired_accel_north (m/s²)
│   ├─ desired_accel_east (m/s²)
│   ├─ current_yaw (rad)
│   └─ g (gravity = 9.81 m/s²)
│   │
│   ├─ Convert body frame acceleration to tilt angle:
│   │    ├─ For small angles: tan(θ) ≈ accel / g
│   │    ├─ target_roll = atan2(accel_east, g)
│   │    └─ target_pitch = atan2(accel_north, g)
│   │
│   └─ Limit to max lean angle (45° typical)

ALTITUDE CONTROL STAGE:
├─> AC_PosControl::update_altitude_controller()
│   Brief: Vertical position controller
│   Inputs:
│   ├─ Desired altitude
│   ├─ Current altitude from EKF
│   ├─ Pilot throttle input
│   └─ Position control gains
│   │
│   ├─ Altitude error:
│   │    └─ alt_error = desired_alt − current_alt
│   │
│   ├─ Vertical velocity controller:
│   │    └─ Converts altitude error to desired vertical velocity
│   │
│   ├─ Acceleration controller:
│   │    └─ Converts velocity error to desired vertical acceleration
│   │
│   └─ Output: throttle_correction (combined with hover throttle)

ATTITUDE CONTROL (Same as STABILIZE):
├─> attitude_control->input_attitude_roll_pitch_yaw()
│   Inputs:
│   ├─ target_roll from position controller
│   ├─ target_pitch from position controller
│   ├─ target_yaw from pilot or auto-yaw
│   └─ throttle_out from altitude controller
│   │
│   └─ [Same PID loops as STABILIZE mode]
│        ├─ Angle controller → rate targets
│        └─ Rate controller → motor commands

COMPLETE LOITER CONTROL LOOP:
┌────────────────────────────────────────────────────────────────┐
│ t=0ms:    read_AHRS() updates position/velocity/attitude      │
│ t=1ms:    loiter_nav->update() calculates nav acceleration    │
│ t=2ms:    pos_control->update_xy() calculates attitude targets│
│ t=3ms:    attitude_control->input() calculates rate targets   │
│ t=4ms:    run_rate_controller_main() generates motor commands │
│ t=4.5ms:  motors_output_main() sends PWM to ESCs              │
│ t=5ms:    Next iteration                                      │
└────────────────────────────────────────────────────────────────┘

Position error correction bandwidth ≈ 1-2 Hz (slower than attitude control)
Allows smooth position following without jerky movements
```

---

## 5. FAILSAFE TRIGGER PATH
### RC loss detection → Failsafe trigger → Mode switch → Safe action

```
RC LOSS DETECTION:
├─> RC_Channel_Copter monitoring (continuous, ~400Hz)
│   File: (part of RC_Channel class)
│   │
│   └─ Each RC_Channel tracks:
│       ├─ consecutive_valid_count (increments on valid PWM)
│       └─ failsafe_pin value (if defined)
│
├─> RC_Channels::has_valid_input()  [checked every loop]
│   Brief: Determine if RC receiver is healthy
│   │
│   ├─ Valid if:
│   │  ├─ At least one RC channel received valid signal in last 150ms
│   │  └─ All channels in valid range (900-2100 μs or configured min/max)
│   │
│   └─ Invalid if:
│       ├─ All channels lost for > 150ms (failsafe_short_timeout)
│       └─ Loss persists > FS_LONG_TIMEOUT

FAILSAFE STATE MACHINE:
├─> Input: RC link status (valid or invalid)
│   └─ Transition detection happens in main loop
│
├─ Valid RC → Invalid RC transition:
│   File: (RC failsafe check integrated into events/scheduler)
│   │
│   ├─ On first detection of lost RC:
│   │  ├─> Log failsafe event
│   │  ├─> Set failsafe.radio = true
│   │  └─> Call failsafe_radio_on_event()
│   │
│   └─ failsafe_radio_on_event()  [ArduCopter/events.cpp:13]
│       Brief: Handle RC loss
│       │
│       ├─ Determine desired action based on FS_THR_ENABLE parameter:
│       │  ├─ FS_THR_DISABLED: do nothing
│       │  ├─ FS_THR_ENABLED_ALWAYS_LAND: switch to LAND mode
│       │  ├─ FS_THR_ENABLED_ALWAYS_RTL: switch to RTL mode
│       │  ├─ FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL: try SmartRTL, fallback to RTL
│       │  ├─ FS_THR_ENABLED_AUTO_RTL_OR_RTL: if in Auto mission, land start; else RTL
│       │  └─ FS_THR_ENABLED_BRAKE_OR_LAND: switch to BRAKE then land
│       │
│       ├─ Condition checks:
│       │  ├─ If landed: disarm motors, don't change mode
│       │  ├─ If in LAND mode and FS_OPTIONS::CONTINUE_IF_LANDING: stay in LAND
│       │  ├─ If in AUTO and FS_OPTIONS::RC_CONTINUE_IF_AUTO: continue mission
│       │  └─ If in GUIDED and FS_OPTIONS::RC_CONTINUE_IF_GUIDED: stay in GUIDED
│       │
│       └─ Call do_failsafe_action(desired_action)
│            See failsafe action handler below

├─ Invalid RC → Valid RC transition:
│   │
│   └─ failsafe_radio_off_event()  [ArduCopter/events.cpp:82]
│       Brief: Handle RC recovery
│       │
│       ├─ No automatic mode change (pilot can recover)
│       ├─ Clear RC override flags
│       └─ Send "Radio Failsafe Cleared" message to GCS

FAILSAFE ACTION HANDLER:
├─> Copter::do_failsafe_action()  [ArduCopter/events.cpp:474]
│   File: ArduCopter/events.cpp:474
│   Inputs:
│   ├─ desired_action: NONE, LAND, RTL, SMARTRTL, TERMINATE, etc.
│   └─ reason: RADIO_FAILSAFE, GCS_FAILSAFE, BATTERY_FAILSAFE, etc.
│   │
│   ├─ Switch based on desired_action:
│   │
│   ├─ Case FailsafeAction::LAND:
│   │  └─> set_mode_land_with_pause(reason)
│   │      Brief: Switch to LAND mode with optional pause
│   │      │
│   │      └─> Copter::set_mode(Mode::Number::LAND, reason)
│   │          [See mode switch path below]
│   │
│   ├─ Case FailsafeAction::RTL:
│   │  └─> set_mode_RTL_or_land_with_pause(reason)
│   │      Brief: Try RTL, fallback to LAND
│   │      │
│   │      ├─ If GPS valid and position_ok():
│   │      │  └─> set_mode(Mode::Number::RTL, reason)
│   │      │
│   │      └─ Else:
│   │          └─> set_mode(Mode::Number::LAND, reason)
│   │
│   ├─ Case FailsafeAction::SMARTRTL:
│   │  └─> set_mode_SmartRTL_or_RTL(reason)
│   │      Brief: Try SmartRTL, fallback to RTL
│   │      │
│   │      └─> Checks SmartRTL history availability
│   │
│   ├─ Case FailsafeAction::AUTO_DO_LAND_START:
│   │  └─> set_mode_auto_do_land_start_or_RTL(reason)
│   │      Brief: Jump to landing sequence if in AUTO, else RTL
│   │
│   └─ Case FailsafeAction::BRAKE_LAND:
│       └─> set_mode_brake_or_land_with_pause(reason)
│           Brief: Switch to BRAKE (stop all movement) then land
│
│   After mode change:
│   └─ If FS_OPTION::RELEASE_GRIPPER set:
│       └─> gripper.release()

GPS LOSS DETECTION:
├─> AP_GPS::update()  [50Hz]
│   Brief: Monitor GPS health
│   │
│   └─ Tracks:
│       ├─ Signal quality (number of satellites)
│       ├─ HDOP (horizontal dilution of precision)
│       ├─ Fix type (no fix, 2D, 3D)
│       └─ Age of last fix
│
├─> Copter::gpsglitch_check()  [10Hz]
│   File: ArduCopter/events.cpp:299
│   Brief: Detect sudden GPS jumps (glitches)
│   │
│   ├─ Tracks suspected GPS glitch:
│   │  └─ sudden jump in position > threshold
│   │
│   ├─ If glitch detected for > glitch_time threshold:
│   │  └─ Set failsafe.radio = true (triggers RTL if enabled)
│   │
│   └─ When GPS recovers:
│       └─ Clear glitch status

EKF FAILURE DETECTION:
├─> AP_Scheduler main loop calls EKF update [400Hz]
│   │
│   └─ EKF2/EKF3 detects internal consistency failures:
│       ├─ Excessive innovation (measurement disagreement)
│       ├─ Covariance matrix divergence
│       ├─ Gyro/accel bias estimates out of bounds
│       └─ Variance checks (position, velocity, angle)

├─> Copter::ekf_check()  [10Hz]
│   File: ArduCopter/ekf_check.cpp
│   Brief: Monitor EKF health
│   │
│   ├─ Checks EKF failsafe flags:
│   │  └─ ahrs.initialised()
│   │  └─ EKF innovations within bounds
│   │  └─ Position/velocity variance not excessive
│   │
│   ├─ If EKF failsafe triggered:
│   │  │
│   │  ├─ Set failsafe.ekf = true
│   │  ├─ Log event
│   │  │
│   │  └─ Determine action based on FS_EKF_ACTION:
│   │      ├─ Land immediately
│   │      ├─ Switch to manual mode
│   │      ├─ Switch to ALT_HOLD
│   │      └─ Continue (for testing only)
│   │
│   └─ Call set_mode() if action requires mode change

BATTERY FAILSAFE:
├─> AP_BattMonitor::update_logging()  [~1Hz]
│   Brief: Monitor battery voltage and current
│   │
│   ├─ Checks thresholds:
│   │  ├─ Critical voltage (FS_BATT_VOLTAGE): immediate action
│   │  ├─ Low voltage (FS_BATT_VOLTAGE_LOW): warning
│   │  └─ Time until depleted (estimated from consumption rate)
│   │
│   ├─ If critical battery detected:
│   │  │
│   │  └─ Call handle_battery_failsafe()
│   │      File: ArduCopter/events.cpp:99
│   │      Inputs:
│   │      ├─ type_str: "Battery Critical" or "Battery Low"
│   │      └─ action: failsafe action (from BATT_FS_XXX_ACT parameter)
│   │      │
│   │      ├─ If landed: disarm motors immediately
│   │      │
│   │      ├─ If in LAND mode and FS_OPTION::CONTINUE_IF_LANDING: continue
│   │      │
│   │      └─ Else: call do_failsafe_action(action)
│   │           Typically: LAND mode

GCS (GROUND CONTROL STATION) FAILSAFE:
├─> Copter::failsafe_gcs_check()  [10Hz task]
│   File: ArduCopter/events.cpp:126
│   Brief: Monitor GCS link status
│   │
│   ├─ Tracks:
│   │  └─ Time since last GCS heartbeat
│   │
│   ├─ If GCS timeout exceeded (FS_GCS_TIMEOUT parameter):
│   │  │
│   │  ├─ Set failsafe.gcs = true
│   │  │
│   │  └─ Call failsafe_gcs_on_event()
│   │      Brief: Handle GCS loss
│   │      │
│   │      ├─ Determine action from FS_GCS parameter:
│   │      │  ├─ Disabled: do nothing
│   │      │  ├─ Continue mission: stay in AUTO
│   │      │  ├─ RTL: return to launch
│   │      │  └─ Land: switch to LAND mode
│   │      │
│   │      ├─ Condition checks:
│   │      │  ├─ If landed: disarm
│   │      │  ├─ If in LAND and continue option set: stay in LAND
│   │      │  └─ If in AUTO and continue option set: continue mission
│   │      │
│   │      └─ do_failsafe_action(desired_action)
│   │
│   └─ When GCS recovers:
│       └─> failsafe_gcs_off_event(): clear failsafe status

TERRAIN FAILSAFE:
├─> Copter::failsafe_terrain_check()  [10Hz]
│   File: ArduCopter/events.cpp:243
│   Brief: Detect missing terrain data in terrain-relative flight
│   │
│   ├─ Only active in modes that require terrain (LAND, LOITER terrain-relative)
│   │
│   ├─ Tracks:
│   │  └─ Duration of terrain data unavailability
│   │
│   └─ If timeout exceeded (FS_TERRAIN_TIMEOUT):
│       └─> failsafe_terrain_on_event()
│           └─> Land immediately (terrain mode is dangerous without data)

MAIN LOOP LOCKUP FAILSAFE:
├─> Copter::failsafe_check()  [1kHz timer interrupt]
│   File: ArduCopter/failsafe.cpp:35
│   Brief: Detect main loop hang/crash
│   │
│   ├─ Registered as failsafe check callback from hardware timer
│   │ (Called from interrupt, not from main loop)
│   │
│   ├─ Monitors:
│   │  └─ scheduler.ticks() value changes
│   │     (increments only when main loop runs)
│   │
│   ├─ If no change for 2 seconds:
│   │  │
│   │  ├─ Main loop is hung/crashed
│   │  ├─ Set in_failsafe = true
│   │  ├─ Call motors->output_min() (reduce motor power)
│   │  └─ Log CPU failsafe error
│   │
│   ├─ Every 1 second in failsafe:
│   │  └─> motors->armed(false) - fully disarm motors
│   │       └─ Send minimum PWM (1000 μs) to ESCs
│   │
│   └─ When main loop recovers:
│       └─> Clear in_failsafe flag and log recovery

FAILSAFE PRIORITY:
Most critical → Least critical
1. Main loop lockup (immediate motor cutoff via interrupt)
2. EKF failure (immediate mode change or disarm)
3. Battery critical (immediate land/RTL based on config)
4. RC loss (land/RTL based on config, only if armed)
5. GCS loss (RTL if enabled, lower priority than RC)
6. Terrain data loss (only in terrain-relative modes)
7. GPS glitch detection (affects navigation modes)
```

---

## 6. MODE SWITCH PATH
### set_mode() → exit old mode → init new mode → run new mode

```
TRIGGER POINTS FOR MODE CHANGE:

A. Pilot Mode Switch Input:
   ├─> rc_loop()  [250Hz]
   │   └─> rc().read_mode_switch()
   │       └─ Read pilot's 6-position mode switch
   │
   └─> Pilot moves switch to new position
       └─> Events: Call set_mode(new_mode, ModeReason::RC_COMMAND)

B. GCS Mode Change Command:
   ├─> GCS::update_receive()  [400Hz]
   │   └─ Receive MAVLink SET_MODE message
   │       └─> set_mode(mode_num, ModeReason::GCS_COMMAND)

C. Failsafe Mode Change:
   └─> [From failsafe handler]
       └─> set_mode(mode_num, ModeReason::RADIO_FAILSAFE/etc)

D. Automatic Mode Transition:
   └─> Within flight mode
       └─> mode->set_desired_mode() or similar

MODE SWITCH EXECUTION:
├─> Copter::set_mode(Mode::Number new_mode, ModeReason reason)
│   File: ArduCopter/mode.cpp:233
│   Brief: Change flight mode with extensive checks
│   │
│   ├─ STEP 1: Basic Validity Checks
│   │  │
│   │  ├─ Already in requested mode?
│   │  │  └─ Update reason, send GCS message, return success
│   │  │
│   │  ├─ GCS mode change disabled?
│   │  │  ├─ If reason == GCS_COMMAND
│   │  │  └─ Check FS_GCS_ENABLE parameter
│   │  │      └─ Return false if disabled
│   │  │
│   │  └─ Get mode object from mode number
│   │      ├─ Call mode_from_mode_num(mode)
│   │      └─ Returns pointer to mode instance or nullptr
│   │
│   ├─ STEP 2: Pre-Check Conditions
│   │  │
│   │  ├─ Determine ignore_checks flag:
│   │  │  └─ true if disarmed (allow any mode when disarmed)
│   │  │  └─ false if armed (enforce restrictions)
│   │  │
│   │  ├─ Helicopter rotor check [if HELI_FRAME]:
│   │  │  ├─ If mode requires automatic throttle
│   │  │  └─ And rotor not yet at full RPM
│   │  │      └─ Reject mode change (rotor_runup_complete check)
│   │  │
│   │  ├─ Throttle check [if MULTICOPTER]:
│   │  │  ├─ If switching from auto to manual throttle
│   │  │  └─ With high throttle stick value (risk of sudden climb)
│   │  │      └─ Reject mode change
│   │  │
│   │  ├─ Position requirement check:
│   │  │  ├─ If mode requires GPS/position
│   │  │  └─ And position_ok() == false
│   │  │      └─ Reject mode change (e.g., LOITER, AUTO, RTL)
│   │  │
│   │  ├─ Altitude requirement check:
│   │  │  ├─ If switching from manual to automatic throttle
│   │  │  └─ And ekf_alt_ok() == false
│   │  │      └─ Reject mode change (no altitude estimate)
│   │  │
│   │  ├─ Geofence check [if AP_FENCE_ENABLED]:
│   │  │  ├─ If fence is breached
│   │  │  └─ And fence recovery mode change disabled
│   │  │      └─ Reject mode change
│   │  │
│   │  └─ RC failsafe check:
│   │      ├─ If in RC failsafe
│   │      └─ And new mode doesn't allow entry during failsafe
│   │          └─ Reject mode change
│   │
│   ├─ STEP 3: Initialize New Mode
│   │  │
│   │  └─> new_flightmode->init(ignore_checks)
│   │      Brief: Mode-specific initialization
│   │      │
│   │      ├─ For STABILIZE::init():
│   │      │  └─ No special initialization needed
│   │      │
│   │      ├─ For LOITER::init():
│   │      │  │
│   │      │  ├─> loiter_nav->init_target()
│   │      │  │    └─ Set loiter center to current position
│   │      │  │
│   │      │  ├─> Update pilot input acceleration targets
│   │      │  │
│   │      │  └─> Initialize position controller
│   │      │       └─> pos_control->init_U_controller() for altitude
│   │      │
│   │      ├─ For ALT_HOLD::init():
│   │      │  │
│   │      │  ├─> pos_control->init_U_controller()
│   │      │  │    └─ Initialize vertical position controller
│   │      │  │
│   │      │  ├─> Set target altitude to current altitude
│   │      │  │
│   │      │  └─> Set hover throttle as initial throttle output
│   │      │
│   │      ├─ For AUTO::init():
│   │      │  │
│   │      │  ├─> mission.reset()
│   │      │  │    └─ Reset mission to first command
│   │      │  │
│   │      │  ├─> wp_nav->init()
│   │      │  │    └─ Initialize waypoint navigation
│   │      │  │
│   │      │  └─> pos_control->init() for all axes
│   │      │
│   │      ├─ For RTL::init():
│   │      │  │
│   │      │  ├─> rtl_compute_return_speed()
│   │      │  │    └─ Calculate return speed based on distance to home
│   │      │  │
│   │      │  ├─> wp_nav->init_loiter_target(home_location)
│   │      │  │    └─ Set target waypoint to home
│   │      │  │
│   │      │  └─> pos_control->init() for position control
│   │      │
│   │      └─ For LAND::init():
│   │          │
│   │          ├─> pos_control->init_U_controller() for descent control
│   │          │
│   │          ├─> Set initial desired descent rate
│   │          │
│   │          └─> Initialize landing detection state
│   │
│   ├─ STEP 4: Exit Old Mode
│   │  │
│   │  └─> Copter::exit_mode(old_flightmode, new_flightmode)
│   │      File: ArduCopter/mode.cpp:421
│   │      Brief: Cleanup from previous mode
│   │      │
│   │      ├─> Throttle smoothing transition:
│   │      │  │
│   │      │  ├─ If switching from manual → automatic throttle
│   │      │  └─ And armed and flying
│   │      │      │
│   │      │      └─ set_accel_throttle_I_from_pilot_throttle()
│   │      │          Brief: Smoothly transition pilot throttle to autopilot
│   │      │          │
│   │      │          ├─ Get pilot's current throttle output
│   │      │          ├─ Calculate difference from hover throttle
│   │      │          └─> Set position controller's vertical accel integrator
│   │      │              └─ Prevents sudden jump in throttle
│   │      │
│   │      ├─ Takeoff cancellation:
│   │      │  └─> old_flightmode->takeoff_stop()
│   │      │      └─ Interrupt any in-progress auto-takeoff
│   │      │
│   │      └─> Optional mode-specific cleanup:
│   │          ├─ OLD: mission->stop()
│   │          ├─ OLD: wp_nav->clear_wp_destination()
│   │          └─ Other mode-specific cleanup
│   │
│   ├─ STEP 5: Update Flight Mode Pointer
│   │  │
│   │  └─> flightmode = new_flightmode
│   │      └─ Global pointer now references new mode
│   │
│   ├─ STEP 6: Update System State
│   │  │
│   │  ├─> control_mode_reason = reason
│   │  │    └─ Track why mode changed (debug)
│   │  │
│   │  ├─> logger.Write_Mode(mode_num, reason)  [if HAL_LOGGING_ENABLED]
│   │  │    └─ Log mode change event to dataflash
│   │  │
│   │  ├─> gcs().send_message(MSG_HEARTBEAT)
│   │  │    └─ Notify GCS that mode changed
│   │  │
│   │  ├─> ADSB update [if HAL_ADSB_ENABLED]:
│   │  │    └─ Inform ADSB system if switching to/from auto modes
│   │  │
│   │  ├─> Geofence manual recovery [if AP_FENCE_ENABLED]:
│   │  │    └─ Clear fence recovery mode if mode changed
│   │  │
│   │  └─> Camera auto-mode update [if AP_CAMERA_ENABLED]:
│   │      └─ Inform camera if in AUTO mode
│   │
│   ├─ STEP 7: Update Control Parameters
│   │  │
│   │  ├─> attitude_control->set_roll_pitch_rate_tc(...)
│   │  │    └─ Set rate shaping time constant for ACRO/SPORT modes
│   │  │
│   │  └─> attitude_control->set_yaw_rate_tc(...)
│   │       └─ Set yaw rate time constant (affects smoothness)
│   │
│   ├─ STEP 8: Notify User
│   │  │
│   │  ├─> notify_flight_mode()
│   │  │    └─ Set LED pattern for new mode
│   │  │
│   │  └─> AP_Notify::events.user_mode_change = 1
│   │       └─ Make buzzer sound for mode change
│   │
│   └─ Return true (success)

IMMEDIATE EFFECT AFTER MODE CHANGE:
├─ update_flight_mode()  [FAST_TASK, next execution]
│  │
│  └─> flightmode->run()
│      └─ Now executes NEW mode logic
│
└─ New mode's control algorithm active next loop iteration

EXAMPLE MODE CHANGE SEQUENCE (STABILIZE → LOITER):
┌─────────────────────────────────────────────────────────┐
│ t=0ms:    Pilot moves mode switch to LOITER              │
│ t=2.5ms:  rc_loop() reads new switch position            │
│ t=3.5ms:  Events detected mode change, call set_mode()  │
│           ├─ Check position_ok() - must have GPS         │
│           ├─ Call ModeLoiter::init(false)                │
│           │  ├─ Set loiter center to current position    │
│           │  └─ Init position controller                 │
│           ├─ Call exit_mode(STABILIZE, LOITER)           │
│           │  └─ Smooth throttle transition               │
│           ├─ flightmode = &mode_loiter                   │
│           ├─ Send mode change to GCS                     │
│           └─ Play mode change buzzer                     │
│ t=5ms:    Next update_flight_mode()                      │
│           └─> ModeLoiter::run() active                   │
│ t=5.5ms:  Quad starts holding position using GPS         │
│           ├─ loiter_nav calculates position error        │
│           ├─ pos_control generates attitude correction   │
│           └─ attitude_control mixes to motors            │
│ Result:   Smooth transition from manual to auto position │
└─────────────────────────────────────────────────────────┘
```

---

## 7. ARMING PATH
### Pre-arm checks → arm() → motor output enabled

```
PRE-ARM CHECKS (Can fail and prevent arming):
├─> AP_Arming_Copter::pre_arm_checks(display_failure=true)
│   File: ArduCopter/AP_Arming_Copter.cpp:8
│   Brief: Verify system is safe to arm
│   Returns: true if all checks pass, false if any check fails
│   │
│   └─> AP_Arming_Copter::run_pre_arm_checks(display_failure)
│       File: ArduCopter/AP_Arming_Copter.cpp:17
│       Brief: Execute all pre-arm checks
│       │
│       ├─ AIRFRAME CHECKS:
│       │  ├─> Check frame type is supported
│       │  ├─> Check motor count matches frame
│       │  └─> Check motor PWM output configured
│       │
│       ├─ PARAMETER CHECKS:
│       │  ├─> Check critical parameters are in valid ranges
│       │  ├─> Check PID gains configured
│       │  └─> Check failsafe parameters sensible
│       │
│       ├─ SENSOR CHECKS (via AP_Arming base class):
│       │  ├─ inertial_nav_check():
│       │  │  ├─> Check INS (accelerometer/gyroscope) calibrated
│       │  │  └─> Run accelerometer self-test
│       │  │
│       │  ├─ compass_check():
│       │  │  ├─> Check compass detected
│       │  │  └─> Check compass heading stable
│       │  │
│       │  ├─ gps_check():
│       │  │  ├─> Check GPS enabled (if required by mode)
│       │  │  ├─> Check GPS has 3D fix
│       │  │  └─> Check number of satellites sufficient
│       │  │
│       │  └─ rc_check():
│       │      ├─> Check RC channels detected
│       │      └─> Check RC input in valid range
│       │
│       ├─ ATTITUDE CHECK [Copter-specific]:
│       │  └─> pre_arm_ekf_attitude_check()
│       │      File: ArduCopter/AP_Arming_Copter.cpp:410
│       │      Brief: Verify EKF attitude agrees with accelerometers
│       │      │
│       │      ├─> Get EKF attitude estimate
│       │      ├─> Get accelerometer-based attitude
│       │      ├─> Compare: must be within 30°
│       │      └─ If diverged: likely calibration issue or environmental magnet
│       │
│       ├─ MOTOR/ESC CHECKS:
│       │  ├─> Check motor commands respond
│       │  └─> Check ESC calibration valid
│       │
│       ├─ BATTERY CHECK:
│       │  ├─> Check battery voltage adequate
│       │  └─> Check battery not in failsafe
│       │
│       ├─ RCMAP CHECK:
│       │  └─> Check RC channel mapping configured
│       │
│       └─ Return: true if all checks pass, false otherwise
│           └─ If failed, display reason to GCS
│
├─ If any check fails:
│  │
│  ├─ Display error message: "Pre-arm check failed: <reason>"
│  ├─> AP_Notify::flags.arming_failed = true (LED blinks red)
│  └─ Return: Arming denied

PRE-ARM CHECKS PASSED - PROCEED TO ARM:
├─> Copter::arm_motors(AP_Arming::Method method, bool do_checks=true)
│   [Called from GCS, RC switch, or external command]
│   │
│   └─> AP_Arming_Copter::arm(method, do_checks)
│       File: ArduCopter/AP_Arming_Copter.cpp:675
│       Brief: Final arming sequence (no returns without risk)
│       │
│       ├─ GUARD: Prevent re-entrance
│       │  │
│       │  ├─ Static flag: in_arm_motors
│       │  └─ If already running: return false (prevent recursive calls)
│       │
│       ├─ Check if already armed
│       │  └─ If motors->armed() == true: return true (already armed)
│       │
│       ├─ Call AP_Arming::arm() [base class]
│       │  Brief: Final generic checks
│       │  │
│       │  ├─ If do_checks == true:
│       │  │  └─> arm_checks(method) [additional checks]
│       │  │      ├─> Check system ready
│       │  │      ├─> Check not in failsafe
│       │  │      └─> Check method-specific requirements
│       │  │
│       │  └─ If all checks pass:
│       │      └─> Return true (arming approved)
│       │
│       ├─ If AP_Arming::arm() returned false:
│       │  ├─> AP_Notify::events.arming_failed = true
│       │  └─> Return false (arming aborted)
│       │
│       ├─ LOGGING SETUP:
│       │  └─> AP::logger().set_vehicle_armed(true)
│       │      └─ Prepare dataflash logging for flight data
│       │
│       ├─ FAILSAFE MANAGEMENT:
│       │  │
│       │  ├─> failsafe_disable()
│       │  │    └─ Disable main loop watchdog during arming sequence
│       │  │        (complex initialization may cause delays)
│       │  │
│       │  └─ [Later] failsafe_enable()
│       │       └─ Re-enable watchdog after safe state reached
│       │
│       ├─ NOTIFICATION SETUP:
│       │  │
│       │  ├─> AP_Notify::flags.armed = true
│       │  │    └─ Set global armed flag
│       │  │
│       │  └─> For i=0 to 10:
│       │      └─> AP::notify().update()
│       │          └─ Call multiple times to ensure LED/buzzer activated
│       │
│       ├─ BEARING INITIALIZATION:
│       │  │
│       │  ├─> init_simple_bearing()
│       │  │    └─ Record current heading as "simple mode" reference
│       │  │
│       │  ├─> initial_armed_bearing_rad = ahrs.get_yaw_rad()
│       │  │    └─ Save yaw for absolute reference (SUPER_SIMPLE mode)
│       │  │
│       │  └─> update_super_simple_bearing(false)
│       │       └─ Initialize bearing relative to home for super-simple mode
│       │
│       ├─ HOME POSITION SETUP:
│       │  │
│       │  └─ Check if home position already set:
│       │      │
│       │      ├─ If home NOT set (first flight):
│       │      │  ├─> ahrs.resetHeightDatum()
│       │      │  │    └─ EKF uses current altitude as altitude reference
│       │      │  │
│       │      │  ├─> arming_altitude_m = 0
│       │      │  │    └─ Arm height is zero (unknown home altitude)
│       │      │  │
│       │      │  └─ Log EKF_ALT_RESET event
│       │      │
│       │      └─ If home already set but NOT locked (can be updated):
│       │          ├─> set_home_to_current_location(false)
│       │          │    └─ Update home to current position (normal arm)
│       │          │
│       │          └─> Get current altitude as arming altitude
│       │              └─ Track altitude at time of arming
│       │
│       ├─ SMARTRTL PREPARATION:
│       │  └─> g2.smart_rtl.set_home(position_ok())
│       │      └─ Set SmartRTL home point
│       │
│       ├─ HARDWARE SETUP:
│       │  │
│       │  ├─> hal.util->set_soft_armed(true)
│       │  │    └─ Notify HAL that system is armed
│       │  │
│       │  └─> sprayer.test_pump(false)  [if HAL_SPRAYER_ENABLED]
│       │       └─ Turn off sprayer test mode if active
│       │
│       ├─ MOTOR OUTPUT INITIALIZATION:
│       │  │
│       │  ├─> motors->output_min()
│       │  │    Brief: Send minimum throttle (1000 PWM) to all motors
│       │  │    Purpose: ESC initialization (no motor spin yet)
│       │  │    │
│       │  │    └─ All 4 motors receive 1000 μs PWM command
│       │  │
│       │  └─ [Short delay for ESC to recognize min command]
│       │
│       ├─ FINAL MOTOR ARM:
│       │  │
│       │  └─> motors->armed(true)
│       │      Brief: Arm motors - now responds to throttle commands
│       │      └─ ESCs enable gate drivers and power MOSFETs
│       │          └─ Motors can now spin per throttle command
│       │
│       ├─ FLIGHT LOGGING:
│       │  │
│       │  ├─> AP::logger().Write_Mode(mode_number, control_mode_reason)
│       │  │    └─ Log initial flight mode at arming
│       │  │
│       │  └─ Dataflash now recording all flight data
│       │
│       ├─ SCHEDULER OPTIMIZATION:
│       │  └─> AP::scheduler().perf_info.ignore_this_loop()
│       │       └─ Don't count arming overhead in performance metrics
│       │
│       ├─ ARM-TIME TRACKING:
│       │  │
│       │  ├─> arm_time_ms = millis()
│       │  │    └─ Record exact time of arming for various calculations
│       │  │
│       │  └─> ap.in_arming_delay = true
│       │       └─ Set flag indicating motors haven't spun up yet
│       │           (prevent autonomous takeoff during 3-second delay)
│       │
│       ├─ AIRMODE INITIALIZATION:
│       │  │
│       │  └─ ap.armed_with_airmode_switch = false
│       │       └─ Assume armed with normal switch (not airmode switch)
│       │           (may be overridden in switch.cpp if airmode switch used)
│       │
│       └─ Return true
│            └─ ARMING SUCCESSFUL!

CRITICAL SAFETY FEATURES DURING ARMING:
├─ Minimum throttle output:
│  └─ 1000 PWM (0% throttle) sent to motors during init
│     └─ Prevents motor spin-up even if throttle stick moved
│
├─ Arming delay (3 seconds):
│  └─ Even when armed, throttle commands ignored for first 3 seconds
│     └─ Provides safety window for pilot/observers
│
├─ Interlock switch:
│  ├─ If safety switch enabled:
│  │  └─ Motor output only active if physical switch engaged
│  │
│  └─ If motor interlock aux function configured:
│     └─ Motors only spin if interlock switch active
│
└─ Failsafe watchdog:
   └─ Re-enabled immediately after arm completes
      └─ If main loop hangs, motors disarm within 2 seconds

POST-ARMING STATE:
├─ Motors armed: motors->armed() == true
├─ ESCs powered and responsive
├─ In arming delay: ap.in_arming_delay == true (for 3 seconds)
├─ Minimum throttle being sent: 1000 PWM
├─ Flight mode active and running
├─ All sensors being updated continuously
└─ System ready for pilot to command takeoff or mode-specific operation

COMPLETE ARMING SEQUENCE TIMELINE:
┌──────────────────────────────────────────────────────────────┐
│ t=-100ms: Pilot moves arm switch (or GCS sends ARM command)  │
│ t=0ms:    rc_loop() detects arm signal                      │
│ t=1ms:    Events detected arm request                        │
│           ├─> Check pre-arm conditions                      │
│           ├─> Motors pass arm_checks()                      │
│           └─> Call motors->arm()                            │
│ t=2ms:    motors->armed(true)                               │
│           ├─ ESCs activated                                 │
│           └─ 1000 μs PWM sent to all motors                 │
│ t=3ms:    ap.in_arming_delay = true                         │
│           └─ 3-second safety delay starts                   │
│ t=50ms:   update_flight_mode() active with armed state      │
│           └─ Flight mode logic running but minimal response │
│ t=1000ms: Should_disarm_check sees low throttle             │
│           └─ Auto-disarm timer starts                       │
│ t=3000ms: Arming delay expires                              │
│           └─> ap.in_arming_delay = false                    │
│           └─ Pilot can now control throttle fully           │
│ t=3500ms: Pilot raises throttle stick                       │
│           └─ Motors spin up smoothly toward desired RPM     │
│ t>4000ms: Quad takes off when thrust exceeds weight         │
│           └─ Flight in progress!                            │
└──────────────────────────────────────────────────────────────┘

Typical sequence: 3-4 seconds from arm command to liftoff
Safety mechanisms: 4 independent safety layers prevent accidental spin-up
```

---

## SUMMARY TABLE: KEY EXECUTION RATES

| **Function** | **File** | **Rate** | **Purpose** |
|---|---|---|---|
| **INS Update** | AP_InertialSensor | 400Hz (FAST) | Raw IMU data |
| **EKF Update** | AP_AHRS (EKF2/3) | 400Hz (FAST) | State estimation |
| **Rate Controller** | AC_AttitudeControl | 400Hz (FAST) | Gyro stab. |
| **Motor Output** | AP_Motors | 400Hz (FAST) | PWM to ESCs |
| **Flight Mode Run** | Mode class | 400Hz (FAST) | Control algorithm |
| **RC Read** | Copter/radio | 250Hz (Sched) | Pilot input |
| **GPS Update** | AP_GPS | 50Hz (Sched) | Position |
| **Throttle Loop** | Copter | 50Hz (Sched) | Throttle logic |
| **Batt/Compass** | Copter | 10Hz (Sched) | Sensors |
| **EKF Health Check** | Copter | 10Hz (Sched) | Failsafe detect |
| **GCS Telemetry Send** | GCS_MAVLINK | 50Hz (Sched) | Downlink |
| **GCS Cmd Receive** | GCS_MAVLINK | 400Hz (Sched) | Uplink |
| **Failsafe Check** | Copter | 1kHz (Interrupt) | Watchdog |

---

## FUNCTION LOCATION REFERENCE

```
startup/init: init_ardupilot() [system.cpp:16]
main loop: AP_Vehicle::loop() [libraries/AP_Vehicle.cpp:551]
scheduler: AP_Scheduler::loop() [libraries/AP_Scheduler/AP_Scheduler.cpp]

sensor paths:
  - INS: AP_InertialSensor::update()
  - AHRS/EKF: ahrs.update() [AP_AHRS, EKF2/3]
  - read_inertia(): ArduCopter/inertia.cpp:4
  - read_AHRS(): ArduCopter/Copter.cpp:899

control paths:
  - update_flight_mode(): ArduCopter/mode.cpp:410
  - run_rate_controller_main(): ArduCopter/Attitude.cpp:10
  - motors_output_main(): ArduCopter/motors.cpp:120
  - ModeStabilize::run(): ArduCopter/mode_stabilize.cpp:9
  - ModeLoiter::run(): ArduCopter/mode_loiter.cpp:80

mode operations:
  - set_mode(): ArduCopter/mode.cpp:233
  - exit_mode(): ArduCopter/mode.cpp:421

failsafes:
  - RC failsafe: failsafe_radio_on_event() [events.cpp:13]
  - Battery: handle_battery_failsafe() [events.cpp:99]
  - GCS: failsafe_gcs_check() [events.cpp:126]
  - EKF: ekf_check() [ekf_check.cpp]
  - Main loop: failsafe_check() [failsafe.cpp:35]

arming:
  - pre_arm_checks(): AP_Arming_Copter.cpp:8
  - arm(): AP_Arming_Copter.cpp:675
  - disarm(): AP_Arming_Copter.cpp:790
```

---

## KEY ARCHITECTURAL POINTS

1. **Scheduler-Driven Design**: All code runs through AP_Scheduler::loop()
   - Fast tasks run every iteration (~400Hz)
   - Scheduled tasks interleaved with priorities

2. **Sensor-to-Control Pipeline**: Multi-stage state estimation
   - IMU → EKF → Flight mode control → Motor mixing → PWM output
   - Latency: ~5-10ms total

3. **Extensive Pre-Flight Checks**: ArduCopter won't arm without:
   - Calibrated sensors (accel, compass)
   - Valid GPS (if required by mode)
   - Reasonable parameter values
   - Attitude agreement between EKF and IMU

4. **Multi-Layer Failsafes**:
   - RC loss → configurable action (Land/RTL/etc)
   - Battery low → land or RTL
   - GCS loss → RTL if enabled
   - EKF failure → mode change
   - Main loop hang → disarm via interrupt

5. **Mode Architecture**: Flexible, with init/run/exit pattern
   - Each mode has full control when active
   - Can be exited cleanly for mode changes or failsafes
   - Pilot input, position control, and autonomous sequences all integrated

