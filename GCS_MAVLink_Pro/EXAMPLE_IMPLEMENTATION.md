# Example: Minimal Custom GCS Implementation

This document provides a complete, working example of creating a custom GCS module for a hypothetical "SimpleRover" vehicle.

## Example Structure

```
SimpleRover/
├── GCS_SimpleRover.h              # GCS manager class header
├── GCS_SimpleRover.cpp            # GCS manager implementation
├── GCS_MAVLink_SimpleRover.h      # Channel class header
├── GCS_MAVLink_SimpleRover.cpp    # Channel implementation
├── SimpleRover.h                  # Main vehicle class
└── SimpleRover.cpp                # Main vehicle implementation
```

---

## File 1: GCS_SimpleRover.h

```cpp
#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_SimpleRover.h"

class GCS_SimpleRover : public GCS
{
    friend class SimpleRover;  // Allow SimpleRover to access protected members

public:
    // This macro generates two overloaded chan() methods
    // that return GCS_MAVLINK_SimpleRover* instead of base GCS_MAVLINK*
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_SimpleRover);

    // Vehicle identification - required overrides
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;
    const char* frame_string() const override { return "SimpleRover"; }

    // Vehicle state
    bool vehicle_initialised() const override;

    // Sensor status updates - add rover-specific sensors
    void update_vehicle_sensor_status_flags(void) override;

protected:
    // Timing constraint - how much time must remain in main loop
    // before we're allowed to send MAVLink messages
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 200;  // Need 200 microseconds minimum
    }

    // Factory method to create channel instances
    GCS_MAVLINK_SimpleRover *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_SimpleRover(uart);
    }
};
```

---

## File 2: GCS_SimpleRover.cpp

```cpp
#include "GCS_SimpleRover.h"
#include "SimpleRover.h"

// Implement required pure virtual functions

uint32_t GCS_SimpleRover::custom_mode() const
{
    // Return the current flight mode as a number
    // This appears in the GCS as the custom mode
    return (uint32_t)simplerover.control_mode;
}

MAV_TYPE GCS_SimpleRover::frame_type() const
{
    // Tell the GCS what type of vehicle we are
    return MAV_TYPE_GROUND_ROVER;
}

bool GCS_SimpleRover::vehicle_initialised() const
{
    // Has the vehicle finished initialization?
    return simplerover.is_initialized;
}

void GCS_SimpleRover::update_vehicle_sensor_status_flags(void)
{
    // This is called by the base class to update sensor status
    // Add rover-specific sensors to the status bitmasks

    // Motors - always present on a rover
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    // Motors enabled if we're not in a failsafe
    if (!simplerover.in_failsafe()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // Motors healthy if they're responding
    if (simplerover.motors_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // Add wheel encoders if we have them
    if (simplerover.wheel_encoders_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;

        if (simplerover.wheel_encoders_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        }

        if (simplerover.wheel_encoders_healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL;
        }
    }

    // Add lidar if present
    #if AP_RANGEFINDER_ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;

        if (simplerover.rangefinder_enabled) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;

            if (rangefinder->has_data_orient(ROTATION_PITCH_270)) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
            }
        }
    }
    #endif
}
```

---

## File 3: GCS_MAVLink_SimpleRover.h

```cpp
#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_SimpleRover : public GCS_MAVLINK
{
public:
    // Inherit base class constructor
    using GCS_MAVLINK::GCS_MAVLINK;

protected:
    // ========================================
    // REQUIRED PURE VIRTUAL OVERRIDES
    // ========================================

    // Send navigation controller output
    void send_nav_controller_output() const override;

    // Send PID tuning data
    void send_pid_tuning() override;

    // Return MAVLink base mode flags
    uint8_t base_mode() const override;

    // Return MAVLink system state
    MAV_STATE vehicle_system_status() const override;

    // ========================================
    // OPTIONAL VIRTUAL OVERRIDES
    // ========================================

    // VFR_HUD message components
    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    // Message handling
    void handle_message(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;

    // Command handling
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;

    // ========================================
    // CUSTOM HANDLERS
    // ========================================

private:
    // Handle rover-specific messages
    void handle_set_target_speed(const mavlink_message_t &msg);
    void handle_set_steering_angle(const mavlink_message_t &msg);

    // Handle rover-specific commands
    MAV_RESULT handle_cmd_set_max_speed(const mavlink_command_int_t &packet);
    MAV_RESULT handle_cmd_wheel_calibration(const mavlink_command_int_t &packet);

    // Send rover-specific messages
    void send_wheel_encoder_data();
    void send_rover_status();
};
```

---

## File 4: GCS_MAVLink_SimpleRover.cpp

```cpp
#include "SimpleRover.h"
#include "GCS_MAVLink_SimpleRover.h"

// ========================================
// REQUIRED OVERRIDES - IMPLEMENTATION
// ========================================

uint8_t GCS_MAVLINK_SimpleRover::base_mode() const
{
    uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    // Set ARMED flag if motors are armed
    if (simplerover.motors_armed()) {
        mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // Set MANUAL flag if in manual control mode
    if (simplerover.control_mode == SimpleRover::Mode::MANUAL) {
        mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

    // Set AUTO flag if in autonomous mode
    if (simplerover.control_mode == SimpleRover::Mode::AUTO) {
        mode |= MAV_MODE_FLAG_AUTO_ENABLED;
        mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

    // Set STABILIZE flag if we're doing any stabilization
    if (simplerover.control_mode != SimpleRover::Mode::MANUAL) {
        mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    return mode;
}

MAV_STATE GCS_MAVLINK_SimpleRover::vehicle_system_status() const
{
    // Boot phase
    if (!simplerover.is_initialized) {
        return MAV_STATE_BOOT;
    }

    // Critical failure
    if (simplerover.in_failsafe()) {
        return MAV_STATE_CRITICAL;
    }

    // Active (motors armed and moving)
    if (simplerover.motors_armed()) {
        return MAV_STATE_ACTIVE;
    }

    // Standby (initialized but not armed)
    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_SimpleRover::send_nav_controller_output() const
{
    if (!simplerover.is_initialized) {
        return;
    }

    // NAV_CONTROLLER_OUTPUT message provides navigation status
    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // nav_roll (degrees) - not used for rovers
        0,  // nav_pitch (degrees) - not used for rovers
        simplerover.nav_controller.target_heading_deg(),  // nav_bearing
        simplerover.nav_controller.target_bearing_deg(),  // target_bearing
        simplerover.nav_controller.distance_to_target_m(), // wp_dist (meters)
        0,  // alt_error (meters) - not used for rovers
        simplerover.get_speed_error(),  // aspd_error
        simplerover.nav_controller.crosstrack_error_m()   // xtrack_error
    );
}

void GCS_MAVLINK_SimpleRover::send_pid_tuning()
{
    // Send steering PID tuning data if enabled
    if (simplerover.gcs_pid_mask & (1 << 0)) {
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }

        const AP_PIDInfo &pid = simplerover.steering_controller.get_pid_info();

        mavlink_msg_pid_tuning_send(
            chan,
            PID_TUNING_STEERING,  // axis
            pid.target,           // desired
            pid.actual,           // achieved
            pid.FF,               // FF term
            pid.P,                // P term
            pid.I,                // I term
            pid.D,                // D term
            pid.slew_rate,        // slew rate
            pid.Dmod              // D term modifier
        );
    }

    // Send throttle PID tuning data if enabled
    if (simplerover.gcs_pid_mask & (1 << 1)) {
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }

        const AP_PIDInfo &pid = simplerover.throttle_controller.get_pid_info();

        mavlink_msg_pid_tuning_send(
            chan,
            PID_TUNING_THROTTLE,  // axis
            pid.target,
            pid.actual,
            pid.FF,
            pid.P,
            pid.I,
            pid.D,
            pid.slew_rate,
            pid.Dmod
        );
    }
}

// ========================================
// VFR_HUD OVERRIDES
// ========================================

float GCS_MAVLINK_SimpleRover::vfr_hud_airspeed() const
{
    // For a rover, "airspeed" is actually ground speed
    return simplerover.get_speed_ms();
}

int16_t GCS_MAVLINK_SimpleRover::vfr_hud_throttle() const
{
    // Return throttle as percentage 0-100
    return simplerover.get_throttle_percent();
}

float GCS_MAVLINK_SimpleRover::vfr_hud_alt() const
{
    // For a rover, altitude is just GPS altitude
    return AP::gps().location().alt * 0.01f;  // cm to meters
}

// ========================================
// MESSAGE HANDLING
// ========================================

void GCS_MAVLINK_SimpleRover::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    // Example: Custom message to set target speed
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
        handle_set_target_speed(msg);
        break;

    // Let base class handle everything else
    default:
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

void GCS_MAVLINK_SimpleRover::handle_set_target_speed(const mavlink_message_t &msg)
{
    // Decode message
    mavlink_set_position_target_local_ned_t packet;
    mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

    // Check if in correct mode
    if (simplerover.control_mode != SimpleRover::Mode::GUIDED) {
        send_text(MAV_SEVERITY_WARNING, "Not in GUIDED mode");
        return;
    }

    // Extract velocity command (vx = forward speed)
    float target_speed_ms = packet.vx;

    // Validate
    if (fabsf(target_speed_ms) > simplerover.max_speed_ms) {
        send_text(MAV_SEVERITY_WARNING, "Speed %.1f exceeds max %.1f",
                 target_speed_ms, simplerover.max_speed_ms);
        return;
    }

    // Apply command
    simplerover.set_target_speed(target_speed_ms);

    send_text(MAV_SEVERITY_INFO, "Target speed set to %.1f m/s", target_speed_ms);
}

// ========================================
// MESSAGE SENDING
// ========================================

bool GCS_MAVLINK_SimpleRover::try_send_message(enum ap_message id)
{
    switch(id) {

    // Example: Custom periodic message
    case MSG_WHEEL_ENCODERS:
        CHECK_PAYLOAD_SIZE(WHEEL_DISTANCE);
        send_wheel_encoder_data();
        break;

    default:
        // Let base class handle it
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}

void GCS_MAVLINK_SimpleRover::send_wheel_encoder_data()
{
    // Example of sending a MAVLink message with rover-specific data

    float left_distance = simplerover.wheel_encoders.left_distance_m();
    float right_distance = simplerover.wheel_encoders.right_distance_m();

    // Use WHEEL_DISTANCE message
    double distances[2] = { left_distance, right_distance };

    mavlink_msg_wheel_distance_send(
        chan,
        AP_HAL::millis() * 1000ULL,  // time_usec
        2,                            // count (2 wheels)
        distances                     // distance array
    );
}

// ========================================
// COMMAND HANDLING
// ========================================

MAV_RESULT GCS_MAVLINK_SimpleRover::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    switch (packet.command) {

    // Example: Custom command to set max speed
    case MAV_CMD_DO_CHANGE_SPEED:
        return handle_cmd_set_max_speed(packet);

    // Custom command for wheel calibration
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        // param6 = wheel encoder calibration
        if (packet.z > 0) {
            return handle_cmd_wheel_calibration(packet);
        }
        // Fall through to base class
        break;

    default:
        break;
    }

    // Always call base class for unhandled commands
    return GCS_MAVLINK::handle_command_int_packet(packet, msg);
}

MAV_RESULT GCS_MAVLINK_SimpleRover::handle_cmd_set_max_speed(
    const mavlink_command_int_t &packet)
{
    // param2 = speed in m/s
    float new_max_speed = packet.param2;

    // Validate
    if (new_max_speed < 0.1f || new_max_speed > 20.0f) {
        send_text(MAV_SEVERITY_WARNING, "Invalid max speed: %.1f", new_max_speed);
        return MAV_RESULT_DENIED;
    }

    // Apply
    simplerover.max_speed_ms = new_max_speed;

    send_text(MAV_SEVERITY_INFO, "Max speed set to %.1f m/s", new_max_speed);

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_SimpleRover::handle_cmd_wheel_calibration(
    const mavlink_command_int_t &packet)
{
    // Check if vehicle is stationary
    if (simplerover.get_speed_ms() > 0.1f) {
        send_text(MAV_SEVERITY_WARNING, "Vehicle must be stationary");
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // Start calibration
    simplerover.wheel_encoders.start_calibration();

    send_text(MAV_SEVERITY_INFO, "Wheel encoder calibration started");

    return MAV_RESULT_ACCEPTED;
}
```

---

## File 5: SimpleRover.h (Excerpt)

```cpp
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
// ... other includes ...

#include "GCS_SimpleRover.h"

class SimpleRover
{
public:
    SimpleRover();

    // Control modes
    enum class Mode : uint8_t {
        MANUAL = 0,
        ACRO = 1,
        STEERING = 2,
        HOLD = 3,
        AUTO = 4,
        GUIDED = 5,
        RTL = 6,
    };

    // Main control loops
    void setup();
    void loop();
    void fast_loop();    // 100 Hz
    void main_loop();    // 50 Hz
    void slow_loop();    // 10 Hz

    // GCS interface
    GCS_SimpleRover gcs_instance;
    GCS_SimpleRover &gcs() { return gcs_instance; }

    // State
    bool is_initialized;
    Mode control_mode;
    uint8_t gcs_pid_mask;  // Bitmask for PID tuning messages

    // Vehicle capabilities
    bool motors_armed() const { return _armed; }
    bool motors_healthy() const { return _motors_healthy; }
    bool in_failsafe() const { return _in_failsafe; }
    float get_speed_ms() const { return _current_speed_ms; }
    int16_t get_throttle_percent() const { return _throttle_pct; }

    // Navigation
    struct {
        float target_heading_deg() const { return _target_heading; }
        float target_bearing_deg() const { return _target_bearing; }
        float distance_to_target_m() const { return _distance_to_target; }
        float crosstrack_error_m() const { return _crosstrack_error; }

    private:
        float _target_heading;
        float _target_bearing;
        float _distance_to_target;
        float _crosstrack_error;
    } nav_controller;

    // Controllers (simplified for example)
    struct {
        AP_PIDInfo get_pid_info() const { return _info; }
    private:
        AP_PIDInfo _info;
    } steering_controller, throttle_controller;

    // Wheel encoders
    struct {
        float left_distance_m() const { return _left_dist; }
        float right_distance_m() const { return _right_dist; }
        void start_calibration() { _calibrating = true; }
    private:
        float _left_dist;
        float _right_dist;
        bool _calibrating;
    } wheel_encoders;

    // Settings
    float max_speed_ms;
    bool rangefinder_enabled;

    // Methods
    void set_target_speed(float speed_ms);
    float get_speed_error() const;
    bool wheel_encoders_present() const;
    bool wheel_encoders_enabled() const;
    bool wheel_encoders_healthy() const;

private:
    bool _armed;
    bool _motors_healthy;
    bool _in_failsafe;
    float _current_speed_ms;
    int16_t _throttle_pct;
};

// Global instance
extern SimpleRover simplerover;

// Global GCS accessor
inline GCS &gcs() { return simplerover.gcs(); }
```

---

## File 6: SimpleRover.cpp (Excerpt)

```cpp
#include "SimpleRover.h"

// Global instance
SimpleRover simplerover;

SimpleRover::SimpleRover()
    : is_initialized(false)
    , control_mode(Mode::MANUAL)
    , gcs_pid_mask(0)
    , max_speed_ms(10.0f)
    , rangefinder_enabled(false)
    , _armed(false)
    , _motors_healthy(true)
    , _in_failsafe(false)
    , _current_speed_ms(0.0f)
    , _throttle_pct(0)
{
}

void SimpleRover::setup()
{
    // Initialize HAL
    hal.scheduler->delay(100);

    // Initialize serial manager
    AP::serialmanager().init();

    // Initialize GCS
    gcs().init();
    gcs().setup_console();  // USB/console port
    gcs().setup_uarts();    // Telemetry ports

    gcs().send_text(MAV_SEVERITY_INFO, "SimpleRover initializing");

    // Initialize GPS
    AP::gps().init();

    // Initialize other subsystems
    // ...

    is_initialized = true;

    gcs().send_text(MAV_SEVERITY_INFO, "SimpleRover initialized");
}

void SimpleRover::loop()
{
    // This is called repeatedly

    // Fast loop runs at 100 Hz
    static uint32_t last_fast_loop_us = 0;
    uint32_t now_us = AP_HAL::micros();

    if (now_us - last_fast_loop_us >= 10000) {  // 10ms = 100 Hz
        last_fast_loop_us = now_us;
        fast_loop();
    }

    // Main loop runs at 50 Hz
    static uint32_t last_main_loop_us = 0;
    if (now_us - last_main_loop_us >= 20000) {  // 20ms = 50 Hz
        last_main_loop_us = now_us;
        main_loop();
    }

    // Slow loop runs at 10 Hz
    static uint32_t last_slow_loop_us = 0;
    if (now_us - last_slow_loop_us >= 100000) {  // 100ms = 10 Hz
        last_slow_loop_us = now_us;
        slow_loop();
    }
}

void SimpleRover::fast_loop()
{
    // Read sensors
    // Run attitude control
    // Output to motors
    // (highest priority, minimal GCS interaction)
}

void SimpleRover::main_loop()
{
    // Update GPS
    AP::gps().update();

    // Run navigation
    // ...

    // Update GCS - IMPORTANT!
    gcs().update_receive();  // Parse incoming messages (max 1ms)
    gcs().update_send();     // Send outgoing messages
}

void SimpleRover::slow_loop()
{
    // Battery monitoring
    // Logging
    // Slower telemetry updates
}

void SimpleRover::set_target_speed(float speed_ms)
{
    // Implementation
    gcs().send_text(MAV_SEVERITY_DEBUG, "Target speed: %.2f", speed_ms);
}

float SimpleRover::get_speed_error() const
{
    // Return difference between target and actual speed
    return 0.0f;  // Simplified
}

bool SimpleRover::wheel_encoders_present() const { return true; }
bool SimpleRover::wheel_encoders_enabled() const { return true; }
bool SimpleRover::wheel_encoders_healthy() const { return true; }
```

---

## File 7: main.cpp

```cpp
#include <AP_HAL/AP_HAL.h>
#include "SimpleRover.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

void setup()
{
    simplerover.setup();
}

void loop()
{
    simplerover.loop();
}

AP_HAL_MAIN();
```

---

## Build Configuration

Add to your `wscript`:

```python
def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'GCS_MAVLink',
            'AP_SerialManager',
            'AP_GPS',
            'AP_RangeFinder',
            'AP_BattMonitor',
            # ... other libraries you need ...
        ],
    )

    bld.ap_program(
        program_name=vehicle,
        program_groups=['bin', 'simplerover'],
        use=vehicle + '_libs',
    )
```

---

## Testing the Implementation

### 1. Build
```bash
./waf configure --board=<your_board>
./waf simplerover
```

### 2. Connect GCS

Use Mission Planner, QGroundControl, or MAVProxy:

```bash
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

### 3. Verify Communication

You should see:
- HEARTBEAT messages
- SYS_STATUS showing sensor health
- GPS_RAW_INT with GPS data
- ATTITUDE messages
- VFR_HUD with speed/altitude

### 4. Test Commands

```
# Set mode to GUIDED
mode GUIDED

# Send speed command via SET_POSITION_TARGET_LOCAL_NED
# (if implemented in GCS)

# Test custom command
long MAV_CMD_DO_CHANGE_SPEED 0 2.5 0 0 0 0 0
# Should set max speed to 2.5 m/s
```

### 5. Check Status Text

You should see initialization messages:
```
SimpleRover initializing
SimpleRover initialized
```

---

## Key Takeaways from This Example

1. **Minimal Required Code**: You only need to implement a handful of pure virtual functions
2. **Inheritance Power**: Base classes handle 90% of MAVLink protocol
3. **Customization Points**: Override specific methods for vehicle-specific behavior
4. **Message Routing**: Base class handles routing, you just add your custom cases
5. **Command Handling**: Simple pattern for adding custom commands
6. **Integration**: GCS integrates seamlessly into main vehicle loop

## What the Base Class Handles Automatically

You get for free:
- ✓ MAVLink parsing and serialization
- ✓ Parameter protocol (get/set/list)
- ✓ Mission protocol (upload/download waypoints)
- ✓ Fence protocol
- ✓ Rally point protocol
- ✓ File transfer (FTP)
- ✓ Logging messages (STATUSTEXT)
- ✓ Common commands (ARM, MODE, etc.)
- ✓ Stream rate management
- ✓ Message interval configuration
- ✓ Channel management
- ✓ Heartbeat sending
- ✓ System status
- ✓ And hundreds more messages!

## What You Need to Implement

Minimal requirements:
- ✓ 4 pure virtual methods in GCS_MAVLINK
- ✓ 2 pure virtual methods in GCS
- ✓ GCS factory method
- ✓ VFR_HUD data (optional but recommended)
- ✓ Any vehicle-specific messages/commands

That's it! The rest is optional customization.
