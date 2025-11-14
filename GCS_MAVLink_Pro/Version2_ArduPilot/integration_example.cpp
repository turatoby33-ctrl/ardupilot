/*
 * Integration Example for Version 2
 * Shows how to use minimal_GCS_Common.cpp with real ArduPilot infrastructure
 *
 * This example demonstrates creating a minimal vehicle that uses:
 * - Real GCS.h, GCS.cpp
 * - Real GCS_MAVLink.h, GCS_MAVLink.cpp
 * - Minimal GCS_Common.cpp (HEARTBEAT only)
 *
 * File structure:
 * MyVehicle/
 * ├── GCS_MyVehicle.h           # GCS manager class
 * ├── GCS_MyVehicle.cpp         # GCS manager implementation
 * ├── GCS_MAVLink_MyVehicle.h   # Channel class
 * ├── GCS_MAVLink_MyVehicle.cpp # Channel implementation
 * ├── MyVehicle.h               # Main vehicle class
 * └── MyVehicle.cpp             # Main vehicle implementation
 */

// ============================================
// GCS_MyVehicle.h
// ============================================

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_MyVehicle.h"

class GCS_MyVehicle : public GCS
{
    friend class MyVehicle;

public:
    // Required: Generate chan() methods
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_MyVehicle);

    // Required: Vehicle identification
    uint32_t custom_mode() const override;
    MAV_TYPE frame_type() const override;

    // Optional: Vehicle state
    bool vehicle_initialised() const override;

    // Optional: Sensor status
    void update_vehicle_sensor_status_flags(void) override;

protected:
    // Required: Factory method
    GCS_MAVLINK_MyVehicle *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_MyVehicle(uart);
    }

    // Optional: Timing constraint
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 200;  // 200 microseconds
    }
};

// ============================================
// GCS_MyVehicle.cpp
// ============================================

#include "GCS_MyVehicle.h"
#include "MyVehicle.h"

uint32_t GCS_MyVehicle::custom_mode() const
{
    return (uint32_t)myvehicle.control_mode;
}

MAV_TYPE GCS_MyVehicle::frame_type() const
{
    return MAV_TYPE_GROUND_ROVER;
}

bool GCS_MyVehicle::vehicle_initialised() const
{
    return myvehicle.initialised;
}

void GCS_MyVehicle::update_vehicle_sensor_status_flags(void)
{
    // Add vehicle-specific sensor status
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
}

// ============================================
// GCS_MAVLink_MyVehicle.h
// ============================================

#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_MyVehicle : public GCS_MAVLINK
{
public:
    using GCS_MAVLINK::GCS_MAVLINK;  // Inherit constructor

protected:
    // REQUIRED PURE VIRTUALS

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;

    // OPTIONAL OVERRIDES

    // VFR_HUD data
    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;

    // Message handling
    void handle_message(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;

    // Command handling
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;
};

// ============================================
// GCS_MAVLink_MyVehicle.cpp
// ============================================

#include "MyVehicle.h"
#include "GCS_MAVLink_MyVehicle.h"

// Required implementations

uint8_t GCS_MAVLINK_MyVehicle::base_mode() const
{
    uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    if (myvehicle.is_armed()) {
        mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    return mode;
}

MAV_STATE GCS_MAVLINK_MyVehicle::vehicle_system_status() const
{
    if (!myvehicle.initialised) {
        return MAV_STATE_BOOT;
    }

    if (myvehicle.in_failsafe()) {
        return MAV_STATE_CRITICAL;
    }

    if (myvehicle.is_armed()) {
        return MAV_STATE_ACTIVE;
    }

    return MAV_STATE_STANDBY;
}

void GCS_MAVLINK_MyVehicle::send_nav_controller_output() const
{
    if (!myvehicle.initialised) {
        return;
    }

    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // roll
        0,  // pitch
        myvehicle.nav_heading(),
        myvehicle.nav_bearing(),
        myvehicle.nav_distance(),
        0,  // alt_error
        0,  // aspd_error
        0   // xtrack_error
    );
}

void GCS_MAVLINK_MyVehicle::send_pid_tuning()
{
    // Optional: Send PID tuning data
    // For minimal version, can be empty
}

// Optional implementations

float GCS_MAVLINK_MyVehicle::vfr_hud_airspeed() const
{
    return myvehicle.get_speed();
}

int16_t GCS_MAVLINK_MyVehicle::vfr_hud_throttle() const
{
    return myvehicle.get_throttle();
}

void GCS_MAVLINK_MyVehicle::handle_message(const mavlink_message_t &msg)
{
    // Add custom message handling here
    switch (msg.msgid) {

    // Your custom messages
    // case MAVLINK_MSG_ID_MY_CUSTOM_MSG:
    //     handle_my_custom_msg(msg);
    //     break;

    default:
        // Always call base class!
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

bool GCS_MAVLINK_MyVehicle::try_send_message(enum ap_message id)
{
    // Add custom message sending here
    switch(id) {

    // Your custom messages
    // case MSG_MY_CUSTOM:
    //     CHECK_PAYLOAD_SIZE(MY_CUSTOM);
    //     send_my_custom();
    //     break;

    default:
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}

MAV_RESULT GCS_MAVLINK_MyVehicle::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    // Add custom command handling here
    switch (packet.command) {

    // Your custom commands
    // case MAV_CMD_MY_CUSTOM_CMD:
    //     return handle_my_custom_cmd(packet);

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

// ============================================
// MyVehicle.h
// ============================================

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MyVehicle.h"

class MyVehicle
{
public:
    MyVehicle();

    void setup();
    void loop();

    // GCS interface
    GCS_MyVehicle gcs_instance;
    GCS_MyVehicle &gcs() { return gcs_instance; }

    // State
    bool initialised;
    uint8_t control_mode;

    // Methods used by GCS
    bool is_armed() const { return armed; }
    bool in_failsafe() const { return failsafe; }
    float get_speed() const { return speed; }
    int16_t get_throttle() const { return throttle; }
    int16_t nav_heading() const { return 0; }
    int16_t nav_bearing() const { return 0; }
    uint16_t nav_distance() const { return 0; }

private:
    bool armed;
    bool failsafe;
    float speed;
    int16_t throttle;
};

extern MyVehicle myvehicle;
inline GCS &gcs() { return myvehicle.gcs(); }

// ============================================
// MyVehicle.cpp
// ============================================

#include "MyVehicle.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

MyVehicle myvehicle;

MyVehicle::MyVehicle()
    : initialised(false)
    , control_mode(0)
    , armed(false)
    , failsafe(false)
    , speed(0.0f)
    , throttle(0)
{
}

void MyVehicle::setup()
{
    // Initialize HAL
    hal.scheduler->delay(100);

    // Initialize serial ports
    AP::serialmanager().init();

    // Initialize GCS - IMPORTANT!
    gcs().init();
    gcs().setup_console();  // USB/console
    gcs().setup_uarts();    // Telemetry ports

    gcs().send_text(MAV_SEVERITY_INFO, "MyVehicle initializing");

    // Initialize other subsystems
    // ...

    initialised = true;

    gcs().send_text(MAV_SEVERITY_INFO, "MyVehicle ready");
}

void MyVehicle::loop()
{
    // This is called repeatedly

    // Fast loop (high frequency control)
    // ...

    // Main loop (50 Hz typical)
    static uint32_t last_main_loop_ms = 0;
    uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_main_loop_ms >= 20) {  // 50 Hz
        last_main_loop_ms = now_ms;

        // Update sensors
        // ...

        // Run control loops
        // ...

        // UPDATE GCS - CRITICAL!
        // This handles all MAVLink communication
        gcs().update_receive();  // Parse incoming messages
        gcs().update_send();     // Send outgoing messages
    }
}

// ============================================
// main.cpp
// ============================================

#include <AP_HAL/AP_HAL.h>
#include "MyVehicle.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

void setup()
{
    myvehicle.setup();
}

void loop()
{
    myvehicle.loop();
}

AP_HAL_MAIN();

// ============================================
// wscript (Build Configuration)
// ============================================

/*
def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AP_SerialManager',
            'GCS_MAVLink',
            # ... other libraries ...
        ],
    )

    bld.ap_program(
        program_name=vehicle,
        program_groups=['bin', 'myvehicle'],
        use=vehicle + '_libs',
    )
*/

// ============================================
// USAGE INSTRUCTIONS
// ============================================

/*
 * STEP 1: File Placement
 *
 * Place minimal_GCS_Common.cpp in:
 * libraries/GCS_MAVLink/GCS_Common_Minimal.cpp
 *
 * Place your vehicle files in:
 * MyVehicle/GCS_MyVehicle.h
 * MyVehicle/GCS_MyVehicle.cpp
 * MyVehicle/GCS_MAVLink_MyVehicle.h
 * MyVehicle/GCS_MAVLink_MyVehicle.cpp
 * MyVehicle/MyVehicle.h
 * MyVehicle/MyVehicle.cpp
 *
 * STEP 2: Build Configuration
 *
 * Modify your wscript to use minimal GCS_Common:
 *
 * In libraries/GCS_MAVLink/wscript:
 * - Comment out: 'GCS_Common.cpp'
 * - Add: 'GCS_Common_Minimal.cpp'
 *
 * Or in your vehicle's wscript, specify which
 * GCS_Common to use.
 *
 * STEP 3: Build
 *
 * ./waf configure --board=<your_board>
 * ./waf myvehicle
 *
 * STEP 4: Upload and Test
 *
 * ./waf --upload myvehicle
 *
 * Connect with MAVProxy or Mission Planner:
 * mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
 *
 * You should see HEARTBEAT messages at 1 Hz!
 *
 * STEP 5: Add More Messages
 *
 * Follow the HEARTBEAT pattern in minimal_GCS_Common.cpp
 * to add more message types:
 *
 * 1. Add send function
 * 2. Add handle function
 * 3. Add to message router
 * 4. Call from update_send() or vehicle code
 *
 * DEBUGGING:
 *
 * Enable console output to see what's happening:
 * - hal.console->printf() statements in the code
 * - Connect serial console to see debug output
 * - Use MAVProxy with --debug flag
 * - Check mavlink.log for message traffic
 *
 * COMMON ISSUES:
 *
 * 1. No HEARTBEAT received:
 *    - Check serial port configuration
 *    - Verify baudrate matches
 *    - Ensure update_send() is being called
 *
 * 2. Connection drops:
 *    - HEARTBEAT must be sent every 1 second
 *    - Check main loop timing
 *    - Verify update_send() not blocked
 *
 * 3. Messages not parsed:
 *    - Check update_receive() is being called
 *    - Verify message ID in handle_message()
 *    - Check MAVLink library is linked
 *
 * 4. Build errors:
 *    - Ensure all pure virtuals implemented
 *    - Check header include paths
 *    - Verify GCS_MAVLink library linked
 */

// ============================================
// TRANSITION TO FULL GCS_COMMON
// ============================================

/*
 * Once your minimal version is working, you can
 * transition to the full GCS_Common.cpp:
 *
 * 1. In libraries/GCS_MAVLink/wscript:
 *    - Uncomment: 'GCS_Common.cpp'
 *    - Remove: 'GCS_Common_Minimal.cpp'
 *
 * 2. Rebuild:
 *    ./waf clean
 *    ./waf configure --board=<your_board>
 *    ./waf myvehicle
 *
 * 3. You now have full MAVLink functionality:
 *    - Parameters (get/set/list)
 *    - Missions (upload/download)
 *    - Commands (200+ supported)
 *    - All standard messages
 *    - File transfer (FTP)
 *    - And much more!
 *
 * Your vehicle-specific GCS classes don't need
 * to change - they work with both minimal and
 * full GCS_Common.cpp!
 *
 * This is the power of the GCS architecture:
 * - Vehicle code stays the same
 * - Common functionality is shared
 * - Easy to add features incrementally
 */
