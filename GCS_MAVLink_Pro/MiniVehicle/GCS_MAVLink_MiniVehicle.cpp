/*
 * GCS_MAVLink_MiniVehicle.cpp
 *
 * Implementation of MAVLink channel for MiniVehicle
 * This is where all the MAVLink message handling happens
 */

#include "MiniVehicle.h"
#include "GCS_MAVLink_MiniVehicle.h"

// ========================================
// REQUIRED IMPLEMENTATIONS
// ========================================

// Return MAVLink base mode flags
uint8_t GCS_MAVLINK_MiniVehicle::base_mode() const
{
    /*
     * Base mode is a bitfield indicating vehicle capabilities and state:
     * Bit 0: CUSTOM_MODE_ENABLED - using custom mode numbers
     * Bit 1: TEST_ENABLED - test mode
     * Bit 2: AUTO_ENABLED - autopilot mode
     * Bit 3: GUIDED_ENABLED - guided mode
     * Bit 4: STABILIZE_ENABLED - stabilization active
     * Bit 5: HIL_ENABLED - hardware-in-the-loop
     * Bit 6: MANUAL_INPUT_ENABLED - manual input available
     * Bit 7: SAFETY_ARMED - motors armed
     */

    uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    // Check if vehicle is armed
    if (minivehicle.ap.armed) {
        mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // Check if in auto mode
    if (minivehicle.control_mode == MiniVehicle::Mode::AUTO) {
        mode |= MAV_MODE_FLAG_AUTO_ENABLED;
        mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

    // Check if in guided mode
    if (minivehicle.control_mode == MiniVehicle::Mode::GUIDED) {
        mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
    }

    // Always have manual input capability
    mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // Always have stabilization capability
    if (minivehicle.control_mode != MiniVehicle::Mode::MANUAL) {
        mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    return mode;
}

// Return vehicle system status
MAV_STATE GCS_MAVLINK_MiniVehicle::vehicle_system_status() const
{
    /*
     * System status indicates vehicle health:
     * MAV_STATE_UNINIT - Not initialized
     * MAV_STATE_BOOT - Booting up
     * MAV_STATE_CALIBRATING - Calibrating sensors
     * MAV_STATE_STANDBY - Ready but not active
     * MAV_STATE_ACTIVE - Active (armed, flying, etc.)
     * MAV_STATE_CRITICAL - Critical failure
     * MAV_STATE_EMERGENCY - Emergency
     * MAV_STATE_POWEROFF - Powering off
     */

    // Still initializing?
    if (!minivehicle.ap.initialised) {
        return MAV_STATE_BOOT;
    }

    // In failsafe?
    if (minivehicle.ap.failsafe_crash_check) {
        return MAV_STATE_CRITICAL;
    }

    // Armed and active?
    if (minivehicle.ap.armed) {
        return MAV_STATE_ACTIVE;
    }

    // Ready but not armed
    return MAV_STATE_STANDBY;
}

// Send NAV_CONTROLLER_OUTPUT message
void GCS_MAVLINK_MiniVehicle::send_nav_controller_output() const
{
    /*
     * NAV_CONTROLLER_OUTPUT message provides navigation information:
     * - Target bearing
     * - Distance to target
     * - Altitude error
     * - Cross-track error
     * etc.
     *
     * GCS uses this to display navigation status
     */

    if (!minivehicle.ap.initialised) {
        return;
    }

    mavlink_msg_nav_controller_output_send(
        chan,
        0,  // nav_roll (degrees) - target roll angle
        0,  // nav_pitch (degrees) - target pitch angle
        minivehicle.nav_controller.nav_bearing(),  // nav_bearing (degrees) - current bearing to next waypoint
        minivehicle.nav_controller.target_bearing(),  // target_bearing (degrees) - bearing from current to next WP
        minivehicle.nav_controller.wp_distance(),  // wp_dist (meters) - distance to next waypoint
        0,  // alt_error (meters) - altitude error
        0,  // aspd_error (m/s) - airspeed error
        minivehicle.nav_controller.crosstrack_error()  // xtrack_error (meters) - cross-track error
    );
}

// Send PID_TUNING message
void GCS_MAVLINK_MiniVehicle::send_pid_tuning()
{
    /*
     * PID_TUNING message allows real-time PID tuning:
     * - Shows target vs actual
     * - Shows P, I, D, FF terms
     * - Helps tune controllers
     *
     * For minimal vehicle, we'll skip this (leave empty)
     * Full implementation would send steering, throttle, etc. PIDs
     */

    // Example if you have PIDs:
    // const AP_PIDInfo &pid = minivehicle.steering_controller.get_pid_info();
    // mavlink_msg_pid_tuning_send(chan, PID_TUNING_STEERING, pid.target, ...);
}

// ========================================
// VFR_HUD IMPLEMENTATIONS
// ========================================

// Return airspeed for VFR_HUD message
float GCS_MAVLINK_MiniVehicle::vfr_hud_airspeed() const
{
    // For a ground vehicle, "airspeed" is actually ground speed
    return minivehicle.ahrs.groundspeed();
}

// Return throttle percentage for VFR_HUD message
int16_t GCS_MAVLINK_MiniVehicle::vfr_hud_throttle() const
{
    // Return throttle as 0-100 percentage
    // For minimal vehicle, we'll return 0 if not armed
    if (!minivehicle.ap.armed) {
        return 0;
    }

    // Return actual throttle percentage
    // This is just an example - adjust based on your vehicle
    return 50;  // Placeholder
}

// Return altitude for VFR_HUD message
float GCS_MAVLINK_MiniVehicle::vfr_hud_alt() const
{
    // Return altitude from AHRS
    Location loc;
    if (minivehicle.ahrs.get_location(loc)) {
        return loc.alt * 0.01f;  // Convert cm to meters
    }
    return 0.0f;
}

// ========================================
// MESSAGE HANDLING
// ========================================

// Handle incoming MAVLink messages
void GCS_MAVLINK_MiniVehicle::handle_message(const mavlink_message_t &msg)
{
    /*
     * This is where we add custom message handling
     * All messages are routed through here
     *
     * IMPORTANT: Always call base class for unhandled messages!
     */

    switch (msg.msgid) {

    // Add your custom message handlers here
    // Example:
    // case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
    //     handle_set_position_target(msg);
    //     break;

    default:
        // CRITICAL: Call base class for unhandled messages
        // Base class handles HEARTBEAT, parameters, missions, etc.
        GCS_MAVLINK::handle_message(msg);
        break;
    }
}

// Try to send a specific message
bool GCS_MAVLINK_MiniVehicle::try_send_message(enum ap_message id)
{
    /*
     * This is where we add custom periodic messages
     * Called by the message stream system
     *
     * IMPORTANT: Always call base class for unhandled IDs!
     */

    switch(id) {

    // Add your custom messages here
    // Example:
    // case MSG_MY_CUSTOM_DATA:
    //     CHECK_PAYLOAD_SIZE(MY_CUSTOM_DATA);
    //     send_my_custom_data();
    //     break;

    default:
        // CRITICAL: Call base class for unhandled messages
        return GCS_MAVLINK::try_send_message(id);
    }

    return true;
}

// ========================================
// COMMAND HANDLING
// ========================================

// Handle MAVLink commands
MAV_RESULT GCS_MAVLINK_MiniVehicle::handle_command_int_packet(
    const mavlink_command_int_t &packet,
    const mavlink_message_t &msg)
{
    /*
     * This is where we add custom command handling
     * Commands are actions requested by the GCS
     *
     * IMPORTANT: Always call base class for unhandled commands!
     */

    switch (packet.command) {

    // Example custom command
    case MAV_CMD_DO_SET_MODE:
        // Handle mode change
        // Could set minivehicle.control_mode based on packet.param1
        return MAV_RESULT_ACCEPTED;

    // Add more custom commands here
    // case MAV_CMD_MY_CUSTOM_COMMAND:
    //     return handle_cmd_my_custom(packet);

    default:
        // CRITICAL: Call base class for unhandled commands
        // Base class handles ARM, MODE, CALIBRATION, etc.
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

// Example custom command handler
MAV_RESULT GCS_MAVLINK_MiniVehicle::handle_cmd_do_test(
    const mavlink_command_int_t &packet)
{
    // Example command implementation
    // param1, param2, etc. are command parameters
    float param1 = packet.param1;

    // Validate parameters
    if (param1 < 0) {
        return MAV_RESULT_DENIED;
    }

    // Execute command
    // ...

    // Send result
    return MAV_RESULT_ACCEPTED;
}
