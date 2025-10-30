/**
 * @file GCS_Common.cpp
 * @brief EduCopter Common GCS Message Handlers
 *
 * Implements all common MAVLink message send/receive functions.
 * This is the largest file containing message handlers for heartbeat,
 * status, attitude, position, commands, and more.
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>
#include <cstdio>

#if EDUCOPTER_GCS_ENABLED

// External vehicle functions (must be provided by vehicle code)
extern uint32_t millis();
extern uint16_t millis16();
extern float getAttitudeRoll();
extern float getAttitudePitch();
extern float getAttitudeYaw();
extern float getAttitudeRollRate();
extern float getAttitudePitchRate();
extern float getAttitudeYawRate();
extern int32_t getLatitude();
extern int32_t getLongitude();
extern float getAltitude();
extern float getRelativeAltitude();
extern int16_t getVelocityX();
extern int16_t getVelocityY();
extern int16_t getVelocityZ();
extern uint16_t getHeading();
extern float getBatteryVoltage();
extern float getBatteryCurrent();
extern int8_t getBatteryRemaining();
extern uint16_t getCPULoad();
extern uint32_t getSensorsPresentMask();
extern uint32_t getSensorsEnabledMask();
extern uint32_t getSensorsHealthMask();

namespace EduCopter {
namespace GCS {

// ========== SEND FUNCTIONS ==========

void GCSChannel::sendHeartbeat() {
    if (!hasPayloadSpace(MAVLINK_MSG_ID_HEARTBEAT)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        getBaseMode(),
        0, // custom_mode (vehicle-specific)
        getSystemStatus()
    );

    sendMessage(&msg);
}

void GCSChannel::sendSystemStatus() {
    if (!hasPayloadSpace(MAVLINK_MSG_ID_SYS_STATUS)) {
        return;
    }

    // Gather system status
    uint32_t onboard_control_sensors_present = getSensorsPresentMask();
    uint32_t onboard_control_sensors_enabled = getSensorsEnabledMask();
    uint32_t onboard_control_sensors_health = getSensorsHealthMask();
    uint16_t load = getCPULoad(); // CPU load in d%
    uint16_t voltage_battery = (uint16_t)(getBatteryVoltage() * 1000);
    int16_t current_battery = (int16_t)(getBatteryCurrent() * 100);
    int8_t battery_remaining = getBatteryRemaining();

    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        onboard_control_sensors_present,
        onboard_control_sensors_enabled,
        onboard_control_sensors_health,
        load,
        voltage_battery,
        current_battery,
        battery_remaining,
        0, 0, 0, 0, 0, 0, 0
    );

    sendMessage(&msg);
}

void GCSChannel::sendAttitude() {
    if (!hasPayloadSpace(MAVLINK_MSG_ID_ATTITUDE)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        millis(),
        getAttitudeRoll(),
        getAttitudePitch(),
        getAttitudeYaw(),
        getAttitudeRollRate(),
        getAttitudePitchRate(),
        getAttitudeYawRate()
    );

    sendMessage(&msg);
}

void GCSChannel::sendGlobalPosition() {
    if (!hasPayloadSpace(MAVLINK_MSG_ID_GLOBAL_POSITION_INT)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        millis(),
        getLatitude(),
        getLongitude(),
        (int32_t)(getAltitude() * 1000), // alt in mm
        (int32_t)(getRelativeAltitude() * 1000), // relative_alt in mm
        getVelocityX(),
        getVelocityY(),
        getVelocityZ(),
        getHeading()
    );

    sendMessage(&msg);
}

// Additional send functions would be implemented here...
// sendLocalPosition(), sendGPSRaw(), sendBatteryStatus(), etc.
// Each following the same pattern

// ========== COMMAND HANDLERS ==========

MAV_RESULT GCSChannel::handleCommandPreflightCalibration(const mavlink_command_long_t& cmd) {
    // Handle preflight calibration commands
    // param1: gyro cal
    // param2: mag cal
    // param3: ground pressure
    // param4: radio cal
    // param5: accel cal
    // param6: compass mot
    // param7: airspeed cal

    if (cmd.param1 > 0) {
        // Calibrate gyros
        sendText(MAV_SEVERITY_INFO, "Calibrating gyros...");
        // Perform calibration...
        return MAV_RESULT_ACCEPTED;
    }

    if (cmd.param5 > 0) {
        // Calibrate accelerometers
        sendText(MAV_SEVERITY_INFO, "Calibrating accels...");
        // Perform calibration...
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCSChannel::handleCommandComponentArmDisarm(const mavlink_command_long_t& cmd) {
    bool arm = (cmd.param1 > 0.5f);

    if (arm) {
        // Arm vehicle
        sendText(MAV_SEVERITY_INFO, "Arming motors");
        // Call vehicle arm function
        return MAV_RESULT_ACCEPTED;
    } else {
        // Disarm vehicle
        sendText(MAV_SEVERITY_INFO, "Disarming motors");
        // Call vehicle disarm function
        return MAV_RESULT_ACCEPTED;
    }
}

MAV_RESULT GCSChannel::handleCommandDoSetHome(const mavlink_command_long_t& cmd) {
    bool use_current = (cmd.param1 > 0.5f);

    if (use_current) {
        // Set home to current position
        sendText(MAV_SEVERITY_INFO, "Home set to current position");
        return MAV_RESULT_ACCEPTED;
    } else {
        // Set home to specified position
        // Use cmd.param5 (latitude), cmd.param6 (longitude), cmd.param7 (altitude)
        sendText(MAV_SEVERITY_INFO, "Home set to specified position");
        return MAV_RESULT_ACCEPTED;
    }
}

MAV_RESULT GCSChannel::handleCommandDoSetMode(const mavlink_command_long_t& cmd) {
    uint8_t base_mode = (uint8_t)cmd.param1;
    uint32_t custom_mode = (uint32_t)cmd.param2;

    // Set flight mode
    char buf[80];
    snprintf(buf, sizeof(buf), "Mode change to %u", custom_mode);
    sendText(MAV_SEVERITY_INFO, buf);
    // Call vehicle set_mode function

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCSChannel::handleCommandGetHomePosition(const mavlink_command_long_t& cmd) {
    // Send HOME_POSITION message
    sendText(MAV_SEVERITY_INFO, "Sending home position");
    // sendHomePosition();
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCSChannel::handleCommandSetMessageInterval(const mavlink_command_long_t& cmd) {
    uint32_t msg_id = (uint32_t)cmd.param1;
    int32_t interval_us = (int32_t)cmd.param2;

    // Set message interval
    char buf[80];
    snprintf(buf, sizeof(buf), "Set msg %u interval %d", msg_id, interval_us);
    sendText(MAV_SEVERITY_INFO, buf);

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCSChannel::handleCommandRequestMessage(const mavlink_command_long_t& cmd) {
    uint32_t msg_id = (uint32_t)cmd.param1;

    // Send requested message immediately
    char buf[80];
    snprintf(buf, sizeof(buf), "Sending msg %u", msg_id);
    sendText(MAV_SEVERITY_INFO, buf);

    return MAV_RESULT_ACCEPTED;
}

// Additional command handlers would go here...

// ========== MESSAGE HANDLERS ==========

void GCSChannel::handleCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    // Check if command is for us
    if (cmd.target_system != m_mavlink.getSystemID() &&
        cmd.target_system != 0) {
        return;
    }

    MAV_RESULT result = MAV_RESULT_UNSUPPORTED;

    // Route to appropriate handler
    switch (cmd.command) {
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            result = handleCommandPreflightCalibration(cmd);
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            result = handleCommandComponentArmDisarm(cmd);
            break;

        case MAV_CMD_DO_SET_HOME:
            result = handleCommandDoSetHome(cmd);
            break;

        case MAV_CMD_DO_SET_MODE:
            result = handleCommandDoSetMode(cmd);
            break;

        case MAV_CMD_GET_HOME_POSITION:
            result = handleCommandGetHomePosition(cmd);
            break;

        case MAV_CMD_SET_MESSAGE_INTERVAL:
            result = handleCommandSetMessageInterval(cmd);
            break;

        case MAV_CMD_REQUEST_MESSAGE:
            result = handleCommandRequestMessage(cmd);
            break;

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
    }

    // Send COMMAND_ACK
    mavlink_message_t ack_msg;
    mavlink_msg_command_ack_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &ack_msg,
        cmd.command,
        result,
        0, 0, 0, 0
    );

    sendMessage(&ack_msg);
}

void GCSChannel::handleCommandInt(const mavlink_message_t& msg) {
    // COMMAND_INT is similar to COMMAND_LONG but with integer coordinates
    // For now, convert to COMMAND_LONG and handle
    mavlink_command_int_t cmd_int;
    mavlink_msg_command_int_decode(&msg, &cmd_int);

    mavlink_command_long_t cmd_long;
    cmd_long.target_system = cmd_int.target_system;
    cmd_long.target_component = cmd_int.target_component;
    cmd_long.command = cmd_int.command;
    cmd_long.confirmation = 0;
    cmd_long.param1 = cmd_int.param1;
    cmd_long.param2 = cmd_int.param2;
    cmd_long.param3 = cmd_int.param3;
    cmd_long.param4 = cmd_int.param4;
    cmd_long.param5 = cmd_int.x;
    cmd_long.param6 = cmd_int.y;
    cmd_long.param7 = cmd_int.z;

    // Create fake message and handle
    mavlink_message_t long_msg;
    mavlink_msg_command_long_encode(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &long_msg,
        &cmd_long
    );

    handleCommandLong(long_msg);
}

// ========== PLACEHOLDER IMPLEMENTATIONS ==========
// These would be fully implemented in a production system

void GCSChannel::sendRawIMU() {
    // Send RAW_IMU message with accelerometer, gyro, mag data
}

void GCSChannel::sendScaledIMU() {
    // Send SCALED_IMU message with scaled sensor data
}

void GCSChannel::sendScaledPressure() {
    // Send SCALED_PRESSURE message with barometer data
}

void GCSChannel::sendBatteryStatus() {
    // Send BATTERY_STATUS message
}

void GCSChannel::sendRCChannels() {
    // Send RC_CHANNELS message
}

void GCSChannel::sendServoOutputRaw() {
    // Send SERVO_OUTPUT_RAW message
}

void GCSChannel::sendVFRHUD() {
    // Send VFR_HUD message
}

// ... Additional 60+ send functions would be here ...

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
