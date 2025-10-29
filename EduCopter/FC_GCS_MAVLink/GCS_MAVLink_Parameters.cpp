/**
 * @file GCS_MAVLink_Parameters.cpp
 * @brief MAVLink stream rate configuration and management
 *
 * This file implements stream rate configuration for different message groups.
 * Allows ground stations to request specific message rates using
 * REQUEST_DATA_STREAM and SET_MESSAGE_INTERVAL commands.
 *
 * Message groups:
 * - RAW_SENSORS: IMU, barometer, magnetometer
 * - EXTENDED_STATUS: Battery, GPS status, system health
 * - RC_CHANNELS: RC input channels
 * - POSITION: Global and local position
 * - EXTRA1: Attitude, angular rates
 * - EXTRA2: VFR_HUD, navigation output
 * - EXTRA3: Various additional telemetry
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>

namespace EduCopter {
namespace GCS {

/**
 * @brief Stream rate groups
 *
 * Defines which messages belong to which stream group.
 */
enum class StreamRateGroup : uint8_t {
    RAW_SENSORS = 0,
    EXTENDED_STATUS = 1,
    RC_CHANNELS = 2,
    POSITION = 6,
    EXTRA1 = 10,
    EXTRA2 = 11,
    EXTRA3 = 12
};

/**
 * @brief Get messages in a stream group
 *
 * Returns bitmask of messages that belong to a specific stream group.
 *
 * @param group Stream rate group
 * @return Bitmask of MessageID values
 */
static uint64_t getStreamGroupMessages(MAV_DATA_STREAM group)
{
    uint64_t mask = 0;

    switch (group) {
        case MAV_DATA_STREAM_RAW_SENSORS:
            // IMU, barometer, magnetometer
            mask |= (1ULL << static_cast<uint8_t>(MessageID::RAW_IMU));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SCALED_IMU));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SCALED_PRESSURE));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::RAW_PRESSURE));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SENSOR_OFFSETS));
            break;

        case MAV_DATA_STREAM_EXTENDED_STATUS:
            // System status, battery, GPS
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SYSTEM_STATUS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SYS_STATUS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::POWER_STATUS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::MEMINFO));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::MISSION_CURRENT));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::GPS_RAW_INT));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::GPS_STATUS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::NAV_CONTROLLER_OUTPUT));
            break;

        case MAV_DATA_STREAM_RC_CHANNELS:
            // RC inputs
            mask |= (1ULL << static_cast<uint8_t>(MessageID::RC_CHANNELS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::RC_CHANNELS_RAW));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SERVO_OUTPUT_RAW));
            break;

        case MAV_DATA_STREAM_POSITION:
            // Position and velocity
            mask |= (1ULL << static_cast<uint8_t>(MessageID::GLOBAL_POSITION));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::GLOBAL_POSITION_INT));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::LOCAL_POSITION_NED));
            break;

        case MAV_DATA_STREAM_EXTRA1:
            // Attitude
            mask |= (1ULL << static_cast<uint8_t>(MessageID::ATTITUDE));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::SIMSTATE));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::AHRS));
            break;

        case MAV_DATA_STREAM_EXTRA2:
            // VFR and navigation
            mask |= (1ULL << static_cast<uint8_t>(MessageID::VFR_HUD));
            break;

        case MAV_DATA_STREAM_EXTRA3:
            // Additional telemetry
            mask |= (1ULL << static_cast<uint8_t>(MessageID::AHRS2));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::HWSTATUS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::WIND));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::RANGEFINDER));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::TERRAIN_REPORT));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::BATTERY2));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::MOUNT_STATUS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::OPTICAL_FLOW));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::MAG_CAL_REPORT));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::MAG_CAL_PROGRESS));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::EKF_STATUS_REPORT));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::VIBRATION));
            mask |= (1ULL << static_cast<uint8_t>(MessageID::RPM));
            break;

        case MAV_DATA_STREAM_ALL:
            // All messages
            mask = 0xFFFFFFFFFFFFFFFFULL;
            break;

        default:
            break;
    }

    return mask;
}

/**
 * @brief Handle REQUEST_DATA_STREAM message
 *
 * Configures stream rates for message groups.
 * Legacy method - newer GCS use MESSAGE_INTERVAL.
 *
 * @param msg MAVLink message
 */
void GCSChannel::handleRequestDataStream(const mavlink_message_t& msg)
{
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(&msg, &packet);

    // Check if request is for us
    if (packet.target_system != m_mavlink.getSystemID()) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    // Get messages in this stream group
    uint64_t messageMask = getStreamGroupMessages(
        static_cast<MAV_DATA_STREAM>(packet.req_stream_id));

    if (messageMask == 0) {
        return; // Unknown stream
    }

    // Calculate interval in milliseconds
    uint16_t intervalMS = 0;
    if (packet.start_stop && packet.req_message_rate > 0) {
        // Convert Hz to milliseconds
        intervalMS = 1000 / packet.req_message_rate;
    }

    // Apply rate to all messages in group
    for (uint8_t msgID = 0; msgID < static_cast<uint8_t>(MessageID::MESSAGE_COUNT); msgID++) {
        if (messageMask & (1ULL << msgID)) {
            setMessageInterval(static_cast<MessageID>(msgID), intervalMS);
        }
    }

    // Log the change
    const char* streamName = "UNKNOWN";
    switch (packet.req_stream_id) {
        case MAV_DATA_STREAM_RAW_SENSORS: streamName = "RAW_SENSORS"; break;
        case MAV_DATA_STREAM_EXTENDED_STATUS: streamName = "EXTENDED_STATUS"; break;
        case MAV_DATA_STREAM_RC_CHANNELS: streamName = "RC_CHANNELS"; break;
        case MAV_DATA_STREAM_POSITION: streamName = "POSITION"; break;
        case MAV_DATA_STREAM_EXTRA1: streamName = "EXTRA1"; break;
        case MAV_DATA_STREAM_EXTRA2: streamName = "EXTRA2"; break;
        case MAV_DATA_STREAM_EXTRA3: streamName = "EXTRA3"; break;
        case MAV_DATA_STREAM_ALL: streamName = "ALL"; break;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "Stream %s: %s at %uHz",
             streamName,
             packet.start_stop ? "started" : "stopped",
             packet.req_message_rate);
    sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Handle MAV_CMD_SET_MESSAGE_INTERVAL command
 *
 * Modern method to set individual message intervals.
 *
 * @param cmd Command parameters
 * @return Command result
 */
MAV_RESULT GCSChannel::handleCommandSetMessageInterval(const mavlink_command_long_t& cmd)
{
    // param1: Message ID
    // param2: Interval in microseconds (-1 = disable, 0 = default)

    uint32_t msgID = static_cast<uint32_t>(cmd.param1);
    int32_t intervalUS = static_cast<int32_t>(cmd.param2);

    // Convert MAVLink message ID to internal MessageID
    MessageID internalID = mavlinkIDToMessageID(msgID);

    if (internalID == MessageID::MESSAGE_COUNT) {
        return MAV_RESULT_UNSUPPORTED; // Unknown message
    }

    uint16_t intervalMS = 0;

    if (intervalUS == -1) {
        // Disable message
        intervalMS = 0;
    } else if (intervalUS == 0) {
        // Use default rate
        intervalMS = getDefaultMessageInterval(internalID);
    } else {
        // Convert microseconds to milliseconds
        intervalMS = intervalUS / 1000;
        if (intervalMS == 0 && intervalUS > 0) {
            intervalMS = 1; // Minimum 1ms
        }
    }

    setMessageInterval(internalID, intervalMS);

    char buf[80];
    snprintf(buf, sizeof(buf), "Message %u interval: %ums",
             msgID, intervalMS);
    sendText(MAV_SEVERITY_INFO, buf);

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Handle MAV_CMD_GET_MESSAGE_INTERVAL command
 *
 * Returns the current interval for a message.
 *
 * @param cmd Command parameters
 * @return Command result
 */
MAV_RESULT GCSChannel::handleCommandGetMessageInterval(const mavlink_command_long_t& cmd)
{
    // param1: Message ID

    uint32_t msgID = static_cast<uint32_t>(cmd.param1);

    // Convert MAVLink message ID to internal MessageID
    MessageID internalID = mavlinkIDToMessageID(msgID);

    if (internalID == MessageID::MESSAGE_COUNT) {
        return MAV_RESULT_UNSUPPORTED;
    }

    uint16_t intervalMS = getMessageInterval(internalID);
    uint32_t intervalUS = intervalMS * 1000;

    // Send MESSAGE_INTERVAL response
    if (hasPayloadSpace(MAVLINK_MSG_ID_MESSAGE_INTERVAL)) {
        mavlink_message_t msg;
        mavlink_msg_message_interval_pack(
            m_mavlink.getSystemID(),
            m_mavlink.getComponentID(),
            &msg,
            msgID,
            intervalUS
        );
        sendMessage(&msg);
    }

    return MAV_RESULT_ACCEPTED;
}

/**
 * @brief Convert MAVLink message ID to internal MessageID
 *
 * Maps MAVLink protocol message IDs to internal scheduling IDs.
 *
 * @param mavlinkID MAVLink message ID
 * @return Internal MessageID
 */
MessageID GCSChannel::mavlinkIDToMessageID(uint32_t mavlinkID)
{
    switch (mavlinkID) {
        case MAVLINK_MSG_ID_HEARTBEAT: return MessageID::HEARTBEAT;
        case MAVLINK_MSG_ID_SYS_STATUS: return MessageID::SYS_STATUS;
        case MAVLINK_MSG_ID_SYSTEM_TIME: return MessageID::SYSTEM_TIME;
        case MAVLINK_MSG_ID_ATTITUDE: return MessageID::ATTITUDE;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return MessageID::GLOBAL_POSITION_INT;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: return MessageID::LOCAL_POSITION_NED;
        case MAVLINK_MSG_ID_GPS_RAW_INT: return MessageID::GPS_RAW_INT;
        case MAVLINK_MSG_ID_RC_CHANNELS: return MessageID::RC_CHANNELS;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: return MessageID::RC_CHANNELS_RAW;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: return MessageID::SERVO_OUTPUT_RAW;
        case MAVLINK_MSG_ID_VFR_HUD: return MessageID::VFR_HUD;
        case MAVLINK_MSG_ID_RAW_IMU: return MessageID::RAW_IMU;
        case MAVLINK_MSG_ID_SCALED_IMU: return MessageID::SCALED_IMU;
        case MAVLINK_MSG_ID_SCALED_PRESSURE: return MessageID::SCALED_PRESSURE;
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: return MessageID::NAV_CONTROLLER_OUTPUT;
        case MAVLINK_MSG_ID_MISSION_CURRENT: return MessageID::MISSION_CURRENT;
        case MAVLINK_MSG_ID_BATTERY_STATUS: return MessageID::BATTERY_STATUS;
        // Add more mappings as needed
        default: return MessageID::MESSAGE_COUNT; // Unknown
    }
}

/**
 * @brief Get default interval for a message
 *
 * Returns the default streaming interval in milliseconds.
 *
 * @param msgID Message ID
 * @return Default interval in milliseconds (0 = disabled by default)
 */
uint16_t GCSChannel::getDefaultMessageInterval(MessageID msgID)
{
    switch (msgID) {
        // High-rate messages (10Hz)
        case MessageID::HEARTBEAT:
        case MessageID::ATTITUDE:
            return 100;

        // Medium-rate messages (5Hz)
        case MessageID::GLOBAL_POSITION_INT:
        case MessageID::LOCAL_POSITION_NED:
        case MessageID::VFR_HUD:
            return 200;

        // Low-rate messages (1-2Hz)
        case MessageID::SYS_STATUS:
        case MessageID::GPS_RAW_INT:
        case MessageID::RC_CHANNELS:
            return 1000;

        // Very low-rate messages (0.5Hz or on-demand)
        case MessageID::MISSION_CURRENT:
        case MessageID::NAV_CONTROLLER_OUTPUT:
            return 2000;

        // Disabled by default (send only on request)
        default:
            return 0;
    }
}

/**
 * @brief Set all stream rates to defaults
 *
 * Resets all message intervals to their default values.
 */
void GCSChannel::resetStreamRates()
{
    for (uint8_t i = 0; i < static_cast<uint8_t>(MessageID::MESSAGE_COUNT); i++) {
        MessageID msgID = static_cast<MessageID>(i);
        uint16_t defaultInterval = getDefaultMessageInterval(msgID);
        setMessageInterval(msgID, defaultInterval);
    }

    sendText(MAV_SEVERITY_INFO, "Stream rates reset to defaults");
}

/**
 * @brief Disable all streams
 *
 * Stops all automatic message streaming.
 */
void GCSChannel::disableAllStreams()
{
    for (uint8_t i = 0; i < static_cast<uint8_t>(MessageID::MESSAGE_COUNT); i++) {
        setMessageInterval(static_cast<MessageID>(i), 0);
    }

    sendText(MAV_SEVERITY_INFO, "All streams disabled");
}

/**
 * @brief Send current stream configuration
 *
 * Sends text message listing active streams and their rates.
 */
void GCSChannel::sendStreamConfig()
{
    char buf[100];
    snprintf(buf, sizeof(buf), "Active streams:");
    sendText(MAV_SEVERITY_INFO, buf);

    for (uint8_t i = 0; i < static_cast<uint8_t>(MessageID::MESSAGE_COUNT); i++) {
        MessageID msgID = static_cast<MessageID>(i);
        uint16_t interval = getMessageInterval(msgID);

        if (interval > 0) {
            uint16_t rateHz = 1000 / interval;
            snprintf(buf, sizeof(buf), "  Msg %u: %uHz (%ums)",
                     i, rateHz, interval);
            sendText(MAV_SEVERITY_INFO, buf);
        }
    }
}

} // namespace GCS
} // namespace EduCopter
