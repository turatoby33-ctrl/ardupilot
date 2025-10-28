/**
 * @file MissionItemProtocol_Waypoints.cpp
 * @brief Waypoint mission item protocol implementation
 *
 * This file implements the waypoint-specific mission protocol for EduCopter.
 * Handles upload/download of waypoint missions including:
 * - Takeoff
 * - Navigate to waypoint
 * - Land
 * - Return to launch
 * - Loiter
 * - Change speed/altitude
 * - Do commands (set servo, etc.)
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "MissionItemProtocol_Waypoints.h"
#include "GCS.h"
#include <cstring>
#include <cmath>

namespace EduCopter {
namespace GCS {

// External mission storage interface (implemented by vehicle)
extern uint16_t getWaypointCount();
extern bool getWaypoint(uint16_t index, mavlink_mission_item_int_t& outItem);
extern bool setWaypoint(uint16_t index, const mavlink_mission_item_int_t& item);
extern bool appendWaypoint(const mavlink_mission_item_int_t& item);
extern bool clearWaypoints();
extern uint16_t getCurrentWaypointIndex();
extern bool setCurrentWaypoint(uint16_t index);

// Maximum waypoints supported
static const uint16_t MAX_WAYPOINTS = 255;

/**
 * @brief Constructor
 */
MissionItemProtocol_Waypoints::MissionItemProtocol_Waypoints(GCSChannel& channel)
    : MissionItemProtocol(channel, MAV_MISSION_TYPE_MISSION)
{
}

/**
 * @brief Get total waypoint count
 */
uint16_t MissionItemProtocol_Waypoints::getItemCount() const
{
    return getWaypointCount();
}

/**
 * @brief Get maximum waypoint capacity
 */
uint16_t MissionItemProtocol_Waypoints::getMaxItemCount() const
{
    return MAX_WAYPOINTS;
}

/**
 * @brief Get waypoint item by index
 *
 * Retrieves a waypoint from storage and formats it as MAVLink mission item.
 *
 * @param index Waypoint index
 * @param outItem Output mission item
 * @return Mission result code
 */
MAV_MISSION_RESULT MissionItemProtocol_Waypoints::getItem(
    uint16_t index,
    mavlink_mission_item_int_t& outItem)
{
    if (index >= getWaypointCount()) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    if (!getWaypoint(index, outItem)) {
        return MAV_MISSION_ERROR;
    }

    // Set target system/component for response
    outItem.target_system = getTargetSystem();
    outItem.target_component = getTargetComponent();
    outItem.seq = index;
    outItem.mission_type = MAV_MISSION_TYPE_MISSION;

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Store waypoint item
 *
 * Validates and stores a waypoint item received from GCS.
 *
 * @param item Mission item to store
 * @return Mission result code
 */
MAV_MISSION_RESULT MissionItemProtocol_Waypoints::storeItem(
    const mavlink_mission_item_int_t& item)
{
    // Validate waypoint command
    MAV_MISSION_RESULT validation = validateWaypoint(item);
    if (validation != MAV_MISSION_ACCEPTED) {
        return validation;
    }

    // Append waypoint
    if (!appendWaypoint(item)) {
        return MAV_MISSION_ERROR;
    }

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Clear all waypoints
 */
MAV_MISSION_RESULT MissionItemProtocol_Waypoints::clearAllItems()
{
    if (clearWaypoints()) {
        m_channel.sendText(MAV_SEVERITY_INFO, "Waypoints cleared");
        return MAV_MISSION_ACCEPTED;
    } else {
        return MAV_MISSION_ERROR;
    }
}

/**
 * @brief Called when waypoint upload completes
 */
void MissionItemProtocol_Waypoints::onUploadComplete()
{
    uint16_t count = getWaypointCount();
    char buf[64];
    snprintf(buf, sizeof(buf), "Waypoint upload complete: %u items", count);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);

    // Set current waypoint to first item
    if (count > 0) {
        setCurrentWaypoint(0);
    }
}

/**
 * @brief Called when waypoint download completes
 */
void MissionItemProtocol_Waypoints::onDownloadComplete()
{
    m_channel.sendText(MAV_SEVERITY_INFO, "Waypoint download complete");
}

/**
 * @brief Validate waypoint command and parameters
 */
MAV_MISSION_RESULT MissionItemProtocol_Waypoints::validateWaypoint(
    const mavlink_mission_item_int_t& item)
{
    // Check supported commands
    switch (item.command) {
        // Navigation commands
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_LOITER_UNLIM:
        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LAND:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        case MAV_CMD_NAV_LOITER_TO_ALT:
        case MAV_CMD_NAV_SPLINE_WAYPOINT:
            break;

        // Conditional commands
        case MAV_CMD_CONDITION_DELAY:
        case MAV_CMD_CONDITION_DISTANCE:
        case MAV_CMD_CONDITION_YAW:
            break;

        // Do commands
        case MAV_CMD_DO_JUMP:
        case MAV_CMD_DO_CHANGE_SPEED:
        case MAV_CMD_DO_SET_HOME:
        case MAV_CMD_DO_SET_SERVO:
        case MAV_CMD_DO_SET_RELAY:
        case MAV_CMD_DO_REPEAT_SERVO:
        case MAV_CMD_DO_REPEAT_RELAY:
        case MAV_CMD_DO_SET_ROI:
        case MAV_CMD_DO_DIGICAM_CONTROL:
        case MAV_CMD_DO_MOUNT_CONTROL:
        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            break;

        default:
            // Unsupported command
            char buf[80];
            snprintf(buf, sizeof(buf), "Unsupported waypoint command: %u",
                     item.command);
            m_channel.sendText(MAV_SEVERITY_WARNING, buf);
            return MAV_MISSION_UNSUPPORTED;
    }

    // Validate coordinates for navigation commands
    if (item.command <= MAV_CMD_NAV_LAST) {
        // Check latitude/longitude are valid
        int32_t lat = item.x;
        int32_t lon = item.y;

        if (lat < -900000000 || lat > 900000000) {
            m_channel.sendText(MAV_SEVERITY_WARNING, "Invalid latitude");
            return MAV_MISSION_INVALID_PARAM5_X;
        }

        if (lon < -1800000000 || lon > 1800000000) {
            m_channel.sendText(MAV_SEVERITY_WARNING, "Invalid longitude");
            return MAV_MISSION_INVALID_PARAM6_Y;
        }

        // Check altitude is reasonable (allow negative for below sea level)
        float alt = item.z;
        if (std::isnan(alt) || std::isinf(alt) || alt < -500.0f || alt > 50000.0f) {
            m_channel.sendText(MAV_SEVERITY_WARNING, "Invalid altitude");
            return MAV_MISSION_INVALID_PARAM7;
        }
    }

    // Validate frame
    switch (item.frame) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_INT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        case MAV_FRAME_MISSION:
            break;

        default:
            char buf[80];
            snprintf(buf, sizeof(buf), "Unsupported frame: %u", item.frame);
            m_channel.sendText(MAV_SEVERITY_WARNING, buf);
            return MAV_MISSION_UNSUPPORTED_FRAME;
    }

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Handle MISSION_SET_CURRENT message
 *
 * Sets the current active waypoint.
 */
void MissionItemProtocol_Waypoints::handleMissionSetCurrent(
    const mavlink_message_t& msg)
{
    mavlink_mission_set_current_t packet;
    mavlink_msg_mission_set_current_decode(&msg, &packet);

    // Check if message is for us
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.target_component != m_channel.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    // Validate sequence
    if (packet.seq >= getWaypointCount()) {
        m_channel.sendText(MAV_SEVERITY_WARNING, "Invalid waypoint index");
        return;
    }

    // Set current waypoint
    if (setCurrentWaypoint(packet.seq)) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Current waypoint set to %u", packet.seq);
        m_channel.sendText(MAV_SEVERITY_INFO, buf);

        // Send confirmation
        sendCurrentWaypoint();
    } else {
        m_channel.sendText(MAV_SEVERITY_ERROR, "Failed to set current waypoint");
    }
}

/**
 * @brief Send MISSION_CURRENT message
 *
 * Sends the current active waypoint index.
 */
void MissionItemProtocol_Waypoints::sendCurrentWaypoint()
{
    uint16_t current = getCurrentWaypointIndex();

    mavlink_message_t msg;
    mavlink_msg_mission_current_pack(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        current
    );

    m_channel.sendMessage(&msg);
}

/**
 * @brief Send waypoint reached notification
 */
void MissionItemProtocol_Waypoints::sendWaypointReached(uint16_t index)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_reached_pack(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        index
    );

    m_channel.sendMessage(&msg);

    char buf[64];
    snprintf(buf, sizeof(buf), "Waypoint %u reached", index);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Get waypoint distance and bearing
 *
 * Calculates distance and bearing to a specific waypoint.
 */
extern void getWaypointDistanceBearing(uint16_t index, float& outDistanceM,
                                        float& outBearingDeg);

void MissionItemProtocol_Waypoints::sendWaypointDistance()
{
    uint16_t current = getCurrentWaypointIndex();

    if (current >= getWaypointCount()) {
        return;
    }

    float distance, bearing;
    getWaypointDistanceBearing(current, distance, bearing);

    char buf[80];
    snprintf(buf, sizeof(buf), "WP %u: %.1fm, %.0f deg",
             current, distance, bearing);
    m_channel.sendText(MAV_SEVERITY_DEBUG, buf);
}

/**
 * @brief Check if mission is valid
 *
 * Validates the entire mission for common errors.
 */
bool MissionItemProtocol_Waypoints::isMissionValid()
{
    uint16_t count = getWaypointCount();

    if (count == 0) {
        return false; // No mission loaded
    }

    // Check first command (should be takeoff or waypoint)
    mavlink_mission_item_int_t firstItem;
    if (!getWaypoint(0, firstItem)) {
        return false;
    }

    // Warn if first command is not takeoff
    if (firstItem.command != MAV_CMD_NAV_TAKEOFF &&
        firstItem.command != MAV_CMD_NAV_WAYPOINT) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "Mission should start with TAKEOFF or WAYPOINT");
    }

    return true;
}

} // namespace GCS
} // namespace EduCopter
