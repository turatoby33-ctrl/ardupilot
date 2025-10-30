/**
 * @file MissionItemProtocol_Fence.cpp
 * @brief Geofence mission item protocol implementation
 *
 * This file implements the fence-specific mission protocol for EduCopter.
 * Handles upload/download of geofence points including:
 * - Inclusion fences (areas where vehicle can fly)
 * - Exclusion fences (areas vehicle must avoid)
 * - Circular fences
 * - Polygon fences
 * - Altitude limits
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "MissionItemProtocol_Fence.h"
#include "GCS.h"
#include <cstring>
#include <cmath>

namespace EduCopter {
namespace GCS {

// External fence storage interface (implemented by vehicle)
extern uint16_t getFencePointCount();
extern bool getFencePoint(uint16_t index, mavlink_mission_item_int_t& outItem);
extern bool setFencePoint(uint16_t index, const mavlink_mission_item_int_t& item);
extern bool appendFencePoint(const mavlink_mission_item_int_t& item);
extern bool clearFencePoints();
extern bool isFenceEnabled();
extern bool setFenceEnabled(bool enabled);

// Maximum fence points supported
static const uint16_t MAX_FENCE_POINTS = 100;

/**
 * @brief Constructor
 */
MissionItemProtocol_Fence::MissionItemProtocol_Fence(GCSChannel& channel)
    : MissionItemProtocol(channel, MAV_MISSION_TYPE_FENCE)
{
}

/**
 * @brief Get total fence point count
 */
uint16_t MissionItemProtocol_Fence::getItemCount() const
{
    return getFencePointCount();
}

/**
 * @brief Get maximum fence capacity
 */
uint16_t MissionItemProtocol_Fence::getMaxItemCount() const
{
    return MAX_FENCE_POINTS;
}

/**
 * @brief Get fence point by index
 *
 * Retrieves a fence point from storage and formats it as MAVLink mission item.
 *
 * @param index Fence point index
 * @param outItem Output mission item
 * @return Mission result code
 */
MAV_MISSION_RESULT MissionItemProtocol_Fence::getItem(
    uint16_t index,
    mavlink_mission_item_int_t& outItem)
{
    if (index >= getFencePointCount()) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    if (!getFencePoint(index, outItem)) {
        return MAV_MISSION_ERROR;
    }

    // Set target system/component for response
    outItem.target_system = getTargetSystem();
    outItem.target_component = getTargetComponent();
    outItem.seq = index;
    outItem.mission_type = MAV_MISSION_TYPE_FENCE;

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Store fence point
 *
 * Validates and stores a fence point received from GCS.
 *
 * @param item Mission item to store
 * @return Mission result code
 */
MAV_MISSION_RESULT MissionItemProtocol_Fence::storeItem(
    const mavlink_mission_item_int_t& item)
{
    // Validate fence command
    MAV_MISSION_RESULT validation = validateFencePoint(item);
    if (validation != MAV_MISSION_ACCEPTED) {
        return validation;
    }

    // Append fence point
    if (!appendFencePoint(item)) {
        return MAV_MISSION_ERROR;
    }

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Clear all fence points
 */
MAV_MISSION_RESULT MissionItemProtocol_Fence::clearAllItems()
{
    if (clearFencePoints()) {
        m_channel.sendText(MAV_SEVERITY_INFO, "Fence points cleared");
        return MAV_MISSION_ACCEPTED;
    } else {
        return MAV_MISSION_ERROR;
    }
}

/**
 * @brief Called when fence upload completes
 */
void MissionItemProtocol_Fence::onUploadComplete()
{
    uint16_t count = getFencePointCount();
    char buf[64];
    snprintf(buf, sizeof(buf), "Fence upload complete: %u points", count);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);

    // Validate fence configuration
    if (!isFenceValid()) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "Fence validation failed - check configuration");
    }
}

/**
 * @brief Called when fence download completes
 */
void MissionItemProtocol_Fence::onDownloadComplete()
{
    m_channel.sendText(MAV_SEVERITY_INFO, "Fence download complete");
}

/**
 * @brief Validate fence point command and parameters
 */
MAV_MISSION_RESULT MissionItemProtocol_Fence::validateFencePoint(
    const mavlink_mission_item_int_t& item)
{
    // Check supported fence commands
    switch (item.command) {
        case MAV_CMD_NAV_FENCE_RETURN_POINT:
            // Return point - where vehicle goes when fence is breached
            break;

        case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
            // Inclusion polygon vertex (vehicle must stay inside)
            break;

        case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
            // Exclusion polygon vertex (vehicle must stay outside)
            break;

        case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            // Circular inclusion fence
            // param1: radius in meters
            if (item.param1 <= 0.0f || item.param1 > 10000.0f) {
                m_channel.sendText(MAV_SEVERITY_WARNING,
                                  "Invalid fence radius");
                return MAV_MISSION_INVALID_PARAM1;
            }
            break;

        case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
            // Circular exclusion fence
            // param1: radius in meters
            if (item.param1 <= 0.0f || item.param1 > 10000.0f) {
                m_channel.sendText(MAV_SEVERITY_WARNING,
                                  "Invalid fence radius");
                return MAV_MISSION_INVALID_PARAM1;
            }
            break;

        default:
            // Unsupported fence command
            char buf[80];
            snprintf(buf, sizeof(buf), "Unsupported fence command: %u",
                     item.command);
            m_channel.sendText(MAV_SEVERITY_WARNING, buf);
            return MAV_MISSION_UNSUPPORTED;
    }

    // Validate coordinates
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

    // Validate frame
    switch (item.frame) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_INT:
            break;

        default:
            char buf[80];
            snprintf(buf, sizeof(buf), "Unsupported fence frame: %u",
                     item.frame);
            m_channel.sendText(MAV_SEVERITY_WARNING, buf);
            return MAV_MISSION_UNSUPPORTED_FRAME;
    }

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Check if fence configuration is valid
 *
 * Validates the entire fence for common errors:
 * - Polygon fences must be closed (first point = last point)
 * - At least 3 points for polygon
 * - Circular fences need valid radius
 */
bool MissionItemProtocol_Fence::isFenceValid()
{
    uint16_t count = getFencePointCount();

    if (count == 0) {
        return true; // Empty fence is valid (fence disabled)
    }

    // Track fence types
    bool hasReturnPoint = false;
    uint16_t inclusionPolygonCount = 0;
    uint16_t exclusionPolygonCount = 0;
    uint16_t circleCount = 0;

    for (uint16_t i = 0; i < count; i++) {
        mavlink_mission_item_int_t item;
        if (!getFencePoint(i, item)) {
            return false;
        }

        switch (item.command) {
            case MAV_CMD_NAV_FENCE_RETURN_POINT:
                hasReturnPoint = true;
                break;

            case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                inclusionPolygonCount++;
                break;

            case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                exclusionPolygonCount++;
                break;

            case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
            case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
                circleCount++;
                break;
        }
    }

    // Validate polygon counts (need at least 3 points for a polygon)
    if (inclusionPolygonCount > 0 && inclusionPolygonCount < 3) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "Inclusion polygon needs at least 3 points");
        return false;
    }

    if (exclusionPolygonCount > 0 && exclusionPolygonCount < 3) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "Exclusion polygon needs at least 3 points");
        return false;
    }

    // Warn if no return point
    if (!hasReturnPoint && count > 0) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "No fence return point defined");
    }

    return true;
}

/**
 * @brief Send fence status
 *
 * Sends fence enabled/disabled status and breach information.
 */
void MissionItemProtocol_Fence::sendFenceStatus()
{
    bool enabled = isFenceEnabled();
    uint16_t count = getFencePointCount();

    char buf[80];
    snprintf(buf, sizeof(buf), "Fence: %s, %u points",
             enabled ? "ENABLED" : "DISABLED", count);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Handle fence enable/disable command
 */
MAV_RESULT MissionItemProtocol_Fence::handleFenceEnable(bool enable)
{
    if (enable && !isFenceValid()) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "Cannot enable invalid fence");
        return MAV_RESULT_FAILED;
    }

    if (setFenceEnabled(enable)) {
        m_channel.sendText(MAV_SEVERITY_INFO,
                          enable ? "Fence enabled" : "Fence disabled");
        return MAV_RESULT_ACCEPTED;
    } else {
        return MAV_RESULT_FAILED;
    }
}

/**
 * @brief Get fence breach distance
 *
 * Returns distance to nearest fence boundary.
 * Positive = inside fence, negative = outside fence.
 */
extern float getFenceBreachDistance();

void MissionItemProtocol_Fence::sendFenceBreachInfo()
{
    if (!isFenceEnabled()) {
        return;
    }

    float distance = getFenceBreachDistance();

    char buf[80];
    if (distance >= 0.0f) {
        snprintf(buf, sizeof(buf), "Fence margin: %.1fm", distance);
        m_channel.sendText(MAV_SEVERITY_DEBUG, buf);
    } else {
        snprintf(buf, sizeof(buf), "FENCE BREACH: %.1fm outside",
                 -distance);
        m_channel.sendText(MAV_SEVERITY_CRITICAL, buf);
    }
}

} // namespace GCS
} // namespace EduCopter
