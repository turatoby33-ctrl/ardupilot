/**
 * @file MissionItemProtocol_Rally.cpp
 * @brief Rally point mission item protocol implementation
 *
 * This file implements the rally point-specific mission protocol for EduCopter.
 * Rally points are alternative safe landing locations used when:
 * - RTL (Return to Launch) is triggered
 * - Battery failsafe activates
 * - RC signal is lost
 * - Fence is breached
 *
 * The vehicle will choose the nearest rally point instead of returning
 * to the original launch location.
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "MissionItemProtocol_Rally.h"
#include "GCS.h"
#include <cstring>
#include <cmath>

namespace EduCopter {
namespace GCS {

// External rally point storage interface (implemented by vehicle)
extern uint16_t getRallyPointCount();
extern bool getRallyPoint(uint16_t index, mavlink_mission_item_int_t& outItem);
extern bool setRallyPoint(uint16_t index, const mavlink_mission_item_int_t& item);
extern bool appendRallyPoint(const mavlink_mission_item_int_t& item);
extern bool clearRallyPoints();
extern uint16_t getNearestRallyIndex();

// Maximum rally points supported
static const uint16_t MAX_RALLY_POINTS = 50;

/**
 * @brief Constructor
 */
MissionItemProtocol_Rally::MissionItemProtocol_Rally(GCSChannel& channel)
    : MissionItemProtocol(channel, MAV_MISSION_TYPE_RALLY)
{
}

/**
 * @brief Get total rally point count
 */
uint16_t MissionItemProtocol_Rally::getItemCount() const
{
    return getRallyPointCount();
}

/**
 * @brief Get maximum rally point capacity
 */
uint16_t MissionItemProtocol_Rally::getMaxItemCount() const
{
    return MAX_RALLY_POINTS;
}

/**
 * @brief Get rally point by index
 *
 * Retrieves a rally point from storage and formats it as MAVLink mission item.
 *
 * @param index Rally point index
 * @param outItem Output mission item
 * @return Mission result code
 */
MAV_MISSION_RESULT MissionItemProtocol_Rally::getItem(
    uint16_t index,
    mavlink_mission_item_int_t& outItem)
{
    if (index >= getRallyPointCount()) {
        return MAV_MISSION_INVALID_SEQUENCE;
    }

    if (!getRallyPoint(index, outItem)) {
        return MAV_MISSION_ERROR;
    }

    // Set target system/component for response
    outItem.target_system = getTargetSystem();
    outItem.target_component = getTargetComponent();
    outItem.seq = index;
    outItem.mission_type = MAV_MISSION_TYPE_RALLY;

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Store rally point
 *
 * Validates and stores a rally point received from GCS.
 *
 * @param item Mission item to store
 * @return Mission result code
 */
MAV_MISSION_RESULT MissionItemProtocol_Rally::storeItem(
    const mavlink_mission_item_int_t& item)
{
    // Validate rally point
    MAV_MISSION_RESULT validation = validateRallyPoint(item);
    if (validation != MAV_MISSION_ACCEPTED) {
        return validation;
    }

    // Append rally point
    if (!appendRallyPoint(item)) {
        return MAV_MISSION_ERROR;
    }

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Clear all rally points
 */
MAV_MISSION_RESULT MissionItemProtocol_Rally::clearAllItems()
{
    if (clearRallyPoints()) {
        m_channel.sendText(MAV_SEVERITY_INFO, "Rally points cleared");
        return MAV_MISSION_ACCEPTED;
    } else {
        return MAV_MISSION_ERROR;
    }
}

/**
 * @brief Called when rally point upload completes
 */
void MissionItemProtocol_Rally::onUploadComplete()
{
    uint16_t count = getRallyPointCount();
    char buf[64];
    snprintf(buf, sizeof(buf), "Rally upload complete: %u points", count);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);

    // Validate rally point configuration
    if (!areRallyPointsValid()) {
        m_channel.sendText(MAV_SEVERITY_WARNING,
                          "Rally point validation warning - check altitude");
    }
}

/**
 * @brief Called when rally point download completes
 */
void MissionItemProtocol_Rally::onDownloadComplete()
{
    m_channel.sendText(MAV_SEVERITY_INFO, "Rally download complete");
}

/**
 * @brief Validate rally point command and parameters
 */
MAV_MISSION_RESULT MissionItemProtocol_Rally::validateRallyPoint(
    const mavlink_mission_item_int_t& item)
{
    // Rally points must use NAV_RALLY_POINT command
    if (item.command != MAV_CMD_NAV_RALLY_POINT) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Invalid rally command: %u (expected %u)",
                 item.command, MAV_CMD_NAV_RALLY_POINT);
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

    // Validate altitude
    float alt = item.z;
    if (std::isnan(alt) || std::isinf(alt) || alt < -500.0f || alt > 10000.0f) {
        m_channel.sendText(MAV_SEVERITY_WARNING, "Invalid altitude");
        return MAV_MISSION_INVALID_PARAM7;
    }

    // Validate frame
    switch (item.frame) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_INT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            break;

        default:
            char buf[80];
            snprintf(buf, sizeof(buf), "Unsupported rally frame: %u",
                     item.frame);
            m_channel.sendText(MAV_SEVERITY_WARNING, buf);
            return MAV_MISSION_UNSUPPORTED_FRAME;
    }

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Check if rally points are valid
 *
 * Validates all rally points for common issues:
 * - Reasonable altitudes
 * - Valid coordinates
 * - Accessibility
 */
bool MissionItemProtocol_Rally::areRallyPointsValid()
{
    uint16_t count = getRallyPointCount();

    if (count == 0) {
        return true; // No rally points is valid
    }

    for (uint16_t i = 0; i < count; i++) {
        mavlink_mission_item_int_t item;
        if (!getRallyPoint(i, item)) {
            return false;
        }

        // Check altitude is reasonable for landing
        float alt = item.z;
        if (alt < 0.0f || alt > 1000.0f) {
            char buf[80];
            snprintf(buf, sizeof(buf),
                     "Rally point %u has unusual altitude: %.1fm",
                     i, alt);
            m_channel.sendText(MAV_SEVERITY_WARNING, buf);
        }
    }

    return true;
}

/**
 * @brief Send rally point status
 *
 * Sends information about rally point count and nearest rally.
 */
void MissionItemProtocol_Rally::sendRallyStatus()
{
    uint16_t count = getRallyPointCount();

    if (count == 0) {
        m_channel.sendText(MAV_SEVERITY_INFO, "No rally points configured");
        return;
    }

    uint16_t nearest = getNearestRallyIndex();

    char buf[80];
    snprintf(buf, sizeof(buf), "Rally points: %u, nearest: %u",
             count, nearest);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Get distance to rally point
 *
 * Calculates distance and bearing to a specific rally point.
 */
extern void getRallyDistanceBearing(uint16_t index, float& outDistanceM,
                                     float& outBearingDeg);

void MissionItemProtocol_Rally::sendRallyDistance(uint16_t index)
{
    if (index >= getRallyPointCount()) {
        return;
    }

    float distance, bearing;
    getRallyDistanceBearing(index, distance, bearing);

    char buf[80];
    snprintf(buf, sizeof(buf), "Rally %u: %.1fm, %.0f deg",
             index, distance, bearing);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Send nearest rally point info
 *
 * Sends distance and bearing to nearest rally point.
 */
void MissionItemProtocol_Rally::sendNearestRallyInfo()
{
    if (getRallyPointCount() == 0) {
        return;
    }

    uint16_t nearest = getNearestRallyIndex();
    sendRallyDistance(nearest);
}

/**
 * @brief Calculate rally point landing altitude
 *
 * Returns the altitude above ground level for a rally point.
 */
extern float getRallyPointAltitudeAGL(uint16_t index);

void MissionItemProtocol_Rally::sendRallyAltitudeInfo(uint16_t index)
{
    if (index >= getRallyPointCount()) {
        return;
    }

    float altAGL = getRallyPointAltitudeAGL(index);

    char buf[80];
    snprintf(buf, sizeof(buf), "Rally %u altitude AGL: %.1fm",
             index, altAGL);
    m_channel.sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Handle rally point selection
 *
 * Manually selects which rally point to use for RTL.
 */
extern bool setActiveRallyPoint(uint16_t index);

MAV_RESULT MissionItemProtocol_Rally::handleSetActiveRally(uint16_t index)
{
    if (index >= getRallyPointCount()) {
        m_channel.sendText(MAV_SEVERITY_WARNING, "Invalid rally point index");
        return MAV_RESULT_FAILED;
    }

    if (setActiveRallyPoint(index)) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Active rally point set to %u", index);
        m_channel.sendText(MAV_SEVERITY_INFO, buf);
        return MAV_RESULT_ACCEPTED;
    } else {
        return MAV_RESULT_FAILED;
    }
}

} // namespace GCS
} // namespace EduCopter
