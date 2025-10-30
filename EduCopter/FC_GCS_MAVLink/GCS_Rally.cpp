/**
 * @file GCS_Rally.cpp
 * @brief Rally point message handlers and commands
 *
 * This file implements rally point-related MAVLink message handlers including:
 * - Rally point status reporting
 * - Nearest rally point selection
 * - Rally point distance calculation
 * - Rally-related commands
 *
 * Works in conjunction with MissionItemProtocol_Rally for rally point transfers.
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>
#include <cmath>

namespace EduCopter {
namespace GCS {

#if EDUCOPTER_RALLY_ENABLED

// External rally point interface (implemented by vehicle)
extern uint16_t getRallyPointCount();
extern uint16_t getNearestRallyIndex();
extern bool getRallyLocation(uint16_t index, int32_t& outLat, int32_t& outLon, float& outAlt);
extern void getRallyDistanceBearing(uint16_t index, float& outDistanceM, float& outBearingDeg);
extern bool setActiveRallyPoint(uint16_t index);
extern uint16_t getActiveRallyIndex();

/**
 * @brief Send rally point status
 *
 * Sends RALLY_POINT message for a specific rally point.
 */
void GCSChannel::sendRallyPoint(uint16_t index)
{
    if (index >= getRallyPointCount()) {
        return;
    }

    if (!hasPayloadSpace(MAVLINK_MSG_ID_RALLY_POINT)) {
        return;
    }

    int32_t lat, lon;
    float alt;

    if (!getRallyLocation(index, lat, lon, alt)) {
        return;
    }

    // Get total count
    uint16_t count = getRallyPointCount();

    // Convert altitude to int16_t (cm)
    int16_t altCM = static_cast<int16_t>(alt * 100.0f);

    // Rally flags (default: land immediately)
    uint8_t flags = 0;

    mavlink_message_t msg;
    mavlink_msg_rally_point_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        index,
        count,
        lat,
        lon,
        altCM,
        0,  // Break altitude (not used)
        0,  // Land direction (not used)
        flags
    );

    sendMessage(&msg);
}

/**
 * @brief Send rally point count
 *
 * Sends text message with rally point count.
 */
void GCSChannel::sendRallyInfo()
{
    uint16_t count = getRallyPointCount();

    char buf[80];
    if (count == 0) {
        sendText(MAV_SEVERITY_INFO, "No rally points configured");
    } else {
        uint16_t nearest = getNearestRallyIndex();
        snprintf(buf, sizeof(buf), "Rally points: %u, nearest: %u",
                 count, nearest);
        sendText(MAV_SEVERITY_INFO, buf);
    }
}

/**
 * @brief Send nearest rally point distance
 *
 * Sends distance and bearing to nearest rally point.
 */
void GCSChannel::sendNearestRallyDistance()
{
    uint16_t count = getRallyPointCount();

    if (count == 0) {
        return;
    }

    uint16_t nearest = getNearestRallyIndex();

    if (nearest >= count) {
        return;
    }

    float distance, bearing;
    getRallyDistanceBearing(nearest, distance, bearing);

    char buf[80];
    snprintf(buf, sizeof(buf), "Nearest rally %u: %.1fm @ %.0f deg",
             nearest, distance, bearing);
    sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Handle MAV_CMD_DO_SET_RALLY command
 *
 * Manually selects which rally point to use.
 */
MAV_RESULT GCSChannel::handleCommandSetRally(const mavlink_command_long_t& cmd)
{
    // param1: Rally point index
    uint16_t index = static_cast<uint16_t>(cmd.param1);

    if (index >= getRallyPointCount()) {
        sendText(MAV_SEVERITY_WARNING, "Invalid rally point index");
        return MAV_RESULT_FAILED;
    }

    if (setActiveRallyPoint(index)) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Active rally point: %u", index);
        sendText(MAV_SEVERITY_INFO, buf);
        return MAV_RESULT_ACCEPTED;
    } else {
        return MAV_RESULT_FAILED;
    }
}

/**
 * @brief Send active rally point info
 *
 * Sends information about currently active rally point.
 */
void GCSChannel::sendActiveRallyInfo()
{
    uint16_t count = getRallyPointCount();

    if (count == 0) {
        sendText(MAV_SEVERITY_INFO, "No rally points");
        return;
    }

    uint16_t active = getActiveRallyIndex();

    if (active >= count) {
        sendText(MAV_SEVERITY_INFO, "No active rally point");
        return;
    }

    int32_t lat, lon;
    float alt;

    if (getRallyLocation(active, lat, lon, alt)) {
        char buf[100];
        snprintf(buf, sizeof(buf),
                 "Active rally %u: %d, %d @ %.1fm",
                 active, lat, lon, alt);
        sendText(MAV_SEVERITY_INFO, buf);
    }
}

/**
 * @brief Check rally point health
 *
 * Validates rally point configuration.
 */
void GCSChannel::checkRallyHealth()
{
    uint16_t count = getRallyPointCount();

    if (count == 0) {
        return; // No rally points is OK
    }

    // Check if any rally points have invalid coordinates
    for (uint16_t i = 0; i < count; i++) {
        int32_t lat, lon;
        float alt;

        if (!getRallyLocation(i, lat, lon, alt)) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Rally point %u is invalid", i);
            sendText(MAV_SEVERITY_WARNING, buf);
            continue;
        }

        // Check for zero coordinates (likely invalid)
        if (lat == 0 && lon == 0) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "Rally point %u has zero coordinates", i);
            sendText(MAV_SEVERITY_WARNING, buf);
        }

        // Check for unreasonable altitude
        if (alt < 0.0f || alt > 5000.0f) {
            char buf[80];
            snprintf(buf, sizeof(buf),
                     "Rally point %u has unusual altitude: %.1fm",
                     i, alt);
            sendText(MAV_SEVERITY_WARNING, buf);
        }
    }
}

/**
 * @brief Send rally point distances
 *
 * Sends distance to all rally points.
 */
void GCSChannel::sendAllRallyDistances()
{
    uint16_t count = getRallyPointCount();

    if (count == 0) {
        sendText(MAV_SEVERITY_INFO, "No rally points");
        return;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "Rally distances:");
    sendText(MAV_SEVERITY_INFO, buf);

    for (uint16_t i = 0; i < count; i++) {
        float distance, bearing;
        getRallyDistanceBearing(i, distance, bearing);

        snprintf(buf, sizeof(buf), "  Rally %u: %.1fm @ %.0f deg",
                 i, distance, bearing);
        sendText(MAV_SEVERITY_INFO, buf);
    }
}

/**
 * @brief Update rally point monitoring
 *
 * Called periodically to update rally point information.
 */
void GCSChannel::updateRally()
{
    // Update nearest rally point calculation if needed
    // This is typically done by the vehicle's navigation system
}

#else // EDUCOPTER_RALLY_ENABLED

// Rally disabled - provide stub implementations
void GCSChannel::sendRallyPoint(uint16_t index)
{
    // No-op
}

void GCSChannel::sendRallyInfo()
{
    sendText(MAV_SEVERITY_INFO, "Rally points not supported in this build");
}

void GCSChannel::sendNearestRallyDistance()
{
    // No-op
}

MAV_RESULT GCSChannel::handleCommandSetRally(const mavlink_command_long_t& cmd)
{
    sendText(MAV_SEVERITY_WARNING, "Rally points not supported");
    return MAV_RESULT_UNSUPPORTED;
}

void GCSChannel::updateRally()
{
    // No-op
}

#endif // EDUCOPTER_RALLY_ENABLED

} // namespace GCS
} // namespace EduCopter
