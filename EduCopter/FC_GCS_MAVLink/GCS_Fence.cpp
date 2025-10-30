/**
 * @file GCS_Fence.cpp
 * @brief Geofence message handlers and commands
 *
 * This file implements fence-related MAVLink message handlers including:
 * - Fence enable/disable commands
 * - Fence breach reporting
 * - Fence status messages
 * - Fence parameter configuration
 *
 * Works in conjunction with MissionItemProtocol_Fence for fence point transfers.
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>

namespace EduCopter {
namespace GCS {

#if EDUCOPTER_FENCE_ENABLED

// External fence interface (implemented by vehicle)
extern bool isFenceEnabled();
extern bool setFenceEnabled(bool enabled);
extern bool isFenceBreached();
extern float getFenceBreachDistance();
extern uint16_t getFencePointCount();
extern float getMaxFenceAltitude();
extern float getMaxFenceRadius();
extern void setMaxFenceAltitude(float altMeters);
extern void setMaxFenceRadius(float radiusMeters);

/**
 * @brief Handle MAV_CMD_DO_FENCE_ENABLE command
 *
 * Enables or disables the geofence.
 */
MAV_RESULT GCSChannel::handleCommandFenceEnable(const mavlink_command_long_t& cmd)
{
    // param1: 0=disable, 1=enable, 2=disable floor only
    uint8_t action = static_cast<uint8_t>(cmd.param1);

    bool enable = (action == 1);

    // Check if fence is configured
    if (enable && getFencePointCount() == 0) {
        sendText(MAV_SEVERITY_WARNING, "No fence points configured");
        return MAV_RESULT_FAILED;
    }

    if (setFenceEnabled(enable)) {
        sendText(MAV_SEVERITY_INFO,
                 enable ? "Fence enabled" : "Fence disabled");
        return MAV_RESULT_ACCEPTED;
    } else {
        sendText(MAV_SEVERITY_ERROR, "Fence enable/disable failed");
        return MAV_RESULT_FAILED;
    }
}

/**
 * @brief Send fence status message
 *
 * Sends FENCE_STATUS message with breach information.
 */
void GCSChannel::sendFenceStatus()
{
    if (!hasPayloadSpace(MAVLINK_MSG_ID_FENCE_STATUS)) {
        return;
    }

    uint8_t breachStatus = 0;
    uint16_t breachCount = 0;
    uint8_t breachType = 0;
    uint32_t breachTime = 0;

    if (isFenceBreached()) {
        breachStatus = 1;
        breachCount = 1; // Simple implementation
        breachType = FENCE_BREACH_BOUNDARY; // Could be more specific
        breachTime = millis();
    }

    mavlink_message_t msg;
    mavlink_msg_fence_status_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        breachStatus,
        breachCount,
        breachType,
        breachTime
    );

    sendMessage(&msg);
}

/**
 * @brief Send fence breach notification
 *
 * Sends text alert when fence is breached.
 */
void GCSChannel::sendFenceBreachNotification()
{
    if (!isFenceBreached()) {
        return;
    }

    float distance = getFenceBreachDistance();

    char buf[80];
    snprintf(buf, sizeof(buf), "FENCE BREACH! %.1fm outside boundary",
             -distance);
    sendText(MAV_SEVERITY_CRITICAL, buf);
}

/**
 * @brief Send fence configuration info
 *
 * Sends fence parameters as text messages.
 */
void GCSChannel::sendFenceInfo()
{
    char buf[100];

    bool enabled = isFenceEnabled();
    uint16_t pointCount = getFencePointCount();
    float maxAlt = getMaxFenceAltitude();
    float maxRadius = getMaxFenceRadius();

    snprintf(buf, sizeof(buf), "Fence: %s, Points: %u",
             enabled ? "ENABLED" : "DISABLED", pointCount);
    sendText(MAV_SEVERITY_INFO, buf);

    if (maxAlt > 0.0f) {
        snprintf(buf, sizeof(buf), "Max altitude: %.1fm", maxAlt);
        sendText(MAV_SEVERITY_INFO, buf);
    }

    if (maxRadius > 0.0f) {
        snprintf(buf, sizeof(buf), "Max radius: %.1fm", maxRadius);
        sendText(MAV_SEVERITY_INFO, buf);
    }
}

/**
 * @brief Handle fence parameter set
 *
 * Sets fence-related parameters like max altitude and radius.
 */
bool GCSChannel::setFenceParameter(const char* paramName, float value)
{
    if (strcmp(paramName, "FENCE_ALT_MAX") == 0) {
        if (value > 0.0f && value < 10000.0f) {
            setMaxFenceAltitude(value);
            char buf[80];
            snprintf(buf, sizeof(buf), "Fence max altitude set to %.1fm", value);
            sendText(MAV_SEVERITY_INFO, buf);
            return true;
        }
    } else if (strcmp(paramName, "FENCE_RADIUS") == 0) {
        if (value > 0.0f && value < 50000.0f) {
            setMaxFenceRadius(value);
            char buf[80];
            snprintf(buf, sizeof(buf), "Fence radius set to %.1fm", value);
            sendText(MAV_SEVERITY_INFO, buf);
            return true;
        }
    }

    return false;
}

/**
 * @brief Check fence health
 *
 * Validates fence configuration and reports issues.
 */
void GCSChannel::checkFenceHealth()
{
    if (!isFenceEnabled()) {
        return;
    }

    uint16_t pointCount = getFencePointCount();

    if (pointCount == 0) {
        sendText(MAV_SEVERITY_ERROR,
                 "Fence enabled but no points configured!");
    } else if (pointCount < 3) {
        sendText(MAV_SEVERITY_WARNING,
                 "Fence has less than 3 points");
    }

    float maxAlt = getMaxFenceAltitude();
    if (maxAlt > 0.0f && maxAlt < 10.0f) {
        sendText(MAV_SEVERITY_WARNING,
                 "Fence altitude limit is very low");
    }
}

/**
 * @brief Update fence monitoring
 *
 * Called periodically to check for fence breaches.
 */
void GCSChannel::updateFence()
{
    static uint32_t lastBreachCheckMS = 0;
    static bool wasBreached = false;

    if (!isFenceEnabled()) {
        return;
    }

    uint32_t nowMS = millis();

    // Check for breaches at 10Hz
    if (nowMS - lastBreachCheckMS < 100) {
        return;
    }

    lastBreachCheckMS = nowMS;

    bool breached = isFenceBreached();

    // Send notification on breach transition
    if (breached && !wasBreached) {
        sendFenceBreachNotification();
        sendFenceStatus();
    }

    wasBreached = breached;
}

#else // EDUCOPTER_FENCE_ENABLED

// Fence disabled - provide stub implementations
MAV_RESULT GCSChannel::handleCommandFenceEnable(const mavlink_command_long_t& cmd)
{
    sendText(MAV_SEVERITY_WARNING, "Fence not supported");
    return MAV_RESULT_UNSUPPORTED;
}

void GCSChannel::sendFenceStatus()
{
    // No-op
}

void GCSChannel::sendFenceInfo()
{
    sendText(MAV_SEVERITY_INFO, "Fence not supported in this build");
}

void GCSChannel::updateFence()
{
    // No-op
}

#endif // EDUCOPTER_FENCE_ENABLED

} // namespace GCS
} // namespace EduCopter
