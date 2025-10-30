/**
 * @file GCS_Param.cpp
 * @brief Parameter management and MAVLink parameter protocol implementation
 *
 * This file implements the MAVLink parameter protocol for EduCopter, allowing
 * ground stations to discover, read, and write vehicle parameters.
 *
 * Supports:
 * - PARAM_REQUEST_LIST: List all parameters
 * - PARAM_REQUEST_READ: Read specific parameter by index or name
 * - PARAM_SET: Set parameter value
 * - PARAM_VALUE: Send parameter value to GCS
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

// External parameter system interface (implemented by vehicle)
extern uint16_t getParameterCount();
extern const char* getParameterName(uint16_t index);
extern float getParameterValue(uint16_t index);
extern bool setParameterValue(uint16_t index, float value);
extern int16_t findParameterIndex(const char* name);
extern MAV_PARAM_TYPE getParameterType(uint16_t index);

// Parameter streaming state
struct ParamStreamState {
    bool active;
    uint16_t currentIndex;
    uint16_t totalParams;
    uint32_t lastSendMS;
};

static ParamStreamState s_paramStream = {false, 0, 0, 0};

/**
 * @brief Handle PARAM_REQUEST_LIST message
 *
 * Starts streaming all parameters to the requesting system.
 * Parameters are sent incrementally to avoid overwhelming the channel.
 *
 * @param channel Channel that received the request
 * @param msg MAVLink message
 */
void GCSChannel::handleParamRequestList(const mavlink_message_t& msg)
{
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(&msg, &packet);

    // Check if request is for us
    if (packet.target_system != m_mavlink.getSystemID() &&
        packet.target_system != 0) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    // Start parameter streaming
    s_paramStream.active = true;
    s_paramStream.currentIndex = 0;
    s_paramStream.totalParams = getParameterCount();
    s_paramStream.lastSendMS = millis();

    sendText(MAV_SEVERITY_INFO, "Sending parameters...");
}

/**
 * @brief Handle PARAM_REQUEST_READ message
 *
 * Sends a specific parameter by index or name.
 *
 * @param channel Channel that received the request
 * @param msg MAVLink message
 */
void GCSChannel::handleParamRequestRead(const mavlink_message_t& msg)
{
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(&msg, &packet);

    // Check if request is for us
    if (packet.target_system != m_mavlink.getSystemID() &&
        packet.target_system != 0) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    int16_t paramIndex = -1;

    // Find parameter by index or name
    if (packet.param_index >= 0) {
        paramIndex = packet.param_index;
    } else {
        // Search by name (ensure null termination)
        char paramName[17];
        memcpy(paramName, packet.param_id, 16);
        paramName[16] = '\0';
        paramIndex = findParameterIndex(paramName);
    }

    // Send parameter if found
    if (paramIndex >= 0 && paramIndex < getParameterCount()) {
        sendParameter(paramIndex);
    } else {
        sendText(MAV_SEVERITY_WARNING, "Parameter not found");
    }
}

/**
 * @brief Handle PARAM_SET message
 *
 * Sets a parameter value and sends confirmation.
 * Validates the parameter before setting.
 *
 * @param channel Channel that received the request
 * @param msg MAVLink message
 */
void GCSChannel::handleParamSet(const mavlink_message_t& msg)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(&msg, &packet);

    // Check if request is for us
    if (packet.target_system != m_mavlink.getSystemID()) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    // Find parameter by name (ensure null termination)
    char paramName[17];
    memcpy(paramName, packet.param_id, 16);
    paramName[16] = '\0';

    int16_t paramIndex = findParameterIndex(paramName);

    if (paramIndex < 0 || paramIndex >= getParameterCount()) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Unknown parameter: %s", paramName);
        sendText(MAV_SEVERITY_WARNING, buf);
        return;
    }

    // Get current value for comparison
    float oldValue = getParameterValue(paramIndex);
    float newValue = packet.param_value;

    // Validate new value (basic sanity checks)
    if (std::isnan(newValue) || std::isinf(newValue)) {
        sendText(MAV_SEVERITY_WARNING, "Invalid parameter value");
        sendParameter(paramIndex); // Send current value
        return;
    }

    // Set the parameter
    bool success = setParameterValue(paramIndex, newValue);

    if (success) {
        // Log the change
        char buf[100];
        snprintf(buf, sizeof(buf), "Param %s: %.4f -> %.4f",
                 paramName, oldValue, newValue);
        sendText(MAV_SEVERITY_INFO, buf);

        // Send confirmation with actual value (may differ slightly)
        sendParameter(paramIndex);
    } else {
        sendText(MAV_SEVERITY_WARNING, "Parameter set failed");
        sendParameter(paramIndex); // Send current value
    }
}

/**
 * @brief Send a parameter value message
 *
 * Sends PARAM_VALUE message for a specific parameter.
 *
 * @param index Parameter index to send
 */
void GCSChannel::sendParameter(uint16_t index)
{
    if (index >= getParameterCount()) {
        return;
    }

    if (!hasPayloadSpace(MAVLINK_MSG_ID_PARAM_VALUE)) {
        return;
    }

    // Get parameter info
    const char* name = getParameterName(index);
    float value = getParameterValue(index);
    MAV_PARAM_TYPE type = getParameterType(index);
    uint16_t totalParams = getParameterCount();

    // Prepare parameter name (max 16 chars, null-terminated)
    char paramID[16];
    memset(paramID, 0, sizeof(paramID));
    strncpy(paramID, name, 16);

    // Pack and send message
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        paramID,
        value,
        type,
        totalParams,
        index
    );

    sendMessage(&msg);
}

/**
 * @brief Update parameter streaming
 *
 * Called periodically to send queued parameters.
 * Sends parameters at controlled rate to avoid overwhelming channel.
 */
void GCSChannel::updateParamStream()
{
    if (!s_paramStream.active) {
        return;
    }

    uint32_t nowMS = millis();

    // Send parameters at ~50Hz (20ms interval) to avoid overload
    if (nowMS - s_paramStream.lastSendMS < 20) {
        return;
    }

    // Send next parameter
    if (s_paramStream.currentIndex < s_paramStream.totalParams) {
        sendParameter(s_paramStream.currentIndex);
        s_paramStream.currentIndex++;
        s_paramStream.lastSendMS = nowMS;
    } else {
        // All parameters sent
        s_paramStream.active = false;
        sendText(MAV_SEVERITY_INFO, "Parameters sent");
    }
}

/**
 * @brief Send parameter value by name
 *
 * Convenience function to send parameter by name instead of index.
 *
 * @param name Parameter name
 */
void GCSChannel::sendParameterByName(const char* name)
{
    int16_t index = findParameterIndex(name);
    if (index >= 0) {
        sendParameter(index);
    }
}

/**
 * @brief Check if parameter streaming is active
 *
 * @return true if currently streaming parameters
 */
bool GCSChannel::isStreamingParams() const
{
    return s_paramStream.active;
}

/**
 * @brief Cancel ongoing parameter stream
 *
 * Stops parameter streaming if active.
 */
void GCSChannel::cancelParamStream()
{
    if (s_paramStream.active) {
        s_paramStream.active = false;
        sendText(MAV_SEVERITY_INFO, "Parameter stream cancelled");
    }
}

#if EDUCOPTER_PARAM_TABLE_ENABLED
/**
 * @brief Get parameter metadata
 *
 * Returns metadata about a parameter including min/max values,
 * default value, and units.
 *
 * @param index Parameter index
 * @param outMin Output minimum value
 * @param outMax Output maximum value
 * @param outDefault Output default value
 * @return true if metadata available
 */
extern bool getParameterMetadata(uint16_t index, float& outMin,
                                 float& outMax, float& outDefault);

bool GCSChannel::sendParameterMetadata(uint16_t index)
{
    if (index >= getParameterCount()) {
        return false;
    }

    float minVal, maxVal, defVal;
    if (!getParameterMetadata(index, minVal, maxVal, defVal)) {
        return false;
    }

    // Send as text (MAVLink doesn't have standard metadata message)
    char buf[100];
    const char* name = getParameterName(index);
    snprintf(buf, sizeof(buf), "Param %s: min=%.2f max=%.2f def=%.2f",
             name, minVal, maxVal, defVal);
    sendText(MAV_SEVERITY_INFO, buf);

    return true;
}
#endif // EDUCOPTER_PARAM_TABLE_ENABLED

#if EDUCOPTER_PARAM_PERSISTENT_ENABLED
/**
 * @brief Save parameters to persistent storage
 *
 * Saves all parameters to EEPROM/flash.
 *
 * @return true if save successful
 */
extern bool saveParameters();

MAV_RESULT GCSChannel::handleCommandPreflightStorage(const mavlink_command_long_t& cmd)
{
    // param1: 0=read, 1=write, 2=reset
    int action = (int)cmd.param1;

    if (action == 1) {
        // Write parameters
        if (saveParameters()) {
            sendText(MAV_SEVERITY_INFO, "Parameters saved");
            return MAV_RESULT_ACCEPTED;
        } else {
            sendText(MAV_SEVERITY_ERROR, "Parameter save failed");
            return MAV_RESULT_FAILED;
        }
    } else if (action == 0) {
        // Read parameters (reload from storage)
        extern bool loadParameters();
        if (loadParameters()) {
            sendText(MAV_SEVERITY_INFO, "Parameters loaded");
            return MAV_RESULT_ACCEPTED;
        } else {
            sendText(MAV_SEVERITY_ERROR, "Parameter load failed");
            return MAV_RESULT_FAILED;
        }
    } else if (action == 2) {
        // Reset to defaults
        extern bool resetParameters();
        if (resetParameters()) {
            sendText(MAV_SEVERITY_INFO, "Parameters reset to defaults");
            return MAV_RESULT_ACCEPTED;
        } else {
            sendText(MAV_SEVERITY_ERROR, "Parameter reset failed");
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_UNSUPPORTED;
}
#endif // EDUCOPTER_PARAM_PERSISTENT_ENABLED

/**
 * @brief Send parameter count
 *
 * Sends a text message with total parameter count.
 * Useful for debugging.
 */
void GCSChannel::sendParameterCount()
{
    char buf[64];
    uint16_t count = getParameterCount();
    snprintf(buf, sizeof(buf), "Total parameters: %u", count);
    sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Validate parameter index
 *
 * Checks if parameter index is valid.
 *
 * @param index Parameter index
 * @return true if valid
 */
bool GCSChannel::isValidParameterIndex(uint16_t index) const
{
    return index < getParameterCount();
}

/**
 * @brief Search for parameters by prefix
 *
 * Sends all parameters matching a name prefix.
 * Useful for parameter groups (e.g., all "PID_*" params).
 *
 * @param prefix Parameter name prefix
 */
void GCSChannel::sendParametersByPrefix(const char* prefix)
{
    if (!prefix || prefix[0] == '\0') {
        return;
    }

    size_t prefixLen = strlen(prefix);
    uint16_t count = getParameterCount();
    uint16_t matchCount = 0;

    for (uint16_t i = 0; i < count; i++) {
        const char* name = getParameterName(i);
        if (strncmp(name, prefix, prefixLen) == 0) {
            sendParameter(i);
            matchCount++;
        }
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Sent %u parameters with prefix '%s'",
             matchCount, prefix);
    sendText(MAV_SEVERITY_INFO, buf);
}

} // namespace GCS
} // namespace EduCopter
