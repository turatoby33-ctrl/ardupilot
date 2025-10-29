/**
 * @file GCS.cpp
 * @brief EduCopter GCS Manager Implementation
 *
 * Main GCS system coordination and channel management.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#include "GCS.h"

#if EDUCOPTER_GCS_ENABLED

#include <cstring>
#include <cstdio>
#include <cstdarg>

// External functions (must be provided by vehicle code)
extern uint32_t millis();
extern uint16_t millis16();

namespace EduCopter {
namespace GCS {

// ========== GCSChannel Implementation ==========

GCSChannel::GCSChannel(uint8_t channelID)
    : m_channelID(channelID)
    , m_active(false)
    , m_streaming(false)
    , m_mavlink(channelID)
    , m_lastHeartbeatMS(0)
    , m_gcsSystemID(0)
    , m_gcsComponentID(0)
    , m_currentBucket(0)
{
    // Initialize stream rates
    std::memset(m_streamRates, 0, sizeof(m_streamRates));

    // Initialize message buckets
    for (uint8_t i = 0; i < 10; i++) {
        m_buckets[i].messageMask = 0;
        m_buckets[i].intervalMS = 0;
        m_buckets[i].lastSentMS = 0;
    }
}

bool GCSChannel::initialize(uint8_t systemID, uint8_t componentID) {
    // Initialize MAVLink channel
    if (!m_mavlink.initialize(systemID, componentID)) {
        return false;
    }

    m_active = true;

    // Set default stream rates (4 Hz)
    for (uint8_t i = 0; i < 10; i++) {
        m_streamRates[i] = 4; // 4 Hz default
    }

    return true;
}

void GCSChannel::updateReceive(uint32_t maxTimeUS) {
    if (!m_active) {
        return;
    }

    uint32_t startUS = micros(); // Would need micros() function

    // This is a placeholder - actual implementation would:
    // 1. Read bytes from UART
    // 2. Parse with m_mavlink.parseByte()
    // 3. Handle complete messages
    // 4. Route via MAVLinkRouter

    // Pseudo-code:
    // while ((micros() - startUS) < maxTimeUS) {
    //     if (uart->available()) {
    //         uint8_t byte = uart->read();
    //         mavlink_message_t msg;
    //         if (m_mavlink.parseByte(byte, msg)) {
    //             handleMessage(msg);
    //         }
    //     }
    // }
}

void GCSChannel::updateSend() {
    if (!m_active) {
        return;
    }

    // Send queued messages from buckets
    uint16_t nowMS = millis16();

    // Try to send from current bucket
    if (m_buckets[m_currentBucket].messageMask != 0) {
        // Check if it's time to send
        uint16_t dt = nowMS - m_buckets[m_currentBucket].lastSentMS;
        if (dt >= m_buckets[m_currentBucket].intervalMS) {
            // Find next message to send from this bucket
            MessageID msgID = findNextMessageToSend();

            if (msgID != MessageID::MESSAGE_COUNT) {
                // Try to send it
                if (trySendMessage(msgID)) {
                    m_buckets[m_currentBucket].lastSentMS = nowMS;
                }
            }
        }
    }

    // Move to next bucket
    m_currentBucket = (m_currentBucket + 1) % 10;
}

void GCSChannel::sendMessage(MessageID msgID) {
    // Add to appropriate bucket based on priority
    uint8_t bucketIndex = 0;

    // High priority messages (heartbeat, status)
    if (msgID == MessageID::HEARTBEAT ||
        msgID == MessageID::SYSTEM_STATUS) {
        bucketIndex = 0;
    }
    // Medium priority (attitude, position)
    else if (msgID == MessageID::ATTITUDE ||
             msgID == MessageID::GLOBAL_POSITION) {
        bucketIndex = 1;
    }
    // Low priority (everything else)
    else {
        bucketIndex = 2;
    }

    // Add to bucket
    m_buckets[bucketIndex].messageMask |= (1UL << static_cast<uint8_t>(msgID));
}

void GCSChannel::sendText(MAV_SEVERITY severity, const char* text) {
    if (!m_active) {
        return;
    }

    // Pack STATUSTEXT message
    mavlink_message_t msg;
    mavlink_msg_statustext_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        severity,
        text,
        0, // id
        0  // chunk_seq
    );

    // Send immediately
    uint16_t len;
    const uint8_t* buffer = m_mavlink.packMessage(msg, len);

    // Would send buffer via UART here
    // uart->write(buffer, len);
}

void GCSChannel::sendTextF(MAV_SEVERITY severity, const char* fmt, ...) {
    char text[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    sendText(severity, text);
}

uint16_t GCSChannel::getTxSpace() const {
    // Would query UART TX buffer space
    // For now, return a reasonable value
    return 1024;
}

bool GCSChannel::hasPayloadSpace(uint32_t msgID) const {
    uint16_t required = getRequiredBufferSpace(msgID, m_mavlink.isMAVLink2());
    return getTxSpace() >= required;
}

bool GCSChannel::trySendMessage(MessageID msgID) {
    // Default implementation - override in vehicle-specific code
    // This would call the appropriate send function based on msgID

    switch (msgID) {
        case MessageID::HEARTBEAT:
            sendHeartbeat();
            return true;

        case MessageID::SYSTEM_STATUS:
            sendSystemStatus();
            return true;

        case MessageID::ATTITUDE:
            sendAttitude();
            return true;

        // Add more cases...

        default:
            return false;
    }
}

void GCSChannel::handleMessage(const mavlink_message_t& msg) {
    // Route to appropriate handler
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_INT:
            handleCommandInt(msg);
            break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handleParamRequestList(msg);
            break;

        case MAVLINK_MSG_ID_PARAM_SET:
            handleParamSet(msg);
            break;

        case MAVLINK_MSG_ID_MISSION_COUNT:
            handleMissionCount(msg);
            break;

        // Add more handlers...

        default:
            // Unknown message
            break;
    }
}

MessageID GCSChannel::findNextMessageToSend() {
    // Find first message in current bucket
    for (uint8_t i = 0; i < static_cast<uint8_t>(MessageID::MESSAGE_COUNT); i++) {
        if (m_buckets[m_currentBucket].messageMask & (1UL << i)) {
            // Clear this message from mask
            m_buckets[m_currentBucket].messageMask &= ~(1UL << i);
            return static_cast<MessageID>(i);
        }
    }

    return MessageID::MESSAGE_COUNT; // No messages
}

void GCSChannel::scheduleMessage(MessageID msgID, uint16_t intervalMS) {
    // Determine bucket based on interval
    uint8_t bucketIndex = 0;

    if (intervalMS <= 100) bucketIndex = 0;      // 10+ Hz
    else if (intervalMS <= 250) bucketIndex = 1; // 4-10 Hz
    else if (intervalMS <= 500) bucketIndex = 2; // 2-4 Hz
    else if (intervalMS <= 1000) bucketIndex = 3; // 1-2 Hz
    else bucketIndex = 4;                         // < 1 Hz

    m_buckets[bucketIndex].intervalMS = intervalMS;
    m_buckets[bucketIndex].messageMask |= (1UL << static_cast<uint8_t>(msgID));
}

// ========== Common Send Functions (Placeholders) ==========

void GCSChannel::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        getBaseMode(),
        0, // custom_mode
        getSystemStatus()
    );

    uint16_t len;
    const uint8_t* buffer = m_mavlink.packMessage(msg, len);
    // Send via UART
}

void GCSChannel::sendSystemStatus() {
    // Placeholder - would gather system status and send
}

void GCSChannel::sendAttitude() {
    // Placeholder - would get attitude from AHRS and send
}

void GCSChannel::sendAttitudeQuaternion() {
    // Placeholder
}

void GCSChannel::sendGlobalPosition() {
    // Placeholder
}

void GCSChannel::sendLocalPosition() {
    // Placeholder
}

void GCSChannel::sendGPSRaw() {
    // Placeholder
}

void GCSChannel::sendRCChannels() {
    // Placeholder
}

void GCSChannel::sendServoOutputRaw() {
    // Placeholder
}

void GCSChannel::sendRawIMU() {
    // Placeholder
}

void GCSChannel::sendScaledIMU() {
    // Placeholder
}

void GCSChannel::sendScaledPressure() {
    // Placeholder
}

void GCSChannel::sendBatteryStatus() {
    // Placeholder
}

void GCSChannel::sendPowerStatus() {
    // Placeholder
}

void GCSChannel::sendMissionCurrent() {
    // Placeholder
}

void GCSChannel::sendMissionItemReached() {
    // Placeholder
}

void GCSChannel::sendParameterValue() {
    // Placeholder
}

void GCSChannel::sendVFRHUD() {
    // Placeholder
}

void GCSChannel::sendSystemTime() {
    // Placeholder
}

void GCSChannel::sendMemInfo() {
    // Placeholder
}

void GCSChannel::sendVibration() {
    // Placeholder
}

void GCSChannel::sendExtendedSysState() {
    // Placeholder
}

void GCSChannel::sendAutopilotVersion() {
    // Placeholder
}

// ========== Message Handlers (Placeholders) ==========

void GCSChannel::handleHeartbeat(const mavlink_message_t& msg) {
    m_lastHeartbeatMS = millis();
    m_gcsSystemID = msg.sysid;
    m_gcsComponentID = msg.compid;
}

void GCSChannel::handleCommandInt(const mavlink_message_t& msg) {
    // Decode and handle command
}

void GCSChannel::handleCommandLong(const mavlink_message_t& msg) {
    // Decode and handle command
}

void GCSChannel::handleParamRequestList(const mavlink_message_t& msg) {
    // Start parameter send
}

void GCSChannel::handleParamRequestRead(const mavlink_message_t& msg) {
    // Send specific parameter
}

void GCSChannel::handleParamSet(const mavlink_message_t& msg) {
    // Set parameter value
}

void GCSChannel::handleMissionRequestList(const mavlink_message_t& msg) {
    // Start mission download
}

void GCSChannel::handleMissionCount(const mavlink_message_t& msg) {
    // Start mission upload
}

void GCSChannel::handleMissionItem(const mavlink_message_t& msg) {
    // Receive mission item
}

void GCSChannel::handleRCChannelsOverride(const mavlink_message_t& msg) {
    // Override RC inputs
}

void GCSChannel::handleSetMode(const mavlink_message_t& msg) {
    // Change flight mode
}

// ========== GCS Implementation ==========

GCS* GCS::s_instance = nullptr;

GCS::GCS()
    : m_systemID(1)
    , m_channelCount(0)
    , m_waypointProtocol(nullptr)
    , m_fenceProtocol(nullptr)
    , m_rallyProtocol(nullptr)
    , m_ftp(nullptr)
    , m_statustextHead(0)
    , m_statustextTail(0)
{
    // Initialize channel array
    for (uint8_t i = 0; i < EDUCOPTER_MAX_MAVLINK_CHANNELS; i++) {
        m_channels[i] = nullptr;
    }
}

GCS& GCS::getInstance() {
    if (s_instance == nullptr) {
        s_instance = new GCS();
    }
    return *s_instance;
}

bool GCS::initialize(uint8_t systemID) {
    m_systemID = systemID;

    // Initialize router
    m_router.initialize();

    // Create protocol handlers
#if EDUCOPTER_MISSION_ENABLED
    m_waypointProtocol = new MissionItemProtocol_Waypoints();
#endif

#if EDUCOPTER_FENCE_ENABLED
    m_fenceProtocol = new MissionItemProtocol_Fence();
#endif

#if EDUCOPTER_RALLY_ENABLED
    m_rallyProtocol = new MissionItemProtocol_Rally();
#endif

#if EDUCOPTER_FTP_ENABLED
    m_ftp = new GCS_FTP();
    m_ftp->initialize();
#endif

    return true;
}

void GCS::setupUARTs() {
    // This would scan for available UARTs and create channels
    // Placeholder implementation
}

void GCS::updateReceive() {
    for (uint8_t i = 0; i < m_channelCount; i++) {
        if (m_channels[i] != nullptr) {
            m_channels[i]->updateReceive();
        }
    }
}

void GCS::updateSend() {
    for (uint8_t i = 0; i < m_channelCount; i++) {
        if (m_channels[i] != nullptr) {
            m_channels[i]->updateSend();
        }
    }
}

void GCS::sendMessage(MessageID msgID) {
    for (uint8_t i = 0; i < m_channelCount; i++) {
        if (m_channels[i] != nullptr && m_channels[i]->isActive()) {
            m_channels[i]->sendMessage(msgID);
        }
    }
}

void GCS::sendText(MAV_SEVERITY severity, const char* text) {
    for (uint8_t i = 0; i < m_channelCount; i++) {
        if (m_channels[i] != nullptr && m_channels[i]->isActive()) {
            m_channels[i]->sendText(severity, text);
        }
    }
}

void GCS::sendTextF(MAV_SEVERITY severity, const char* fmt, ...) {
    char text[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    sendText(severity, text);
}

GCSChannel* GCS::getChannel(uint8_t index) {
    if (index < m_channelCount) {
        return m_channels[index];
    }
    return nullptr;
}

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
