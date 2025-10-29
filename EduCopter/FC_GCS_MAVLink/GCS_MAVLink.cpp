/**
 * @file GCS_MAVLink.cpp
 * @brief EduCopter MAVLink Channel Implementation
 *
 * Implements MAVLink 2.0 protocol integration for a single channel.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#include "GCS_MAVLink.h"

#if EDUCOPTER_GCS_ENABLED

#include <cstring>
#include <cstdio>

namespace EduCopter {
namespace GCS {

// ========== MAVLinkChannel Implementation ==========

MAVLinkChannel::MAVLinkChannel(uint8_t channelID)
    : m_channelID(channelID)
    , m_systemID(0)
    , m_componentID(0)
#if EDUCOPTER_MAVLINK_SIGNING_ENABLED
    , m_signingEnabled(false)
#endif
{
    std::memset(&m_status, 0, sizeof(m_status));
    std::memset(&m_txMsg, 0, sizeof(m_txMsg));
    std::memset(m_txBuffer, 0, sizeof(m_txBuffer));
}

bool MAVLinkChannel::initialize(uint8_t systemID, uint8_t componentID) {
    m_systemID = systemID;
    m_componentID = componentID;

    // Initialize MAVLink status
    m_status.packet_rx_success_count = 0;
    m_status.packet_rx_drop_count = 0;

#if EDUCOPTER_MAVLINK2_ENABLED
    // Enable MAVLink 2 by default
    m_status.flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
#else
    // Force MAVLink 1
    m_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
#endif

    return true;
}

bool MAVLinkChannel::parseByte(uint8_t byte, mavlink_message_t& outMessage) {
    // Parse using MAVLink library
    mavlink_status_t* status = &m_status;
    uint8_t result = mavlink_frame_char_buffer(
        &outMessage,
        status,
        byte,
        &outMessage,
        status
    );

    if (result == MAVLINK_FRAMING_OK) {
        // Complete message received
        return true;
    }

    return false;
}

uint8_t* MAVLinkChannel::startMessage(uint32_t msgid, uint8_t len) {
    // Initialize message header
    m_txMsg.msgid = msgid;
    m_txMsg.len = len;
    m_txMsg.sysid = m_systemID;
    m_txMsg.compid = m_componentID;
    m_txMsg.seq = m_status.current_tx_seq++;

    // Return pointer to payload
    return (uint8_t*)&m_txMsg.payload64[0];
}

const uint8_t* MAVLinkChannel::finalizeMessage(uint16_t& outLength) {
    // Pack message into buffer
    outLength = mavlink_msg_to_send_buffer(m_txBuffer, &m_txMsg);

#if EDUCOPTER_MAVLINK_SIGNING_ENABLED
    if (m_signingEnabled) {
        // Add signature
        // Note: Actual signing implementation would go here
        // For now, just indicating where it would happen
    }
#endif

    return m_txBuffer;
}

const uint8_t* MAVLinkChannel::packMessage(const mavlink_message_t& msg, uint16_t& outLength) {
    // Copy message
    std::memcpy(&m_txMsg, &msg, sizeof(mavlink_message_t));

    // Pack into buffer
    outLength = mavlink_msg_to_send_buffer(m_txBuffer, &m_txMsg);

    return m_txBuffer;
}

void MAVLinkChannel::resetStatistics() {
    m_status.packet_rx_success_count = 0;
    m_status.packet_rx_drop_count = 0;
}

#if EDUCOPTER_MAVLINK_SIGNING_ENABLED

bool MAVLinkChannel::enableSigning(const uint8_t key[32], uint64_t timestamp) {
    // Copy signing key
    std::memcpy(m_signing.secret_key, key, 32);

    // Set initial timestamp
    m_signing.timestamp = timestamp;

    // Enable signing
    m_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    m_signing.link_id = m_channelID;
    m_signing.sign_outgoing = 1;

    m_signingEnabled = true;

    return true;
}

void MAVLinkChannel::disableSigning() {
    m_signingEnabled = false;
    m_signing.sign_outgoing = 0;
}

void MAVLinkChannel::updateSigningTimestamp(uint64_t timestamp) {
    if (m_signingEnabled) {
        m_signing.timestamp = timestamp;
    }
}

#endif // EDUCOPTER_MAVLINK_SIGNING_ENABLED

uint16_t MAVLinkChannel::calculateCRC(const mavlink_message_t& msg) const {
    // Use MAVLink CRC calculation
    return crc_calculate((const uint8_t*)&msg, msg.len);
}

// ========== Helper Functions ==========

bool mavlinkFrameToAltFrame(MAV_FRAME mavFrame, uint8_t& altFrame) {
    switch (mavFrame) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_INT:
            altFrame = 0; // ABSOLUTE
            return true;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            altFrame = 1; // ABOVE_HOME
            return true;

        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            altFrame = 2; // ABOVE_TERRAIN
            return true;

        default:
            return false;
    }
}

MAV_FRAME altFrameToMAVLinkFrame(uint8_t altFrame) {
    switch (altFrame) {
        case 0: // ABSOLUTE
            return MAV_FRAME_GLOBAL_INT;

        case 1: // ABOVE_HOME
            return MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

        case 2: // ABOVE_TERRAIN
            return MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;

        default:
            return MAV_FRAME_GLOBAL_INT;
    }
}

bool isValidMAVLinkMessageID(uint32_t msgid) {
    // Message IDs are valid if they're defined in the MAVLink spec
    // For now, accept all IDs (could add validation table)
    return (msgid < 300000); // Reasonable upper bound
}

const char* getMAVLinkMessageName(uint32_t msgid) {
    // Return message name based on ID
    switch (msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_SYS_STATUS: return "SYS_STATUS";
        case MAVLINK_MSG_ID_SYSTEM_TIME: return "SYSTEM_TIME";
        case MAVLINK_MSG_ID_PING: return "PING";
        case MAVLINK_MSG_ID_COMMAND_INT: return "COMMAND_INT";
        case MAVLINK_MSG_ID_COMMAND_LONG: return "COMMAND_LONG";
        case MAVLINK_MSG_ID_COMMAND_ACK: return "COMMAND_ACK";
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: return "PARAM_REQUEST_LIST";
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: return "PARAM_REQUEST_READ";
        case MAVLINK_MSG_ID_PARAM_VALUE: return "PARAM_VALUE";
        case MAVLINK_MSG_ID_PARAM_SET: return "PARAM_SET";
        case MAVLINK_MSG_ID_GPS_RAW_INT: return "GPS_RAW_INT";
        case MAVLINK_MSG_ID_ATTITUDE: return "ATTITUDE";
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_RC_CHANNELS: return "RC_CHANNELS";
        case MAVLINK_MSG_ID_MISSION_CURRENT: return "MISSION_CURRENT";
        case MAVLINK_MSG_ID_MISSION_COUNT: return "MISSION_COUNT";
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: return "MISSION_ITEM_INT";
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT: return "MISSION_REQUEST_INT";
        case MAVLINK_MSG_ID_MISSION_ACK: return "MISSION_ACK";
        case MAVLINK_MSG_ID_VFR_HUD: return "VFR_HUD";
        case MAVLINK_MSG_ID_BATTERY_STATUS: return "BATTERY_STATUS";
        case MAVLINK_MSG_ID_STATUSTEXT: return "STATUSTEXT";
        default: return "UNKNOWN";
    }
}

uint16_t getRequiredBufferSpace(uint32_t msgid, bool isMAVLink2) {
    // Get message length from MAVLink tables
    const mavlink_msg_entry_t* entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) {
        return 0;
    }

    // Calculate total required space
    uint8_t overhead = isMAVLink2 ?
        MAVLINK_NUM_NON_PAYLOAD_BYTES :
        MAVLINK_NUM_NON_PAYLOAD_BYTES_V1;

    return entry->max_msg_len + overhead;
}

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
