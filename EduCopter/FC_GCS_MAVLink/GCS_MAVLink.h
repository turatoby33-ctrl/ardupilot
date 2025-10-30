/**
 * @file GCS_MAVLink.h
 * @brief EduCopter MAVLink Protocol Integration
 *
 * Custom MAVLink 2.0 integration layer for EduCopter.
 * Provides clean C++ wrapper around the C MAVLink library.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "GCS_config.h"

#if EDUCOPTER_GCS_ENABLED

#include <stdint.h>
#include <cstring>

// MAVLink protocol version selection
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEPARATE_HELPERS

// Configure MAVLink for EduCopter
#define MAVLINK_COMM_NUM_BUFFERS EDUCOPTER_MAX_MAVLINK_CHANNELS
#define MAVLINK_MAX_PAYLOAD_LEN 255

// MAVLink 2.0 by default
#if EDUCOPTER_MAVLINK2_ENABLED
#define MAVLINK_USE_MESSAGE_INFO
#endif

// Include MAVLink generated headers
#include <mavlink/v2.0/common/mavlink.h>
#include <mavlink/v2.0/ardupilotmega/ardupilotmega.h>

namespace EduCopter {
namespace GCS {

/**
 * @class MAVLinkChannel
 * @brief Represents a single MAVLink communication channel
 *
 * Each telemetry port (USB, UART1, UART2, etc.) gets one channel instance.
 * Handles parsing incoming bytes and formatting outgoing messages.
 */
class MAVLinkChannel {
public:
    /**
     * @brief Constructor
     * @param channelID Channel number (0-based)
     */
    explicit MAVLinkChannel(uint8_t channelID);

    /**
     * @brief Destructor
     */
    ~MAVLinkChannel() = default;

    // ========== INITIALIZATION ==========

    /**
     * @brief Initialize channel
     * @param systemID MAVLink system ID for this vehicle
     * @param componentID MAVLink component ID
     * @return true if successful
     */
    bool initialize(uint8_t systemID, uint8_t componentID);

    // ========== RECEIVING ==========

    /**
     * @brief Parse a byte from the input stream
     * @param byte Incoming byte
     * @param outMessage Output message (if complete packet received)
     * @return true if complete message received
     */
    bool parseByte(uint8_t byte, mavlink_message_t& outMessage);

    /**
     * @brief Get current parse status
     * @return Pointer to MAVLink status structure
     */
    mavlink_status_t* getStatus() { return &m_status; }

    // ========== SENDING ==========

    /**
     * @brief Start sending a message
     * @param msgid MAVLink message ID
     * @param len Payload length
     * @return Pointer to payload buffer
     */
    uint8_t* startMessage(uint32_t msgid, uint8_t len);

    /**
     * @brief Finalize and get message buffer
     * @param outLength Output: message length in bytes
     * @return Pointer to formatted message
     */
    const uint8_t* finalizeMessage(uint16_t& outLength);

    /**
     * @brief Send a pre-packed message
     * @param msg Packed MAVLink message
     * @param outLength Output: message length
     * @return Pointer to formatted message
     */
    const uint8_t* packMessage(const mavlink_message_t& msg, uint16_t& outLength);

    // ========== CHANNEL PROPERTIES ==========

    /**
     * @brief Get channel ID
     * @return Channel number
     */
    uint8_t getChannelID() const { return m_channelID; }

    /**
     * @brief Get system ID
     * @return MAVLink system ID
     */
    uint8_t getSystemID() const { return m_systemID; }

    /**
     * @brief Get component ID
     * @return MAVLink component ID
     */
    uint8_t getComponentID() const { return m_componentID; }

    /**
     * @brief Check if using MAVLink 2.0
     * @return true if MAVLink 2, false if MAVLink 1
     */
    bool isMAVLink2() const {
#if EDUCOPTER_MAVLINK2_ENABLED
        return (m_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) == 0;
#else
        return false;
#endif
    }

    /**
     * @brief Get packet overhead size
     * @return Bytes of overhead per packet
     */
    uint8_t getPacketOverhead() const {
        return isMAVLink2() ? MAVLINK_NUM_NON_PAYLOAD_BYTES : MAVLINK_NUM_NON_PAYLOAD_BYTES_V1;
    }

    // ========== STATISTICS ==========

    /**
     * @brief Get number of packets received
     * @return Packet count
     */
    uint32_t getPacketsReceived() const { return m_status.packet_rx_success_count; }

    /**
     * @brief Get number of packets dropped
     * @return Dropped packet count
     */
    uint32_t getPacketsDropped() const { return m_status.packet_rx_drop_count; }

    /**
     * @brief Reset statistics
     */
    void resetStatistics();

    // ========== SIGNING (Security) ==========

#if EDUCOPTER_MAVLINK_SIGNING_ENABLED
    /**
     * @brief Enable message signing
     * @param key 32-byte signing key
     * @param timestamp Initial timestamp
     * @return true if successful
     */
    bool enableSigning(const uint8_t key[32], uint64_t timestamp);

    /**
     * @brief Disable message signing
     */
    void disableSigning();

    /**
     * @brief Check if signing is enabled
     * @return true if signing enabled
     */
    bool isSigningEnabled() const { return m_signingEnabled; }

    /**
     * @brief Update signing timestamp
     * @param timestamp New timestamp
     */
    void updateSigningTimestamp(uint64_t timestamp);
#endif

private:
    // ========== MEMBER VARIABLES ==========

    uint8_t m_channelID;        ///< Channel number (0-based)
    uint8_t m_systemID;         ///< MAVLink system ID
    uint8_t m_componentID;      ///< MAVLink component ID

    mavlink_status_t m_status;  ///< Parse status
    mavlink_message_t m_txMsg;  ///< Message being sent
    uint8_t m_txBuffer[MAVLINK_MAX_PACKET_LEN]; ///< TX buffer

#if EDUCOPTER_MAVLINK_SIGNING_ENABLED
    bool m_signingEnabled;      ///< Signing enabled flag
    mavlink_signing_t m_signing; ///< Signing state
    mavlink_signing_streams_t m_signingStreams; ///< Signing streams
#endif

    // ========== HELPER METHODS ==========

    /**
     * @brief Calculate CRC for a message
     * @param msg Message to CRC
     * @return CRC value
     */
    uint16_t calculateCRC(const mavlink_message_t& msg) const;
};

// ========== GLOBAL HELPER FUNCTIONS ==========

/**
 * @brief Convert MAVLink frame to location frame
 * @param mavFrame MAVLink frame type
 * @param[out] altFrame Output altitude frame
 * @return true if conversion successful
 */
bool mavlinkFrameToAltFrame(MAV_FRAME mavFrame, uint8_t& altFrame);

/**
 * @brief Convert location frame to MAVLink frame
 * @param altFrame Altitude frame
 * @return MAVLink frame type
 */
MAV_FRAME altFrameToMAVLinkFrame(uint8_t altFrame);

/**
 * @brief Check if message ID is valid
 * @param msgid MAVLink message ID
 * @return true if valid
 */
bool isValidMAVLinkMessageID(uint32_t msgid);

/**
 * @brief Get message name from ID
 * @param msgid MAVLink message ID
 * @return Message name string
 */
const char* getMAVLinkMessageName(uint32_t msgid);

/**
 * @brief Calculate required buffer space for a message
 * @param msgid MAVLink message ID
 * @param isMAVLink2 true if using MAVLink 2.0
 * @return Required bytes
 */
uint16_t getRequiredBufferSpace(uint32_t msgid, bool isMAVLink2);

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
