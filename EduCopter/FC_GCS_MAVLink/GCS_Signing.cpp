/**
 * @file GCS_Signing.cpp
 * @brief MAVLink message signing implementation for security
 *
 * This file implements MAVLink 2.0 message signing using SHA-256 HMAC.
 * Message signing provides:
 * - Authentication: Verify messages come from trusted sources
 * - Integrity: Detect message tampering
 * - Replay protection: Prevent replay attacks
 *
 * Signing uses:
 * - 32-byte secret key (shared between vehicle and GCS)
 * - 48-bit timestamp (prevents replay attacks)
 * - 6-byte signature (truncated SHA-256)
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>
#include <cstdint>

namespace EduCopter {
namespace GCS {

#if EDUCOPTER_SIGNING_ENABLED

// External crypto interface (implemented by vehicle)
extern void crypto_sha256_hmac(const uint8_t* key, uint32_t keyLen,
                                const uint8_t* data, uint32_t dataLen,
                                uint8_t* outHash);
extern uint64_t getSecureTimestamp48(); // Get 48-bit timestamp
extern bool loadSigningKey(uint8_t* outKey, uint32_t keySize);
extern bool saveSigningKey(const uint8_t* key, uint32_t keySize);

// Signing configuration
static const uint32_t SIGNING_KEY_SIZE = 32; // 256 bits
static const uint32_t SIGNING_SIGNATURE_SIZE = 6; // 48 bits (truncated)

// Signing state
struct SigningState {
    bool enabled;
    bool acceptUnsigned;
    uint8_t key[SIGNING_KEY_SIZE];
    uint64_t timestamp;
    uint8_t linkID;
};

static SigningState s_signing = {false, true, {0}, 0, 0};

/**
 * @brief Initialize message signing
 *
 * Loads signing key from persistent storage.
 */
bool GCSChannel::initializeSigning()
{
    // Load signing key
    if (!loadSigningKey(s_signing.key, SIGNING_KEY_SIZE)) {
        sendText(MAV_SEVERITY_WARNING, "No signing key loaded");
        s_signing.enabled = false;
        return false;
    }

    s_signing.enabled = true;
    s_signing.acceptUnsigned = true; // Initially accept unsigned messages
    s_signing.timestamp = getSecureTimestamp48();
    s_signing.linkID = 0;

    sendText(MAV_SEVERITY_INFO, "Message signing enabled");
    return true;
}

/**
 * @brief Enable/disable message signing
 */
bool GCSChannel::setSigningEnabled(bool enabled)
{
    if (enabled && !s_signing.enabled) {
        // Trying to enable - need valid key
        return initializeSigning();
    }

    s_signing.enabled = enabled;

    sendText(MAV_SEVERITY_INFO,
             enabled ? "Signing enabled" : "Signing disabled");

    return true;
}

/**
 * @brief Check if signing is enabled
 */
bool GCSChannel::isSigningEnabled() const
{
    return s_signing.enabled;
}

/**
 * @brief Set whether to accept unsigned messages
 *
 * @param accept true to accept unsigned messages (less secure)
 */
void GCSChannel::setAcceptUnsignedMessages(bool accept)
{
    s_signing.acceptUnsigned = accept;

    char buf[80];
    snprintf(buf, sizeof(buf), "Accept unsigned: %s",
             accept ? "YES" : "NO");
    sendText(MAV_SEVERITY_INFO, buf);
}

/**
 * @brief Sign an outgoing message
 *
 * Adds signature to MAVLink 2 message.
 */
bool GCSChannel::signMessage(mavlink_message_t* msg)
{
    if (!s_signing.enabled) {
        return false; // Signing disabled
    }

    // Get current timestamp
    uint64_t timestamp = getSecureTimestamp48();

    // Ensure timestamp is monotonically increasing
    if (timestamp <= s_signing.timestamp) {
        timestamp = s_signing.timestamp + 1;
    }
    s_signing.timestamp = timestamp;

    // Prepare signing buffer
    // Format: link_id (1 + timestamp (6) + message_data + CRC (2)
    uint8_t sigBuffer[1 + 6 + 263 + 2]; // Max MAVLink 2 message
    uint32_t sigLen = 0;

    // Add link ID
    sigBuffer[sigLen++] = s_signing.linkID;

    // Add timestamp (48 bits, little-endian)
    for (int i = 0; i < 6; i++) {
        sigBuffer[sigLen++] = (timestamp >> (i * 8)) & 0xFF;
    }

    // Add message data (header + payload + checksum)
    uint32_t msgLen = msg->len + 12; // Header + payload + checksum
    memcpy(sigBuffer + sigLen, (uint8_t*)msg, msgLen);
    sigLen += msgLen;

    // Calculate HMAC-SHA256
    uint8_t hash[32];
    crypto_sha256_hmac(s_signing.key, SIGNING_KEY_SIZE,
                       sigBuffer, sigLen, hash);

    // Store signature (first 6 bytes of hash)
    msg->signature[0] = s_signing.linkID;
    for (int i = 0; i < 6; i++) {
        msg->signature[1 + i] = (timestamp >> (i * 8)) & 0xFF;
    }
    memcpy(&msg->signature[7], hash, SIGNING_SIGNATURE_SIZE);

    // Mark message as signed
    msg->incompat_flags |= MAVLINK_IFLAG_SIGNED;

    return true;
}

/**
 * @brief Verify signature on incoming message
 *
 * Returns true if signature is valid or if unsigned messages are accepted.
 */
bool GCSChannel::verifyMessageSignature(const mavlink_message_t* msg)
{
    // Check if message is signed
    if (!(msg->incompat_flags & MAVLINK_IFLAG_SIGNED)) {
        // Message is unsigned
        if (s_signing.acceptUnsigned) {
            return true; // Accept unsigned
        } else {
            sendText(MAV_SEVERITY_WARNING, "Unsigned message rejected");
            return false; // Reject unsigned
        }
    }

    if (!s_signing.enabled) {
        // We're not set up for signing
        return s_signing.acceptUnsigned;
    }

    // Extract timestamp from signature
    uint64_t msgTimestamp = 0;
    for (int i = 0; i < 6; i++) {
        msgTimestamp |= ((uint64_t)msg->signature[1 + i]) << (i * 8);
    }

    // Check for replay attack (timestamp must be newer)
    if (msgTimestamp <= s_signing.timestamp) {
        char buf[80];
        snprintf(buf, sizeof(buf), "Replay attack detected (old timestamp)");
        sendText(MAV_SEVERITY_WARNING, buf);
        return false;
    }

    // Prepare verification buffer (same as signing)
    uint8_t sigBuffer[1 + 6 + 263 + 2];
    uint32_t sigLen = 0;

    // Add link ID
    sigBuffer[sigLen++] = msg->signature[0];

    // Add timestamp
    for (int i = 0; i < 6; i++) {
        sigBuffer[sigLen++] = msg->signature[1 + i];
    }

    // Add message data
    uint32_t msgLen = msg->len + 12;
    memcpy(sigBuffer + sigLen, (const uint8_t*)msg, msgLen);
    sigLen += msgLen;

    // Calculate expected HMAC
    uint8_t hash[32];
    crypto_sha256_hmac(s_signing.key, SIGNING_KEY_SIZE,
                       sigBuffer, sigLen, hash);

    // Compare signatures (first 6 bytes)
    if (memcmp(&msg->signature[7], hash, SIGNING_SIGNATURE_SIZE) != 0) {
        sendText(MAV_SEVERITY_WARNING, "Invalid signature");
        return false;
    }

    // Signature valid - update our timestamp
    s_signing.timestamp = msgTimestamp;

    return true;
}

/**
 * @brief Handle SETUP_SIGNING message
 *
 * Configures signing parameters.
 */
void GCSChannel::handleSetupSigning(const mavlink_message_t& msg)
{
    mavlink_setup_signing_t packet;
    mavlink_msg_setup_signing_decode(&msg, &packet);

    // Check if message is for us
    if (packet.target_system != m_mavlink.getSystemID()) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    // Extract secret key
    memcpy(s_signing.key, packet.secret_key, SIGNING_KEY_SIZE);

    // Set initial timestamp
    s_signing.timestamp = packet.initial_timestamp;

    // Enable signing
    s_signing.enabled = true;
    s_signing.linkID = 0;

    // Save key to persistent storage
    if (saveSigningKey(s_signing.key, SIGNING_KEY_SIZE)) {
        sendText(MAV_SEVERITY_INFO, "Signing key saved");
    } else {
        sendText(MAV_SEVERITY_WARNING, "Failed to save signing key");
    }

    sendText(MAV_SEVERITY_INFO, "Message signing configured");
}

/**
 * @brief Send signing status
 */
void GCSChannel::sendSigningStatus()
{
    char buf[100];

    snprintf(buf, sizeof(buf), "Signing: %s",
             s_signing.enabled ? "ENABLED" : "DISABLED");
    sendText(MAV_SEVERITY_INFO, buf);

    if (s_signing.enabled) {
        snprintf(buf, sizeof(buf), "Accept unsigned: %s",
                 s_signing.acceptUnsigned ? "YES" : "NO");
        sendText(MAV_SEVERITY_INFO, buf);

        snprintf(buf, sizeof(buf), "Timestamp: %llu",
                 (unsigned long long)s_signing.timestamp);
        sendText(MAV_SEVERITY_INFO, buf);
    }
}

/**
 * @brief Generate new signing key
 *
 * Creates a new random signing key.
 */
extern void crypto_random_bytes(uint8_t* buffer, uint32_t length);

bool GCSChannel::generateSigningKey()
{
    // Generate random key
    crypto_random_bytes(s_signing.key, SIGNING_KEY_SIZE);

    // Save to storage
    if (saveSigningKey(s_signing.key, SIGNING_KEY_SIZE)) {
        sendText(MAV_SEVERITY_INFO, "New signing key generated");
        s_signing.enabled = true;
        s_signing.timestamp = getSecureTimestamp48();
        return true;
    } else {
        sendText(MAV_SEVERITY_ERROR, "Failed to save signing key");
        return false;
    }
}

#else // EDUCOPTER_SIGNING_ENABLED

// Signing disabled - provide stub implementations
bool GCSChannel::initializeSigning()
{
    return false;
}

bool GCSChannel::setSigningEnabled(bool enabled)
{
    sendText(MAV_SEVERITY_WARNING, "Signing not supported in this build");
    return false;
}

bool GCSChannel::isSigningEnabled() const
{
    return false;
}

void GCSChannel::setAcceptUnsignedMessages(bool accept)
{
    // No-op
}

bool GCSChannel::signMessage(mavlink_message_t* msg)
{
    return false;
}

bool GCSChannel::verifyMessageSignature(const mavlink_message_t* msg)
{
    return true; // Accept all messages
}

void GCSChannel::handleSetupSigning(const mavlink_message_t& msg)
{
    sendText(MAV_SEVERITY_WARNING, "Signing not supported");
}

void GCSChannel::sendSigningStatus()
{
    sendText(MAV_SEVERITY_INFO, "Signing not supported in this build");
}

#endif // EDUCOPTER_SIGNING_ENABLED

} // namespace GCS
} // namespace EduCopter
