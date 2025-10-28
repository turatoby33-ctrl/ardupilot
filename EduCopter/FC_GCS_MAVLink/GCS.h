/**
 * @file GCS.h
 * @brief EduCopter Ground Control Station Management
 *
 * Main GCS system that coordinates all MAVLink channels and protocols.
 * This is the primary interface for EduCopter vehicle code.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "GCS_config.h"

#if EDUCOPTER_GCS_ENABLED

#include "GCS_MAVLink.h"
#include "MAVLink_routing.h"
#include "MissionItemProtocol.h"
#include "ap_message.h"
#include <stdint.h>

namespace EduCopter {
namespace GCS {

// Forward declarations
class GCSChannel;
class MissionItemProtocol_Waypoints;
class MissionItemProtocol_Fence;
class MissionItemProtocol_Rally;
class GCS_FTP;

/**
 * @struct StatusText
 * @brief Queued text message for GCS
 */
struct StatusText {
    char text[50];              ///< Message text
    MAV_SEVERITY severity;      ///< Severity level
    uint32_t timestampMS;       ///< Creation time
    uint8_t channelMask;        ///< Channels to send on
};

/**
 * @class GCSChannel
 * @brief Represents a single GCS communication channel
 *
 * Each telemetry port (USB, UART, etc.) gets one channel.
 * Handles message sending/receiving, scheduling, and protocol state.
 */
class GCSChannel {
public:
    /**
     * @brief Constructor
     * @param channelID Channel number (0-based)
     */
    explicit GCSChannel(uint8_t channelID);

    /**
     * @brief Virtual destructor (for vehicle-specific subclasses)
     */
    virtual ~GCSChannel() = default;

    // ========== INITIALIZATION ==========

    /**
     * @brief Initialize channel
     * @param systemID MAVLink system ID
     * @param componentID MAVLink component ID
     * @return true if successful
     */
    virtual bool initialize(uint8_t systemID, uint8_t componentID);

    // ========== UPDATE LOOP ==========

    /**
     * @brief Update receive (process incoming messages)
     * @param maxTimeUS Maximum time to spend receiving (microseconds)
     */
    void updateReceive(uint32_t maxTimeUS = 1000);

    /**
     * @brief Update send (send queued messages)
     */
    void updateSend();

    // ========== MESSAGE SENDING ==========

    /**
     * @brief Queue a message to send
     * @param msgID Message identifier from MessageID enum
     */
    void sendMessage(MessageID msgID);

    /**
     * @brief Send a MAVLink message immediately
     * @param msg Pointer to packed MAVLink message
     */
    void sendMessage(const mavlink_message_t* msg);

    /**
     * @brief Send text to GCS
     * @param severity Message severity
     * @param text Message text
     */
    void sendText(MAV_SEVERITY severity, const char* text);

    /**
     * @brief Send formatted text
     * @param severity Severity
     * @param fmt printf-style format
     * @param ... Arguments
     */
    void sendTextF(MAV_SEVERITY severity, const char* fmt, ...);

    // ========== CHANNEL STATE ==========

    /**
     * @brief Check if channel is active
     * @return true if active
     */
    bool isActive() const { return m_active; }

    /**
     * @brief Check if streaming telemetry
     * @return true if streaming
     */
    bool isStreaming() const { return m_streaming; }

    /**
     * @brief Get channel ID
     */
    uint8_t getChannelID() const { return m_channelID; }

    /**
     * @brief Get MAVLink channel reference
     */
    MAVLinkChannel& getMAVLinkChannel() { return m_mavlink; }

    // ========== TX BUFFER MANAGEMENT ==========

    /**
     * @brief Get available TX buffer space
     * @return Bytes available
     */
    uint16_t getTxSpace() const;

    /**
     * @brief Check if message will fit in TX buffer
     * @param msgID MAVLink message ID
     * @return true if fits
     */
    bool hasPayloadSpace(uint32_t msgID) const;

    // ========== VIRTUAL METHODS (Override in vehicle code) ==========

    /**
     * @brief Send vehicle-specific navigation controller output
     */
    virtual void sendNavControllerOutput() = 0;

    /**
     * @brief Send PID tuning data
     */
    virtual void sendPIDTuning() = 0;

    /**
     * @brief Try to send a message
     * @param msgID Message to send
     * @return true if sent
     */
    virtual bool trySendMessage(MessageID msgID);

    /**
     * @brief Handle received message (vehicle-specific)
     * @param msg Received message
     */
    virtual void handleMessage(const mavlink_message_t& msg);

    /**
     * @brief Get vehicle base mode
     * @return MAV_MODE bits
     */
    virtual uint8_t getBaseMode() const = 0;

    /**
     * @brief Get vehicle system status
     * @return MAV_STATE
     */
    virtual MAV_STATE getSystemStatus() const = 0;

protected:
    // ========== COMMON SEND FUNCTIONS ==========

    // Core telemetry
    void sendHeartbeat();
    void sendSystemStatus();
    void sendAttitude();
    void sendAttitudeQuaternion();
    void sendGlobalPosition();
    void sendLocalPosition();
    void sendGPSRaw();
    void sendRCChannels();
    void sendServoOutputRaw();

    // Sensors
    void sendRawIMU();
    void sendScaledIMU();
    void sendScaledPressure();

    // Battery & power
    void sendBatteryStatus();
    void sendPowerStatus();

    // Mission
    void sendMissionCurrent();
    void sendMissionItemReached();

    // Parameters
    void sendParameterValue();

    // Status & debug
    void sendVFRHUD();
    void sendSystemTime();
    void sendMemInfo();
    void sendVibration();

    // Extended
    void sendExtendedSysState();
    void sendAutopilotVersion();

    // ========== PARAMETER FUNCTIONS ==========

    void sendParameter(uint16_t index);
    void sendParameterByName(const char* name);
    void updateParamStream();
    void cancelParamStream();
    bool isStreamingParams() const;
    void sendParameterCount();
    void sendParametersByPrefix(const char* prefix);
    bool isValidParameterIndex(uint16_t index) const;

    // ========== STREAM RATE FUNCTIONS ==========

    void handleRequestDataStream(const mavlink_message_t& msg);
    MAV_RESULT handleCommandGetMessageInterval(const mavlink_command_long_t& cmd);
    MessageID mavlinkIDToMessageID(uint32_t mavlinkID);
    uint16_t getDefaultMessageInterval(MessageID msgID);
    void resetStreamRates();
    void disableAllStreams();
    void sendStreamConfig();
    void setMessageInterval(MessageID msgID, uint16_t intervalMS);
    uint16_t getMessageInterval(MessageID msgID) const;

    // ========== FENCE FUNCTIONS ==========

    MAV_RESULT handleCommandFenceEnable(const mavlink_command_long_t& cmd);
    void sendFenceStatus();
    void sendFenceBreachNotification();
    void sendFenceInfo();
    bool setFenceParameter(const char* paramName, float value);
    void checkFenceHealth();
    void updateFence();

    // ========== RALLY FUNCTIONS ==========

    void sendRallyPoint(uint16_t index);
    void sendRallyInfo();
    void sendNearestRallyDistance();
    MAV_RESULT handleCommandSetRally(const mavlink_command_long_t& cmd);
    void sendActiveRallyInfo();
    void checkRallyHealth();
    void sendAllRallyDistances();
    void updateRally();

    // ========== SERVO/RELAY FUNCTIONS ==========

    MAV_RESULT handleCommandSetServo(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandSetRelay(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandRepeatServo(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandRepeatRelay(const mavlink_command_long_t& cmd);
    void updateServoRelay();
    void sendRelayStatus();

    // ========== SIGNING FUNCTIONS ==========

    bool initializeSigning();
    bool setSigningEnabled(bool enabled);
    bool isSigningEnabled() const;
    void setAcceptUnsignedMessages(bool accept);
    bool signMessage(mavlink_message_t* msg);
    bool verifyMessageSignature(const mavlink_message_t* msg);
    void handleSetupSigning(const mavlink_message_t& msg);
    void sendSigningStatus();
    bool generateSigningKey();

    // ========== SERIAL CONTROL FUNCTIONS ==========

    void handleSerialControl(const mavlink_message_t& msg);
    void sendSerialControlResponse(uint8_t device, uint8_t flags);
    void updateSerialControl();
    void closeSerialControl();
    void sendSerialStatus();
    void handleGPSPassthrough(bool enable);

    // ========== DEVICE OPERATION FUNCTIONS ==========

    void handleDeviceOpRead(const mavlink_message_t& msg);
    void handleDeviceOpWrite(const mavlink_message_t& msg);
    void sendDeviceOpReadReply(uint32_t requestID, uint8_t result,
                               const uint8_t* data, uint8_t length);
    void sendDeviceOpWriteReply(uint32_t requestID, uint8_t result);
    bool readSensorRegister(uint8_t sensorType, uint8_t regAddr, uint8_t& outValue);
    bool writeSensorRegister(uint8_t sensorType, uint8_t regAddr, uint8_t value);
    void dumpSensorRegisters(uint8_t sensorType);

    // ========== MISSION FUNCTION HELPERS ==========

    void sendCurrentWaypoint();
    void sendWaypointReached(uint16_t index);
    void sendWaypointDistance();
    bool isMissionValid();

    // ========== HELPER FUNCTIONS ==========

    uint8_t getSystemID() const { return m_mavlink.getSystemID(); }
    uint8_t getComponentID() const { return m_mavlink.getComponentID(); }

    // ========== COMMON MESSAGE HANDLERS ==========

    void handleHeartbeat(const mavlink_message_t& msg);
    void handleCommandInt(const mavlink_message_t& msg);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleParamRequestList(const mavlink_message_t& msg);
    void handleParamRequestRead(const mavlink_message_t& msg);
    void handleParamSet(const mavlink_message_t& msg);
    void handleMissionRequestList(const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItem(const mavlink_message_t& msg);
    void handleRCChannelsOverride(const mavlink_message_t& msg);
    void handleSetMode(const mavlink_message_t& msg);

    // ========== COMMAND HANDLERS ==========

    MAV_RESULT handleCommandPreflightCalibration(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandComponentArmDisarm(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandDoSetHome(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandDoSetMode(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandGetHomePosition(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandSetMessageInterval(const mavlink_command_long_t& cmd);
    MAV_RESULT handleCommandRequestMessage(const mavlink_command_long_t& cmd);

    // ========== MESSAGE SCHEDULING ==========

    /**
     * @brief Message bucket for rate-limited sending
     */
    struct MessageBucket {
        uint32_t messageMask;     ///< Bitmask of messages in bucket
        uint16_t intervalMS;      ///< Send interval
        uint16_t lastSentMS;      ///< Last send time
    };

    MessageBucket m_buckets[10];  ///< Message buckets
    uint8_t m_currentBucket;      ///< Current bucket being sent

    /**
     * @brief Find next message to send
     * @return Message ID, or MessageID::MESSAGE_COUNT if none
     */
    MessageID findNextMessageToSend();

    /**
     * @brief Add message to appropriate bucket
     * @param msgID Message to add
     * @param intervalMS Send interval
     */
    void scheduleMessage(MessageID msgID, uint16_t intervalMS);

    // ========== MEMBER VARIABLES ==========

    uint8_t m_channelID;              ///< Channel number
    bool m_active;                    ///< Channel is active
    bool m_streaming;                 ///< Currently streaming
    MAVLinkChannel m_mavlink;         ///< MAVLink protocol handler
    uint32_t m_lastHeartbeatMS;       ///< Last heartbeat time
    uint8_t m_gcsSystemID;            ///< GCS system ID
    uint8_t m_gcsComponentID;         ///< GCS component ID

    // Stream rates (parameters)
    uint16_t m_streamRates[10];       ///< Configured stream rates
};

/**
 * @class GCS
 * @brief Global GCS manager (singleton)
 *
 * Coordinates all GCS channels and provides vehicle-wide GCS services.
 */
class GCS {
public:
    /**
     * @brief Get singleton instance
     * @return GCS instance
     */
    static GCS& getInstance();

    /**
     * @brief Initialize GCS system
     * @param systemID MAVLink system ID for this vehicle
     * @return true if successful
     */
    bool initialize(uint8_t systemID = 1);

    /**
     * @brief Setup UART ports for GCS
     */
    void setupUARTs();

    // ========== UPDATE LOOP ==========

    /**
     * @brief Update all channels (receive)
     */
    void updateReceive();

    /**
     * @brief Update all channels (send)
     */
    void updateSend();

    // ========== MESSAGING ==========

    /**
     * @brief Send message to all active channels
     * @param msgID Message to send
     */
    void sendMessage(MessageID msgID);

    /**
     * @brief Send text to all channels
     * @param severity Message severity
     * @param text Message text
     */
    void sendText(MAV_SEVERITY severity, const char* text);

    /**
     * @brief Send formatted text to all channels
     */
    void sendTextF(MAV_SEVERITY severity, const char* fmt, ...);

    // ========== CHANNEL ACCESS ==========

    /**
     * @brief Get channel by index
     * @param index Channel index (0-based)
     * @return Pointer to channel, or nullptr if invalid
     */
    GCSChannel* getChannel(uint8_t index);

    /**
     * @brief Get number of active channels
     */
    uint8_t getChannelCount() const { return m_channelCount; }

    // ========== PROTOCOL HANDLERS ==========

    /**
     * @brief Get waypoint mission protocol
     */
    MissionItemProtocol_Waypoints* getWaypointProtocol() { return m_waypointProtocol; }

    /**
     * @brief Get fence protocol
     */
    MissionItemProtocol_Fence* getFenceProtocol() { return m_fenceProtocol; }

    /**
     * @brief Get rally protocol
     */
    MissionItemProtocol_Rally* getRallyProtocol() { return m_rallyProtocol; }

    /**
     * @brief Get FTP handler
     */
    GCS_FTP* getFTP() { return m_ftp; }

    /**
     * @brief Get router
     */
    MAVLinkRouter& getRouter() { return m_router; }

    // ========== SYSTEM ID ==========

    /**
     * @brief Get vehicle system ID
     */
    uint8_t getSystemID() const { return m_systemID; }

protected:
    // Prevent direct instantiation (singleton)
    GCS();
    ~GCS() = default;

    // Prevent copying
    GCS(const GCS&) = delete;
    GCS& operator=(const GCS&) = delete;

private:
    // ========== MEMBER VARIABLES ==========

    static GCS* s_instance;                   ///< Singleton instance

    uint8_t m_systemID;                       ///< Vehicle system ID
    uint8_t m_channelCount;                   ///< Number of active channels
    GCSChannel* m_channels[EDUCOPTER_MAX_MAVLINK_CHANNELS]; ///< Channel array

    // Protocol handlers
    MissionItemProtocol_Waypoints* m_waypointProtocol;
    MissionItemProtocol_Fence* m_fenceProtocol;
    MissionItemProtocol_Rally* m_rallyProtocol;
    GCS_FTP* m_ftp;

    // Routing
    MAVLinkRouter m_router;

    // Status text queue
    StatusText m_statustextQueue[EDUCOPTER_STATUSTEXT_QUEUE_SIZE];
    uint8_t m_statustextHead;
    uint8_t m_statustextTail;
};

// ========== GLOBAL ACCESS FUNCTION ==========

/**
 * @brief Get global GCS instance
 * @return GCS singleton
 */
inline GCS& gcs() {
    return GCS::getInstance();
}

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
