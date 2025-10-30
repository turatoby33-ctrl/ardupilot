/**
 * @file MissionItemProtocol.cpp
 * @brief Base implementation for MAVLink mission item protocol
 *
 * This file implements the common mission item transfer protocol used for
 * waypoints, fence points, and rally points. It handles the state machine
 * for uploading and downloading mission items from ground stations.
 *
 * Protocol flow for download (GCS reads from vehicle):
 * 1. GCS sends MISSION_REQUEST_LIST
 * 2. Vehicle responds with MISSION_COUNT
 * 3. GCS sends MISSION_REQUEST_INT for each item
 * 4. Vehicle responds with MISSION_ITEM_INT
 * 5. GCS sends MISSION_ACK when complete
 *
 * Protocol flow for upload (GCS writes to vehicle):
 * 1. GCS sends MISSION_COUNT
 * 2. Vehicle requests items with MISSION_REQUEST_INT
 * 3. GCS sends MISSION_ITEM_INT
 * 4. Vehicle sends MISSION_ACK when complete
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "MissionItemProtocol.h"
#include "GCS.h"
#include <cstring>

namespace EduCopter {
namespace GCS {

/**
 * @brief Mission transfer state
 */
enum class MissionState : uint8_t {
    IDLE = 0,
    DOWNLOAD_REQUESTED,  // GCS requested mission download
    DOWNLOADING,         // Sending items to GCS
    UPLOAD_STARTED,      // GCS initiated upload
    UPLOADING,          // Receiving items from GCS
    UPLOAD_COMPLETE     // Upload done, waiting for ACK
};

/**
 * @brief Mission transfer context
 */
struct MissionContext {
    MissionState state;
    uint8_t targetSystem;
    uint8_t targetComponent;
    uint16_t itemCount;
    uint16_t currentIndex;
    uint32_t lastActivityMS;
    MAV_MISSION_TYPE missionType;
};

static MissionContext s_context = {
    MissionState::IDLE, 0, 0, 0, 0, 0, MAV_MISSION_TYPE_MISSION
};

// Timeout for mission transfers (30 seconds)
static const uint32_t MISSION_TIMEOUT_MS = 30000;

/**
 * @brief Constructor
 */
MissionItemProtocol::MissionItemProtocol(GCSChannel& channel,
                                         MAV_MISSION_TYPE type)
    : m_channel(channel)
    , m_missionType(type)
{
}

/**
 * @brief Update mission protocol state machine
 *
 * Handles timeouts and retries.
 */
void MissionItemProtocol::update()
{
    if (s_context.state == MissionState::IDLE) {
        return;
    }

    // Check for timeout
    uint32_t nowMS = millis();
    if (nowMS - s_context.lastActivityMS > MISSION_TIMEOUT_MS) {
        // Timeout - abort transfer
        sendAck(MAV_MISSION_ERROR);
        resetState();
        m_channel.sendText(MAV_SEVERITY_WARNING, "Mission transfer timeout");
    }
}

/**
 * @brief Handle MISSION_REQUEST_LIST message
 *
 * GCS is requesting to download the mission from vehicle.
 */
void MissionItemProtocol::handleMissionRequestList(const mavlink_message_t& msg)
{
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(&msg, &packet);

    // Check if request is for us and correct mission type
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.target_component != m_channel.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    if (packet.mission_type != m_missionType) {
        return; // Different mission type
    }

    // Start download sequence
    s_context.state = MissionState::DOWNLOAD_REQUESTED;
    s_context.targetSystem = msg.sysid;
    s_context.targetComponent = msg.compid;
    s_context.itemCount = getItemCount();
    s_context.currentIndex = 0;
    s_context.lastActivityMS = millis();
    s_context.missionType = m_missionType;

    // Send mission count
    sendCount();

    if (s_context.itemCount > 0) {
        s_context.state = MissionState::DOWNLOADING;
    } else {
        s_context.state = MissionState::IDLE;
    }
}

/**
 * @brief Handle MISSION_REQUEST_INT message
 *
 * GCS is requesting a specific mission item.
 */
void MissionItemProtocol::handleMissionRequestInt(const mavlink_message_t& msg)
{
    mavlink_mission_request_int_t packet;
    mavlink_msg_mission_request_int_decode(&msg, &packet);

    // Verify we're in correct state
    if (s_context.state != MissionState::DOWNLOADING &&
        s_context.state != MissionState::UPLOAD_STARTED &&
        s_context.state != MissionState::UPLOADING) {
        return;
    }

    // Check if request is for us and correct mission type
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.mission_type != m_missionType) {
        return;
    }

    // Validate index
    if (packet.seq >= s_context.itemCount) {
        sendAck(MAV_MISSION_INVALID_SEQUENCE);
        resetState();
        return;
    }

    s_context.lastActivityMS = millis();

    // Send requested item
    MAV_MISSION_RESULT result = sendItem(packet.seq);

    if (result != MAV_MISSION_ACCEPTED) {
        sendAck(result);
        resetState();
    }
}

/**
 * @brief Handle MISSION_COUNT message
 *
 * GCS is starting an upload of mission items to vehicle.
 */
void MissionItemProtocol::handleMissionCount(const mavlink_message_t& msg)
{
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(&msg, &packet);

    // Check if request is for us and correct mission type
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.target_component != m_channel.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    if (packet.mission_type != m_missionType) {
        return;
    }

    // Validate count
    if (packet.count > getMaxItemCount()) {
        sendAck(MAV_MISSION_NO_SPACE);
        return;
    }

    // Start upload sequence
    s_context.state = MissionState::UPLOAD_STARTED;
    s_context.targetSystem = msg.sysid;
    s_context.targetComponent = msg.compid;
    s_context.itemCount = packet.count;
    s_context.currentIndex = 0;
    s_context.lastActivityMS = millis();
    s_context.missionType = m_missionType;

    // Clear existing items
    MAV_MISSION_RESULT result = clearAllItems();
    if (result != MAV_MISSION_ACCEPTED) {
        sendAck(result);
        resetState();
        return;
    }

    if (packet.count == 0) {
        // Empty mission - just ACK
        sendAck(MAV_MISSION_ACCEPTED);
        resetState();
        return;
    }

    // Request first item
    requestItem(0);
    s_context.state = MissionState::UPLOADING;
}

/**
 * @brief Handle MISSION_ITEM_INT message
 *
 * GCS is sending a mission item during upload.
 */
void MissionItemProtocol::handleMissionItemInt(const mavlink_message_t& msg)
{
    mavlink_mission_item_int_t packet;
    mavlink_msg_mission_item_int_decode(&msg, &packet);

    // Verify we're in upload state
    if (s_context.state != MissionState::UPLOADING) {
        return;
    }

    // Check if item is for us and correct mission type
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.mission_type != m_missionType) {
        return;
    }

    // Validate sequence
    if (packet.seq != s_context.currentIndex) {
        sendAck(MAV_MISSION_INVALID_SEQUENCE);
        resetState();
        return;
    }

    s_context.lastActivityMS = millis();

    // Store the item
    MAV_MISSION_RESULT result = storeItem(packet);

    if (result != MAV_MISSION_ACCEPTED) {
        sendAck(result);
        resetState();
        return;
    }

    s_context.currentIndex++;

    // Check if upload is complete
    if (s_context.currentIndex >= s_context.itemCount) {
        // All items received
        sendAck(MAV_MISSION_ACCEPTED);
        resetState();
        onUploadComplete();
    } else {
        // Request next item
        requestItem(s_context.currentIndex);
    }
}

/**
 * @brief Handle MISSION_ACK message
 *
 * GCS acknowledges mission transfer.
 */
void MissionItemProtocol::handleMissionAck(const mavlink_message_t& msg)
{
    mavlink_mission_ack_t packet;
    mavlink_msg_mission_ack_decode(&msg, &packet);

    // Check if ACK is for us and correct mission type
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.mission_type != m_missionType) {
        return;
    }

    // Reset state
    resetState();

    if (packet.type == MAV_MISSION_ACCEPTED) {
        onDownloadComplete();
    } else {
        m_channel.sendText(MAV_SEVERITY_WARNING, "Mission transfer failed");
    }
}

/**
 * @brief Handle MISSION_CLEAR_ALL message
 *
 * GCS requests clearing all mission items.
 */
void MissionItemProtocol::handleMissionClearAll(const mavlink_message_t& msg)
{
    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(&msg, &packet);

    // Check if request is for us and correct mission type
    if (packet.target_system != m_channel.getSystemID()) {
        return;
    }

    if (packet.target_component != m_channel.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    if (packet.mission_type != m_missionType) {
        return;
    }

    // Clear all items
    MAV_MISSION_RESULT result = clearAllItems();
    sendAck(result);
}

/**
 * @brief Send mission count to GCS
 */
void MissionItemProtocol::sendCount()
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        s_context.targetSystem,
        s_context.targetComponent,
        s_context.itemCount,
        m_missionType
    );

    m_channel.sendMessage(&msg);
}

/**
 * @brief Send mission item to GCS
 */
MAV_MISSION_RESULT MissionItemProtocol::sendItem(uint16_t index)
{
    mavlink_mission_item_int_t item;

    // Get item from derived class
    MAV_MISSION_RESULT result = getItem(index, item);

    if (result != MAV_MISSION_ACCEPTED) {
        return result;
    }

    // Pack and send
    mavlink_message_t msg;
    mavlink_msg_mission_item_int_encode(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        &item
    );

    m_channel.sendMessage(&msg);

    return MAV_MISSION_ACCEPTED;
}

/**
 * @brief Request mission item from GCS
 */
void MissionItemProtocol::requestItem(uint16_t index)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_int_pack(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        s_context.targetSystem,
        s_context.targetComponent,
        index,
        m_missionType
    );

    m_channel.sendMessage(&msg);
}

/**
 * @brief Send mission ACK to GCS
 */
void MissionItemProtocol::sendAck(MAV_MISSION_RESULT result)
{
    mavlink_message_t msg;
    mavlink_msg_mission_ack_pack(
        m_channel.getSystemID(),
        m_channel.getComponentID(),
        &msg,
        s_context.targetSystem,
        s_context.targetComponent,
        result,
        m_missionType
    );

    m_channel.sendMessage(&msg);
}

/**
 * @brief Reset protocol state
 */
void MissionItemProtocol::resetState()
{
    s_context.state = MissionState::IDLE;
    s_context.itemCount = 0;
    s_context.currentIndex = 0;
}

/**
 * @brief Check if transfer is in progress
 */
bool MissionItemProtocol::isTransferInProgress() const
{
    return s_context.state != MissionState::IDLE;
}

/**
 * @brief Get current transfer state
 */
const char* MissionItemProtocol::getStateName() const
{
    switch (s_context.state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::DOWNLOAD_REQUESTED: return "DOWNLOAD_REQUESTED";
        case MissionState::DOWNLOADING: return "DOWNLOADING";
        case MissionState::UPLOAD_STARTED: return "UPLOAD_STARTED";
        case MissionState::UPLOADING: return "UPLOADING";
        case MissionState::UPLOAD_COMPLETE: return "UPLOAD_COMPLETE";
        default: return "UNKNOWN";
    }
}

} // namespace GCS
} // namespace EduCopter
