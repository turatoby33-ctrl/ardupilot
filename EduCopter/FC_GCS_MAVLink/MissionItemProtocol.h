/**
 * @file MissionItemProtocol.h
 * @brief EduCopter Mission Item Transfer Protocol (Base Class)
 *
 * Handles upload/download of mission items (waypoints, fence, rally)
 * using MAVLink mission protocol.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "GCS_config.h"

#if EDUCOPTER_MISSION_ENABLED

#include "GCS_MAVLink.h"
#include "ap_message.h"
#include <stdint.h>

namespace EduCopter {
namespace GCS {

// Forward declaration
class GCSChannel;

/**
 * @class MissionItemProtocol
 * @brief Base class for mission item transfer protocols
 *
 * Provides common functionality for uploading/downloading waypoints,
 * geofence points, and rally points from ground control stations.
 */
class MissionItemProtocol {
public:
    /**
     * @brief Constructor
     * @param missionType Type of mission (waypoint/fence/rally)
     */
    explicit MissionItemProtocol(MAV_MISSION_TYPE missionType);

    /**
     * @brief Virtual destructor
     */
    virtual ~MissionItemProtocol() = default;

    // ========== MESSAGE HANDLERS ==========

    /**
     * @brief Handle MISSION_COUNT message (start upload)
     * @param channel Channel message arrived on
     * @param msg MAVLink message
     */
    void handleMissionCount(GCSChannel* channel, const mavlink_message_t& msg);

    /**
     * @brief Handle MISSION_ITEM_INT message
     * @param channel Channel message arrived on
     * @param msg MAVLink message
     */
    void handleMissionItem(GCSChannel* channel, const mavlink_message_t& msg);

    /**
     * @brief Handle MISSION_REQUEST_LIST message (start download)
     * @param channel Channel message arrived on
     * @param msg MAVLink message
     */
    void handleMissionRequestList(GCSChannel* channel, const mavlink_message_t& msg);

    /**
     * @brief Handle MISSION_REQUEST_INT message
     * @param channel Channel message arrived on
     * @param msg MAVLink message
     */
    void handleMissionRequestInt(GCSChannel* channel, const mavlink_message_t& msg);

    /**
     * @brief Handle MISSION_CLEAR_ALL message
     * @param channel Channel message arrived on
     * @param msg MAVLink message
     */
    void handleMissionClearAll(GCSChannel* channel, const mavlink_message_t& msg);

    /**
     * @brief Handle MISSION_WRITE_PARTIAL_LIST message
     * @param channel Channel message arrived on
     * @param msg MAVLink message
     */
    void handleMissionWritePartialList(GCSChannel* channel, const mavlink_message_t& msg);

    // ========== PROTOCOL STATE ==========

    /**
     * @brief Check if currently receiving items
     * @return true if upload in progress
     */
    bool isReceiving() const { return m_receiving; }

    /**
     * @brief Get mission type
     * @return MAV_MISSION_TYPE
     */
    MAV_MISSION_TYPE getMissionType() const { return m_missionType; }

    // ========== UPDATE ==========

    /**
     * @brief Update protocol state (check timeouts, etc.)
     * @param nowMS Current time in milliseconds
     */
    void update(uint32_t nowMS);

    /**
     * @brief Send queued mission request
     */
    void sendQueuedRequest();

protected:
    // ========== VIRTUAL METHODS (Must Override) ==========

    /**
     * @brief Get number of items in mission
     * @return Item count
     */
    virtual uint16_t getItemCount() const = 0;

    /**
     * @brief Get maximum number of items allowed
     * @return Maximum items
     */
    virtual uint16_t getMaxItems() const = 0;

    /**
     * @brief Get mission item at index
     * @param index Item index
     * @param[out] item Output mission item
     * @return MAV_MISSION_RESULT
     */
    virtual MAV_MISSION_RESULT getItem(uint16_t index,
                                      mavlink_mission_item_int_t& item) = 0;

    /**
     * @brief Append item to mission
     * @param item Mission item to add
     * @return MAV_MISSION_RESULT
     */
    virtual MAV_MISSION_RESULT appendItem(const mavlink_mission_item_int_t& item) = 0;

    /**
     * @brief Replace item in mission
     * @param index Item index
     * @param item New mission item
     * @return MAV_MISSION_RESULT
     */
    virtual MAV_MISSION_RESULT replaceItem(uint16_t index,
                                          const mavlink_mission_item_int_t& item) = 0;

    /**
     * @brief Clear all items from mission
     * @return true if successful
     */
    virtual bool clearAllItems() = 0;

    /**
     * @brief Truncate mission to specified length
     * @param count New item count
     * @return true if successful
     */
    virtual bool truncate(uint16_t count) = 0;

    /**
     * @brief Called when mission upload completes successfully
     * @param channel Channel that completed the upload
     * @return MAV_MISSION_RESULT
     */
    virtual MAV_MISSION_RESULT onComplete(GCSChannel* channel) { return MAV_MISSION_ACCEPTED; }

    /**
     * @brief Called when mission upload times out
     */
    virtual void onTimeout() {}

    // ========== HELPER METHODS ==========

    /**
     * @brief Send MISSION_ACK message
     * @param channel Channel to send on
     * @param msg Original request message
     * @param result Mission result code
     */
    void sendMissionAck(GCSChannel* channel, const mavlink_message_t& msg,
                       MAV_MISSION_RESULT result);

    /**
     * @brief Send MISSION_REQUEST_INT message
     * @param channel Channel to send on
     * @param seq Sequence number to request
     */
    void sendMissionRequest(GCSChannel* channel, uint16_t seq);

private:
    // ========== MEMBER VARIABLES ==========

    MAV_MISSION_TYPE m_missionType; ///< Type of mission
    bool m_receiving;               ///< Currently receiving items
    bool m_sending;                 ///< Currently sending items
    GCSChannel* m_activeChannel;    ///< Active transfer channel
    uint8_t m_targetSystem;         ///< GCS system ID
    uint8_t m_targetComponent;      ///< GCS component ID
    uint16_t m_itemCount;           ///< Expected number of items
    uint16_t m_itemIndex;           ///< Current item index
    uint16_t m_requestIndex;        ///< Item being requested
    uint32_t m_lastReceiveMS;       ///< Last receive time
    uint32_t m_lastRequestMS;       ///< Last request time

    // ========== HELPER METHODS ==========

    /**
     * @brief Start receiving mission items
     * @param channel Channel to receive on
     * @param count Number of items to expect
     * @param startIndex Starting index
     * @param endIndex Ending index
     * @return true if started successfully
     */
    bool startReceive(GCSChannel* channel, uint16_t count,
                     uint16_t startIndex, uint16_t endIndex);

    /**
     * @brief Cancel current transfer
     */
    void cancelTransfer();

    /**
     * @brief Check for timeout
     * @param nowMS Current time
     * @return true if timed out
     */
    bool checkTimeout(uint32_t nowMS);
};

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_MISSION_ENABLED
