/**
 * @file MAVLink_routing.h
 * @brief EduCopter MAVLink Message Routing System
 *
 * Routes messages between multiple MAVLink channels and components.
 * Enables communication between GCS, companion computers, and onboard systems.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "GCS_config.h"

#if EDUCOPTER_GCS_ENABLED

#include "GCS_MAVLink.h"
#include <stdint.h>

namespace EduCopter {
namespace GCS {

/// Maximum number of route entries
#define EDUCOPTER_MAX_ROUTES 20

/**
 * @struct RouteEntry
 * @brief Represents a single routing table entry
 */
struct RouteEntry {
    uint8_t systemID;         ///< Destination system ID
    uint8_t componentID;      ///< Destination component ID
    uint8_t channel;          ///< Channel to use for sending
    uint8_t mavType;          ///< MAV_TYPE of destination
    uint32_t lastSeenMS;      ///< Last time we saw this system
    bool isActive;            ///< Entry is valid
};

/**
 * @class MAVLinkRouter
 * @brief Handles routing of MAVLink messages between channels
 *
 * Automatically learns routes from heartbeat messages and forwards
 * messages to appropriate destinations.
 */
class MAVLinkRouter {
public:
    /**
     * @brief Constructor
     */
    MAVLinkRouter();

    /**
     * @brief Destructor
     */
    ~MAVLinkRouter() = default;

    // ========== INITIALIZATION ==========

    /**
     * @brief Initialize router
     */
    void initialize();

    // ========== ROUTING ==========

    /**
     * @brief Check if message should be routed and forward if needed
     * @param channel Channel message arrived on
     * @param msg Received message
     * @return true if message should be processed locally
     */
    bool checkAndForward(uint8_t channel, const mavlink_message_t& msg);

    /**
     * @brief Forward message to specific destination
     * @param msg Message to forward
     * @param targetChannel Channel to send on
     * @return true if forwarded successfully
     */
    bool forwardMessage(const mavlink_message_t& msg, uint8_t targetChannel);

    /**
     * @brief Send message to all known components with our system ID
     * @param msg Message to broadcast
     */
    void sendToComponents(const mavlink_message_t& msg);

    // ========== ROUTE LEARNING ==========

    /**
     * @brief Learn route from a received message
     * @param channel Channel message arrived on
     * @param msg Message containing routing info
     */
    void learnRoute(uint8_t channel, const mavlink_message_t& msg);

    /**
     * @brief Handle heartbeat for route learning
     * @param channel Channel heartbeat arrived on
     * @param msg Heartbeat message
     */
    void handleHeartbeat(uint8_t channel, const mavlink_message_t& msg);

    // ========== ROUTE LOOKUP ==========

    /**
     * @brief Find route for a system/component
     * @param systemID Target system ID
     * @param componentID Target component ID (0 = any)
     * @param[out] channel Output channel number
     * @return true if route found
     */
    bool findRoute(uint8_t systemID, uint8_t componentID, uint8_t& channel);

    /**
     * @brief Find component by MAV_TYPE
     * @param mavType MAV_TYPE to search for
     * @param[out] systemID Output system ID
     * @param[out] componentID Output component ID
     * @param[out] channel Output channel
     * @return true if found
     */
    bool findByMAVType(uint8_t mavType, uint8_t& systemID,
                      uint8_t& componentID, uint8_t& channel);

    // ========== ROUTE MANAGEMENT ==========

    /**
     * @brief Add or update a route
     * @param systemID System ID
     * @param componentID Component ID
     * @param channel Channel to use
     * @param mavType MAV_TYPE
     * @return true if added/updated
     */
    bool addRoute(uint8_t systemID, uint8_t componentID,
                  uint8_t channel, uint8_t mavType);

    /**
     * @brief Remove a route
     * @param systemID System ID
     * @param componentID Component ID
     * @return true if removed
     */
    bool removeRoute(uint8_t systemID, uint8_t componentID);

    /**
     * @brief Clear all routes
     */
    void clearAllRoutes();

    /**
     * @brief Remove stale routes (not seen recently)
     * @param timeoutMS Timeout in milliseconds
     */
    void removeStaleRoutes(uint32_t timeoutMS = 30000);

    // ========== STATISTICS ==========

    /**
     * @brief Get number of active routes
     * @return Route count
     */
    uint8_t getRouteCount() const { return m_routeCount; }

    /**
     * @brief Get route entry by index
     * @param index Route index
     * @return Pointer to route entry, nullptr if invalid
     */
    const RouteEntry* getRoute(uint8_t index) const;

    /**
     * @brief Get forwarded message count
     * @return Messages forwarded
     */
    uint32_t getForwardedCount() const { return m_forwardedCount; }

    // ========== CHANNEL BLOCKING ==========

    /**
     * @brief Disable routing on a channel
     * @param channel Channel to block
     */
    void blockChannel(uint8_t channel);

    /**
     * @brief Enable routing on a channel
     * @param channel Channel to unblock
     */
    void unblockChannel(uint8_t channel);

    /**
     * @brief Check if channel is blocked
     * @param channel Channel to check
     * @return true if blocked
     */
    bool isChannelBlocked(uint8_t channel) const;

private:
    // ========== MEMBER VARIABLES ==========

    RouteEntry m_routes[EDUCOPTER_MAX_ROUTES]; ///< Routing table
    uint8_t m_routeCount;                      ///< Number of active routes
    uint8_t m_blockedChannels;                 ///< Bitmask of blocked channels
    uint32_t m_forwardedCount;                 ///< Statistics counter

    // ========== HELPER METHODS ==========

    /**
     * @brief Find free slot in routing table
     * @return Index of free slot, or 0xFF if full
     */
    uint8_t findFreeSlot();

    /**
     * @brief Find existing route entry
     * @param systemID System ID
     * @param componentID Component ID
     * @return Index of route, or 0xFF if not found
     */
    uint8_t findRouteIndex(uint8_t systemID, uint8_t componentID);

    /**
     * @brief Get target system/component from message
     * @param msg Message to analyze
     * @param[out] targetSystem Target system ID
     * @param[out] targetComponent Target component ID
     * @return true if message has specific target
     */
    bool getMessageTarget(const mavlink_message_t& msg,
                         uint8_t& targetSystem, uint8_t& targetComponent);

    /**
     * @brief Check if message should be forwarded
     * @param msg Message to check
     * @return true if forwardable
     */
    bool isForwardableMessage(const mavlink_message_t& msg);
};

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
