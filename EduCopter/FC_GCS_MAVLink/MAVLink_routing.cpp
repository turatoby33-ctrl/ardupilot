/**
 * @file MAVLink_routing.cpp
 * @brief EduCopter MAVLink Routing Implementation
 *
 * Routes messages between multiple MAVLink channels and components.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#include "MAVLink_routing.h"

#if EDUCOPTER_GCS_ENABLED

#include <cstring>

// External function to get current time (must be provided by vehicle code)
extern uint32_t millis();

namespace EduCopter {
namespace GCS {

// ========== MAVLinkRouter Implementation ==========

MAVLinkRouter::MAVLinkRouter()
    : m_routeCount(0)
    , m_blockedChannels(0)
    , m_forwardedCount(0)
{
    // Initialize routing table
    for (uint8_t i = 0; i < EDUCOPTER_MAX_ROUTES; i++) {
        m_routes[i].isActive = false;
    }
}

void MAVLinkRouter::initialize() {
    clearAllRoutes();
    m_blockedChannels = 0;
    m_forwardedCount = 0;
}

bool MAVLinkRouter::checkAndForward(uint8_t channel, const mavlink_message_t& msg) {
    // Learn route from this message
    learnRoute(channel, msg);

    // Check if message has a specific target
    uint8_t targetSystem, targetComponent;
    if (!getMessageTarget(msg, targetSystem, targetComponent)) {
        // No specific target - process locally
        return true;
    }

    // Check if message is for us
    // (This would need access to our system ID - simplified for now)
    // if (targetSystem == ourSystemID) return true;

    // Find route to target
    uint8_t targetChannel;
    if (findRoute(targetSystem, targetComponent, targetChannel)) {
        // Check if not the same channel it arrived on
        if (targetChannel != channel) {
            // Forward message
            if (forwardMessage(msg, targetChannel)) {
                m_forwardedCount++;
            }
        }
    }

    // Process locally if addressed to us
    return true;
}

bool MAVLinkRouter::forwardMessage(const mavlink_message_t& msg, uint8_t targetChannel) {
    // Check if channel is blocked
    if (isChannelBlocked(targetChannel)) {
        return false;
    }

    // Check if message type should be forwarded
    if (!isForwardableMessage(msg)) {
        return false;
    }

    // Message would be forwarded here using the channel's send function
    // This requires integration with GCS channel management
    // For now, return true to indicate forwarding was attempted

    return true;
}

void MAVLinkRouter::sendToComponents(const mavlink_message_t& msg) {
    // Find all components with our system ID and forward
    for (uint8_t i = 0; i < m_routeCount; i++) {
        if (m_routes[i].isActive) {
            // Check if this is a component (not external system)
            // Simplified: forward to all learned routes
            forwardMessage(msg, m_routes[i].channel);
        }
    }
}

void MAVLinkRouter::learnRoute(uint8_t channel, const mavlink_message_t& msg) {
    // Only learn from certain message types
    if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT &&
        msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK &&
        msg.msgid != MAVLINK_MSG_ID_PARAM_VALUE) {
        return; // Not a route-learning message
    }

    // Check if route already exists
    uint8_t idx = findRouteIndex(msg.sysid, msg.compid);

    if (idx != 0xFF) {
        // Update existing route
        m_routes[idx].channel = channel;
        m_routes[idx].lastSeenMS = millis();
    } else {
        // Add new route
        idx = findFreeSlot();
        if (idx != 0xFF) {
            m_routes[idx].systemID = msg.sysid;
            m_routes[idx].componentID = msg.compid;
            m_routes[idx].channel = channel;
            m_routes[idx].mavType = 0; // Unknown for now
            m_routes[idx].lastSeenMS = millis();
            m_routes[idx].isActive = true;
            m_routeCount++;
        }
    }
}

void MAVLinkRouter::handleHeartbeat(uint8_t channel, const mavlink_message_t& msg) {
    // Decode heartbeat to get MAV_TYPE
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    // Find or create route
    uint8_t idx = findRouteIndex(msg.sysid, msg.compid);

    if (idx == 0xFF) {
        idx = findFreeSlot();
    }

    if (idx != 0xFF) {
        m_routes[idx].systemID = msg.sysid;
        m_routes[idx].componentID = msg.compid;
        m_routes[idx].channel = channel;
        m_routes[idx].mavType = heartbeat.type;
        m_routes[idx].lastSeenMS = millis();
        m_routes[idx].isActive = true;

        if (idx >= m_routeCount) {
            m_routeCount = idx + 1;
        }
    }
}

bool MAVLinkRouter::findRoute(uint8_t systemID, uint8_t componentID, uint8_t& channel) {
    for (uint8_t i = 0; i < m_routeCount; i++) {
        if (m_routes[i].isActive &&
            m_routes[i].systemID == systemID &&
            (componentID == 0 || m_routes[i].componentID == componentID)) {
            channel = m_routes[i].channel;
            return true;
        }
    }
    return false;
}

bool MAVLinkRouter::findByMAVType(uint8_t mavType, uint8_t& systemID,
                                  uint8_t& componentID, uint8_t& channel) {
    for (uint8_t i = 0; i < m_routeCount; i++) {
        if (m_routes[i].isActive && m_routes[i].mavType == mavType) {
            systemID = m_routes[i].systemID;
            componentID = m_routes[i].componentID;
            channel = m_routes[i].channel;
            return true;
        }
    }
    return false;
}

bool MAVLinkRouter::addRoute(uint8_t systemID, uint8_t componentID,
                             uint8_t channel, uint8_t mavType) {
    // Check if route exists
    uint8_t idx = findRouteIndex(systemID, componentID);

    if (idx == 0xFF) {
        // Find free slot
        idx = findFreeSlot();
        if (idx == 0xFF) {
            return false; // No space
        }
    }

    // Add/update route
    m_routes[idx].systemID = systemID;
    m_routes[idx].componentID = componentID;
    m_routes[idx].channel = channel;
    m_routes[idx].mavType = mavType;
    m_routes[idx].lastSeenMS = millis();
    m_routes[idx].isActive = true;

    if (idx >= m_routeCount) {
        m_routeCount = idx + 1;
    }

    return true;
}

bool MAVLinkRouter::removeRoute(uint8_t systemID, uint8_t componentID) {
    uint8_t idx = findRouteIndex(systemID, componentID);
    if (idx != 0xFF) {
        m_routes[idx].isActive = false;
        // Compact routing table if this was the last entry
        while (m_routeCount > 0 && !m_routes[m_routeCount - 1].isActive) {
            m_routeCount--;
        }
        return true;
    }
    return false;
}

void MAVLinkRouter::clearAllRoutes() {
    for (uint8_t i = 0; i < EDUCOPTER_MAX_ROUTES; i++) {
        m_routes[i].isActive = false;
    }
    m_routeCount = 0;
}

void MAVLinkRouter::removeStaleRoutes(uint32_t timeoutMS) {
    uint32_t nowMS = millis();

    for (uint8_t i = 0; i < m_routeCount; i++) {
        if (m_routes[i].isActive) {
            if ((nowMS - m_routes[i].lastSeenMS) > timeoutMS) {
                m_routes[i].isActive = false;
            }
        }
    }

    // Compact table
    while (m_routeCount > 0 && !m_routes[m_routeCount - 1].isActive) {
        m_routeCount--;
    }
}

const RouteEntry* MAVLinkRouter::getRoute(uint8_t index) const {
    if (index < m_routeCount && m_routes[index].isActive) {
        return &m_routes[index];
    }
    return nullptr;
}

void MAVLinkRouter::blockChannel(uint8_t channel) {
    if (channel < 8) {
        m_blockedChannels |= (1 << channel);
    }
}

void MAVLinkRouter::unblockChannel(uint8_t channel) {
    if (channel < 8) {
        m_blockedChannels &= ~(1 << channel);
    }
}

bool MAVLinkRouter::isChannelBlocked(uint8_t channel) const {
    if (channel < 8) {
        return (m_blockedChannels & (1 << channel)) != 0;
    }
    return false;
}

// ========== Private Helper Methods ==========

uint8_t MAVLinkRouter::findFreeSlot() {
    for (uint8_t i = 0; i < EDUCOPTER_MAX_ROUTES; i++) {
        if (!m_routes[i].isActive) {
            return i;
        }
    }
    return 0xFF; // No free slots
}

uint8_t MAVLinkRouter::findRouteIndex(uint8_t systemID, uint8_t componentID) {
    for (uint8_t i = 0; i < m_routeCount; i++) {
        if (m_routes[i].isActive &&
            m_routes[i].systemID == systemID &&
            m_routes[i].componentID == componentID) {
            return i;
        }
    }
    return 0xFF; // Not found
}

bool MAVLinkRouter::getMessageTarget(const mavlink_message_t& msg,
                                     uint8_t& targetSystem, uint8_t& targetComponent) {
    // Check message type and extract target if present
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_INT:
        case MAVLINK_MSG_ID_COMMAND_LONG:
        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        case MAVLINK_MSG_ID_PARAM_SET:
            // These messages have target_system and target_component
            // Simplified extraction (would need proper decoding)
            targetSystem = ((uint8_t*)msg.payload64)[0];
            targetComponent = ((uint8_t*)msg.payload64)[1];
            return true;

        default:
            return false; // No specific target
    }
}

bool MAVLinkRouter::isForwardableMessage(const mavlink_message_t& msg) {
    // Don't forward certain message types
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
        case MAVLINK_MSG_ID_STATUSTEXT:
        case MAVLINK_MSG_ID_PARAM_VALUE:
            return false; // These are broadcast/local

        default:
            return true; // Forward by default
    }
}

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_GCS_ENABLED
