/**
 * @file MissionItemProtocol_Waypoints.h
 * @brief EduCopter Waypoint Mission Protocol
 *
 * Handles upload/download of waypoint missions.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "MissionItemProtocol.h"

#if EDUCOPTER_MISSION_ENABLED

namespace EduCopter {
namespace GCS {

/**
 * @class MissionItemProtocol_Waypoints
 * @brief Waypoint mission transfer protocol
 */
class MissionItemProtocol_Waypoints : public MissionItemProtocol {
public:
    /**
     * @brief Constructor
     */
    MissionItemProtocol_Waypoints();

    /**
     * @brief Destructor
     */
    ~MissionItemProtocol_Waypoints() override = default;

protected:
    // Override base class methods
    uint16_t getItemCount() const override;
    uint16_t getMaxItems() const override;
    MAV_MISSION_RESULT getItem(uint16_t index, mavlink_mission_item_int_t& item) override;
    MAV_MISSION_RESULT appendItem(const mavlink_mission_item_int_t& item) override;
    MAV_MISSION_RESULT replaceItem(uint16_t index, const mavlink_mission_item_int_t& item) override;
    bool clearAllItems() override;
    bool truncate(uint16_t count) override;
    MAV_MISSION_RESULT onComplete(GCSChannel* channel) override;
};

} // namespace GCS
} // namespace EduCopter

#endif // EDUCOPTER_MISSION_ENABLED
