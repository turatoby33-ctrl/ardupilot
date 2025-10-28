/**
 * @file MissionItemProtocol_Fence.h
 * @brief EduCopter Geofence Protocol
 *
 * Handles upload/download of geofence points.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "MissionItemProtocol.h"

#if EDUCOPTER_FENCE_ENABLED

namespace EduCopter {
namespace GCS {

/**
 * @class MissionItemProtocol_Fence
 * @brief Geofence transfer protocol
 */
class MissionItemProtocol_Fence : public MissionItemProtocol {
public:
    /**
     * @brief Constructor
     */
    MissionItemProtocol_Fence();

    /**
     * @brief Destructor
     */
    ~MissionItemProtocol_Fence() override = default;

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

#endif // EDUCOPTER_FENCE_ENABLED
