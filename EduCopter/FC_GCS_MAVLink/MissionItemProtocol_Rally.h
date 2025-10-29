/**
 * @file MissionItemProtocol_Rally.h
 * @brief EduCopter Rally Point Protocol
 *
 * Handles upload/download of rally (safe return) points.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include "MissionItemProtocol.h"

#if EDUCOPTER_RALLY_ENABLED

namespace EduCopter {
namespace GCS {

/**
 * @class MissionItemProtocol_Rally
 * @brief Rally point transfer protocol
 */
class MissionItemProtocol_Rally : public MissionItemProtocol {
public:
    /**
     * @brief Constructor
     * @param channel GCS channel for communication
     */
    explicit MissionItemProtocol_Rally(GCSChannel& channel);

    /**
     * @brief Destructor
     */
    ~MissionItemProtocol_Rally() override = default;

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

#endif // EDUCOPTER_RALLY_ENABLED
