/*
 * GCS_MiniVehicle.h
 *
 * GCS Manager class for MiniVehicle
 * Inherits from base GCS class and provides vehicle-specific functionality
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_MiniVehicle.h"

class GCS_MiniVehicle : public GCS
{
    friend class MiniVehicle;  // Allow MiniVehicle to access protected members

public:
    // Required macro: Generates chan() methods that return GCS_MAVLINK_MiniVehicle*
    // instead of base GCS_MAVLINK*
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_MiniVehicle);

    // Required: Return current vehicle mode as uint32_t
    uint32_t custom_mode() const override;

    // Required: Return MAVLink vehicle type
    MAV_TYPE frame_type() const override;

    // Optional: Return vehicle frame description
    const char* frame_string() const override { return "MiniVehicle"; }

    // Optional: Check if vehicle is initialized
    bool vehicle_initialised() const override;

    // Optional: Update vehicle-specific sensor status flags
    void update_vehicle_sensor_status_flags(void) override;

protected:
    // Required: Minimum time that must remain in scheduler loop before
    // we're allowed to send MAVLink messages (microseconds)
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 200;  // 200 microseconds
    }

    // Required: Factory method to create channel objects
    GCS_MAVLINK_MiniVehicle *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_MiniVehicle(uart);
    }
};
