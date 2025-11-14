/*
 * GCS_MiniVehicle.cpp
 *
 * Implementation of GCS manager for MiniVehicle
 */

#include "GCS_MiniVehicle.h"
#include "MiniVehicle.h"

// Return the current vehicle mode
uint32_t GCS_MiniVehicle::custom_mode() const
{
    // Cast vehicle mode enum to uint32_t
    // This appears in HEARTBEAT message and GCS displays it
    return (uint32_t)minivehicle.control_mode;
}

// Return the MAVLink vehicle type
MAV_TYPE GCS_MiniVehicle::frame_type() const
{
    // Tell the GCS what type of vehicle we are
    // For a simple test vehicle, we'll use GROUND_ROVER
    // Other options: MAV_TYPE_QUADROTOR, MAV_TYPE_HELICOPTER, etc.
    return MAV_TYPE_GROUND_ROVER;
}

// Check if vehicle has completed initialization
bool GCS_MiniVehicle::vehicle_initialised() const
{
    return minivehicle.ap.initialised;
}

// Update vehicle-specific sensor status flags
void GCS_MiniVehicle::update_vehicle_sensor_status_flags(void)
{
    /*
     * Update sensor status bitmasks:
     * - control_sensors_present: What sensors exist
     * - control_sensors_enabled: What sensors are turned on
     * - control_sensors_health: What sensors are working
     *
     * These appear in SYS_STATUS message and GCS displays them
     */

    // Motors/Actuators - always present on MiniVehicle
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    // Motors enabled if not in failsafe
    if (!minivehicle.ap.failsafe_crash_check) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // Motors healthy if responding
    control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    // Add more sensors as needed:
    // - MAV_SYS_STATUS_SENSOR_3D_GYRO
    // - MAV_SYS_STATUS_SENSOR_3D_ACCEL
    // - MAV_SYS_STATUS_SENSOR_3D_MAG
    // - MAV_SYS_STATUS_SENSOR_GPS
    // - MAV_SYS_STATUS_SENSOR_LASER_POSITION (rangefinder)
    // etc.
}
