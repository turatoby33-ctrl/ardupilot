/*
 * system.cpp
 *
 * System-level functions for MiniVehicle
 * Includes parameter definitions and system utilities
 */

#include "MiniVehicle.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

// ========================================
// PARAMETER TABLE
// ========================================

/*
 * Parameters allow configuration via GCS
 * This is a minimal set - real vehicles have hundreds
 *
 * Parameters can be:
 * - Read/set via MAVLink (PARAM_REQUEST_LIST, PARAM_SET, etc.)
 * - Stored in EEPROM
 * - Modified via Mission Planner/QGC
 */

const AP_Param::GroupInfo MiniVehicle::var_info[] = {
    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink System ID
    // @Description: System ID of this vehicle
    // @Range: 1 250
    // @User: Standard
    // AP_GROUPINFO("SYSID_THISMAV", 0, MiniVehicle, sysid_this_mav, 1),

    // In a real vehicle, you would define parameters here:
    // AP_GROUPINFO("PARAM_NAME", index, ClassName, variable, default_value),

    // Examples:
    // AP_GROUPINFO("CRUISE_SPEED", 1, MiniVehicle, cruise_speed, 5.0f),
    // AP_GROUPINFO("MAX_THROTTLE", 2, MiniVehicle, max_throttle, 100),

    AP_GROUPEND
};

// ========================================
// SYSTEM INFO
// ========================================

/*
 * System identification strings
 * Used by GCS to identify vehicle type and version
 */

const char* MiniVehicle::get_vehicle_name() const
{
    return "MiniVehicle";
}

uint32_t MiniVehicle::get_vehicle_version() const
{
    // Version format: major.minor.patch
    // Encoded as: (major << 24) | (minor << 16) | (patch << 8)
    return (1 << 24) | (0 << 16) | (0 << 8);  // Version 1.0.0
}

// ========================================
// SYSTEM STATUS
// ========================================

/*
 * Functions to report system status to GCS
 * Called by GCS infrastructure to populate telemetry
 */

bool MiniVehicle::is_armed() const
{
    return ap.armed;
}

bool MiniVehicle::in_failsafe() const
{
    return ap.failsafe_crash_check;
}

// ========================================
// ARM/DISARM
// ========================================

/*
 * Arm/disarm functions
 * In a real vehicle, these would:
 * - Check pre-arm conditions
 * - Run pre-arm checks
 * - Enable/disable motors
 * - Log events
 */

bool MiniVehicle::arm(bool force)
{
    // Check if already armed
    if (ap.armed) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Already armed");
        return false;
    }

    // Pre-arm checks (only if not forced)
    if (!force) {
        if (!ap.initialised) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cannot arm: not initialized");
            return false;
        }

        if (ap.failsafe_crash_check) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cannot arm: in failsafe");
            return false;
        }

        // In a real vehicle, check:
        // - GPS lock
        // - Compass calibrated
        // - Gyros calibrated
        // - Battery voltage OK
        // - Geofence loaded
        // - etc.
    }

    // Arm!
    ap.armed = true;

    hal.console->printf("*** ARMED ***\n");
    gcs().send_text(MAV_SEVERITY_INFO, "Armed");

    return true;
}

bool MiniVehicle::disarm(bool force)
{
    // Check if already disarmed
    if (!ap.armed) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Already disarmed");
        return false;
    }

    // Safety checks (only if not forced)
    if (!force) {
        // In a real vehicle, check:
        // - Vehicle is not moving
        // - Not in the air
        // - etc.
    }

    // Disarm!
    ap.armed = false;

    hal.console->printf("*** DISARMED ***\n");
    gcs().send_text(MAV_SEVERITY_INFO, "Disarmed");

    return true;
}

// ========================================
// MODE MANAGEMENT
// ========================================

/*
 * Mode management functions
 * Handle mode changes requested by GCS or pilot
 */

bool MiniVehicle::set_mode(Mode mode, ModeReason reason)
{
    // Check if mode is valid
    if ((uint8_t)mode >= (uint8_t)Mode::NUM_MODES) {
        return false;
    }

    // Check if mode change is allowed
    // In a real vehicle, some modes require:
    // - GPS lock
    // - Home position set
    // - Mission loaded
    // - etc.

    Mode old_mode = control_mode;

    // Make the change
    control_mode = mode;

    // Mode change logging
    const char* mode_names[] = {"MANUAL", "AUTO", "GUIDED", "HOLD"};
    const char* reason_names[] = {"UNKNOWN", "GCS_COMMAND", "RC_COMMAND", "FAILSAFE", "AUTO"};

    hal.console->printf("Mode: %s -> %s (reason: %s)\n",
                       mode_names[(uint8_t)old_mode],
                       mode_names[(uint8_t)mode],
                       reason_names[(uint8_t)reason]);

    gcs().send_text(MAV_SEVERITY_INFO, "Mode: %s", mode_names[(uint8_t)mode]);

    return true;
}

MiniVehicle::Mode MiniVehicle::get_mode() const
{
    return control_mode;
}

// ========================================
// UTILITY FUNCTIONS
// ========================================

/*
 * Helper functions for system management
 */

void MiniVehicle::reboot(bool hold_in_bootloader)
{
    hal.console->printf("\n\n*** REBOOTING ***\n\n");
    gcs().send_text(MAV_SEVERITY_WARNING, "Rebooting");

    // Give time for message to be sent
    hal.scheduler->delay(100);

    // Reboot
    hal.scheduler->reboot(hold_in_bootloader);
}

void MiniVehicle::print_status()
{
    hal.console->printf("\n");
    hal.console->printf("========================================\n");
    hal.console->printf("   MiniVehicle Status\n");
    hal.console->printf("========================================\n");
    hal.console->printf("Initialized:  %s\n", ap.initialised ? "YES" : "NO");
    hal.console->printf("Armed:        %s\n", ap.armed ? "YES" : "NO");
    hal.console->printf("Failsafe:     %s\n", ap.failsafe_crash_check ? "YES" : "NO");

    const char* mode_names[] = {"MANUAL", "AUTO", "GUIDED", "HOLD"};
    hal.console->printf("Mode:         %s\n", mode_names[(uint8_t)control_mode]);

    hal.console->printf("========================================\n");
    hal.console->printf("\n");
}
