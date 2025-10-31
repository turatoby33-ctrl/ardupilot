#pragma once

#include "DataTypes.h"
#include "AttitudeControl.h"
#include "Motors.h"
#include <cstdint>
#include <string>

namespace CustomCopter {

// Forward declarations
class Copter;

// ============================================================================
// Flight Mode Numbers (matching ArduCopter)
// ============================================================================
enum class FlightMode : uint8_t {
    STABILIZE    = 0,   // Manual angle control with self-leveling
    ACRO         = 1,   // Manual rate control (no self-leveling)
    ALT_HOLD     = 2,   // STABILIZE + altitude hold
    AUTO         = 3,   // Autonomous waypoint navigation
    GUIDED       = 4,   // External position/velocity control
    LOITER       = 5,   // Position hold with GPS
    RTL          = 6,   // Return to launch
    CIRCLE       = 7,   // Circle around a point
    LAND         = 9,   // Autonomous landing
    DRIFT        = 11,  // Drift mode (like a flying car)
    SPORT        = 13,  // High-rate manual control
    FLIP         = 14,  // Automated flip
    AUTOTUNE     = 15,  // Automatic PID tuning
    POS_HOLD     = 16,  // Position hold (deprecated, use LOITER)
    BRAKE        = 17,  // Rapid stop
    THROW        = 18,  // Throw to launch
    AVOID_ADSB   = 19,  // Avoidance of manned aircraft
    GUIDED_NOGPS = 20,  // GUIDED without GPS
    SMART_RTL    = 21,  // Return via path traveled
    FLOWHOLD     = 22,  // Optical flow position hold
    FOLLOW       = 23,  // Follow another vehicle
    ZIGZAG       = 24,  // ZigZag auto mode
    SYSTEMID     = 25,  // System identification
    AUTOROTATE   = 26,  // Helicopter autorotation
    AUTO_RTL     = 27,  // Auto RTL mode
    TURTLE       = 28,  // Turtle mode (flip over)
};

// ============================================================================
// Mode Base Class
// Based on ArduCopter Mode class
// ============================================================================
class Mode {
public:
    // Constructor
    Mode();
    virtual ~Mode() = default;

    // ========================================================================
    // Mode Identification
    // ========================================================================

    // Get mode number
    virtual FlightMode mode_number() const = 0;

    // Get mode name
    virtual const char* name() const = 0;

    // Get mode name string (static helper)
    static const char* mode_string(FlightMode mode);

    // ========================================================================
    // Mode Initialization and Entry/Exit
    // ========================================================================

    // Initialize mode (called once at startup)
    virtual bool init(bool ignore_checks);

    // Enter mode (called when switching to this mode)
    virtual bool enter();

    // Exit mode (called when switching away from this mode)
    virtual void exit();

    // ========================================================================
    // Mode Execution
    // ========================================================================

    // Run mode (called at main loop rate - 400Hz)
    virtual void run() = 0;

    // ========================================================================
    // Mode Capabilities
    // ========================================================================

    // Does this mode require GPS?
    virtual bool requires_GPS() const { return false; }

    // Does this mode have manual throttle control?
    virtual bool has_manual_throttle() const { return false; }

    // Does this mode require altitude hold?
    virtual bool requires_altitude_hold() const { return false; }

    // Is this mode an auto mode (autonomous)?
    virtual bool is_auto_mode() const { return false; }

    // Can this mode be armed?
    virtual bool allows_arming(bool from_gcs) const { return true; }

    // ========================================================================
    // RC Input Processing
    // ========================================================================

    // Get pilot's desired roll angle (radians)
    float get_pilot_desired_roll() const;

    // Get pilot's desired pitch angle (radians)
    float get_pilot_desired_pitch() const;

    // Get pilot's desired yaw rate (rad/s)
    float get_pilot_desired_yaw_rate() const;

    // Get pilot's throttle input (0.0 to 1.0)
    float get_pilot_throttle() const;

    // Check if pilot wants to take off (throttle above threshold)
    bool get_pilot_wants_takeoff() const;

    // Check if pilot wants to land (throttle below threshold)
    bool get_pilot_wants_land() const;

    // ========================================================================
    // Mode State
    // ========================================================================

    // Check if mode is landed
    bool is_landed() const { return landed_; }

    // Check if mode is taking off
    bool is_taking_off() const { return taking_off_; }

    // Check if mode is landing
    bool is_landing() const { return landing_; }

    // ========================================================================
    // Access to Copter Systems
    // ========================================================================

    // Set copter reference (called by Copter class)
    void set_copter(Copter* copter) { copter_ = copter; }

protected:
    // ========================================================================
    // Helper Functions for Derived Modes
    // ========================================================================

    // Get attitude controller
    AttitudeControl* get_attitude_control();

    // Get motors
    Motors* get_motors();

    // Get current attitude
    const Attitude& get_attitude() const;

    // Get gyro rates
    const Vector3f& get_gyro_rates() const;

    // Get throttle hover value (for altitude hold modes)
    float get_throttle_hover() const { return 0.5f; }

    // Constrain angle to maximum lean angle
    float constrain_lean_angle(float angle_rad) const;

    // Get maximum lean angle (radians)
    float get_angle_max() const { return angle_max_rad_; }

    // Get RC input scaled to range
    float get_rc_input(uint8_t channel, float min, float max) const;

    // ========================================================================
    // Member Variables
    // ========================================================================

    Copter* copter_;  // Reference to main copter object

    // Mode state
    bool landed_;
    bool taking_off_;
    bool landing_;

    // Angle limits (radians)
    float angle_max_rad_;  // Maximum lean angle (default 45 degrees)

    // RC input scaling
    float roll_sensitivity_;   // Roll stick sensitivity (default 1.0)
    float pitch_sensitivity_;  // Pitch stick sensitivity (default 1.0)
    float yaw_sensitivity_;    // Yaw stick sensitivity (default 1.0)

    // Throttle parameters
    float throttle_takeoff_;   // Throttle for takeoff (0.0-1.0)
    float throttle_land_;      // Throttle for landing (0.0-1.0)
};

} // namespace CustomCopter
