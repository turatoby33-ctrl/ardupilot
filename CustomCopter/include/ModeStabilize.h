#pragma once

#include "Mode.h"

namespace CustomCopter {

// ============================================================================
// STABILIZE Mode
// Manual angle control with self-leveling
//
// This is the most basic flight mode and the recommended mode for beginners.
// The pilot's roll and pitch stick inputs control the lean angle of the copter.
// When the sticks are released, the copter will self-level.
//
// Control:
// - Roll stick: Controls roll angle (left/right tilt)
// - Pitch stick: Controls pitch angle (forward/back tilt)
// - Yaw stick: Controls yaw rotation rate
// - Throttle stick: Controls throttle directly (no altitude hold)
//
// Based on ArduCopter Copter::ModeStabilize
// ============================================================================
class ModeStabilize : public Mode {
public:
    ModeStabilize();
    ~ModeStabilize() override = default;

    // ========================================================================
    // Mode Identification
    // ========================================================================

    FlightMode mode_number() const override {
        return FlightMode::STABILIZE;
    }

    const char* name() const override {
        return "STABILIZE";
    }

    // ========================================================================
    // Mode Capabilities
    // ========================================================================

    bool requires_GPS() const override {
        return false;  // STABILIZE doesn't need GPS
    }

    bool has_manual_throttle() const override {
        return true;  // Pilot controls throttle directly
    }

    bool requires_altitude_hold() const override {
        return false;  // No altitude hold in STABILIZE
    }

    bool is_auto_mode() const override {
        return false;  // Manual mode
    }

    bool allows_arming(bool from_gcs) const override {
        (void)from_gcs;
        return true;  // Can arm in STABILIZE
    }

    // ========================================================================
    // Mode Initialization and Entry/Exit
    // ========================================================================

    bool init(bool ignore_checks) override;
    bool enter() override;
    void exit() override;

    // ========================================================================
    // Mode Execution
    // ========================================================================

    // Main run function (called at 400 Hz)
    void run() override;

private:
    // ========================================================================
    // Helper Functions
    // ========================================================================

    // Get pilot's desired angles and rates
    void get_pilot_desired_lean_angles(float& roll_out, float& pitch_out);

    // Run attitude controller
    void run_attitude_controller();

    // ========================================================================
    // Member Variables
    // ========================================================================

    // Target angles (radians)
    float target_roll_;
    float target_pitch_;
    float target_yaw_rate_;

    // Throttle
    float throttle_out_;
};

} // namespace CustomCopter
