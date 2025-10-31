#pragma once

#include "DataTypes.h"
#include "PID.h"
#include "IMU.h"

namespace CustomCopter {

// Forward declarations
class Motors;

// ============================================================================
// Attitude Control
// Based on ArduPilot's AC_AttitudeControl_Multi
// Implements 3-axis attitude stabilization with cascaded PID control
// ============================================================================
class AttitudeControl {
public:
    AttitudeControl();
    ~AttitudeControl() = default;

    // Initialize attitude controller
    void init();

    // Set the loop update rate
    void set_dt(float dt);

    // Set motor output object
    void set_motors(Motors* motors) { motors_ = motors; }

    // ========================================================================
    // Angle Control (Outer Loop)
    // ========================================================================

    // Input desired Euler angles (radians) and calculate rate targets
    // roll_rad, pitch_rad: Desired roll and pitch angles
    // yaw_rad: Desired yaw angle
    // returns: true if successful
    void input_euler_angle_roll_pitch_yaw(float roll_rad, float pitch_rad,
                                          float yaw_rad, bool slew_yaw);

    // Input desired Euler angles for roll/pitch with yaw rate
    void input_euler_angle_roll_pitch_euler_rate_yaw(float roll_rad,
                                                      float pitch_rad,
                                                      float yaw_rate_rads);

    // ========================================================================
    // Rate Control (Inner Loop)
    // ========================================================================

    // Run the rate controller (call at 400 Hz)
    // Reads gyro, computes PID, outputs motor commands
    void rate_controller_run();

    // Set body-frame rate targets directly (for ACRO mode)
    void input_rate_bf_roll_pitch_yaw(float roll_rate_rads,
                                      float pitch_rate_rads,
                                      float yaw_rate_rads);

    // ========================================================================
    // PID Configuration
    // ========================================================================

    // Get/set rate PID gains
    void set_rate_roll_gains(const PID::Gains& gains) { pid_rate_roll_.set_gains(gains); }
    void set_rate_pitch_gains(const PID::Gains& gains) { pid_rate_pitch_.set_gains(gains); }
    void set_rate_yaw_gains(const PID::Gains& gains) { pid_rate_yaw_.set_gains(gains); }

    const PID::Gains& get_rate_roll_gains() const { return pid_rate_roll_.get_gains(); }
    const PID::Gains& get_rate_pitch_gains() const { return pid_rate_pitch_.get_gains(); }
    const PID::Gains& get_rate_yaw_gains() const { return pid_rate_yaw_.get_gains(); }

    // Set angle P gains
    void set_angle_roll_p(float p) { angle_p_roll_ = p; }
    void set_angle_pitch_p(float p) { angle_p_pitch_ = p; }
    void set_angle_yaw_p(float p) { angle_p_yaw_ = p; }

    float get_angle_roll_p() const { return angle_p_roll_; }
    float get_angle_pitch_p() const { return angle_p_pitch_; }
    float get_angle_yaw_p() const { return angle_p_yaw_; }

    // ========================================================================
    // Throttle/Attitude Mix
    // ========================================================================

    // Set throttle mix (0 = full attitude control, 1 = full throttle priority)
    void set_throttle_mix_min() { throttle_rpy_mix_ = throttle_rpy_mix_min_; }
    void set_throttle_mix_man() { throttle_rpy_mix_ = throttle_rpy_mix_man_; }
    void set_throttle_mix_max() { throttle_rpy_mix_ = throttle_rpy_mix_max_; }

    // Set throttle output (0.0 to 1.0)
    void set_throttle_out(float throttle);

    // Get throttle output
    float get_throttle_out() const { return throttle_out_; }

    // ========================================================================
    // State Access
    // ========================================================================

    // Get current attitude estimate
    const Attitude& get_attitude() const { return attitude_; }

    // Get rate targets
    Vector3f get_rate_targets() const {
        return Vector3f(rate_target_roll_, rate_target_pitch_, rate_target_yaw_);
    }

    // Get angle targets
    Vector3f get_angle_targets() const {
        return Vector3f(angle_target_roll_, angle_target_pitch_, angle_target_yaw_);
    }

    // Set current attitude (from AHRS/EKF)
    void set_attitude(const Attitude& attitude) { attitude_ = attitude; }

    // Set current gyro rates
    void set_gyro_rates(const Vector3f& rates) { gyro_rates_ = rates; }

    // Reset integrators (call when disarming or changing modes)
    void reset_rate_controller_I();

private:
    // Convert angle error to rate target
    float angle_to_rate(float angle_error, float angle_p);

    // Apply angle limits
    float constrain_angle(float angle);

    // Motors interface
    Motors* motors_;

    // PID controllers (rate control - inner loop)
    PID pid_rate_roll_;
    PID pid_rate_pitch_;
    PID pid_rate_yaw_;

    // Angle P gains (outer loop)
    float angle_p_roll_;
    float angle_p_pitch_;
    float angle_p_yaw_;

    // State
    Attitude attitude_;           // Current attitude estimate
    Vector3f gyro_rates_;        // Current gyro rates (rad/s)

    // Targets (angle control)
    float angle_target_roll_;
    float angle_target_pitch_;
    float angle_target_yaw_;

    // Targets (rate control)
    float rate_target_roll_;
    float rate_target_pitch_;
    float rate_target_yaw_;

    // Throttle
    float throttle_out_;

    // Throttle-attitude mix parameters
    float throttle_rpy_mix_;      // Current mix value
    float throttle_rpy_mix_min_;  // Landing (prioritize throttle)
    float throttle_rpy_mix_man_;  // Manual flight
    float throttle_rpy_mix_max_;  // Auto flight (prioritize attitude)

    // Timing
    float dt_;

    // Limits
    float angle_limit_rad_;       // Maximum angle command
    float rate_limit_roll_pitch_; // Maximum roll/pitch rate
    float rate_limit_yaw_;        // Maximum yaw rate

    bool initialized_;
};

} // namespace CustomCopter
