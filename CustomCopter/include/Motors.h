#pragma once

#include "DataTypes.h"
#include "HAL.h"
#include <cstdint>
#include <array>

namespace CustomCopter {

// ============================================================================
// Motor Frame Configuration
// ============================================================================
enum class FrameType {
    QUAD_X,      // X configuration (default)
    QUAD_PLUS,   // + configuration
    HEXA_X,      // Hexacopter X
    OCTA_X       // Octacopter X
};

// ============================================================================
// Motors Class
// Based on ArduPilot AP_MotorsMatrix and AP_Motors_Class
// Handles motor mixing, thrust linearization, and PWM output
// ============================================================================
class Motors {
public:
    Motors();
    virtual ~Motors() = default;

    // Initialize motors with HAL PWM interface
    bool init(PWM* pwm);

    // Set frame type (default: QUAD_X)
    void set_frame_type(FrameType type);

    // Get number of motors for current frame
    uint8_t get_num_motors() const { return num_motors_; }

    // ========================================================================
    // Motor Control Input (from attitude controller)
    // ========================================================================

    // Set individual axis commands (-1.0 to +1.0)
    void set_roll(float roll) { roll_input_ = constrain_float(roll, -1.0f, 1.0f); }
    void set_pitch(float pitch) { pitch_input_ = constrain_float(pitch, -1.0f, 1.0f); }
    void set_yaw(float yaw) { yaw_input_ = constrain_float(yaw, -1.0f, 1.0f); }

    // Set throttle (0.0 to 1.0)
    void set_throttle(float throttle) {
        throttle_input_ = constrain_float(throttle, 0.0f, 1.0f);
    }

    // Set all at once
    void set_rpy_throttle(float roll, float pitch, float yaw, float throttle);

    // ========================================================================
    // Arming and Safety
    // ========================================================================

    // Arm/disarm motors
    void arm() { armed_ = true; }
    void disarm() { armed_ = false; }
    bool is_armed() const { return armed_; }

    // Enable motor output (for pre-arm testing)
    void enable_output(bool enable) { output_enabled_ = enable; }

    // ========================================================================
    // Motor Mixing and Output
    // ========================================================================

    // Update motor outputs (call at loop rate - 400Hz)
    void output();

    // Get individual motor outputs (0.0 to 1.0)
    float get_motor_output(uint8_t motor_num) const;

    // Get motor PWM value (microseconds)
    uint16_t get_motor_pwm(uint8_t motor_num) const;

    // ========================================================================
    // PWM Configuration
    // ========================================================================

    // Set PWM output range (default: 1000-2000 us)
    void set_pwm_range(uint16_t min_us, uint16_t max_us);

    // Get PWM range
    uint16_t get_pwm_min() const { return pwm_min_; }
    uint16_t get_pwm_max() const { return pwm_max_; }

    // Set motor PWM frequency (default: 400Hz)
    void set_pwm_frequency(uint32_t freq_hz) { pwm_freq_hz_ = freq_hz; }

    // ========================================================================
    // Thrust Curve Configuration
    // ========================================================================

    // Set thrust curve expo (0.0 = linear, 0.65 = default ArduPilot)
    void set_thrust_expo(float expo) {
        thrust_expo_ = constrain_float(expo, 0.0f, 1.0f);
    }

    // Get thrust curve expo
    float get_thrust_expo() const { return thrust_expo_; }

    // Set hover throttle (0.0 to 1.0, default 0.5)
    void set_hover_throttle(float thr) {
        hover_throttle_ = constrain_float(thr, 0.0f, 1.0f);
    }

    // ========================================================================
    // Battery Compensation
    // ========================================================================

    // Set battery voltage for thrust compensation
    void set_battery_voltage(float voltage);

    // Set battery voltage limits
    void set_battery_voltage_limits(float min_v, float max_v);

    // Enable/disable battery compensation
    void enable_battery_compensation(bool enable) {
        battery_compensation_enabled_ = enable;
    }

    // ========================================================================
    // Status and Health
    // ========================================================================

    // Check if motors are healthy
    bool is_healthy() const { return healthy_; }

    // Get thrust loss status (true if at motor limits)
    bool is_thrust_loss() const { return thrust_loss_; }

    // Get throttle limit status
    bool is_throttle_limit() const { return throttle_limit_; }

private:
    // ========================================================================
    // Motor Mixing Matrix
    // ========================================================================

    struct MotorFactors {
        float roll;     // Roll factor
        float pitch;    // Pitch factor
        float yaw;      // Yaw factor
        float throttle; // Throttle factor (usually 1.0)
        uint8_t pwm_channel; // PWM output channel
    };

    // Initialize motor factors for frame type
    void init_quad_x();
    void init_quad_plus();
    void init_hexa_x();
    void init_octa_x();

    // Calculate motor mixing
    void calculate_motor_mix();

    // Apply thrust linearization curve
    float apply_thrust_curve(float throttle) const;

    // Compensate for battery voltage drop
    float compensate_battery_voltage(float throttle) const;

    // Constrain motor output and check limits
    void constrain_motor_outputs();

    // Convert throttle (0-1) to PWM (microseconds)
    uint16_t throttle_to_pwm(float throttle) const;

    // ========================================================================
    // Member Variables
    // ========================================================================

    // HAL interface
    PWM* pwm_;

    // Frame configuration
    FrameType frame_type_;
    uint8_t num_motors_;
    std::array<MotorFactors, 8> motor_factors_;  // Max 8 motors

    // Motor inputs (from attitude controller)
    float roll_input_;
    float pitch_input_;
    float yaw_input_;
    float throttle_input_;

    // Motor outputs (0.0 to 1.0, before PWM conversion)
    std::array<float, 8> motor_output_;

    // Motor PWM outputs (microseconds)
    std::array<uint16_t, 8> motor_pwm_;

    // PWM configuration
    uint16_t pwm_min_;      // Minimum PWM (default 1000 us)
    uint16_t pwm_max_;      // Maximum PWM (default 2000 us)
    uint32_t pwm_freq_hz_;  // PWM frequency (default 400 Hz)

    // Thrust curve parameters
    float thrust_expo_;      // Thrust curve expo (0.0-1.0, default 0.65)
    float hover_throttle_;   // Hover throttle (0.0-1.0, default 0.5)

    // Battery compensation
    float battery_voltage_;
    float battery_voltage_min_;
    float battery_voltage_max_;
    float battery_voltage_nominal_;  // Nominal voltage for compensation
    bool battery_compensation_enabled_;

    // Status flags
    bool armed_;
    bool output_enabled_;
    bool healthy_;
    bool thrust_loss_;       // True if motors are saturated
    bool throttle_limit_;    // True if throttle is limited

    // Motor limits (for detecting thrust loss)
    float motor_limit_lower_;  // Lower limit (default 0.0)
    float motor_limit_upper_;  // Upper limit (default 1.0)
};

} // namespace CustomCopter
