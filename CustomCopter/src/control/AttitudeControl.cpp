#include "AttitudeControl.h"
#include "Motors.h"
#include <cmath>
#include <algorithm>

namespace CustomCopter {

AttitudeControl::AttitudeControl()
    : motors_(nullptr)
    , angle_p_roll_(4.5f)
    , angle_p_pitch_(4.5f)
    , angle_p_yaw_(4.5f)
    , angle_target_roll_(0.0f)
    , angle_target_pitch_(0.0f)
    , angle_target_yaw_(0.0f)
    , rate_target_roll_(0.0f)
    , rate_target_pitch_(0.0f)
    , rate_target_yaw_(0.0f)
    , throttle_out_(0.0f)
    , throttle_rpy_mix_(0.5f)
    , throttle_rpy_mix_min_(0.1f)
    , throttle_rpy_mix_man_(0.5f)
    , throttle_rpy_mix_max_(0.9f)
    , dt_(0.0025f)  // 400 Hz default
    , angle_limit_rad_(radians(45.0f))
    , rate_limit_roll_pitch_(radians(360.0f))  // 360 deg/s
    , rate_limit_yaw_(radians(180.0f))         // 180 deg/s
    , initialized_(false)
{
    // Initialize rate PIDs with ArduPilot defaults for multicopter
    PID::Gains roll_pitch_gains;
    roll_pitch_gains.kP = 0.135f;
    roll_pitch_gains.kI = 0.135f;
    roll_pitch_gains.kD = 0.0036f;
    roll_pitch_gains.kFF = 0.0f;
    roll_pitch_gains.imax = 0.5f;
    roll_pitch_gains.filt_T_hz = 20.0f;
    roll_pitch_gains.filt_E_hz = 0.0f;
    roll_pitch_gains.filt_D_hz = 20.0f;

    pid_rate_roll_.set_gains(roll_pitch_gains);
    pid_rate_pitch_.set_gains(roll_pitch_gains);

    // Yaw has different gains
    PID::Gains yaw_gains;
    yaw_gains.kP = 0.180f;
    yaw_gains.kI = 0.018f;
    yaw_gains.kD = 0.0f;
    yaw_gains.kFF = 0.0f;
    yaw_gains.imax = 0.5f;
    yaw_gains.filt_T_hz = 20.0f;
    yaw_gains.filt_E_hz = 2.5f;  // Lower for yaw
    yaw_gains.filt_D_hz = 20.0f;

    pid_rate_yaw_.set_gains(yaw_gains);
}

void AttitudeControl::init() {
    initialized_ = true;
}

void AttitudeControl::set_dt(float dt) {
    dt_ = dt;
}

void AttitudeControl::input_euler_angle_roll_pitch_yaw(float roll_rad,
                                                        float pitch_rad,
                                                        float yaw_rad,
                                                        bool slew_yaw) {
    // Constrain angle commands to limits
    angle_target_roll_ = constrain_angle(roll_rad);
    angle_target_pitch_ = constrain_angle(pitch_rad);
    angle_target_yaw_ = wrap_PI(yaw_rad);

    // Convert angle targets to rate targets (P controller)
    float roll_error = wrap_PI(angle_target_roll_ - attitude_.roll);
    float pitch_error = wrap_PI(angle_target_pitch_ - attitude_.pitch);
    float yaw_error = wrap_PI(angle_target_yaw_ - attitude_.yaw);

    rate_target_roll_ = angle_to_rate(roll_error, angle_p_roll_);
    rate_target_pitch_ = angle_to_rate(pitch_error, angle_p_pitch_);
    rate_target_yaw_ = angle_to_rate(yaw_error, angle_p_yaw_);

    // Apply rate limits
    rate_target_roll_ = constrain_float(rate_target_roll_,
                                        -rate_limit_roll_pitch_,
                                        rate_limit_roll_pitch_);
    rate_target_pitch_ = constrain_float(rate_target_pitch_,
                                         -rate_limit_roll_pitch_,
                                         rate_limit_roll_pitch_);
    rate_target_yaw_ = constrain_float(rate_target_yaw_,
                                       -rate_limit_yaw_,
                                       rate_limit_yaw_);
}

void AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(
    float roll_rad, float pitch_rad, float yaw_rate_rads) {

    // Constrain angle commands to limits
    angle_target_roll_ = constrain_angle(roll_rad);
    angle_target_pitch_ = constrain_angle(pitch_rad);

    // Convert angle targets to rate targets (P controller)
    float roll_error = wrap_PI(angle_target_roll_ - attitude_.roll);
    float pitch_error = wrap_PI(angle_target_pitch_ - attitude_.pitch);

    rate_target_roll_ = angle_to_rate(roll_error, angle_p_roll_);
    rate_target_pitch_ = angle_to_rate(pitch_error, angle_p_pitch_);

    // Yaw is a direct rate command
    rate_target_yaw_ = yaw_rate_rads;

    // Apply rate limits
    rate_target_roll_ = constrain_float(rate_target_roll_,
                                        -rate_limit_roll_pitch_,
                                        rate_limit_roll_pitch_);
    rate_target_pitch_ = constrain_float(rate_target_pitch_,
                                         -rate_limit_roll_pitch_,
                                         rate_limit_roll_pitch_);
    rate_target_yaw_ = constrain_float(rate_target_yaw_,
                                       -rate_limit_yaw_,
                                       rate_limit_yaw_);
}

void AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_rads,
                                                    float pitch_rate_rads,
                                                    float yaw_rate_rads) {
    // Direct rate commands (for ACRO mode)
    rate_target_roll_ = constrain_float(roll_rate_rads,
                                        -rate_limit_roll_pitch_,
                                        rate_limit_roll_pitch_);
    rate_target_pitch_ = constrain_float(pitch_rate_rads,
                                         -rate_limit_roll_pitch_,
                                         rate_limit_roll_pitch_);
    rate_target_yaw_ = constrain_float(yaw_rate_rads,
                                       -rate_limit_yaw_,
                                       rate_limit_yaw_);
}

void AttitudeControl::rate_controller_run() {
    if (!motors_) {
        return;  // No motors connected
    }

    // Run PID rate controllers for each axis
    // Input: rate target and current rate (from gyro)
    // Output: motor command (-1 to +1)

    float roll_out = pid_rate_roll_.update_with_derivative(
        rate_target_roll_,
        gyro_rates_.x,
        -gyro_rates_.x,  // Derivative is negative of rate
        dt_
    );

    float pitch_out = pid_rate_pitch_.update_with_derivative(
        rate_target_pitch_,
        gyro_rates_.y,
        -gyro_rates_.y,
        dt_
    );

    float yaw_out = pid_rate_yaw_.update_with_derivative(
        rate_target_yaw_,
        gyro_rates_.z,
        -gyro_rates_.z,
        dt_
    );

    // Constrain outputs to reasonable range
    roll_out = constrain_float(roll_out, -1.0f, 1.0f);
    pitch_out = constrain_float(pitch_out, -1.0f, 1.0f);
    yaw_out = constrain_float(yaw_out, -1.0f, 1.0f);

    // Send commands to motors
    motors_->set_roll(roll_out);
    motors_->set_pitch(pitch_out);
    motors_->set_yaw(yaw_out);
    motors_->set_throttle(throttle_out_);
}

void AttitudeControl::set_throttle_out(float throttle) {
    throttle_out_ = constrain_float(throttle, 0.0f, 1.0f);
}

void AttitudeControl::reset_rate_controller_I() {
    pid_rate_roll_.reset_I();
    pid_rate_pitch_.reset_I();
    pid_rate_yaw_.reset_I();
}

float AttitudeControl::angle_to_rate(float angle_error, float angle_p) {
    // Simple P controller to convert angle error to rate target
    return angle_error * angle_p;
}

float AttitudeControl::constrain_angle(float angle) {
    return constrain_float(angle, -angle_limit_rad_, angle_limit_rad_);
}

} // namespace CustomCopter
