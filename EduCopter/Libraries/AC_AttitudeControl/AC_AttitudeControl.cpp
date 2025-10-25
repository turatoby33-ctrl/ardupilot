#include "AC_AttitudeControl.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_HAL/AP_HAL.h"
#include <cmath>
#include <cstdio>

extern AP_HAL* hal;

AC_AttitudeControl::AC_AttitudeControl(AP_AHRS& ahrs) :
    _ahrs(ahrs),
    // Rate PIDs (these are typical starting values for a 450-size quad)
    _pid_rate_roll(0.15f, 0.1f, 0.004f, 0.5f, 20.0f),
    _pid_rate_pitch(0.15f, 0.1f, 0.004f, 0.5f, 20.0f),
    _pid_rate_yaw(0.2f, 0.02f, 0.0f, 0.5f, 20.0f),
    // Angle P gains
    _p_angle_roll(4.5f),
    _p_angle_pitch(4.5f),
    _p_angle_yaw(4.5f),
    // Limits
    _roll_rate_max(4.5f),  // ~250 deg/s
    _pitch_rate_max(4.5f),
    _yaw_rate_max(2.0f),   // ~115 deg/s
    _roll_angle_max(0.785f),  // 45 degrees
    _pitch_angle_max(0.785f),
    // Targets and outputs
    _target_roll(0.0f),
    _target_pitch(0.0f),
    _target_yaw(0.0f),
    _target_roll_rate(0.0f),
    _target_pitch_rate(0.0f),
    _target_yaw_rate(0.0f),
    _roll_out(0.0f),
    _pitch_out(0.0f),
    _yaw_out(0.0f),
    _throttle_out(0.0f),
    _last_update_us(0)
{
}

void AC_AttitudeControl::init() {
    printf("AC_AttitudeControl: Initializing attitude controller\n");
    printf("AC_AttitudeControl: Rate PIDs - Roll P:%.3f I:%.3f D:%.3f\n",
           _pid_rate_roll.get_kp(), _pid_rate_roll.get_ki(), _pid_rate_roll.get_kd());
    printf("AC_AttitudeControl: Rate PIDs - Pitch P:%.3f I:%.3f D:%.3f\n",
           _pid_rate_pitch.get_kp(), _pid_rate_pitch.get_ki(), _pid_rate_pitch.get_kd());
    printf("AC_AttitudeControl: Rate PIDs - Yaw P:%.3f I:%.3f D:%.3f\n",
           _pid_rate_yaw.get_kp(), _pid_rate_yaw.get_ki(), _pid_rate_yaw.get_kd());

    _last_update_us = hal->micros64();
}

void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float roll_angle_cd, float pitch_angle_cd, float yaw_angle_cd, bool slew_yaw) {
    // Convert from centidegrees to radians
    _target_roll = roll_angle_cd * 0.01f * DEG_TO_RAD;
    _target_pitch = pitch_angle_cd * 0.01f * DEG_TO_RAD;
    _target_yaw = yaw_angle_cd * 0.01f * DEG_TO_RAD;

    // Constrain angles
    _target_roll = AP_Math::constrain(_target_roll, -_roll_angle_max, _roll_angle_max);
    _target_pitch = AP_Math::constrain(_target_pitch, -_pitch_angle_max, _pitch_angle_max);

    // Calculate target rates (P-only angle controller)
    float roll_error = AP_Math::wrap_PI(_target_roll - _ahrs.get_roll());
    float pitch_error = AP_Math::wrap_PI(_target_pitch - _ahrs.get_pitch());
    float yaw_error = AP_Math::wrap_PI(_target_yaw - _ahrs.get_yaw());

    _target_roll_rate = roll_error * _p_angle_roll;
    _target_pitch_rate = pitch_error * _p_angle_pitch;
    _target_yaw_rate = yaw_error * _p_angle_yaw;

    // Constrain rates
    _target_roll_rate = AP_Math::constrain(_target_roll_rate, -_roll_rate_max, _roll_rate_max);
    _target_pitch_rate = AP_Math::constrain(_target_pitch_rate, -_pitch_rate_max, _pitch_rate_max);
    _target_yaw_rate = AP_Math::constrain(_target_yaw_rate, -_yaw_rate_max, _yaw_rate_max);
}

void AC_AttitudeControl::rate_controller_run() {
    // Calculate dt
    uint64_t now_us = hal->micros64();
    float dt = (now_us - _last_update_us) * 1.0e-6f;
    _last_update_us = now_us;

    if (dt > 0.1f) {
        dt = 0.0025f;  // Default to 400Hz if dt is too large
    }

    // Get current rates from AHRS
    const Vector3f& gyro = _ahrs.get_gyro();

    // Run rate PIDs
    _roll_out = _pid_rate_roll.update_all(_target_roll_rate, gyro.x, dt);
    _pitch_out = _pid_rate_pitch.update_all(_target_pitch_rate, gyro.y, dt);
    _yaw_out = _pid_rate_yaw.update_all(_target_yaw_rate, gyro.z, dt);

    // Constrain outputs to -1 to 1
    _roll_out = AP_Math::constrain(_roll_out, -1.0f, 1.0f);
    _pitch_out = AP_Math::constrain(_pitch_out, -1.0f, 1.0f);
    _yaw_out = AP_Math::constrain(_yaw_out, -1.0f, 1.0f);
}

void AC_AttitudeControl::get_motor_outputs(float& roll_out, float& pitch_out, float& yaw_out) {
    roll_out = _roll_out;
    pitch_out = _pitch_out;
    yaw_out = _yaw_out;
}

void AC_AttitudeControl::set_rate_limits(float roll_rate_max, float pitch_rate_max, float yaw_rate_max) {
    _roll_rate_max = roll_rate_max;
    _pitch_rate_max = pitch_rate_max;
    _yaw_rate_max = yaw_rate_max;
}

void AC_AttitudeControl::set_angle_limits(float roll_angle_max, float pitch_angle_max) {
    _roll_angle_max = roll_angle_max;
    _pitch_angle_max = pitch_angle_max;
}

void AC_AttitudeControl::set_throttle_out(float throttle, bool apply_angle_boost) {
    _throttle_out = throttle;

    // Apply angle boost to compensate for tilt
    if (apply_angle_boost) {
        float roll = _ahrs.get_roll();
        float pitch = _ahrs.get_pitch();

        // Calculate tilt compensation (1/cos(angle))
        float tilt_compensation = 1.0f / sqrtf(cosf(roll) * cosf(roll) * cosf(pitch) * cosf(pitch));
        tilt_compensation = AP_Math::constrain(tilt_compensation, 1.0f, 1.5f);

        _throttle_out *= tilt_compensation;
    }

    // Constrain throttle
    _throttle_out = AP_Math::constrain(_throttle_out, 0.0f, 1.0f);
}
