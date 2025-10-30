#pragma once

#include "../AP_AHRS/AP_AHRS.h"
#include "../AC_PID/AC_PID.h"
#include "../AP_Math/vector3.h"

// Attitude controller for multicopters
class AC_AttitudeControl {
public:
    AC_AttitudeControl(AP_AHRS& ahrs);

    void init();

    // Angle control (outer loop)
    void input_euler_angle_roll_pitch_yaw(float roll_angle_cd, float pitch_angle_cd, float yaw_angle_cd, bool slew_yaw);

    // Rate control (inner loop) - returns motor outputs
    void rate_controller_run();

    // Get motor outputs (normalized -1 to 1)
    void get_motor_outputs(float& roll_out, float& pitch_out, float& yaw_out);

    // Set rate limits
    void set_rate_limits(float roll_rate_max, float pitch_rate_max, float yaw_rate_max);

    // Set angle limits
    void set_angle_limits(float roll_angle_max, float pitch_angle_max);

    // Throttle control
    void set_throttle_out(float throttle, bool apply_angle_boost);
    float get_throttle_out() const { return _throttle_out; }

    // Get PIDs for tuning
    AC_PID& get_rate_roll_pid() { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() { return _pid_rate_yaw; }

private:
    AP_AHRS& _ahrs;

    // Rate PIDs (inner loop)
    AC_PID _pid_rate_roll;
    AC_PID _pid_rate_pitch;
    AC_PID _pid_rate_yaw;

    // Angle PIDs (outer loop) - simplified as P-only
    float _p_angle_roll;
    float _p_angle_pitch;
    float _p_angle_yaw;

    // Target angles (radians)
    float _target_roll;
    float _target_pitch;
    float _target_yaw;

    // Target rates (rad/s)
    float _target_roll_rate;
    float _target_pitch_rate;
    float _target_yaw_rate;

    // Rate limits (rad/s)
    float _roll_rate_max;
    float _pitch_rate_max;
    float _yaw_rate_max;

    // Angle limits (radians)
    float _roll_angle_max;
    float _pitch_angle_max;

    // Motor outputs
    float _roll_out;
    float _pitch_out;
    float _yaw_out;
    float _throttle_out;

    // Timing
    uint64_t _last_update_us;
};
