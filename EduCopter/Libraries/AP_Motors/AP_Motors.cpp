#include "AP_Motors.h"
#include "../AP_Math/AP_Math.h"
#include <cmath>
#include <cstdio>

extern AP_HAL* hal;

AP_Motors::AP_Motors(FrameType frame_type) :
    _frame_type(frame_type),
    _roll_in(0.0f),
    _pitch_in(0.0f),
    _yaw_in(0.0f),
    _throttle_in(0.0f),
    _pwm_min(1000),
    _pwm_max(2000),
    _spin_min(0.1f),
    _spin_max(0.95f),
    _armed(false)
{
    for (int i = 0; i < 4; i++) {
        _motor_out[i] = 0.0f;
    }
}

void AP_Motors::init() {
    printf("AP_Motors: Initializing motors\n");

    // Set PWM frequency to 400Hz (typical for ESCs)
    if (hal->rcout) {
        hal->rcout->set_freq(400);
    }

    // Initialize motors to minimum PWM
    for (int i = 0; i < 4; i++) {
        if (hal->rcout) {
            hal->rcout->write(i, _pwm_min);
        }
    }

    printf("AP_Motors: Frame type: %s\n",
           _frame_type == FRAME_QUAD_X ? "Quad-X" : "Quad-Plus");
}

void AP_Motors::armed(bool arm) {
    _armed = arm;

    if (!_armed) {
        // Reset inputs when disarmed
        _roll_in = 0.0f;
        _pitch_in = 0.0f;
        _yaw_in = 0.0f;
        _throttle_in = 0.0f;

        printf("AP_Motors: Disarmed\n");
    } else {
        printf("AP_Motors: Armed\n");
    }
}

void AP_Motors::output() {
    if (!hal->rcout) {
        return;
    }

    if (_armed) {
        output_armed_stabilizing();
    } else {
        output_disarmed();
    }
}

void AP_Motors::output_armed_stabilizing() {
    // Perform motor mixing based on frame type
    if (_frame_type == FRAME_QUAD_X) {
        output_quad_x();
    } else {
        output_quad_plus();
    }

    // Output to motors
    hal->rcout->cork();  // Begin atomic update
    for (int i = 0; i < 4; i++) {
        uint16_t pwm = throttle_to_pwm(_motor_out[i]);
        hal->rcout->write(i, pwm);
    }
    hal->rcout->push();  // End atomic update
}

void AP_Motors::output_disarmed() {
    // Output minimum PWM when disarmed
    hal->rcout->cork();
    for (int i = 0; i < 4; i++) {
        hal->rcout->write(i, _pwm_min);
        _motor_out[i] = 0.0f;
    }
    hal->rcout->push();
}

void AP_Motors::output_quad_x() {
    /*
     * Quad-X frame:
     *    CW  CCW
     *      \ /
     *       X
     *      / \
     *    CCW  CW
     *
     * Motor layout:
     *    2   1
     *     \ /
     *      X
     *     / \
     *    3   4
     *
     * Motor 1: Front-Right (CW)
     * Motor 2: Front-Left (CCW)
     * Motor 3: Rear-Left (CW)
     * Motor 4: Rear-Right (CCW)
     */

    float throttle = AP_Math::constrain(_throttle_in, 0.0f, 1.0f);

    // Mixing:
    // Motor 1 (FR): Throttle - Roll - Pitch + Yaw
    // Motor 2 (FL): Throttle + Roll - Pitch - Yaw
    // Motor 3 (RL): Throttle + Roll + Pitch + Yaw
    // Motor 4 (RR): Throttle - Roll + Pitch - Yaw

    _motor_out[0] = throttle - _roll_in - _pitch_in + _yaw_in;  // Front-Right
    _motor_out[1] = throttle + _roll_in - _pitch_in - _yaw_in;  // Front-Left
    _motor_out[2] = throttle + _roll_in + _pitch_in + _yaw_in;  // Rear-Left
    _motor_out[3] = throttle - _roll_in + _pitch_in - _yaw_in;  // Rear-Right

    // Constrain motors and apply thrust curve
    for (int i = 0; i < 4; i++) {
        _motor_out[i] = AP_Math::constrain(_motor_out[i], 0.0f, 1.0f);

        // Apply minimum spin when armed
        if (_motor_out[i] > 0.0f) {
            _motor_out[i] = _spin_min + (_spin_max - _spin_min) * _motor_out[i];
        }

        // Apply thrust curve
        _motor_out[i] = apply_thrust_curve(_motor_out[i]);
    }
}

void AP_Motors::output_quad_plus() {
    /*
     * Quad-Plus frame:
     *       1(CW)
     *        |
     *   2----+----4
     *  (CCW) | (CCW)
     *        |
     *       3(CW)
     */

    float throttle = AP_Math::constrain(_throttle_in, 0.0f, 1.0f);

    // Mixing for Plus frame
    _motor_out[0] = throttle - _pitch_in + _yaw_in;  // Front
    _motor_out[1] = throttle + _roll_in - _yaw_in;   // Left
    _motor_out[2] = throttle + _pitch_in + _yaw_in;  // Rear
    _motor_out[3] = throttle - _roll_in - _yaw_in;   // Right

    // Constrain and apply thrust curve
    for (int i = 0; i < 4; i++) {
        _motor_out[i] = AP_Math::constrain(_motor_out[i], 0.0f, 1.0f);

        if (_motor_out[i] > 0.0f) {
            _motor_out[i] = _spin_min + (_spin_max - _spin_min) * _motor_out[i];
        }

        _motor_out[i] = apply_thrust_curve(_motor_out[i]);
    }
}

float AP_Motors::apply_thrust_curve(float thrust) {
    // Linearize thrust (motors have non-linear thrust curve)
    // Simple square root approximation
    if (thrust > 0.0f) {
        return sqrtf(thrust);
    }
    return 0.0f;
}

uint16_t AP_Motors::throttle_to_pwm(float throttle) {
    // Convert 0-1 throttle to PWM microseconds
    throttle = AP_Math::constrain(throttle, 0.0f, 1.0f);
    return _pwm_min + (uint16_t)(throttle * (_pwm_max - _pwm_min));
}
