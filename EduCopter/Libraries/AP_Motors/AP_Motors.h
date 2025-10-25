#pragma once

#include "../AP_HAL/AP_HAL.h"
#include <stdint.h>

// Motor class for quadcopter
class AP_Motors {
public:
    // Motor frame types
    enum FrameType {
        FRAME_QUAD_X = 0,
        FRAME_QUAD_PLUS = 1,
    };

    AP_Motors(FrameType frame_type = FRAME_QUAD_X);

    void init();

    // Set motor outputs
    // roll, pitch, yaw: -1.0 to 1.0
    // throttle: 0.0 to 1.0
    void set_roll(float roll) { _roll_in = roll; }
    void set_pitch(float pitch) { _pitch_in = pitch; }
    void set_yaw(float yaw) { _yaw_in = yaw; }
    void set_throttle(float throttle) { _throttle_in = throttle; }

    // Output to motors
    void output();

    // Arm/disarm
    void armed(bool arm);
    bool is_armed() const { return _armed; }

    // Spin motors when armed
    void output_armed_stabilizing();
    void output_disarmed();

    // Get individual motor outputs (0-1000)
    float get_motor_output(uint8_t motor) const {
        return (motor < 4) ? _motor_out[motor] : 0.0f;
    }

    // PWM range
    void set_pwm_range(uint16_t min_pwm, uint16_t max_pwm) {
        _pwm_min = min_pwm;
        _pwm_max = max_pwm;
    }

    // Motor spin parameters
    void set_spin_min(float spin_min) { _spin_min = spin_min; }
    void set_spin_max(float spin_max) { _spin_max = spin_max; }

private:
    FrameType _frame_type;

    // Input values (normalized)
    float _roll_in;
    float _pitch_in;
    float _yaw_in;
    float _throttle_in;

    // Motor outputs (0-1)
    float _motor_out[4];

    // PWM parameters
    uint16_t _pwm_min;
    uint16_t _pwm_max;
    float _spin_min;  // Minimum spin when armed (0-1)
    float _spin_max;  // Maximum spin (0-1)

    // State
    bool _armed;

    // Motor mixing
    void output_quad_x();
    void output_quad_plus();

    // Helpers
    float apply_thrust_curve(float thrust);
    uint16_t throttle_to_pwm(float throttle);
};
