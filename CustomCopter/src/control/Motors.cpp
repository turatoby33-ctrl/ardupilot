#include "Motors.h"
#include <algorithm>
#include <cmath>

namespace CustomCopter {

Motors::Motors()
    : pwm_(nullptr)
    , frame_type_(FrameType::QUAD_X)
    , num_motors_(4)
    , roll_input_(0.0f)
    , pitch_input_(0.0f)
    , yaw_input_(0.0f)
    , throttle_input_(0.0f)
    , pwm_min_(1000)
    , pwm_max_(2000)
    , pwm_freq_hz_(400)
    , thrust_expo_(0.65f)  // ArduPilot default
    , hover_throttle_(0.5f)
    , battery_voltage_(12.6f)  // 3S LiPo nominal
    , battery_voltage_min_(10.5f)
    , battery_voltage_max_(12.6f)
    , battery_voltage_nominal_(12.6f)
    , battery_compensation_enabled_(true)
    , armed_(false)
    , output_enabled_(false)
    , healthy_(false)
    , thrust_loss_(false)
    , throttle_limit_(false)
    , motor_limit_lower_(0.0f)
    , motor_limit_upper_(1.0f)
{
    // Initialize motor outputs to zero
    motor_output_.fill(0.0f);
    motor_pwm_.fill(pwm_min_);

    // Initialize default frame type
    init_quad_x();
}

bool Motors::init(HAL::PWM* pwm) {
    if (!pwm) {
        return false;
    }

    pwm_ = pwm;

    // Initialize PWM channels for each motor
    for (uint8_t i = 0; i < num_motors_; i++) {
        uint8_t channel = motor_factors_[i].pwm_channel;
        if (!pwm_->init(channel, pwm_freq_hz_)) {
            healthy_ = false;
            return false;
        }
    }

    healthy_ = true;
    return true;
}

void Motors::set_frame_type(FrameType type) {
    frame_type_ = type;

    switch (frame_type_) {
        case FrameType::QUAD_X:
            init_quad_x();
            break;
        case FrameType::QUAD_PLUS:
            init_quad_plus();
            break;
        case FrameType::HEXA_X:
            init_hexa_x();
            break;
        case FrameType::OCTA_X:
            init_octa_x();
            break;
    }
}

void Motors::init_quad_x() {
    // Quad X frame configuration
    // Motor numbering (viewed from top, nose forward):
    //     1 (Front Right)    4 (Front Left)
    //          \             /
    //           \           /
    //            \         /
    //             \       /
    //              \     /
    //               \   /
    //                \ /
    //                 X
    //                / \
    //               /   \
    //              /     \
    //             /       \
    //            /         \
    //           /           \
    //          /             \
    //     2 (Rear Right)    3 (Rear Left)
    //
    // Motor rotation (ArduPilot standard):
    // Motors 1,3 rotate CW (looking down)
    // Motors 2,4 rotate CCW (looking down)

    num_motors_ = 4;

    // Motor 1 - Front Right (CW)
    motor_factors_[0].roll     =  1.0f;  // Right
    motor_factors_[0].pitch    = -1.0f;  // Forward
    motor_factors_[0].yaw      =  1.0f;  // CW
    motor_factors_[0].throttle =  1.0f;
    motor_factors_[0].pwm_channel = 0;

    // Motor 2 - Rear Right (CCW)
    motor_factors_[1].roll     =  1.0f;  // Right
    motor_factors_[1].pitch    =  1.0f;  // Backward
    motor_factors_[1].yaw      = -1.0f;  // CCW
    motor_factors_[1].throttle =  1.0f;
    motor_factors_[1].pwm_channel = 1;

    // Motor 3 - Rear Left (CW)
    motor_factors_[2].roll     = -1.0f;  // Left
    motor_factors_[2].pitch    =  1.0f;  // Backward
    motor_factors_[2].yaw      =  1.0f;  // CW
    motor_factors_[2].throttle =  1.0f;
    motor_factors_[2].pwm_channel = 2;

    // Motor 4 - Front Left (CCW)
    motor_factors_[3].roll     = -1.0f;  // Left
    motor_factors_[3].pitch    = -1.0f;  // Forward
    motor_factors_[3].yaw      = -1.0f;  // CCW
    motor_factors_[3].throttle =  1.0f;
    motor_factors_[3].pwm_channel = 3;
}

void Motors::init_quad_plus() {
    // Quad + frame configuration
    num_motors_ = 4;

    // Motor 1 - Front
    motor_factors_[0].roll     =  0.0f;
    motor_factors_[0].pitch    = -1.0f;
    motor_factors_[0].yaw      =  1.0f;
    motor_factors_[0].throttle =  1.0f;
    motor_factors_[0].pwm_channel = 0;

    // Motor 2 - Right
    motor_factors_[1].roll     =  1.0f;
    motor_factors_[1].pitch    =  0.0f;
    motor_factors_[1].yaw      = -1.0f;
    motor_factors_[1].throttle =  1.0f;
    motor_factors_[1].pwm_channel = 1;

    // Motor 3 - Rear
    motor_factors_[2].roll     =  0.0f;
    motor_factors_[2].pitch    =  1.0f;
    motor_factors_[2].yaw      =  1.0f;
    motor_factors_[2].throttle =  1.0f;
    motor_factors_[2].pwm_channel = 2;

    // Motor 4 - Left
    motor_factors_[3].roll     = -1.0f;
    motor_factors_[3].pitch    =  0.0f;
    motor_factors_[3].yaw      = -1.0f;
    motor_factors_[3].throttle =  1.0f;
    motor_factors_[3].pwm_channel = 3;
}

void Motors::init_hexa_x() {
    // Hexacopter X configuration
    num_motors_ = 6;
    // TODO: Implement hexa X mixing
}

void Motors::init_octa_x() {
    // Octacopter X configuration
    num_motors_ = 8;
    // TODO: Implement octa X mixing
}

void Motors::set_rpy_throttle(float roll, float pitch, float yaw, float throttle) {
    roll_input_ = constrain_float(roll, -1.0f, 1.0f);
    pitch_input_ = constrain_float(pitch, -1.0f, 1.0f);
    yaw_input_ = constrain_float(yaw, -1.0f, 1.0f);
    throttle_input_ = constrain_float(throttle, 0.0f, 1.0f);
}

void Motors::set_pwm_range(uint16_t min_us, uint16_t max_us) {
    pwm_min_ = min_us;
    pwm_max_ = max_us;
}

void Motors::set_battery_voltage(float voltage) {
    battery_voltage_ = voltage;
}

void Motors::set_battery_voltage_limits(float min_v, float max_v) {
    battery_voltage_min_ = min_v;
    battery_voltage_max_ = max_v;
}

void Motors::output() {
    // Reset status flags
    thrust_loss_ = false;
    throttle_limit_ = false;

    // If not armed or output not enabled, set motors to minimum
    if (!armed_ || !output_enabled_) {
        for (uint8_t i = 0; i < num_motors_; i++) {
            motor_output_[i] = 0.0f;
            motor_pwm_[i] = pwm_min_;

            if (pwm_ && output_enabled_) {
                pwm_->write(motor_factors_[i].pwm_channel, pwm_min_);
            }
        }
        return;
    }

    // Calculate motor mixing
    calculate_motor_mix();

    // Apply thrust curve and battery compensation
    for (uint8_t i = 0; i < num_motors_; i++) {
        float output = motor_output_[i];

        // Apply thrust curve linearization
        output = apply_thrust_curve(output);

        // Apply battery voltage compensation
        if (battery_compensation_enabled_) {
            output = compensate_battery_voltage(output);
        }

        motor_output_[i] = output;
    }

    // Constrain outputs and check for thrust loss
    constrain_motor_outputs();

    // Convert to PWM and output
    for (uint8_t i = 0; i < num_motors_; i++) {
        motor_pwm_[i] = throttle_to_pwm(motor_output_[i]);

        if (pwm_) {
            pwm_->write(motor_factors_[i].pwm_channel, motor_pwm_[i]);
        }
    }
}

void Motors::calculate_motor_mix() {
    // Standard motor mixing algorithm
    // Each motor = throttle + roll_factor*roll + pitch_factor*pitch + yaw_factor*yaw

    for (uint8_t i = 0; i < num_motors_; i++) {
        float output = 0.0f;

        // Add throttle component
        output += motor_factors_[i].throttle * throttle_input_;

        // Add roll component
        output += motor_factors_[i].roll * roll_input_;

        // Add pitch component
        output += motor_factors_[i].pitch * pitch_input_;

        // Add yaw component
        output += motor_factors_[i].yaw * yaw_input_;

        motor_output_[i] = output;
    }
}

float Motors::apply_thrust_curve(float throttle) const {
    // Apply thrust curve expo to linearize thrust output
    // ArduPilot's thrust curve: output = throttle * (1 - expo) + throttle^2 * expo
    // This compensates for non-linear motor/propeller thrust

    if (throttle <= 0.0f) {
        return 0.0f;
    }

    if (thrust_expo_ <= 0.0f) {
        return throttle;  // Linear
    }

    // Apply expo curve
    float throttle_squared = throttle * throttle;
    return throttle * (1.0f - thrust_expo_) + throttle_squared * thrust_expo_;
}

float Motors::compensate_battery_voltage(float throttle) const {
    // Compensate for battery voltage drop
    // As battery voltage drops, increase throttle to maintain thrust

    if (battery_voltage_ <= 0.0f || battery_voltage_nominal_ <= 0.0f) {
        return throttle;
    }

    // Calculate voltage ratio
    float voltage_ratio = battery_voltage_nominal_ / battery_voltage_;

    // Constrain ratio to reasonable range (0.8 to 1.2)
    voltage_ratio = constrain_float(voltage_ratio, 0.8f, 1.2f);

    // Apply compensation
    return throttle * voltage_ratio;
}

void Motors::constrain_motor_outputs() {
    // Constrain motor outputs to valid range [0, 1]
    // Check for thrust loss (motor saturation)

    bool any_motor_limited = false;

    for (uint8_t i = 0; i < num_motors_; i++) {
        float original_output = motor_output_[i];

        // Constrain to range
        motor_output_[i] = constrain_float(motor_output_[i],
                                           motor_limit_lower_,
                                           motor_limit_upper_);

        // Check if motor was limited
        if (std::abs(motor_output_[i] - original_output) > 0.001f) {
            any_motor_limited = true;
        }
    }

    // Set thrust loss flag if any motor was limited
    if (any_motor_limited) {
        thrust_loss_ = true;
    }

    // Check if all motors are near upper limit (throttle limiting)
    bool all_near_max = true;
    for (uint8_t i = 0; i < num_motors_; i++) {
        if (motor_output_[i] < motor_limit_upper_ - 0.1f) {
            all_near_max = false;
            break;
        }
    }
    throttle_limit_ = all_near_max;
}

uint16_t Motors::throttle_to_pwm(float throttle) const {
    // Convert throttle [0, 1] to PWM [pwm_min, pwm_max]
    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return pwm_min_ + static_cast<uint16_t>(throttle * (pwm_max_ - pwm_min_));
}

float Motors::get_motor_output(uint8_t motor_num) const {
    if (motor_num < num_motors_) {
        return motor_output_[motor_num];
    }
    return 0.0f;
}

uint16_t Motors::get_motor_pwm(uint8_t motor_num) const {
    if (motor_num < num_motors_) {
        return motor_pwm_[motor_num];
    }
    return pwm_min_;
}

} // namespace CustomCopter
