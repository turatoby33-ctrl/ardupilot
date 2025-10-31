#include "ModeStabilize.h"
#include "Copter.h"

namespace CustomCopter {

ModeStabilize::ModeStabilize()
    : Mode()
    , target_roll_(0.0f)
    , target_pitch_(0.0f)
    , target_yaw_rate_(0.0f)
    , throttle_out_(0.0f)
{
}

bool ModeStabilize::init(bool ignore_checks) {
    // Call base class init
    if (!Mode::init(ignore_checks)) {
        return false;
    }

    // STABILIZE mode has minimal requirements
    // No GPS needed, no altitude hold, just basic attitude control
    return true;
}

bool ModeStabilize::enter() {
    // Call base class enter
    if (!Mode::enter()) {
        return false;
    }

    // Reset target angles
    target_roll_ = 0.0f;
    target_pitch_ = 0.0f;
    target_yaw_rate_ = 0.0f;
    throttle_out_ = 0.0f;

    // Reset attitude controller integrator
    AttitudeControl* attitude_control = get_attitude_control();
    if (attitude_control) {
        attitude_control->reset_rate_controller_I();
    }

    return true;
}

void ModeStabilize::exit() {
    // Call base class exit
    Mode::exit();
}

void ModeStabilize::run() {
    // ========================================================================
    // STABILIZE Mode Main Loop (runs at 400 Hz)
    // ========================================================================
    //
    // Flow:
    // 1. Get pilot input (RC sticks)
    // 2. Convert to desired angles/rates
    // 3. Run attitude controller (angle -> rate -> motor commands)
    // 4. Set throttle
    // 5. Output to motors
    // ========================================================================

    AttitudeControl* attitude_control = get_attitude_control();
    Motors* motors = get_motors();

    if (!attitude_control || !motors) {
        return;  // Can't run without these
    }

    // Step 1: Get pilot's desired lean angles (roll, pitch)
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch);

    // Step 2: Get pilot's desired yaw rate
    target_yaw_rate_ = get_pilot_desired_yaw_rate();

    // Step 3: Get pilot's throttle
    throttle_out_ = get_pilot_throttle();

    // Step 4: Set attitude controller inputs
    // In STABILIZE mode, we use angle control for roll/pitch, rate control for yaw
    // This is the "input_euler_angle_roll_pitch_euler_rate_yaw" function
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        target_roll,
        target_pitch,
        target_yaw_rate_
    );

    // Step 5: Set throttle to attitude controller (for mixing)
    attitude_control->set_throttle_out(throttle_out_);

    // Step 6: Run the attitude controller (converts angle/rate targets to motor commands)
    // This runs the cascaded control:
    //   - Outer loop: angle error -> rate target (P controller)
    //   - Inner loop: rate error -> motor command (PID controller)
    attitude_control->rate_controller_run();

    // Motors are updated by the attitude controller via set_roll/pitch/yaw/throttle
}

void ModeStabilize::get_pilot_desired_lean_angles(float& roll_out, float& pitch_out) {
    // Get pilot's roll and pitch stick inputs
    // These are already scaled to angles by the base class functions

    roll_out = get_pilot_desired_roll();
    pitch_out = get_pilot_desired_pitch();

    // Constrain to maximum lean angle
    roll_out = constrain_lean_angle(roll_out);
    pitch_out = constrain_lean_angle(pitch_out);

    // Store for telemetry/logging
    target_roll_ = roll_out;
    target_pitch_ = pitch_out;
}

void ModeStabilize::run_attitude_controller() {
    // This function is not needed in our implementation
    // The attitude controller is run directly in the run() function
}

} // namespace CustomCopter
