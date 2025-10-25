#include "mode_stabilize.h"
#include "EduCopter.h"
#include <cstdio>

extern EduCopter copter;

bool ModeStabilize::init(bool ignore_checks) {
    printf("Mode: Switched to STABILIZE\n");
    return true;
}

void ModeStabilize::run() {
    // Get pilot inputs
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.angle_max.get());

    float target_yaw_rate = get_pilot_desired_yaw_rate();
    float throttle_out = get_pilot_desired_throttle();

    // Calculate target yaw from yaw rate
    float current_yaw = copter.get_ahrs().get_yaw();
    float dt = 1.0f / MAIN_LOOP_RATE;
    float target_yaw = current_yaw + target_yaw_rate * dt;

    // Convert to centidegrees
    float target_yaw_cd = target_yaw * RAD_TO_DEG * 100.0f;

    // Call attitude controller
    copter.get_attitude_control().input_euler_angle_roll_pitch_yaw(
        target_roll, target_pitch, target_yaw_cd, false);

    // Set throttle
    copter.get_attitude_control().set_throttle_out(throttle_out, true);

    // Run rate controller
    copter.get_attitude_control().rate_controller_run();

    // Output to motors
    float roll_out, pitch_out, yaw_out;
    copter.get_attitude_control().get_motor_outputs(roll_out, pitch_out, yaw_out);

    copter.get_motors().set_roll(roll_out);
    copter.get_motors().set_pitch(pitch_out);
    copter.get_motors().set_yaw(yaw_out);
    copter.get_motors().set_throttle(throttle_out);
}
