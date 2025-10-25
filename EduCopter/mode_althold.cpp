#include "mode_althold.h"
#include "EduCopter.h"
#include "Libraries/AP_Math/AP_Math.h"
#include <cstdio>

extern EduCopter copter;
extern AP_HAL* hal;

bool ModeAltHold::init(bool ignore_checks) {
    // Set target altitude to current altitude
    target_altitude = copter.get_ahrs().get_altitude();
    target_climb_rate = 0.0f;
    altitude_set = true;

    printf("Mode: Switched to ALT_HOLD (target alt: %.2f m)\n", target_altitude);
    return true;
}

void ModeAltHold::run() {
    // Get pilot inputs
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.angle_max.get());

    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // Get pilot throttle and convert to climb rate
    uint16_t throttle_pwm = copter.get_pilot_throttle();
    float throttle_in = (throttle_pwm - RC_MID) / 500.0f;

    // Apply deadzone
    if (fabsf(throttle_in) < RC_DEADZONE / 500.0f) {
        throttle_in = 0.0f;
    }

    // Convert throttle to climb rate (m/s)
    // Throttle up = positive climb rate, down = negative
    float max_climb_rate = 2.5f;  // m/s
    target_climb_rate = throttle_in * max_climb_rate;

    // Update target altitude based on climb rate
    float dt = 1.0f / MAIN_LOOP_RATE;
    target_altitude += target_climb_rate * dt;

    // Get current altitude
    float current_altitude = copter.get_ahrs().get_altitude();

    // Run altitude controller to get throttle
    float throttle_out = altitude_controller(target_altitude, current_altitude, dt);

    // Calculate target yaw
    float current_yaw = copter.get_ahrs().get_yaw();
    float target_yaw = current_yaw + target_yaw_rate * dt;
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

float ModeAltHold::altitude_controller(float target_alt, float current_alt, float dt) {
    // Simple P controller for altitude
    float altitude_error = target_alt - current_alt;

    // P gain
    float kp = 2.0f;  // Aggressive P gain

    // Calculate desired climb rate
    float desired_climb_rate = altitude_error * kp;

    // Constrain climb rate
    float max_climb_rate = 2.5f;  // m/s
    desired_climb_rate = AP_Math::constrain(desired_climb_rate, -max_climb_rate, max_climb_rate);

    // Get current climb rate from baro
    float current_climb_rate = -copter.get_ahrs().get_velocity().z;  // NED frame

    // PI controller for climb rate
    static float climb_rate_integrator = 0.0f;
    float climb_rate_error = desired_climb_rate - current_climb_rate;

    float kp_rate = 0.5f;
    float ki_rate = 0.25f;

    climb_rate_integrator += climb_rate_error * ki_rate * dt;
    climb_rate_integrator = AP_Math::constrain(climb_rate_integrator, -0.3f, 0.3f);

    float throttle_out = ALT_HOLD_THROTTLE_NEUTRAL +
                        climb_rate_error * kp_rate +
                        climb_rate_integrator;

    // Constrain throttle
    throttle_out = AP_Math::constrain(throttle_out, 0.1f, 0.9f);

    return throttle_out;
}
