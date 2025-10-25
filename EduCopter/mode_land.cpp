#include "mode_land.h"
#include "EduCopter.h"
#include "Libraries/AP_Math/AP_Math.h"
#include <cstdio>

extern EduCopter copter;
extern AP_HAL* hal;

bool ModeLand::init(bool ignore_checks) {
    land_state = LAND_STATE_DESCEND;
    target_climb_rate = -0.5f;  // m/s (descending)
    land_start_time_us = hal->micros64();

    printf("Mode: Switched to LAND\n");
    return true;
}

void ModeLand::run() {
    // Update land state machine
    update_land_state();

    // Get current state
    float current_altitude = copter.get_ahrs().get_altitude();

    float throttle_out = 0.0f;

    switch (land_state) {
        case LAND_STATE_DESCEND:
            // Descend at constant rate
            {
                float current_climb_rate = -copter.get_ahrs().get_velocity().z;
                float climb_rate_error = target_climb_rate - current_climb_rate;

                // Simple P controller for descent rate
                float kp = 0.5f;
                throttle_out = ALT_HOLD_THROTTLE_NEUTRAL + climb_rate_error * kp;
                throttle_out = AP_Math::constrain(throttle_out, 0.2f, 0.7f);
            }
            break;

        case LAND_STATE_FINAL:
            // Very slow descent near ground
            throttle_out = 0.15f;
            break;

        case LAND_STATE_COMPLETE:
            // Motors off
            throttle_out = 0.0f;

            // Disarm after 1 second on ground
            if (hal->micros64() - land_complete_time_us > 1000000) {
                copter.disarm();
            }
            break;
    }

    // Keep level attitude
    copter.get_attitude_control().input_euler_angle_roll_pitch_yaw(0, 0, 0, false);

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

void ModeLand::update_land_state() {
    float current_altitude = copter.get_ahrs().get_altitude();
    const Vector3f& accel = copter.get_ins().get_accel();

    switch (land_state) {
        case LAND_STATE_DESCEND:
            // Transition to final when close to ground (< 2m)
            if (current_altitude < 2.0f) {
                land_state = LAND_STATE_FINAL;
                target_climb_rate = -0.2f;  // Slow descent
                printf("Land: Entering final descent\n");
            }
            break;

        case LAND_STATE_FINAL:
            // Detect landing - low altitude and low vertical acceleration
            if (current_altitude < 0.3f && fabsf(accel.z - GRAVITY_MSS) < 2.0f) {
                land_state = LAND_STATE_COMPLETE;
                land_complete_time_us = hal->micros64();
                printf("Land: Complete\n");
            }
            break;

        case LAND_STATE_COMPLETE:
            // Stay in this state
            break;
    }
}
