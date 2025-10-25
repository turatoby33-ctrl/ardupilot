#include "mode.h"
#include "EduCopter.h"
#include "Libraries/AP_Math/AP_Math.h"

extern EduCopter copter;
extern AP_HAL* hal;

Mode::Mode(EduCopter& copter_ref) :
    copter(copter_ref)
{
}

float Mode::get_pilot_desired_throttle() const {
    uint16_t throttle = copter.get_pilot_throttle();

    // Convert from PWM (1000-2000) to 0-1
    float throttle_out = (throttle - RC_MIN) / (float)(RC_MAX - RC_MIN);
    throttle_out = AP_Math::constrain(throttle_out, 0.0f, 1.0f);

    return throttle_out;
}

void Mode::get_pilot_desired_lean_angles(float& roll_out, float& pitch_out, float angle_max) const {
    // Get RC inputs
    uint16_t roll_pwm = copter.pilot_roll;
    uint16_t pitch_pwm = copter.pilot_pitch;

    // Convert to -1 to 1
    float roll_in = (roll_pwm - RC_MID) / 500.0f;
    float pitch_in = (pitch_pwm - RC_MID) / 500.0f;

    // Apply deadzone
    if (fabsf(roll_in) < RC_DEADZONE / 500.0f) {
        roll_in = 0.0f;
    }
    if (fabsf(pitch_in) < RC_DEADZONE / 500.0f) {
        pitch_in = 0.0f;
    }

    // Convert to angles (in centidegrees)
    roll_out = roll_in * angle_max * 100.0f;
    pitch_out = pitch_in * angle_max * 100.0f;

    // Constrain
    roll_out = AP_Math::constrain(roll_out, -angle_max * 100.0f, angle_max * 100.0f);
    pitch_out = AP_Math::constrain(pitch_out, -angle_max * 100.0f, angle_max * 100.0f);
}

float Mode::get_pilot_desired_yaw_rate() const {
    uint16_t yaw_pwm = copter.pilot_yaw;

    // Convert to -1 to 1
    float yaw_in = (yaw_pwm - RC_MID) / 500.0f;

    // Apply deadzone
    if (fabsf(yaw_in) < RC_DEADZONE / 500.0f) {
        return 0.0f;
    }

    // Convert to yaw rate (rad/s)
    // Maximum yaw rate of ~115 deg/s = 2.0 rad/s
    float yaw_rate = yaw_in * 2.0f;

    return yaw_rate;
}
