#include "Mode.h"
#include "Copter.h"
#include <cmath>

namespace CustomCopter {

Mode::Mode()
    : copter_(nullptr)
    , landed_(true)
    , taking_off_(false)
    , landing_(false)
    , angle_max_rad_(radians(45.0f))  // 45 degrees default
    , roll_sensitivity_(1.0f)
    , pitch_sensitivity_(1.0f)
    , yaw_sensitivity_(1.0f)
    , throttle_takeoff_(0.7f)
    , throttle_land_(0.3f)
{
}

bool Mode::init(bool ignore_checks) {
    // Base implementation - can be overridden
    (void)ignore_checks;
    return true;
}

bool Mode::enter() {
    // Reset mode state when entering
    landed_ = false;
    taking_off_ = false;
    landing_ = false;
    return true;
}

void Mode::exit() {
    // Base implementation - nothing to do
}

const char* Mode::mode_string(FlightMode mode) {
    switch (mode) {
        case FlightMode::STABILIZE:     return "STABILIZE";
        case FlightMode::ACRO:          return "ACRO";
        case FlightMode::ALT_HOLD:      return "ALT_HOLD";
        case FlightMode::AUTO:          return "AUTO";
        case FlightMode::GUIDED:        return "GUIDED";
        case FlightMode::LOITER:        return "LOITER";
        case FlightMode::RTL:           return "RTL";
        case FlightMode::CIRCLE:        return "CIRCLE";
        case FlightMode::LAND:          return "LAND";
        case FlightMode::DRIFT:         return "DRIFT";
        case FlightMode::SPORT:         return "SPORT";
        case FlightMode::FLIP:          return "FLIP";
        case FlightMode::AUTOTUNE:      return "AUTOTUNE";
        case FlightMode::POS_HOLD:      return "POS_HOLD";
        case FlightMode::BRAKE:         return "BRAKE";
        case FlightMode::THROW:         return "THROW";
        case FlightMode::AVOID_ADSB:    return "AVOID_ADSB";
        case FlightMode::GUIDED_NOGPS:  return "GUIDED_NOGPS";
        case FlightMode::SMART_RTL:     return "SMART_RTL";
        case FlightMode::FLOWHOLD:      return "FLOWHOLD";
        case FlightMode::FOLLOW:        return "FOLLOW";
        case FlightMode::ZIGZAG:        return "ZIGZAG";
        case FlightMode::SYSTEMID:      return "SYSTEMID";
        case FlightMode::AUTOROTATE:    return "AUTOROTATE";
        case FlightMode::AUTO_RTL:      return "AUTO_RTL";
        case FlightMode::TURTLE:        return "TURTLE";
        default:                        return "UNKNOWN";
    }
}

float Mode::get_pilot_desired_roll() const {
    // Get RC roll input and scale to angle
    // RC input is typically -1.0 to +1.0
    // Scale by max angle and sensitivity
    float rc_roll = get_rc_input(0, -1.0f, 1.0f);  // Channel 0 = roll
    return rc_roll * angle_max_rad_ * roll_sensitivity_;
}

float Mode::get_pilot_desired_pitch() const {
    // Get RC pitch input and scale to angle
    float rc_pitch = get_rc_input(1, -1.0f, 1.0f);  // Channel 1 = pitch
    return rc_pitch * angle_max_rad_ * pitch_sensitivity_;
}

float Mode::get_pilot_desired_yaw_rate() const {
    // Get RC yaw input and scale to rate
    // Default max yaw rate: 180 deg/s = π rad/s
    float rc_yaw = get_rc_input(3, -1.0f, 1.0f);  // Channel 3 = yaw
    float max_yaw_rate = radians(180.0f);  // 180 deg/s
    return rc_yaw * max_yaw_rate * yaw_sensitivity_;
}

float Mode::get_pilot_throttle() const {
    // Get RC throttle input (0.0 to 1.0)
    return get_rc_input(2, 0.0f, 1.0f);  // Channel 2 = throttle
}

bool Mode::get_pilot_wants_takeoff() const {
    // Pilot wants takeoff if throttle is above threshold
    return get_pilot_throttle() > throttle_takeoff_;
}

bool Mode::get_pilot_wants_land() const {
    // Pilot wants to land if throttle is below threshold
    return get_pilot_throttle() < throttle_land_;
}

float Mode::constrain_lean_angle(float angle_rad) const {
    return constrain_float(angle_rad, -angle_max_rad_, angle_max_rad_);
}

float Mode::get_rc_input(uint8_t channel, float min, float max) const {
    // TODO: This will be implemented when we add RC input handling
    // For now, return neutral/zero values
    (void)channel;

    if (min == 0.0f && max == 1.0f) {
        return 0.0f;  // Throttle neutral
    } else {
        return 0.0f;  // Other channels neutral
    }
}

AttitudeControl* Mode::get_attitude_control() {
    if (copter_) {
        return copter_->get_attitude_control();
    }
    return nullptr;
}

Motors* Mode::get_motors() {
    if (copter_) {
        return copter_->get_motors();
    }
    return nullptr;
}

const Attitude& Mode::get_attitude() const {
    if (copter_) {
        return copter_->get_attitude();
    }
    static Attitude default_attitude;
    return default_attitude;
}

const Vector3f& Mode::get_gyro_rates() const {
    if (copter_) {
        return copter_->get_gyro_rates();
    }
    static Vector3f default_rates;
    return default_rates;
}

} // namespace CustomCopter
