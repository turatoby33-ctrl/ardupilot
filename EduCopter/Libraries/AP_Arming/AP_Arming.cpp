#include "AP_Arming.h"
#include "../AP_HAL/AP_HAL.h"
#include <cstdio>

extern AP_HAL* hal;

AP_Arming::AP_Arming(AP_InertialSensor& ins, AP_Baro& baro, AP_GPS& gps,
                     AP_Compass& compass, AP_AHRS& ahrs) :
    _ins(ins),
    _baro(baro),
    _gps(gps),
    _compass(compass),
    _ahrs(ahrs),
    _armed(false),
    _last_result(ARMING_OK),
    _checks_enabled(CHECK_ALL)
{
}

void AP_Arming::init() {
    printf("AP_Arming: Initializing arming checks\n");
    _armed = false;
}

AP_Arming::ArmingResult AP_Arming::arm_checks(bool report) {
    if (_armed) {
        _last_result = ARMING_FAILED_ALREADY_ARMED;
        if (report) printf("AP_Arming: Already armed\n");
        return _last_result;
    }

    // Check IMU
    if ((_checks_enabled & CHECK_IMU) && !check_imu()) {
        _last_result = ARMING_FAILED_IMU_NOT_CALIBRATED;
        if (report) printf("AP_Arming: IMU not calibrated\n");
        return _last_result;
    }

    // Check barometer
    if ((_checks_enabled & CHECK_BARO) && !check_baro()) {
        _last_result = ARMING_FAILED_BARO_NOT_HEALTHY;
        if (report) printf("AP_Arming: Barometer not healthy\n");
        return _last_result;
    }

    // Check GPS (optional for indoor flight)
    if ((_checks_enabled & CHECK_GPS) && !check_gps()) {
        _last_result = ARMING_FAILED_GPS_NOT_HEALTHY;
        if (report) printf("AP_Arming: GPS not healthy (can be disabled for indoor flight)\n");
        return _last_result;
    }

    // Check compass
    if ((_checks_enabled & CHECK_COMPASS) && !check_compass()) {
        _last_result = ARMING_FAILED_COMPASS_NOT_CALIBRATED;
        if (report) printf("AP_Arming: Compass not calibrated\n");
        return _last_result;
    }

    // Check AHRS
    if ((_checks_enabled & CHECK_AHRS) && !check_ahrs()) {
        _last_result = ARMING_FAILED_AHRS_NOT_HEALTHY;
        if (report) printf("AP_Arming: AHRS not healthy\n");
        return _last_result;
    }

    // Check throttle is low
    if ((_checks_enabled & CHECK_THROTTLE) && !check_throttle()) {
        _last_result = ARMING_FAILED_THROTTLE_TOO_HIGH;
        if (report) printf("AP_Arming: Throttle too high\n");
        return _last_result;
    }

    _last_result = ARMING_OK;
    if (report) printf("AP_Arming: All pre-arm checks passed\n");
    return _last_result;
}

bool AP_Arming::arm(bool report) {
    // Run arming checks
    if (arm_checks(report) != ARMING_OK) {
        return false;
    }

    _armed = true;

    if (report) {
        printf("AP_Arming: *** ARMED ***\n");
    }

    return true;
}

bool AP_Arming::disarm(bool force) {
    if (!_armed && !force) {
        return false;
    }

    _armed = false;
    printf("AP_Arming: Disarmed\n");

    return true;
}

bool AP_Arming::check_imu() {
    return _ins.is_calibrated() && _ins.healthy();
}

bool AP_Arming::check_baro() {
    return _baro.healthy();
}

bool AP_Arming::check_gps() {
    // GPS is optional for some flight modes
    // Check if GPS is present and has fix
    return _gps.healthy() && _gps.have_fix();
}

bool AP_Arming::check_compass() {
    return _compass.is_calibrated() && _compass.healthy();
}

bool AP_Arming::check_ahrs() {
    return _ahrs.healthy();
}

bool AP_Arming::check_throttle() {
    // Check that throttle is low (below 10%)
    if (hal->rcin) {
        uint16_t throttle_pwm = hal->rcin->read(2);  // Throttle is usually channel 3 (index 2)
        return throttle_pwm < 1100;  // Below 10%
    }
    return true;
}

void AP_Arming::set_check_enabled(uint32_t check_mask, bool enabled) {
    if (enabled) {
        _checks_enabled |= check_mask;
    } else {
        _checks_enabled &= ~check_mask;
    }
}

const char* AP_Arming::get_result_string(ArmingResult result) const {
    switch (result) {
        case ARMING_OK:
            return "OK";
        case ARMING_FAILED_IMU_NOT_CALIBRATED:
            return "IMU not calibrated";
        case ARMING_FAILED_BARO_NOT_HEALTHY:
            return "Barometer not healthy";
        case ARMING_FAILED_GPS_NOT_HEALTHY:
            return "GPS not healthy";
        case ARMING_FAILED_COMPASS_NOT_CALIBRATED:
            return "Compass not calibrated";
        case ARMING_FAILED_AHRS_NOT_HEALTHY:
            return "AHRS not healthy";
        case ARMING_FAILED_THROTTLE_TOO_HIGH:
            return "Throttle too high";
        case ARMING_FAILED_ALREADY_ARMED:
            return "Already armed";
        default:
            return "Unknown error";
    }
}
