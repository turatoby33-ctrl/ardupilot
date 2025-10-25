#pragma once

#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../AP_Baro/AP_Baro.h"
#include "../AP_GPS/AP_GPS.h"
#include "../AP_Compass/AP_Compass.h"
#include "../AP_AHRS/AP_AHRS.h"
#include <stdint.h>

class AP_Arming {
public:
    // Arming check results
    enum ArmingResult {
        ARMING_OK = 0,
        ARMING_FAILED_IMU_NOT_CALIBRATED,
        ARMING_FAILED_BARO_NOT_HEALTHY,
        ARMING_FAILED_GPS_NOT_HEALTHY,
        ARMING_FAILED_COMPASS_NOT_CALIBRATED,
        ARMING_FAILED_AHRS_NOT_HEALTHY,
        ARMING_FAILED_THROTTLE_TOO_HIGH,
        ARMING_FAILED_ALREADY_ARMED,
        ARMING_FAILED_UNKNOWN
    };

    AP_Arming(AP_InertialSensor& ins, AP_Baro& baro, AP_GPS& gps,
              AP_Compass& compass, AP_AHRS& ahrs);

    void init();

    // Check if vehicle can be armed
    ArmingResult arm_checks(bool report);

    // Arm/disarm
    bool arm(bool report = true);
    bool disarm(bool force = false);

    // Get arming state
    bool is_armed() const { return _armed; }

    // Get last arming result
    ArmingResult get_last_result() const { return _last_result; }
    const char* get_result_string(ArmingResult result) const;

    // Enable/disable specific checks
    void set_check_enabled(uint32_t check_mask, bool enabled);

    // Check flags
    enum ArmingCheckFlags {
        CHECK_IMU       = (1 << 0),
        CHECK_BARO      = (1 << 1),
        CHECK_GPS       = (1 << 2),
        CHECK_COMPASS   = (1 << 3),
        CHECK_AHRS      = (1 << 4),
        CHECK_THROTTLE  = (1 << 5),
        CHECK_ALL       = 0xFFFFFFFF
    };

private:
    // Sensors
    AP_InertialSensor& _ins;
    AP_Baro& _baro;
    AP_GPS& _gps;
    AP_Compass& _compass;
    AP_AHRS& _ahrs;

    // State
    bool _armed;
    ArmingResult _last_result;
    uint32_t _checks_enabled;

    // Individual checks
    bool check_imu();
    bool check_baro();
    bool check_gps();
    bool check_compass();
    bool check_ahrs();
    bool check_throttle();
};
