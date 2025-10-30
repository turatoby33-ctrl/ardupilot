#pragma once

#include "../AP_Math/vector3.h"
#include "../AP_HAL/AP_HAL.h"

class AP_InertialSensor {
public:
    AP_InertialSensor();

    void init();
    bool update();

    // Get accelerometer data (m/s^2)
    const Vector3f& get_accel() const { return accel; }

    // Get gyroscope data (rad/s)
    const Vector3f& get_gyro() const { return gyro; }

    // Get delta velocity (m/s) - integrated accel since last update
    const Vector3f& get_delta_velocity() const { return delta_velocity; }

    // Get delta angle (rad) - integrated gyro since last update
    const Vector3f& get_delta_angle() const { return delta_angle; }

    // Get sample rate
    float get_sample_rate() const { return sample_rate_hz; }
    float get_delta_time() const { return delta_time; }

    // Calibration
    void calibrate_accel();
    void calibrate_gyro();
    bool is_calibrated() const { return calibrated; }

    // Health check
    bool healthy() const { return _healthy; }

private:
    Vector3f accel;
    Vector3f gyro;
    Vector3f delta_velocity;
    Vector3f delta_angle;

    Vector3f accel_offset;
    Vector3f gyro_offset;

    float sample_rate_hz;
    float delta_time;
    uint64_t last_update_us;

    bool calibrated;
    bool _healthy;
};
