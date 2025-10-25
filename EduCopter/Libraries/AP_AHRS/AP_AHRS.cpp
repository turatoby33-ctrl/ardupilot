#include "AP_AHRS.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_HAL/AP_HAL.h"
#include <cmath>
#include <cstring>
#include <cstdio>

extern AP_HAL* hal;

AP_AHRS::AP_AHRS(AP_InertialSensor& ins, AP_Baro& baro, AP_GPS& gps, AP_Compass& compass) :
    _ins(ins),
    _baro(baro),
    _gps(gps),
    _compass(compass),
    roll(0.0f),
    pitch(0.0f),
    yaw(0.0f),
    home_lat(0.0f),
    home_lon(0.0f),
    home_alt(0.0f),
    _have_home(false),
    last_update_us(0),
    dt(0.01f),
    ekf_healthy(false)
{
    gyro.zero();
    position_ned.zero();
    velocity_ned.zero();

    // Initialize EKF state and covariance
    memset(ekf_state, 0, sizeof(ekf_state));
    memset(ekf_covariance, 0, sizeof(ekf_covariance));

    // Initialize covariance diagonal
    for (int i = 0; i < 10; i++) {
        ekf_covariance[i][i] = 1.0f;
    }
}

void AP_AHRS::init() {
    printf("AP_AHRS: Initializing AHRS/EKF\n");
    last_update_us = hal->micros64();
    ekf_healthy = true;
}

bool AP_AHRS::update() {
    // Calculate dt
    uint64_t now_us = hal->micros64();
    dt = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    if (dt > 0.1f) {
        dt = 0.01f;  // Limit dt
    }

    // Update attitude from IMU
    update_attitude();

    // Update position estimate
    update_position();

    ekf_healthy = true;
    return true;
}

void AP_AHRS::update_attitude() {
    // Get IMU data
    const Vector3f& accel = _ins.get_accel();
    gyro = _ins.get_gyro();

    // Complementary filter for attitude estimation
    // This is a simplified approach - a full EKF would be more accurate

    // Integrate gyro for attitude prediction
    float roll_rate = gyro.x + sinf(roll) * tanf(pitch) * gyro.y + cosf(roll) * tanf(pitch) * gyro.z;
    float pitch_rate = cosf(roll) * gyro.y - sinf(roll) * gyro.z;
    float yaw_rate = sinf(roll) / cosf(pitch) * gyro.y + cosf(roll) / cosf(pitch) * gyro.z;

    roll += roll_rate * dt;
    pitch += pitch_rate * dt;
    yaw += yaw_rate * dt;

    // Correct attitude using accelerometer (for roll and pitch)
    if (accel.length() > 0.1f) {
        // Calculate roll and pitch from accelerometer
        float accel_roll = atan2f(accel.y, accel.z);
        float accel_pitch = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));

        // Complementary filter (98% gyro, 2% accel)
        float alpha = 0.02f;
        roll = (1.0f - alpha) * roll + alpha * accel_roll;
        pitch = (1.0f - alpha) * pitch + alpha * accel_pitch;
    }

    // Correct yaw using compass
    if (_compass.healthy()) {
        float compass_yaw = _compass.get_heading();

        // Complementary filter for yaw (95% gyro, 5% compass)
        float alpha = 0.05f;
        float yaw_error = AP_Math::wrap_PI(compass_yaw - yaw);
        yaw += alpha * yaw_error;
    }

    // Wrap angles
    roll = AP_Math::wrap_PI(roll);
    pitch = AP_Math::wrap_PI(pitch);
    yaw = AP_Math::wrap_2PI(yaw);

    // Update quaternion
    attitude_quat.from_euler(roll, pitch, yaw);
}

void AP_AHRS::update_position() {
    // Simple position estimation using barometer and GPS
    // In a full EKF, this would be integrated with IMU accelerations

    // Update altitude from barometer
    if (_baro.healthy()) {
        float baro_alt = _baro.get_altitude();
        float baro_climb_rate = _baro.get_climb_rate();

        // Use barometer for vertical position
        position_ned.z = -baro_alt;  // NED frame: down is positive
        velocity_ned.z = -baro_climb_rate;
    }

    // Update horizontal position from GPS
    if (_gps.healthy() && _have_home) {
        // Simple conversion from lat/lon to NED (assuming flat earth)
        float lat = _gps.get_latitude();
        float lon = _gps.get_longitude();

        // Approximate conversion (meters per degree)
        float dlat = (lat - home_lat) * 111319.9f;  // meters
        float dlon = (lon - home_lon) * 111319.9f * cosf(home_lat * DEG_TO_RAD);

        position_ned.x = dlat;   // North
        position_ned.y = dlon;   // East

        // Get velocity from GPS
        Vector3f gps_vel = _gps.get_velocity_ned();
        velocity_ned.x = gps_vel.x;
        velocity_ned.y = gps_vel.y;
    } else if (!_have_home && _gps.healthy()) {
        // Set home position on first GPS fix
        set_home(_gps.get_latitude(), _gps.get_longitude(), _gps.get_altitude());
    }
}

void AP_AHRS::set_home(float lat, float lon, float alt) {
    home_lat = lat;
    home_lon = lon;
    home_alt = alt;
    _have_home = true;

    printf("AP_AHRS: Home position set to %.6f, %.6f, %.1f\n", lat, lon, alt);
}

void AP_AHRS::reset() {
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    position_ned.zero();
    velocity_ned.zero();

    // Reset EKF covariance
    memset(ekf_covariance, 0, sizeof(ekf_covariance));
    for (int i = 0; i < 10; i++) {
        ekf_covariance[i][i] = 1.0f;
    }

    printf("AP_AHRS: Reset complete\n");
}
