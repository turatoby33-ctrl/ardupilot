#pragma once

#include "../AP_Math/vector3.h"
#include "../AP_Math/quaternion.h"
#include "../AP_InertialSensor/AP_InertialSensor.h"
#include "../AP_Baro/AP_Baro.h"
#include "../AP_GPS/AP_GPS.h"
#include "../AP_Compass/AP_Compass.h"

// Attitude and Heading Reference System with EKF
class AP_AHRS {
public:
    AP_AHRS(AP_InertialSensor& ins, AP_Baro& baro, AP_GPS& gps, AP_Compass& compass);

    void init();
    bool update();

    // Attitude (Euler angles in radians)
    float get_roll() const { return roll; }
    float get_pitch() const { return pitch; }
    float get_yaw() const { return yaw; }

    // Attitude quaternion
    const Quaternion& get_quaternion() const { return attitude_quat; }

    // Angular rates (rad/s)
    const Vector3f& get_gyro() const { return gyro; }

    // Position (NED frame, meters)
    const Vector3f& get_position() const { return position_ned; }

    // Velocity (NED frame, m/s)
    const Vector3f& get_velocity() const { return velocity_ned; }

    // Altitude (meters above home)
    float get_altitude() const { return -position_ned.z; }

    // Heading (radians, 0 = North)
    float get_heading() const { return yaw; }

    // Get home position
    void set_home(float lat, float lon, float alt);
    bool have_home() const { return _have_home; }

    // EKF status
    bool healthy() const { return ekf_healthy; }
    void reset();

private:
    // Sensors
    AP_InertialSensor& _ins;
    AP_Baro& _baro;
    AP_GPS& _gps;
    AP_Compass& _compass;

    // Attitude (Euler angles)
    float roll, pitch, yaw;

    // Attitude quaternion
    Quaternion attitude_quat;

    // Angular rates
    Vector3f gyro;

    // Position and velocity (NED frame)
    Vector3f position_ned;
    Vector3f velocity_ned;

    // Home position
    float home_lat, home_lon, home_alt;
    bool _have_home;

    // EKF state
    float ekf_state[10];  // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, baro_bias, ...
    float ekf_covariance[10][10];  // State covariance matrix

    // Timing
    uint64_t last_update_us;
    float dt;

    // Health
    bool ekf_healthy;

    // Internal methods
    void update_attitude();
    void update_position();
    void predict_covariance();
    void update_baro();
    void update_gps();
};
