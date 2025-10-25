#pragma once

#include "../AP_Math/vector3.h"
#include "../AP_Math/quaternion.h"
#include <stdint.h>

// Quadcopter physics simulation
class SITL_Physics {
public:
    SITL_Physics();

    // Update physics simulation
    void update(float dt);

    // Set motor outputs (PWM 1000-2000)
    void set_motor_pwm(uint8_t motor, uint16_t pwm);

    // Get simulated sensor data
    void get_imu(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
    void get_baro(float& pressure, float& temperature, float& altitude);
    void get_gps(float& lat, float& lon, float& alt, float& vn, float& ve, float& vd);
    void get_compass(float& mx, float& my, float& mz);

    // Get state
    const Vector3f& get_position() const { return position; }
    const Vector3f& get_velocity() const { return velocity; }
    const Quaternion& get_attitude() const { return attitude; }
    const Vector3f& get_gyro() const { return gyro; }

    // Reset to initial conditions
    void reset();

private:
    // State
    Vector3f position;       // NED frame (m)
    Vector3f velocity;       // NED frame (m/s)
    Vector3f accel;          // Body frame (m/s^2)
    Quaternion attitude;     // Body to NED rotation
    Vector3f gyro;           // Body frame (rad/s)
    Vector3f angular_accel;  // Body frame (rad/s^2)

    // Motor outputs (0-1)
    float motor_thrust[4];

    // Ground level
    float ground_level;
    bool on_ground;

    // Home position (for GPS)
    double home_latitude;
    double home_longitude;
    float home_altitude;

    // Vehicle parameters
    float mass;              // kg
    float arm_length;        // m (distance from center to motor)
    float thrust_coefficient; // N per throttle unit
    float drag_coefficient;
    float moment_of_inertia[3]; // kg*m^2 (roll, pitch, yaw)

    // Aerodynamics
    Vector3f calculate_forces();
    Vector3f calculate_moments();

    // Integration
    void integrate_position(float dt);
    void integrate_velocity(float dt);
    void integrate_attitude(float dt);

    // Ground collision
    void check_ground_collision();

    // Sensor simulation with noise
    void add_noise_accel(float& ax, float& ay, float& az);
    void add_noise_gyro(float& gx, float& gy, float& gz);
};
