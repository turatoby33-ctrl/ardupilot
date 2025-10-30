#include "SITL_Physics.h"
#include "../AP_Math/AP_Math.h"
#include <cmath>
#include <cstring>
#include <random>

SITL_Physics::SITL_Physics() :
    ground_level(0.0f),
    on_ground(true),
    home_latitude(-35.363261),  // Canberra, Australia
    home_longitude(149.165230),
    home_altitude(584.0f),
    mass(1.5f),                 // 1.5 kg quadcopter
    arm_length(0.225f),         // 225mm arms (450mm frame)
    thrust_coefficient(12.0f),  // Newtons per unit throttle
    drag_coefficient(0.1f)
{
    // Reset to initial state
    reset();

    // Moment of inertia for a 450-size quad (approximate)
    moment_of_inertia[0] = 0.015f;  // Roll (Ixx)
    moment_of_inertia[1] = 0.015f;  // Pitch (Iyy)
    moment_of_inertia[2] = 0.025f;  // Yaw (Izz)
}

void SITL_Physics::reset() {
    position.zero();
    position.z = -1.0f;  // Start 1m above ground (NED: down is positive)

    velocity.zero();
    accel.zero();

    attitude = Quaternion(1, 0, 0, 0);  // Level attitude
    gyro.zero();
    angular_accel.zero();

    memset(motor_thrust, 0, sizeof(motor_thrust));

    on_ground = false;
}

void SITL_Physics::update(float dt) {
    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    // Calculate forces and moments
    Vector3f forces = calculate_forces();
    Vector3f moments = calculate_moments();

    // Integrate angular velocity
    angular_accel.x = moments.x / moment_of_inertia[0];
    angular_accel.y = moments.y / moment_of_inertia[1];
    angular_accel.z = moments.z / moment_of_inertia[2];

    gyro += angular_accel * dt;

    // Integrate attitude
    integrate_attitude(dt);

    // Rotate forces to NED frame
    Vector3f forces_ned = attitude.rotate(forces);

    // Add gravity
    forces_ned.z += mass * GRAVITY_MSS;

    // Calculate acceleration in NED frame
    Vector3f accel_ned = forces_ned / mass;

    // Integrate velocity
    velocity += accel_ned * dt;

    // Integrate position
    position += velocity * dt;

    // Check ground collision
    check_ground_collision();

    // Calculate body frame acceleration for IMU
    // Convert NED acceleration back to body frame for sensor
    Quaternion attitude_inv(attitude.q1, -attitude.q2, -attitude.q3, -attitude.q4);
    Vector3f gravity_ned(0, 0, GRAVITY_MSS);
    Vector3f gravity_body = attitude_inv.rotate(gravity_ned);

    accel = attitude_inv.rotate(accel_ned) + gravity_body;
}

Vector3f SITL_Physics::calculate_forces() {
    // Calculate total thrust and individual motor thrusts
    float total_thrust = 0.0f;
    for (int i = 0; i < 4; i++) {
        total_thrust += motor_thrust[i] * thrust_coefficient;
    }

    // Thrust is in body Z direction (up in body frame = negative Z)
    Vector3f thrust_force(0, 0, -total_thrust);

    // Add drag (simplified)
    Vector3f vel_body = attitude.rotate(velocity);  // Need to invert this
    Quaternion attitude_inv(attitude.q1, -attitude.q2, -attitude.q3, -attitude.q4);
    vel_body = attitude_inv.rotate(velocity);

    Vector3f drag = vel_body * -drag_coefficient;

    return thrust_force + drag;
}

Vector3f SITL_Physics::calculate_moments() {
    /*
     * Quad-X motor layout:
     *    2(CCW)  1(CW)
     *       \   /
     *        \ /
     *         X
     *        / \
     *       /   \
     *   3(CW)    4(CCW)
     */

    float arm = arm_length;

    // Thrust from each motor
    float f1 = motor_thrust[0] * thrust_coefficient;
    float f2 = motor_thrust[1] * thrust_coefficient;
    float f3 = motor_thrust[2] * thrust_coefficient;
    float f4 = motor_thrust[3] * thrust_coefficient;

    // Roll moment (about X axis)
    // Positive roll = right wing down = M2 and M3 higher than M1 and M4
    float roll_moment = arm / sqrtf(2.0f) * (f2 + f3 - f1 - f4);

    // Pitch moment (about Y axis)
    // Positive pitch = nose up = M3 and M4 higher than M1 and M2
    float pitch_moment = arm / sqrtf(2.0f) * (f3 + f4 - f1 - f2);

    // Yaw moment (about Z axis)
    // CW motors (1, 3) produce negative yaw moment when spinning
    // CCW motors (2, 4) produce positive yaw moment when spinning
    float yaw_drag_coefficient = 0.01f;  // Simplified
    float yaw_moment = yaw_drag_coefficient * (-f1 + f2 - f3 + f4);

    return Vector3f(roll_moment, pitch_moment, yaw_moment);
}

void SITL_Physics::integrate_attitude(float dt) {
    // Integrate using quaternion derivative
    // q_dot = 0.5 * q * omega
    Quaternion omega_quat(0, gyro.x, gyro.y, gyro.z);
    Quaternion q_dot = attitude * omega_quat;

    attitude.q1 += 0.5f * q_dot.q1 * dt;
    attitude.q2 += 0.5f * q_dot.q2 * dt;
    attitude.q3 += 0.5f * q_dot.q3 * dt;
    attitude.q4 += 0.5f * q_dot.q4 * dt;

    attitude.normalize();
}

void SITL_Physics::check_ground_collision() {
    // Check if we've hit the ground (NED: down is positive)
    if (position.z >= ground_level) {
        position.z = ground_level;
        on_ground = true;

        // Zero out vertical velocity if going down
        if (velocity.z > 0) {
            velocity.z = 0;
        }

        // Apply ground friction
        velocity.x *= 0.95f;
        velocity.y *= 0.95f;

        // Limit angular rates on ground
        gyro *= 0.9f;
    } else {
        on_ground = false;
    }
}

void SITL_Physics::set_motor_pwm(uint8_t motor, uint16_t pwm) {
    if (motor >= 4) return;

    // Convert PWM (1000-2000) to thrust (0-1)
    float thrust = (pwm - 1000.0f) / 1000.0f;
    motor_thrust[motor] = AP_Math::constrain(thrust, 0.0f, 1.0f);
}

void SITL_Physics::get_imu(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    // Return body frame acceleration and angular rates
    ax = accel.x;
    ay = accel.y;
    az = accel.z;

    gx = gyro.x;
    gy = gyro.y;
    gz = gyro.z;

    // Add sensor noise
    add_noise_accel(ax, ay, az);
    add_noise_gyro(gx, gy, gz);
}

void SITL_Physics::get_baro(float& pressure, float& temperature, float& altitude) {
    // Calculate altitude (negative of position.z since NED down is positive)
    altitude = -position.z;

    // Calculate pressure using barometric formula
    // P = P0 * (1 - 0.0065 * h / 288.15) ^ 5.255
    float h = altitude + home_altitude;
    pressure = 101325.0f * powf(1.0f - 0.0065f * h / 288.15f, 5.255f);

    // Temperature decreases with altitude
    temperature = 15.0f - 0.0065f * h;

    // Add small noise
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0f, 1.0f);

    pressure += distribution(generator) * 10.0f;
    temperature += distribution(generator) * 0.5f;
}

void SITL_Physics::get_gps(float& lat, float& lon, float& alt, float& vn, float& ve, float& vd) {
    // Convert NED position to lat/lon
    // Simple flat-earth approximation
    float dlat = position.x / 111319.9f;  // meters to degrees latitude
    float dlon = position.y / (111319.9f * cosf(home_latitude * DEG_TO_RAD));

    lat = home_latitude + dlat;
    lon = home_longitude + dlon;
    alt = home_altitude - position.z;  // MSL altitude

    // Velocity
    vn = velocity.x;
    ve = velocity.y;
    vd = velocity.z;

    // Add GPS noise
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0f, 1.0f);

    lat += distribution(generator) * 1e-7f;  // ~1cm noise
    lon += distribution(generator) * 1e-7f;
    alt += distribution(generator) * 0.5f;   // 0.5m noise
}

void SITL_Physics::get_compass(float& mx, float& my, float& mz) {
    // Earth's magnetic field in NED frame (simplified)
    // Pointing roughly north with downward component
    Vector3f mag_ned(0.4f, 0.0f, 0.5f);  // Arbitrary units (milliGauss)

    // Rotate to body frame
    Quaternion attitude_inv(attitude.q1, -attitude.q2, -attitude.q3, -attitude.q4);
    Vector3f mag_body = attitude_inv.rotate(mag_ned);

    mx = mag_body.x;
    my = mag_body.y;
    mz = mag_body.z;

    // Add noise
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0f, 1.0f);

    mx += distribution(generator) * 0.01f;
    my += distribution(generator) * 0.01f;
    mz += distribution(generator) * 0.01f;
}

void SITL_Physics::add_noise_accel(float& ax, float& ay, float& az) {
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0f, 1.0f);

    // IMU noise (typical for MEMS sensors)
    ax += distribution(generator) * 0.05f;  // m/s^2
    ay += distribution(generator) * 0.05f;
    az += distribution(generator) * 0.05f;
}

void SITL_Physics::add_noise_gyro(float& gx, float& gy, float& gz) {
    static std::default_random_engine generator;
    static std::normal_distribution<float> distribution(0.0f, 1.0f);

    // Gyro noise
    gx += distribution(generator) * 0.001f;  // rad/s
    gy += distribution(generator) * 0.001f;
    gz += distribution(generator) * 0.001f;
}
