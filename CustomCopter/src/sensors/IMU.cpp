#include "IMU.h"
#include <cmath>
#include <random>
#include <chrono>

namespace CustomCopter {

// ============================================================================
// IMU Base Class
// ============================================================================

IMU::IMU()
    : sample_rate_hz_(1000)
    , healthy_(false)
    , calibrated_(false)
{
    accel_offsets_ = Vector3f(0, 0, 0);
    gyro_offsets_ = Vector3f(0, 0, 0);
}

bool IMU::init(uint32_t sample_rate_hz) {
    sample_rate_hz_ = sample_rate_hz;
    healthy_ = true;
    return true;
}

bool IMU::update() {
    // Read raw sensor data
    Vector3f raw_accel, raw_gyro;

    if (!read_raw_accel(raw_accel) || !read_raw_gyro(raw_gyro)) {
        healthy_ = false;
        return false;
    }

    // Apply calibration offsets
    data_.accel_ms2 = raw_accel - accel_offsets_;
    data_.gyro_rads = raw_gyro - gyro_offsets_;

    // Update timestamp
    auto now = std::chrono::steady_clock::now();
    data_.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();

    healthy_ = true;
    return true;
}

bool IMU::calibrate_accel() {
    // Accumulate samples for calibration
    const int NUM_SAMPLES = 100;
    Vector3f sum(0, 0, 0);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        Vector3f raw_accel;
        if (!read_raw_accel(raw_accel)) {
            return false;
        }
        sum += raw_accel;
        // Small delay between samples
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Calculate average
    Vector3f average = sum / static_cast<float>(NUM_SAMPLES);

    // Set offsets (subtract gravity from Z axis)
    // Assuming Z-down convention, gravity should be ~+9.81 m/s²
    accel_offsets_ = average;
    accel_offsets_.z -= 9.81f;

    calibrated_ = true;
    return true;
}

bool IMU::calibrate_gyro() {
    // Accumulate samples for calibration
    const int NUM_SAMPLES = 100;
    Vector3f sum(0, 0, 0);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        Vector3f raw_gyro;
        if (!read_raw_gyro(raw_gyro)) {
            return false;
        }
        sum += raw_gyro;
        // Small delay between samples
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Calculate average (gyro bias)
    gyro_offsets_ = sum / static_cast<float>(NUM_SAMPLES);

    calibrated_ = true;
    return true;
}

void IMU::set_accel_offsets(const Vector3f& offsets) {
    accel_offsets_ = offsets;
}

void IMU::set_gyro_offsets(const Vector3f& offsets) {
    gyro_offsets_ = offsets;
}

// ============================================================================
// Simulated IMU Implementation
// ============================================================================

IMU_Simulated::IMU_Simulated()
    : IMU()
    , roll_rad_(0.0f)
    , pitch_rad_(0.0f)
    , yaw_rad_(0.0f)
    , accel_noise_(0.1f)   // 0.1 m/s² noise
    , gyro_noise_(0.01f)   // 0.01 rad/s noise
{
    rates_rads_ = Vector3f(0, 0, 0);
}

bool IMU_Simulated::init(uint32_t sample_rate_hz) {
    return IMU::init(sample_rate_hz);
}

void IMU_Simulated::set_attitude(float roll_rad, float pitch_rad, float yaw_rad) {
    roll_rad_ = roll_rad;
    pitch_rad_ = pitch_rad;
    yaw_rad_ = yaw_rad;
}

void IMU_Simulated::set_rates(float roll_rate_rads, float pitch_rate_rads,
                               float yaw_rate_rads) {
    rates_rads_.x = roll_rate_rads;
    rates_rads_.y = pitch_rate_rads;
    rates_rads_.z = yaw_rate_rads;
}

bool IMU_Simulated::read_raw_accel(Vector3f& accel) {
    // Calculate acceleration based on attitude
    // In body frame: gravity component depends on tilt

    const float GRAVITY = 9.81f;  // m/s²

    // Rotation matrix from NED to body frame
    float cr = std::cos(roll_rad_);
    float sr = std::sin(roll_rad_);
    float cp = std::cos(pitch_rad_);
    float sp = std::sin(pitch_rad_);

    // Transform gravity vector (0, 0, g) from NED to body frame
    // Body frame convention: X-forward, Y-right, Z-down
    accel.x = -GRAVITY * sp + generate_noise(accel_noise_);
    accel.y = GRAVITY * sr * cp + generate_noise(accel_noise_);
    accel.z = GRAVITY * cr * cp + generate_noise(accel_noise_);

    return true;
}

bool IMU_Simulated::read_raw_gyro(Vector3f& gyro) {
    // Return the set angular rates plus noise
    gyro.x = rates_rads_.x + generate_noise(gyro_noise_);
    gyro.y = rates_rads_.y + generate_noise(gyro_noise_);
    gyro.z = rates_rads_.z + generate_noise(gyro_noise_);

    return true;
}

float IMU_Simulated::generate_noise(float stddev) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0.0f, stddev);
    return dist(gen);
}

} // namespace CustomCopter
