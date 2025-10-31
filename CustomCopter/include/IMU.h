#pragma once

#include "DataTypes.h"
#include "HAL.h"

namespace CustomCopter {

// ============================================================================
// IMU (Inertial Measurement Unit) Interface
// Provides accelerometer and gyroscope data
// ============================================================================
class IMU {
public:
    IMU();
    virtual ~IMU() = default;

    // Initialize IMU
    // sample_rate_hz: Sample rate in Hz (typically 1000 Hz)
    virtual bool init(uint32_t sample_rate_hz);

    // Update IMU data (read from hardware)
    // Should be called at the configured sample rate
    virtual bool update();

    // Get the latest IMU data
    const IMUData& get_data() const { return data_; }

    // Get accelerometer data (m/s²)
    const Vector3f& get_accel() const { return data_.accel_ms2; }

    // Get gyroscope data (rad/s)
    const Vector3f& get_gyro() const { return data_.gyro_rads; }

    // Calibrate accelerometer (vehicle must be level)
    bool calibrate_accel();

    // Calibrate gyroscope (vehicle must be stationary)
    bool calibrate_gyro();

    // Set calibration offsets
    void set_accel_offsets(const Vector3f& offsets);
    void set_gyro_offsets(const Vector3f& offsets);

    // Get calibration offsets
    const Vector3f& get_accel_offsets() const { return accel_offsets_; }
    const Vector3f& get_gyro_offsets() const { return gyro_offsets_; }

    // Get sample rate
    uint32_t get_sample_rate_hz() const { return sample_rate_hz_; }

    // Check if IMU is healthy
    bool is_healthy() const { return healthy_; }

protected:
    // Read raw sensor data from hardware
    // To be implemented by specific IMU driver (MPU6000, ICM20689, etc.)
    virtual bool read_raw_accel(Vector3f& accel) = 0;
    virtual bool read_raw_gyro(Vector3f& gyro) = 0;

    IMUData data_;
    Vector3f accel_offsets_;
    Vector3f gyro_offsets_;

    uint32_t sample_rate_hz_;
    bool healthy_;
    bool calibrated_;
};

// ============================================================================
// Simulated IMU for testing (generates synthetic data)
// ============================================================================
class IMU_Simulated : public IMU {
public:
    IMU_Simulated();
    ~IMU_Simulated() override = default;

    bool init(uint32_t sample_rate_hz) override;

    // Set simulated attitude (for testing)
    void set_attitude(float roll_rad, float pitch_rad, float yaw_rad);

    // Set simulated angular rates (for testing)
    void set_rates(float roll_rate_rads, float pitch_rate_rads, float yaw_rate_rads);

protected:
    bool read_raw_accel(Vector3f& accel) override;
    bool read_raw_gyro(Vector3f& gyro) override;

private:
    // Simulated state
    float roll_rad_;
    float pitch_rad_;
    float yaw_rad_;
    Vector3f rates_rads_;

    // Noise parameters (for realistic simulation)
    float accel_noise_;
    float gyro_noise_;

    // Generate Gaussian noise
    float generate_noise(float stddev);
};

} // namespace CustomCopter
