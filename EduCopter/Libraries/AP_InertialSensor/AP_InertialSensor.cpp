#include "AP_InertialSensor.h"
#include "../AP_HAL/AP_HAL.h"
#include <cstdio>

extern AP_HAL* hal;

AP_InertialSensor::AP_InertialSensor() :
    sample_rate_hz(1000.0f),
    delta_time(0.001f),
    last_update_us(0),
    calibrated(false),
    _healthy(false)
{
    accel.zero();
    gyro.zero();
    delta_velocity.zero();
    delta_angle.zero();
    accel_offset.zero();
    gyro_offset.zero();
}

void AP_InertialSensor::init() {
    printf("AP_InertialSensor: Initializing IMU\n");
    last_update_us = hal->micros64();
    _healthy = true;
}

bool AP_InertialSensor::update() {
    if (!hal->ins) {
        return false;
    }

    // Get raw sensor data from HAL
    float ax, ay, az, gx, gy, gz;
    hal->ins->get_accel(ax, ay, az);
    hal->ins->get_gyro(gx, gy, gz);

    // Apply calibration offsets
    accel.x = ax - accel_offset.x;
    accel.y = ay - accel_offset.y;
    accel.z = az - accel_offset.z;

    gyro.x = gx - gyro_offset.x;
    gyro.y = gy - gyro_offset.y;
    gyro.z = gz - gyro_offset.z;

    // Calculate delta time
    uint64_t now_us = hal->micros64();
    delta_time = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    // Calculate delta velocity and delta angle
    delta_velocity = accel * delta_time;
    delta_angle = gyro * delta_time;

    _healthy = true;
    return true;
}

void AP_InertialSensor::calibrate_accel() {
    printf("AP_InertialSensor: Calibrating accelerometer...\n");

    // Simple calibration - average 100 samples
    Vector3f accel_sum(0, 0, 0);
    for (int i = 0; i < 100; i++) {
        float ax, ay, az, gx, gy, gz;
        hal->ins->get_accel(ax, ay, az);
        hal->ins->get_gyro(gx, gy, gz);
        accel_sum.x += ax;
        accel_sum.y += ay;
        accel_sum.z += az;
        hal->scheduler->delay(10);
    }

    // Calculate offsets (assume vehicle is level, so Z should read 9.81 m/s^2)
    accel_offset.x = accel_sum.x / 100.0f;
    accel_offset.y = accel_sum.y / 100.0f;
    accel_offset.z = (accel_sum.z / 100.0f) - 9.81f;

    printf("AP_InertialSensor: Accel offsets: %.3f, %.3f, %.3f\n",
           accel_offset.x, accel_offset.y, accel_offset.z);
}

void AP_InertialSensor::calibrate_gyro() {
    printf("AP_InertialSensor: Calibrating gyroscope...\n");

    // Simple calibration - average 100 samples
    Vector3f gyro_sum(0, 0, 0);
    for (int i = 0; i < 100; i++) {
        float ax, ay, az, gx, gy, gz;
        hal->ins->get_accel(ax, ay, az);
        hal->ins->get_gyro(gx, gy, gz);
        gyro_sum.x += gx;
        gyro_sum.y += gy;
        gyro_sum.z += gz;
        hal->scheduler->delay(10);
    }

    // Calculate offsets (gyro should read 0 when stationary)
    gyro_offset.x = gyro_sum.x / 100.0f;
    gyro_offset.y = gyro_sum.y / 100.0f;
    gyro_offset.z = gyro_sum.z / 100.0f;

    calibrated = true;

    printf("AP_InertialSensor: Gyro offsets: %.3f, %.3f, %.3f\n",
           gyro_offset.x, gyro_offset.y, gyro_offset.z);
}
