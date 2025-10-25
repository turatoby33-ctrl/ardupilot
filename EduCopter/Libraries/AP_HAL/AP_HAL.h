#pragma once

#include <stdint.h>
#include <cstdio>

// Forward declarations
class AP_HAL_InertialSensor;
class AP_HAL_Baro;
class AP_HAL_GPS;
class AP_HAL_Compass;
class AP_HAL_RCInput;
class AP_HAL_RCOutput;
class AP_HAL_Scheduler;
class AP_HAL_Util;

// Main HAL class - provides access to all hardware
class AP_HAL {
public:
    AP_HAL_InertialSensor* ins;
    AP_HAL_Baro* baro;
    AP_HAL_GPS* gps;
    AP_HAL_Compass* compass;
    AP_HAL_RCInput* rcin;
    AP_HAL_RCOutput* rcout;
    AP_HAL_Scheduler* scheduler;
    AP_HAL_Util* util;

    AP_HAL() : ins(nullptr), baro(nullptr), gps(nullptr), compass(nullptr),
               rcin(nullptr), rcout(nullptr), scheduler(nullptr), util(nullptr) {}

    virtual void init() = 0;
    virtual uint64_t micros64() = 0;
    virtual uint32_t millis() = 0;
};

// Inertial Sensor (IMU) interface
class AP_HAL_InertialSensor {
public:
    virtual bool update() = 0;
    virtual void get_accel(float& ax, float& ay, float& az) = 0;
    virtual void get_gyro(float& gx, float& gy, float& gz) = 0;
    virtual float get_accel_scale() { return 1.0f; }
    virtual float get_gyro_scale() { return 1.0f; }
};

// Barometer interface
class AP_HAL_Baro {
public:
    virtual bool update() = 0;
    virtual float get_pressure() = 0;    // Pascal
    virtual float get_temperature() = 0;  // Celsius
    virtual float get_altitude() = 0;     // meters
};

// GPS interface
class AP_HAL_GPS {
public:
    enum GPS_Status {
        NO_GPS = 0,
        NO_FIX = 1,
        GPS_OK_FIX_2D = 2,
        GPS_OK_FIX_3D = 3,
        GPS_OK_FIX_3D_DGPS = 4,
        GPS_OK_FIX_3D_RTK_FLOAT = 5,
        GPS_OK_FIX_3D_RTK_FIXED = 6
    };

    virtual bool update() = 0;
    virtual GPS_Status status() = 0;
    virtual float get_latitude() = 0;   // degrees
    virtual float get_longitude() = 0;  // degrees
    virtual float get_altitude() = 0;   // meters
    virtual float get_ground_speed() = 0; // m/s
    virtual float get_ground_course() = 0; // degrees
    virtual uint8_t get_num_sats() = 0;
};

// Compass/Magnetometer interface
class AP_HAL_Compass {
public:
    virtual bool update() = 0;
    virtual void get_field(float& mx, float& my, float& mz) = 0; // milliGauss
    virtual float get_declination() { return 0.0f; } // radians
};

// RC Input interface
class AP_HAL_RCInput {
public:
    virtual bool update() = 0;
    virtual uint16_t read(uint8_t channel) = 0; // PWM microseconds (1000-2000)
    virtual uint8_t num_channels() = 0;
    virtual bool failsafe() = 0;
};

// RC Output interface
class AP_HAL_RCOutput {
public:
    virtual void write(uint8_t channel, uint16_t pwm) = 0; // PWM microseconds
    virtual void cork() = 0;   // Begin atomic update
    virtual void push() = 0;   // End atomic update
    virtual void set_freq(uint16_t freq) = 0; // Set PWM frequency
};

// Scheduler interface
class AP_HAL_Scheduler {
public:
    virtual void delay(uint32_t ms) = 0;
    virtual void delay_microseconds(uint32_t us) = 0;
    virtual void register_timer_process(void (*proc)(void)) = 0;
};

// Utility functions
class AP_HAL_Util {
public:
    virtual void set_system_clock(uint64_t time_utc_usec) {}
};

// Global HAL instance
extern AP_HAL* hal;
