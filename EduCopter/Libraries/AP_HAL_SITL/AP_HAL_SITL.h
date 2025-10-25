#pragma once

#include "../AP_HAL/AP_HAL.h"
#include "../SITL/SITL_Physics.h"
#include <sys/time.h>

// SITL implementations of HAL classes
class SITL_InertialSensor : public AP_HAL_InertialSensor {
public:
    SITL_InertialSensor(SITL_Physics& physics);
    bool update() override;
    void get_accel(float& ax, float& ay, float& az) override;
    void get_gyro(float& gx, float& gy, float& gz) override;

private:
    SITL_Physics& _physics;
    float _accel[3];
    float _gyro[3];
};

class SITL_Baro : public AP_HAL_Baro {
public:
    SITL_Baro(SITL_Physics& physics);
    bool update() override;
    float get_pressure() override { return _pressure; }
    float get_temperature() override { return _temperature; }
    float get_altitude() override { return _altitude; }

private:
    SITL_Physics& _physics;
    float _pressure;
    float _temperature;
    float _altitude;
};

class SITL_GPS : public AP_HAL_GPS {
public:
    SITL_GPS(SITL_Physics& physics);
    bool update() override;
    GPS_Status status() override { return _status; }
    float get_latitude() override { return _latitude; }
    float get_longitude() override { return _longitude; }
    float get_altitude() override { return _altitude; }
    float get_ground_speed() override { return _ground_speed; }
    float get_ground_course() override { return _ground_course; }
    uint8_t get_num_sats() override { return _num_sats; }

private:
    SITL_Physics& _physics;
    GPS_Status _status;
    float _latitude, _longitude, _altitude;
    float _ground_speed, _ground_course;
    uint8_t _num_sats;
};

class SITL_Compass : public AP_HAL_Compass {
public:
    SITL_Compass(SITL_Physics& physics);
    bool update() override;
    void get_field(float& mx, float& my, float& mz) override;

private:
    SITL_Physics& _physics;
    float _field[3];
};

class SITL_RCInput : public AP_HAL_RCInput {
public:
    SITL_RCInput();
    bool update() override;
    uint16_t read(uint8_t channel) override;
    uint8_t num_channels() override { return 8; }
    bool failsafe() override { return false; }

    void set_channel(uint8_t channel, uint16_t value);

private:
    uint16_t _channels[16];
};

class SITL_RCOutput : public AP_HAL_RCOutput {
public:
    SITL_RCOutput(SITL_Physics& physics);
    void write(uint8_t channel, uint16_t pwm) override;
    void cork() override {}
    void push() override;
    void set_freq(uint16_t freq) override { _freq = freq; }

private:
    SITL_Physics& _physics;
    uint16_t _pwm[16];
    uint16_t _freq;
};

class SITL_Scheduler : public AP_HAL_Scheduler {
public:
    SITL_Scheduler();
    void delay(uint32_t ms) override;
    void delay_microseconds(uint32_t us) override;
    void register_timer_process(void (*proc)(void)) override {}
};

class SITL_Util : public AP_HAL_Util {
public:
    SITL_Util() {}
};

// Main SITL HAL class
class AP_HAL_SITL : public AP_HAL {
public:
    AP_HAL_SITL();
    ~AP_HAL_SITL();

    void init() override;
    uint64_t micros64() override;
    uint32_t millis() override;

    // Get physics simulator
    SITL_Physics& get_physics() { return *_physics; }

    // Set RC input
    void set_rc_input(uint8_t channel, uint16_t value);

private:
    SITL_Physics* _physics;
    SITL_InertialSensor* _sitl_ins;
    SITL_Baro* _sitl_baro;
    SITL_GPS* _sitl_gps;
    SITL_Compass* _sitl_compass;
    SITL_RCInput* _sitl_rcin;
    SITL_RCOutput* _sitl_rcout;
    SITL_Scheduler* _sitl_scheduler;
    SITL_Util* _sitl_util;

    struct timeval _start_time;
};
