#include "AP_HAL_SITL.h"
#include <unistd.h>
#include <cstdio>
#include <cstring>

// SITL InertialSensor implementation
SITL_InertialSensor::SITL_InertialSensor(SITL_Physics& physics) :
    _physics(physics)
{
    memset(_accel, 0, sizeof(_accel));
    memset(_gyro, 0, sizeof(_gyro));
}

bool SITL_InertialSensor::update() {
    _physics.get_imu(_accel[0], _accel[1], _accel[2], _gyro[0], _gyro[1], _gyro[2]);
    return true;
}

void SITL_InertialSensor::get_accel(float& ax, float& ay, float& az) {
    ax = _accel[0];
    ay = _accel[1];
    az = _accel[2];
}

void SITL_InertialSensor::get_gyro(float& gx, float& gy, float& gz) {
    gx = _gyro[0];
    gy = _gyro[1];
    gz = _gyro[2];
}

// SITL Barometer implementation
SITL_Baro::SITL_Baro(SITL_Physics& physics) :
    _physics(physics),
    _pressure(101325.0f),
    _temperature(20.0f),
    _altitude(0.0f)
{
}

bool SITL_Baro::update() {
    _physics.get_baro(_pressure, _temperature, _altitude);
    return true;
}

// SITL GPS implementation
SITL_GPS::SITL_GPS(SITL_Physics& physics) :
    _physics(physics),
    _status(GPS_OK_FIX_3D),
    _latitude(0.0f),
    _longitude(0.0f),
    _altitude(0.0f),
    _ground_speed(0.0f),
    _ground_course(0.0f),
    _num_sats(10)
{
}

bool SITL_GPS::update() {
    float vn, ve, vd;
    _physics.get_gps(_latitude, _longitude, _altitude, vn, ve, vd);

    // Calculate ground speed and course
    _ground_speed = sqrtf(vn * vn + ve * ve);
    _ground_course = atan2f(ve, vn) * 180.0f / M_PI;
    if (_ground_course < 0) {
        _ground_course += 360.0f;
    }

    return true;
}

// SITL Compass implementation
SITL_Compass::SITL_Compass(SITL_Physics& physics) :
    _physics(physics)
{
    memset(_field, 0, sizeof(_field));
}

bool SITL_Compass::update() {
    _physics.get_compass(_field[0], _field[1], _field[2]);
    return true;
}

void SITL_Compass::get_field(float& mx, float& my, float& mz) {
    mx = _field[0];
    my = _field[1];
    mz = _field[2];
}

// SITL RC Input implementation
SITL_RCInput::SITL_RCInput() {
    // Initialize to safe values (centered)
    for (int i = 0; i < 16; i++) {
        _channels[i] = 1500;
    }
    _channels[2] = 1000;  // Throttle low
}

bool SITL_RCInput::update() {
    return true;
}

uint16_t SITL_RCInput::read(uint8_t channel) {
    if (channel < 16) {
        return _channels[channel];
    }
    return 1500;
}

void SITL_RCInput::set_channel(uint8_t channel, uint16_t value) {
    if (channel < 16) {
        _channels[channel] = value;
    }
}

// SITL RC Output implementation
SITL_RCOutput::SITL_RCOutput(SITL_Physics& physics) :
    _physics(physics),
    _freq(400)
{
    for (int i = 0; i < 16; i++) {
        _pwm[i] = 1000;
    }
}

void SITL_RCOutput::write(uint8_t channel, uint16_t pwm) {
    if (channel < 16) {
        _pwm[channel] = pwm;
    }
}

void SITL_RCOutput::push() {
    // Send motor commands to physics simulator
    for (int i = 0; i < 4; i++) {
        _physics.set_motor_pwm(i, _pwm[i]);
    }
}

// SITL Scheduler implementation
SITL_Scheduler::SITL_Scheduler() {
}

void SITL_Scheduler::delay(uint32_t ms) {
    usleep(ms * 1000);
}

void SITL_Scheduler::delay_microseconds(uint32_t us) {
    usleep(us);
}

// Main SITL HAL implementation
AP_HAL_SITL::AP_HAL_SITL() {
    _physics = new SITL_Physics();

    _sitl_ins = new SITL_InertialSensor(*_physics);
    _sitl_baro = new SITL_Baro(*_physics);
    _sitl_gps = new SITL_GPS(*_physics);
    _sitl_compass = new SITL_Compass(*_physics);
    _sitl_rcin = new SITL_RCInput();
    _sitl_rcout = new SITL_RCOutput(*_physics);
    _sitl_scheduler = new SITL_Scheduler();
    _sitl_util = new SITL_Util();

    ins = _sitl_ins;
    baro = _sitl_baro;
    gps = _sitl_gps;
    compass = _sitl_compass;
    rcin = _sitl_rcin;
    rcout = _sitl_rcout;
    scheduler = _sitl_scheduler;
    util = _sitl_util;

    gettimeofday(&_start_time, nullptr);
}

AP_HAL_SITL::~AP_HAL_SITL() {
    delete _physics;
    delete _sitl_ins;
    delete _sitl_baro;
    delete _sitl_gps;
    delete _sitl_compass;
    delete _sitl_rcin;
    delete _sitl_rcout;
    delete _sitl_scheduler;
    delete _sitl_util;
}

void AP_HAL_SITL::init() {
    printf("SITL: Initializing SITL HAL\n");
}

uint64_t AP_HAL_SITL::micros64() {
    struct timeval now;
    gettimeofday(&now, nullptr);

    uint64_t since_start = (now.tv_sec - _start_time.tv_sec) * 1000000ULL +
                          (now.tv_usec - _start_time.tv_usec);

    return since_start;
}

uint32_t AP_HAL_SITL::millis() {
    return micros64() / 1000;
}

void AP_HAL_SITL::set_rc_input(uint8_t channel, uint16_t value) {
    _sitl_rcin->set_channel(channel, value);
}
