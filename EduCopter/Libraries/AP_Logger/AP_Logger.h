#pragma once

#include <stdint.h>
#include <cstdio>
#include <string>

// Simplified logger for EduCopter
class AP_Logger {
public:
    AP_Logger();
    ~AP_Logger();

    void init();

    // Start/stop logging
    bool start_logging();
    void stop_logging();
    bool logging_enabled() const { return _logging_enabled; }

    // Log messages
    void log_attitude(uint64_t time_us, float roll, float pitch, float yaw);
    void log_rate(uint64_t time_us, float roll_rate, float pitch_rate, float yaw_rate);
    void log_imu(uint64_t time_us, float ax, float ay, float az, float gx, float gy, float gz);
    void log_control(uint64_t time_us, float roll_des, float pitch_des, float yaw_des, float throttle);
    void log_motors(uint64_t time_us, float m1, float m2, float m3, float m4);
    void log_position(uint64_t time_us, float x, float y, float z);
    void log_velocity(uint64_t time_us, float vx, float vy, float vz);
    void log_mode(uint64_t time_us, uint8_t mode);
    void log_event(uint64_t time_us, const char* event);

    // Get log filename
    const char* get_filename() const { return _filename.c_str(); }

private:
    FILE* _log_file;
    bool _logging_enabled;
    std::string _filename;
    uint32_t _log_counter;

    void write_header();
};
