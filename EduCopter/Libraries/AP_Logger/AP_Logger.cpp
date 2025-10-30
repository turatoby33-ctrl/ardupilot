#include "AP_Logger.h"
#include "../AP_HAL/AP_HAL.h"
#include <ctime>
#include <cstring>

extern AP_HAL* hal;

AP_Logger::AP_Logger() :
    _log_file(nullptr),
    _logging_enabled(false),
    _log_counter(0)
{
}

AP_Logger::~AP_Logger() {
    stop_logging();
}

void AP_Logger::init() {
    printf("AP_Logger: Initializing logger\n");
}

bool AP_Logger::start_logging() {
    if (_logging_enabled) {
        printf("AP_Logger: Already logging\n");
        return true;
    }

    // Generate filename with timestamp
    char filename[256];
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    snprintf(filename, sizeof(filename), "educopter_%04d%02d%02d_%02d%02d%02d.csv",
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    _filename = filename;

    // Open log file
    _log_file = fopen(_filename.c_str(), "w");
    if (!_log_file) {
        printf("AP_Logger: Failed to open log file %s\n", _filename.c_str());
        return false;
    }

    write_header();

    _logging_enabled = true;
    printf("AP_Logger: Started logging to %s\n", _filename.c_str());

    return true;
}

void AP_Logger::stop_logging() {
    if (!_logging_enabled) {
        return;
    }

    if (_log_file) {
        fclose(_log_file);
        _log_file = nullptr;
    }

    _logging_enabled = false;
    printf("AP_Logger: Stopped logging\n");
}

void AP_Logger::write_header() {
    if (!_log_file) {
        return;
    }

    fprintf(_log_file, "time_us,type,data\n");
    fflush(_log_file);
}

void AP_Logger::log_attitude(uint64_t time_us, float roll, float pitch, float yaw) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,ATT,%.6f,%.6f,%.6f\n", time_us, roll, pitch, yaw);
}

void AP_Logger::log_rate(uint64_t time_us, float roll_rate, float pitch_rate, float yaw_rate) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,RATE,%.6f,%.6f,%.6f\n", time_us, roll_rate, pitch_rate, yaw_rate);
}

void AP_Logger::log_imu(uint64_t time_us, float ax, float ay, float az, float gx, float gy, float gz) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,IMU,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            time_us, ax, ay, az, gx, gy, gz);
}

void AP_Logger::log_control(uint64_t time_us, float roll_des, float pitch_des, float yaw_des, float throttle) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,CTRL,%.6f,%.6f,%.6f,%.6f\n",
            time_us, roll_des, pitch_des, yaw_des, throttle);
}

void AP_Logger::log_motors(uint64_t time_us, float m1, float m2, float m3, float m4) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,MOT,%.6f,%.6f,%.6f,%.6f\n", time_us, m1, m2, m3, m4);
}

void AP_Logger::log_position(uint64_t time_us, float x, float y, float z) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,POS,%.6f,%.6f,%.6f\n", time_us, x, y, z);
}

void AP_Logger::log_velocity(uint64_t time_us, float vx, float vy, float vz) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,VEL,%.6f,%.6f,%.6f\n", time_us, vx, vy, vz);
}

void AP_Logger::log_mode(uint64_t time_us, uint8_t mode) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,MODE,%d\n", time_us, mode);
}

void AP_Logger::log_event(uint64_t time_us, const char* event) {
    if (!_logging_enabled || !_log_file) {
        return;
    }

    fprintf(_log_file, "%llu,EVENT,%s\n", time_us, event);
}
