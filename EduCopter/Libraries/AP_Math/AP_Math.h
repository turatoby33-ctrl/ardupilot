#pragma once

#include "vector3.h"
#include "quaternion.h"
#include <cmath>
#include <algorithm>

// Math utilities
namespace AP_Math {
    // Constrain value between min and max
    template<typename T>
    inline T constrain(T val, T min_val, T max_val) {
        return std::max(min_val, std::min(val, max_val));
    }

    // Wrap angle to -PI to PI
    inline float wrap_PI(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    // Wrap angle to 0 to 2*PI
    inline float wrap_2PI(float angle) {
        while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
        while (angle < 0.0f) angle += 2.0f * M_PI;
        return angle;
    }

    // Convert degrees to radians
    inline float radians(float deg) {
        return deg * (M_PI / 180.0f);
    }

    // Convert radians to degrees
    inline float degrees(float rad) {
        return rad * (180.0f / M_PI);
    }

    // Safe square root
    inline float safe_sqrt(float v) {
        return sqrtf(std::max(0.0f, v));
    }

    // Is value zero (with tolerance)
    inline bool is_zero(float v, float tolerance = 1e-6f) {
        return fabsf(v) < tolerance;
    }

    // Linear interpolation
    inline float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }

    // Low pass filter coefficient
    inline float calc_lowpass_alpha_dt(float dt, float cutoff_freq) {
        if (dt <= 0.0f || cutoff_freq <= 0.0f) {
            return 1.0f;
        }
        float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
        return dt / (dt + rc);
    }
}

// Constants
#define GRAVITY_MSS 9.80665f
#define EARTH_RADIUS 6378137.0f
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
