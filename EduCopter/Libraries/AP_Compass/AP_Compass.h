#pragma once

#include "../AP_HAL/AP_HAL.h"
#include "../AP_Math/vector3.h"

class AP_Compass {
public:
    AP_Compass();

    void init();
    bool update();

    // Get magnetic field vector (milliGauss)
    const Vector3f& get_field() const { return field; }

    // Get heading (radians, 0 = North, clockwise)
    float get_heading() const { return heading; }

    // Declination (difference between magnetic and true north)
    float get_declination() const { return declination; }
    void set_declination(float dec_rad) { declination = dec_rad; }

    // Calibration
    void calibrate();
    bool is_calibrated() const { return calibrated; }

    // Health check
    bool healthy() const { return _healthy; }

private:
    Vector3f field;          // Magnetic field vector
    Vector3f field_offset;   // Calibration offsets
    float heading;           // Heading in radians
    float declination;       // Magnetic declination
    bool calibrated;
    bool _healthy;
};
