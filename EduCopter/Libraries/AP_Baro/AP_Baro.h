#pragma once

#include "../AP_HAL/AP_HAL.h"

class AP_Baro {
public:
    AP_Baro();

    void init();
    bool update();

    // Get measurements
    float get_pressure() const { return pressure; }      // Pascal
    float get_temperature() const { return temperature; } // Celsius
    float get_altitude() const { return altitude; }       // meters (relative to ground level)
    float get_climb_rate() const { return climb_rate; }   // m/s

    // Calibration
    void calibrate();
    void set_ground_pressure(float pressure_pa);

    // Health check
    bool healthy() const { return _healthy; }

private:
    float pressure;           // Current pressure (Pa)
    float temperature;        // Current temperature (C)
    float altitude;           // Altitude above ground (m)
    float climb_rate;         // Vertical velocity (m/s)

    float ground_pressure;    // Pressure at ground level (Pa)
    float last_altitude;      // For climb rate calculation
    uint64_t last_update_us;

    bool _healthy;
};
