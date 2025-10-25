#pragma once

#include "../AP_HAL/AP_HAL.h"
#include "../AP_Math/vector3.h"

class AP_GPS {
public:
    enum GPS_Status {
        NO_GPS = 0,
        NO_FIX = 1,
        GPS_OK_FIX_2D = 2,
        GPS_OK_FIX_3D = 3
    };

    struct GPS_State {
        GPS_Status status;
        float latitude;          // degrees
        float longitude;         // degrees
        float altitude;          // meters (MSL)
        float ground_speed;      // m/s
        float ground_course;     // degrees
        uint8_t num_sats;
        float hdop;             // Horizontal dilution of precision
        uint64_t last_fix_time_us;
    };

    AP_GPS();

    void init();
    bool update();

    // Get GPS state
    const GPS_State& get_state() const { return state; }
    GPS_Status status() const { return state.status; }

    // Position
    float get_latitude() const { return state.latitude; }
    float get_longitude() const { return state.longitude; }
    float get_altitude() const { return state.altitude; }

    // Velocity
    float get_ground_speed() const { return state.ground_speed; }
    float get_ground_course() const { return state.ground_course; }
    Vector3f get_velocity_ned() const { return velocity_ned; }

    // Quality
    uint8_t get_num_sats() const { return state.num_sats; }
    bool have_fix() const { return state.status >= GPS_OK_FIX_3D; }

    // Health check
    bool healthy() const { return _healthy && (hal->millis() - state.last_fix_time_us/1000) < 1000; }

private:
    GPS_State state;
    Vector3f velocity_ned;  // NED velocity
    bool _healthy;
};
