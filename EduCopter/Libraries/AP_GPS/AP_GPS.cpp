#include "AP_GPS.h"
#include "../AP_Math/AP_Math.h"
#include <cstdio>
#include <cmath>

extern AP_HAL* hal;

AP_GPS::AP_GPS() :
    _healthy(false)
{
    state.status = NO_GPS;
    state.latitude = 0.0f;
    state.longitude = 0.0f;
    state.altitude = 0.0f;
    state.ground_speed = 0.0f;
    state.ground_course = 0.0f;
    state.num_sats = 0;
    state.hdop = 99.9f;
    state.last_fix_time_us = 0;

    velocity_ned.zero();
}

void AP_GPS::init() {
    printf("AP_GPS: Initializing GPS\n");
    _healthy = true;
}

bool AP_GPS::update() {
    if (!hal->gps) {
        return false;
    }

    // Get data from HAL
    auto hal_status = hal->gps->status();
    state.status = static_cast<GPS_Status>(hal_status);

    if (state.status >= GPS_OK_FIX_2D) {
        state.latitude = hal->gps->get_latitude();
        state.longitude = hal->gps->get_longitude();
        state.altitude = hal->gps->get_altitude();
        state.ground_speed = hal->gps->get_ground_speed();
        state.ground_course = hal->gps->get_ground_course();
        state.num_sats = hal->gps->get_num_sats();
        state.last_fix_time_us = hal->micros64();

        // Convert ground speed and course to NED velocity
        float course_rad = state.ground_course * DEG_TO_RAD;
        velocity_ned.x = state.ground_speed * cosf(course_rad);  // North
        velocity_ned.y = state.ground_speed * sinf(course_rad);  // East
        velocity_ned.z = 0.0f;  // Down (we don't have vertical speed from basic GPS)

        _healthy = true;
    } else {
        _healthy = false;
    }

    return _healthy;
}
