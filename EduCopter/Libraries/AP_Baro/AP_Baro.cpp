#include "AP_Baro.h"
#include "../AP_Math/AP_Math.h"
#include <cmath>
#include <cstdio>

extern AP_HAL* hal;

AP_Baro::AP_Baro() :
    pressure(101325.0f),
    temperature(20.0f),
    altitude(0.0f),
    climb_rate(0.0f),
    ground_pressure(101325.0f),
    last_altitude(0.0f),
    last_update_us(0),
    _healthy(false)
{
}

void AP_Baro::init() {
    printf("AP_Baro: Initializing barometer\n");
    last_update_us = hal->micros64();
    _healthy = true;
}

bool AP_Baro::update() {
    if (!hal->baro) {
        return false;
    }

    // Get raw sensor data
    pressure = hal->baro->get_pressure();
    temperature = hal->baro->get_temperature();

    // Calculate altitude using barometric formula
    // h = 44330 * (1 - (P/P0)^0.1903)
    altitude = 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.1903f));

    // Calculate climb rate
    uint64_t now_us = hal->micros64();
    float dt = (now_us - last_update_us) * 1.0e-6f;

    if (dt > 0.001f) {
        climb_rate = (altitude - last_altitude) / dt;

        // Apply low-pass filter to climb rate
        float alpha = AP_Math::calc_lowpass_alpha_dt(dt, 2.0f); // 2Hz cutoff
        climb_rate = last_altitude == 0.0f ? 0.0f :
                     climb_rate * alpha + climb_rate * (1.0f - alpha);

        last_altitude = altitude;
        last_update_us = now_us;
    }

    _healthy = true;
    return true;
}

void AP_Baro::calibrate() {
    printf("AP_Baro: Calibrating (setting ground pressure)...\n");

    // Average pressure over 1 second
    float pressure_sum = 0.0f;
    int samples = 0;

    for (int i = 0; i < 100; i++) {
        if (hal->baro->update()) {
            pressure_sum += hal->baro->get_pressure();
            samples++;
        }
        hal->scheduler->delay(10);
    }

    if (samples > 0) {
        ground_pressure = pressure_sum / samples;
        printf("AP_Baro: Ground pressure set to %.2f Pa\n", ground_pressure);
    }
}

void AP_Baro::set_ground_pressure(float pressure_pa) {
    ground_pressure = pressure_pa;
}
