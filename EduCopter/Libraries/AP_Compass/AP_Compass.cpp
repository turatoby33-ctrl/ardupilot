#include "AP_Compass.h"
#include "../AP_Math/AP_Math.h"
#include <cmath>
#include <cstdio>

extern AP_HAL* hal;

AP_Compass::AP_Compass() :
    heading(0.0f),
    declination(0.0f),
    calibrated(false),
    _healthy(false)
{
    field.zero();
    field_offset.zero();
}

void AP_Compass::init() {
    printf("AP_Compass: Initializing compass\n");
    _healthy = true;
}

bool AP_Compass::update() {
    if (!hal->compass) {
        return false;
    }

    // Get raw magnetic field
    float mx, my, mz;
    hal->compass->get_field(mx, my, mz);

    // Apply calibration
    field.x = mx - field_offset.x;
    field.y = my - field_offset.y;
    field.z = mz - field_offset.z;

    // Calculate heading (assuming level flight)
    // heading = atan2(-my, mx)
    heading = atan2f(-field.y, field.x);

    // Apply declination
    heading += declination;

    // Wrap to 0-2PI
    heading = AP_Math::wrap_2PI(heading);

    _healthy = true;
    return true;
}

void AP_Compass::calibrate() {
    printf("AP_Compass: Calibrating compass...\n");
    printf("AP_Compass: Rotate vehicle in all directions for 30 seconds\n");

    // Simple calibration - find min/max values
    Vector3f field_min(1000, 1000, 1000);
    Vector3f field_max(-1000, -1000, -1000);

    uint32_t start_ms = hal->millis();
    while (hal->millis() - start_ms < 30000) {
        float mx, my, mz;
        hal->compass->get_field(mx, my, mz);

        field_min.x = std::min(field_min.x, mx);
        field_min.y = std::min(field_min.y, my);
        field_min.z = std::min(field_min.z, mz);

        field_max.x = std::max(field_max.x, mx);
        field_max.y = std::max(field_max.y, my);
        field_max.z = std::max(field_max.z, mz);

        hal->scheduler->delay(100);

        if ((hal->millis() - start_ms) % 5000 == 0) {
            printf(".");
            fflush(stdout);
        }
    }

    // Calculate offsets (center of min/max sphere)
    field_offset.x = (field_max.x + field_min.x) / 2.0f;
    field_offset.y = (field_max.y + field_min.y) / 2.0f;
    field_offset.z = (field_max.z + field_min.z) / 2.0f;

    calibrated = true;

    printf("\nAP_Compass: Calibration complete\n");
    printf("AP_Compass: Offsets: %.2f, %.2f, %.2f\n",
           field_offset.x, field_offset.y, field_offset.z);
}
