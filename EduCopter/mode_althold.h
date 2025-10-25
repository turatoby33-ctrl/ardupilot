#pragma once

#include "mode.h"

// Altitude Hold mode - manual attitude control with automatic altitude hold
class ModeAltHold : public Mode {
public:
    ModeAltHold(EduCopter& copter) : Mode(copter) {}

    bool init(bool ignore_checks) override;
    void run() override;

    Number mode_number() const override { return ALT_HOLD; }
    const char* name() const override { return "ALT_HOLD"; }

private:
    float target_altitude;
    float target_climb_rate;
    bool altitude_set;

    // Altitude controller
    float altitude_controller(float target_alt, float current_alt, float dt);
};
