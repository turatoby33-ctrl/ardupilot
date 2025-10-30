#pragma once

#include "mode.h"

// Land mode - autonomous landing
class ModeLand : public Mode {
public:
    ModeLand(EduCopter& copter) : Mode(copter) {}

    bool init(bool ignore_checks) override;
    void run() override;

    Number mode_number() const override { return LAND; }
    const char* name() const override { return "LAND"; }

    bool allows_arming() const override { return false; }  // Can't arm in land mode

private:
    enum LandState {
        LAND_STATE_DESCEND,
        LAND_STATE_FINAL,
        LAND_STATE_COMPLETE
    };

    LandState land_state;
    float target_climb_rate;
    uint64_t land_start_time_us;
    uint64_t land_complete_time_us;

    void update_land_state();
};
