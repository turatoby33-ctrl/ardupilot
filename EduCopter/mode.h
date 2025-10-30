#pragma once

#include <stdint.h>

// Forward declarations
class EduCopter;

// Base class for all flight modes
class Mode {
public:
    Mode(EduCopter& copter);
    virtual ~Mode() {}

    // Mode number
    enum Number {
        STABILIZE = 0,
        ALT_HOLD = 2,
        LAND = 9,
    };

    // Initialize mode
    virtual bool init(bool ignore_checks) = 0;

    // Run mode (called at 400Hz)
    virtual void run() = 0;

    // Get mode number
    virtual Number mode_number() const = 0;

    // Get mode name
    virtual const char* name() const = 0;

    // Mode requires GPS
    virtual bool requires_GPS() const { return false; }

    // Mode allows arming
    virtual bool allows_arming() const { return true; }

protected:
    EduCopter& copter;

    // Helper functions
    float get_pilot_desired_throttle() const;
    void get_pilot_desired_lean_angles(float& roll_out, float& pitch_out, float angle_max) const;
    float get_pilot_desired_yaw_rate() const;
};
