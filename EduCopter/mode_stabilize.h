#pragma once

#include "mode.h"

// Stabilize mode - manual attitude control with manual throttle
class ModeStabilize : public Mode {
public:
    ModeStabilize(EduCopter& copter) : Mode(copter) {}

    bool init(bool ignore_checks) override;
    void run() override;

    Number mode_number() const override { return STABILIZE; }
    const char* name() const override { return "STABILIZE"; }

private:
    // No additional state needed
};
