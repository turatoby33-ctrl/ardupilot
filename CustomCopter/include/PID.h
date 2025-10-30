#pragma once

#include "DataTypes.h"

namespace CustomCopter {

// ============================================================================
// PID Controller
// Based on ArduPilot's AC_PID implementation
// ============================================================================
class PID {
public:
    // PID gains and limits
    struct Gains {
        float kP;           // Proportional gain
        float kI;           // Integral gain
        float kD;           // Derivative gain
        float kFF;          // Feed-forward gain
        float imax;         // Maximum integrator value
        float filt_T_hz;    // Target filter frequency (Hz)
        float filt_E_hz;    // Error filter frequency (Hz)
        float filt_D_hz;    // Derivative filter frequency (Hz)
    };

    PID();
    PID(const Gains& gains);

    // Set gains
    void set_gains(const Gains& gains);
    void set_kP(float kP) { gains_.kP = kP; }
    void set_kI(float kI) { gains_.kI = kI; }
    void set_kD(float kD) { gains_.kD = kD; }
    void set_kFF(float kFF) { gains_.kFF = kFF; }
    void set_imax(float imax) { gains_.imax = imax; }

    // Get gains
    const Gains& get_gains() const { return gains_; }
    float get_kP() const { return gains_.kP; }
    float get_kI() const { return gains_.kI; }
    float get_kD() const { return gains_.kD; }

    // Update PID controller
    // target: Desired value
    // measurement: Current value
    // dt: Time step in seconds
    // Returns: Control output
    float update(float target, float measurement, float dt);

    // Update with derivative measurement (for rate control)
    float update_with_derivative(float target, float measurement,
                                 float derivative, float dt);

    // Reset integrator
    void reset_I();

    // Reset entire PID state
    void reset();

    // Get individual terms
    float get_p() const { return p_term_; }
    float get_i() const { return i_term_; }
    float get_d() const { return d_term_; }
    float get_ff() const { return ff_term_; }

    // Set integrator value directly
    void set_integrator(float i);

    // Get integrator value
    float get_integrator() const { return integrator_; }

private:
    // Apply low-pass filter
    float apply_filter(float input, float& state, float cutoff_hz, float dt);

    Gains gains_;

    // Internal state
    float integrator_;
    float last_error_;
    float last_derivative_;
    float error_filtered_;
    float derivative_filtered_;

    // PID output terms
    float p_term_;
    float i_term_;
    float d_term_;
    float ff_term_;

    bool first_run_;
};

} // namespace CustomCopter
