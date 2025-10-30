#include "PID.h"
#include <cmath>
#include <algorithm>

namespace CustomCopter {

PID::PID()
    : integrator_(0.0f)
    , last_error_(0.0f)
    , last_derivative_(0.0f)
    , error_filtered_(0.0f)
    , derivative_filtered_(0.0f)
    , p_term_(0.0f)
    , i_term_(0.0f)
    , d_term_(0.0f)
    , ff_term_(0.0f)
    , first_run_(true)
{
    // Default gains (ArduPilot defaults for roll/pitch rate)
    gains_.kP = 0.135f;
    gains_.kI = 0.135f;
    gains_.kD = 0.0036f;
    gains_.kFF = 0.0f;
    gains_.imax = 0.5f;
    gains_.filt_T_hz = 20.0f;
    gains_.filt_E_hz = 0.0f;
    gains_.filt_D_hz = 20.0f;
}

PID::PID(const Gains& gains)
    : gains_(gains)
    , integrator_(0.0f)
    , last_error_(0.0f)
    , last_derivative_(0.0f)
    , error_filtered_(0.0f)
    , derivative_filtered_(0.0f)
    , p_term_(0.0f)
    , i_term_(0.0f)
    , d_term_(0.0f)
    , ff_term_(0.0f)
    , first_run_(true)
{
}

void PID::set_gains(const Gains& gains) {
    gains_ = gains;
}

float PID::update(float target, float measurement, float dt) {
    // Calculate error
    float error = target - measurement;

    // Apply error filter if configured
    if (gains_.filt_E_hz > 0.0f) {
        error = apply_filter(error, error_filtered_, gains_.filt_E_hz, dt);
    }

    // Proportional term
    p_term_ = gains_.kP * error;

    // Integral term
    integrator_ += gains_.kI * error * dt;
    // Anti-windup: Clamp integrator
    integrator_ = constrain_float(integrator_, -gains_.imax, gains_.imax);
    i_term_ = integrator_;

    // Derivative term
    float derivative;
    if (first_run_) {
        derivative = 0.0f;
        first_run_ = false;
    } else {
        derivative = (error - last_error_) / dt;
    }

    // Apply derivative filter
    if (gains_.filt_D_hz > 0.0f) {
        derivative = apply_filter(derivative, derivative_filtered_, gains_.filt_D_hz, dt);
    }

    d_term_ = gains_.kD * derivative;

    // Feed-forward term
    ff_term_ = gains_.kFF * target;

    // Store for next iteration
    last_error_ = error;
    last_derivative_ = derivative;

    // Return total output
    return p_term_ + i_term_ + d_term_ + ff_term_;
}

float PID::update_with_derivative(float target, float measurement,
                                  float derivative, float dt) {
    // Calculate error
    float error = target - measurement;

    // Apply error filter if configured
    if (gains_.filt_E_hz > 0.0f) {
        error = apply_filter(error, error_filtered_, gains_.filt_E_hz, dt);
    }

    // Proportional term
    p_term_ = gains_.kP * error;

    // Integral term
    integrator_ += gains_.kI * error * dt;
    // Anti-windup: Clamp integrator
    integrator_ = constrain_float(integrator_, -gains_.imax, gains_.imax);
    i_term_ = integrator_;

    // Derivative term (using provided derivative)
    // Apply derivative filter
    if (gains_.filt_D_hz > 0.0f) {
        derivative = apply_filter(derivative, derivative_filtered_, gains_.filt_D_hz, dt);
    }

    d_term_ = gains_.kD * derivative;

    // Feed-forward term
    ff_term_ = gains_.kFF * target;

    // Store for next iteration
    last_error_ = error;
    last_derivative_ = derivative;
    first_run_ = false;

    // Return total output
    return p_term_ + i_term_ + d_term_ + ff_term_;
}

void PID::reset_I() {
    integrator_ = 0.0f;
    i_term_ = 0.0f;
}

void PID::reset() {
    integrator_ = 0.0f;
    last_error_ = 0.0f;
    last_derivative_ = 0.0f;
    error_filtered_ = 0.0f;
    derivative_filtered_ = 0.0f;
    p_term_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
    ff_term_ = 0.0f;
    first_run_ = true;
}

void PID::set_integrator(float i) {
    integrator_ = constrain_float(i, -gains_.imax, gains_.imax);
    i_term_ = integrator_;
}

float PID::apply_filter(float input, float& state, float cutoff_hz, float dt) {
    if (cutoff_hz <= 0.0f || dt <= 0.0f) {
        return input;
    }

    // First-order low-pass filter
    // Alpha = dt / (dt + 1/(2*pi*fc))
    float rc = 1.0f / (2.0f * M_PI * cutoff_hz);
    float alpha = dt / (dt + rc);

    state = state + alpha * (input - state);
    return state;
}

} // namespace CustomCopter
