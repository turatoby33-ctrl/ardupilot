#include "AC_PID.h"
#include "../AP_Math/AP_Math.h"
#include <cmath>

AC_PID::AC_PID(float kp, float ki, float kd, float imax, float filt_hz) :
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _imax(imax),
    _filt_hz(filt_hz),
    _filt_alpha(1.0f),
    _integrator(0.0f),
    _derivative(0.0f),
    _error_last(0.0f),
    _p(0.0f),
    _i(0.0f),
    _d(0.0f)
{
    set_filt_hz(filt_hz);
}

void AC_PID::set_filt_hz(float filt_hz) {
    _filt_hz = filt_hz;
}

float AC_PID::update_all(float target, float measurement, float dt) {
    float error = target - measurement;
    return update_error(error, dt);
}

float AC_PID::update_error(float error, float dt) {
    // Calculate P term
    _p = update_p(error);

    // Calculate I term
    _i = update_i(error, dt);

    // Calculate D term
    _d = update_d(error, dt);

    // Return total
    return _p + _i + _d;
}

float AC_PID::update_p(float error) {
    _p = _kp * error;
    return _p;
}

float AC_PID::update_i(float error, float dt) {
    if (!AP_Math::is_zero(_ki) && dt > 0.0f) {
        // Integrate error
        _integrator += error * _ki * dt;

        // Constrain integrator
        _integrator = AP_Math::constrain(_integrator, -_imax, _imax);

        _i = _integrator;
    } else {
        _i = 0.0f;
    }

    return _i;
}

float AC_PID::update_d(float error, float dt) {
    if (!AP_Math::is_zero(_kd) && dt > 0.0f) {
        // Calculate derivative
        float derivative = (error - _error_last) / dt;

        // Apply low-pass filter
        float alpha = AP_Math::calc_lowpass_alpha_dt(dt, _filt_hz);
        _derivative = _derivative * (1.0f - alpha) + derivative * alpha;

        _d = _kd * _derivative;
    } else {
        _d = 0.0f;
    }

    _error_last = error;
    return _d;
}

void AC_PID::reset_I() {
    _integrator = 0.0f;
    _i = 0.0f;
}

void AC_PID::reset_filter() {
    _derivative = 0.0f;
    _error_last = 0.0f;
    _d = 0.0f;
}

void AC_PID::reset() {
    reset_I();
    reset_filter();
    _p = 0.0f;
}

// AC_PID_FF implementation
AC_PID_FF::AC_PID_FF(float kp, float ki, float kd, float kff, float imax, float filt_hz) :
    AC_PID(kp, ki, kd, imax, filt_hz),
    _kff(kff),
    _ff(0.0f)
{
}

float AC_PID_FF::update_all(float target, float measurement, float dt, float target_rate) {
    float error = target - measurement;

    // Calculate P term
    _p = update_p(error);

    // Calculate I term
    _i = update_i(error, dt);

    // Calculate D term
    _d = update_d(error, dt);

    // Calculate feed-forward term
    _ff = _kff * target_rate;

    // Return total
    return _p + _i + _d + _ff;
}
