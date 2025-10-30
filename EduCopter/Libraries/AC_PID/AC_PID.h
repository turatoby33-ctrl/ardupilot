#pragma once

#include "../AP_Param/AP_Param.h"
#include <stdint.h>

// PID controller class
class AC_PID {
public:
    AC_PID(float kp, float ki, float kd, float imax, float filt_hz);

    // Update PID controller
    float update_all(float target, float measurement, float dt);
    float update_error(float error, float dt);

    // Update individual components
    float update_p(float error);
    float update_i(float error, float dt);
    float update_d(float error, float dt);

    // Reset integrator and derivative
    void reset_I();
    void reset_filter();
    void reset();

    // Set gains
    void set_kp(float kp) { _kp = kp; }
    void set_ki(float ki) { _ki = ki; }
    void set_kd(float kd) { _kd = kd; }
    void set_imax(float imax) { _imax = imax; }
    void set_filt_hz(float filt_hz);

    // Get gains
    float get_kp() const { return _kp; }
    float get_ki() const { return _ki; }
    float get_kd() const { return _kd; }
    float get_imax() const { return _imax; }

    // Get components
    float get_p() const { return _p; }
    float get_i() const { return _i; }
    float get_d() const { return _d; }

    // Get integrator
    float get_integrator() const { return _integrator; }
    void set_integrator(float i) { _integrator = i; }

protected:
    // Gains
    float _kp;
    float _ki;
    float _kd;
    float _imax;  // Maximum integrator value

    // Filter
    float _filt_hz;
    float _filt_alpha;

    // State
    float _integrator;
    float _derivative;
    float _error_last;

    // Output components
    float _p;
    float _i;
    float _d;
};

// PID with feed-forward
class AC_PID_FF : public AC_PID {
public:
    AC_PID_FF(float kp, float ki, float kd, float kff, float imax, float filt_hz);

    // Update with feed-forward
    float update_all(float target, float measurement, float dt, float target_rate);

    // Set/get feed-forward gain
    void set_kff(float kff) { _kff = kff; }
    float get_kff() const { return _kff; }
    float get_ff() const { return _ff; }

private:
    float _kff;
    float _ff;
};
