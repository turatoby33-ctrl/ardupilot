#pragma once

#include "Libraries/AP_HAL/AP_HAL.h"
#include "Libraries/AP_Math/AP_Math.h"
#include "Libraries/AP_Param/AP_Param.h"
#include "Libraries/AP_Scheduler/AP_Scheduler.h"
#include "Libraries/AP_InertialSensor/AP_InertialSensor.h"
#include "Libraries/AP_Baro/AP_Baro.h"
#include "Libraries/AP_GPS/AP_GPS.h"
#include "Libraries/AP_Compass/AP_Compass.h"
#include "Libraries/AP_AHRS/AP_AHRS.h"
#include "Libraries/AC_PID/AC_PID.h"
#include "Libraries/AC_AttitudeControl/AC_AttitudeControl.h"
#include "Libraries/AP_Motors/AP_Motors.h"
#include "Libraries/AP_Arming/AP_Arming.h"
#include "Libraries/GCS_MAVLink/GCS_MAVLink.h"
#include "Libraries/AP_Logger/AP_Logger.h"

#include "mode.h"
#include "mode_stabilize.h"
#include "mode_althold.h"
#include "mode_land.h"

#include "config.h"

// Main copter class
class EduCopter {
public:
    friend class Mode;
    friend class ModeStabilize;
    friend class ModeAltHold;
    friend class ModeLand;

    EduCopter();

    // Initialization
    void init();
    void init_ardupilot();

    // Main loop
    void loop();
    void fast_loop();
    void rc_loop();
    void throttle_loop();
    void update_baro();
    void update_gps();
    void update_compass();
    void update_ahrs();

    // Flight mode management
    bool set_mode(Mode::Number mode, uint8_t reason);
    Mode* get_mode() { return flightmode; }

    // RC input
    void read_radio();
    uint16_t get_pilot_throttle() const { return pilot_throttle; }

    // Motor output
    void motors_output();

    // Arming
    bool arm();
    void disarm();
    bool is_armed() const { return motors.is_armed(); }

    // Sensor access
    AP_InertialSensor& get_ins() { return ins; }
    AP_Baro& get_baro() { return barometer; }
    AP_GPS& get_gps() { return gps; }
    AP_Compass& get_compass() { return compass; }
    AP_AHRS& get_ahrs() { return ahrs; }

    // Control access
    AC_AttitudeControl& get_attitude_control() { return attitude_control; }
    AP_Motors& get_motors() { return motors; }

private:
    // Sensors
    AP_InertialSensor ins;
    AP_Baro barometer;
    AP_GPS gps;
    AP_Compass compass;

    // State estimation
    AP_AHRS ahrs;

    // Control
    AC_AttitudeControl attitude_control;
    AP_Motors motors;

    // Systems
    AP_Arming arming_check;
    GCS_MAVLink mavlink;
    AP_Logger logger;
    AP_Scheduler scheduler;

    // Flight modes
    Mode* flightmode;
    ModeStabilize mode_stabilize;
    ModeAltHold mode_althold;
    ModeLand mode_land;

    // RC inputs (1000-2000 PWM)
    uint16_t pilot_throttle;
    uint16_t pilot_roll;
    uint16_t pilot_pitch;
    uint16_t pilot_yaw;

    // Parameters
    AP_ParamFloat angle_max;  // Maximum lean angle (degrees)

    // Timing
    uint64_t fast_loop_last_us;
    uint32_t fast_loop_count;
};

// Global instance
extern EduCopter copter;
