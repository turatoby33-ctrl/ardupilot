#pragma once

// Configuration file for EduCopter

// Loop rates
#define MAIN_LOOP_RATE 400  // Hz
#define MAIN_LOOP_PERIOD_US (1000000 / MAIN_LOOP_RATE)

// RC channels
#define RC_CHANNEL_ROLL 0
#define RC_CHANNEL_PITCH 1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW 3

// RC limits
#define RC_MIN 1000
#define RC_MID 1500
#define RC_MAX 2000
#define RC_DEADZONE 20

// Motor limits
#define MOTOR_PWM_MIN 1000
#define MOTOR_PWM_MAX 2000

// Flight limits
#define ANGLE_MAX_DEFAULT 45.0f  // Maximum lean angle (degrees)
#define ALT_HOLD_THROTTLE_NEUTRAL 0.5f

// Landing parameters
#define LAND_SPEED 50.0f  // cm/s
#define LAND_DETECTOR_ACCEL_MAX 200.0f  // cm/s/s
#define LAND_DETECTOR_MAYBE_LANDED_TIME_MS 250
#define LAND_DETECTOR_LANDED_TIME_MS 1000

// Logging
#define LOG_ATTITUDE_RATE 10  // Hz
#define LOG_POSITION_RATE 5   // Hz
