#include "EduCopter.h"
#include "Libraries/AP_HAL/AP_HAL.h"
#include <cstdio>

extern AP_HAL* hal;

// Global instance
EduCopter copter;

EduCopter::EduCopter() :
    ins(),
    barometer(),
    gps(),
    compass(),
    ahrs(ins, barometer, gps, compass),
    attitude_control(ahrs),
    motors(AP_Motors::FRAME_QUAD_X),
    arming_check(ins, barometer, gps, compass, ahrs),
    mavlink(ahrs, gps, barometer),
    logger(),
    scheduler(),
    flightmode(nullptr),
    mode_stabilize(*this),
    mode_althold(*this),
    mode_land(*this),
    pilot_throttle(1000),
    pilot_roll(1500),
    pilot_pitch(1500),
    pilot_yaw(1500),
    angle_max(ANGLE_MAX_DEFAULT, "ANGLE_MAX"),
    fast_loop_last_us(0),
    fast_loop_count(0)
{
}

void EduCopter::init() {
    printf("\n");
    printf("=========================================\n");
    printf("  EduCopter Flight Controller v1.0\n");
    printf("  Educational Quadcopter System\n");
    printf("=========================================\n\n");

    init_ardupilot();
}

void EduCopter::init_ardupilot() {
    // Initialize HAL first
    if (hal) {
        hal->init();
        printf("System: HAL initialized\n");
    }

    // Load parameters
    AP_Param::load_all();

    // Initialize sensors
    ins.init();
    barometer.init();
    gps.init();
    compass.init();

    // Calibrate sensors
    printf("\nSystem: Calibrating sensors...\n");
    ins.calibrate_gyro();
    ins.calibrate_accel();
    barometer.calibrate();

    // Initialize AHRS
    ahrs.init();

    // Initialize attitude controller
    attitude_control.init();

    // Initialize motors
    motors.init();
    motors.set_pwm_range(MOTOR_PWM_MIN, MOTOR_PWM_MAX);

    // Initialize arming checks
    arming_check.init();

    // Disable GPS check for indoor testing
    arming_check.set_check_enabled(AP_Arming::CHECK_GPS, false);

    // Initialize MAVLink
    mavlink.init();

    // Initialize logger
    logger.init();

    // Setup scheduler tasks
    scheduler.init();

    // Register scheduler tasks
    scheduler.register_task([this]() { this->fast_loop(); }, 400, 2000);
    scheduler.register_task([this]() { this->rc_loop(); }, 50, 500);
    scheduler.register_task([this]() { this->throttle_loop(); }, 50, 500);
    scheduler.register_task([this]() { this->update_baro(); }, 20, 1000);
    scheduler.register_task([this]() { this->update_gps(); }, 10, 1000);
    scheduler.register_task([this]() { this->update_compass(); }, 10, 1000);

    // Set initial mode to STABILIZE
    set_mode(Mode::STABILIZE, 0);

    printf("\nSystem: Initialization complete\n");
    printf("System: Ready to arm\n");
    printf("System: Use throttle low + yaw right to arm\n\n");

    // Start logging
    logger.start_logging();

    fast_loop_last_us = hal->micros64();
}

void EduCopter::loop() {
    // Run scheduler
    uint64_t time_available_us = MAIN_LOOP_PERIOD_US;
    scheduler.run(time_available_us);

    // Update MAVLink
    mavlink.update();

    // Maintain loop rate
    uint64_t now_us = hal->micros64();
    uint64_t elapsed_us = now_us - fast_loop_last_us;

    if (elapsed_us < MAIN_LOOP_PERIOD_US) {
        uint32_t delay_us = MAIN_LOOP_PERIOD_US - elapsed_us;
        hal->scheduler->delay_microseconds(delay_us);
    }

    fast_loop_last_us = hal->micros64();
}

void EduCopter::fast_loop() {
    // Update IMU
    ins.update();

    // Update AHRS
    update_ahrs();

    // Run flight mode
    if (flightmode != nullptr) {
        flightmode->run();
    }

    // Output to motors
    motors_output();

    // Logging (at reduced rate)
    if (fast_loop_count % (MAIN_LOOP_RATE / LOG_ATTITUDE_RATE) == 0) {
        uint64_t time_us = hal->micros64();
        logger.log_attitude(time_us, ahrs.get_roll(), ahrs.get_pitch(), ahrs.get_yaw());
        logger.log_rate(time_us, ahrs.get_gyro().x, ahrs.get_gyro().y, ahrs.get_gyro().z);

        const Vector3f& accel = ins.get_accel();
        const Vector3f& gyro = ins.get_gyro();
        logger.log_imu(time_us, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
    }

    if (fast_loop_count % (MAIN_LOOP_RATE / LOG_POSITION_RATE) == 0) {
        uint64_t time_us = hal->micros64();
        const Vector3f& pos = ahrs.get_position();
        logger.log_position(time_us, pos.x, pos.y, pos.z);

        const Vector3f& vel = ahrs.get_velocity();
        logger.log_velocity(time_us, vel.x, vel.y, vel.z);
    }

    fast_loop_count++;
}

void EduCopter::rc_loop() {
    read_radio();
}

void EduCopter::throttle_loop() {
    // Check for arming command (throttle low + yaw right)
    if (!is_armed()) {
        if (pilot_throttle < 1100 && pilot_yaw > 1800) {
            arm();
        }
    } else {
        // Check for disarm command (throttle low + yaw left)
        if (pilot_throttle < 1100 && pilot_yaw < 1200) {
            disarm();
        }
    }
}

void EduCopter::update_baro() {
    barometer.update();
}

void EduCopter::update_gps() {
    gps.update();
}

void EduCopter::update_compass() {
    compass.update();
}

void EduCopter::update_ahrs() {
    ahrs.update();
}

void EduCopter::read_radio() {
    if (!hal->rcin) {
        return;
    }

    hal->rcin->update();

    pilot_roll = hal->rcin->read(RC_CHANNEL_ROLL);
    pilot_pitch = hal->rcin->read(RC_CHANNEL_PITCH);
    pilot_throttle = hal->rcin->read(RC_CHANNEL_THROTTLE);
    pilot_yaw = hal->rcin->read(RC_CHANNEL_YAW);
}

void EduCopter::motors_output() {
    motors.output();
}

bool EduCopter::set_mode(Mode::Number mode_num, uint8_t reason) {
    Mode* new_mode = nullptr;

    switch (mode_num) {
        case Mode::STABILIZE:
            new_mode = &mode_stabilize;
            break;

        case Mode::ALT_HOLD:
            new_mode = &mode_althold;
            break;

        case Mode::LAND:
            new_mode = &mode_land;
            break;

        default:
            printf("Mode: Unknown mode %d\n", mode_num);
            return false;
    }

    // Initialize new mode
    if (new_mode && new_mode->init(false)) {
        flightmode = new_mode;
        logger.log_mode(hal->micros64(), mode_num);
        return true;
    }

    return false;
}

bool EduCopter::arm() {
    if (motors.is_armed()) {
        printf("Arming: Already armed\n");
        return false;
    }

    // Run arming checks
    if (!arming_check.arm(true)) {
        printf("Arming: Failed - %s\n",
               arming_check.get_result_string(arming_check.get_last_result()));
        return false;
    }

    // Arm motors
    motors.armed(true);

    printf("Arming: *** ARMED ***\n");
    logger.log_event(hal->micros64(), "ARMED");

    return true;
}

void EduCopter::disarm() {
    if (!motors.is_armed()) {
        return;
    }

    motors.armed(false);
    arming_check.disarm(false);

    printf("Arming: Disarmed\n");
    logger.log_event(hal->micros64(), "DISARMED");
}
