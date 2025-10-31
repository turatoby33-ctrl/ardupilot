#include "Copter.h"
#include "IMU.h"
#include <iostream>

namespace CustomCopter {

Copter::Copter()
    : scheduler_()
    , imu_(nullptr)
    , attitude_control_()
    , motors_()
    , current_mode_(nullptr)
    , current_mode_number_(FlightMode::STABILIZE)
    , mode_stabilize_()
    , attitude_()
    , gyro_rates_()
    , armed_(false)
    , initialized_(false)
{
}

bool Copter::init() {
    std::cout << "CustomCopter: Initializing..." << std::endl;

    // Initialize scheduler first (provides timing for everything else)
    scheduler_.init();
    scheduler_.set_loop_rate_hz(400);  // 400 Hz main loop

    // Initialize hardware
    if (!init_hardware()) {
        std::cerr << "ERROR: Hardware initialization failed" << std::endl;
        return false;
    }

    // Initialize sensors
    if (!init_sensors()) {
        std::cerr << "ERROR: Sensor initialization failed" << std::endl;
        return false;
    }

    // Initialize attitude control
    if (!init_attitude_control()) {
        std::cerr << "ERROR: Attitude control initialization failed" << std::endl;
        return false;
    }

    // Initialize motors
    if (!init_motors()) {
        std::cerr << "ERROR: Motor initialization failed" << std::endl;
        return false;
    }

    // Initialize flight modes
    if (!init_modes()) {
        std::cerr << "ERROR: Mode initialization failed" << std::endl;
        return false;
    }

    // Set initial mode to STABILIZE
    if (!set_mode(FlightMode::STABILIZE, true)) {
        std::cerr << "ERROR: Failed to set initial mode" << std::endl;
        return false;
    }

    // Setup scheduler tasks
    setup_scheduler();

    initialized_ = true;
    std::cout << "CustomCopter: Initialization complete" << std::endl;
    return true;
}

bool Copter::init_hardware() {
    // TODO: Initialize HAL based on platform
    // For now, we'll use simulated hardware
    std::cout << "  Initializing hardware..." << std::endl;
    return true;
}

bool Copter::init_sensors() {
    std::cout << "  Initializing sensors..." << std::endl;

    // Create simulated IMU for testing
    imu_ = std::make_unique<IMU_Simulated>();

    if (!imu_->init(1000)) {  // 1000 Hz sample rate
        return false;
    }

    std::cout << "    IMU initialized (simulated)" << std::endl;
    return true;
}

bool Copter::init_attitude_control() {
    std::cout << "  Initializing attitude control..." << std::endl;

    // Initialize attitude controller
    attitude_control_.init();

    // Set default dt (400 Hz loop = 2.5ms = 0.0025s)
    attitude_control_.set_dt(1.0f / 400.0f);

    // Connect motors to attitude controller
    attitude_control_.set_motors(&motors_);

    std::cout << "    Attitude control initialized" << std::endl;
    return true;
}

bool Copter::init_motors() {
    std::cout << "  Initializing motors..." << std::endl;

    // Set frame type (Quad X default)
    motors_.set_frame_type(FrameType::QUAD_X);

    // TODO: Initialize with HAL PWM interface
    // For now, motors will work without actual PWM output
    // motors_.init(hal.get_pwm());

    motors_.enable_output(true);

    std::cout << "    Motors initialized (Quad X frame)" << std::endl;
    return true;
}

bool Copter::init_modes() {
    std::cout << "  Initializing flight modes..." << std::endl;

    // Set copter reference for all modes
    mode_stabilize_.set_copter(this);

    // Initialize modes
    mode_stabilize_.init(true);

    std::cout << "    STABILIZE mode initialized" << std::endl;
    return true;
}

void Copter::setup_scheduler() {
    // Add tasks to scheduler
    // Tasks are scheduled at different rates for efficiency

    // FAST_TASK (400 Hz) - Critical control loop
    scheduler_.add_task(
        [this]() { this->fast_loop(); },
        "fast_loop",
        0,      // 0 = run every loop (FAST_TASK)
        2000,   // 2000 us = 2ms expected max time
        0       // Priority 0 (highest)
    );

    // TODO: Add other tasks:
    // - RC input read (50 Hz)
    // - GPS update (10 Hz)
    // - Compass update (10 Hz)
    // - Barometer update (20 Hz)
    // - Battery monitor (10 Hz)
    // - GCS telemetry (10 Hz)
}

void Copter::loop() {
    // Main loop - runs continuously
    while (true) {
        // Tick the scheduler - this runs all scheduled tasks
        scheduler_.tick();

        // Small delay to prevent CPU spinning
        scheduler_.delay_microseconds(100);
    }
}

void Copter::fast_loop() {
    // Fast loop runs at 400 Hz
    // This is the critical control loop

    // 1. Update sensors (read IMU)
    update_sensors();

    // 2. Update attitude estimate
    update_attitude();

    // 3. Run current flight mode
    run_mode();

    // 4. Motor output is handled by the mode
}

void Copter::update_sensors() {
    // Update IMU (gyro + accel)
    if (imu_) {
        imu_->update();

        // Get gyro data for rate controller
        gyro_rates_ = imu_->get_gyro();
    }
}

void Copter::update_attitude() {
    // TODO: This should use AHRS/EKF for proper attitude estimation
    // For now, we'll use a simple approach

    // In a real implementation, this would run an Extended Kalman Filter (EKF)
    // that fuses IMU, GPS, compass, and barometer data to estimate attitude,
    // position, and velocity.

    // For testing, we can use the simulated IMU's attitude directly
    if (auto* sim_imu = dynamic_cast<IMU_Simulated*>(imu_.get())) {
        // Simulated IMU provides attitude directly
        // In reality, we'd integrate gyro data and correct with accel/mag
        // attitude_ = sim_imu->get_attitude();  // TODO: Add this method
    }

    // Update attitude controller with current attitude and rates
    attitude_control_.set_attitude(attitude_);
    attitude_control_.set_gyro_rates(gyro_rates_);
}

void Copter::run_mode() {
    if (current_mode_) {
        current_mode_->run();
    }
}

bool Copter::set_mode(FlightMode mode, bool ignore_checks) {
    // Get the mode object
    Mode* new_mode = nullptr;

    switch (mode) {
        case FlightMode::STABILIZE:
            new_mode = &mode_stabilize_;
            break;

        // TODO: Add other modes
        default:
            std::cerr << "ERROR: Unsupported mode: "
                      << static_cast<int>(mode) << std::endl;
            return false;
    }

    if (!new_mode) {
        return false;
    }

    // Exit current mode
    if (current_mode_) {
        current_mode_->exit();
    }

    // Enter new mode
    if (!new_mode->enter()) {
        std::cerr << "ERROR: Failed to enter mode: "
                  << new_mode->name() << std::endl;
        return false;
    }

    // Update mode
    current_mode_ = new_mode;
    current_mode_number_ = mode;

    std::cout << "Mode changed to: " << new_mode->name() << std::endl;
    return true;
}

bool Copter::is_healthy() const {
    // Check if all systems are healthy
    bool healthy = true;

    if (!initialized_) {
        healthy = false;
    }

    if (imu_ && !imu_->is_healthy()) {
        healthy = false;
    }

    if (!motors_.is_healthy()) {
        healthy = false;
    }

    return healthy;
}

} // namespace CustomCopter
