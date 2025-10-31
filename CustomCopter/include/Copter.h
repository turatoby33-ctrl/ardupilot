#pragma once

#include "DataTypes.h"
#include "HAL.h"
#include "Scheduler.h"
#include "IMU.h"
#include "AttitudeControl.h"
#include "Motors.h"
#include "Mode.h"
#include "ModeStabilize.h"
#include <memory>
#include <cstdint>

namespace CustomCopter {

// ============================================================================
// Main Copter Class
// Based on ArduCopter Copter class
// This is the main flight controller class that ties everything together
// ============================================================================
class Copter {
public:
    Copter();
    ~Copter() = default;

    // ========================================================================
    // Initialization
    // ========================================================================

    // Initialize copter (call once at startup)
    bool init();

    // ========================================================================
    // Main Loop
    // ========================================================================

    // Main loop (runs continuously)
    void loop();

    // Fast loop (runs at 400 Hz)
    void fast_loop();

    // ========================================================================
    // System Access (for modes)
    // ========================================================================

    AttitudeControl* get_attitude_control() { return &attitude_control_; }
    Motors* get_motors() { return &motors_; }
    IMU* get_imu() { return imu_.get(); }
    Scheduler* get_scheduler() { return &scheduler_; }

    const Attitude& get_attitude() const { return attitude_; }
    const Vector3f& get_gyro_rates() const { return gyro_rates_; }

    // ========================================================================
    // Mode Management
    // ========================================================================

    // Set flight mode
    bool set_mode(FlightMode mode, bool ignore_checks = false);

    // Get current mode
    Mode* get_current_mode() { return current_mode_; }
    FlightMode get_current_mode_number() const { return current_mode_number_; }

    // ========================================================================
    // Arming
    // ========================================================================

    void arm() { armed_ = true; }
    void disarm() { armed_ = false; }
    bool is_armed() const { return armed_; }

    // ========================================================================
    // Status
    // ========================================================================

    bool is_healthy() const;

private:
    // ========================================================================
    // Initialization Helpers
    // ========================================================================

    bool init_hardware();
    bool init_sensors();
    bool init_attitude_control();
    bool init_motors();
    bool init_modes();
    void setup_scheduler();

    // ========================================================================
    // Loop Functions
    // ========================================================================

    void update_sensors();
    void update_attitude();
    void run_mode();

    // ========================================================================
    // Core Systems
    // ========================================================================

    // Scheduler (400 Hz main loop)
    Scheduler scheduler_;

    // Sensors
    std::unique_ptr<IMU> imu_;

    // Control systems
    AttitudeControl attitude_control_;
    Motors motors_;

    // HAL (Hardware Abstraction Layer)
    // TODO: Initialize actual HAL based on platform

    // ========================================================================
    // Flight Modes
    // ========================================================================

    Mode* current_mode_;
    FlightMode current_mode_number_;

    // Mode instances
    ModeStabilize mode_stabilize_;
    // TODO: Add other modes (ALT_HOLD, LOITER, etc.)

    // ========================================================================
    // State
    // ========================================================================

    // Vehicle attitude (roll, pitch, yaw)
    Attitude attitude_;

    // Gyro rates (rad/s)
    Vector3f gyro_rates_;

    // Arming state
    bool armed_;

    // Initialization state
    bool initialized_;
};

} // namespace CustomCopter
