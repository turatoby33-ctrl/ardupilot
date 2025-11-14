/*
 * MiniVehicle.cpp
 *
 * Main vehicle implementation for MiniVehicle
 * Demonstrates the essential patterns for ArduPilot vehicle integration
 */

#include "MiniVehicle.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

// ========================================
// GLOBAL INSTANCE
// ========================================

MiniVehicle minivehicle;

// ========================================
// CONSTRUCTOR
// ========================================

MiniVehicle::MiniVehicle()
    : ahrs()
{
    // Initialize state flags
    ap.initialised = false;
    ap.armed = false;
    ap.failsafe_crash_check = false;
}

// ========================================
// SETUP - Called once at startup
// ========================================

void MiniVehicle::setup()
{
    // This is called once when the vehicle boots up
    hal.console->printf("\n\n");
    hal.console->printf("========================================\n");
    hal.console->printf("   MiniVehicle Starting Up\n");
    hal.console->printf("========================================\n\n");

    // Initialize ArduPilot core
    init_ardupilot();

    // Ground startup procedures
    startup_ground();

    // Mark as initialized
    ap.initialised = true;

    hal.console->printf("\n========================================\n");
    hal.console->printf("   MiniVehicle Ready!\n");
    hal.console->printf("========================================\n\n");

    // Send startup message to GCS
    gcs().send_text(MAV_SEVERITY_INFO, "MiniVehicle %u.%u.%u ready",
                   1, 0, 0);  // Version 1.0.0
}

// ========================================
// LOOP - Called repeatedly
// ========================================

void MiniVehicle::loop()
{
    /*
     * This is the main vehicle loop
     * Called as fast as possible by the scheduler
     *
     * We divide work into:
     * - Fast loop: High-frequency control (50 Hz typical)
     * - Slow loop: Low-frequency updates (10 Hz typical)
     * - GCS updates: MAVLink communication
     */

    uint32_t now_ms = AP_HAL::millis();

    // ========================================
    // FAST LOOP (50 Hz)
    // ========================================

    if (now_ms - _fast_loop_timer_ms >= FAST_LOOP_PERIOD_MS) {
        _fast_loop_timer_ms = now_ms;
        fast_loop();
    }

    // ========================================
    // SLOW LOOP (10 Hz)
    // ========================================

    if (now_ms - _slow_loop_timer_ms >= SLOW_LOOP_PERIOD_MS) {
        _slow_loop_timer_ms = now_ms;
        slow_loop();
    }

    // ========================================
    // GCS UPDATES - CRITICAL!
    // ========================================

    /*
     * INTEGRATION STEP 3: Call GCS functions in main loop
     *
     * These MUST be called regularly for MAVLink to work!
     */
    update_gcs();
}

// ========================================
// INITIALIZATION
// ========================================

void MiniVehicle::init_ardupilot()
{
    hal.console->printf(">>> Initializing ArduPilot Core\n");

    // Small delay for hardware to stabilize
    hal.scheduler->delay(100);

    // Initialize serial manager
    // This sets up all UART ports based on parameters
    hal.console->printf("  - Initializing serial ports\n");
    AP::serialmanager().init();

    // ========================================
    // INTEGRATION STEP 4: Initialize GCS
    // ========================================

    /*
     * This is CRITICAL for MAVLink communication!
     * Must be called in this order:
     */

    hal.console->printf("  - Initializing GCS\n");

    // 1. Initialize GCS system
    gcs().init();

    // 2. Setup console (USB/primary serial)
    gcs().setup_console();

    // 3. Setup telemetry UARTs
    gcs().setup_uarts();

    hal.console->printf("  - GCS initialized\n");

    // Initialize AHRS
    hal.console->printf("  - Initializing AHRS\n");
    ahrs.init();

    hal.console->printf("<<< ArduPilot Core initialized\n\n");
}

void MiniVehicle::startup_ground()
{
    hal.console->printf(">>> Ground Startup\n");

    // Set initial mode
    set_mode(Mode::MANUAL, ModeReason::AUTO);

    // Initialize navigation controller with test data
    nav_controller.set_nav_bearing(0);
    nav_controller.set_target_bearing(0);
    nav_controller.set_wp_distance(0);
    nav_controller.set_crosstrack_error(0.0f);

    hal.console->printf("<<< Ground Startup complete\n\n");
}

// ========================================
// MAIN LOOP FUNCTIONS
// ========================================

void MiniVehicle::fast_loop()
{
    /*
     * Fast loop runs at 50 Hz
     * Used for time-critical control:
     * - Reading sensors
     * - Running control loops
     * - Updating motors
     */

    // In a real vehicle, this would:
    // - Read IMU
    // - Run attitude controller
    // - Update motor outputs
    // - Check critical failsafes

    // For this minimal version, we just update AHRS
    ahrs.update();
}

void MiniVehicle::slow_loop()
{
    /*
     * Slow loop runs at 10 Hz
     * Used for less time-critical updates:
     * - GPS updates
     * - Battery monitoring
     * - Mode changes
     * - Logging
     */

    // Check for failsafe conditions
    check_failsafes();

    // In a real vehicle, this would also:
    // - Update GPS
    // - Update battery monitor
    // - Update compass
    // - Write logs
}

void MiniVehicle::update_gcs()
{
    /*
     * INTEGRATION STEP 3: Call GCS functions in main loop
     *
     * These functions handle all MAVLink communication:
     * - update_receive(): Parse incoming messages from GCS
     * - update_send(): Send periodic messages to GCS
     *
     * CRITICAL: Must be called regularly (ideally every loop iteration)
     */

    gcs().update_receive();  // Handle incoming MAVLink messages
    gcs().update_send();     // Send outgoing MAVLink messages
}

// ========================================
// STATE MANAGEMENT
// ========================================

void MiniVehicle::set_mode(Mode mode)
{
    // Validate mode change
    // In a real vehicle, check if mode is allowed

    Mode old_mode = control_mode;
    control_mode = mode;

    // Log mode change
    const char* mode_str[] = {"MANUAL", "AUTO", "GUIDED", "HOLD"};
    hal.console->printf("Mode: %s -> %s\n",
                       mode_str[(uint8_t)old_mode],
                       mode_str[(uint8_t)mode]);

    // Send to GCS
    gcs().send_text(MAV_SEVERITY_INFO, "Mode: %s", mode_str[(uint8_t)mode]);
}

void MiniVehicle::check_failsafes()
{
    /*
     * Check for failsafe conditions
     * In a real vehicle, this would check:
     * - Radio signal loss
     * - Battery voltage
     * - GPS loss
     * - Geofence breach
     * - EKF errors
     */

    // For this minimal version, just clear the flag
    ap.failsafe_crash_check = false;

    // In a real vehicle:
    // if (radio_failsafe || battery_failsafe || gps_failsafe) {
    //     ap.failsafe_crash_check = true;
    //     gcs().send_text(MAV_SEVERITY_CRITICAL, "FAILSAFE!");
    // }
}
