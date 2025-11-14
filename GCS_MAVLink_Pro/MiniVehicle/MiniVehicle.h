/*
 * MiniVehicle.h
 *
 * Main vehicle class definition for MiniVehicle
 * This is the vehicle's core class that ties everything together
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include "GCS_MiniVehicle.h"

class MiniVehicle
{
    friend class GCS_MAVLINK_MiniVehicle;
    friend class GCS_MiniVehicle;

public:
    MiniVehicle();

    // Main vehicle functions
    void setup();
    void loop();

    // ========================================
    // VEHICLE MODES
    // ========================================

    enum class Mode : uint8_t {
        MANUAL = 0,
        AUTO = 1,
        GUIDED = 2,
        HOLD = 3,
        NUM_MODES
    };

    enum class ModeReason : uint8_t {
        UNKNOWN = 0,
        GCS_COMMAND = 1,
        RC_COMMAND = 2,
        FAILSAFE = 3,
        AUTO = 4
    };

    Mode control_mode = Mode::MANUAL;

    // ========================================
    // VEHICLE STATE FLAGS
    // ========================================

    /*
     * The 'ap' structure holds various state flags
     * This pattern is used throughout ArduPilot vehicles
     */
    struct {
        bool initialised : 1;           // Vehicle has completed initialization
        bool armed : 1;                 // Motors are armed
        bool failsafe_crash_check : 1;  // Failsafe condition active
    } ap;

    // ========================================
    // NAVIGATION CONTROLLER
    // ========================================

    /*
     * Navigation controller provides navigation info
     * In a real vehicle, this would be a full controller
     * For this minimal version, we provide simple stubs
     */
    struct NavController {
        int16_t nav_bearing() const { return _nav_bearing; }
        int16_t target_bearing() const { return _target_bearing; }
        uint16_t wp_distance() const { return _wp_distance; }
        float crosstrack_error() const { return _crosstrack_error; }

        // Simple setters for testing
        void set_nav_bearing(int16_t bearing) { _nav_bearing = bearing; }
        void set_target_bearing(int16_t bearing) { _target_bearing = bearing; }
        void set_wp_distance(uint16_t distance) { _wp_distance = distance; }
        void set_crosstrack_error(float error) { _crosstrack_error = error; }

    private:
        int16_t _nav_bearing = 0;
        int16_t _target_bearing = 0;
        uint16_t _wp_distance = 0;
        float _crosstrack_error = 0.0f;
    } nav_controller;

    // ========================================
    // AHRS (Attitude Heading Reference System)
    // ========================================

    /*
     * AHRS provides attitude, position, and velocity
     * We use the real AP_AHRS object
     */
    AP_AHRS ahrs;

    // ========================================
    // GCS ACCESS
    // ========================================

    /*
     * GCS instance for MAVLink communication
     * This is the critical connection to ground control
     */
    GCS_MiniVehicle gcs_instance;
    GCS_MiniVehicle &gcs() { return gcs_instance; }

    // ========================================
    // PUBLIC INTERFACE
    // ========================================

    // System info
    const char* get_vehicle_name() const;
    uint32_t get_vehicle_version() const;

    // State queries
    bool is_armed() const;
    bool in_failsafe() const;

    // Arm/disarm
    bool arm(bool force = false);
    bool disarm(bool force = false);

    // Mode management
    bool set_mode(Mode mode, ModeReason reason);
    Mode get_mode() const;

    // System control
    void reboot(bool hold_in_bootloader = false);
    void print_status();

    // ========================================
    // PARAMETER TABLE
    // ========================================

    static const struct AP_Param::GroupInfo var_info[];

private:
    // ========================================
    // PRIVATE METHODS
    // ========================================

    // Initialization steps
    void init_ardupilot();
    void startup_ground();

    // Main loop functions
    void fast_loop();      // High-frequency control (50-400 Hz)
    void slow_loop();      // Low-frequency updates (1-10 Hz)
    void update_gcs();     // GCS communication updates

    // State management
    void check_failsafes();

    // ========================================
    // TIMING
    // ========================================

    uint32_t _fast_loop_timer_ms = 0;
    uint32_t _slow_loop_timer_ms = 0;

    // Loop rates
    static constexpr uint32_t FAST_LOOP_RATE_HZ = 50;    // 50 Hz
    static constexpr uint32_t SLOW_LOOP_RATE_HZ = 10;    // 10 Hz

    static constexpr uint32_t FAST_LOOP_PERIOD_MS = 1000 / FAST_LOOP_RATE_HZ;
    static constexpr uint32_t SLOW_LOOP_PERIOD_MS = 1000 / SLOW_LOOP_RATE_HZ;
};

// ========================================
// GLOBAL INSTANCE
// ========================================

/*
 * Global vehicle instance
 * This pattern is used by all ArduPilot vehicles
 */
extern MiniVehicle minivehicle;

/*
 * Global GCS access function
 * Allows libraries to access GCS without knowing vehicle type
 */
inline GCS &gcs() { return minivehicle.gcs(); }
