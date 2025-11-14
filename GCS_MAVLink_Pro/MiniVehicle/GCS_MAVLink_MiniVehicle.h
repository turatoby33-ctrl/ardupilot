/*
 * GCS_MAVLink_MiniVehicle.h
 *
 * MAVLink channel class for MiniVehicle
 * Handles a single MAVLink connection (serial port)
 */

#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_MiniVehicle : public GCS_MAVLINK
{
public:
    // Inherit constructor from base class
    using GCS_MAVLINK::GCS_MAVLINK;

protected:
    // ========================================
    // REQUIRED PURE VIRTUAL FUNCTIONS
    // These MUST be implemented
    // ========================================

    // Return MAVLink base mode flags
    uint8_t base_mode() const override;

    // Return vehicle system status
    MAV_STATE vehicle_system_status() const override;

    // Send NAV_CONTROLLER_OUTPUT message
    void send_nav_controller_output() const override;

    // Send PID_TUNING message
    void send_pid_tuning() override;

    // ========================================
    // OPTIONAL VIRTUAL FUNCTIONS
    // Override these to customize behavior
    // ========================================

    // VFR_HUD message components
    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    // Handle incoming messages
    void handle_message(const mavlink_message_t &msg) override;

    // Try to send a specific message
    bool try_send_message(enum ap_message id) override;

    // Handle MAVLink commands
    MAV_RESULT handle_command_int_packet(
        const mavlink_command_int_t &packet,
        const mavlink_message_t &msg) override;

private:
    // ========================================
    // CUSTOM MESSAGE/COMMAND HANDLERS
    // Add your vehicle-specific handlers here
    // ========================================

    // Example custom command handler
    MAV_RESULT handle_cmd_do_test(const mavlink_command_int_t &packet);
};
