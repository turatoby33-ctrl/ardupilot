/*
 * Minimal Standalone GCS_Common Implementation
 * Version 1: HEARTBEAT Send/Receive Only
 *
 * This file contains the common functionality shared across all GCS channels.
 * In the full ArduPilot, GCS_Common.cpp is massive (~83,000 tokens).
 * This minimal version shows ONLY the HEARTBEAT functionality.
 */

#include "minimal_GCS.h"

/*
 * ============================================
 * HEARTBEAT Message Handling
 * ============================================
 *
 * The HEARTBEAT message (ID 0) is the most fundamental MAVLink message.
 * It's sent by both the vehicle and the GCS to indicate:
 * - System is alive and operational
 * - Current system state (armed, mode, health)
 * - Vehicle type and autopilot type
 *
 * HEARTBEAT must be sent at minimum 1 Hz to maintain connection.
 */

// This demonstrates the pattern used in real GCS_Common.cpp
// In the real file, handle_heartbeat is part of GCS_MAVLINK class

void GCS_MAVLINK::handle_heartbeat(const mavlink_message_t &msg)
{
    // Decode the HEARTBEAT payload
    mavlink_heartbeat_t heartbeat;
    memcpy(&heartbeat, msg.payload, sizeof(heartbeat));

    print_message("=== HEARTBEAT RECEIVED ===");
    print_message("  From: sysid=%u compid=%u", msg.sysid, msg.compid);
    print_message("  Type: %u (0=Generic, 2=Quad, 4=Heli, 6+=GCS)", heartbeat.type);
    print_message("  Autopilot: %u (0=Generic, 3=ArduPilot)", heartbeat.autopilot);
    print_message("  Base Mode: 0x%02X", heartbeat.base_mode);
    print_message("    - Custom Mode Enabled: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) ? "Yes" : "No");
    print_message("    - Test Enabled: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_TEST_ENABLED) ? "Yes" : "No");
    print_message("    - Auto Enabled: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) ? "Yes" : "No");
    print_message("    - Guided Enabled: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) ? "Yes" : "No");
    print_message("    - Stabilize Enabled: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) ? "Yes" : "No");
    print_message("    - Manual Input Enabled: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) ? "Yes" : "No");
    print_message("    - Safety Armed: %s",
                 (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "Yes" : "No");
    print_message("  Custom Mode: %u", heartbeat.custom_mode);
    print_message("  System Status: %u (0=Uninit, 1=Boot, 3=Standby, 4=Active, 5=Critical)",
                 heartbeat.system_status);
    print_message("  MAVLink Version: %u", heartbeat.mavlink_version);

    // Update the last heartbeat time
    _last_gcs_heartbeat_ms = millis();

    // Determine if this is a GCS or another vehicle
    bool is_gcs = false;
    if (heartbeat.type == MAV_TYPE_GENERIC) {
        is_gcs = true;  // Generic type is usually a GCS
    } else if (heartbeat.type >= 6) {
        is_gcs = true;  // Types 6+ are GCS stations
    }

    if (is_gcs) {
        print_message("  >>> This is a GROUND CONTROL STATION <<<");

        // In the real implementation, this is where we'd:
        // 1. Update GCS connection status
        // 2. Potentially send queued messages
        // 3. Update failsafe timers
        // 4. Log the connection event

        // Check if this is a new GCS connection
        static bool first_connection = true;
        if (first_connection) {
            print_message("*** GCS CONNECTED FOR THE FIRST TIME ***");
            first_connection = false;

            // In real code, this would trigger:
            // - Send initial parameters
            // - Send system status
            // - Enable telemetry streams
        }
    } else {
        print_message("  >>> This is another VEHICLE <<<");
    }

    print_message("========================\n");
}

void GCS_MAVLINK::send_heartbeat()
{
    /*
     * Create and send a HEARTBEAT message
     *
     * This is one of the simplest messages to send, but also the most critical.
     * The GCS will consider the vehicle disconnected if it doesn't receive
     * a HEARTBEAT for more than 2.5 seconds.
     */

    print_message("=== SENDING HEARTBEAT ===");

    // Create the heartbeat payload
    mavlink_heartbeat_t heartbeat;

    // Type: What kind of vehicle are we?
    heartbeat.type = frame_type();  // Virtual function, vehicle-specific
    print_message("  Type: %u", heartbeat.type);

    // Autopilot: What autopilot software?
    heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    print_message("  Autopilot: ArduPilot (%u)", heartbeat.autopilot);

    // Base mode: Bitfield of system modes and states
    heartbeat.base_mode = base_mode();  // Virtual function, vehicle-specific
    print_message("  Base Mode: 0x%02X", heartbeat.base_mode);

    // Custom mode: Vehicle-specific mode (e.g., Copter: STABILIZE=0, ALT_HOLD=2, etc.)
    heartbeat.custom_mode = custom_mode();  // Virtual function, vehicle-specific
    print_message("  Custom Mode: %u", heartbeat.custom_mode);

    // System status: Overall system health
    heartbeat.system_status = system_status();  // Virtual function, vehicle-specific
    print_message("  System Status: %u", heartbeat.system_status);

    // MAVLink version (always 3 for MAVLink 1.0/2.0)
    heartbeat.mavlink_version = 3;

    // Pack into a MAVLink message
    mavlink_message_t msg;
    msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    msg.sysid = _system_id;
    msg.compid = _component_id;
    msg.len = sizeof(mavlink_heartbeat_t);
    memcpy(msg.payload, &heartbeat, sizeof(heartbeat));

    // Send it!
    send_message(msg);

    print_message("  >>> HEARTBEAT SENT <<<");
    print_message("======================\n");
}

/*
 * ============================================
 * Why HEARTBEAT is Important
 * ============================================
 *
 * 1. CONNECTION MONITORING
 *    - GCS knows vehicle is alive
 *    - Vehicle knows GCS is alive
 *    - Failsafe triggers if heartbeats stop
 *
 * 2. STATE INFORMATION
 *    - Armed/disarmed status
 *    - Current flight mode
 *    - System health
 *    - Available capabilities
 *
 * 3. SYSTEM IDENTIFICATION
 *    - Vehicle type (quad, plane, rover, etc.)
 *    - Autopilot type (ArduPilot, PX4, etc.)
 *    - System/component ID for routing
 *
 * 4. HANDSHAKING
 *    - Initial connection establishment
 *    - Protocol version negotiation
 *    - Capability discovery
 *
 * ============================================
 * Real GCS_Common.cpp Comparison
 * ============================================
 *
 * In the full ArduPilot GCS_Common.cpp:
 *
 * - Contains ~300 different message handlers
 * - Contains ~200 different message senders
 * - Handles parameters, missions, commands, etc.
 * - Manages stream rates and message intervals
 * - Implements FTP, logging, and more
 * - ~83,000 tokens of code!
 *
 * This minimal version shows just the HEARTBEAT,
 * which is the foundation of all MAVLink communication.
 *
 * To add more functionality, you would add more
 * handle_xxx() and send_xxx() methods following
 * the same pattern as HEARTBEAT.
 */
