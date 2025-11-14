/*
 * Minimal ArduPilot GCS_Common Implementation
 * Version 2: HEARTBEAT Send/Receive for Real ArduPilot Integration
 *
 * This file is designed to work with the REAL ArduPilot GCS infrastructure:
 * - libraries/GCS_MAVLink/GCS.h
 * - libraries/GCS_MAVLink/GCS.cpp
 * - libraries/GCS_MAVLink/GCS_MAVLink.h
 * - libraries/GCS_MAVLink/GCS_MAVLink.cpp
 *
 * IMPORTANT: This is a MINIMAL implementation showing only HEARTBEAT.
 * The real GCS_Common.cpp is ~83,000 tokens with hundreds of messages.
 * This demonstrates the pattern for adding messages to the real system.
 *
 * TO USE THIS:
 * 1. Copy this file to: libraries/GCS_MAVLink/GCS_Common_Minimal.cpp
 * 2. Add to your vehicle's wscript build
 * 3. Include it instead of full GCS_Common.cpp for testing
 * 4. See integration example for complete setup
 */

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// ============================================
// Constructor
// ============================================

GCS_MAVLINK::GCS_MAVLINK(AP_HAL::UARTDriver &uart)
{
    AP_Param::setup_object_defaults(this, var_info);
    _port = &uart;
}

// ============================================
// Initialization
// ============================================

bool GCS_MAVLINK::init(uint8_t instance)
{
    // Get associated mavlink channel
    chan = (mavlink_channel_t)(MAVLINK_COMM_0 + instance);
    if (!valid_channel(chan)) {
        return false;
    }

    // Find MAVLink protocol instance
    uartstate = AP::serialmanager().find_protocol_instance(
        AP_SerialManager::SerialProtocol_MAVLink, instance);
    if (uartstate == nullptr) {
        return false;
    }

    // Configure UART
    _port->begin(uartstate->baudrate());
    mavlink_comm_port[chan] = _port;

    hal.console->printf("GCS_MAVLINK: Channel %u initialized at %u baud\n",
                       chan, uartstate->baudrate());

    return true;
}

// ============================================
// Update Loops
// ============================================

void GCS_MAVLINK::update_receive(uint32_t max_time_us)
{
    /*
     * Parse incoming bytes from UART
     * In the full implementation, this reads from _port and parses
     * MAVLink protocol byte-by-byte.
     *
     * For this minimal version, we show the structure but
     * actual parsing is done by MAVLink library.
     */

    uint32_t start_us = AP_HAL::micros();

    while ((AP_HAL::micros() - start_us) < max_time_us) {
        int16_t byte = _port->read();
        if (byte < 0) {
            break;  // No more data
        }

        // Parse the byte
        mavlink_message_t msg;
        mavlink_status_t status;

        if (mavlink_parse_char(chan, (uint8_t)byte, &msg, &status)) {
            // Complete message received!
            hal.console->printf("GCS_MAVLINK: Received message ID %u from sysid=%u\n",
                              msg.msgid, msg.sysid);

            // Handle it
            handle_message(msg);
        }
    }
}

void GCS_MAVLINK::update_send()
{
    /*
     * Send periodic messages
     * In the full implementation, this handles:
     * - Streams (EXTRA1, EXTRA2, POSITION, etc.)
     * - Deferred messages (HEARTBEAT, PARAM_VALUE, etc.)
     * - Queued messages (mission items, parameters, etc.)
     *
     * For this minimal version, we just send HEARTBEAT at 1 Hz.
     */

    static uint32_t last_heartbeat_ms = 0;
    uint32_t now = AP_HAL::millis();

    // Send HEARTBEAT at 1 Hz
    if (now - last_heartbeat_ms >= 1000) {
        send_heartbeat();
        last_heartbeat_ms = now;
    }

    // In full implementation, would also send:
    // - Other periodic messages based on stream rates
    // - Queued parameters
    // - Queued mission items
    // - Statustext messages
    // etc.
}

// ============================================
// HEARTBEAT: Send
// ============================================

void GCS_MAVLINK::send_heartbeat() const
{
    /*
     * Send HEARTBEAT message
     *
     * This is identical to the full implementation.
     * HEARTBEAT is the foundation - it's sent even when
     * all other telemetry is disabled.
     */

    // Check if we have space in the output buffer
    if (!HAVE_PAYLOAD_SPACE(chan, HEARTBEAT)) {
        hal.console->printf("GCS_MAVLINK: No space for HEARTBEAT\n");
        return;
    }

    // Get vehicle-specific information
    uint8_t baseMode = base_mode();
    uint32_t customMode = gcs().custom_mode();
    uint8_t systemStatus = (uint8_t)vehicle_system_status();

    // Send the HEARTBEAT message
    mavlink_msg_heartbeat_send(
        chan,
        gcs().frame_type(),           // MAV_TYPE
        MAV_AUTOPILOT_ARDUPILOTMEGA,  // ArduPilot
        baseMode,                      // base_mode flags
        customMode,                    // custom_mode (vehicle-specific)
        systemStatus                   // system_status
    );

    hal.console->printf("GCS_MAVLINK: HEARTBEAT sent - base_mode=0x%02X, custom_mode=%u, status=%u\n",
                       baseMode, customMode, systemStatus);
}

// ============================================
// HEARTBEAT: Receive
// ============================================

void GCS_MAVLINK::handle_heartbeat(const mavlink_message_t &msg)
{
    /*
     * Handle received HEARTBEAT message
     *
     * This is called when a HEARTBEAT (msgid=0) is received.
     * In the full implementation, this does:
     * - Update GCS connection status
     * - Reset failsafe timers
     * - Log the connection
     * - Trigger initial parameter/mission downloads
     */

    // Decode the HEARTBEAT
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    hal.console->printf("=== HEARTBEAT RECEIVED ===\n");
    hal.console->printf("  From: sysid=%u compid=%u\n", msg.sysid, msg.compid);
    hal.console->printf("  Type: %u\n", heartbeat.type);
    hal.console->printf("  Autopilot: %u\n", heartbeat.autopilot);
    hal.console->printf("  Base Mode: 0x%02X\n", heartbeat.base_mode);
    hal.console->printf("    Armed: %s\n",
                       (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? "YES" : "NO");
    hal.console->printf("  Custom Mode: %u\n", heartbeat.custom_mode);
    hal.console->printf("  System Status: %u\n", heartbeat.system_status);
    hal.console->printf("========================\n");

    // Update last heartbeat time
    last_heartbeat_time = AP_HAL::millis();

    // Inform global GCS that we saw a heartbeat from our GCS
    // This resets failsafe timers
    sysid_mygcs_seen(last_heartbeat_time);

    // Check if this is from a GCS (not another vehicle)
    if (heartbeat.type == MAV_TYPE_GCS ||
        heartbeat.type == MAV_TYPE_ONBOARD_CONTROLLER ||
        heartbeat.type == MAV_TYPE_GENERIC) {

        hal.console->printf("GCS_MAVLINK: GCS connection detected\n");

        // In full implementation, would trigger:
        // - Parameter list download (if requested)
        // - Mission list download (if requested)
        // - Stream rate initialization
        // - Send system capabilities
        // etc.
    }
}

// ============================================
// Message Router
// ============================================

void GCS_MAVLINK::handle_message(const mavlink_message_t &msg)
{
    /*
     * Route incoming messages to appropriate handlers
     *
     * In the full GCS_Common.cpp, this is a MASSIVE switch statement
     * with ~200 message handlers covering:
     * - Parameters (REQUEST_LIST, REQUEST_READ, SET)
     * - Missions (COUNT, REQUEST, ITEM, CLEAR_ALL)
     * - Commands (COMMAND_INT, COMMAND_LONG)
     * - RC input (RC_CHANNELS_OVERRIDE, MANUAL_CONTROL)
     * - And hundreds more...
     *
     * This minimal version shows only HEARTBEAT.
     */

    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        handle_heartbeat(msg);
        break;
    }

    // In full implementation, would have cases for:
    // case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    // case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    // case MAVLINK_MSG_ID_PARAM_SET:
    // case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    // case MAVLINK_MSG_ID_MISSION_COUNT:
    // case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    // case MAVLINK_MSG_ID_COMMAND_INT:
    // case MAVLINK_MSG_ID_COMMAND_LONG:
    // ... hundreds more ...

    default:
        hal.console->printf("GCS_MAVLINK: Unhandled message ID %u\n", msg.msgid);
        break;
    }
}

// ============================================
// Required Method Stubs
// ============================================

/*
 * The following methods are called by the GCS infrastructure
 * but are vehicle-specific. They must be implemented in your
 * vehicle's GCS_MAVLink_YourVehicle class.
 *
 * These are just stubs to allow compilation.
 */

void GCS_MAVLINK::send_nav_controller_output() const
{
    // Vehicle must implement
    // hal.console->printf("send_nav_controller_output() not implemented\n");
}

void GCS_MAVLINK::send_pid_tuning()
{
    // Vehicle must implement
    // hal.console->printf("send_pid_tuning() not implemented\n");
}

// ============================================
// Global GCS Functions
// ============================================

/*
 * These are global GCS functions that need minimal implementation
 */

void GCS::init()
{
    mavlink_system.sysid = sysid_this_mav();
    hal.console->printf("GCS: Initialized with sysid=%u\n", mavlink_system.sysid);
}

void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t mask)
{
    // Minimal text sending
    // Full implementation queues and sends STATUSTEXT messages

    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    hal.util->vsnprintf(text, sizeof(text), fmt, arg_list);

    hal.console->printf("[%u] TEXT: %s\n", AP_HAL::millis(), text);

    // In full implementation, would:
    // - Queue in statustext_queue
    // - Send STATUSTEXT message to all active channels
    // - Handle text segmentation for long messages
}

void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list, statustext_send_channel_mask());
    va_end(arg_list);
}

// ============================================
// Notes for Integration
// ============================================

/*
 * TO INTEGRATE THIS INTO YOUR VEHICLE:
 *
 * 1. Create your vehicle's GCS classes:
 *    - GCS_YourVehicle (inherits from GCS)
 *    - GCS_MAVLINK_YourVehicle (inherits from GCS_MAVLINK)
 *
 * 2. Implement required pure virtuals:
 *    - base_mode()
 *    - system_status()
 *    - send_nav_controller_output()
 *    - send_pid_tuning()
 *
 * 3. Call GCS functions in your main loop:
 *    - gcs().update_receive()
 *    - gcs().update_send()
 *
 * 4. Initialize in your setup:
 *    - gcs().init()
 *    - gcs().setup_console()
 *    - gcs().setup_uarts()
 *
 * See the integration example for complete code.
 *
 * ADDING MORE MESSAGES:
 *
 * To add more message types, follow the HEARTBEAT pattern:
 *
 * 1. Add send function:
 *    void GCS_MAVLINK::send_xxx() {
 *        CHECK_PAYLOAD_SIZE(XXX);
 *        mavlink_msg_xxx_send(chan, ...);
 *    }
 *
 * 2. Add handle function:
 *    void GCS_MAVLINK::handle_xxx(const mavlink_message_t &msg) {
 *        mavlink_xxx_t packet;
 *        mavlink_msg_xxx_decode(&msg, &packet);
 *        // Process...
 *    }
 *
 * 3. Add to handle_message() switch:
 *    case MAVLINK_MSG_ID_XXX:
 *        handle_xxx(msg);
 *        break;
 *
 * 4. Call send function in update_send() or vehicle code:
 *    send_xxx();
 *
 * This pattern works for ALL MAVLink messages!
 */

#endif // HAL_GCS_ENABLED
