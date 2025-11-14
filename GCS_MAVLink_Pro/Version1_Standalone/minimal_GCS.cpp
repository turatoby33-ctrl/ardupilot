/*
 * Minimal Standalone GCS Implementation
 * Version 1: Basic GCS Infrastructure
 */

#include "minimal_GCS.h"
#include <stdarg.h>
#include <sys/time.h>

// ============================================
// Helper Functions Implementation
// ============================================

uint32_t get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

void print_message(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    printf("[%u] %s\n", get_time_ms(), buffer);
}

// ============================================
// GCS_MAVLINK Implementation
// ============================================

GCS_MAVLINK::GCS_MAVLINK()
    : _chan(0)
    , _last_gcs_heartbeat_ms(0)
    , _system_id(1)
    , _component_id(1)
    , _seq(0)
    , _tx_buffer_len(0)
{
}

bool GCS_MAVLINK::init(uint8_t instance) {
    _chan = instance;
    print_message("GCS_MAVLINK: Channel %u initialized", _chan);
    return true;
}

uint32_t GCS_MAVLINK::millis() const {
    return get_time_ms();
}

bool GCS_MAVLINK::is_active() const {
    // Consider active if we received a heartbeat in the last 2.5 seconds
    return (millis() - _last_gcs_heartbeat_ms) < 2500;
}

uint8_t GCS_MAVLINK::base_mode() const {
    uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    // Not armed by default
    return mode;
}

MAV_STATE GCS_MAVLINK::system_status() const {
    return MAV_STATE_STANDBY;
}

// Simple MAVLink parser (minimal version)
bool GCS_MAVLINK::parse_byte(uint8_t byte, mavlink_message_t &msg, mavlink_status_t &status) {
    // This is a simplified parser for demonstration
    // In real implementation, this would be in the MAVLink library

    // For this minimal version, we'll just simulate receiving complete messages
    // A real parser would handle byte-by-byte state machine parsing

    return false;  // Simplified - actual parsing happens in handle_message
}

void GCS_MAVLINK::send_message(const mavlink_message_t &msg) {
    // Pack the message into the TX buffer
    // In a real implementation, this would serialize the message properly

    print_message("GCS_MAVLINK: Sending message ID %u (len=%u)", msg.msgid, msg.len);

    // For demonstration, we'll just store it in our buffer
    // In real code, this would write to a UART or network socket
    if (_tx_buffer_len + msg.len + 8 < sizeof(_tx_buffer)) {
        // Simplified: Just copy message info
        _tx_buffer[_tx_buffer_len++] = 0xFE;  // STX (MAVLink 1.0)
        _tx_buffer[_tx_buffer_len++] = msg.len;
        _tx_buffer[_tx_buffer_len++] = _seq++;
        _tx_buffer[_tx_buffer_len++] = _system_id;
        _tx_buffer[_tx_buffer_len++] = _component_id;
        _tx_buffer[_tx_buffer_len++] = msg.msgid;

        // Copy payload
        for (uint8_t i = 0; i < msg.len; i++) {
            _tx_buffer[_tx_buffer_len++] = msg.payload[i];
        }

        // Checksum (simplified - real version would calculate CRC)
        _tx_buffer[_tx_buffer_len++] = 0x00;
        _tx_buffer[_tx_buffer_len++] = 0x00;
    }
}

void GCS_MAVLINK::update_receive() {
    // In a real implementation, this would read from UART/socket and parse bytes
    // For this minimal version, we'll simulate it in the test program
}

void GCS_MAVLINK::update_send() {
    // Send periodic messages
    static uint32_t last_heartbeat_ms = 0;
    uint32_t now = millis();

    // Send HEARTBEAT at 1 Hz
    if (now - last_heartbeat_ms >= 1000) {
        send_heartbeat();
        last_heartbeat_ms = now;
    }
}

void GCS_MAVLINK::send_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // Fill in heartbeat data
    heartbeat.type = frame_type();
    heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    heartbeat.base_mode = base_mode();
    heartbeat.custom_mode = custom_mode();
    heartbeat.system_status = system_status();
    heartbeat.mavlink_version = 3;  // MAVLink version

    // Pack into message
    msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    msg.sysid = _system_id;
    msg.compid = _component_id;
    msg.len = sizeof(mavlink_heartbeat_t);
    memcpy(msg.payload, &heartbeat, sizeof(heartbeat));

    // Send it
    send_message(msg);

    print_message("GCS_MAVLINK: HEARTBEAT sent - type=%u, autopilot=%u, base_mode=%u, status=%u",
                 heartbeat.type, heartbeat.autopilot, heartbeat.base_mode, heartbeat.system_status);
}

void GCS_MAVLINK::handle_message(const mavlink_message_t &msg) {
    print_message("GCS_MAVLINK: Received message ID %u from sysid=%u compid=%u",
                 msg.msgid, msg.sysid, msg.compid);

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_heartbeat(msg);
        break;

    default:
        print_message("GCS_MAVLINK: Unhandled message ID %u", msg.msgid);
        break;
    }
}

void GCS_MAVLINK::handle_heartbeat(const mavlink_message_t &msg) {
    mavlink_heartbeat_t heartbeat;
    memcpy(&heartbeat, msg.payload, sizeof(heartbeat));

    print_message("GCS_MAVLINK: HEARTBEAT received - type=%u, autopilot=%u, base_mode=%u, status=%u",
                 heartbeat.type, heartbeat.autopilot, heartbeat.base_mode, heartbeat.system_status);

    // Update last heartbeat time
    _last_gcs_heartbeat_ms = millis();

    // Check if it's from a GCS (not another vehicle)
    if (heartbeat.type == MAV_TYPE_GENERIC ||
        heartbeat.type >= 6) {  // GCS types are typically 6+
        print_message("GCS_MAVLINK: GCS connection detected (sysid=%u)", msg.sysid);
    }
}

// ============================================
// GCS Implementation
// ============================================

// Global GCS instance
static GCS g_gcs;

GCS::GCS()
    : _num_channels(0)
{
    _channels[0] = nullptr;
}

void GCS::init() {
    print_message("GCS: Initializing");

    // Create a single channel
    _channels[0] = new GCS_MAVLINK();
    _channels[0]->init(0);
    _num_channels = 1;

    print_message("GCS: Initialized with %u channel(s)", _num_channels);
}

void GCS::update_receive() {
    for (uint8_t i = 0; i < _num_channels; i++) {
        if (_channels[i]) {
            _channels[i]->update_receive();
        }
    }
}

void GCS::update_send() {
    for (uint8_t i = 0; i < _num_channels; i++) {
        if (_channels[i]) {
            _channels[i]->update_send();
        }
    }
}

// Global accessor
GCS& gcs() {
    return g_gcs;
}
