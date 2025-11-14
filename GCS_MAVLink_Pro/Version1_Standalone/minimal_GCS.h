/*
 * Minimal Standalone GCS Implementation
 * Version 1: Standalone HEARTBEAT Send/Receive
 *
 * This is a simplified, standalone version to demonstrate basic GCS concepts
 * without requiring the full ArduPilot infrastructure.
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// Minimal MAVLink type definitions (normally from mavlink headers)
#define MAVLINK_COMM_0 0

typedef enum {
    MAV_TYPE_GENERIC = 0,
    MAV_TYPE_QUADROTOR = 2,
    MAV_TYPE_HELICOPTER = 4,
    MAV_TYPE_GROUND_ROVER = 10,
} MAV_TYPE;

typedef enum {
    MAV_AUTOPILOT_GENERIC = 0,
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
} MAV_AUTOPILOT;

typedef enum {
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6,
    MAV_STATE_POWEROFF = 7,
} MAV_STATE;

typedef enum {
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
    MAV_MODE_FLAG_TEST_ENABLED = 2,
    MAV_MODE_FLAG_AUTO_ENABLED = 4,
    MAV_MODE_FLAG_GUIDED_ENABLED = 8,
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    MAV_MODE_FLAG_HIL_ENABLED = 32,
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
    MAV_MODE_FLAG_SAFETY_ARMED = 128,
} MAV_MODE_FLAG;

// Minimal MAVLink message structure
typedef struct {
    uint8_t msgid;
    uint8_t sysid;
    uint8_t compid;
    uint8_t len;
    uint8_t seq;
    uint8_t payload[255];
} mavlink_message_t;

// Minimal MAVLink status
typedef struct {
    uint8_t parse_state;
    uint8_t packet_received;
} mavlink_status_t;

// HEARTBEAT message ID
#define MAVLINK_MSG_ID_HEARTBEAT 0

// HEARTBEAT payload structure
typedef struct {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} mavlink_heartbeat_t;

// ============================================
// Minimal GCS_MAVLINK Class
// ============================================

class GCS_MAVLINK {
public:
    GCS_MAVLINK();
    virtual ~GCS_MAVLINK() {}

    // Initialization
    bool init(uint8_t instance);

    // Main update functions
    void update_receive();
    void update_send();

    // Send HEARTBEAT
    void send_heartbeat();

    // Handle received messages
    void handle_message(const mavlink_message_t &msg);

    // Channel info
    uint8_t get_channel() const { return _chan; }
    uint32_t get_last_heartbeat_time() const { return _last_gcs_heartbeat_ms; }
    bool is_active() const;

    // Timing
    uint32_t millis() const;

protected:
    // Virtual functions for vehicle-specific behavior
    virtual uint8_t base_mode() const;
    virtual MAV_STATE system_status() const;
    virtual uint32_t custom_mode() const { return 0; }
    virtual MAV_TYPE frame_type() const { return MAV_TYPE_GENERIC; }

    // Handle specific messages
    virtual void handle_heartbeat(const mavlink_message_t &msg);

    // Channel ID
    uint8_t _chan;

    // Last time we received a heartbeat from GCS
    uint32_t _last_gcs_heartbeat_ms;

    // Our system ID
    uint8_t _system_id;
    uint8_t _component_id;

    // Sequence number for outgoing messages
    uint8_t _seq;

    // Simulated serial buffer (for demonstration)
    uint8_t _tx_buffer[1024];
    uint16_t _tx_buffer_len;

    // Parse incoming bytes
    bool parse_byte(uint8_t byte, mavlink_message_t &msg, mavlink_status_t &status);

    // Pack and send message
    void send_message(const mavlink_message_t &msg);
};

// ============================================
// Minimal GCS Class (Global Manager)
// ============================================

class GCS {
public:
    GCS();

    // Initialize GCS
    void init();

    // Get channel
    GCS_MAVLINK* chan(uint8_t ofs) {
        if (ofs >= _num_channels) return nullptr;
        return _channels[ofs];
    }

    // Update all channels
    void update_receive();
    void update_send();

    // Vehicle info (pure virtual in real implementation)
    virtual uint32_t custom_mode() const { return 0; }
    virtual MAV_TYPE frame_type() const { return MAV_TYPE_GENERIC; }

    // Number of channels
    uint8_t num_channels() const { return _num_channels; }

protected:
    GCS_MAVLINK* _channels[1];  // Single channel for simplicity
    uint8_t _num_channels;
};

// ============================================
// Helper Functions
// ============================================

// Get current time in milliseconds (simple implementation)
uint32_t get_time_ms();

// Print message (for debugging)
void print_message(const char* format, ...);

// Global GCS instance accessor
GCS& gcs();
