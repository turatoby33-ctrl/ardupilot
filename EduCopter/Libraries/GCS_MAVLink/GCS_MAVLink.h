#pragma once

#include "../AP_AHRS/AP_AHRS.h"
#include "../AP_GPS/AP_GPS.h"
#include "../AP_Baro/AP_Baro.h"
#include <stdint.h>

// Simplified MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_COMMAND_LONG 76

// MAVLink system types
#define MAV_TYPE_QUADROTOR 2

// MAVLink autopilot types
#define MAV_AUTOPILOT_GENERIC 0

// MAVLink modes
#define MAV_MODE_FLAG_SAFETY_ARMED 128
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1

// MAVLink state
#define MAV_STATE_STANDBY 3
#define MAV_STATE_ACTIVE 4

// Simplified MAVLink communication
class GCS_MAVLink {
public:
    GCS_MAVLink(AP_AHRS& ahrs, AP_GPS& gps, AP_Baro& baro);

    void init();
    void update();

    // Send messages
    void send_heartbeat(bool armed, uint8_t flight_mode);
    void send_sys_status(float battery_voltage, float battery_remaining);
    void send_attitude();
    void send_global_position_int();
    void send_vfr_hud(float airspeed, float groundspeed, float climb_rate, float throttle, float alt);

    // Receive and process messages
    void handle_message();

    // Set streams
    void set_stream_rate(uint16_t rate_hz) { _stream_rate_hz = rate_hz; }

private:
    AP_AHRS& _ahrs;
    AP_GPS& _gps;
    AP_Baro& _baro;

    uint16_t _stream_rate_hz;
    uint64_t _last_heartbeat_us;
    uint64_t _last_attitude_us;
    uint64_t _last_position_us;
    uint64_t _last_vfr_us;

    uint8_t _system_id;
    uint8_t _component_id;

    // Helper to send MAVLink packet
    void send_packet(const uint8_t* buf, uint16_t len);

    // Simple packet building
    void build_heartbeat_packet(uint8_t* buf, uint16_t& len, bool armed, uint8_t flight_mode);
    void build_attitude_packet(uint8_t* buf, uint16_t& len);
    void build_global_position_packet(uint8_t* buf, uint16_t& len);
};
