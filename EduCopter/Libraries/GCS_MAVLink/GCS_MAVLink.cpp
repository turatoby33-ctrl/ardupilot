#include "GCS_MAVLink.h"
#include "../AP_HAL/AP_HAL.h"
#include <cstring>
#include <cstdio>

extern AP_HAL* hal;

GCS_MAVLink::GCS_MAVLink(AP_AHRS& ahrs, AP_GPS& gps, AP_Baro& baro) :
    _ahrs(ahrs),
    _gps(gps),
    _baro(baro),
    _stream_rate_hz(4),
    _last_heartbeat_us(0),
    _last_attitude_us(0),
    _last_position_us(0),
    _last_vfr_us(0),
    _system_id(1),
    _component_id(1)
{
}

void GCS_MAVLink::init() {
    printf("GCS_MAVLink: Initializing MAVLink communication\n");
    printf("GCS_MAVLink: System ID: %d, Component ID: %d\n", _system_id, _component_id);
}

void GCS_MAVLink::update() {
    uint64_t now_us = hal->micros64();

    // Send heartbeat at 1 Hz
    if (now_us - _last_heartbeat_us >= 1000000) {
        send_heartbeat(false, 0);
        _last_heartbeat_us = now_us;
    }

    // Send attitude at stream rate
    uint64_t stream_interval_us = 1000000 / _stream_rate_hz;
    if (now_us - _last_attitude_us >= stream_interval_us) {
        send_attitude();
        _last_attitude_us = now_us;
    }

    // Send position at stream rate
    if (now_us - _last_position_us >= stream_interval_us) {
        send_global_position_int();
        _last_position_us = now_us;
    }

    // Send VFR HUD at stream rate
    if (now_us - _last_vfr_us >= stream_interval_us) {
        send_vfr_hud(0.0f, _gps.get_ground_speed(), _baro.get_climb_rate(), 0.0f, _ahrs.get_altitude());
        _last_vfr_us = now_us;
    }
}

void GCS_MAVLink::send_heartbeat(bool armed, uint8_t flight_mode) {
    uint8_t buf[64];
    uint16_t len = 0;
    build_heartbeat_packet(buf, len, armed, flight_mode);
    send_packet(buf, len);
}

void GCS_MAVLink::send_attitude() {
    uint8_t buf[64];
    uint16_t len = 0;
    build_attitude_packet(buf, len);
    send_packet(buf, len);
}

void GCS_MAVLink::send_global_position_int() {
    uint8_t buf[64];
    uint16_t len = 0;
    build_global_position_packet(buf, len);
    send_packet(buf, len);
}

void GCS_MAVLink::send_sys_status(float battery_voltage, float battery_remaining) {
    // Simplified - just print for now
    // In a full implementation, this would build and send a proper MAVLink packet
}

void GCS_MAVLink::send_vfr_hud(float airspeed, float groundspeed, float climb_rate, float throttle, float alt) {
    // Simplified - just print for now
}

void GCS_MAVLink::handle_message() {
    // Simplified - would parse incoming MAVLink messages
    // and handle commands from QGroundControl
}

void GCS_MAVLink::build_heartbeat_packet(uint8_t* buf, uint16_t& len, bool armed, uint8_t flight_mode) {
    // This is a simplified MAVLink v1 heartbeat packet
    // In a real implementation, you would use the MAVLink library

    /*
     * MAVLink v1 packet structure:
     * [STX][LEN][SEQ][SYS][COMP][MSG][PAYLOAD][CRC_L][CRC_H]
     */

    len = 0;

    // STX
    buf[len++] = 0xFE;  // MAVLink v1 start byte

    // Payload length
    buf[len++] = 9;  // Heartbeat payload is 9 bytes

    // Packet sequence
    static uint8_t seq = 0;
    buf[len++] = seq++;

    // System ID
    buf[len++] = _system_id;

    // Component ID
    buf[len++] = _component_id;

    // Message ID
    buf[len++] = MAVLINK_MSG_ID_HEARTBEAT;

    // Payload
    uint32_t custom_mode = 0;
    buf[len++] = custom_mode & 0xFF;
    buf[len++] = (custom_mode >> 8) & 0xFF;
    buf[len++] = (custom_mode >> 16) & 0xFF;
    buf[len++] = (custom_mode >> 24) & 0xFF;

    buf[len++] = MAV_TYPE_QUADROTOR;
    buf[len++] = MAV_AUTOPILOT_GENERIC;

    uint8_t base_mode = 0;
    if (armed) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }
    buf[len++] = base_mode;

    buf[len++] = MAV_STATE_ACTIVE;

    buf[len++] = 3;  // MAVLink version

    // CRC (simplified - would calculate proper CRC in full implementation)
    buf[len++] = 0x00;
    buf[len++] = 0x00;
}

void GCS_MAVLink::build_attitude_packet(uint8_t* buf, uint16_t& len) {
    len = 0;

    buf[len++] = 0xFE;  // STX
    buf[len++] = 28;    // Payload length for ATTITUDE

    static uint8_t seq = 0;
    buf[len++] = seq++;

    buf[len++] = _system_id;
    buf[len++] = _component_id;
    buf[len++] = MAVLINK_MSG_ID_ATTITUDE;

    // Payload: time_boot_ms (4), roll (4), pitch (4), yaw (4), rollspeed (4), pitchspeed (4), yawspeed (4)
    uint32_t time_boot_ms = hal->millis();
    memcpy(&buf[len], &time_boot_ms, 4);
    len += 4;

    float roll = _ahrs.get_roll();
    memcpy(&buf[len], &roll, 4);
    len += 4;

    float pitch = _ahrs.get_pitch();
    memcpy(&buf[len], &pitch, 4);
    len += 4;

    float yaw = _ahrs.get_yaw();
    memcpy(&buf[len], &yaw, 4);
    len += 4;

    const Vector3f& gyro = _ahrs.get_gyro();
    memcpy(&buf[len], &gyro.x, 4);
    len += 4;
    memcpy(&buf[len], &gyro.y, 4);
    len += 4;
    memcpy(&buf[len], &gyro.z, 4);
    len += 4;

    // CRC
    buf[len++] = 0x00;
    buf[len++] = 0x00;
}

void GCS_MAVLink::build_global_position_packet(uint8_t* buf, uint16_t& len) {
    len = 0;

    buf[len++] = 0xFE;  // STX
    buf[len++] = 28;    // Payload length

    static uint8_t seq = 0;
    buf[len++] = seq++;

    buf[len++] = _system_id;
    buf[len++] = _component_id;
    buf[len++] = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;

    // Payload
    uint32_t time_boot_ms = hal->millis();
    memcpy(&buf[len], &time_boot_ms, 4);
    len += 4;

    int32_t lat = (int32_t)(_gps.get_latitude() * 1e7);
    memcpy(&buf[len], &lat, 4);
    len += 4;

    int32_t lon = (int32_t)(_gps.get_longitude() * 1e7);
    memcpy(&buf[len], &lon, 4);
    len += 4;

    int32_t alt = (int32_t)(_gps.get_altitude() * 1000);
    memcpy(&buf[len], &alt, 4);
    len += 4;

    int32_t relative_alt = (int32_t)(_ahrs.get_altitude() * 1000);
    memcpy(&buf[len], &relative_alt, 4);
    len += 4;

    const Vector3f& vel = _ahrs.get_velocity();
    int16_t vx = (int16_t)(vel.x * 100);
    int16_t vy = (int16_t)(vel.y * 100);
    int16_t vz = (int16_t)(vel.z * 100);
    memcpy(&buf[len], &vx, 2);
    len += 2;
    memcpy(&buf[len], &vy, 2);
    len += 2;
    memcpy(&buf[len], &vz, 2);
    len += 2;

    uint16_t hdg = (uint16_t)(_ahrs.get_yaw() * 5729.58f);  // rad to centidegrees
    memcpy(&buf[len], &hdg, 2);
    len += 2;

    // CRC
    buf[len++] = 0x00;
    buf[len++] = 0x00;
}

void GCS_MAVLink::send_packet(const uint8_t* buf, uint16_t len) {
    // In a real implementation, this would send via serial/UDP
    // For now, we just log that a packet was sent
    // printf("MAVLink: Sent packet (msg_id=%d, len=%d)\n", buf[5], len);
}
