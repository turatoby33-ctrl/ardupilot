/**
 * @file mavlink_stubs.h
 * @brief Minimal MAVLink definitions for testing
 *
 * This file provides the minimum MAVLink types and constants needed
 * to compile and run the EduCopter GCS tests.
 */

#pragma once

#include <cstdint>
#include <cstring>

// MAVLink protocol constants
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_SIGNATURE_BLOCK_LEN 13

// MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_FENCE_STATUS 162
#define MAVLINK_MSG_ID_RALLY_POINT 175
#define MAVLINK_MSG_ID_SERIAL_CONTROL 126
#define MAVLINK_MSG_ID_DEVICE_OP_READ 11000
#define MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY 11001
#define MAVLINK_MSG_ID_DEVICE_OP_WRITE 11002
#define MAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY 11003
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36

// MAVLink flags
#define MAVLINK_IFLAG_SIGNED 0x01

// MAVLink component IDs
#define MAV_COMP_ID_ALL 0
#define MAV_COMP_ID_AUTOPILOT1 1
#define MAV_COMP_ID_ONBOARD_COMPUTER 191
#define MAV_COMP_ID_CAMERA 100

// Serial control flags
#define SERIAL_CONTROL_FLAG_REPLY 0x01
#define SERIAL_CONTROL_FLAG_RESPOND 0x02
#define SERIAL_CONTROL_FLAG_EXCLUSIVE 0x04
#define SERIAL_CONTROL_FLAG_BLOCKING 0x08
#define SERIAL_CONTROL_FLAG_MULTI 0x10

// Device op bus types
#define DEVICE_OP_BUSTYPE_I2C 0
#define DEVICE_OP_BUSTYPE_SPI 1

// Fence breach types
#define FENCE_BREACH_BOUNDARY 1

// MAVLink types enum
typedef enum MAV_TYPE {
    MAV_TYPE_GENERIC = 0,
    MAV_TYPE_FIXED_WING = 1,
    MAV_TYPE_QUADROTOR = 2,
    MAV_TYPE_HELICOPTER = 4,
    MAV_TYPE_GROUND_ROVER = 10,
    MAV_TYPE_SUBMARINE = 12,
    MAV_TYPE_HEXAROTOR = 13,
    MAV_TYPE_OCTOROTOR = 14,
    MAV_TYPE_VTOL_DUOROTOR = 19,
    MAV_TYPE_VTOL_QUADROTOR = 20,
    MAV_TYPE_ONBOARD_CONTROLLER = 18,
    MAV_TYPE_CAMERA = 30,
    MAV_TYPE_GCS = 6,
    MAV_TYPE_GIMBAL = 26
} mav_type_t;

// Autopilot types
typedef enum MAV_AUTOPILOT {
    MAV_AUTOPILOT_GENERIC = 0,
    MAV_AUTOPILOT_INVALID = 8
} mav_autopilot_t;

// System states
typedef enum MAV_STATE {
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6,
    MAV_STATE_POWEROFF = 7
} mav_state_t;

// Mode flags
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define MAV_MODE_FLAG_TEST_ENABLED 2
#define MAV_MODE_FLAG_AUTO_ENABLED 4
#define MAV_MODE_FLAG_GUIDED_ENABLED 8
#define MAV_MODE_FLAG_STABILIZE_ENABLED 16
#define MAV_MODE_FLAG_HIL_ENABLED 32
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED 64
#define MAV_MODE_FLAG_SAFETY_ARMED 128

// MAVLink message structure
typedef struct __mavlink_message {
    uint16_t checksum;
    uint8_t magic;
    uint8_t len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid : 24;
    uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES + 7) / 8];
    uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
} mavlink_message_t;

// Heartbeat message structure
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} mavlink_heartbeat_t;

// Helper to pack heartbeat
static inline uint16_t mavlink_msg_heartbeat_pack(
    uint8_t system_id,
    uint8_t component_id,
    mavlink_message_t* msg,
    uint8_t type,
    uint8_t autopilot,
    uint8_t base_mode,
    uint32_t custom_mode,
    uint8_t system_status)
{
    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->len = 9;

    mavlink_heartbeat_t* payload = (mavlink_heartbeat_t*)msg->payload64;
    payload->type = type;
    payload->autopilot = autopilot;
    payload->base_mode = base_mode;
    payload->custom_mode = custom_mode;
    payload->system_status = system_status;
    payload->mavlink_version = 3;

    return msg->len;
}

// Helper to decode heartbeat
static inline void mavlink_msg_heartbeat_decode(
    const mavlink_message_t* msg,
    mavlink_heartbeat_t* heartbeat)
{
    memcpy(heartbeat, msg->payload64, sizeof(mavlink_heartbeat_t));
}

// Command long structure
typedef struct __mavlink_command_long_t {
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t confirmation;
} mavlink_command_long_t;

// Mission item int structure
typedef struct __mavlink_mission_item_int_t {
    float param1;
    float param2;
    float param3;
    float param4;
    int32_t x;
    int32_t y;
    float z;
    uint16_t seq;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
    uint8_t current;
    uint8_t autocontinue;
    uint8_t mission_type;
} mavlink_mission_item_int_t;

// MAVLink result codes
typedef enum MAV_RESULT {
    MAV_RESULT_ACCEPTED = 0,
    MAV_RESULT_FAILED = 4,
    MAV_RESULT_UNSUPPORTED = 3
} mav_result_t;

// MAVLink mission types
typedef enum MAV_MISSION_TYPE {
    MAV_MISSION_TYPE_MISSION = 0,
    MAV_MISSION_TYPE_FENCE = 1,
    MAV_MISSION_TYPE_RALLY = 2
} mav_mission_type_t;

// MAVLink mission results
typedef enum MAV_MISSION_RESULT {
    MAV_MISSION_ACCEPTED = 0,
    MAV_MISSION_ERROR = 1,
    MAV_MISSION_UNSUPPORTED = 2,
    MAV_MISSION_NO_SPACE = 3,
    MAV_MISSION_INVALID = 4,
    MAV_MISSION_INVALID_PARAM1 = 5,
    MAV_MISSION_INVALID_PARAM5_X = 9,
    MAV_MISSION_INVALID_PARAM6_Y = 10,
    MAV_MISSION_INVALID_PARAM7 = 11,
    MAV_MISSION_INVALID_SEQUENCE = 12,
    MAV_MISSION_UNSUPPORTED_FRAME = 8
} mav_mission_result_t;

// Severity levels
typedef enum MAV_SEVERITY {
    MAV_SEVERITY_EMERGENCY = 0,
    MAV_SEVERITY_ALERT = 1,
    MAV_SEVERITY_CRITICAL = 2,
    MAV_SEVERITY_ERROR = 3,
    MAV_SEVERITY_WARNING = 4,
    MAV_SEVERITY_NOTICE = 5,
    MAV_SEVERITY_INFO = 6,
    MAV_SEVERITY_DEBUG = 7
} mav_severity_t;

// MAVLink commands
#define MAV_CMD_NAV_WAYPOINT 16
#define MAV_CMD_NAV_LOITER_UNLIM 17
#define MAV_CMD_NAV_LOITER_TURNS 18
#define MAV_CMD_NAV_LOITER_TIME 19
#define MAV_CMD_NAV_RETURN_TO_LAUNCH 20
#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_NAV_TAKEOFF 22
#define MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT 30
#define MAV_CMD_NAV_LOITER_TO_ALT 31
#define MAV_CMD_NAV_SPLINE_WAYPOINT 82
#define MAV_CMD_NAV_FENCE_RETURN_POINT 5000
#define MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION 5001
#define MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION 5002
#define MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION 5003
#define MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION 5004
#define MAV_CMD_NAV_RALLY_POINT 5100
#define MAV_CMD_CONDITION_DELAY 112
#define MAV_CMD_CONDITION_DISTANCE 114
#define MAV_CMD_CONDITION_YAW 115
#define MAV_CMD_DO_JUMP 177
#define MAV_CMD_DO_CHANGE_SPEED 178
#define MAV_CMD_DO_SET_HOME 179
#define MAV_CMD_DO_SET_SERVO 183
#define MAV_CMD_DO_SET_RELAY 181
#define MAV_CMD_DO_REPEAT_SERVO 184
#define MAV_CMD_DO_REPEAT_RELAY 182
#define MAV_CMD_DO_SET_ROI 201
#define MAV_CMD_DO_DIGICAM_CONTROL 203
#define MAV_CMD_DO_MOUNT_CONTROL 205
#define MAV_CMD_DO_SET_CAM_TRIGG_DIST 206
#define MAV_CMD_NAV_LAST 95

// MAVLink frames
#define MAV_FRAME_GLOBAL 0
#define MAV_FRAME_GLOBAL_RELATIVE_ALT 3
#define MAV_FRAME_GLOBAL_INT 5
#define MAV_FRAME_GLOBAL_RELATIVE_ALT_INT 6
#define MAV_FRAME_MISSION 2
