/**
 * @file test_harness.h
 * @brief Test harness with MAVLink mocks and external dependencies for integration tests
 *
 * This file provides:
 * - Minimal MAVLink type definitions
 * - Mock implementations of external functions required by EduCopter classes
 * - Test utilities
 *
 * @author EduCopter Test Suite
 * @date 2025
 */

#pragma once

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>

// Test macros
#define TEST_PASS() return true
#define TEST_FAIL(msg) do { printf("  FAIL: %s\n", msg); return false; } while(0)
#define TEST_ASSERT(cond, msg) if (!(cond)) TEST_FAIL(msg)

//=============================================================================
// MAVLink Type Definitions (minimal set for testing)
//=============================================================================

#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2

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
    uint8_t ck[2];
    uint8_t signature[13];
} mavlink_message_t;

// MAVLink status structure
typedef struct __mavlink_status {
    uint8_t msg_received;
    uint8_t buffer_overrun;
    uint8_t parse_error;
    uint8_t parse_state;
    uint8_t packet_idx;
    uint8_t current_rx_seq;
    uint8_t current_tx_seq;
    uint16_t packet_rx_success_count;
    uint16_t packet_rx_drop_count;
    uint8_t flags;
    uint8_t signature_wait;
    uint8_t signing_streams;
} mavlink_status_t;

// MAVLink channel structure
typedef struct __mavlink_channel {
    mavlink_status_t status;
    mavlink_message_t rx_msg;
} mavlink_channel_t;

// Mission item integer structure
typedef struct __mavlink_mission_item_int_t {
    float param1;
    float param2;
    float param3;
    float param4;
    int32_t x;          // Latitude
    int32_t y;          // Longitude
    float z;            // Altitude
    uint16_t seq;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t frame;
    uint8_t current;
    uint8_t autocontinue;
    uint8_t mission_type;
} mavlink_mission_item_int_t;

// Serial control structure
typedef struct __mavlink_serial_control_t {
    uint32_t baudrate;
    uint16_t timeout;
    uint8_t device;
    uint8_t flags;
    uint16_t count;
    uint8_t data[70];
} mavlink_serial_control_t;

// Enums
typedef enum MAV_RESULT {
    MAV_RESULT_ACCEPTED = 0,
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    MAV_RESULT_DENIED = 2,
    MAV_RESULT_UNSUPPORTED = 3,
    MAV_RESULT_FAILED = 4,
    MAV_RESULT_IN_PROGRESS = 5
} MAV_RESULT;

typedef enum MAV_MISSION_RESULT {
    MAV_MISSION_ACCEPTED = 0,
    MAV_MISSION_ERROR = 1,
    MAV_MISSION_UNSUPPORTED_FRAME = 2,
    MAV_MISSION_UNSUPPORTED = 3,
    MAV_MISSION_NO_SPACE = 4,
    MAV_MISSION_INVALID = 5,
    MAV_MISSION_INVALID_PARAM1 = 6,
    MAV_MISSION_INVALID_PARAM2 = 7,
    MAV_MISSION_INVALID_PARAM3 = 8,
    MAV_MISSION_INVALID_PARAM4 = 9,
    MAV_MISSION_INVALID_PARAM5_X = 10,
    MAV_MISSION_INVALID_PARAM6_Y = 11,
    MAV_MISSION_INVALID_PARAM7 = 12,
    MAV_MISSION_INVALID_SEQUENCE = 13,
    MAV_MISSION_DENIED = 14
} MAV_MISSION_RESULT;

typedef enum MAV_MISSION_TYPE {
    MAV_MISSION_TYPE_MISSION = 0,
    MAV_MISSION_TYPE_FENCE = 1,
    MAV_MISSION_TYPE_RALLY = 2,
    MAV_MISSION_TYPE_ALL = 255
} MAV_MISSION_TYPE;

typedef enum MAV_CMD {
    MAV_CMD_NAV_WAYPOINT = 16,
    MAV_CMD_NAV_LOITER_UNLIM = 17,
    MAV_CMD_NAV_LOITER_TURNS = 18,
    MAV_CMD_NAV_LOITER_TIME = 19,
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    MAV_CMD_NAV_LAND = 21,
    MAV_CMD_NAV_TAKEOFF = 22,
    MAV_CMD_DO_SET_SERVO = 183,
    MAV_CMD_DO_REPEAT_SERVO = 184
} MAV_CMD;

typedef enum MAV_FRAME {
    MAV_FRAME_GLOBAL = 0,
    MAV_FRAME_LOCAL_NED = 1,
    MAV_FRAME_MISSION = 2,
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    MAV_FRAME_LOCAL_ENU = 4,
    MAV_FRAME_GLOBAL_INT = 5,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    MAV_FRAME_LOCAL_OFFSET_NED = 7,
    MAV_FRAME_BODY_NED = 8,
    MAV_FRAME_BODY_OFFSET_NED = 9,
    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
} MAV_FRAME;

typedef enum MAV_SEVERITY {
    MAV_SEVERITY_EMERGENCY = 0,
    MAV_SEVERITY_ALERT = 1,
    MAV_SEVERITY_CRITICAL = 2,
    MAV_SEVERITY_ERROR = 3,
    MAV_SEVERITY_WARNING = 4,
    MAV_SEVERITY_NOTICE = 5,
    MAV_SEVERITY_INFO = 6,
    MAV_SEVERITY_DEBUG = 7
} MAV_SEVERITY;

// Serial control device types
enum SERIAL_CONTROL_DEV {
    SERIAL_CONTROL_DEV_TELEM1 = 0,
    SERIAL_CONTROL_DEV_TELEM2 = 1,
    SERIAL_CONTROL_DEV_GPS1 = 2,
    SERIAL_CONTROL_DEV_GPS2 = 3,
    SERIAL_CONTROL_DEV_SHELL = 10
};

// Serial control flags
#define SERIAL_CONTROL_FLAG_REPLY 0x01
#define SERIAL_CONTROL_FLAG_RESPOND 0x02
#define SERIAL_CONTROL_FLAG_EXCLUSIVE 0x04
#define SERIAL_CONTROL_FLAG_BLOCKING 0x08
#define SERIAL_CONTROL_FLAG_MULTI 0x10

//=============================================================================
// Mock External Functions (required by EduCopter classes)
//=============================================================================

// Time function
static uint32_t g_mockTimeMS = 0;

inline uint32_t millis() {
    return g_mockTimeMS;
}

inline void setMockTime(uint32_t timeMS) {
    g_mockTimeMS = timeMS;
}

inline void advanceMockTime(uint32_t deltaMS) {
    g_mockTimeMS += deltaMS;
}

// System info functions (placeholders)
inline uint8_t getSystemID() { return 1; }
inline uint8_t getComponentID() { return 1; }
inline uint8_t getSystemType() { return 2; } // MAV_TYPE_QUADROTOR
inline uint8_t getAutopilotType() { return 0; } // MAV_AUTOPILOT_GENERIC

// Vehicle status functions (placeholders)
inline uint8_t getSystemStatus() { return 4; } // MAV_STATE_ACTIVE
inline uint32_t getCustomMode() { return 0; }
inline uint8_t getBaseMode() { return 0x80; } // MAV_MODE_FLAG_SAFETY_ARMED

// Battery functions (placeholders)
inline float getBatteryVoltage() { return 12.6f; }
inline float getBatteryCurrent() { return 10.5f; }
inline int8_t getBatteryRemaining() { return 75; }

// GPS functions (placeholders)
inline int32_t getGPSLatitude() { return 47363467; } // 47.363467°
inline int32_t getGPSLongitude() { return 8522225; } // 8.522225°
inline int32_t getGPSAltitude() { return 500000; } // 500m
inline uint8_t getGPSFixType() { return 3; } // 3D fix
inline uint8_t getGPSSatellites() { return 12; }

// Position functions (placeholders)
inline int32_t getLatitude() { return getGPSLatitude(); }
inline int32_t getLongitude() { return getGPSLongitude(); }
inline int32_t getAltitude() { return getGPSAltitude(); }

// Attitude functions (placeholders)
inline float getRoll() { return 0.0f; }
inline float getPitch() { return 0.0f; }
inline float getYaw() { return 0.0f; }
inline float getRollRate() { return 0.0f; }
inline float getPitchRate() { return 0.0f; }
inline float getYawRate() { return 0.0f; }

// RC functions (placeholders)
inline uint16_t getRCInput(uint8_t channel) { return 1500; }
inline uint8_t getRCChannelCount() { return 8; }

//=============================================================================
// Test Utilities
//=============================================================================

/**
 * @brief Print test header
 */
inline void printTestHeader(const char* suiteName) {
    printf("\n=== %s ===\n\n", suiteName);
}

/**
 * @brief Run a test and print result
 */
inline bool runTest(const char* testName, bool (*testFunc)()) {
    printf("Running: %s\n", testName);
    if (testFunc()) {
        printf("  PASS\n");
        return true;
    } else {
        // Error already printed by TEST_FAIL
        return false;
    }
}

/**
 * @brief Print test summary
 */
inline void printTestSummary(int total, int passed) {
    int failed = total - passed;
    printf("\n=== Test Summary ===\n");
    printf("Total:  %d\n", total);
    printf("Passed: %d\n", passed);
    printf("Failed: %d\n", failed);
    printf("Success Rate: %.1f%%\n", (100.0 * passed) / total);

    if (failed == 0) {
        printf("\n✓ All tests passed!\n\n");
    } else {
        printf("\n✗ Some tests failed!\n\n");
    }
}
