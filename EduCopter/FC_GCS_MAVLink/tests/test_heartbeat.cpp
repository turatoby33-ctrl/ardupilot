/**
 * @file test_heartbeat.cpp
 * @brief Comprehensive test suite for MAVLink heartbeat functionality
 *
 * Tests the heartbeat transmission and reception:
 * - Heartbeat generation and packing
 * - Heartbeat parsing and validation
 * - Heartbeat timing and intervals
 * - System status reporting
 * - Connection monitoring
 * - Timeout detection
 *
 * @author EduCopter Development Team
 * @date 2025-10-28
 * @version 1.0.0
 */

#include "../GCS_MAVLink.h"
#include "../GCS.h"
#include <cstdio>
#include <cstring>
#include <cassert>

// Test framework macros
#define TEST_ASSERT(condition, msg) \
    if (!(condition)) { \
        printf("FAIL: %s - %s\n", __func__, msg); \
        return false; \
    }

#define TEST_PASS() \
    printf("PASS: %s\n", __func__); \
    return true;

// Mock time functions
static uint32_t g_mockTime = 0;
static uint32_t g_mockTimeMicros = 0;

uint32_t millis() {
    return g_mockTime;
}

uint32_t micros() {
    return g_mockTimeMicros;
}

uint16_t millis16() {
    return static_cast<uint16_t>(g_mockTime);
}

// Mock vehicle state
static uint8_t g_baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
static uint8_t g_systemStatus = MAV_STATE_ACTIVE;
static uint32_t g_customMode = 0;

uint8_t getBaseMode() {
    return g_baseMode;
}

uint8_t getSystemStatus() {
    return g_systemStatus;
}

// Test statistics
struct TestStats {
    int passed;
    int failed;
    int total;
};

static TestStats g_stats = {0, 0, 0};

namespace EduCopter {
namespace GCS {

/**
 * @test Test heartbeat message packing
 */
bool test_heartbeat_packing() {
    mavlink_message_t msg;

    // Pack heartbeat message
    mavlink_msg_heartbeat_pack(
        1,                          // system_id
        1,                          // component_id
        &msg,
        MAV_TYPE_QUADROTOR,        // type
        MAV_AUTOPILOT_GENERIC,     // autopilot
        MAV_MODE_FLAG_SAFETY_ARMED, // base_mode
        12345,                      // custom_mode
        MAV_STATE_ACTIVE           // system_status
    );

    // Verify message was packed
    TEST_ASSERT(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT, "Message ID should be HEARTBEAT");
    TEST_ASSERT(msg.sysid == 1, "System ID should be 1");
    TEST_ASSERT(msg.compid == 1, "Component ID should be 1");

    // Decode and verify fields
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    TEST_ASSERT(heartbeat.type == MAV_TYPE_QUADROTOR, "Type should be QUADROTOR");
    TEST_ASSERT(heartbeat.autopilot == MAV_AUTOPILOT_GENERIC, "Autopilot should be GENERIC");
    TEST_ASSERT(heartbeat.base_mode == MAV_MODE_FLAG_SAFETY_ARMED, "Base mode incorrect");
    TEST_ASSERT(heartbeat.custom_mode == 12345, "Custom mode should be 12345");
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_ACTIVE, "System status should be ACTIVE");

    TEST_PASS();
}

/**
 * @test Test heartbeat message size
 */
bool test_heartbeat_message_size() {
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE
    );

    // Heartbeat payload is 9 bytes
    TEST_ASSERT(msg.len == 9, "Heartbeat payload should be 9 bytes");

    TEST_PASS();
}

/**
 * @test Test different vehicle types
 */
bool test_vehicle_types() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    uint8_t types[] = {
        MAV_TYPE_QUADROTOR,
        MAV_TYPE_HELICOPTER,
        MAV_TYPE_FIXED_WING,
        MAV_TYPE_GROUND_ROVER,
        MAV_TYPE_SUBMARINE,
        MAV_TYPE_HEXAROTOR,
        MAV_TYPE_OCTOROTOR,
        MAV_TYPE_VTOL_DUOROTOR,
        MAV_TYPE_VTOL_QUADROTOR
    };

    for (int i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        mavlink_msg_heartbeat_pack(
            1, 1, &msg,
            types[i],
            MAV_AUTOPILOT_GENERIC,
            0, 0, MAV_STATE_ACTIVE
        );

        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        TEST_ASSERT(heartbeat.type == types[i], "Vehicle type mismatch");
    }

    TEST_PASS();
}

/**
 * @test Test system states
 */
bool test_system_states() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    uint8_t states[] = {
        MAV_STATE_UNINIT,
        MAV_STATE_BOOT,
        MAV_STATE_CALIBRATING,
        MAV_STATE_STANDBY,
        MAV_STATE_ACTIVE,
        MAV_STATE_CRITICAL,
        MAV_STATE_EMERGENCY,
        MAV_STATE_POWEROFF
    };

    for (int i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
        mavlink_msg_heartbeat_pack(
            1, 1, &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            0, 0, states[i]
        );

        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        TEST_ASSERT(heartbeat.system_status == states[i], "System state mismatch");
    }

    TEST_PASS();
}

/**
 * @test Test base mode flags
 */
bool test_base_mode_flags() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // Test individual flags
    uint8_t flags[] = {
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        MAV_MODE_FLAG_TEST_ENABLED,
        MAV_MODE_FLAG_AUTO_ENABLED,
        MAV_MODE_FLAG_GUIDED_ENABLED,
        MAV_MODE_FLAG_STABILIZE_ENABLED,
        MAV_MODE_FLAG_HIL_ENABLED,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        MAV_MODE_FLAG_SAFETY_ARMED
    };

    for (int i = 0; i < sizeof(flags) / sizeof(flags[0]); i++) {
        mavlink_msg_heartbeat_pack(
            1, 1, &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            flags[i], 0, MAV_STATE_ACTIVE
        );

        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        TEST_ASSERT((heartbeat.base_mode & flags[i]) != 0, "Base mode flag not set");
    }

    // Test combined flags
    uint8_t combined = MAV_MODE_FLAG_SAFETY_ARMED |
                       MAV_MODE_FLAG_STABILIZE_ENABLED |
                       MAV_MODE_FLAG_GUIDED_ENABLED;

    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        combined, 0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    TEST_ASSERT((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0, "Armed flag missing");
    TEST_ASSERT((heartbeat.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) != 0, "Stabilize flag missing");
    TEST_ASSERT((heartbeat.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) != 0, "Guided flag missing");

    TEST_PASS();
}

/**
 * @test Test custom mode values
 */
bool test_custom_modes() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    uint32_t modes[] = {0, 1, 2, 3, 4, 5, 10, 100, 1000, 0xFFFFFFFF};

    for (int i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        mavlink_msg_heartbeat_pack(
            1, 1, &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            modes[i],
            MAV_STATE_ACTIVE
        );

        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        TEST_ASSERT(heartbeat.custom_mode == modes[i], "Custom mode mismatch");
    }

    TEST_PASS();
}

/**
 * @test Test heartbeat interval timing
 */
bool test_heartbeat_timing() {
    g_mockTime = 0;

    // Standard heartbeat interval is 1Hz (1000ms)
    const uint32_t HEARTBEAT_INTERVAL = 1000;

    uint32_t lastHeartbeatTime = 0;
    int heartbeatCount = 0;

    // Simulate 5 seconds
    for (g_mockTime = 0; g_mockTime <= 5000; g_mockTime += 100) {
        if (g_mockTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
            heartbeatCount++;
            lastHeartbeatTime = g_mockTime;
        }
    }

    // Should have sent approximately 5 heartbeats (at 0, 1000, 2000, 3000, 4000, 5000)
    TEST_ASSERT(heartbeatCount >= 5 && heartbeatCount <= 6,
                "Should send ~5-6 heartbeats in 5 seconds");

    TEST_PASS();
}

/**
 * @test Test heartbeat from different components
 */
bool test_multiple_components() {
    mavlink_message_t msg1, msg2, msg3;

    // Flight controller heartbeat
    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg1,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE
    );

    // Companion computer heartbeat
    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg2,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );

    // Camera heartbeat
    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_CAMERA, &msg3,
        MAV_TYPE_CAMERA,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );

    TEST_ASSERT(msg1.compid == MAV_COMP_ID_AUTOPILOT1, "Autopilot component ID incorrect");
    TEST_ASSERT(msg2.compid == MAV_COMP_ID_ONBOARD_COMPUTER, "Computer component ID incorrect");
    TEST_ASSERT(msg3.compid == MAV_COMP_ID_CAMERA, "Camera component ID incorrect");

    TEST_PASS();
}

/**
 * @test Test heartbeat timeout detection
 */
bool test_heartbeat_timeout() {
    g_mockTime = 1000;

    uint32_t lastHeartbeatTime = 1000;
    const uint32_t TIMEOUT_MS = 3000; // 3 second timeout

    // Check connection is good initially
    g_mockTime = 2000;
    bool timedOut = (g_mockTime - lastHeartbeatTime) > TIMEOUT_MS;
    TEST_ASSERT(!timedOut, "Should not timeout after 1 second");

    // Check after 2.5 seconds - still good
    g_mockTime = 3500;
    timedOut = (g_mockTime - lastHeartbeatTime) > TIMEOUT_MS;
    TEST_ASSERT(!timedOut, "Should not timeout after 2.5 seconds");

    // Check after 4 seconds - should timeout
    g_mockTime = 5000;
    timedOut = (g_mockTime - lastHeartbeatTime) > TIMEOUT_MS;
    TEST_ASSERT(timedOut, "Should timeout after 4 seconds");

    TEST_PASS();
}

/**
 * @test Test MAVLink version field
 */
bool test_mavlink_version() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    // MAVLink version should be 3 (for MAVLink 2.0)
    TEST_ASSERT(heartbeat.mavlink_version == 3, "MAVLink version should be 3");

    TEST_PASS();
}

/**
 * @test Test heartbeat from GCS
 */
bool test_gcs_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // GCS heartbeat
    mavlink_msg_heartbeat_pack(
        255, 190, &msg,  // Typical GCS system/component IDs
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    TEST_ASSERT(msg.sysid == 255, "GCS system ID typically 255");
    TEST_ASSERT(msg.compid == 190, "GCS component ID typically 190");
    TEST_ASSERT(heartbeat.type == MAV_TYPE_GCS, "Type should be GCS");
    TEST_ASSERT(heartbeat.autopilot == MAV_AUTOPILOT_INVALID, "GCS has no autopilot");

    TEST_PASS();
}

/**
 * @test Test armed/disarmed detection
 */
bool test_armed_detection() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // Test disarmed
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0,  // No armed flag
        0, MAV_STATE_STANDBY
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    bool armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    TEST_ASSERT(!armed, "Should be disarmed");
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_STANDBY, "Should be in standby");

    // Test armed
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_SAFETY_ARMED,
        0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    TEST_ASSERT(armed, "Should be armed");
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_ACTIVE, "Should be active");

    TEST_PASS();
}

/**
 * @test Test flight mode detection
 */
bool test_flight_mode_detection() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // Test manual mode
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED,
        0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    bool manual = (heartbeat.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) != 0;
    bool stabilize = (heartbeat.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) != 0;
    TEST_ASSERT(manual, "Manual mode should be enabled");
    TEST_ASSERT(stabilize, "Stabilize should be enabled");

    // Test auto mode
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED,
        0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    bool autoMode = (heartbeat.base_mode & MAV_MODE_FLAG_AUTO_ENABLED) != 0;
    TEST_ASSERT(autoMode, "Auto mode should be enabled");

    // Test guided mode
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED,
        0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    bool guided = (heartbeat.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) != 0;
    TEST_ASSERT(guided, "Guided mode should be enabled");

    TEST_PASS();
}

/**
 * @test Test connection quality based on heartbeat intervals
 */
bool test_connection_quality() {
    g_mockTime = 0;

    uint32_t heartbeatTimes[10];
    uint32_t intervals[9];

    // Simulate receiving 10 heartbeats with varying intervals
    for (int i = 0; i < 10; i++) {
        heartbeatTimes[i] = g_mockTime;
        g_mockTime += 1000 + (i * 50); // Gradually increasing jitter
    }

    // Calculate intervals
    for (int i = 0; i < 9; i++) {
        intervals[i] = heartbeatTimes[i + 1] - heartbeatTimes[i];
    }

    // Check first few intervals are good (around 1000ms)
    TEST_ASSERT(intervals[0] >= 900 && intervals[0] <= 1100, "First interval should be ~1000ms");
    TEST_ASSERT(intervals[1] >= 950 && intervals[1] <= 1150, "Second interval should be ~1050ms");

    // Later intervals should have more jitter
    TEST_ASSERT(intervals[8] > 1300, "Last interval should show degradation");

    TEST_PASS();
}

/**
 * @test Test emergency state detection
 */
bool test_emergency_state() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // Normal operation
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_SAFETY_ARMED,
        0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_ACTIVE, "Should be active");

    // Critical state
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_SAFETY_ARMED,
        0, MAV_STATE_CRITICAL
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_CRITICAL, "Should be critical");

    // Emergency state
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_SAFETY_ARMED,
        0, MAV_STATE_EMERGENCY
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_EMERGENCY, "Should be emergency");

    TEST_PASS();
}

} // namespace GCS
} // namespace EduCopter

// Run all tests
void runAllTests() {
    using namespace EduCopter::GCS;

    printf("\n=== MAVLink Heartbeat Test Suite ===\n\n");

    // Test list
    struct Test {
        const char* name;
        bool (*func)();
    };

    Test tests[] = {
        {"Heartbeat Packing", test_heartbeat_packing},
        {"Heartbeat Message Size", test_heartbeat_message_size},
        {"Vehicle Types", test_vehicle_types},
        {"System States", test_system_states},
        {"Base Mode Flags", test_base_mode_flags},
        {"Custom Modes", test_custom_modes},
        {"Heartbeat Timing", test_heartbeat_timing},
        {"Multiple Components", test_multiple_components},
        {"Heartbeat Timeout", test_heartbeat_timeout},
        {"MAVLink Version", test_mavlink_version},
        {"GCS Heartbeat", test_gcs_heartbeat},
        {"Armed Detection", test_armed_detection},
        {"Flight Mode Detection", test_flight_mode_detection},
        {"Connection Quality", test_connection_quality},
        {"Emergency State", test_emergency_state}
    };

    int numTests = sizeof(tests) / sizeof(Test);

    // Run each test
    for (int i = 0; i < numTests; i++) {
        g_stats.total++;
        if (tests[i].func()) {
            g_stats.passed++;
        } else {
            g_stats.failed++;
        }
    }

    // Print summary
    printf("\n=== Test Summary ===\n");
    printf("Total:  %d\n", g_stats.total);
    printf("Passed: %d\n", g_stats.passed);
    printf("Failed: %d\n", g_stats.failed);
    printf("Success Rate: %.1f%%\n",
           (g_stats.passed * 100.0) / g_stats.total);

    if (g_stats.failed == 0) {
        printf("\n✓ All tests passed!\n\n");
    } else {
        printf("\n✗ Some tests failed!\n\n");
    }
}

// Main entry point
int main(int argc, char** argv) {
    runAllTests();
    return (g_stats.failed == 0) ? 0 : 1;
}
