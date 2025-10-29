/**
 * @file test_heartbeat_standalone.cpp
 * @brief Standalone heartbeat tests (simplified for testing without full dependencies)
 */

#include "mavlink_stubs.h"
#include <cstdio>
#include <cstring>

// Test framework macros
#define TEST_ASSERT(condition, msg) \
    if (!(condition)) { \
        printf("FAIL: %s - %s\n", __func__, msg); \
        return false; \
    }

#define TEST_PASS() \
    printf("PASS: %s\n", __func__); \
    return true;

// Mock time
static uint32_t g_mockTime = 0;
uint32_t millis() { return g_mockTime; }

// Test statistics
struct TestStats {
    int passed;
    int failed;
    int total;
};

static TestStats g_stats = {0, 0, 0};

/**
 * Test functions
 */

bool test_heartbeat_packing() {
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_SAFETY_ARMED,
        12345,
        MAV_STATE_ACTIVE
    );

    TEST_ASSERT(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT, "Message ID should be HEARTBEAT");
    TEST_ASSERT(msg.sysid == 1, "System ID should be 1");
    TEST_ASSERT(msg.compid == 1, "Component ID should be 1");

    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    TEST_ASSERT(heartbeat.type == MAV_TYPE_QUADROTOR, "Type should be QUADROTOR");
    TEST_ASSERT(heartbeat.autopilot == MAV_AUTOPILOT_GENERIC, "Autopilot GENERIC");
    TEST_ASSERT(heartbeat.base_mode == MAV_MODE_FLAG_SAFETY_ARMED, "Base mode");
    TEST_ASSERT(heartbeat.custom_mode == 12345, "Custom mode 12345");
    TEST_ASSERT(heartbeat.system_status == MAV_STATE_ACTIVE, "Status ACTIVE");

    TEST_PASS();
}

bool test_heartbeat_message_size() {
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE
    );

    TEST_ASSERT(msg.len == 9, "Heartbeat payload is 9 bytes");
    TEST_PASS();
}

bool test_vehicle_types() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    uint8_t types[] = {
        MAV_TYPE_QUADROTOR,
        MAV_TYPE_HELICOPTER,
        MAV_TYPE_FIXED_WING,
        MAV_TYPE_GROUND_ROVER,
        MAV_TYPE_GCS
    };

    for (size_t i = 0; i < sizeof(types); i++) {
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

    for (size_t i = 0; i < sizeof(states); i++) {
        mavlink_msg_heartbeat_pack(
            1, 1, &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            0, 0, states[i]
        );

        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        TEST_ASSERT(heartbeat.system_status == states[i], "State mismatch");
    }

    TEST_PASS();
}

bool test_base_mode_flags() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    uint8_t flags[] = {
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        MAV_MODE_FLAG_AUTO_ENABLED,
        MAV_MODE_FLAG_GUIDED_ENABLED,
        MAV_MODE_FLAG_STABILIZE_ENABLED,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        MAV_MODE_FLAG_SAFETY_ARMED
    };

    for (size_t i = 0; i < sizeof(flags); i++) {
        mavlink_msg_heartbeat_pack(
            1, 1, &msg,
            MAV_TYPE_QUADROTOR,
            MAV_AUTOPILOT_GENERIC,
            flags[i], 0, MAV_STATE_ACTIVE
        );

        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        TEST_ASSERT((heartbeat.base_mode & flags[i]) != 0, "Flag not set");
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
    TEST_ASSERT((heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0, "Armed");
    TEST_ASSERT((heartbeat.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED) != 0, "Stabilize");
    TEST_ASSERT((heartbeat.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED) != 0, "Guided");

    TEST_PASS();
}

bool test_custom_modes() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    uint32_t modes[] = {0, 1, 5, 100, 1000, 0xFFFFFFFF};

    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
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

bool test_heartbeat_timing() {
    g_mockTime = 0;
    const uint32_t HEARTBEAT_INTERVAL = 1000;

    uint32_t lastHeartbeatTime = 0;
    int heartbeatCount = 0;

    for (g_mockTime = 0; g_mockTime <= 5000; g_mockTime += 100) {
        if (g_mockTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
            heartbeatCount++;
            lastHeartbeatTime = g_mockTime;
        }
    }

    TEST_ASSERT(heartbeatCount >= 5 && heartbeatCount <= 6, "Should send ~5-6 heartbeats in 5s");
    TEST_PASS();
}

bool test_multiple_components() {
    mavlink_message_t msg1, msg2, msg3;

    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_AUTOPILOT1, &msg1,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0, 0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg2,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_CAMERA, &msg3,
        MAV_TYPE_CAMERA,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );

    TEST_ASSERT(msg1.compid == MAV_COMP_ID_AUTOPILOT1, "Autopilot compid");
    TEST_ASSERT(msg2.compid == MAV_COMP_ID_ONBOARD_COMPUTER, "Computer compid");
    TEST_ASSERT(msg3.compid == MAV_COMP_ID_CAMERA, "Camera compid");

    TEST_PASS();
}

bool test_heartbeat_timeout() {
    g_mockTime = 1000;
    uint32_t lastHeartbeatTime = 1000;
    const uint32_t TIMEOUT_MS = 3000;

    g_mockTime = 2000;
    bool timedOut = (g_mockTime - lastHeartbeatTime) > TIMEOUT_MS;
    TEST_ASSERT(!timedOut, "Should not timeout after 1 second");

    g_mockTime = 5000;
    timedOut = (g_mockTime - lastHeartbeatTime) > TIMEOUT_MS;
    TEST_ASSERT(timedOut, "Should timeout after 4 seconds");

    TEST_PASS();
}

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
    TEST_ASSERT(heartbeat.mavlink_version == 3, "MAVLink version should be 3");

    TEST_PASS();
}

bool test_gcs_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    mavlink_msg_heartbeat_pack(
        255, 190, &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    TEST_ASSERT(msg.sysid == 255, "GCS sysid 255");
    TEST_ASSERT(msg.compid == 190, "GCS compid 190");
    TEST_ASSERT(heartbeat.type == MAV_TYPE_GCS, "Type GCS");
    TEST_ASSERT(heartbeat.autopilot == MAV_AUTOPILOT_INVALID, "No autopilot");

    TEST_PASS();
}

bool test_armed_detection() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    // Disarmed
    mavlink_msg_heartbeat_pack(
        1, 1, &msg,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_GENERIC,
        0,
        0, MAV_STATE_STANDBY
    );

    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    bool armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    TEST_ASSERT(!armed, "Should be disarmed");

    // Armed
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

    TEST_PASS();
}

void runAllTests() {
    printf("\n=== Simplified MAVLink Heartbeat Tests ===\n\n");

    struct Test {
        const char* name;
        bool (*func)();
    };

    Test tests[] = {
        {"Heartbeat Packing", test_heartbeat_packing},
        {"Message Size", test_heartbeat_message_size},
        {"Vehicle Types", test_vehicle_types},
        {"System States", test_system_states},
        {"Base Mode Flags", test_base_mode_flags},
        {"Custom Modes", test_custom_modes},
        {"Heartbeat Timing", test_heartbeat_timing},
        {"Multiple Components", test_multiple_components},
        {"Heartbeat Timeout", test_heartbeat_timeout},
        {"MAVLink Version", test_mavlink_version},
        {"GCS Heartbeat", test_gcs_heartbeat},
        {"Armed Detection", test_armed_detection}
    };

    int numTests = sizeof(tests) / sizeof(Test);

    for (int i = 0; i < numTests; i++) {
        g_stats.total++;
        if (tests[i].func()) {
            g_stats.passed++;
        } else {
            g_stats.failed++;
        }
    }

    printf("\n=== Test Summary ===\n");
    printf("Total:  %d\n", g_stats.total);
    printf("Passed: %d\n", g_stats.passed);
    printf("Failed: %d\n", g_stats.failed);
    printf("Success Rate: %.1f%%\n", (g_stats.passed * 100.0) / g_stats.total);

    if (g_stats.failed == 0) {
        printf("\n✓ All tests passed!\n\n");
    } else {
        printf("\n✗ Some tests failed!\n\n");
    }
}

int main(int argc, char** argv) {
    runAllTests();
    return (g_stats.failed == 0) ? 0 : 1;
}
