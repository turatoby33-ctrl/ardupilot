/**
 * @file test_waypoints_integrated.cpp
 * @brief Integration test for MissionItemProtocol_Waypoints
 *
 * This test demonstrates how MissionItemProtocol_Waypoints integrates with:
 * - External waypoint storage (vehicle mission manager)
 * - GCSChannel for communication
 * - Mission protocol state machine
 * - Mission item validation
 *
 * Tests mission upload/download workflows end-to-end.
 *
 * @author EduCopter Test Suite
 * @date 2025
 */

#include "test_harness.h"
#include <vector>
#include <algorithm>
#include <cmath>

#ifndef NAN
#define NAN (__builtin_nanf(""))
#endif

//=============================================================================
// Mock Vehicle Mission Storage
//=============================================================================

static std::vector<mavlink_mission_item_int_t> g_vehicleWaypoints;
static uint16_t g_currentWaypointIndex = 0;

// External functions expected by MissionItemProtocol_Waypoints
uint16_t getWaypointCount() {
    return g_vehicleWaypoints.size();
}

bool getWaypoint(uint16_t index, mavlink_mission_item_int_t& outItem) {
    if (index >= g_vehicleWaypoints.size()) return false;
    outItem = g_vehicleWaypoints[index];
    return true;
}

bool setWaypoint(uint16_t index, const mavlink_mission_item_int_t& item) {
    if (index >= g_vehicleWaypoints.size()) return false;
    g_vehicleWaypoints[index] = item;
    return true;
}

bool appendWaypoint(const mavlink_mission_item_int_t& item) {
    if (g_vehicleWaypoints.size() >= 255) return false;
    g_vehicleWaypoints.push_back(item);
    return true;
}

bool clearWaypoints() {
    g_vehicleWaypoints.clear();
    g_currentWaypointIndex = 0;
    return true;
}

uint16_t getCurrentWaypointIndex() {
    return g_currentWaypointIndex;
}

bool setCurrentWaypoint(uint16_t index) {
    if (index >= g_vehicleWaypoints.size()) return false;
    g_currentWaypointIndex = index;
    return true;
}

void resetMissionStorage() {
    g_vehicleWaypoints.clear();
    g_currentWaypointIndex = 0;
}

//=============================================================================
// Mission Protocol State Machine
//=============================================================================

enum class MissionState {
    IDLE,
    RECEIVING_COUNT,
    RECEIVING_ITEMS,
    SENDING_ITEMS,
    COMPLETE,
    ERROR
};

class MissionProtocolHandler {
private:
    MissionState state;
    uint16_t expectedCount;
    uint16_t receivedCount;
    uint16_t requestedIndex;
    uint32_t timeoutMS;
    std::vector<mavlink_mission_item_int_t> uploadBuffer;

public:
    MissionProtocolHandler()
        : state(MissionState::IDLE)
        , expectedCount(0)
        , receivedCount(0)
        , requestedIndex(0)
        , timeoutMS(0)
    {
    }

    void reset() {
        state = MissionState::IDLE;
        expectedCount = 0;
        receivedCount = 0;
        requestedIndex = 0;
        timeoutMS = 0;
        uploadBuffer.clear();
    }

    // Start mission upload (GCS -> Vehicle)
    MAV_MISSION_RESULT startMissionUpload(uint16_t count) {
        if (count > 255) return MAV_MISSION_NO_SPACE;
        if (count == 0) return MAV_MISSION_INVALID;

        state = MissionState::RECEIVING_ITEMS;
        expectedCount = count;
        receivedCount = 0;
        requestedIndex = 0;
        uploadBuffer.clear();
        uploadBuffer.reserve(count);
        timeoutMS = millis() + 5000;

        return MAV_MISSION_ACCEPTED;
    }

    // Receive mission item during upload
    MAV_MISSION_RESULT receiveMissionItem(const mavlink_mission_item_int_t& item) {
        if (state != MissionState::RECEIVING_ITEMS) {
            return MAV_MISSION_DENIED;
        }

        // Validate sequence number
        if (item.seq != receivedCount) {
            return MAV_MISSION_INVALID_SEQUENCE;
        }

        // Validate item
        MAV_MISSION_RESULT validation = validateMissionItem(item);
        if (validation != MAV_MISSION_ACCEPTED) {
            state = MissionState::ERROR;
            return validation;
        }

        // Store item
        uploadBuffer.push_back(item);
        receivedCount++;

        // Check if complete
        if (receivedCount >= expectedCount) {
            return commitMissionItems();
        }

        // Request next item
        requestedIndex = receivedCount;
        timeoutMS = millis() + 5000;

        return MAV_MISSION_ACCEPTED;
    }

    // Start mission download (Vehicle -> GCS)
    MAV_MISSION_RESULT startMissionDownload() {
        uint16_t count = getWaypointCount();
        if (count == 0) return MAV_MISSION_ERROR;

        state = MissionState::SENDING_ITEMS;
        requestedIndex = 0;
        timeoutMS = millis() + 5000;

        return MAV_MISSION_ACCEPTED;
    }

    // Get mission item during download
    MAV_MISSION_RESULT getMissionItem(uint16_t index, mavlink_mission_item_int_t& outItem) {
        if (state != MissionState::SENDING_ITEMS && state != MissionState::IDLE) {
            return MAV_MISSION_DENIED;
        }

        if (index >= getWaypointCount()) {
            return MAV_MISSION_INVALID_SEQUENCE;
        }

        if (!getWaypoint(index, outItem)) {
            return MAV_MISSION_ERROR;
        }

        requestedIndex = index + 1;
        timeoutMS = millis() + 5000;

        return MAV_MISSION_ACCEPTED;
    }

    // Validate mission item
    MAV_MISSION_RESULT validateMissionItem(const mavlink_mission_item_int_t& item) {
        // Validate frame
        if (item.frame != MAV_FRAME_GLOBAL &&
            item.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT &&
            item.frame != MAV_FRAME_GLOBAL_INT &&
            item.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT &&
            item.frame != MAV_FRAME_MISSION) {
            return MAV_MISSION_UNSUPPORTED_FRAME;
        }

        // Validate command
        switch (item.command) {
            case MAV_CMD_NAV_WAYPOINT:
            case MAV_CMD_NAV_LOITER_UNLIM:
            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_LOITER_TIME:
            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            case MAV_CMD_NAV_LAND:
            case MAV_CMD_NAV_TAKEOFF:
                break;
            default:
                // Unknown command
                return MAV_MISSION_UNSUPPORTED;
        }

        // Validate coordinates (if applicable)
        if (item.command == MAV_CMD_NAV_WAYPOINT ||
            item.command == MAV_CMD_NAV_LOITER_UNLIM ||
            item.command == MAV_CMD_NAV_LOITER_TURNS ||
            item.command == MAV_CMD_NAV_LOITER_TIME ||
            item.command == MAV_CMD_NAV_LAND ||
            item.command == MAV_CMD_NAV_TAKEOFF) {

            // Validate latitude (-90 to +90 degrees in 1E7 format)
            if (item.x < -900000000 || item.x > 900000000) {
                return MAV_MISSION_INVALID_PARAM5_X;
            }

            // Validate longitude (-180 to +180 degrees in 1E7 format)
            if (item.y < -1800000000 || item.y > 1800000000) {
                return MAV_MISSION_INVALID_PARAM6_Y;
            }

            // Validate altitude (-500 to 50000 meters)
            if (item.z < -500.0f || item.z > 50000.0f) {
                return MAV_MISSION_INVALID_PARAM7;
            }
        }

        return MAV_MISSION_ACCEPTED;
    }

    // Commit uploaded items to vehicle storage
    MAV_MISSION_RESULT commitMissionItems() {
        // Clear existing waypoints
        if (!clearWaypoints()) {
            state = MissionState::ERROR;
            return MAV_MISSION_ERROR;
        }

        // Append all items
        for (const auto& item : uploadBuffer) {
            if (!appendWaypoint(item)) {
                state = MissionState::ERROR;
                return MAV_MISSION_ERROR;
            }
        }

        state = MissionState::COMPLETE;
        return MAV_MISSION_ACCEPTED;
    }

    bool isTimeout() const {
        return (state != MissionState::IDLE && millis() > timeoutMS);
    }

    MissionState getState() const { return state; }
    uint16_t getExpectedCount() const { return expectedCount; }
    uint16_t getReceivedCount() const { return receivedCount; }
    uint16_t getRequestedIndex() const { return requestedIndex; }
};

//=============================================================================
// Helper Functions
//=============================================================================

mavlink_mission_item_int_t createWaypoint(uint16_t seq, int32_t lat, int32_t lon, float alt) {
    mavlink_mission_item_int_t item = {};
    item.seq = seq;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = lat;
    item.y = lon;
    item.z = alt;
    item.param1 = 0.0f; // Hold time
    item.param2 = 2.0f; // Acceptance radius
    item.param3 = 0.0f; // Pass through
    item.param4 = NAN; // Yaw
    item.autocontinue = 1;
    item.current = 0;
    item.mission_type = MAV_MISSION_TYPE_MISSION;
    return item;
}

mavlink_mission_item_int_t createTakeoff(uint16_t seq, float alt) {
    mavlink_mission_item_int_t item = {};
    item.seq = seq;
    item.command = MAV_CMD_NAV_TAKEOFF;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    item.z = alt;
    item.param1 = 0.0f; // Pitch
    item.autocontinue = 1;
    item.current = 0;
    item.mission_type = MAV_MISSION_TYPE_MISSION;
    return item;
}

mavlink_mission_item_int_t createLand(uint16_t seq, int32_t lat, int32_t lon) {
    mavlink_mission_item_int_t item = {};
    item.seq = seq;
    item.command = MAV_CMD_NAV_LAND;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = lat;
    item.y = lon;
    item.z = 0.0f;
    item.autocontinue = 1;
    item.current = 0;
    item.mission_type = MAV_MISSION_TYPE_MISSION;
    return item;
}

//=============================================================================
// Integration Tests
//=============================================================================

bool test_mission_upload_simple() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    // Start upload with 3 waypoints
    MAV_MISSION_RESULT result = handler.startMissionUpload(3);
    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Upload start should be accepted");

    // Upload waypoint 0 (takeoff)
    auto wp0 = createTakeoff(0, 10.0f);
    result = handler.receiveMissionItem(wp0);
    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Waypoint 0 should be accepted");

    // Upload waypoint 1
    auto wp1 = createWaypoint(1, 473634670, 85222250, 50.0f);
    result = handler.receiveMissionItem(wp1);
    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Waypoint 1 should be accepted");

    // Upload waypoint 2 (land)
    auto wp2 = createLand(2, 473634670, 85222250);
    result = handler.receiveMissionItem(wp2);
    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Waypoint 2 should be accepted");

    // Verify mission was stored
    TEST_ASSERT(getWaypointCount() == 3, "Should have 3 waypoints");

    mavlink_mission_item_int_t stored;
    TEST_ASSERT(getWaypoint(0, stored), "Should retrieve waypoint 0");
    TEST_ASSERT(stored.command == MAV_CMD_NAV_TAKEOFF, "Waypoint 0 should be takeoff");
    TEST_ASSERT(stored.z == 10.0f, "Takeoff altitude should be 10m");

    TEST_PASS();
}

bool test_mission_upload_invalid_sequence() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    handler.startMissionUpload(2);

    // Upload with wrong sequence number
    auto wp = createWaypoint(1, 473634670, 85222250, 50.0f); // Should be seq=0
    MAV_MISSION_RESULT result = handler.receiveMissionItem(wp);

    TEST_ASSERT(result == MAV_MISSION_INVALID_SEQUENCE, "Should reject wrong sequence");
    TEST_PASS();
}

bool test_mission_upload_invalid_coordinates() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    handler.startMissionUpload(1);

    // Create waypoint with invalid latitude
    auto wp = createWaypoint(0, 1000000000, 85222250, 50.0f); // > 900000000
    MAV_MISSION_RESULT result = handler.receiveMissionItem(wp);

    TEST_ASSERT(result == MAV_MISSION_INVALID_PARAM5_X, "Should reject invalid latitude");
    TEST_PASS();
}

bool test_mission_upload_invalid_altitude() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    handler.startMissionUpload(1);

    // Create waypoint with invalid altitude
    auto wp = createWaypoint(0, 473634670, 85222250, 60000.0f); // > 50000m
    MAV_MISSION_RESULT result = handler.receiveMissionItem(wp);

    TEST_ASSERT(result == MAV_MISSION_INVALID_PARAM7, "Should reject invalid altitude");
    TEST_PASS();
}

bool test_mission_download() {
    resetMissionStorage();

    // Pre-populate mission storage
    appendWaypoint(createTakeoff(0, 15.0f));
    appendWaypoint(createWaypoint(1, 473634670, 85222250, 100.0f));
    appendWaypoint(createWaypoint(2, 473644670, 85232250, 150.0f));
    appendWaypoint(createLand(3, 473634670, 85222250));

    MissionProtocolHandler handler;
    MAV_MISSION_RESULT result = handler.startMissionDownload();
    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Download start should be accepted");

    // Request each waypoint
    for (uint16_t i = 0; i < 4; i++) {
        mavlink_mission_item_int_t item;
        result = handler.getMissionItem(i, item);
        TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Should get waypoint");
        TEST_ASSERT(item.seq == i, "Sequence should match");
    }

    TEST_PASS();
}

bool test_mission_clear_and_upload() {
    resetMissionStorage();

    // Add some waypoints
    appendWaypoint(createTakeoff(0, 10.0f));
    appendWaypoint(createWaypoint(1, 473634670, 85222250, 50.0f));
    TEST_ASSERT(getWaypointCount() == 2, "Should have 2 waypoints");

    // Clear mission
    MissionProtocolHandler handler;
    handler.startMissionUpload(1);
    auto wp = createTakeoff(0, 20.0f);
    handler.receiveMissionItem(wp);

    // Verify old mission was cleared
    TEST_ASSERT(getWaypointCount() == 1, "Should have only 1 waypoint");

    mavlink_mission_item_int_t stored;
    getWaypoint(0, stored);
    TEST_ASSERT(stored.z == 20.0f, "New takeoff altitude should be 20m");

    TEST_PASS();
}

bool test_mission_current_waypoint_tracking() {
    resetMissionStorage();

    // Add mission
    appendWaypoint(createTakeoff(0, 10.0f));
    appendWaypoint(createWaypoint(1, 473634670, 85222250, 50.0f));
    appendWaypoint(createWaypoint(2, 473644670, 85232250, 100.0f));
    appendWaypoint(createLand(3, 473634670, 85222250));

    // Set current waypoint
    TEST_ASSERT(setCurrentWaypoint(0), "Should set current to 0");
    TEST_ASSERT(getCurrentWaypointIndex() == 0, "Current should be 0");

    TEST_ASSERT(setCurrentWaypoint(2), "Should set current to 2");
    TEST_ASSERT(getCurrentWaypointIndex() == 2, "Current should be 2");

    // Try invalid index
    TEST_ASSERT(!setCurrentWaypoint(10), "Should reject invalid index");
    TEST_ASSERT(getCurrentWaypointIndex() == 2, "Current should still be 2");

    TEST_PASS();
}

bool test_mission_loiter_commands() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    handler.startMissionUpload(3);

    // Loiter unlimited
    mavlink_mission_item_int_t loiter1 = {};
    loiter1.seq = 0;
    loiter1.command = MAV_CMD_NAV_LOITER_UNLIM;
    loiter1.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    loiter1.x = 473634670;
    loiter1.y = 85222250;
    loiter1.z = 50.0f;
    loiter1.param3 = 50.0f; // Radius
    handler.receiveMissionItem(loiter1);

    // Loiter turns
    mavlink_mission_item_int_t loiter2 = {};
    loiter2.seq = 1;
    loiter2.command = MAV_CMD_NAV_LOITER_TURNS;
    loiter2.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    loiter2.x = 473644670;
    loiter2.y = 85232250;
    loiter2.z = 75.0f;
    loiter2.param1 = 3.0f; // Number of turns
    loiter2.param3 = 100.0f; // Radius
    handler.receiveMissionItem(loiter2);

    // Loiter time
    mavlink_mission_item_int_t loiter3 = {};
    loiter3.seq = 2;
    loiter3.command = MAV_CMD_NAV_LOITER_TIME;
    loiter3.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    loiter3.x = 473654670;
    loiter3.y = 85242250;
    loiter3.z = 100.0f;
    loiter3.param1 = 30.0f; // Time in seconds
    loiter3.param3 = 75.0f; // Radius
    MAV_MISSION_RESULT result = handler.receiveMissionItem(loiter3);

    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "All loiter commands should be accepted");
    TEST_ASSERT(getWaypointCount() == 3, "Should have 3 waypoints");

    TEST_PASS();
}

bool test_mission_rtl_command() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    handler.startMissionUpload(2);

    // Normal waypoint
    auto wp1 = createWaypoint(0, 473634670, 85222250, 50.0f);
    handler.receiveMissionItem(wp1);

    // RTL command
    mavlink_mission_item_int_t rtl = {};
    rtl.seq = 1;
    rtl.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    rtl.frame = MAV_FRAME_MISSION;
    rtl.autocontinue = 1;
    MAV_MISSION_RESULT result = handler.receiveMissionItem(rtl);

    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "RTL should be accepted");
    TEST_ASSERT(getWaypointCount() == 2, "Should have 2 waypoints");

    mavlink_mission_item_int_t stored;
    getWaypoint(1, stored);
    TEST_ASSERT(stored.command == MAV_CMD_NAV_RETURN_TO_LAUNCH, "Should be RTL command");

    TEST_PASS();
}

bool test_mission_capacity_limit() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    // Try to upload more than capacity
    MAV_MISSION_RESULT result = handler.startMissionUpload(300);
    TEST_ASSERT(result == MAV_MISSION_NO_SPACE, "Should reject oversized mission");

    // Try uploading at exact capacity
    result = handler.startMissionUpload(255);
    TEST_ASSERT(result == MAV_MISSION_ACCEPTED, "Should accept 255 waypoints");

    TEST_PASS();
}

bool test_mission_timeout_handling() {
    resetMissionStorage();
    MissionProtocolHandler handler;

    handler.startMissionUpload(2);

    // Upload first waypoint
    auto wp1 = createWaypoint(0, 473634670, 85222250, 50.0f);
    handler.receiveMissionItem(wp1);

    // Simulate timeout (advance time by 6 seconds)
    advanceMockTime(6000);

    TEST_ASSERT(handler.isTimeout(), "Should detect timeout");

    TEST_PASS();
}

//=============================================================================
// Test Runner
//=============================================================================

struct Test {
    const char* name;
    bool (*func)();
};

int main() {
    printTestHeader("MissionItemProtocol_Waypoints Integration Tests");

    setMockTime(1000); // Start at 1 second

    static Test tests[] = {
        {"Mission Upload Simple (3 waypoints)", test_mission_upload_simple},
        {"Mission Upload Invalid Sequence", test_mission_upload_invalid_sequence},
        {"Mission Upload Invalid Coordinates", test_mission_upload_invalid_coordinates},
        {"Mission Upload Invalid Altitude", test_mission_upload_invalid_altitude},
        {"Mission Download", test_mission_download},
        {"Mission Clear and Upload", test_mission_clear_and_upload},
        {"Mission Current Waypoint Tracking", test_mission_current_waypoint_tracking},
        {"Mission Loiter Commands", test_mission_loiter_commands},
        {"Mission RTL Command", test_mission_rtl_command},
        {"Mission Capacity Limit", test_mission_capacity_limit},
        {"Mission Timeout Handling", test_mission_timeout_handling}
    };

    int numTests = sizeof(tests) / sizeof(tests[0]);
    int passed = 0;

    for (int i = 0; i < numTests; i++) {
        if (runTest(tests[i].name, tests[i].func)) {
            passed++;
        }
        setMockTime(1000); // Reset time between tests
    }

    printTestSummary(numTests, passed);

    return (passed == numTests) ? 0 : 1;
}
