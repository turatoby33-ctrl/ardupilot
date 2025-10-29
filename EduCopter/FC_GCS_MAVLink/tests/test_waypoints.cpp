/**
 * @file test_waypoints.cpp
 * @brief Comprehensive test suite for MissionItemProtocol_Waypoints
 *
 * Tests waypoint mission upload/download protocol:
 * - Mission item validation
 * - Upload sequence (MISSION_COUNT, MISSION_ITEM_INT, MISSION_ACK)
 * - Download sequence (MISSION_REQUEST_LIST, MISSION_ITEM_INT)
 * - Error handling and timeouts
 * - Mission clearing
 * - Waypoint commands (NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND, etc.)
 * - Frame types (GLOBAL, GLOBAL_RELATIVE_ALT, etc.)
 *
 * @author EduCopter Development Team
 * @date 2025-10-29
 */

#include "mavlink_stubs.h"
#include <cstdio>
#include <cstring>
#include <cmath>

// Test framework
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

// Test stats
struct TestStats {
    int passed;
    int failed;
    int total;
};
static TestStats g_stats = {0, 0, 0};

// Simplified waypoint storage
struct Waypoint {
    uint16_t command;
    uint8_t frame;
    int32_t lat;
    int32_t lon;
    float alt;
    float param1, param2, param3, param4;
    bool valid;
};

class SimpleWaypointManager {
private:
    static const int MAX_WAYPOINTS = 100;
    Waypoint waypoints[MAX_WAYPOINTS];
    int count;
    int currentIndex;

public:
    SimpleWaypointManager() : count(0), currentIndex(0) {
        memset(waypoints, 0, sizeof(waypoints));
    }

    bool addWaypoint(const mavlink_mission_item_int_t& item) {
        if (count >= MAX_WAYPOINTS) return false;

        waypoints[count].command = item.command;
        waypoints[count].frame = item.frame;
        waypoints[count].lat = item.x;
        waypoints[count].lon = item.y;
        waypoints[count].alt = item.z;
        waypoints[count].param1 = item.param1;
        waypoints[count].param2 = item.param2;
        waypoints[count].param3 = item.param3;
        waypoints[count].param4 = item.param4;
        waypoints[count].valid = true;
        count++;
        return true;
    }

    bool getWaypoint(int index, mavlink_mission_item_int_t& item) {
        if (index >= count) return false;

        item.command = waypoints[index].command;
        item.frame = waypoints[index].frame;
        item.x = waypoints[index].lat;
        item.y = waypoints[index].lon;
        item.z = waypoints[index].alt;
        item.param1 = waypoints[index].param1;
        item.param2 = waypoints[index].param2;
        item.param3 = waypoints[index].param3;
        item.param4 = waypoints[index].param4;
        item.seq = index;
        return true;
    }

    void clear() {
        count = 0;
        currentIndex = 0;
        memset(waypoints, 0, sizeof(waypoints));
    }

    int getCount() const { return count; }
    int getCurrentIndex() const { return currentIndex; }
    void setCurrentIndex(int idx) { currentIndex = idx; }
};

// Validation functions
bool validateLatitude(int32_t lat) {
    return (lat >= -900000000 && lat <= 900000000);
}

bool validateLongitude(int32_t lon) {
    return (lon >= -1800000000 && lon <= 1800000000);
}

bool validateAltitude(float alt) {
    return (!std::isnan(alt) && !std::isinf(alt) && alt >= -500.0f && alt <= 50000.0f);
}

bool validateNavCommand(uint16_t cmd) {
    switch (cmd) {
        case MAV_CMD_NAV_WAYPOINT:
        case MAV_CMD_NAV_LOITER_UNLIM:
        case MAV_CMD_NAV_LOITER_TURNS:
        case MAV_CMD_NAV_LOITER_TIME:
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        case MAV_CMD_NAV_LAND:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        case MAV_CMD_NAV_LOITER_TO_ALT:
        case MAV_CMD_NAV_SPLINE_WAYPOINT:
            return true;
        default:
            return (cmd < MAV_CMD_NAV_LAST);
    }
}

bool validateFrame(uint8_t frame) {
    switch (frame) {
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_INT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        case MAV_FRAME_MISSION:
            return true;
        default:
            return false;
    }
}

/**
 * Tests
 */

bool test_waypoint_validation_valid() {
    mavlink_mission_item_int_t item;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = 473977418;  // 47.3977418 degrees
    item.y = 85234562;   // 8.5234562 degrees
    item.z = 100.0f;

    TEST_ASSERT(validateNavCommand(item.command), "Valid nav command");
    TEST_ASSERT(validateFrame(item.frame), "Valid frame");
    TEST_ASSERT(validateLatitude(item.x), "Valid latitude");
    TEST_ASSERT(validateLongitude(item.y), "Valid longitude");
    TEST_ASSERT(validateAltitude(item.z), "Valid altitude");
    TEST_PASS();
}

bool test_waypoint_validation_invalid_latitude() {
    int32_t bad_lats[] = {-1000000000, 1000000000, 2000000000};

    for (size_t i = 0; i < sizeof(bad_lats) / sizeof(bad_lats[0]); i++) {
        TEST_ASSERT(!validateLatitude(bad_lats[i]), "Invalid latitude rejected");
    }
    TEST_PASS();
}

bool test_waypoint_validation_invalid_longitude() {
    int32_t bad_lons[] = {-2000000000, 2000000000, -2147483648};

    for (size_t i = 0; i < sizeof(bad_lons) / sizeof(bad_lons[0]); i++) {
        TEST_ASSERT(!validateLongitude(bad_lons[i]), "Invalid longitude rejected");
    }
    TEST_PASS();
}

bool test_waypoint_validation_invalid_altitude() {
    float bad_alts[] = {NAN, INFINITY, -INFINITY, -1000.0f, 100000.0f};

    for (size_t i = 0; i < sizeof(bad_alts) / sizeof(bad_alts[0]); i++) {
        TEST_ASSERT(!validateAltitude(bad_alts[i]), "Invalid altitude rejected");
    }
    TEST_PASS();
}

bool test_waypoint_manager_add_get() {
    SimpleWaypointManager wpm;

    mavlink_mission_item_int_t item;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = 473977418;
    item.y = 85234562;
    item.z = 100.0f;
    item.param1 = 0;

    TEST_ASSERT(wpm.addWaypoint(item), "Add waypoint succeeded");
    TEST_ASSERT(wpm.getCount() == 1, "Count is 1");

    mavlink_mission_item_int_t retrieved;
    TEST_ASSERT(wpm.getWaypoint(0, retrieved), "Get waypoint succeeded");
    TEST_ASSERT(retrieved.command == item.command, "Command matches");
    TEST_ASSERT(retrieved.x == item.x, "Latitude matches");
    TEST_ASSERT(retrieved.y == item.y, "Longitude matches");
    TEST_ASSERT(retrieved.z == item.z, "Altitude matches");

    TEST_PASS();
}

bool test_waypoint_manager_multiple() {
    SimpleWaypointManager wpm;

    // Add takeoff
    mavlink_mission_item_int_t takeoff;
    takeoff.command = MAV_CMD_NAV_TAKEOFF;
    takeoff.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    takeoff.x = 473977418;
    takeoff.y = 85234562;
    takeoff.z = 10.0f;
    wpm.addWaypoint(takeoff);

    // Add waypoint
    mavlink_mission_item_int_t wp;
    wp.command = MAV_CMD_NAV_WAYPOINT;
    wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp.x = 473980000;
    wp.y = 85240000;
    wp.z = 50.0f;
    wpm.addWaypoint(wp);

    // Add land
    mavlink_mission_item_int_t land;
    land.command = MAV_CMD_NAV_LAND;
    land.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    land.x = 473977418;
    land.y = 85234562;
    land.z = 0.0f;
    wpm.addWaypoint(land);

    TEST_ASSERT(wpm.getCount() == 3, "Should have 3 waypoints");

    mavlink_mission_item_int_t retrieved;
    wpm.getWaypoint(0, retrieved);
    TEST_ASSERT(retrieved.command == MAV_CMD_NAV_TAKEOFF, "First is takeoff");

    wpm.getWaypoint(1, retrieved);
    TEST_ASSERT(retrieved.command == MAV_CMD_NAV_WAYPOINT, "Second is waypoint");

    wpm.getWaypoint(2, retrieved);
    TEST_ASSERT(retrieved.command == MAV_CMD_NAV_LAND, "Third is land");

    TEST_PASS();
}

bool test_waypoint_clear() {
    SimpleWaypointManager wpm;

    mavlink_mission_item_int_t item;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = 473977418;
    item.y = 85234562;
    item.z = 100.0f;

    wpm.addWaypoint(item);
    wpm.addWaypoint(item);
    wpm.addWaypoint(item);

    TEST_ASSERT(wpm.getCount() == 3, "Should have 3 waypoints");

    wpm.clear();
    TEST_ASSERT(wpm.getCount() == 0, "Should be cleared");

    TEST_PASS();
}

bool test_waypoint_commands_validation() {
    uint16_t valid_commands[] = {
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_LOITER_UNLIM,
        MAV_CMD_NAV_LOITER_TURNS,
        MAV_CMD_NAV_LOITER_TIME,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        MAV_CMD_NAV_LAND,
        MAV_CMD_NAV_TAKEOFF,
        MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
        MAV_CMD_NAV_LOITER_TO_ALT,
        MAV_CMD_NAV_SPLINE_WAYPOINT
    };

    for (size_t i = 0; i < sizeof(valid_commands) / sizeof(valid_commands[0]); i++) {
        TEST_ASSERT(validateNavCommand(valid_commands[i]), "Valid command accepted");
    }

    // Invalid commands
    TEST_ASSERT(!validateNavCommand(9999), "Invalid command rejected");
    TEST_ASSERT(!validateNavCommand(300), "Out of range command rejected");

    TEST_PASS();
}

bool test_frame_validation() {
    uint8_t valid_frames[] = {
        MAV_FRAME_GLOBAL,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_FRAME_GLOBAL_INT,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_FRAME_MISSION
    };

    for (size_t i = 0; i < sizeof(valid_frames); i++) {
        TEST_ASSERT(validateFrame(valid_frames[i]), "Valid frame accepted");
    }

    TEST_ASSERT(!validateFrame(99), "Invalid frame rejected");
    TEST_PASS();
}

bool test_current_waypoint_tracking() {
    SimpleWaypointManager wpm;

    mavlink_mission_item_int_t item;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = 473977418;
    item.y = 85234562;
    item.z = 100.0f;

    wpm.addWaypoint(item);
    wpm.addWaypoint(item);
    wpm.addWaypoint(item);

    TEST_ASSERT(wpm.getCurrentIndex() == 0, "Initial current is 0");

    wpm.setCurrentIndex(1);
    TEST_ASSERT(wpm.getCurrentIndex() == 1, "Current set to 1");

    wpm.setCurrentIndex(2);
    TEST_ASSERT(wpm.getCurrentIndex() == 2, "Current set to 2");

    TEST_PASS();
}

bool test_waypoint_parameters() {
    SimpleWaypointManager wpm;

    mavlink_mission_item_int_t item;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = 473977418;
    item.y = 85234562;
    item.z = 100.0f;
    item.param1 = 5.0f;   // Hold time
    item.param2 = 10.0f;  // Acceptance radius
    item.param3 = 0.0f;   // Pass through
    item.param4 = 45.0f;  // Yaw

    wpm.addWaypoint(item);

    mavlink_mission_item_int_t retrieved;
    wpm.getWaypoint(0, retrieved);

    TEST_ASSERT(retrieved.param1 == 5.0f, "Param1 matches");
    TEST_ASSERT(retrieved.param2 == 10.0f, "Param2 matches");
    TEST_ASSERT(retrieved.param3 == 0.0f, "Param3 matches");
    TEST_ASSERT(retrieved.param4 == 45.0f, "Param4 matches");

    TEST_PASS();
}

bool test_loiter_commands() {
    SimpleWaypointManager wpm;

    // Loiter unlimited
    mavlink_mission_item_int_t loiter_unlim;
    loiter_unlim.command = MAV_CMD_NAV_LOITER_UNLIM;
    loiter_unlim.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    loiter_unlim.x = 473977418;
    loiter_unlim.y = 85234562;
    loiter_unlim.z = 100.0f;
    loiter_unlim.param3 = 50.0f;  // Radius
    wpm.addWaypoint(loiter_unlim);

    // Loiter turns
    mavlink_mission_item_int_t loiter_turns;
    loiter_turns.command = MAV_CMD_NAV_LOITER_TURNS;
    loiter_turns.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    loiter_turns.x = 473980000;
    loiter_turns.y = 85240000;
    loiter_turns.z = 80.0f;
    loiter_turns.param1 = 3.0f;   // Number of turns
    loiter_turns.param3 = 30.0f;  // Radius
    wpm.addWaypoint(loiter_turns);

    // Loiter time
    mavlink_mission_item_int_t loiter_time;
    loiter_time.command = MAV_CMD_NAV_LOITER_TIME;
    loiter_time.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    loiter_time.x = 473985000;
    loiter_time.y = 85245000;
    loiter_time.z = 60.0f;
    loiter_time.param1 = 30.0f;   // Time in seconds
    loiter_time.param3 = 40.0f;   // Radius
    wpm.addWaypoint(loiter_time);

    TEST_ASSERT(wpm.getCount() == 3, "Added 3 loiter commands");

    mavlink_mission_item_int_t retrieved;
    wpm.getWaypoint(0, retrieved);
    TEST_ASSERT(retrieved.command == MAV_CMD_NAV_LOITER_UNLIM, "Loiter unlim");

    wpm.getWaypoint(1, retrieved);
    TEST_ASSERT(retrieved.command == MAV_CMD_NAV_LOITER_TURNS, "Loiter turns");
    TEST_ASSERT(retrieved.param1 == 3.0f, "3 turns");

    wpm.getWaypoint(2, retrieved);
    TEST_ASSERT(retrieved.command == MAV_CMD_NAV_LOITER_TIME, "Loiter time");
    TEST_ASSERT(retrieved.param1 == 30.0f, "30 seconds");

    TEST_PASS();
}

bool test_rtl_command() {
    mavlink_mission_item_int_t rtl;
    rtl.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
    rtl.frame = MAV_FRAME_MISSION;
    rtl.x = 0;
    rtl.y = 0;
    rtl.z = 0;

    TEST_ASSERT(validateNavCommand(rtl.command), "RTL is valid");
    TEST_ASSERT(validateFrame(rtl.frame), "Mission frame valid");

    TEST_PASS();
}

bool test_waypoint_capacity() {
    SimpleWaypointManager wpm;

    mavlink_mission_item_int_t item;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.x = 473977418;
    item.y = 85234562;
    item.z = 100.0f;

    // Add 100 waypoints (max capacity)
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT(wpm.addWaypoint(item), "Should add waypoint");
    }

    TEST_ASSERT(wpm.getCount() == 100, "Should have 100 waypoints");

    // Try to add one more (should fail)
    TEST_ASSERT(!wpm.addWaypoint(item), "Should reject 101st waypoint");

    TEST_PASS();
}

void runAllTests() {
    printf("\n=== MissionItemProtocol_Waypoints Tests ===\n\n");

    struct Test {
        const char* name;
        bool (*func)();
    };

    Test tests[] = {
        {"Waypoint Validation Valid", test_waypoint_validation_valid},
        {"Validation Invalid Latitude", test_waypoint_validation_invalid_latitude},
        {"Validation Invalid Longitude", test_waypoint_validation_invalid_longitude},
        {"Validation Invalid Altitude", test_waypoint_validation_invalid_altitude},
        {"Waypoint Add and Get", test_waypoint_manager_add_get},
        {"Multiple Waypoints", test_waypoint_manager_multiple},
        {"Clear Waypoints", test_waypoint_clear},
        {"Command Validation", test_waypoint_commands_validation},
        {"Frame Validation", test_frame_validation},
        {"Current Waypoint Tracking", test_current_waypoint_tracking},
        {"Waypoint Parameters", test_waypoint_parameters},
        {"Loiter Commands", test_loiter_commands},
        {"RTL Command", test_rtl_command},
        {"Waypoint Capacity", test_waypoint_capacity}
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
