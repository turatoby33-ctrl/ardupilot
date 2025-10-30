/**
 * @file test_routing.cpp
 * @brief Comprehensive test suite for MAVLink routing functionality
 *
 * Tests the MAVLink router's ability to:
 * - Learn routes from heartbeat messages
 * - Forward messages to appropriate destinations
 * - Handle route timeouts and cleanup
 * - Manage route table capacity
 * - Block/unblock channels
 *
 * @author EduCopter Development Team
 * @date 2025-10-28
 * @version 1.0.0
 */

#include "../MAVLink_routing.h"
#include "../GCS_MAVLink.h"
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

// Mock time function
static uint32_t g_mockTime = 0;
uint32_t millis() {
    return g_mockTime;
}

// Test statistics
struct TestStats {
    int passed;
    int failed;
    int total;
};

static TestStats g_stats = {0, 0, 0};

// Helper function to create a test message
mavlink_message_t createTestMessage(uint8_t sysid, uint8_t compid, uint32_t msgid) {
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.sysid = sysid;
    msg.compid = compid;
    msg.msgid = msgid;
    return msg;
}

// Helper function to create a heartbeat message
mavlink_message_t createHeartbeat(uint8_t sysid, uint8_t compid, uint8_t mavType) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        sysid,
        compid,
        &msg,
        mavType,
        MAV_AUTOPILOT_GENERIC,
        0, 0, 0
    );
    return msg;
}

namespace EduCopter {
namespace GCS {

/**
 * @test Test router initialization
 */
bool test_router_initialization() {
    MAVLinkRouter router;
    router.initialize();

    TEST_ASSERT(router.getRouteCount() == 0, "Route count should be 0 after init");
    TEST_ASSERT(router.getForwardedCount() == 0, "Forwarded count should be 0");

    TEST_PASS();
}

/**
 * @test Test route learning from heartbeat
 */
bool test_route_learning_from_heartbeat() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Create heartbeat from GCS (sysid=255, compid=190, type=MAV_TYPE_GCS)
    mavlink_message_t hb = createHeartbeat(255, 190, MAV_TYPE_GCS);

    // Learn route on channel 0
    router.handleHeartbeat(0, hb);

    TEST_ASSERT(router.getRouteCount() == 1, "Should have learned 1 route");

    // Verify route can be found
    uint8_t channel;
    bool found = router.findRoute(255, 190, channel);
    TEST_ASSERT(found, "Should find route for system 255, component 190");
    TEST_ASSERT(channel == 0, "Route should be on channel 0");

    TEST_PASS();
}

/**
 * @test Test learning multiple routes
 */
bool test_multiple_routes() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Learn route from GCS on channel 0
    mavlink_message_t hb1 = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.handleHeartbeat(0, hb1);

    // Learn route from companion computer on channel 1
    mavlink_message_t hb2 = createHeartbeat(1, 191, MAV_TYPE_ONBOARD_CONTROLLER);
    router.handleHeartbeat(1, hb2);

    // Learn route from camera on channel 2
    mavlink_message_t hb3 = createHeartbeat(1, 100, MAV_TYPE_CAMERA);
    router.handleHeartbeat(2, hb3);

    TEST_ASSERT(router.getRouteCount() == 3, "Should have 3 routes");

    // Verify all routes
    uint8_t channel;
    TEST_ASSERT(router.findRoute(255, 190, channel) && channel == 0, "GCS route incorrect");
    TEST_ASSERT(router.findRoute(1, 191, channel) && channel == 1, "Companion route incorrect");
    TEST_ASSERT(router.findRoute(1, 100, channel) && channel == 2, "Camera route incorrect");

    TEST_PASS();
}

/**
 * @test Test route update when same system appears on different channel
 */
bool test_route_update() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Learn GCS route on channel 0
    mavlink_message_t hb1 = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.handleHeartbeat(0, hb1);

    uint8_t channel;
    router.findRoute(255, 190, channel);
    TEST_ASSERT(channel == 0, "Initial route should be channel 0");

    // Same GCS appears on channel 1 (might have switched ports)
    g_mockTime = 2000;
    router.handleHeartbeat(1, hb1);

    // Route should be updated to channel 1
    router.findRoute(255, 190, channel);
    TEST_ASSERT(channel == 1, "Route should update to channel 1");
    TEST_ASSERT(router.getRouteCount() == 1, "Should still have only 1 route");

    TEST_PASS();
}

/**
 * @test Test finding route by MAV_TYPE
 */
bool test_find_by_mav_type() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add GCS
    mavlink_message_t hb1 = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.handleHeartbeat(0, hb1);

    // Add camera
    mavlink_message_t hb2 = createHeartbeat(1, 100, MAV_TYPE_CAMERA);
    router.handleHeartbeat(1, hb2);

    // Find GCS by type
    uint8_t sysid, compid, channel;
    bool found = router.findByMAVType(MAV_TYPE_GCS, sysid, compid, channel);

    TEST_ASSERT(found, "Should find GCS by MAV_TYPE");
    TEST_ASSERT(sysid == 255, "GCS system ID should be 255");
    TEST_ASSERT(compid == 190, "GCS component ID should be 190");
    TEST_ASSERT(channel == 0, "GCS should be on channel 0");

    // Find camera by type
    found = router.findByMAVType(MAV_TYPE_CAMERA, sysid, compid, channel);
    TEST_ASSERT(found, "Should find camera by MAV_TYPE");
    TEST_ASSERT(sysid == 1, "Camera system ID should be 1");

    // Try to find non-existent type
    found = router.findByMAVType(MAV_TYPE_SUBMARINE, sysid, compid, channel);
    TEST_ASSERT(!found, "Should not find submarine");

    TEST_PASS();
}

/**
 * @test Test route removal
 */
bool test_route_removal() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add 3 routes
    mavlink_message_t hb1 = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.handleHeartbeat(0, hb1);

    mavlink_message_t hb2 = createHeartbeat(1, 191, MAV_TYPE_ONBOARD_CONTROLLER);
    router.handleHeartbeat(1, hb2);

    mavlink_message_t hb3 = createHeartbeat(1, 100, MAV_TYPE_CAMERA);
    router.handleHeartbeat(2, hb3);

    TEST_ASSERT(router.getRouteCount() == 3, "Should have 3 routes");

    // Remove middle route
    bool removed = router.removeRoute(1, 191);
    TEST_ASSERT(removed, "Should successfully remove route");
    TEST_ASSERT(router.getRouteCount() == 2, "Should have 2 routes after removal");

    // Verify route is gone
    uint8_t channel;
    bool found = router.findRoute(1, 191, channel);
    TEST_ASSERT(!found, "Removed route should not be found");

    // Verify other routes still exist
    TEST_ASSERT(router.findRoute(255, 190, channel), "GCS route should still exist");
    TEST_ASSERT(router.findRoute(1, 100, channel), "Camera route should still exist");

    TEST_PASS();
}

/**
 * @test Test stale route removal (timeout)
 */
bool test_stale_route_removal() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add route at time 1000
    mavlink_message_t hb = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.handleHeartbeat(0, hb);

    TEST_ASSERT(router.getRouteCount() == 1, "Should have 1 route");

    // Advance time but still within timeout (30 seconds)
    g_mockTime = 20000; // 20 seconds later
    router.removeStaleRoutes(30000);
    TEST_ASSERT(router.getRouteCount() == 1, "Route should not be removed yet");

    // Advance past timeout
    g_mockTime = 35000; // 35 seconds from start
    router.removeStaleRoutes(30000);
    TEST_ASSERT(router.getRouteCount() == 0, "Stale route should be removed");

    TEST_PASS();
}

/**
 * @test Test channel blocking
 */
bool test_channel_blocking() {
    MAVLinkRouter router;
    router.initialize();

    // Block channel 2
    router.blockChannel(2);
    TEST_ASSERT(router.isChannelBlocked(2), "Channel 2 should be blocked");
    TEST_ASSERT(!router.isChannelBlocked(0), "Channel 0 should not be blocked");
    TEST_ASSERT(!router.isChannelBlocked(1), "Channel 1 should not be blocked");

    // Unblock channel 2
    router.unblockChannel(2);
    TEST_ASSERT(!router.isChannelBlocked(2), "Channel 2 should be unblocked");

    // Block multiple channels
    router.blockChannel(0);
    router.blockChannel(1);
    router.blockChannel(2);
    TEST_ASSERT(router.isChannelBlocked(0), "Channel 0 should be blocked");
    TEST_ASSERT(router.isChannelBlocked(1), "Channel 1 should be blocked");
    TEST_ASSERT(router.isChannelBlocked(2), "Channel 2 should be blocked");

    TEST_PASS();
}

/**
 * @test Test clear all routes
 */
bool test_clear_all_routes() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add multiple routes
    for (uint8_t i = 0; i < 5; i++) {
        mavlink_message_t hb = createHeartbeat(i, 0, MAV_TYPE_GENERIC);
        router.handleHeartbeat(i, hb);
    }

    TEST_ASSERT(router.getRouteCount() == 5, "Should have 5 routes");

    router.clearAllRoutes();
    TEST_ASSERT(router.getRouteCount() == 0, "All routes should be cleared");

    TEST_PASS();
}

/**
 * @test Test route table capacity
 */
bool test_route_table_capacity() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add routes up to maximum (EDUCOPTER_MAX_ROUTES = 20)
    for (uint8_t i = 0; i < 20; i++) {
        mavlink_message_t hb = createHeartbeat(i, 0, MAV_TYPE_GENERIC);
        router.handleHeartbeat(0, hb);
    }

    TEST_ASSERT(router.getRouteCount() == 20, "Should have 20 routes");

    // Try to add one more - should still be 20 (table full)
    mavlink_message_t hb = createHeartbeat(99, 0, MAV_TYPE_GENERIC);
    router.handleHeartbeat(0, hb);

    TEST_ASSERT(router.getRouteCount() == 20, "Should still have 20 routes (table full)");

    TEST_PASS();
}

/**
 * @test Test route learning from different message types
 */
bool test_route_learning_from_messages() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Heartbeat should learn route
    mavlink_message_t hb = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.learnRoute(0, hb);
    TEST_ASSERT(router.getRouteCount() == 1, "Heartbeat should learn route");

    // COMMAND_ACK should learn route
    mavlink_message_t ack = createTestMessage(1, 1, MAVLINK_MSG_ID_COMMAND_ACK);
    router.learnRoute(1, ack);
    TEST_ASSERT(router.getRouteCount() == 2, "COMMAND_ACK should learn route");

    // PARAM_VALUE should learn route
    mavlink_message_t param = createTestMessage(2, 2, MAVLINK_MSG_ID_PARAM_VALUE);
    router.learnRoute(2, param);
    TEST_ASSERT(router.getRouteCount() == 3, "PARAM_VALUE should learn route");

    // Other messages should NOT learn routes
    mavlink_message_t attitude = createTestMessage(3, 3, MAVLINK_MSG_ID_ATTITUDE);
    router.learnRoute(3, attitude);
    TEST_ASSERT(router.getRouteCount() == 3, "ATTITUDE should not learn route");

    TEST_PASS();
}

/**
 * @test Test manual route addition
 */
bool test_manual_route_addition() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Manually add route
    bool added = router.addRoute(100, 50, 3, MAV_TYPE_GIMBAL);
    TEST_ASSERT(added, "Should successfully add manual route");
    TEST_ASSERT(router.getRouteCount() == 1, "Should have 1 route");

    // Verify route
    uint8_t channel;
    bool found = router.findRoute(100, 50, channel);
    TEST_ASSERT(found, "Should find manually added route");
    TEST_ASSERT(channel == 3, "Route should be on channel 3");

    // Verify MAV_TYPE
    uint8_t sysid, compid;
    found = router.findByMAVType(MAV_TYPE_GIMBAL, sysid, compid, channel);
    TEST_ASSERT(found, "Should find by MAV_TYPE");
    TEST_ASSERT(sysid == 100 && compid == 50, "System/component IDs should match");

    TEST_PASS();
}

/**
 * @test Test wildcard component ID lookup
 */
bool test_wildcard_component_lookup() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add route for system 1, component 100
    mavlink_message_t hb = createHeartbeat(1, 100, MAV_TYPE_CAMERA);
    router.handleHeartbeat(0, hb);

    // Look up with specific component - should find
    uint8_t channel;
    bool found = router.findRoute(1, 100, channel);
    TEST_ASSERT(found, "Should find with specific component");

    // Look up with wildcard component (0) - should find
    found = router.findRoute(1, 0, channel);
    TEST_ASSERT(found, "Should find with wildcard component");
    TEST_ASSERT(channel == 0, "Channel should be 0");

    // Look up non-existent component - should not find
    found = router.findRoute(1, 200, channel);
    TEST_ASSERT(!found, "Should not find non-existent component");

    TEST_PASS();
}

/**
 * @test Test route aging and refresh
 */
bool test_route_aging_and_refresh() {
    MAVLinkRouter router;
    router.initialize();
    g_mockTime = 1000;

    // Add route at time 1000
    mavlink_message_t hb = createHeartbeat(255, 190, MAV_TYPE_GCS);
    router.handleHeartbeat(0, hb);

    // Check route exists
    const RouteEntry* entry = router.getRoute(0);
    TEST_ASSERT(entry != nullptr, "Route entry should exist");
    TEST_ASSERT(entry->lastSeenMS == 1000, "Last seen should be 1000");

    // Advance time and refresh route
    g_mockTime = 15000;
    router.handleHeartbeat(0, hb);

    // Check route was refreshed
    entry = router.getRoute(0);
    TEST_ASSERT(entry != nullptr, "Route entry should still exist");
    TEST_ASSERT(entry->lastSeenMS == 15000, "Last seen should be updated to 15000");

    TEST_PASS();
}

} // namespace GCS
} // namespace EduCopter

// Run all tests
void runAllTests() {
    using namespace EduCopter::GCS;

    printf("\n=== MAVLink Routing Test Suite ===\n\n");

    // Test list
    struct Test {
        const char* name;
        bool (*func)();
    };

    Test tests[] = {
        {"Router Initialization", test_router_initialization},
        {"Route Learning from Heartbeat", test_route_learning_from_heartbeat},
        {"Multiple Routes", test_multiple_routes},
        {"Route Update", test_route_update},
        {"Find by MAV_TYPE", test_find_by_mav_type},
        {"Route Removal", test_route_removal},
        {"Stale Route Removal", test_stale_route_removal},
        {"Channel Blocking", test_channel_blocking},
        {"Clear All Routes", test_clear_all_routes},
        {"Route Table Capacity", test_route_table_capacity},
        {"Route Learning from Messages", test_route_learning_from_messages},
        {"Manual Route Addition", test_manual_route_addition},
        {"Wildcard Component Lookup", test_wildcard_component_lookup},
        {"Route Aging and Refresh", test_route_aging_and_refresh}
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
