/**
 * @file test_routing_standalone.cpp
 * @brief Standalone routing tests (simplified for testing without full dependencies)
 */

#include "mavlink_stubs.h"
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

// Simplified route entry structure
struct RouteEntry {
    uint8_t sysid;
    uint8_t compid;
    uint8_t channel;
    uint8_t mavType;
    uint32_t lastSeenMS;
    bool active;
};

// Simplified router class
class SimpleRouter {
private:
    static const int MAX_ROUTES = 20;
    RouteEntry routes[MAX_ROUTES];
    int routeCount;
    bool blockedChannels[8];

public:
    SimpleRouter() : routeCount(0) {
        memset(routes, 0, sizeof(routes));
        memset(blockedChannels, 0, sizeof(blockedChannels));
    }

    void initialize() {
        routeCount = 0;
        memset(routes, 0, sizeof(routes));
        memset(blockedChannels, 0, sizeof(blockedChannels));
    }

    int getRouteCount() const { return routeCount; }

    bool addRoute(uint8_t sysid, uint8_t compid, uint8_t channel, uint8_t mavType) {
        if (routeCount >= MAX_ROUTES) return false;

        // Check if route already exists
        for (int i = 0; i < routeCount; i++) {
            if (routes[i].sysid == sysid && routes[i].compid == compid) {
                // Update existing route
                routes[i].channel = channel;
                routes[i].mavType = mavType;
                routes[i].lastSeenMS = millis();
                return true;
            }
        }

        // Add new route
        routes[routeCount].sysid = sysid;
        routes[routeCount].compid = compid;
        routes[routeCount].channel = channel;
        routes[routeCount].mavType = mavType;
        routes[routeCount].lastSeenMS = millis();
        routes[routeCount].active = true;
        routeCount++;
        return true;
    }

    bool findRoute(uint8_t sysid, uint8_t compid, uint8_t& outChannel) {
        for (int i = 0; i < routeCount; i++) {
            if (routes[i].sysid == sysid &&
                (routes[i].compid == compid || compid == 0)) {
                outChannel = routes[i].channel;
                return true;
            }
        }
        return false;
    }

    bool findByMAVType(uint8_t mavType, uint8_t& outSysid, uint8_t& outCompid, uint8_t& outChannel) {
        for (int i = 0; i < routeCount; i++) {
            if (routes[i].mavType == mavType) {
                outSysid = routes[i].sysid;
                outCompid = routes[i].compid;
                outChannel = routes[i].channel;
                return true;
            }
        }
        return false;
    }

    bool removeRoute(uint8_t sysid, uint8_t compid) {
        for (int i = 0; i < routeCount; i++) {
            if (routes[i].sysid == sysid && routes[i].compid == compid) {
                // Shift remaining routes
                for (int j = i; j < routeCount - 1; j++) {
                    routes[j] = routes[j + 1];
                }
                routeCount--;
                return true;
            }
        }
        return false;
    }

    void removeStaleRoutes(uint32_t timeoutMS) {
        uint32_t now = millis();
        for (int i = routeCount - 1; i >= 0; i--) {
            if (now - routes[i].lastSeenMS > timeoutMS) {
                removeRoute(routes[i].sysid, routes[i].compid);
            }
        }
    }

    void clearAllRoutes() {
        routeCount = 0;
        memset(routes, 0, sizeof(routes));
    }

    void blockChannel(uint8_t channel) {
        if (channel < 8) blockedChannels[channel] = true;
    }

    void unblockChannel(uint8_t channel) {
        if (channel < 8) blockedChannels[channel] = false;
    }

    bool isChannelBlocked(uint8_t channel) const {
        return (channel < 8) ? blockedChannels[channel] : false;
    }

    const RouteEntry* getRoute(int index) const {
        return (index >= 0 && index < routeCount) ? &routes[index] : nullptr;
    }
};

/**
 * Test functions
 */

bool test_router_initialization() {
    SimpleRouter router;
    router.initialize();

    TEST_ASSERT(router.getRouteCount() == 0, "Route count should be 0");
    TEST_PASS();
}

bool test_route_addition() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    bool added = router.addRoute(255, 190, 0, MAV_TYPE_GCS);
    TEST_ASSERT(added, "Should add route");
    TEST_ASSERT(router.getRouteCount() == 1, "Should have 1 route");
    TEST_PASS();
}

bool test_route_lookup() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(255, 190, 0, MAV_TYPE_GCS);

    uint8_t channel;
    bool found = router.findRoute(255, 190, channel);
    TEST_ASSERT(found, "Should find route");
    TEST_ASSERT(channel == 0, "Channel should be 0");
    TEST_PASS();
}

bool test_multiple_routes() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(255, 190, 0, MAV_TYPE_GCS);
    router.addRoute(1, 191, 1, MAV_TYPE_ONBOARD_CONTROLLER);
    router.addRoute(1, 100, 2, MAV_TYPE_CAMERA);

    TEST_ASSERT(router.getRouteCount() == 3, "Should have 3 routes");

    uint8_t channel;
    TEST_ASSERT(router.findRoute(255, 190, channel) && channel == 0, "GCS route");
    TEST_ASSERT(router.findRoute(1, 191, channel) && channel == 1, "Companion route");
    TEST_ASSERT(router.findRoute(1, 100, channel) && channel == 2, "Camera route");
    TEST_PASS();
}

bool test_route_update() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(255, 190, 0, MAV_TYPE_GCS);

    g_mockTime = 2000;
    router.addRoute(255, 190, 1, MAV_TYPE_GCS);

    TEST_ASSERT(router.getRouteCount() == 1, "Should still have 1 route");

    uint8_t channel;
    router.findRoute(255, 190, channel);
    TEST_ASSERT(channel == 1, "Route should update to channel 1");
    TEST_PASS();
}

bool test_find_by_mav_type() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(255, 190, 0, MAV_TYPE_GCS);
    router.addRoute(1, 100, 1, MAV_TYPE_CAMERA);

    uint8_t sysid, compid, channel;
    bool found = router.findByMAVType(MAV_TYPE_GCS, sysid, compid, channel);

    TEST_ASSERT(found, "Should find GCS");
    TEST_ASSERT(sysid == 255, "GCS sysid should be 255");
    TEST_ASSERT(compid == 190, "GCS compid should be 190");
    TEST_ASSERT(channel == 0, "GCS on channel 0");
    TEST_PASS();
}

bool test_route_removal() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(255, 190, 0, MAV_TYPE_GCS);
    router.addRoute(1, 191, 1, MAV_TYPE_ONBOARD_CONTROLLER);
    router.addRoute(1, 100, 2, MAV_TYPE_CAMERA);

    TEST_ASSERT(router.getRouteCount() == 3, "Should have 3 routes");

    bool removed = router.removeRoute(1, 191);
    TEST_ASSERT(removed, "Should remove route");
    TEST_ASSERT(router.getRouteCount() == 2, "Should have 2 routes");

    uint8_t channel;
    bool found = router.findRoute(1, 191, channel);
    TEST_ASSERT(!found, "Route should not be found");
    TEST_PASS();
}

bool test_stale_route_removal() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(255, 190, 0, MAV_TYPE_GCS);
    TEST_ASSERT(router.getRouteCount() == 1, "Should have 1 route");

    g_mockTime = 35000;
    router.removeStaleRoutes(30000);
    TEST_ASSERT(router.getRouteCount() == 0, "Stale route removed");
    TEST_PASS();
}

bool test_channel_blocking() {
    SimpleRouter router;
    router.initialize();

    router.blockChannel(2);
    TEST_ASSERT(router.isChannelBlocked(2), "Channel 2 blocked");
    TEST_ASSERT(!router.isChannelBlocked(0), "Channel 0 not blocked");

    router.unblockChannel(2);
    TEST_ASSERT(!router.isChannelBlocked(2), "Channel 2 unblocked");
    TEST_PASS();
}

bool test_clear_all_routes() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    for (uint8_t i = 0; i < 5; i++) {
        router.addRoute(i, 0, i, MAV_TYPE_GENERIC);
    }

    TEST_ASSERT(router.getRouteCount() == 5, "Should have 5 routes");

    router.clearAllRoutes();
    TEST_ASSERT(router.getRouteCount() == 0, "All routes cleared");
    TEST_PASS();
}

bool test_wildcard_lookup() {
    SimpleRouter router;
    router.initialize();
    g_mockTime = 1000;

    router.addRoute(1, 100, 0, MAV_TYPE_CAMERA);

    uint8_t channel;
    bool found = router.findRoute(1, 100, channel);
    TEST_ASSERT(found, "Find with specific component");

    found = router.findRoute(1, 0, channel);
    TEST_ASSERT(found, "Find with wildcard component");

    found = router.findRoute(1, 200, channel);
    TEST_ASSERT(!found, "Should not find non-existent component");
    TEST_PASS();
}

void runAllTests() {
    printf("\n=== Simplified MAVLink Routing Tests ===\n\n");

    struct Test {
        const char* name;
        bool (*func)();
    };

    Test tests[] = {
        {"Router Initialization", test_router_initialization},
        {"Route Addition", test_route_addition},
        {"Route Lookup", test_route_lookup},
        {"Multiple Routes", test_multiple_routes},
        {"Route Update", test_route_update},
        {"Find by MAV_TYPE", test_find_by_mav_type},
        {"Route Removal", test_route_removal},
        {"Stale Route Removal", test_stale_route_removal},
        {"Channel Blocking", test_channel_blocking},
        {"Clear All Routes", test_clear_all_routes},
        {"Wildcard Lookup", test_wildcard_lookup}
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
