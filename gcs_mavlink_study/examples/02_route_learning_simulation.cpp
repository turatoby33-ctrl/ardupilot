/**
 * @file 02_route_learning_simulation.cpp
 * @brief Simulates MAVLink routing table learning and message forwarding
 *
 * This example demonstrates:
 * 1. How routes are learned when messages arrive
 * 2. How messages are forwarded based on routing table
 * 3. Private channel isolation
 * 4. Finding devices by MAV_TYPE
 */

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/MAVLink_routing.h>
#include <AP_HAL/AP_HAL.h>

// Simulate different MAVLink devices
struct SimulatedDevice {
    const char *name;
    uint8_t sysid;
    uint8_t compid;
    MAV_TYPE mav_type;
    mavlink_channel_t channel;
};

// Our test network
SimulatedDevice test_devices[] = {
    {"QGroundControl",    255, 190, MAV_TYPE_GCS,                 MAVLINK_COMM_0},
    {"Mission Planner",   254, 190, MAV_TYPE_GCS,                 MAVLINK_COMM_1},
    {"Raspberry Pi",      100, 191, MAV_TYPE_ONBOARD_CONTROLLER,  MAVLINK_COMM_2},
    {"Camera",            1,   100, MAV_TYPE_CAMERA,              MAVLINK_COMM_3},
    {"Gimbal",            1,   101, MAV_TYPE_GIMBAL,              MAVLINK_COMM_3},
};

// Simulate receiving a heartbeat from a device
void simulate_heartbeat_received(const SimulatedDevice &device) {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat {};

    heartbeat.type = device.mav_type;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;  // Not an autopilot
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(
        device.sysid,
        device.compid,
        &msg,
        &heartbeat
    );

    // Get the GCS link for this channel
    GCS_MAVLINK *link = gcs().chan(device.channel);
    if (link == nullptr) {
        hal.console->printf("ERROR: Channel %d not initialized\n", device.channel);
        return;
    }

    hal.console->printf("Receiving HEARTBEAT from %s (sysid=%d, compid=%d) on channel %d\n",
                       device.name, device.sysid, device.compid, device.channel);

    // Simulate packet reception (this learns the route)
    link->packetReceived(*(link->channel_status()), msg);
}

// Example 1: Learn routes from heartbeats
void example_learn_routes() {
    hal.console->printf("\n=== Example 1: Learning Routes ===\n");

    // Simulate heartbeats from all devices
    for (size_t i = 0; i < ARRAY_SIZE(test_devices); i++) {
        simulate_heartbeat_received(test_devices[i]);
        hal.scheduler->delay(10);
    }

    hal.console->printf("\nRouting table should now have %d entries\n",
                       ARRAY_SIZE(test_devices));
}

// Example 2: Find a device by MAV_TYPE
void example_find_by_type() {
    hal.console->printf("\n=== Example 2: Find Device by Type ===\n");

    uint8_t sysid, compid;
    mavlink_channel_t channel;

    // Find GCS
    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GCS, sysid, compid, channel)) {
        hal.console->printf("Found GCS: sysid=%d, compid=%d, channel=%d\n",
                           sysid, compid, channel);
    }

    // Find gimbal
    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, sysid, compid, channel)) {
        hal.console->printf("Found GIMBAL: sysid=%d, compid=%d, channel=%d\n",
                           sysid, compid, channel);
    }

    // Find camera
    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_CAMERA, sysid, compid, channel)) {
        hal.console->printf("Found CAMERA: sysid=%d, compid=%d, channel=%d\n",
                           sysid, compid, channel);
    }

    // Try to find something that doesn't exist
    if (!GCS_MAVLINK::find_by_mavtype(MAV_TYPE_SUBMARINE, sysid, compid, channel)) {
        hal.console->printf("SUBMARINE not found (as expected)\n");
    }
}

// Example 3: Simulate message forwarding
void example_message_forwarding() {
    hal.console->printf("\n=== Example 3: Message Forwarding ===\n");

    // Scenario: QGC sends PARAM_SET to autopilot (us)
    hal.console->printf("\nScenario 1: QGC sends PARAM_SET to autopilot\n");
    hal.console->printf("Expected: Process locally, don't forward\n");

    mavlink_message_t msg;
    mavlink_param_set_t param_set {};

    param_set.target_system = 1;  // Our sysid (the autopilot)
    param_set.target_component = 1;  // Our compid
    strncpy(param_set.param_id, "WPNAV_SPEED", sizeof(param_set.param_id));
    param_set.param_value = 500.0f;
    param_set.param_type = MAV_PARAM_TYPE_REAL32;

    mavlink_msg_param_set_encode(
        255,  // From QGC
        190,
        &msg,
        &param_set
    );

    // This would normally trigger routing logic
    hal.console->printf("Message: PARAM_SET, target_system=%d, target_component=%d\n",
                       param_set.target_system, param_set.target_component);

    // Scenario 2: QGC sends command to camera
    hal.console->printf("\nScenario 2: QGC sends command to camera\n");
    hal.console->printf("Expected: Don't process locally, forward to camera's channel\n");

    mavlink_command_long_t cmd {};
    cmd.target_system = 1;      // Our sysid (but different component)
    cmd.target_component = 100; // Camera's component ID
    cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
    cmd.param1 = 1;  // Take photo

    mavlink_msg_command_long_encode(
        255,  // From QGC
        190,
        &msg,
        &cmd
    );

    hal.console->printf("Message: COMMAND_LONG, target_system=%d, target_component=%d\n",
                       cmd.target_system, cmd.target_component);
    hal.console->printf("Would forward to channel %d (camera's channel)\n",
                       test_devices[3].channel);  // Camera is at index 3
}

// Example 4: Private channel demonstration
void example_private_channel() {
    hal.console->printf("\n=== Example 4: Private Channel ===\n");

    // Set channel 2 (Raspberry Pi) as private
    GCS_MAVLINK::set_channel_private(MAVLINK_COMM_2);

    hal.console->printf("Channel 2 set as PRIVATE\n");

    // Check if it's private
    if (GCS_MAVLINK::is_private(MAVLINK_COMM_2)) {
        hal.console->printf("Channel 2 is confirmed private\n");
        hal.console->printf("  - Messages FROM this channel: NOT forwarded\n");
        hal.console->printf("  - Messages TO this channel: Sent if targeted\n");
        hal.console->printf("  - Broadcasts: NOT forwarded to this channel\n");
    }

    // Show which channels are private
    uint8_t private_mask = GCS_MAVLINK::private_channel_mask();
    hal.console->printf("\nPrivate channel bitmask: 0x%02X\n", private_mask);
    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (private_mask & (1 << i)) {
            hal.console->printf("  Channel %d: PRIVATE\n", i);
        }
    }
}

// Example 5: Send to all components on this vehicle
void example_send_to_components() {
    hal.console->printf("\n=== Example 5: Send to Components ===\n");

    // Create a command to send to all components
    mavlink_command_long_t cmd {};
    cmd.target_system = mavlink_system.sysid;  // Our sysid
    cmd.target_component = MAV_COMP_ID_ALL;    // All components
    cmd.command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    cmd.param1 = 0;  // Autopilot reboot

    hal.console->printf("Sending REBOOT command to all components on sysid=%d\n",
                       mavlink_system.sysid);

    // This would send to camera (compid=100) and gimbal (compid=101)
    // but NOT to autopilot (ourselves) or GCS (different sysid)
    hal.console->printf("Would send to:\n");
    for (size_t i = 0; i < ARRAY_SIZE(test_devices); i++) {
        if (test_devices[i].sysid == mavlink_system.sysid &&
            test_devices[i].compid != mavlink_system.compid) {
            hal.console->printf("  - %s (sysid=%d, compid=%d)\n",
                               test_devices[i].name,
                               test_devices[i].sysid,
                               test_devices[i].compid);
        }
    }
}

// Example 6: Routing table overflow
void example_routing_table_overflow() {
    hal.console->printf("\n=== Example 6: Routing Table Limits ===\n");

    hal.console->printf("Maximum routes: %d\n", MAVLINK_MAX_ROUTES);

    // Simulate adding many devices
    hal.console->printf("\nSimulating addition of 25 devices...\n");

    for (uint8_t i = 0; i < 25; i++) {
        if (i < MAVLINK_MAX_ROUTES) {
            hal.console->printf("  Device %d: Added to routing table\n", i);
        } else {
            hal.console->printf("  Device %d: REJECTED - routing table full!\n", i);
        }
    }

    hal.console->printf("\nRouting table would have %d entries (max %d)\n",
                       (25 < MAVLINK_MAX_ROUTES) ? 25 : MAVLINK_MAX_ROUTES,
                       MAVLINK_MAX_ROUTES);
}

// Example 7: Route refresh and staleness
void example_route_refresh() {
    hal.console->printf("\n=== Example 7: Route Refresh ===\n");

    hal.console->printf("Routes are learned from incoming messages\n");
    hal.console->printf("Heartbeats refresh existing routes\n");
    hal.console->printf("No automatic route expiration (persist until reboot)\n\n");

    SimulatedDevice &qgc = test_devices[0];

    hal.console->printf("Initial heartbeat from %s:\n", qgc.name);
    hal.console->printf("  Route learned: sysid=%d, compid=%d, channel=%d\n",
                       qgc.sysid, qgc.compid, qgc.channel);

    hal.console->printf("\nQGC reconnects on different channel...\n");
    // Simulate QGC moving to channel 1
    SimulatedDevice qgc_moved = qgc;
    qgc_moved.channel = MAVLINK_COMM_1;

    hal.console->printf("New heartbeat from %s:\n", qgc_moved.name);
    hal.console->printf("  Route UPDATED: sysid=%d, compid=%d, channel=%d (NEW)\n",
                       qgc_moved.sysid, qgc_moved.compid, qgc_moved.channel);

    hal.console->printf("\nSame sysid/compid updates channel, doesn't create duplicate\n");
}

// Example 8: Visualize routing table
void example_visualize_routing_table() {
    hal.console->printf("\n=== Example 8: Routing Table Visualization ===\n\n");

    hal.console->printf("┌──────────────────────┬───────┬────────┬──────────┬────────────────────────┐\n");
    hal.console->printf("│ Device Name          │ SysID │ CompID │ Channel  │ MAV_TYPE               │\n");
    hal.console->printf("├──────────────────────┼───────┼────────┼──────────┼────────────────────────┤\n");

    for (size_t i = 0; i < ARRAY_SIZE(test_devices); i++) {
        const SimulatedDevice &dev = test_devices[i];

        const char *type_name;
        switch (dev.mav_type) {
            case MAV_TYPE_GCS:                 type_name = "GCS"; break;
            case MAV_TYPE_ONBOARD_CONTROLLER:  type_name = "ONBOARD_CONTROLLER"; break;
            case MAV_TYPE_CAMERA:              type_name = "CAMERA"; break;
            case MAV_TYPE_GIMBAL:              type_name = "GIMBAL"; break;
            default:                           type_name = "UNKNOWN"; break;
        }

        hal.console->printf("│ %-20s │ %5d │ %6d │ COMM_%d   │ %-22s │\n",
                           dev.name,
                           dev.sysid,
                           dev.compid,
                           dev.channel,
                           type_name);
    }

    hal.console->printf("└──────────────────────┴───────┴────────┴──────────┴────────────────────────┘\n");
}

// Example 9: Message type detection and routing
void example_message_type_routing() {
    hal.console->printf("\n=== Example 9: Message Type Routing ===\n\n");

    struct {
        const char *name;
        bool has_target;
        const char *routing_behavior;
    } message_types[] = {
        {"HEARTBEAT",        false, "Broadcast, process locally, don't forward"},
        {"ATTITUDE",         false, "Broadcast to all channels"},
        {"GPS_RAW_INT",      false, "Broadcast to all channels"},
        {"PARAM_SET",        true,  "Forward to target if not us"},
        {"COMMAND_LONG",     true,  "Forward to target if not us"},
        {"MISSION_ITEM_INT", true,  "Forward to target if not us"},
        {"RADIO_STATUS",     false, "Process locally only, never forward"},
    };

    for (size_t i = 0; i < ARRAY_SIZE(message_types); i++) {
        hal.console->printf("%-20s | Target: %-5s | %s\n",
                           message_types[i].name,
                           message_types[i].has_target ? "YES" : "NO",
                           message_types[i].routing_behavior);
    }
}

// Main demonstration function
void demonstrate_routing() {
    hal.console->printf("\n");
    hal.console->printf("╔════════════════════════════════════════════════════════════╗\n");
    hal.console->printf("║    MAVLink Routing System - Interactive Demonstration     ║\n");
    hal.console->printf("╚════════════════════════════════════════════════════════════╝\n");

    // Our autopilot identity
    hal.console->printf("\nAutopilot Identity:\n");
    hal.console->printf("  System ID: %d\n", mavlink_system.sysid);
    hal.console->printf("  Component ID: %d\n", mavlink_system.compid);

    // Run examples
    example_learn_routes();
    hal.scheduler->delay(500);

    example_visualize_routing_table();
    hal.scheduler->delay(500);

    example_find_by_type();
    hal.scheduler->delay(500);

    example_message_forwarding();
    hal.scheduler->delay(500);

    example_private_channel();
    hal.scheduler->delay(500);

    example_send_to_components();
    hal.scheduler->delay(500);

    example_routing_table_overflow();
    hal.scheduler->delay(500);

    example_route_refresh();
    hal.scheduler->delay(500);

    example_message_type_routing();
    hal.scheduler->delay(500);

    hal.console->printf("\n");
    hal.console->printf("╔════════════════════════════════════════════════════════════╗\n");
    hal.console->printf("║              Routing Demonstration Complete                ║\n");
    hal.console->printf("╚════════════════════════════════════════════════════════════╝\n");
    hal.console->printf("\n");
}

/**
 * KEY CONCEPTS DEMONSTRATED:
 *
 * 1. Route Learning:
 *    - Automatic from incoming messages
 *    - Stores sysid, compid, channel, mavtype
 *    - Updates channel if device moves
 *    - Maximum 20 routes
 *
 * 2. Message Forwarding:
 *    - Broadcast messages go to all channels
 *    - Targeted messages go to specific route
 *    - Messages from private channels not forwarded
 *    - Prevents routing loops (won't send back to sender)
 *
 * 3. Device Discovery:
 *    - find_by_mavtype() finds devices by type
 *    - find_by_mavtype_and_compid() for specific component
 *    - Useful for gimbal, camera, companion computer control
 *
 * 4. Private Channels:
 *    - Isolate companion computer traffic
 *    - Prevent message loops
 *    - Reduce bandwidth on GCS links
 *
 * 5. Component Communication:
 *    - send_to_components() sends to all on this vehicle
 *    - Excludes ourselves and GCS
 *    - Useful for system-wide commands
 *
 * USAGE:
 * Call demonstrate_routing() from your vehicle's setup() or loop()
 * to see routing system in action.
 */
