/*
 * Test Program for Minimal Standalone GCS
 * Version 1: HEARTBEAT Send/Receive Demonstration
 *
 * This program demonstrates:
 * 1. Initializing the GCS system
 * 2. Sending periodic HEARTBEAT messages (vehicle → GCS)
 * 3. Receiving and handling HEARTBEAT messages (GCS → vehicle)
 * 4. Monitoring connection status
 *
 * Compile with:
 *   g++ -o test_heartbeat test_heartbeat.cpp minimal_GCS.cpp minimal_GCS_Common.cpp -std=c++11
 *
 * Run with:
 *   ./test_heartbeat
 */

#include "minimal_GCS.h"
#include <unistd.h>  // for sleep/usleep

// ============================================
// Simulated GCS (for testing)
// ============================================

class SimulatedGCS {
public:
    SimulatedGCS()
        : _system_id(255)  // GCS typically uses sysid 255
        , _component_id(1)
        , _seq(0)
        , _send_interval_ms(1000)  // Send at 1 Hz
        , _last_send_ms(0)
    {
    }

    void send_heartbeat_to_vehicle(GCS_MAVLINK* vehicle_channel) {
        uint32_t now = get_time_ms();

        if (now - _last_send_ms >= _send_interval_ms) {
            print_message("\n");
            print_message("╔═══════════════════════════════════════╗");
            print_message("║  SIMULATED GCS SENDING HEARTBEAT      ║");
            print_message("╚═══════════════════════════════════════╝");

            // Create a HEARTBEAT message from GCS
            mavlink_message_t msg;
            mavlink_heartbeat_t heartbeat;

            heartbeat.type = MAV_TYPE_GENERIC;  // GCS type
            heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
            heartbeat.base_mode = 0;  // GCS has no flight modes
            heartbeat.custom_mode = 0;
            heartbeat.system_status = MAV_STATE_ACTIVE;
            heartbeat.mavlink_version = 3;

            msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
            msg.sysid = _system_id;
            msg.compid = _component_id;
            msg.len = sizeof(mavlink_heartbeat_t);
            memcpy(msg.payload, &heartbeat, sizeof(heartbeat));

            // Simulate the vehicle receiving this message
            vehicle_channel->handle_message(msg);

            _last_send_ms = now;
        }
    }

private:
    uint8_t _system_id;
    uint8_t _component_id;
    uint8_t _seq;
    uint32_t _send_interval_ms;
    uint32_t _last_send_ms;
};

// ============================================
// Main Test Program
// ============================================

int main() {
    print_message("╔═══════════════════════════════════════════════════════╗");
    print_message("║   Minimal GCS HEARTBEAT Test Program (Version 1)     ║");
    print_message("║   Standalone Implementation                           ║");
    print_message("╚═══════════════════════════════════════════════════════╝");
    print_message("");

    // Step 1: Initialize GCS system
    print_message(">>> STEP 1: Initialize GCS System");
    gcs().init();
    print_message("");

    // Step 2: Get the channel
    print_message(">>> STEP 2: Get MAVLink Channel");
    GCS_MAVLINK* channel = gcs().chan(0);
    if (!channel) {
        print_message("ERROR: Could not get channel 0!");
        return 1;
    }
    print_message("Channel 0 ready");
    print_message("");

    // Step 3: Create simulated GCS
    print_message(">>> STEP 3: Create Simulated GCS");
    SimulatedGCS simulated_gcs;
    print_message("Simulated GCS created");
    print_message("");

    // Step 4: Run the test loop
    print_message(">>> STEP 4: Running Test Loop (10 seconds)");
    print_message("Will demonstrate:");
    print_message("  - Vehicle sending HEARTBEAT every 1 second");
    print_message("  - GCS sending HEARTBEAT every 1 second");
    print_message("  - Connection status monitoring");
    print_message("");

    uint32_t start_time = get_time_ms();
    uint32_t duration_ms = 10000;  // Run for 10 seconds
    uint32_t last_status_print = 0;

    while ((get_time_ms() - start_time) < duration_ms) {
        uint32_t now = get_time_ms();

        // Vehicle sends HEARTBEAT (automatically at 1 Hz)
        gcs().update_send();

        // Simulated GCS sends HEARTBEAT to vehicle
        simulated_gcs.send_heartbeat_to_vehicle(channel);

        // Print connection status every 2 seconds
        if (now - last_status_print >= 2000) {
            print_message("\n");
            print_message("┌─────────────────────────────────────┐");
            print_message("│  CONNECTION STATUS                  │");
            print_message("├─────────────────────────────────────┤");
            print_message("│  Channel Active: %s", channel->is_active() ? "YES ✓" : "NO ✗");
            print_message("│  Last GCS Heartbeat: %u ms ago",
                         now - channel->get_last_heartbeat_time());
            print_message("└─────────────────────────────────────┘");
            print_message("");

            last_status_print = now;
        }

        // Small delay to prevent CPU spinning
        usleep(100000);  // 100ms
    }

    // Final status
    print_message("\n");
    print_message("╔═══════════════════════════════════════╗");
    print_message("║  TEST COMPLETE                        ║");
    print_message("╚═══════════════════════════════════════╝");
    print_message("");
    print_message("Summary:");
    print_message("  - Duration: %u seconds", duration_ms / 1000);
    print_message("  - Final Connection Status: %s",
                 channel->is_active() ? "ACTIVE ✓" : "INACTIVE ✗");
    print_message("  - Time since last GCS heartbeat: %u ms",
                 get_time_ms() - channel->get_last_heartbeat_time());
    print_message("");
    print_message("What we demonstrated:");
    print_message("  ✓ GCS initialization");
    print_message("  ✓ Channel creation and setup");
    print_message("  ✓ Sending HEARTBEAT messages (vehicle → GCS)");
    print_message("  ✓ Receiving HEARTBEAT messages (GCS → vehicle)");
    print_message("  ✓ Parsing HEARTBEAT payload");
    print_message("  ✓ Connection status monitoring");
    print_message("  ✓ Message timing and intervals");
    print_message("");
    print_message("Next steps:");
    print_message("  - Add more message types (see MAVLink documentation)");
    print_message("  - Implement actual serial/network communication");
    print_message("  - Add parameter protocol");
    print_message("  - Add mission protocol");
    print_message("  - See Version 2 for ArduPilot integration");
    print_message("");

    return 0;
}

/*
 * ============================================
 * Expected Output
 * ============================================
 *
 * When you run this program, you should see:
 *
 * 1. Initialization messages
 * 2. Alternating HEARTBEAT messages:
 *    - Vehicle sending HEARTBEAT
 *    - GCS sending HEARTBEAT
 *    - Vehicle receiving and parsing GCS HEARTBEAT
 * 3. Periodic connection status updates
 * 4. Final summary
 *
 * The output will show detailed information about:
 * - Message contents (type, autopilot, mode, status)
 * - Base mode flags (armed, enabled modes, etc.)
 * - Connection status and timing
 *
 * ============================================
 * Understanding the Output
 * ============================================
 *
 * HEARTBEAT SENT:
 * - Shows the vehicle sending its HEARTBEAT
 * - Includes all fields being sent
 * - Happens every 1000ms (1 Hz)
 *
 * HEARTBEAT RECEIVED:
 * - Shows the GCS HEARTBEAT being received
 * - Parses and displays all fields
 * - Updates connection status
 * - Happens every 1000ms (1 Hz)
 *
 * CONNECTION STATUS:
 * - Shows if GCS is considered "active"
 * - Active = received HEARTBEAT within 2.5 seconds
 * - Displays time since last HEARTBEAT
 *
 * ============================================
 * Compiling and Running
 * ============================================
 *
 * To compile:
 *   g++ -o test_heartbeat test_heartbeat.cpp minimal_GCS.cpp minimal_GCS_Common.cpp -std=c++11
 *
 * To run:
 *   ./test_heartbeat
 *
 * Clean up:
 *   rm test_heartbeat
 *
 * ============================================
 * What This Demonstrates
 * ============================================
 *
 * This minimal implementation shows the core
 * concepts of GCS communication without the
 * complexity of the full ArduPilot system:
 *
 * 1. Message structure (mavlink_message_t)
 * 2. Message sending (creating and packing)
 * 3. Message receiving (parsing and handling)
 * 4. Connection monitoring (heartbeat timing)
 * 5. State management (active/inactive)
 *
 * These same patterns apply to ALL MAVLink
 * messages in the full system.
 */
