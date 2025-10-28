/**
 * @file 01_message_sending_example.cpp
 * @brief Demonstrates how to send MAVLink messages from ArduPilot
 *
 * This example shows:
 * 1. Checking payload space before sending
 * 2. Sending to specific channels
 * 3. Broadcasting to all channels
 * 4. Using deferred message queue
 */

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

// Example 1: Send a heartbeat message to a specific channel
void example_send_heartbeat_specific_channel() {
    // Get channel 0 (usually USB)
    GCS_MAVLINK *usb_link = gcs().chan(0);

    if (usb_link == nullptr) {
        return;  // Channel not initialized
    }

    // Check if channel is active (has seen heartbeat from GCS)
    if (!usb_link->is_active()) {
        return;  // No GCS connected on this channel
    }

    // Check if there's space in the transmit buffer
    if (!HAVE_PAYLOAD_SPACE(usb_link->get_chan(), HEARTBEAT)) {
        // Buffer full, message will be dropped
        return;
    }

    // All checks passed, send heartbeat
    usb_link->send_heartbeat();

    hal.console->printf("Heartbeat sent on channel 0\n");
}

// Example 2: Broadcast a message to all active channels
void example_broadcast_attitude() {
    // This sends to ALL active, non-private channels
    gcs().send_message(MSG_ATTITUDE);

    // Equivalent manual implementation:
    for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
        GCS_MAVLINK *link = gcs().chan(i);

        if (link == nullptr) continue;           // Channel not initialized
        if (!link->is_active()) continue;        // No GCS connected
        if (link->is_private()) continue;        // Skip private channels

        link->send_message(MSG_ATTITUDE);
    }
}

// Example 3: Send text message with severity levels
void example_send_text_messages() {
    // INFO: General information (green in QGC)
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Waypoint %d reached", 5);

    // WARNING: Something needs attention (yellow in QGC)
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Battery low: %d%%", 25);

    // ERROR: Critical issue (red in QGC)
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GPS lost!");

    // CRITICAL: System failure (flashing red)
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Parachute deployed!");

    // DEBUG: Development info (usually not shown to user)
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Loop rate: %d Hz", 400);
}

// Example 4: Queue a message for deferred sending
void example_queue_message() {
    // Queue the message (doesn't send immediately)
    gcs().send_message(MSG_GPS_RAW);

    // Message will be sent in next update_send() cycle
    // respecting stream rates and bandwidth limits
}

// Example 5: Send custom telemetry data using NAMED_VALUE_FLOAT
void example_send_custom_telemetry(float temperature, float voltage) {
    // Send temperature
    gcs().send_named_float("CPU_TEMP", temperature);

    // Send voltage
    gcs().send_named_float("AUX_VOLT", voltage);

    // These appear in QGC's "MAVLink Inspector" under NAMED_VALUE_FLOAT
}

// Example 6: Proper way to send messages in a loop (parameter streaming)
void example_parameter_streaming() {
    GCS_MAVLINK *link = gcs().chan(0);
    if (link == nullptr || !link->is_active()) {
        return;
    }

    const uint32_t start_time = AP_HAL::millis();
    const uint32_t timeout_ms = 100;  // Don't block for more than 100ms

    uint16_t param_index = 0;
    uint16_t param_count = 50;  // Assume 50 parameters to send

    while (param_index < param_count) {
        // Check timeout (prevent blocking main loop)
        if (AP_HAL::millis() - start_time > timeout_ms) {
            break;  // Resume in next iteration
        }

        // Check if we have space
        if (!HAVE_PAYLOAD_SPACE(link->get_chan(), PARAM_VALUE)) {
            break;  // Buffer full, try again later
        }

        // Send parameter
        char param_name[17];
        snprintf(param_name, sizeof(param_name), "PARAM_%d", param_index);
        link->send_parameter_value(param_name, AP_PARAM_FLOAT, param_index * 1.5f);

        param_index++;
    }

    hal.console->printf("Sent %d parameters in %d ms\n",
                       param_index,
                       AP_HAL::millis() - start_time);
}

// Example 7: Check payload space for multiple messages
void example_batch_send() {
    GCS_MAVLINK *link = gcs().chan(0);
    if (link == nullptr) return;

    mavlink_channel_t chan = link->get_chan();

    // Calculate total space needed
    uint16_t total_needed = 0;
    total_needed += PAYLOAD_SIZE(chan, HEARTBEAT);        // ~34 bytes (MAVLink 2)
    total_needed += PAYLOAD_SIZE(chan, ATTITUDE);         // ~40 bytes
    total_needed += PAYLOAD_SIZE(chan, GPS_RAW_INT);      // ~50 bytes
    total_needed += PAYLOAD_SIZE(chan, SYS_STATUS);       // ~45 bytes
    // Total: ~169 bytes

    // Check if we have enough space for all
    if (comm_get_txspace(chan) >= total_needed) {
        // Send all messages atomically
        link->send_heartbeat();
        link->send_message(MSG_ATTITUDE);
        link->send_message(MSG_GPS_RAW);
        link->send_message(MSG_SYS_STATUS);

        hal.console->printf("Sent batch of 4 messages\n");
    } else {
        hal.console->printf("Not enough space: need %d, have %d\n",
                           total_needed,
                           comm_get_txspace(chan));
    }
}

// Example 8: Send with channel locking (thread-safe)
void example_thread_safe_send() {
    GCS_MAVLINK *link = gcs().chan(0);
    if (link == nullptr) return;

    mavlink_channel_t chan = link->get_chan();

    // Lock the channel semaphore
    HAL_Semaphore &sem = comm_chan_lock(chan);
    WITH_SEMAPHORE(sem);

    // Now we have exclusive access to this channel
    // Safe to send multiple messages without interruption

    if (HAVE_PAYLOAD_SPACE(chan, HEARTBEAT)) {
        link->send_heartbeat();
    }

    if (HAVE_PAYLOAD_SPACE(chan, ATTITUDE)) {
        link->send_message(MSG_ATTITUDE);
    }

    // Semaphore automatically released when exiting scope
}

// Example 9: Conditional sending based on link type
void example_conditional_sending() {
    for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
        GCS_MAVLINK *link = gcs().chan(i);
        if (link == nullptr || !link->is_active()) continue;

        // High bandwidth link (USB) gets full telemetry
        if (link->is_high_bandwidth()) {
            link->send_message(MSG_RAW_IMU);           // Raw sensor data
            link->send_message(MSG_SCALED_IMU);
            link->send_message(MSG_SCALED_PRESSURE);
            link->send_message(MSG_HIGHRES_IMU);
        }

        // All links get essential data
        link->send_message(MSG_ATTITUDE);
        link->send_message(MSG_GPS_RAW);
        link->send_message(MSG_BATTERY_STATUS);

        // High latency links get compressed data
#if HAL_HIGH_LATENCY2_ENABLED
        if (link->is_high_latency_link) {
            link->send_message(MSG_HIGH_LATENCY2);  // Single compressed packet
        }
#endif
    }
}

// Example 10: Measure send performance
void example_measure_send_performance() {
    GCS_MAVLINK *link = gcs().chan(0);
    if (link == nullptr) return;

    const uint16_t test_count = 1000;
    uint16_t success_count = 0;
    uint16_t no_space_count = 0;

    uint32_t start_us = AP_HAL::micros();

    for (uint16_t i = 0; i < test_count; i++) {
        if (HAVE_PAYLOAD_SPACE(link->get_chan(), HEARTBEAT)) {
            link->send_heartbeat();
            success_count++;
        } else {
            no_space_count++;
        }

        // Small delay to allow TX buffer to drain
        hal.scheduler->delay_microseconds(100);
    }

    uint32_t elapsed_us = AP_HAL::micros() - start_us;

    hal.console->printf("Send Performance Test:\n");
    hal.console->printf("  Messages sent: %d/%d\n", success_count, test_count);
    hal.console->printf("  Failed (no space): %d\n", no_space_count);
    hal.console->printf("  Time: %d us\n", elapsed_us);
    hal.console->printf("  Avg per message: %d us\n", elapsed_us / test_count);
    hal.console->printf("  Message rate: %d msg/sec\n",
                       (test_count * 1000000UL) / elapsed_us);
}

// Example 11: Send to specific system/component using routing
void example_send_to_component() {
    // Find a gimbal component
    uint8_t gimbal_sysid, gimbal_compid;
    mavlink_channel_t gimbal_chan;

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL,
                                     gimbal_sysid,
                                     gimbal_compid,
                                     gimbal_chan)) {
        // Found gimbal! Send it a command
        mavlink_command_long_t cmd {};
        cmd.target_system = gimbal_sysid;
        cmd.target_component = gimbal_compid;
        cmd.command = MAV_CMD_DO_MOUNT_CONTROL;
        cmd.param1 = -45.0f;  // Pitch angle
        cmd.param2 = 0.0f;    // Roll
        cmd.param3 = 90.0f;   // Yaw

        GCS_MAVLINK *link = gcs().chan(gimbal_chan);
        if (link && HAVE_PAYLOAD_SPACE(gimbal_chan, COMMAND_LONG)) {
            mavlink_msg_command_long_send_struct(gimbal_chan, &cmd);
            hal.console->printf("Gimbal command sent\n");
        }
    } else {
        hal.console->printf("No gimbal found in routing table\n");
    }
}

// Main demonstration function
void demonstrate_message_sending() {
    hal.console->printf("=== MAVLink Message Sending Examples ===\n\n");

    hal.console->printf("1. Specific channel send:\n");
    example_send_heartbeat_specific_channel();
    hal.scheduler->delay(100);

    hal.console->printf("\n2. Broadcast:\n");
    example_broadcast_attitude();
    hal.scheduler->delay(100);

    hal.console->printf("\n3. Text messages:\n");
    example_send_text_messages();
    hal.scheduler->delay(100);

    hal.console->printf("\n4. Queue message:\n");
    example_queue_message();
    hal.scheduler->delay(100);

    hal.console->printf("\n5. Custom telemetry:\n");
    example_send_custom_telemetry(45.2f, 5.1f);
    hal.scheduler->delay(100);

    hal.console->printf("\n6. Batch send:\n");
    example_batch_send();
    hal.scheduler->delay(100);

    hal.console->printf("\n7. Performance test:\n");
    example_measure_send_performance();

    hal.console->printf("\n=== Examples Complete ===\n");
}

/**
 * COMPILATION NOTES:
 *
 * This example is meant to be compiled as part of ArduPilot.
 * To use it:
 *
 * 1. Add this file to: libraries/GCS_MAVLink/examples/
 *
 * 2. Call demonstrate_message_sending() from your vehicle's main loop
 *    or create a test sketch in Tools/
 *
 * 3. Build with: ./waf configure --board=<your_board>
 *                ./waf copter  (or plane/rover)
 *
 * KEY CONCEPTS DEMONSTRATED:
 * - Always check HAVE_PAYLOAD_SPACE before sending
 * - Use gcs().send_message() for broadcast
 * - Use GCS_SEND_TEXT() for text messages
 * - Respect timing constraints (don't block main loop)
 * - Use semaphores for thread-safe sending
 * - Check channel status (active, bandwidth, etc.)
 */
