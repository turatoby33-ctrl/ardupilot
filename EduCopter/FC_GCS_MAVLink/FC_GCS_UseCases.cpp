/**
 * @file FC_GCS_UseCases.cpp
 * @brief Detailed Use Case Examples for EduCopter GCS_MAVLink
 *
 * This file demonstrates common use cases and scenarios:
 * 1. Parameter Management
 * 2. Mission Planning (Waypoints)
 * 3. Geofence Setup
 * 4. Rally Points
 * 5. File Transfer (FTP)
 * 6. Message Signing (Security)
 * 7. Stream Rate Configuration
 * 8. Servo/Relay Control
 * 9. Serial Passthrough
 * 10. Device Operations
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "MissionItemProtocol_Waypoints.h"
#include "MissionItemProtocol_Fence.h"
#include "MissionItemProtocol_Rally.h"
#include "GCS_FTP.h"
#include <cstdio>

using namespace EduCopter::GCS;

// ============================================================================
// USE CASE 1: PARAMETER MANAGEMENT
// ============================================================================

/**
 * @brief Example: Reading and modifying parameters via MAVLink
 *
 * Flow from GCS perspective:
 * 1. GCS sends PARAM_REQUEST_LIST
 * 2. Vehicle responds with PARAM_VALUE for each parameter
 * 3. GCS sends PARAM_SET to change a value
 * 4. Vehicle confirms with PARAM_VALUE
 */
void useCase1_ParameterManagement() {
    printf("\n========================================\n");
    printf("USE CASE 1: Parameter Management\n");
    printf("========================================\n");

    printf("\nScenario: GCS wants to adjust PID gains\n");

    // When GCS sends PARAM_REQUEST_LIST:
    printf("\n1. GCS: Requesting parameter list...\n");
    printf("   -> Vehicle responds with all %u parameters\n", getParameterCount());

    // Vehicle automatically handles this via handleParamRequestList()

    // When GCS wants to change a parameter:
    printf("\n2. GCS: Setting PID_ROLL_P to 0.20\n");

    // Simulate PARAM_SET message
    mavlink_message_t msg;
    mavlink_param_set_t packet;
    packet.target_system = 1;
    packet.target_component = 1;
    strncpy(packet.param_id, "PID_ROLL_P", 16);
    packet.param_value = 0.20f;
    packet.param_type = MAV_PARAM_TYPE_REAL32;

    // Vehicle handles via handleParamSet()
    // This will:
    // - Find parameter by name
    // - Validate new value
    // - Update parameter
    // - Send PARAM_VALUE confirmation

    printf("   -> Parameter updated successfully\n");
    printf("   -> New value: %.4f\n", getParameterValue(0));

    // Saving parameters:
    printf("\n3. GCS: Requesting parameter save\n");
    // GCS sends MAV_CMD_PREFLIGHT_STORAGE with param1=1
    saveParameters();
    printf("   -> Parameters saved to persistent storage\n");
}

// ============================================================================
// USE CASE 2: WAYPOINT MISSION UPLOAD
// ============================================================================

/**
 * @brief Example: Uploading a waypoint mission
 *
 * Flow:
 * 1. GCS sends MISSION_COUNT with number of waypoints
 * 2. Vehicle requests each waypoint with MISSION_REQUEST_INT
 * 3. GCS sends MISSION_ITEM_INT for each waypoint
 * 4. Vehicle sends MISSION_ACK when complete
 */
void useCase2_WaypointMission() {
    printf("\n========================================\n");
    printf("USE CASE 2: Waypoint Mission Upload\n");
    printf("========================================\n");

    printf("\nScenario: GCS uploads a 3-waypoint mission\n");

    // Step 1: GCS sends mission count
    printf("\n1. GCS: Sending mission count (3 waypoints)\n");
    // Vehicle receives MISSION_COUNT message
    // handleMissionCount() in MissionItemProtocol is called

    // Step 2: Vehicle requests first waypoint
    printf("2. Vehicle: Requesting waypoint 0\n");
    // Vehicle sends MISSION_REQUEST_INT(seq=0)

    // Step 3: GCS sends waypoint 0 (TAKEOFF)
    printf("3. GCS: Sending waypoint 0 (TAKEOFF)\n");
    mavlink_mission_item_int_t wp0;
    wp0.seq = 0;
    wp0.command = MAV_CMD_NAV_TAKEOFF;
    wp0.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp0.x = 0; // lat (home)
    wp0.y = 0; // lon (home)
    wp0.z = 10.0f; // 10m altitude
    wp0.param1 = 15.0f; // pitch angle
    // handleMissionItemInt() processes this

    // Step 4: Vehicle requests waypoint 1
    printf("4. Vehicle: Requesting waypoint 1\n");

    // Step 5: GCS sends waypoint 1 (WAYPOINT)
    printf("5. GCS: Sending waypoint 1 (WAYPOINT)\n");
    mavlink_mission_item_int_t wp1;
    wp1.seq = 1;
    wp1.command = MAV_CMD_NAV_WAYPOINT;
    wp1.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp1.x = 473567890;  // lat (degE7)
    wp1.y = -1223456789; // lon (degE7)
    wp1.z = 20.0f; // 20m altitude
    wp1.param1 = 0.0f; // hold time

    // Step 6: Vehicle requests waypoint 2
    printf("6. Vehicle: Requesting waypoint 2\n");

    // Step 7: GCS sends waypoint 2 (LAND)
    printf("7. GCS: Sending waypoint 2 (LAND)\n");
    mavlink_mission_item_int_t wp2;
    wp2.seq = 2;
    wp2.command = MAV_CMD_NAV_LAND;
    wp2.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    wp2.x = 473568000;
    wp2.y = -1223457000;
    wp2.z = 0.0f;

    // Step 8: Upload complete
    printf("8. Vehicle: Mission upload complete\n");
    printf("   -> Sending MISSION_ACK(MAV_MISSION_ACCEPTED)\n");
    printf("   -> Mission ready to execute\n");
}

// ============================================================================
// USE CASE 3: GEOFENCE CONFIGURATION
// ============================================================================

/**
 * @brief Example: Setting up a circular geofence
 */
void useCase3_GeofenceSetup() {
    printf("\n========================================\n");
    printf("USE CASE 3: Geofence Configuration\n");
    printf("========================================\n");

    printf("\nScenario: GCS sets up 100m radius fence\n");

    // Upload fence points
    printf("\n1. GCS: Uploading fence points\n");

    // Return point (where vehicle goes when fence is breached)
    mavlink_mission_item_int_t returnPoint;
    returnPoint.seq = 0;
    returnPoint.command = MAV_CMD_NAV_FENCE_RETURN_POINT;
    returnPoint.frame = MAV_FRAME_GLOBAL_INT;
    returnPoint.x = 473567890;  // Return lat
    returnPoint.y = -1223456789; // Return lon
    returnPoint.z = 0.0f;
    printf("   -> Fence return point set\n");

    // Circular inclusion fence
    mavlink_mission_item_int_t circleFence;
    circleFence.seq = 1;
    circleFence.command = MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION;
    circleFence.frame = MAV_FRAME_GLOBAL_INT;
    circleFence.x = 473567890;  // Center lat
    circleFence.y = -1223456789; // Center lon
    circleFence.param1 = 100.0f; // Radius in meters
    printf("   -> 100m circular fence set\n");

    // Set altitude limits via parameters
    printf("\n2. GCS: Setting altitude limits\n");
    printf("   -> FENCE_ALT_MAX = 100m\n");
    // This is done via PARAM_SET message

    // Enable fence
    printf("\n3. GCS: Enabling fence\n");
    // GCS sends MAV_CMD_DO_FENCE_ENABLE with param1=1
    printf("   -> Fence enabled and active\n");

    // Monitor fence status
    printf("\n4. Vehicle: Monitoring fence\n");
    printf("   -> Checking boundary every 100ms\n");
    printf("   -> Will send FENCE_STATUS on breach\n");
}

// ============================================================================
// USE CASE 4: RALLY POINTS
// ============================================================================

/**
 * @brief Example: Configuring rally points for RTL
 */
void useCase4_RallyPoints() {
    printf("\n========================================\n");
    printf("USE CASE 4: Rally Point Configuration\n");
    printf("========================================\n");

    printf("\nScenario: GCS sets up 3 rally points\n");

    printf("\n1. GCS: Uploading rally points\n");

    // Rally point 1
    mavlink_mission_item_int_t rally1;
    rally1.seq = 0;
    rally1.command = MAV_CMD_NAV_RALLY_POINT;
    rally1.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    rally1.x = 473567000;
    rally1.y = -1223456000;
    rally1.z = 50.0f; // 50m altitude
    printf("   -> Rally point 1 uploaded\n");

    // Rally point 2
    mavlink_mission_item_int_t rally2;
    rally2.seq = 1;
    rally2.command = MAV_CMD_NAV_RALLY_POINT;
    rally2.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    rally2.x = 473568000;
    rally2.y = -1223457000;
    rally2.z = 50.0f;
    printf("   -> Rally point 2 uploaded\n");

    // Rally point 3
    mavlink_mission_item_int_t rally3;
    rally3.seq = 2;
    rally3.command = MAV_CMD_NAV_RALLY_POINT;
    rally3.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    rally3.x = 473569000;
    rally3.y = -1223458000;
    rally3.z = 50.0f;
    printf("   -> Rally point 3 uploaded\n");

    printf("\n2. Vehicle: Rally points active\n");
    printf("   -> On RTL, will fly to nearest rally point\n");
    printf("   -> Currently nearest: Rally 1 (523m away)\n");
}

// ============================================================================
// USE CASE 5: FILE TRANSFER (FTP)
// ============================================================================

/**
 * @brief Example: Downloading log file via MAVLink FTP
 */
void useCase5_FileTransfer() {
    printf("\n========================================\n");
    printf("USE CASE 5: File Transfer (FTP)\n");
    printf("========================================\n");

    printf("\nScenario: GCS downloads flight log\n");

    // List directory
    printf("\n1. GCS: Listing /logs directory\n");
    // GCS sends FILE_TRANSFER_PROTOCOL with ListDirectory opcode
    printf("   -> Found 5 log files:\n");
    printf("      - flight001.bin (1.2 MB)\n");
    printf("      - flight002.bin (2.1 MB)\n");
    printf("      - flight003.bin (1.8 MB)\n");
    printf("      - flight004.bin (2.3 MB)\n");
    printf("      - flight005.bin (1.5 MB)\n");

    // Open file
    printf("\n2. GCS: Opening flight005.bin\n");
    // GCS sends OpenFileRO opcode
    printf("   -> File opened (size: 1536000 bytes)\n");

    // Read file in chunks
    printf("\n3. GCS: Reading file...\n");
    printf("   -> Reading in 239-byte chunks\n");
    printf("   -> Progress: 0%%...");
    // Multiple ReadFile operations
    printf(" 25%%...");
    printf(" 50%%...");
    printf(" 75%%...");
    printf(" 100%%\n");

    printf("\n4. Transfer complete\n");
    printf("   -> File downloaded successfully\n");
    printf("   -> Transfer time: 45 seconds\n");
    printf("   -> Average speed: 34 KB/s\n");
}

// ============================================================================
// USE CASE 6: MESSAGE SIGNING (SECURITY)
// ============================================================================

/**
 * @brief Example: Setting up secure MAVLink communications
 */
void useCase6_MessageSigning() {
    printf("\n========================================\n");
    printf("USE CASE 6: Message Signing (Security)\n");
    printf("========================================\n");

    printf("\nScenario: Enable message signing for security\n");

    // Generate signing key
    printf("\n1. GCS: Generating signing key\n");
    // 32-byte random key generated
    printf("   -> Key: [REDACTED FOR SECURITY]\n");

    // Setup signing
    printf("\n2. GCS: Sending SETUP_SIGNING message\n");
    mavlink_setup_signing_t setup;
    // setup.secret_key contains the key
    setup.initial_timestamp = 1234567890;
    printf("   -> Signing configured\n");

    // Vehicle enables signing
    printf("\n3. Vehicle: Enabling message signing\n");
    printf("   -> All outgoing messages will be signed\n");
    printf("   -> Signature algorithm: SHA-256 HMAC\n");
    printf("   -> Signature size: 6 bytes (48 bits)\n");

    // Verify incoming messages
    printf("\n4. Vehicle: Verifying incoming messages\n");
    printf("   -> Checking signature on each message\n");
    printf("   -> Checking timestamp (replay protection)\n");
    printf("   -> Rejecting unsigned messages\n");

    printf("\n5. Security active\n");
    printf("   -> Tampered messages will be rejected\n");
    printf("   -> Replay attacks prevented\n");
}

// ============================================================================
// USE CASE 7: STREAM RATE CONFIGURATION
// ============================================================================

/**
 * @brief Example: Adjusting telemetry stream rates
 */
void useCase7_StreamRates() {
    printf("\n========================================\n");
    printf("USE CASE 7: Stream Rate Configuration\n");
    printf("========================================\n");

    printf("\nScenario: GCS configures custom stream rates\n");

    // Set high-rate attitude stream
    printf("\n1. GCS: Setting attitude stream to 50Hz\n");
    // GCS sends REQUEST_DATA_STREAM(EXTRA1, 50Hz)
    printf("   -> ATTITUDE message: 50Hz\n");

    // Set medium-rate position stream
    printf("\n2. GCS: Setting position stream to 5Hz\n");
    // GCS sends REQUEST_DATA_STREAM(POSITION, 5Hz)
    printf("   -> GLOBAL_POSITION_INT: 5Hz\n");

    // Set low-rate status stream
    printf("\n3. GCS: Setting status stream to 1Hz\n");
    // GCS sends REQUEST_DATA_STREAM(EXTENDED_STATUS, 1Hz)
    printf("   -> SYS_STATUS: 1Hz\n");
    printf("   -> BATTERY_STATUS: 1Hz\n");

    // Disable RAW_SENSORS
    printf("\n4. GCS: Disabling RAW_SENSORS stream\n");
    // GCS sends REQUEST_DATA_STREAM(RAW_SENSORS, 0Hz)
    printf("   -> RAW_IMU: disabled\n");
    printf("   -> SCALED_PRESSURE: disabled\n");

    printf("\n5. Custom rates active\n");
    printf("   -> Total bandwidth: ~15 KB/s\n");
}

// ============================================================================
// USE CASE 8: SERVO/RELAY CONTROL
// ============================================================================

/**
 * @brief Example: Controlling auxiliary servos and relays
 */
void useCase8_ServoRelayControl() {
    printf("\n========================================\n");
    printf("USE CASE 8: Servo/Relay Control\n");
    printf("========================================\n");

    printf("\nScenario: GCS controls camera gimbal and cargo release\n");

    // Set servo position
    printf("\n1. GCS: Setting gimbal servo (servo 9) to 1500us\n");
    // GCS sends MAV_CMD_DO_SET_SERVO (9, 1500)
    printf("   -> Servo 9: 1500us (center position)\n");

    // Tilt gimbal down
    printf("\n2. GCS: Tilting gimbal down (servo 9) to 1800us\n");
    // GCS sends MAV_CMD_DO_SET_SERVO (9, 1800)
    printf("   -> Servo 9: 1800us (looking down)\n");

    // Activate relay
    printf("\n3. GCS: Activating cargo release (relay 0)\n");
    // GCS sends MAV_CMD_DO_SET_RELAY (0, 1)
    printf("   -> Relay 0: ON\n");
    printf("   -> Cargo released!\n");

    // Deactivate relay
    printf("\n4. GCS: Deactivating relay 0\n");
    // GCS sends MAV_CMD_DO_SET_RELAY (0, 0)
    printf("   -> Relay 0: OFF\n");

    // Pulse servo repeatedly
    printf("\n5. GCS: Pulsing servo 10 (5 times, 1s interval)\n");
    // GCS sends MAV_CMD_DO_REPEAT_SERVO (10, 2000, 5, 1.0)
    printf("   -> Servo 10: pulsing...\n");
    printf("   -> 1... 2... 3... 4... 5... done\n");
}

// ============================================================================
// USE CASE 9: SERIAL PASSTHROUGH
// ============================================================================

/**
 * @brief Example: Accessing GPS via serial passthrough
 */
void useCase9_SerialPassthrough() {
    printf("\n========================================\n");
    printf("USE CASE 9: Serial Passthrough\n");
    printf("========================================\n");

    printf("\nScenario: GCS configures GPS via passthrough\n");

    // Open GPS port
    printf("\n1. GCS: Opening GPS serial port\n");
    // GCS sends SERIAL_CONTROL (device=0, baudrate=115200)
    printf("   -> Port 0 opened at 115200 baud\n");

    // Send GPS configuration command
    printf("\n2. GCS: Sending GPS config command\n");
    // GCS sends SERIAL_CONTROL with UBX config packet
    printf("   -> Sent: [UBX CFG-RATE command]\n");

    // Receive GPS response
    printf("\n3. Vehicle: GPS response received\n");
    // Vehicle sends SERIAL_CONTROL with GPS response
    printf("   -> GPS acknowledged configuration\n");

    // Close port
    printf("\n4. GCS: Closing serial port\n");
    printf("   -> Port 0 closed\n");
    printf("   -> GPS configured for 10Hz output\n");
}

// ============================================================================
// USE CASE 10: DEVICE OPERATIONS
// ============================================================================

/**
 * @brief Example: Reading IMU registers for diagnostics
 */
void useCase10_DeviceOperations() {
    printf("\n========================================\n");
    printf("USE CASE 10: Device Operations\n");
    printf("========================================\n");

    printf("\nScenario: GCS reads IMU chip ID for diagnostics\n");

    // Read WHO_AM_I register
    printf("\n1. GCS: Reading IMU WHO_AM_I register\n");
    // GCS sends DEVICE_OP_READ (I2C, bus=0, addr=0x68, reg=0x75, count=1)
    printf("   -> Reading I2C bus 0, addr 0x68, reg 0x75\n");

    // Vehicle responds
    printf("\n2. Vehicle: Sending register value\n");
    // Vehicle sends DEVICE_OP_READ_REPLY (result=0, data=0x70)
    printf("   -> Register 0x75 = 0x70 (MPU6050 detected)\n");

    // Read temperature
    printf("\n3. GCS: Reading IMU temperature\n");
    // GCS sends DEVICE_OP_READ (I2C, bus=0, addr=0x68, reg=0x41, count=2)
    printf("   -> Reading 2 bytes from register 0x41\n");

    printf("\n4. Vehicle: Temperature data\n");
    printf("   -> Raw value: 0x0F 0xA0\n");
    printf("   -> Temperature: 25.3°C\n");
}

// ============================================================================
// MAIN - RUN ALL USE CASES
// ============================================================================

int main() {
    printf("\n");
    printf("*********************************************\n");
    printf("* EduCopter GCS_MAVLink Use Case Examples  *\n");
    printf("*********************************************\n");

    // Run all use cases
    useCase1_ParameterManagement();
    useCase2_WaypointMission();
    useCase3_GeofenceSetup();
    useCase4_RallyPoints();
    useCase5_FileTransfer();
    useCase6_MessageSigning();
    useCase7_StreamRates();
    useCase8_ServoRelayControl();
    useCase9_SerialPassthrough();
    useCase10_DeviceOperations();

    printf("\n");
    printf("*********************************************\n");
    printf("* All use cases demonstrated successfully  *\n");
    printf("*********************************************\n\n");

    return 0;
}
