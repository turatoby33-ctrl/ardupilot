/**
 * @file FC_GCS_Integration.cpp
 * @brief Complete Integration Example for EduCopter GCS_MAVLink System
 *
 * This file demonstrates how to integrate the FC_GCS_MAVLink system
 * into your EduCopter flight controller. It shows:
 * - Initialization sequence
 * - Main loop integration
 * - Vehicle-specific GCS channel implementation
 * - Message handler registration
 * - Parameter system setup
 * - Mission/fence/rally integration
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstdio>

// ============================================================================
// STEP 1: IMPLEMENT REQUIRED EXTERNAL FUNCTIONS
// ============================================================================

/**
 * These functions must be provided by your vehicle code.
 * They provide vehicle state to the GCS system.
 */

// Time functions
uint32_t millis() {
    // Return milliseconds since boot
    // Example: return HAL_GetTick();
    static uint32_t counter = 0;
    return counter++;
}

uint16_t millis16() {
    return (uint16_t)(millis() & 0xFFFF);
}

// Attitude functions
float getAttitudeRoll() {
    // Return roll angle in radians
    extern float g_roll; // Your vehicle's roll variable
    return g_roll;
}

float getAttitudePitch() {
    extern float g_pitch;
    return g_pitch;
}

float getAttitudeYaw() {
    extern float g_yaw;
    return g_yaw;
}

float getAttitudeRollRate() {
    extern float g_rollRate;
    return g_rollRate;
}

float getAttitudePitchRate() {
    extern float g_pitchRate;
    return g_pitchRate;
}

float getAttitudeYawRate() {
    extern float g_yawRate;
    return g_yawRate;
}

// Position functions
int32_t getLatitude() {
    // Return latitude in degrees * 1E7
    extern double g_latitude;
    return (int32_t)(g_latitude * 1E7);
}

int32_t getLongitude() {
    // Return longitude in degrees * 1E7
    extern double g_longitude;
    return (int32_t)(g_longitude * 1E7);
}

float getAltitude() {
    // Return altitude MSL in meters
    extern float g_altitude;
    return g_altitude;
}

float getRelativeAltitude() {
    // Return altitude above home in meters
    extern float g_altitudeRelative;
    return g_altitudeRelative;
}

// Velocity functions
int16_t getVelocityX() {
    // Return X velocity in cm/s
    extern float g_velocityNorth;
    return (int16_t)(g_velocityNorth * 100.0f);
}

int16_t getVelocityY() {
    // Return Y velocity in cm/s
    extern float g_velocityEast;
    return (int16_t)(g_velocityEast * 100.0f);
}

int16_t getVelocityZ() {
    // Return Z velocity in cm/s (down positive)
    extern float g_velocityDown;
    return (int16_t)(g_velocityDown * 100.0f);
}

uint16_t getHeading() {
    // Return heading in centidegrees (0-36000)
    float yaw = getAttitudeYaw();
    float heading = yaw * 57.2957795f; // Radians to degrees
    if (heading < 0) heading += 360.0f;
    return (uint16_t)(heading * 100.0f);
}

// Battery functions
float getBatteryVoltage() {
    // Return battery voltage in volts
    extern float g_batteryVoltage;
    return g_batteryVoltage;
}

float getBatteryCurrent() {
    // Return battery current in amperes
    extern float g_batteryCurrent;
    return g_batteryCurrent;
}

int8_t getBatteryRemaining() {
    // Return battery percentage (0-100)
    extern uint8_t g_batteryPercent;
    return (int8_t)g_batteryPercent;
}

// System status functions
uint16_t getCPULoad() {
    // Return CPU load in d% (0-1000 = 0-100%)
    extern float g_cpuUsage;
    return (uint16_t)(g_cpuUsage * 10.0f);
}

uint32_t getSensorsPresentMask() {
    // Return bitmask of sensors present
    uint32_t mask = 0;
    mask |= (1 << 0);  // 3D gyro
    mask |= (1 << 1);  // 3D accelerometer
    mask |= (1 << 2);  // 3D magnetometer
    mask |= (1 << 3);  // Absolute pressure
    mask |= (1 << 5);  // GPS
    return mask;
}

uint32_t getSensorsEnabledMask() {
    // Return bitmask of sensors enabled
    return getSensorsPresentMask(); // Same as present for this example
}

uint32_t getSensorsHealthMask() {
    // Return bitmask of sensors healthy
    extern bool g_sensorsHealthy;
    return g_sensorsHealthy ? getSensorsPresentMask() : 0;
}

// ============================================================================
// STEP 2: IMPLEMENT PARAMETER SYSTEM INTERFACE
// ============================================================================

/**
 * Example parameter table
 */
struct Parameter {
    const char* name;
    float value;
    float min;
    float max;
    float defaultValue;
};

static Parameter s_params[] = {
    {"PID_ROLL_P", 0.15f, 0.0f, 1.0f, 0.15f},
    {"PID_ROLL_I", 0.05f, 0.0f, 0.5f, 0.05f},
    {"PID_ROLL_D", 0.01f, 0.0f, 0.1f, 0.01f},
    {"PID_PITCH_P", 0.15f, 0.0f, 1.0f, 0.15f},
    {"PID_PITCH_I", 0.05f, 0.0f, 0.5f, 0.05f},
    {"PID_PITCH_D", 0.01f, 0.0f, 0.1f, 0.01f},
    {"PID_YAW_P", 0.20f, 0.0f, 1.0f, 0.20f},
    {"PID_YAW_I", 0.02f, 0.0f, 0.5f, 0.02f},
    {"BATT_CAPACITY", 5000.0f, 0.0f, 50000.0f, 5000.0f},
    {"RTL_ALT", 20.0f, 5.0f, 100.0f, 20.0f},
};

const uint16_t PARAM_COUNT = sizeof(s_params) / sizeof(s_params[0]);

uint16_t getParameterCount() {
    return PARAM_COUNT;
}

const char* getParameterName(uint16_t index) {
    if (index >= PARAM_COUNT) return nullptr;
    return s_params[index].name;
}

float getParameterValue(uint16_t index) {
    if (index >= PARAM_COUNT) return 0.0f;
    return s_params[index].value;
}

bool setParameterValue(uint16_t index, float value) {
    if (index >= PARAM_COUNT) return false;

    // Validate range
    if (value < s_params[index].min || value > s_params[index].max) {
        return false;
    }

    s_params[index].value = value;
    return true;
}

int16_t findParameterIndex(const char* name) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(s_params[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

MAV_PARAM_TYPE getParameterType(uint16_t index) {
    return MAV_PARAM_TYPE_REAL32; // All float for this example
}

bool getParameterMetadata(uint16_t index, float& outMin,
                          float& outMax, float& outDefault) {
    if (index >= PARAM_COUNT) return false;
    outMin = s_params[index].min;
    outMax = s_params[index].max;
    outDefault = s_params[index].defaultValue;
    return true;
}

bool saveParameters() {
    // Save to EEPROM/flash
    printf("Saving %u parameters to storage\n", PARAM_COUNT);
    return true;
}

bool loadParameters() {
    // Load from EEPROM/flash
    printf("Loading %u parameters from storage\n", PARAM_COUNT);
    return true;
}

bool resetParameters() {
    // Reset to defaults
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        s_params[i].value = s_params[i].defaultValue;
    }
    return true;
}

// ============================================================================
// STEP 3: CREATE VEHICLE-SPECIFIC GCS CHANNEL
// ============================================================================

namespace EduCopter {
namespace GCS {

/**
 * @class EduCopterGCSChannel
 * @brief Vehicle-specific GCS channel implementation
 */
class EduCopterGCSChannel : public GCSChannel {
public:
    explicit EduCopterGCSChannel(uint8_t channelID)
        : GCSChannel(channelID) {}

    // Override pure virtual functions

    void sendNavControllerOutput() override {
        // Send navigation controller data
        printf("Sending nav controller output\n");
    }

    void sendPIDTuning() override {
        // Send PID tuning data
        printf("Sending PID tuning\n");
    }

    uint8_t getBaseMode() const override {
        // Return MAVLink base mode
        uint8_t mode = 0;

        extern bool g_armed;
        if (g_armed) {
            mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }

        mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;

        return mode;
    }

    MAV_STATE getSystemStatus() const override {
        extern bool g_armed;
        extern bool g_calibrating;

        if (g_calibrating) {
            return MAV_STATE_CALIBRATING;
        }

        if (g_armed) {
            return MAV_STATE_ACTIVE;
        }

        return MAV_STATE_STANDBY;
    }

    void handleMessage(const mavlink_message_t& msg) override {
        // Handle vehicle-specific messages
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_COMMAND_LONG:
                handleCommandLong(msg);
                break;

            case MAVLINK_MSG_ID_COMMAND_INT:
                handleCommandInt(msg);
                break;

            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                handleParamRequestList(msg);
                break;

            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                handleParamRequestRead(msg);
                break;

            case MAVLINK_MSG_ID_PARAM_SET:
                handleParamSet(msg);
                break;

            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                handleRequestDataStream(msg);
                break;

            case MAVLINK_MSG_ID_SERIAL_CONTROL:
                handleSerialControl(msg);
                break;

            default:
                printf("Unhandled message: %u\n", msg.msgid);
                break;
        }
    }
};

} // namespace GCS
} // namespace EduCopter

// ============================================================================
// STEP 4: INITIALIZE AND USE THE GCS SYSTEM
// ============================================================================

using namespace EduCopter::GCS;

/**
 * @brief Initialize GCS system
 */
void initializeGCS() {
    printf("Initializing EduCopter GCS system...\n");

    // Get GCS singleton
    GCS& gcs = GCS::getInstance();

    // Initialize with system ID 1
    if (!gcs.initialize(1)) {
        printf("ERROR: GCS initialization failed!\n");
        return;
    }

    // Create custom channel (override default)
    // Note: In production, modify GCS.cpp to use your custom channel class

    printf("GCS system initialized successfully\n");
    printf("System ID: %u\n", gcs.getSystemID());
    printf("Channels: %u\n", gcs.getChannelCount());
}

/**
 * @brief Main loop - call this from your flight controller main loop
 */
void updateGCS() {
    GCS& gcs = GCS::getInstance();

    // Process incoming messages (10ms max)
    gcs.updateReceive();

    // Send outgoing messages
    gcs.updateSend();
}

/**
 * @brief Send status text to GCS
 */
void sendStatusMessage(const char* text, MAV_SEVERITY severity = MAV_SEVERITY_INFO) {
    GCS& gcs = GCS::getInstance();
    gcs.sendText(severity, text);
}

/**
 * @brief Example: Send a specific message
 */
void sendHeartbeatExample() {
    GCS& gcs = GCS::getInstance();
    gcs.sendMessage(MessageID::HEARTBEAT);
}

/**
 * @brief Example usage in main program
 */
int main() {
    printf("=========================================\n");
    printf("EduCopter GCS Integration Example\n");
    printf("=========================================\n\n");

    // Initialize flight controller
    printf("Initializing flight controller...\n");

    // Initialize GCS
    initializeGCS();

    // Main loop
    printf("\nEntering main loop...\n");
    for (int i = 0; i < 10; i++) {
        printf("\n--- Loop iteration %d ---\n", i);

        // Update flight controller
        // ... your FC update code ...

        // Update GCS
        updateGCS();

        // Example: Send status message
        if (i == 5) {
            sendStatusMessage("Halfway through test!");
        }

        // Simulate delay
        // delay(100);
    }

    printf("\nGCS integration example complete!\n");
    return 0;
}
