/**
 * @file QUICKSTART_TEMPLATE.cpp
 * @brief Quick Start Template for EduCopter GCS Integration
 *
 * Copy this file to your project and fill in the TODOs.
 * This template provides the minimum code needed to get GCS working.
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"

// ============================================================================
// STEP 1: INCLUDE YOUR VEHICLE HEADERS
// ============================================================================

// TODO: Include your flight controller headers here
// #include "your_imu.h"
// #include "your_gps.h"
// #include "your_battery.h"
// etc...

// ============================================================================
// STEP 2: IMPLEMENT REQUIRED TIME FUNCTIONS
// ============================================================================

uint32_t millis() {
    // TODO: Return milliseconds since boot
    // Examples:
    // - return HAL_GetTick();                    (STM32)
    // - return millis();                         (Arduino)
    // - return (uint32_t)(clock() * 1000 / CLOCKS_PER_SEC);  (Linux)
    return 0; // REPLACE THIS
}

uint16_t millis16() {
    return (uint16_t)(millis() & 0xFFFF);
}

// ============================================================================
// STEP 3: IMPLEMENT ATTITUDE FUNCTIONS
// ============================================================================

float getAttitudeRoll() {
    // TODO: Return roll angle in RADIANS
    // Example: return your_imu.roll * DEG_TO_RAD;
    return 0.0f; // REPLACE THIS
}

float getAttitudePitch() {
    // TODO: Return pitch angle in RADIANS
    return 0.0f; // REPLACE THIS
}

float getAttitudeYaw() {
    // TODO: Return yaw angle in RADIANS
    return 0.0f; // REPLACE THIS
}

float getAttitudeRollRate() {
    // TODO: Return roll rate in RAD/S
    return 0.0f; // REPLACE THIS
}

float getAttitudePitchRate() {
    // TODO: Return pitch rate in RAD/S
    return 0.0f; // REPLACE THIS
}

float getAttitudeYawRate() {
    // TODO: Return yaw rate in RAD/S
    return 0.0f; // REPLACE THIS
}

// ============================================================================
// STEP 4: IMPLEMENT POSITION FUNCTIONS
// ============================================================================

int32_t getLatitude() {
    // TODO: Return latitude in degrees * 10^7
    // Example: return (int32_t)(your_gps.latitude * 1E7);
    return 0; // REPLACE THIS
}

int32_t getLongitude() {
    // TODO: Return longitude in degrees * 10^7
    return 0; // REPLACE THIS
}

float getAltitude() {
    // TODO: Return altitude above mean sea level in METERS
    return 0.0f; // REPLACE THIS
}

float getRelativeAltitude() {
    // TODO: Return altitude above home/ground in METERS
    return 0.0f; // REPLACE THIS
}

int16_t getVelocityX() {
    // TODO: Return north velocity in CM/S
    return 0; // REPLACE THIS
}

int16_t getVelocityY() {
    // TODO: Return east velocity in CM/S
    return 0; // REPLACE THIS
}

int16_t getVelocityZ() {
    // TODO: Return down velocity in CM/S (positive = down)
    return 0; // REPLACE THIS
}

uint16_t getHeading() {
    // TODO: Return heading in CENTIDEGREES (0-36000)
    // Example: return (uint16_t)((your_compass.heading % 360) * 100);
    return 0; // REPLACE THIS
}

// ============================================================================
// STEP 5: IMPLEMENT BATTERY FUNCTIONS
// ============================================================================

float getBatteryVoltage() {
    // TODO: Return battery voltage in VOLTS
    // Example: return your_battery.voltage;
    return 11.1f; // REPLACE THIS
}

float getBatteryCurrent() {
    // TODO: Return battery current in AMPERES
    return 5.0f; // REPLACE THIS
}

int8_t getBatteryRemaining() {
    // TODO: Return battery percentage (0-100)
    return 75; // REPLACE THIS
}

// ============================================================================
// STEP 6: IMPLEMENT SYSTEM STATUS FUNCTIONS
// ============================================================================

uint16_t getCPULoad() {
    // TODO: Return CPU load in deci-percent (0-1000 = 0-100%)
    return 250; // REPLACE THIS (25% = 250)
}

uint32_t getSensorsPresentMask() {
    // TODO: Return bitmask of present sensors
    // Bit 0  = 3D gyro
    // Bit 1  = 3D accelerometer
    // Bit 2  = 3D magnetometer
    // Bit 3  = Absolute pressure
    // Bit 5  = GPS
    // See MAV_SYS_STATUS_SENSOR enum for full list
    return 0x0000002F; // REPLACE THIS (gyro+accel+mag+pressure+gps)
}

uint32_t getSensorsEnabledMask() {
    // TODO: Return bitmask of enabled sensors
    return getSensorsPresentMask(); // Can be same as present
}

uint32_t getSensorsHealthMask() {
    // TODO: Return bitmask of healthy sensors
    // Example: Check if each sensor is working
    return getSensorsPresentMask(); // Can be same if all healthy
}

// ============================================================================
// STEP 7: IMPLEMENT PARAMETER SYSTEM (MINIMUM)
// ============================================================================

// Define your parameters here
struct Param {
    const char* name;
    float value;
};

// TODO: Add your parameters
static Param params[] = {
    {"SYSID", 1.0f},
    {"COMPID", 1.0f},
    // Add more parameters...
};

const int PARAM_COUNT = sizeof(params) / sizeof(params[0]);

uint16_t getParameterCount() {
    return PARAM_COUNT;
}

const char* getParameterName(uint16_t index) {
    if (index >= PARAM_COUNT) return nullptr;
    return params[index].name;
}

float getParameterValue(uint16_t index) {
    if (index >= PARAM_COUNT) return 0.0f;
    return params[index].value;
}

bool setParameterValue(uint16_t index, float value) {
    if (index >= PARAM_COUNT) return false;
    params[index].value = value;
    return true;
}

int16_t findParameterIndex(const char* name) {
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(params[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

MAV_PARAM_TYPE getParameterType(uint16_t index) {
    return MAV_PARAM_TYPE_REAL32;
}

// Optional: Parameter persistence
bool saveParameters() { return true; }
bool loadParameters() { return true; }
bool resetParameters() { return true; }

// ============================================================================
// STEP 8: IMPLEMENT MISSION STORAGE (MINIMUM - CAN SKIP INITIALLY)
// ============================================================================

// Simple waypoint storage
static mavlink_mission_item_int_t waypoints[50];
static uint16_t waypoint_count = 0;

uint16_t getWaypointCount() { return waypoint_count; }
bool getWaypoint(uint16_t index, mavlink_mission_item_int_t& outItem) {
    if (index >= waypoint_count) return false;
    outItem = waypoints[index];
    return true;
}
bool appendWaypoint(const mavlink_mission_item_int_t& item) {
    if (waypoint_count >= 50) return false;
    waypoints[waypoint_count++] = item;
    return true;
}
bool clearWaypoints() {
    waypoint_count = 0;
    return true;
}
uint16_t getCurrentWaypointIndex() { return 0; }
bool setCurrentWaypoint(uint16_t index) { return true; }

// Fence storage (can leave empty initially)
uint16_t getFencePointCount() { return 0; }
bool getFencePoint(uint16_t index, mavlink_mission_item_int_t& outItem) { return false; }
bool appendFencePoint(const mavlink_mission_item_int_t& item) { return false; }
bool clearFencePoints() { return true; }
bool isFenceEnabled() { return false; }
bool setFenceEnabled(bool enabled) { return false; }

// Rally storage (can leave empty initially)
uint16_t getRallyPointCount() { return 0; }
bool getRallyPoint(uint16_t index, mavlink_mission_item_int_t& outItem) { return false; }
bool appendRallyPoint(const mavlink_mission_item_int_t& item) { return false; }
bool clearRallyPoints() { return true; }

// ============================================================================
// STEP 9: CREATE YOUR CUSTOM GCS CHANNEL
// ============================================================================

namespace EduCopter {
namespace GCS {

// Global variables for vehicle state (declare these in your main code)
extern bool g_armed;
extern bool g_calibrating;

class MyGCSChannel : public GCSChannel {
public:
    explicit MyGCSChannel(uint8_t channelID) : GCSChannel(channelID) {}

    void sendNavControllerOutput() override {
        // TODO: Send navigation data (optional)
    }

    void sendPIDTuning() override {
        // TODO: Send PID tuning data (optional)
    }

    uint8_t getBaseMode() const override {
        uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

        // TODO: Set mode flags based on your vehicle state
        if (g_armed) {
            mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }

        mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;

        return mode;
    }

    MAV_STATE getSystemStatus() const override {
        // TODO: Return appropriate system state
        if (g_calibrating) {
            return MAV_STATE_CALIBRATING;
        }
        if (g_armed) {
            return MAV_STATE_ACTIVE;
        }
        return MAV_STATE_STANDBY;
    }
};

} // namespace GCS
} // namespace EduCopter

// ============================================================================
// STEP 10: INITIALIZE AND USE IN YOUR MAIN PROGRAM
// ============================================================================

using namespace EduCopter::GCS;

// Global vehicle state
bool g_armed = false;
bool g_calibrating = false;

void setup() {
    // TODO: Initialize your hardware
    // init_imu();
    // init_gps();
    // init_battery();
    // etc...

    // Initialize GCS
    GCS& gcs = GCS::getInstance();
    if (!gcs.initialize(1)) { // System ID = 1
        // Handle error
    }

    // GCS is ready!
}

void loop() {
    // TODO: Update your flight controller
    // update_imu();
    // update_estimator();
    // update_controller();
    // update_motors();

    // Update GCS (REQUIRED - add these 2 lines)
    GCS& gcs = GCS::getInstance();
    gcs.updateReceive();  // Process incoming MAVLink messages
    gcs.updateSend();     // Send outgoing telemetry

    // TODO: Rest of your loop
}

// ============================================================================
// OPTIONAL: HELPER FUNCTIONS
// ============================================================================

// Send status message to GCS
void gcs_send_text(const char* msg) {
    GCS::getInstance().sendText(MAV_SEVERITY_INFO, msg);
}

// Send warning to GCS
void gcs_send_warning(const char* msg) {
    GCS::getInstance().sendText(MAV_SEVERITY_WARNING, msg);
}

// Send error to GCS
void gcs_send_error(const char* msg) {
    GCS::getInstance().sendText(MAV_SEVERITY_ERROR, msg);
}

// ============================================================================
// MAIN (for testing)
// ============================================================================

int main() {
    setup();

    while (true) {
        loop();
        // Add delay if needed
    }

    return 0;
}

/*
================================================================================
CHECKLIST:
================================================================================

□ Step 1:  Included your vehicle headers
□ Step 2:  Implemented millis() function
□ Step 3:  Implemented all attitude functions (6 functions)
□ Step 4:  Implemented all position functions (8 functions)
□ Step 5:  Implemented all battery functions (3 functions)
□ Step 6:  Implemented all system status functions (3 functions)
□ Step 7:  Created parameter table
□ Step 8:  (Optional) Implemented mission storage
□ Step 9:  Created custom GCS channel class
□ Step 10: Added gcs.initialize() to setup()
□ Step 11: Added gcs.updateReceive() to loop()
□ Step 12: Added gcs.updateSend() to loop()
□ Step 13: Compiled successfully
□ Step 14: Tested connection with Mission Planner/QGC

================================================================================
TESTING:
================================================================================

1. Connect via USB/UART at 57600 or 115200 baud
2. Open Mission Planner or QGroundControl
3. You should see:
   - Heartbeat messages
   - System status updating
   - Attitude display working
   - GPS position (if GPS connected)
   - Battery voltage
   - Parameters list

4. Test parameter read/write
5. Test mission upload
6. Test telemetry stream rates

================================================================================
TROUBLESHOOTING:
================================================================================

No connection?
  → Check UART pins and baudrate
  → Verify updateReceive() is being called
  → Add debug print in setup()

Parameters not showing?
  → Check getParameterCount() returns > 0
  → Verify parameter names are valid

Telemetry not updating?
  → Check all get*() functions return valid data
  → Verify updateSend() is being called
  → Check stream rates are configured

================================================================================
*/
