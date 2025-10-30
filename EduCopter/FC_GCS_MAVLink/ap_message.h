/**
 * @file ap_message.h
 * @brief EduCopter MAVLink Message ID Enumeration
 *
 * Custom implementation for EduCopter educational multirotor platform.
 * Defines internal message identifiers used for scheduling and queuing
 * telemetry messages to ground control stations.
 *
 * @note These are NOT MAVLink message IDs - they are internal enums
 *       that map to MAVLink messages during transmission.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include <stdint.h>

namespace EduCopter {
namespace GCS {

/**
 * @enum MessageID
 * @brief Internal message identifiers for EduCopter telemetry system
 *
 * These IDs are used internally to queue and schedule messages.
 * Each ID corresponds to one or more MAVLink messages that will
 * be sent to the ground station.
 */
enum class MessageID : uint8_t {
    // ========== CRITICAL MESSAGES (Highest Priority) ==========

    /// System heartbeat - sent at 1 Hz, indicates vehicle is alive
    HEARTBEAT = 0,

    /// System status - battery, sensors, CPU load, errors
    SYSTEM_STATUS = 1,

    /// Power status - voltages, current draw
    POWER_STATUS = 2,

    // ========== ATTITUDE & NAVIGATION ==========

    /// Roll, pitch, yaw angles (Euler)
    ATTITUDE = 10,

    /// Attitude as quaternion (more accurate)
    ATTITUDE_QUATERNION = 11,

    /// Global position (latitude, longitude, altitude)
    GLOBAL_POSITION = 12,

    /// Local position (NED frame)
    LOCAL_POSITION = 13,

    /// VFR HUD data (airspeed, groundspeed, heading, altitude, climb rate)
    VFR_HUD = 14,

    /// Navigation controller output (crosstrack error, bearing, etc.)
    NAV_CONTROLLER_OUTPUT = 15,

    // ========== SENSORS ==========

    /// Raw IMU data (accelerometer, gyro, magnetometer)
    RAW_IMU = 20,

    /// Scaled IMU data (in physical units)
    SCALED_IMU = 21,

    /// Secondary IMU (if available)
    SCALED_IMU2 = 22,

    /// Tertiary IMU (if available)
    SCALED_IMU3 = 23,

    /// Barometric pressure sensor
    SCALED_PRESSURE = 24,

    /// Secondary barometer
    SCALED_PRESSURE2 = 25,

    /// Tertiary barometer
    SCALED_PRESSURE3 = 26,

    /// GPS position and velocity
    GPS_RAW = 27,

    /// GPS RTK data (high-precision)
    GPS_RTK = 28,

    /// Secondary GPS
    GPS2_RAW = 29,

    /// Secondary GPS RTK
    GPS2_RTK = 30,

    // ========== CONTROL INPUTS/OUTPUTS ==========

    /// RC channel values (from receiver)
    RC_CHANNELS = 40,

    /// RC channels raw (PWM values)
    RC_CHANNELS_RAW = 41,

    /// Servo/motor output values
    SERVO_OUTPUT_RAW = 42,

    // ========== MISSION & WAYPOINTS ==========

    /// Current mission item being executed
    MISSION_CURRENT = 50,

    /// Mission item reached notification
    MISSION_ITEM_REACHED = 51,

    /// Request next waypoint from GCS
    MISSION_REQUEST_NEXT_WAYPOINT = 52,

    /// Request next fence point from GCS
    MISSION_REQUEST_NEXT_FENCE = 53,

    /// Request next rally point from GCS
    MISSION_REQUEST_NEXT_RALLY = 54,

    // ========== PARAMETERS ==========

    /// Next parameter value to send
    PARAMETER_VALUE = 60,

    // ========== BATTERY & POWER ==========

    /// Battery status (voltage, current, remaining capacity)
    BATTERY_STATUS = 70,

    /// Battery #2 status
    BATTERY2_STATUS = 71,

    // ========== RANGEFINDERS & DISTANCE SENSORS ==========

    /// Rangefinder/Lidar data
    RANGEFINDER = 80,

    /// Distance sensor (multiple orientations)
    DISTANCE_SENSOR = 81,

    /// Proximity sensor (360° obstacle detection)
    PROXIMITY = 82,

    // ========== OPTICAL FLOW ==========

    /// Optical flow sensor data
    OPTICAL_FLOW = 90,

    // ========== FENCE & SAFETY ==========

    /// Geofence status (breached, distance to fence, etc.)
    FENCE_STATUS = 100,

    // ========== TIME ==========

    /// System time (boot time + GPS time)
    SYSTEM_TIME = 110,

    // ========== EXTENDED STATUS ==========

    /// Extended system state (VTOL state, landed state)
    EXTENDED_SYS_STATE = 120,

    /// Autopilot version and capabilities
    AUTOPILOT_VERSION = 121,

    // ========== VIBRATION ==========

    /// Vibration levels and clipping
    VIBRATION = 130,

    // ========== MOTOR/ESC ==========

    /// RPM sensor data
    RPM = 140,

    /// ESC telemetry (voltage, current, RPM, temperature per ESC)
    ESC_TELEMETRY = 141,

    // ========== TUNING & DEBUG ==========

    /// PID tuning data (for real-time tuning)
    PID_TUNING = 150,

    /// Named debug value (float)
    NAMED_FLOAT = 151,

    /// Memory information
    MEMINFO = 152,

    /// Hardware status
    HWSTATUS = 153,

    // ========== AHRS & EKF ==========

    /// AHRS debug information
    AHRS = 160,

    /// AHRS2 debug information
    AHRS2 = 161,

    /// EKF status report
    EKF_STATUS_REPORT = 162,

    // ========== SIMULATION (SITL) ==========

    /// Simulation state (for SITL testing)
    SIMSTATE = 170,

    /// Simulation state (newer format)
    SIM_STATE = 171,

    // ========== POSITION TARGETS ==========

    /// Position setpoint (global coordinates)
    POSITION_TARGET_GLOBAL = 180,

    /// Position setpoint (local NED)
    POSITION_TARGET_LOCAL = 181,

    /// Attitude setpoint
    ATTITUDE_TARGET = 182,

    // ========== CAMERA & GIMBAL ==========

    /// Camera feedback (trigger confirmation)
    CAMERA_FEEDBACK = 190,

    /// Camera information (resolution, etc.)
    CAMERA_INFORMATION = 191,

    /// Camera settings (mode, zoom, etc.)
    CAMERA_SETTINGS = 192,

    /// Camera capture status
    CAMERA_CAPTURE_STATUS = 193,

    /// Gimbal device attitude
    GIMBAL_DEVICE_ATTITUDE = 194,

    /// Gimbal manager information
    GIMBAL_MANAGER_INFO = 195,

    /// Gimbal manager status
    GIMBAL_MANAGER_STATUS = 196,

    // ========== COLLISION AVOIDANCE ==========

    /// ADSB vehicle (traffic)
    ADSB_VEHICLE = 200,

    // ========== ENVIRONMENTAL ==========

    /// Wind estimate
    WIND = 210,

    // ========== HOME & ORIGIN ==========

    /// Home position
    HOME_POSITION = 220,

    /// GPS global origin
    GPS_GLOBAL_ORIGIN = 221,

    // ========== HIGH LATENCY ==========

    /// Compressed telemetry for satellite links
    HIGH_LATENCY2 = 230,

    // ========== GENERATOR ==========

    /// Generator status
    GENERATOR_STATUS = 240,

    // ========== RELAY ==========

    /// Relay status
    RELAY_STATUS = 250,

    // ========== SENTINEL ==========

    /// Marks the end of valid message IDs - MUST BE LAST
    MESSAGE_COUNT
};

/**
 * @brief Get human-readable name for a message ID
 * @param id Message identifier
 * @return Const string with message name
 */
const char* messageIDToString(MessageID id);

/**
 * @brief Check if message ID is valid
 * @param id Message identifier to check
 * @return true if valid, false otherwise
 */
inline bool isValidMessageID(MessageID id) {
    return id < MessageID::MESSAGE_COUNT;
}

/**
 * @brief Convert message ID to integer
 * @param id Message identifier
 * @return uint8_t representation
 */
inline uint8_t messageIDToInt(MessageID id) {
    return static_cast<uint8_t>(id);
}

} // namespace GCS
} // namespace EduCopter
