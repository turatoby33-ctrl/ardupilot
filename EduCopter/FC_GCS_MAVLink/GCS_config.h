/**
 * @file GCS_config.h
 * @brief EduCopter GCS Configuration and Feature Flags
 *
 * Custom configuration for EduCopter GCS_MAVLink system.
 * Controls which features are compiled into the firmware.
 *
 * @author EduCopter Project
 * @date 2025-10-28
 * @version 1.0.0
 */

#pragma once

#include <stdint.h>

namespace EduCopter {
namespace GCS {
namespace Config {

// ========== CORE GCS CONFIGURATION ==========

/// Enable/disable entire GCS system
#ifndef EDUCOPTER_GCS_ENABLED
#define EDUCOPTER_GCS_ENABLED 1
#endif

/// Maximum number of simultaneous MAVLink connections
#ifndef EDUCOPTER_MAX_MAVLINK_CHANNELS
#define EDUCOPTER_MAX_MAVLINK_CHANNELS 4
#endif

/// Enable MAVLink 2.0 protocol (recommended)
#ifndef EDUCOPTER_MAVLINK2_ENABLED
#define EDUCOPTER_MAVLINK2_ENABLED 1
#endif

// ========== SECURITY ==========

/// Enable MAVLink message signing (authentication)
#ifndef EDUCOPTER_MAVLINK_SIGNING_ENABLED
#define EDUCOPTER_MAVLINK_SIGNING_ENABLED 1
#endif

// ========== TELEMETRY FEATURES ==========

/// Enable high-precision IMU telemetry
#ifndef EDUCOPTER_HIGHRES_IMU_ENABLED
#define EDUCOPTER_HIGHRES_IMU_ENABLED 1
#endif

/// Enable battery monitoring
#ifndef EDUCOPTER_BATTERY_MONITORING_ENABLED
#define EDUCOPTER_BATTERY_MONITORING_ENABLED 1
#endif

/// Enable GPS telemetry
#ifndef EDUCOPTER_GPS_ENABLED
#define EDUCOPTER_GPS_ENABLED 1
#endif

/// Enable RC channels telemetry
#ifndef EDUCOPTER_RC_TELEMETRY_ENABLED
#define EDUCOPTER_RC_TELEMETRY_ENABLED 1
#endif

// ========== MISSION MANAGEMENT ==========

/// Enable mission (waypoint) upload/download
#ifndef EDUCOPTER_MISSION_ENABLED
#define EDUCOPTER_MISSION_ENABLED 1
#endif

/// Enable geofence management
#ifndef EDUCOPTER_FENCE_ENABLED
#define EDUCOPTER_FENCE_ENABLED 1
#endif

/// Enable rally point management
#ifndef EDUCOPTER_RALLY_ENABLED
#define EDUCOPTER_RALLY_ENABLED 1
#endif

/// Maximum number of mission items
#ifndef EDUCOPTER_MAX_MISSION_ITEMS
#define EDUCOPTER_MAX_MISSION_ITEMS 100
#endif

/// Maximum number of fence points
#ifndef EDUCOPTER_MAX_FENCE_POINTS
#define EDUCOPTER_MAX_FENCE_POINTS 50
#endif

/// Maximum number of rally points
#ifndef EDUCOPTER_MAX_RALLY_POINTS
#define EDUCOPTER_MAX_RALLY_POINTS 10
#endif

// ========== PARAMETER SYSTEM ==========

/// Enable parameter get/set protocol
#ifndef EDUCOPTER_PARAMETERS_ENABLED
#define EDUCOPTER_PARAMETERS_ENABLED 1
#endif

/// Maximum number of parameters
#ifndef EDUCOPTER_MAX_PARAMETERS
#define EDUCOPTER_MAX_PARAMETERS 500
#endif

// ========== FILE TRANSFER (FTP) ==========

/// Enable MAVLink FTP for file upload/download
#ifndef EDUCOPTER_FTP_ENABLED
#define EDUCOPTER_FTP_ENABLED 1
#endif

/// Maximum number of concurrent FTP sessions
#ifndef EDUCOPTER_MAX_FTP_SESSIONS
#define EDUCOPTER_MAX_FTP_SESSIONS 3
#endif

// ========== ADVANCED FEATURES ==========

/// Enable serial port passthrough
#ifndef EDUCOPTER_SERIAL_CONTROL_ENABLED
#define EDUCOPTER_SERIAL_CONTROL_ENABLED 1
#endif

/// Enable device operations (I2C/SPI via MAVLink)
#ifndef EDUCOPTER_DEVICE_OP_ENABLED
#define EDUCOPTER_DEVICE_OP_ENABLED 0  // Disabled by default (security)
#endif

/// Enable servo/relay direct control
#ifndef EDUCOPTER_SERVO_RELAY_ENABLED
#define EDUCOPTER_SERVO_RELAY_ENABLED 1
#endif

/// Enable high-latency link support (satellite)
#ifndef EDUCOPTER_HIGH_LATENCY_ENABLED
#define EDUCOPTER_HIGH_LATENCY_ENABLED 1
#endif

// ========== CAMERA & GIMBAL ==========

/// Enable camera trigger/feedback
#ifndef EDUCOPTER_CAMERA_ENABLED
#define EDUCOPTER_CAMERA_ENABLED 1
#endif

/// Enable gimbal control
#ifndef EDUCOPTER_GIMBAL_ENABLED
#define EDUCOPTER_GIMBAL_ENABLED 1
#endif

// ========== COLLISION AVOIDANCE ==========

/// Enable ADSB/ADS-B traffic reception
#ifndef EDUCOPTER_ADSB_ENABLED
#define EDUCOPTER_ADSB_ENABLED 1
#endif

// ========== OPTICAL FLOW ==========

/// Enable optical flow sensor
#ifndef EDUCOPTER_OPTICAL_FLOW_ENABLED
#define EDUCOPTER_OPTICAL_FLOW_ENABLED 1
#endif

// ========== RANGEFINDERS ==========

/// Enable rangefinder/Lidar
#ifndef EDUCOPTER_RANGEFINDER_ENABLED
#define EDUCOPTER_RANGEFINDER_ENABLED 1
#endif

/// Enable proximity sensors (360° obstacle detection)
#ifndef EDUCOPTER_PROXIMITY_ENABLED
#define EDUCOPTER_PROXIMITY_ENABLED 1
#endif

// ========== DEBUG & DEVELOPMENT ==========

/// Enable debug message timing (profiling)
#ifndef EDUCOPTER_GCS_DEBUG_TIMING
#define EDUCOPTER_GCS_DEBUG_TIMING 0  // Disabled by default
#endif

/// Enable verbose logging
#ifndef EDUCOPTER_GCS_VERBOSE_LOGGING
#define EDUCOPTER_GCS_VERBOSE_LOGGING 0  // Disabled by default
#endif

/// Enable developer failure injection commands (testing only)
#ifndef EDUCOPTER_FAILURE_INJECTION_ENABLED
#define EDUCOPTER_FAILURE_INJECTION_ENABLED 0  // Disabled by default
#endif

// ========== BUFFER SIZES ==========

/// MAVLink TX buffer size per channel (bytes)
#ifndef EDUCOPTER_MAVLINK_TX_BUFFER_SIZE
#define EDUCOPTER_MAVLINK_TX_BUFFER_SIZE 4096
#endif

/// MAVLink RX buffer size per channel (bytes)
#ifndef EDUCOPTER_MAVLINK_RX_BUFFER_SIZE
#define EDUCOPTER_MAVLINK_RX_BUFFER_SIZE 2048
#endif

/// Statustext queue size (number of messages)
#ifndef EDUCOPTER_STATUSTEXT_QUEUE_SIZE
#define EDUCOPTER_STATUSTEXT_QUEUE_SIZE 10
#endif

// ========== TIMING CONFIGURATION ==========

/// Heartbeat rate (Hz)
#ifndef EDUCOPTER_HEARTBEAT_RATE_HZ
#define EDUCOPTER_HEARTBEAT_RATE_HZ 1
#endif

/// Default telemetry stream rate (Hz) - if not configured
#ifndef EDUCOPTER_DEFAULT_STREAM_RATE_HZ
#define EDUCOPTER_DEFAULT_STREAM_RATE_HZ 4
#endif

/// Parameter send rate (parameters/second)
#ifndef EDUCOPTER_PARAMETER_SEND_RATE
#define EDUCOPTER_PARAMETER_SEND_RATE 50
#endif

/// Mission item timeout (milliseconds)
#ifndef EDUCOPTER_MISSION_TIMEOUT_MS
#define EDUCOPTER_MISSION_TIMEOUT_MS 5000
#endif

// ========== COMPATIBILITY ==========

/// Support legacy COMMAND_LONG (not just COMMAND_INT)
#ifndef EDUCOPTER_COMMAND_LONG_ENABLED
#define EDUCOPTER_COMMAND_LONG_ENABLED 1
#endif

/// Support legacy MISSION_REQUEST (not just MISSION_REQUEST_INT)
#ifndef EDUCOPTER_MISSION_REQUEST_LEGACY_ENABLED
#define EDUCOPTER_MISSION_REQUEST_LEGACY_ENABLED 1
#endif

// ========== EDUCATIONAL FEATURES ==========

/// Enable detailed status messages for students
#ifndef EDUCOPTER_EDUCATIONAL_MESSAGES
#define EDUCOPTER_EDUCATIONAL_MESSAGES 1
#endif

/// Enable performance monitoring messages
#ifndef EDUCOPTER_PERFORMANCE_MONITORING
#define EDUCOPTER_PERFORMANCE_MONITORING 1
#endif

/// Enable flight mode explanation messages
#ifndef EDUCOPTER_MODE_EXPLANATIONS
#define EDUCOPTER_MODE_EXPLANATIONS 1
#endif

// ========== COMPILE-TIME CHECKS ==========

#if EDUCOPTER_MAX_MAVLINK_CHANNELS > 8
#error "Maximum 8 MAVLink channels supported"
#endif

#if EDUCOPTER_MAX_MISSION_ITEMS > 1000
#error "Maximum 1000 mission items supported"
#endif

#if EDUCOPTER_MAVLINK_TX_BUFFER_SIZE < 1024
#error "TX buffer must be at least 1024 bytes"
#endif

// ========== FEATURE DEPENDENCY CHECKS ==========

#if EDUCOPTER_FTP_ENABLED && !EDUCOPTER_GCS_ENABLED
#error "FTP requires GCS to be enabled"
#endif

#if EDUCOPTER_MISSION_ENABLED && !EDUCOPTER_PARAMETERS_ENABLED
#error "Mission management requires parameter system"
#endif

#if EDUCOPTER_MAVLINK_SIGNING_ENABLED && !EDUCOPTER_MAVLINK2_ENABLED
#error "Message signing requires MAVLink 2.0"
#endif

} // namespace Config
} // namespace GCS
} // namespace EduCopter
