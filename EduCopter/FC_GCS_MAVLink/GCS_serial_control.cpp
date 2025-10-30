/**
 * @file GCS_serial_control.cpp
 * @brief Serial port control and passthrough via MAVLink
 *
 * This file implements SERIAL_CONTROL message handling which allows:
 * - Direct access to serial ports via MAVLink
 * - GPS configuration and debugging
 * - External device communication
 * - Console/shell access
 * - Firmware upload to companion computers
 *
 * Useful for:
 * - Configuring GPS receivers
 * - Accessing system shell
 * - Debugging serial devices
 * - Uploading firmware to external processors
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>

namespace EduCopter {
namespace GCS {

#if EDUCOPTER_SERIAL_CONTROL_ENABLED

// External serial interface (implemented by vehicle)
extern bool serial_openPort(uint8_t device, uint32_t baudrate);
extern bool serial_closePort(uint8_t device);
extern int32_t serial_readPort(uint8_t device, uint8_t* buffer, uint32_t length);
extern int32_t serial_writePort(uint8_t device, const uint8_t* data, uint32_t length);
extern uint32_t serial_available(uint8_t device);
extern void serial_setBaudrate(uint8_t device, uint32_t baudrate);
extern bool serial_isPortValid(uint8_t device);

// Serial control state
struct SerialControlState {
    bool active;
    uint8_t device;
    uint32_t baudrate;
    uint32_t timeout;
    uint32_t lastActivityMS;
    uint8_t targetSystem;
    uint8_t targetComponent;
};

static SerialControlState s_serialControl = {false, 0, 0, 0, 0, 0, 0};

// Timeout for serial control session (30 seconds)
static const uint32_t SERIAL_CONTROL_TIMEOUT_MS = 30000;

/**
 * @brief Handle SERIAL_CONTROL message
 *
 * Processes serial port passthrough requests.
 */
void GCSChannel::handleSerialControl(const mavlink_message_t& msg)
{
    mavlink_serial_control_t packet;
    mavlink_msg_serial_control_decode(&msg, &packet);

    // Validate device
    if (!serial_isPortValid(packet.device)) {
        sendText(MAV_SEVERITY_WARNING, "Invalid serial device");
        return;
    }

    // Update session info
    s_serialControl.targetSystem = msg.sysid;
    s_serialControl.targetComponent = msg.compid;
    s_serialControl.lastActivityMS = millis();

    // Process flags
    bool blockingMode = (packet.flags & SERIAL_CONTROL_FLAG_BLOCKING);
    bool respondFlag = (packet.flags & SERIAL_CONTROL_FLAG_RESPOND);

    // Handle device open/close
    if (packet.flags & SERIAL_CONTROL_FLAG_REPLY) {
        // This is a reply from vehicle - shouldn't happen
        return;
    }

    // Check if we need to open/reconfigure port
    if (!s_serialControl.active ||
        s_serialControl.device != packet.device ||
        s_serialControl.baudrate != packet.baudrate) {

        // Close previous port
        if (s_serialControl.active) {
            serial_closePort(s_serialControl.device);
        }

        // Open new port
        if (!serial_openPort(packet.device, packet.baudrate)) {
            sendText(MAV_SEVERITY_ERROR, "Failed to open serial port");
            s_serialControl.active = false;
            return;
        }

        s_serialControl.active = true;
        s_serialControl.device = packet.device;
        s_serialControl.baudrate = packet.baudrate;
        s_serialControl.timeout = packet.timeout;

        char buf[80];
        snprintf(buf, sizeof(buf), "Serial port %u opened at %u baud",
                 packet.device, packet.baudrate);
        sendText(MAV_SEVERITY_INFO, buf);
    }

    // Write data to serial port
    if (packet.count > 0 && packet.count <= 70) {
        int32_t written = serial_writePort(packet.device, packet.data, packet.count);

        if (written < 0) {
            sendText(MAV_SEVERITY_WARNING, "Serial write failed");
        }
    }

    // Read and send response if requested
    if (respondFlag) {
        sendSerialControlResponse(packet.device, packet.flags);
    }
}

/**
 * @brief Send SERIAL_CONTROL response
 *
 * Sends data read from serial port back to GCS.
 */
void GCSChannel::sendSerialControlResponse(uint8_t device, uint8_t flags)
{
    if (!s_serialControl.active) {
        return;
    }

    if (!hasPayloadSpace(MAVLINK_MSG_ID_SERIAL_CONTROL)) {
        return;
    }

    // Check how much data is available
    uint32_t available = serial_available(device);

    uint8_t buffer[70]; // Max SERIAL_CONTROL payload
    uint8_t count = 0;

    if (available > 0) {
        // Read up to 70 bytes
        uint32_t toRead = (available > 70) ? 70 : available;
        int32_t bytesRead = serial_readPort(device, buffer, toRead);

        if (bytesRead > 0) {
            count = bytesRead;
        }
    }

    // Send response (even if no data - confirms port is open)
    mavlink_message_t msg;
    mavlink_msg_serial_control_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        device,
        flags | SERIAL_CONTROL_FLAG_REPLY,
        s_serialControl.timeout,
        s_serialControl.baudrate,
        count,
        buffer
    );

    sendMessage(&msg);
}

/**
 * @brief Update serial control session
 *
 * Handles timeouts and automatic data forwarding.
 */
void GCSChannel::updateSerialControl()
{
    if (!s_serialControl.active) {
        return;
    }

    uint32_t nowMS = millis();

    // Check for timeout
    if (nowMS - s_serialControl.lastActivityMS > SERIAL_CONTROL_TIMEOUT_MS) {
        // Close port due to inactivity
        serial_closePort(s_serialControl.device);
        s_serialControl.active = false;

        sendText(MAV_SEVERITY_INFO, "Serial control session timeout");
        return;
    }

    // Check for available data to forward
    uint32_t available = serial_available(s_serialControl.device);

    if (available > 0) {
        // Send unsolicited data (for console/shell mode)
        sendSerialControlResponse(s_serialControl.device,
                                  SERIAL_CONTROL_FLAG_RESPOND);
    }
}

/**
 * @brief Close serial control session
 */
void GCSChannel::closeSerialControl()
{
    if (s_serialControl.active) {
        serial_closePort(s_serialControl.device);
        s_serialControl.active = false;
        sendText(MAV_SEVERITY_INFO, "Serial control session closed");
    }
}

/**
 * @brief Send serial port status
 */
void GCSChannel::sendSerialStatus()
{
    if (s_serialControl.active) {
        char buf[100];
        snprintf(buf, sizeof(buf),
                 "Serial control: Port %u @ %u baud",
                 s_serialControl.device,
                 s_serialControl.baudrate);
        sendText(MAV_SEVERITY_INFO, buf);

        uint32_t available = serial_available(s_serialControl.device);
        snprintf(buf, sizeof(buf), "  Available: %u bytes", available);
        sendText(MAV_SEVERITY_INFO, buf);
    } else {
        sendText(MAV_SEVERITY_INFO, "Serial control: inactive");
    }
}

/**
 * @brief Handle serial port passthrough for GPS
 *
 * Convenience function for GPS configuration.
 */
void GCSChannel::handleGPSPassthrough(bool enable)
{
    const uint8_t GPS_DEVICE = 0; // Typically GPS is on serial port 0
    const uint32_t GPS_BAUDRATE = 115200;

    if (enable) {
        if (serial_openPort(GPS_DEVICE, GPS_BAUDRATE)) {
            s_serialControl.active = true;
            s_serialControl.device = GPS_DEVICE;
            s_serialControl.baudrate = GPS_BAUDRATE;
            s_serialControl.lastActivityMS = millis();

            sendText(MAV_SEVERITY_INFO, "GPS passthrough enabled");
        } else {
            sendText(MAV_SEVERITY_ERROR, "Failed to open GPS port");
        }
    } else {
        if (s_serialControl.active && s_serialControl.device == GPS_DEVICE) {
            closeSerialControl();
        }
    }
}

#else // EDUCOPTER_SERIAL_CONTROL_ENABLED

// Serial control disabled - provide stub implementations
void GCSChannel::handleSerialControl(const mavlink_message_t& msg)
{
    sendText(MAV_SEVERITY_WARNING, "Serial control not supported");
}

void GCSChannel::sendSerialControlResponse(uint8_t device, uint8_t flags)
{
    // No-op
}

void GCSChannel::updateSerialControl()
{
    // No-op
}

void GCSChannel::closeSerialControl()
{
    // No-op
}

void GCSChannel::sendSerialStatus()
{
    sendText(MAV_SEVERITY_INFO, "Serial control not supported in this build");
}

void GCSChannel::handleGPSPassthrough(bool enable)
{
    sendText(MAV_SEVERITY_WARNING, "GPS passthrough not supported");
}

#endif // EDUCOPTER_SERIAL_CONTROL_ENABLED

} // namespace GCS
} // namespace EduCopter
