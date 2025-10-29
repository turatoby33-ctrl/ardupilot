/**
 * @file GCS_DeviceOp.cpp
 * @brief Device operation handlers for MAVLink
 *
 * This file implements DEVICE_OP_READ and DEVICE_OP_WRITE message handling
 * for accessing device registers and configuration via MAVLink.
 *
 * Supports operations on:
 * - I2C devices
 * - SPI devices
 * - Flash memory
 * - EEPROM
 * - Configuration registers
 *
 * Useful for:
 * - Sensor calibration
 * - Device diagnostics
 * - Low-level debugging
 * - Firmware configuration
 *
 * @author EduCopter Development Team
 * @date 2025
 */

#include "GCS.h"
#include "GCS_config.h"
#include <cstring>

namespace EduCopter {
namespace GCS {

#if EDUCOPTER_DEVICE_OP_ENABLED

// External device operation interface (implemented by vehicle)
extern bool device_readI2C(uint8_t busNum, uint8_t address, uint8_t regStart,
                           uint8_t* outData, uint8_t length);
extern bool device_writeI2C(uint8_t busNum, uint8_t address, uint8_t regStart,
                            const uint8_t* data, uint8_t length);
extern bool device_readFlash(uint32_t address, uint8_t* outData, uint32_t length);
extern bool device_writeFlash(uint32_t address, const uint8_t* data, uint32_t length);

/**
 * @brief Handle DEVICE_OP_READ message
 *
 * Reads from a device (I2C, flash, etc.).
 */
void GCSChannel::handleDeviceOpRead(const mavlink_message_t& msg)
{
    mavlink_device_op_read_t packet;
    mavlink_msg_device_op_read_decode(&msg, &packet);

    // Check if request is for us
    if (packet.target_system != m_mavlink.getSystemID()) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    uint8_t resultData[128];
    uint8_t resultLength = 0;
    uint8_t result = 0; // 0 = success

    // Dispatch based on bus type
    switch (packet.bustype) {
        case DEVICE_OP_BUSTYPE_I2C: {
            // Read from I2C device
            uint8_t busNum = packet.bus;
            uint8_t address = packet.address;
            uint8_t regStart = packet.regstart;
            uint8_t count = packet.count;

            if (count > 128) {
                count = 128;
            }

            if (device_readI2C(busNum, address, regStart, resultData, count)) {
                resultLength = count;
                result = 0; // Success
            } else {
                result = 1; // Failure
            }
            break;
        }

        case DEVICE_OP_BUSTYPE_SPI:
            // SPI not implemented in this example
            result = 2; // Not supported
            break;

        default:
            result = 2; // Not supported
            break;
    }

    // Send response
    sendDeviceOpReadReply(packet.request_id, result, resultData, resultLength);
}

/**
 * @brief Handle DEVICE_OP_WRITE message
 *
 * Writes to a device (I2C, flash, etc.).
 */
void GCSChannel::handleDeviceOpWrite(const mavlink_message_t& msg)
{
    mavlink_device_op_write_t packet;
    mavlink_msg_device_op_write_decode(&msg, &packet);

    // Check if request is for us
    if (packet.target_system != m_mavlink.getSystemID()) {
        return;
    }

    if (packet.target_component != m_mavlink.getComponentID() &&
        packet.target_component != MAV_COMP_ID_ALL) {
        return;
    }

    uint8_t result = 0; // 0 = success

    // Dispatch based on bus type
    switch (packet.bustype) {
        case DEVICE_OP_BUSTYPE_I2C: {
            // Write to I2C device
            uint8_t busNum = packet.bus;
            uint8_t address = packet.address;
            uint8_t regStart = packet.regstart;
            uint8_t count = packet.count;

            if (count > 128) {
                count = 128;
            }

            if (device_writeI2C(busNum, address, regStart, packet.data, count)) {
                result = 0; // Success
            } else {
                result = 1; // Failure
            }
            break;
        }

        case DEVICE_OP_BUSTYPE_SPI:
            // SPI not implemented in this example
            result = 2; // Not supported
            break;

        default:
            result = 2; // Not supported
            break;
    }

    // Send response
    sendDeviceOpWriteReply(packet.request_id, result);
}

/**
 * @brief Send DEVICE_OP_READ_REPLY message
 */
void GCSChannel::sendDeviceOpReadReply(uint32_t requestID, uint8_t result,
                                       const uint8_t* data, uint8_t length)
{
    if (!hasPayloadSpace(MAVLINK_MSG_ID_DEVICE_OP_READ_REPLY)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_device_op_read_reply_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        requestID,
        result,
        0, // regstart
        length,
        data,
        0  // bank (not used)
    );

    sendMessage(&msg);
}

/**
 * @brief Send DEVICE_OP_WRITE_REPLY message
 */
void GCSChannel::sendDeviceOpWriteReply(uint32_t requestID, uint8_t result)
{
    if (!hasPayloadSpace(MAVLINK_MSG_ID_DEVICE_OP_WRITE_REPLY)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_device_op_write_reply_pack(
        m_mavlink.getSystemID(),
        m_mavlink.getComponentID(),
        &msg,
        requestID,
        result
    );

    sendMessage(&msg);
}

/**
 * @brief Read sensor register (convenience function)
 */
bool GCSChannel::readSensorRegister(uint8_t sensorType, uint8_t regAddr,
                                    uint8_t& outValue)
{
    // Map sensor type to I2C bus/address
    uint8_t busNum = 0;
    uint8_t i2cAddr = 0;

    switch (sensorType) {
        case 0: // IMU
            busNum = 0;
            i2cAddr = 0x68; // Example: MPU6000 address
            break;

        case 1: // Barometer
            busNum = 0;
            i2cAddr = 0x76; // Example: BMP280 address
            break;

        case 2: // Magnetometer
            busNum = 0;
            i2cAddr = 0x1E; // Example: HMC5883L address
            break;

        default:
            return false;
    }

    return device_readI2C(busNum, i2cAddr, regAddr, &outValue, 1);
}

/**
 * @brief Write sensor register (convenience function)
 */
bool GCSChannel::writeSensorRegister(uint8_t sensorType, uint8_t regAddr,
                                     uint8_t value)
{
    // Map sensor type to I2C bus/address
    uint8_t busNum = 0;
    uint8_t i2cAddr = 0;

    switch (sensorType) {
        case 0: // IMU
            busNum = 0;
            i2cAddr = 0x68;
            break;

        case 1: // Barometer
            busNum = 0;
            i2cAddr = 0x76;
            break;

        case 2: // Magnetometer
            busNum = 0;
            i2cAddr = 0x1E;
            break;

        default:
            return false;
    }

    return device_writeI2C(busNum, i2cAddr, regAddr, &value, 1);
}

/**
 * @brief Dump sensor registers (for debugging)
 */
void GCSChannel::dumpSensorRegisters(uint8_t sensorType)
{
    char buf[100];
    snprintf(buf, sizeof(buf), "Sensor %u register dump:", sensorType);
    sendText(MAV_SEVERITY_INFO, buf);

    // Read first 16 registers
    for (uint8_t reg = 0; reg < 16; reg++) {
        uint8_t value;
        if (readSensorRegister(sensorType, reg, value)) {
            snprintf(buf, sizeof(buf), "  Reg 0x%02X: 0x%02X", reg, value);
            sendText(MAV_SEVERITY_INFO, buf);
        }
    }
}

#else // EDUCOPTER_DEVICE_OP_ENABLED

// Device operations disabled - provide stub implementations
void GCSChannel::handleDeviceOpRead(const mavlink_message_t& msg)
{
    sendText(MAV_SEVERITY_WARNING, "Device operations not supported");
}

void GCSChannel::handleDeviceOpWrite(const mavlink_message_t& msg)
{
    sendText(MAV_SEVERITY_WARNING, "Device operations not supported");
}

void GCSChannel::sendDeviceOpReadReply(uint32_t requestID, uint8_t result,
                                       const uint8_t* data, uint8_t length)
{
    // No-op
}

void GCSChannel::sendDeviceOpWriteReply(uint32_t requestID, uint8_t result)
{
    // No-op
}

bool GCSChannel::readSensorRegister(uint8_t sensorType, uint8_t regAddr,
                                    uint8_t& outValue)
{
    return false;
}

bool GCSChannel::writeSensorRegister(uint8_t sensorType, uint8_t regAddr,
                                     uint8_t value)
{
    return false;
}

void GCSChannel::dumpSensorRegisters(uint8_t sensorType)
{
    sendText(MAV_SEVERITY_INFO, "Device operations not supported in this build");
}

#endif // EDUCOPTER_DEVICE_OP_ENABLED

} // namespace GCS
} // namespace EduCopter
