/**
 * @file test_device_op.cpp
 * @brief Test suite for GCS_DeviceOp - Device operations (I2C, SPI, Flash)
 *
 * Tests device operation message handling including:
 * - DEVICE_OP_READ message handling
 * - DEVICE_OP_WRITE message handling
 * - I2C device read/write operations
 * - SPI device operations
 * - Flash memory operations
 * - Sensor register access
 * - Error handling
 *
 * @author EduCopter Test Suite
 * @date 2025
 */

#include <cstdio>
#include <cstring>
#include <cstdint>
#include "mavlink_stubs.h"

// Test result codes
#define TEST_PASS() return true
#define TEST_FAIL(msg) do { printf("  FAIL: %s\n", msg); return false; } while(0)
#define TEST_ASSERT(cond, msg) if (!(cond)) TEST_FAIL(msg)

//=============================================================================
// Mock Device Storage
//=============================================================================

// Simulated I2C device memory (multiple devices)
struct I2CDevice {
    uint8_t busNum;
    uint8_t address;
    uint8_t registers[256];
    bool exists;
};

#define MAX_I2C_DEVICES 8
static I2CDevice g_i2cDevices[MAX_I2C_DEVICES];

// Simulated Flash memory
#define FLASH_SIZE 4096
static uint8_t g_flashMemory[FLASH_SIZE];

void initDeviceStorage() {
    memset(g_i2cDevices, 0, sizeof(g_i2cDevices));
    memset(g_flashMemory, 0xFF, sizeof(g_flashMemory)); // Flash starts erased (0xFF)

    // Create some default I2C devices
    // Device 0: IMU at 0x68
    g_i2cDevices[0].busNum = 0;
    g_i2cDevices[0].address = 0x68;
    g_i2cDevices[0].exists = true;
    g_i2cDevices[0].registers[0x75] = 0x68; // WHO_AM_I register

    // Device 1: Barometer at 0x76
    g_i2cDevices[1].busNum = 0;
    g_i2cDevices[1].address = 0x76;
    g_i2cDevices[1].exists = true;
    g_i2cDevices[1].registers[0xD0] = 0x58; // Chip ID

    // Device 2: Magnetometer at 0x1E
    g_i2cDevices[2].busNum = 0;
    g_i2cDevices[2].address = 0x1E;
    g_i2cDevices[2].exists = true;
    g_i2cDevices[2].registers[0x0A] = 0x48; // ID register
}

I2CDevice* findI2CDevice(uint8_t busNum, uint8_t address) {
    for (int i = 0; i < MAX_I2C_DEVICES; i++) {
        if (g_i2cDevices[i].exists &&
            g_i2cDevices[i].busNum == busNum &&
            g_i2cDevices[i].address == address) {
            return &g_i2cDevices[i];
        }
    }
    return nullptr;
}

bool device_readI2C(uint8_t busNum, uint8_t address, uint8_t regStart,
                    uint8_t* outData, uint8_t length) {
    I2CDevice* dev = findI2CDevice(busNum, address);
    if (!dev) return false;

    for (uint8_t i = 0; i < length; i++) {
        outData[i] = dev->registers[(regStart + i) & 0xFF];
    }
    return true;
}

bool device_writeI2C(uint8_t busNum, uint8_t address, uint8_t regStart,
                     const uint8_t* data, uint8_t length) {
    I2CDevice* dev = findI2CDevice(busNum, address);
    if (!dev) return false;

    for (uint8_t i = 0; i < length; i++) {
        dev->registers[(regStart + i) & 0xFF] = data[i];
    }
    return true;
}

bool device_readFlash(uint32_t address, uint8_t* outData, uint32_t length) {
    if (address + length > FLASH_SIZE) return false;

    memcpy(outData, &g_flashMemory[address], length);
    return true;
}

bool device_writeFlash(uint32_t address, const uint8_t* data, uint32_t length) {
    if (address + length > FLASH_SIZE) return false;

    memcpy(&g_flashMemory[address], data, length);
    return true;
}

//=============================================================================
// Device Operation Request/Reply Functions
//=============================================================================

struct DeviceOpReadRequest {
    uint32_t requestID;
    uint8_t targetSystem;
    uint8_t targetComponent;
    uint8_t busType;
    uint8_t bus;
    uint8_t address;
    uint8_t regStart;
    uint8_t count;
};

struct DeviceOpReadReply {
    uint32_t requestID;
    uint8_t result;
    uint8_t regStart;
    uint8_t count;
    uint8_t data[128];
};

struct DeviceOpWriteRequest {
    uint32_t requestID;
    uint8_t targetSystem;
    uint8_t targetComponent;
    uint8_t busType;
    uint8_t bus;
    uint8_t address;
    uint8_t regStart;
    uint8_t count;
    uint8_t data[128];
};

struct DeviceOpWriteReply {
    uint32_t requestID;
    uint8_t result;
};

bool processDeviceOpRead(const DeviceOpReadRequest& req, DeviceOpReadReply& reply) {
    reply.requestID = req.requestID;
    reply.result = 0; // Success
    reply.regStart = req.regStart;
    reply.count = 0;

    switch (req.busType) {
        case DEVICE_OP_BUSTYPE_I2C: {
            uint8_t count = req.count;
            if (count > 128) count = 128;

            if (device_readI2C(req.bus, req.address, req.regStart, reply.data, count)) {
                reply.count = count;
                reply.result = 0; // Success
            } else {
                reply.result = 1; // Failure
            }
            break;
        }

        case DEVICE_OP_BUSTYPE_SPI:
            reply.result = 2; // Not supported
            break;

        default:
            reply.result = 2; // Not supported
            break;
    }

    return true;
}

bool processDeviceOpWrite(const DeviceOpWriteRequest& req, DeviceOpWriteReply& reply) {
    reply.requestID = req.requestID;
    reply.result = 0; // Success

    switch (req.busType) {
        case DEVICE_OP_BUSTYPE_I2C: {
            uint8_t count = req.count;
            if (count > 128) count = 128;

            if (device_writeI2C(req.bus, req.address, req.regStart, req.data, count)) {
                reply.result = 0; // Success
            } else {
                reply.result = 1; // Failure
            }
            break;
        }

        case DEVICE_OP_BUSTYPE_SPI:
            reply.result = 2; // Not supported
            break;

        default:
            reply.result = 2; // Not supported
            break;
    }

    return true;
}

//=============================================================================
// Sensor Register Access Functions
//=============================================================================

bool readSensorRegister(uint8_t sensorType, uint8_t regAddr, uint8_t& outValue) {
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

    return device_readI2C(busNum, i2cAddr, regAddr, &outValue, 1);
}

bool writeSensorRegister(uint8_t sensorType, uint8_t regAddr, uint8_t value) {
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

//=============================================================================
// Tests
//=============================================================================

bool test_i2c_read_single_register() {
    initDeviceStorage();

    DeviceOpReadRequest req;
    req.requestID = 1;
    req.targetSystem = 1;
    req.targetComponent = 1;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0x68; // IMU
    req.regStart = 0x75; // WHO_AM_I
    req.count = 1;

    DeviceOpReadReply reply;
    processDeviceOpRead(req, reply);

    TEST_ASSERT(reply.result == 0, "Read should succeed");
    TEST_ASSERT(reply.count == 1, "Should read 1 byte");
    TEST_ASSERT(reply.data[0] == 0x68, "WHO_AM_I should be 0x68");
    TEST_PASS();
}

bool test_i2c_read_multiple_registers() {
    initDeviceStorage();

    // Write some test data
    uint8_t testData[4] = {0x11, 0x22, 0x33, 0x44};
    device_writeI2C(0, 0x68, 0x10, testData, 4);

    // Read it back
    DeviceOpReadRequest req;
    req.requestID = 2;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0x68;
    req.regStart = 0x10;
    req.count = 4;

    DeviceOpReadReply reply;
    processDeviceOpRead(req, reply);

    TEST_ASSERT(reply.result == 0, "Read should succeed");
    TEST_ASSERT(reply.count == 4, "Should read 4 bytes");
    TEST_ASSERT(reply.data[0] == 0x11, "Byte 0 correct");
    TEST_ASSERT(reply.data[1] == 0x22, "Byte 1 correct");
    TEST_ASSERT(reply.data[2] == 0x33, "Byte 2 correct");
    TEST_ASSERT(reply.data[3] == 0x44, "Byte 3 correct");
    TEST_PASS();
}

bool test_i2c_write_single_register() {
    initDeviceStorage();

    DeviceOpWriteRequest req;
    req.requestID = 3;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0x68;
    req.regStart = 0x20;
    req.count = 1;
    req.data[0] = 0xAB;

    DeviceOpWriteReply reply;
    processDeviceOpWrite(req, reply);

    TEST_ASSERT(reply.result == 0, "Write should succeed");

    // Verify the write
    uint8_t readBack;
    device_readI2C(0, 0x68, 0x20, &readBack, 1);
    TEST_ASSERT(readBack == 0xAB, "Written value should be 0xAB");
    TEST_PASS();
}

bool test_i2c_write_multiple_registers() {
    initDeviceStorage();

    DeviceOpWriteRequest req;
    req.requestID = 4;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0x68;
    req.regStart = 0x30;
    req.count = 5;
    req.data[0] = 0xAA;
    req.data[1] = 0xBB;
    req.data[2] = 0xCC;
    req.data[3] = 0xDD;
    req.data[4] = 0xEE;

    DeviceOpWriteReply reply;
    processDeviceOpWrite(req, reply);

    TEST_ASSERT(reply.result == 0, "Write should succeed");

    // Verify all bytes written
    uint8_t readBack[5];
    device_readI2C(0, 0x68, 0x30, readBack, 5);
    TEST_ASSERT(readBack[0] == 0xAA, "Byte 0 correct");
    TEST_ASSERT(readBack[1] == 0xBB, "Byte 1 correct");
    TEST_ASSERT(readBack[2] == 0xCC, "Byte 2 correct");
    TEST_ASSERT(readBack[3] == 0xDD, "Byte 3 correct");
    TEST_ASSERT(readBack[4] == 0xEE, "Byte 4 correct");
    TEST_PASS();
}

bool test_i2c_read_nonexistent_device() {
    initDeviceStorage();

    DeviceOpReadRequest req;
    req.requestID = 5;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0xFF; // Non-existent device
    req.regStart = 0x00;
    req.count = 1;

    DeviceOpReadReply reply;
    processDeviceOpRead(req, reply);

    TEST_ASSERT(reply.result == 1, "Read should fail for non-existent device");
    TEST_PASS();
}

bool test_i2c_write_nonexistent_device() {
    initDeviceStorage();

    DeviceOpWriteRequest req;
    req.requestID = 6;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0xFF; // Non-existent device
    req.regStart = 0x00;
    req.count = 1;
    req.data[0] = 0x12;

    DeviceOpWriteReply reply;
    processDeviceOpWrite(req, reply);

    TEST_ASSERT(reply.result == 1, "Write should fail for non-existent device");
    TEST_PASS();
}

bool test_multiple_i2c_devices() {
    initDeviceStorage();

    // Read WHO_AM_I from IMU (0x68)
    uint8_t imuID;
    device_readI2C(0, 0x68, 0x75, &imuID, 1);
    TEST_ASSERT(imuID == 0x68, "IMU WHO_AM_I correct");

    // Read chip ID from Barometer (0x76)
    uint8_t baroID;
    device_readI2C(0, 0x76, 0xD0, &baroID, 1);
    TEST_ASSERT(baroID == 0x58, "Barometer chip ID correct");

    // Read ID from Magnetometer (0x1E)
    uint8_t magID;
    device_readI2C(0, 0x1E, 0x0A, &magID, 1);
    TEST_ASSERT(magID == 0x48, "Magnetometer ID correct");

    TEST_PASS();
}

bool test_sensor_register_read() {
    initDeviceStorage();

    // Read IMU WHO_AM_I
    uint8_t value;
    bool success = readSensorRegister(0, 0x75, value);
    TEST_ASSERT(success, "IMU read should succeed");
    TEST_ASSERT(value == 0x68, "IMU WHO_AM_I should be 0x68");

    // Read Barometer chip ID
    success = readSensorRegister(1, 0xD0, value);
    TEST_ASSERT(success, "Barometer read should succeed");
    TEST_ASSERT(value == 0x58, "Barometer chip ID should be 0x58");

    // Read Magnetometer ID
    success = readSensorRegister(2, 0x0A, value);
    TEST_ASSERT(success, "Magnetometer read should succeed");
    TEST_ASSERT(value == 0x48, "Magnetometer ID should be 0x48");

    TEST_PASS();
}

bool test_sensor_register_write() {
    initDeviceStorage();

    // Write to IMU register
    bool success = writeSensorRegister(0, 0x6B, 0x80);
    TEST_ASSERT(success, "IMU write should succeed");

    // Verify write
    uint8_t value;
    readSensorRegister(0, 0x6B, value);
    TEST_ASSERT(value == 0x80, "Written value should be 0x80");

    TEST_PASS();
}

bool test_sensor_invalid_type() {
    initDeviceStorage();

    uint8_t value;
    bool success = readSensorRegister(99, 0x00, value);
    TEST_ASSERT(!success, "Should fail for invalid sensor type");

    success = writeSensorRegister(99, 0x00, 0x12);
    TEST_ASSERT(!success, "Should fail for invalid sensor type");

    TEST_PASS();
}

bool test_spi_not_supported() {
    initDeviceStorage();

    DeviceOpReadRequest req;
    req.requestID = 7;
    req.busType = DEVICE_OP_BUSTYPE_SPI;
    req.bus = 0;
    req.address = 0;
    req.regStart = 0;
    req.count = 1;

    DeviceOpReadReply reply;
    processDeviceOpRead(req, reply);

    TEST_ASSERT(reply.result == 2, "SPI should return 'not supported'");
    TEST_PASS();
}

bool test_flash_read_write() {
    initDeviceStorage();

    // Write to flash
    uint8_t writeData[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    bool success = device_writeFlash(0x100, writeData, 5);
    TEST_ASSERT(success, "Flash write should succeed");

    // Read from flash
    uint8_t readData[5];
    success = device_readFlash(0x100, readData, 5);
    TEST_ASSERT(success, "Flash read should succeed");

    // Verify data
    for (int i = 0; i < 5; i++) {
        TEST_ASSERT(readData[i] == writeData[i], "Flash data should match");
    }

    TEST_PASS();
}

bool test_flash_out_of_bounds() {
    initDeviceStorage();

    uint8_t data[10];

    // Try to read beyond flash size
    bool success = device_readFlash(FLASH_SIZE - 5, data, 10);
    TEST_ASSERT(!success, "Should fail reading beyond flash size");

    // Try to write beyond flash size
    success = device_writeFlash(FLASH_SIZE - 5, data, 10);
    TEST_ASSERT(!success, "Should fail writing beyond flash size");

    TEST_PASS();
}

bool test_max_data_length() {
    initDeviceStorage();

    // Create request with 128 bytes (max)
    DeviceOpReadRequest req;
    req.requestID = 8;
    req.busType = DEVICE_OP_BUSTYPE_I2C;
    req.bus = 0;
    req.address = 0x68;
    req.regStart = 0;
    req.count = 128;

    DeviceOpReadReply reply;
    processDeviceOpRead(req, reply);

    TEST_ASSERT(reply.result == 0, "Read should succeed");
    TEST_ASSERT(reply.count == 128, "Should read 128 bytes");

    TEST_PASS();
}

bool test_request_id_preservation() {
    initDeviceStorage();

    // Test that request IDs are preserved in replies
    DeviceOpReadRequest req1;
    req1.requestID = 12345;
    req1.busType = DEVICE_OP_BUSTYPE_I2C;
    req1.bus = 0;
    req1.address = 0x68;
    req1.regStart = 0;
    req1.count = 1;

    DeviceOpReadReply reply1;
    processDeviceOpRead(req1, reply1);
    TEST_ASSERT(reply1.requestID == 12345, "Request ID should be preserved");

    DeviceOpWriteRequest req2;
    req2.requestID = 67890;
    req2.busType = DEVICE_OP_BUSTYPE_I2C;
    req2.bus = 0;
    req2.address = 0x68;
    req2.regStart = 0;
    req2.count = 1;
    req2.data[0] = 0x12;

    DeviceOpWriteReply reply2;
    processDeviceOpWrite(req2, reply2);
    TEST_ASSERT(reply2.requestID == 67890, "Request ID should be preserved");

    TEST_PASS();
}

//=============================================================================
// Test Runner
//=============================================================================

struct Test {
    const char* name;
    bool (*func)();
};

int main() {
    printf("\n=== MAVLink Device Operations Test Suite ===\n\n");

    Test tests[] = {
        {"I2C Read Single Register", test_i2c_read_single_register},
        {"I2C Read Multiple Registers", test_i2c_read_multiple_registers},
        {"I2C Write Single Register", test_i2c_write_single_register},
        {"I2C Write Multiple Registers", test_i2c_write_multiple_registers},
        {"I2C Read Non-existent Device", test_i2c_read_nonexistent_device},
        {"I2C Write Non-existent Device", test_i2c_write_nonexistent_device},
        {"Multiple I2C Devices", test_multiple_i2c_devices},
        {"Sensor Register Read", test_sensor_register_read},
        {"Sensor Register Write", test_sensor_register_write},
        {"Sensor Invalid Type", test_sensor_invalid_type},
        {"SPI Not Supported", test_spi_not_supported},
        {"Flash Read/Write", test_flash_read_write},
        {"Flash Out of Bounds", test_flash_out_of_bounds},
        {"Max Data Length (128 bytes)", test_max_data_length},
        {"Request ID Preservation", test_request_id_preservation}
    };

    int numTests = sizeof(tests) / sizeof(tests[0]);
    int passed = 0;
    int failed = 0;

    for (int i = 0; i < numTests; i++) {
        printf("Running: %s\n", tests[i].name);
        if (tests[i].func()) {
            printf("  PASS\n");
            passed++;
        } else {
            failed++;
        }
    }

    printf("\n=== Test Summary ===\n");
    printf("Total:  %d\n", numTests);
    printf("Passed: %d\n", passed);
    printf("Failed: %d\n", failed);
    printf("Success Rate: %.1f%%\n", (100.0 * passed) / numTests);

    if (failed == 0) {
        printf("\n✓ All tests passed!\n\n");
        return 0;
    } else {
        printf("\n✗ Some tests failed!\n\n");
        return 1;
    }
}
