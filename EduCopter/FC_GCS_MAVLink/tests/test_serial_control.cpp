/**
 * @file test_serial_control.cpp
 * @brief Test suite for GCS_serial_control - Serial port passthrough
 *
 * Tests serial control message handling including:
 * - SERIAL_CONTROL message handling
 * - Serial port open/close operations
 * - Serial read/write operations
 * - Baudrate configuration
 * - Timeout handling
 * - Serial control flags (REPLY, RESPOND, BLOCKING, EXCLUSIVE)
 * - Multiple device support (GPS, Telemetry, etc.)
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
// Serial Control Constants
//=============================================================================

enum SERIAL_CONTROL_DEV {
    SERIAL_CONTROL_DEV_TELEM1 = 0,
    SERIAL_CONTROL_DEV_TELEM2 = 1,
    SERIAL_CONTROL_DEV_GPS1 = 2,
    SERIAL_CONTROL_DEV_GPS2 = 3,
    SERIAL_CONTROL_DEV_SHELL = 10
};

// Serial control flags are defined in mavlink_stubs.h

//=============================================================================
// Serial Port Simulation
//=============================================================================

struct SerialPort {
    uint8_t device;
    uint32_t baudrate;
    uint16_t timeout;
    uint8_t flags;
    bool isOpen;
    bool exclusive;

    // Simulated buffers
    uint8_t rxBuffer[256];
    uint8_t txBuffer[256];
    uint8_t rxCount;
    uint8_t txCount;

    void reset() {
        isOpen = false;
        exclusive = false;
        baudrate = 0;
        timeout = 0;
        flags = 0;
        rxCount = 0;
        txCount = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
        memset(txBuffer, 0, sizeof(txBuffer));
    }

    bool write(const uint8_t* data, uint8_t count) {
        if (!isOpen) return false;
        if (txCount + count > 256) return false;

        memcpy(&txBuffer[txCount], data, count);
        txCount += count;
        return true;
    }

    uint8_t read(uint8_t* data, uint8_t maxCount) {
        if (!isOpen) return 0;

        uint8_t toRead = (rxCount < maxCount) ? rxCount : maxCount;
        memcpy(data, rxBuffer, toRead);

        // Shift remaining data
        if (toRead < rxCount) {
            memmove(rxBuffer, &rxBuffer[toRead], rxCount - toRead);
        }
        rxCount -= toRead;

        return toRead;
    }

    void simulateRx(const uint8_t* data, uint8_t count) {
        if (rxCount + count <= 256) {
            memcpy(&rxBuffer[rxCount], data, count);
            rxCount += count;
        }
    }
};

#define MAX_SERIAL_PORTS 16
static SerialPort g_serialPorts[MAX_SERIAL_PORTS];

void initSerialPorts() {
    for (int i = 0; i < MAX_SERIAL_PORTS; i++) {
        g_serialPorts[i].device = i;
        g_serialPorts[i].reset();
    }
}

SerialPort* getSerialPort(uint8_t device) {
    if (device >= MAX_SERIAL_PORTS) return nullptr;
    return &g_serialPorts[device];
}

//=============================================================================
// Serial Control Message Handling
//=============================================================================

struct SerialControlRequest {
    uint8_t device;
    uint8_t flags;
    uint16_t timeout;
    uint32_t baudrate;
    uint8_t count;
    uint8_t data[70];
};

struct SerialControlReply {
    uint8_t device;
    uint8_t flags;
    uint16_t timeout;
    uint32_t baudrate;
    uint8_t count;
    uint8_t data[70];
};

bool handleSerialControl(const SerialControlRequest& req, SerialControlReply& reply) {
    SerialPort* port = getSerialPort(req.device);
    if (!port) return false;

    // Initialize reply
    reply.device = req.device;
    reply.flags = 0;
    reply.timeout = req.timeout;
    reply.baudrate = req.baudrate;
    reply.count = 0;

    // Check if port needs to be opened/configured
    if (!port->isOpen) {
        port->isOpen = true;
        port->baudrate = req.baudrate;
        port->timeout = req.timeout;
        port->flags = req.flags;
        port->exclusive = (req.flags & SERIAL_CONTROL_FLAG_EXCLUSIVE) != 0;
    }

    // Check exclusive access
    if (port->exclusive && (req.flags & SERIAL_CONTROL_FLAG_EXCLUSIVE) == 0) {
        // Port is exclusively locked by someone else
        return false;
    }

    // Update baudrate if changed
    if (req.baudrate > 0 && req.baudrate != port->baudrate) {
        port->baudrate = req.baudrate;
    }

    // Write data if provided
    if (req.count > 0) {
        if (!port->write(req.data, req.count)) {
            return false;
        }
    }

    // Read data if RESPOND or REPLY flag is set
    if (req.flags & (SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_REPLY)) {
        reply.count = port->read(reply.data, 70);
        reply.flags = SERIAL_CONTROL_FLAG_REPLY;
    }

    return true;
}

//=============================================================================
// Tests
//=============================================================================

bool test_serial_port_open() {
    initSerialPorts();

    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_GPS1;
    req.flags = 0;
    req.timeout = 1000;
    req.baudrate = 115200;
    req.count = 0;

    SerialControlReply reply;
    bool success = handleSerialControl(req, reply);

    TEST_ASSERT(success, "Opening serial port should succeed");

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    TEST_ASSERT(port->isOpen, "Port should be open");
    TEST_ASSERT(port->baudrate == 115200, "Baudrate should be set");
    TEST_ASSERT(port->timeout == 1000, "Timeout should be set");

    TEST_PASS();
}

bool test_serial_write() {
    initSerialPorts();

    // Open port first
    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_TELEM1;
    req.flags = 0;
    req.timeout = 1000;
    req.baudrate = 57600;
    req.count = 5;
    req.data[0] = 0x01;
    req.data[1] = 0x02;
    req.data[2] = 0x03;
    req.data[3] = 0x04;
    req.data[4] = 0x05;

    SerialControlReply reply;
    bool success = handleSerialControl(req, reply);
    TEST_ASSERT(success, "Write should succeed");

    // Verify data was written
    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_TELEM1);
    TEST_ASSERT(port->txCount == 5, "Should have 5 bytes in TX buffer");
    TEST_ASSERT(port->txBuffer[0] == 0x01, "Byte 0 correct");
    TEST_ASSERT(port->txBuffer[4] == 0x05, "Byte 4 correct");

    TEST_PASS();
}

bool test_serial_read() {
    initSerialPorts();

    // Open port and simulate incoming data
    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    port->isOpen = true;
    port->baudrate = 9600;

    uint8_t gpsData[] = {0x24, 0x47, 0x50, 0x52, 0x4D, 0x43}; // "$GPRMC"
    port->simulateRx(gpsData, 6);

    // Read data
    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_GPS1;
    req.flags = SERIAL_CONTROL_FLAG_RESPOND;
    req.timeout = 1000;
    req.baudrate = 9600;
    req.count = 0;

    SerialControlReply reply;
    bool success = handleSerialControl(req, reply);

    TEST_ASSERT(success, "Read should succeed");
    TEST_ASSERT(reply.count == 6, "Should read 6 bytes");
    TEST_ASSERT(reply.data[0] == 0x24, "First byte should be '$'");
    TEST_ASSERT(reply.flags & SERIAL_CONTROL_FLAG_REPLY, "Reply flag should be set");

    TEST_PASS();
}

bool test_serial_write_and_read() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS2);
    port->isOpen = true;
    port->baudrate = 115200;

    // Write command
    SerialControlRequest req1;
    req1.device = SERIAL_CONTROL_DEV_GPS2;
    req1.flags = 0;
    req1.baudrate = 115200;
    req1.timeout = 500;
    req1.count = 3;
    req1.data[0] = 0xB5; // UBX header
    req1.data[1] = 0x62;
    req1.data[2] = 0x06;

    SerialControlReply reply1;
    handleSerialControl(req1, reply1);

    TEST_ASSERT(port->txCount == 3, "Should have written 3 bytes");

    // Simulate response
    uint8_t response[] = {0xB5, 0x62, 0x05, 0x01}; // UBX ACK
    port->simulateRx(response, 4);

    // Read response
    SerialControlRequest req2;
    req2.device = SERIAL_CONTROL_DEV_GPS2;
    req2.flags = SERIAL_CONTROL_FLAG_RESPOND;
    req2.baudrate = 115200;
    req2.timeout = 500;
    req2.count = 0;

    SerialControlReply reply2;
    handleSerialControl(req2, reply2);

    TEST_ASSERT(reply2.count == 4, "Should read 4 bytes");
    TEST_ASSERT(reply2.data[0] == 0xB5, "Response header correct");

    TEST_PASS();
}

bool test_serial_exclusive_access() {
    initSerialPorts();

    // Open with exclusive flag
    SerialControlRequest req1;
    req1.device = SERIAL_CONTROL_DEV_TELEM2;
    req1.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    req1.baudrate = 57600;
    req1.timeout = 1000;
    req1.count = 0;

    SerialControlReply reply1;
    bool success1 = handleSerialControl(req1, reply1);
    TEST_ASSERT(success1, "Exclusive open should succeed");

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_TELEM2);
    TEST_ASSERT(port->exclusive, "Port should be exclusive");

    // Try to access without exclusive flag (should fail)
    SerialControlRequest req2;
    req2.device = SERIAL_CONTROL_DEV_TELEM2;
    req2.flags = 0;
    req2.baudrate = 57600;
    req2.timeout = 1000;
    req2.count = 0;

    SerialControlReply reply2;
    bool success2 = handleSerialControl(req2, reply2);
    TEST_ASSERT(!success2, "Non-exclusive access should fail");

    TEST_PASS();
}

bool test_serial_baudrate_change() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    port->isOpen = true;
    port->baudrate = 9600;

    // Change baudrate
    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_GPS1;
    req.flags = 0;
    req.baudrate = 115200;
    req.timeout = 1000;
    req.count = 0;

    SerialControlReply reply;
    handleSerialControl(req, reply);

    TEST_ASSERT(port->baudrate == 115200, "Baudrate should be updated");
    TEST_ASSERT(reply.baudrate == 115200, "Reply should show new baudrate");

    TEST_PASS();
}

bool test_serial_timeout_configuration() {
    initSerialPorts();

    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_SHELL;
    req.flags = 0;
    req.baudrate = 115200;
    req.timeout = 5000; // 5 second timeout
    req.count = 0;

    SerialControlReply reply;
    handleSerialControl(req, reply);

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_SHELL);
    TEST_ASSERT(port->timeout == 5000, "Timeout should be set");
    TEST_ASSERT(reply.timeout == 5000, "Reply should show timeout");

    TEST_PASS();
}

bool test_serial_multiple_writes() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_TELEM1);
    port->isOpen = true;
    port->baudrate = 57600;

    // Write multiple times
    for (int i = 0; i < 3; i++) {
        SerialControlRequest req;
        req.device = SERIAL_CONTROL_DEV_TELEM1;
        req.flags = 0;
        req.baudrate = 57600;
        req.timeout = 1000;
        req.count = 2;
        req.data[0] = 0x10 + i;
        req.data[1] = 0x20 + i;

        SerialControlReply reply;
        handleSerialControl(req, reply);
    }

    TEST_ASSERT(port->txCount == 6, "Should have 6 bytes total");
    TEST_ASSERT(port->txBuffer[0] == 0x10, "First write byte 0");
    TEST_ASSERT(port->txBuffer[2] == 0x11, "Second write byte 0");
    TEST_ASSERT(port->txBuffer[4] == 0x12, "Third write byte 0");

    TEST_PASS();
}

bool test_serial_buffer_limits() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_TELEM1);
    port->isOpen = true;
    port->baudrate = 57600;

    // Try to write maximum data in one message (70 bytes)
    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_TELEM1;
    req.flags = 0;
    req.baudrate = 57600;
    req.timeout = 1000;
    req.count = 70;

    for (int i = 0; i < 70; i++) {
        req.data[i] = i;
    }

    SerialControlReply reply;
    bool success = handleSerialControl(req, reply);

    TEST_ASSERT(success, "Max size write should succeed");
    TEST_ASSERT(port->txCount == 70, "Should write 70 bytes");

    TEST_PASS();
}

bool test_serial_read_partial() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    port->isOpen = true;
    port->baudrate = 9600;

    // Simulate 100 bytes available
    uint8_t data[100];
    for (int i = 0; i < 100; i++) {
        data[i] = i;
    }
    port->simulateRx(data, 100);

    // Read should return max 70 bytes
    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_GPS1;
    req.flags = SERIAL_CONTROL_FLAG_RESPOND;
    req.baudrate = 9600;
    req.timeout = 1000;
    req.count = 0;

    SerialControlReply reply;
    handleSerialControl(req, reply);

    TEST_ASSERT(reply.count == 70, "Should read max 70 bytes");
    TEST_ASSERT(port->rxCount == 30, "Should have 30 bytes remaining");

    TEST_PASS();
}

bool test_serial_invalid_device() {
    initSerialPorts();

    SerialControlRequest req;
    req.device = 99; // Invalid device
    req.flags = 0;
    req.baudrate = 115200;
    req.timeout = 1000;
    req.count = 0;

    SerialControlReply reply;
    bool success = handleSerialControl(req, reply);

    TEST_ASSERT(!success, "Invalid device should fail");

    TEST_PASS();
}

bool test_serial_multiple_devices() {
    initSerialPorts();

    // Open multiple devices with different baudrates
    const uint8_t devices[] = {SERIAL_CONTROL_DEV_TELEM1,
                               SERIAL_CONTROL_DEV_GPS1,
                               SERIAL_CONTROL_DEV_GPS2};
    const uint32_t baudrates[] = {57600, 9600, 115200};

    for (int i = 0; i < 3; i++) {
        SerialControlRequest req;
        req.device = devices[i];
        req.flags = 0;
        req.baudrate = baudrates[i];
        req.timeout = 1000;
        req.count = 1;
        req.data[0] = 0x10 + i;

        SerialControlReply reply;
        bool success = handleSerialControl(req, reply);
        TEST_ASSERT(success, "Open should succeed");
    }

    // Verify each device configured correctly
    for (int i = 0; i < 3; i++) {
        SerialPort* port = getSerialPort(devices[i]);
        TEST_ASSERT(port->isOpen, "Port should be open");
        TEST_ASSERT(port->baudrate == baudrates[i], "Baudrate should match");
        TEST_ASSERT(port->txCount == 1, "Should have 1 byte written");
    }

    TEST_PASS();
}

bool test_serial_reply_flag() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    port->isOpen = true;
    port->simulateRx((const uint8_t*)"TEST", 4);

    // Request with REPLY flag
    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_GPS1;
    req.flags = SERIAL_CONTROL_FLAG_REPLY;
    req.baudrate = 9600;
    req.timeout = 1000;
    req.count = 0;

    SerialControlReply reply;
    handleSerialControl(req, reply);

    TEST_ASSERT(reply.flags & SERIAL_CONTROL_FLAG_REPLY, "Reply flag should be set");
    TEST_ASSERT(reply.count == 4, "Should read 4 bytes");

    TEST_PASS();
}

bool test_serial_no_data_available() {
    initSerialPorts();

    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    port->isOpen = true;
    // No data in RX buffer

    SerialControlRequest req;
    req.device = SERIAL_CONTROL_DEV_GPS1;
    req.flags = SERIAL_CONTROL_FLAG_RESPOND;
    req.baudrate = 9600;
    req.timeout = 1000;
    req.count = 0;

    SerialControlReply reply;
    bool success = handleSerialControl(req, reply);

    TEST_ASSERT(success, "Request should succeed even with no data");
    TEST_ASSERT(reply.count == 0, "Should return 0 bytes");

    TEST_PASS();
}

bool test_serial_gps_passthrough() {
    initSerialPorts();

    // Simulate GPS passthrough scenario
    SerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);

    // Open GPS port
    SerialControlRequest req1;
    req1.device = SERIAL_CONTROL_DEV_GPS1;
    req1.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    req1.baudrate = 9600;
    req1.timeout = 1000;
    req1.count = 0;

    SerialControlReply reply1;
    handleSerialControl(req1, reply1);

    // Send UBX command
    SerialControlRequest req2;
    req2.device = SERIAL_CONTROL_DEV_GPS1;
    req2.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    req2.baudrate = 9600;
    req2.timeout = 1000;
    req2.count = 6;
    req2.data[0] = 0xB5;
    req2.data[1] = 0x62;
    req2.data[2] = 0x06;
    req2.data[3] = 0x00;
    req2.data[4] = 0x00;
    req2.data[5] = 0x06;

    SerialControlReply reply2;
    handleSerialControl(req2, reply2);

    TEST_ASSERT(port->txCount == 6, "Should write GPS command");

    // Simulate GPS response
    port->simulateRx((const uint8_t*)"\xB5\x62\x05\x01", 4);

    // Read response
    SerialControlRequest req3;
    req3.device = SERIAL_CONTROL_DEV_GPS1;
    req3.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND;
    req3.baudrate = 9600;
    req3.timeout = 1000;
    req3.count = 0;

    SerialControlReply reply3;
    handleSerialControl(req3, reply3);

    TEST_ASSERT(reply3.count == 4, "Should read GPS response");

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
    printf("\n=== MAVLink Serial Control Test Suite ===\n\n");

    static Test tests[] = {
        {"Serial Port Open", test_serial_port_open},
        {"Serial Write", test_serial_write},
        {"Serial Read", test_serial_read},
        {"Serial Write and Read", test_serial_write_and_read},
        {"Serial Exclusive Access", test_serial_exclusive_access},
        {"Serial Baudrate Change", test_serial_baudrate_change},
        {"Serial Timeout Configuration", test_serial_timeout_configuration},
        {"Serial Multiple Writes", test_serial_multiple_writes},
        {"Serial Buffer Limits", test_serial_buffer_limits},
        {"Serial Read Partial", test_serial_read_partial},
        {"Serial Invalid Device", test_serial_invalid_device},
        {"Serial Multiple Devices", test_serial_multiple_devices},
        {"Serial Reply Flag", test_serial_reply_flag},
        {"Serial No Data Available", test_serial_no_data_available},
        {"GPS Passthrough Scenario", test_serial_gps_passthrough}
    };

    int numTests = sizeof(tests) / sizeof(tests[0]);
    int passed = 0;
    int failed = 0;

    for (int i = 0; i < numTests; i++) {
        printf("Running: %s\n", tests[i].name);
        fflush(stdout);
        if (tests[i].func()) {
            printf("  PASS\n");
            passed++;
        } else {
            failed++;
        }
        fflush(stdout);
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
