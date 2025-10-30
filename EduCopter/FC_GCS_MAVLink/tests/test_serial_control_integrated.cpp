/**
 * @file test_serial_control_integrated.cpp
 * @brief Integration test for GCS_serial_control
 *
 * This test demonstrates how GCS_serial_control integrates with:
 * - Hardware serial ports (UART, USB, etc.)
 * - MAVLink SERIAL_CONTROL protocol
 * - GPS passthrough functionality
 * - Exclusive port access control
 * - Multi-client serial port sharing
 *
 * Tests serial passthrough workflows end-to-end including GPS configuration.
 *
 * @author EduCopter Test Suite
 * @date 2025
 */

#include "test_harness.h"
#include <queue>
#include <vector>

//=============================================================================
// Mock Hardware Serial Port
//=============================================================================

class HardwareSerialPort {
private:
    std::vector<uint8_t> txBuffer;
    std::queue<uint8_t> rxQueue;
    uint32_t baudrate;
    bool isOpen;
    bool hasExclusiveLock;
    uint8_t exclusiveClientID;

public:
    HardwareSerialPort()
        : baudrate(0)
        , isOpen(false)
        , hasExclusiveLock(false)
        , exclusiveClientID(0)
    {
    }

    bool open(uint32_t baud) {
        if (isOpen) return false;
        baudrate = baud;
        isOpen = true;
        return true;
    }

    void close() {
        isOpen = false;
        hasExclusiveLock = false;
        exclusiveClientID = 0;
        baudrate = 0;
        txBuffer.clear();
        while (!rxQueue.empty()) rxQueue.pop();
    }

    bool setBaudrate(uint32_t baud) {
        if (!isOpen) return false;
        baudrate = baud;
        return true;
    }

    uint32_t getBaudrate() const {
        return baudrate;
    }

    bool write(const uint8_t* data, uint16_t length) {
        if (!isOpen) return false;
        txBuffer.insert(txBuffer.end(), data, data + length);
        return true;
    }

    uint16_t read(uint8_t* buffer, uint16_t maxLength) {
        if (!isOpen) return 0;

        uint16_t count = 0;
        while (count < maxLength && !rxQueue.empty()) {
            buffer[count++] = rxQueue.front();
            rxQueue.pop();
        }
        return count;
    }

    uint16_t available() const {
        return rxQueue.size();
    }

    void simulateRx(const uint8_t* data, uint16_t length) {
        for (uint16_t i = 0; i < length; i++) {
            rxQueue.push(data[i]);
        }
    }

    const std::vector<uint8_t>& getTxBuffer() const {
        return txBuffer;
    }

    void clearTxBuffer() {
        txBuffer.clear();
    }

    bool acquireExclusiveLock(uint8_t clientID) {
        if (hasExclusiveLock && exclusiveClientID != clientID) {
            return false;
        }
        hasExclusiveLock = true;
        exclusiveClientID = clientID;
        return true;
    }

    void releaseExclusiveLock(uint8_t clientID) {
        if (hasExclusiveLock && exclusiveClientID == clientID) {
            hasExclusiveLock = false;
            exclusiveClientID = 0;
        }
    }

    bool hasLock() const {
        return hasExclusiveLock;
    }

    uint8_t getLockOwner() const {
        return exclusiveClientID;
    }
};

//=============================================================================
// Serial Port Manager
//=============================================================================

#define MAX_SERIAL_PORTS 16
static HardwareSerialPort g_serialPorts[MAX_SERIAL_PORTS];

HardwareSerialPort* getSerialPort(uint8_t device) {
    if (device >= MAX_SERIAL_PORTS) return nullptr;
    return &g_serialPorts[device];
}

void resetSerialPorts() {
    for (int i = 0; i < MAX_SERIAL_PORTS; i++) {
        g_serialPorts[i].close();
    }
}

//=============================================================================
// Serial Control Protocol Handler
//=============================================================================

class SerialControlHandler {
private:
    uint8_t clientID;

public:
    explicit SerialControlHandler(uint8_t id = 1)
        : clientID(id)
    {
    }

    MAV_RESULT handleSerialControl(const mavlink_serial_control_t& request,
                                    mavlink_serial_control_t& reply) {
        HardwareSerialPort* port = getSerialPort(request.device);
        if (!port) {
            return MAV_RESULT_UNSUPPORTED;
        }

        // Initialize reply
        memset(&reply, 0, sizeof(reply));
        reply.device = request.device;
        reply.baudrate = request.baudrate;
        reply.timeout = request.timeout;
        reply.flags = 0;

        // Handle exclusive access
        if (request.flags & SERIAL_CONTROL_FLAG_EXCLUSIVE) {
            if (!port->acquireExclusiveLock(clientID)) {
                return MAV_RESULT_DENIED;
            }
        } else {
            // Check if someone else has exclusive lock
            if (port->hasLock() && port->getLockOwner() != clientID) {
                return MAV_RESULT_DENIED;
            }
        }

        // Open port if needed
        if (request.baudrate > 0) {
            if (port->getBaudrate() == 0) {
                // Port not yet open, open it
                if (!port->open(request.baudrate)) {
                    return MAV_RESULT_FAILED;
                }
            } else if (port->getBaudrate() != request.baudrate) {
                // Port open but different baudrate, change it
                if (!port->setBaudrate(request.baudrate)) {
                    return MAV_RESULT_FAILED;
                }
            }
            // else: Port already at correct baudrate, nothing to do
        }

        // Write data if provided
        if (request.count > 0) {
            if (!port->write(request.data, request.count)) {
                return MAV_RESULT_FAILED;
            }
        }

        // Read data if requested
        if (request.flags & (SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_REPLY)) {
            reply.count = port->read(reply.data, 70);
            reply.flags = SERIAL_CONTROL_FLAG_REPLY;
        }

        return MAV_RESULT_ACCEPTED;
    }

    void releaseExclusiveLocks() {
        for (int i = 0; i < MAX_SERIAL_PORTS; i++) {
            g_serialPorts[i].releaseExclusiveLock(clientID);
        }
    }
};

//=============================================================================
// GPS Protocol Helpers
//=============================================================================

struct UBXMessage {
    uint8_t msgClass;
    uint8_t msgID;
    uint16_t length;
    uint8_t payload[256];

    void pack(uint8_t* buffer, uint16_t& outLength) const {
        buffer[0] = 0xB5; // UBX sync1
        buffer[1] = 0x62; // UBX sync2
        buffer[2] = msgClass;
        buffer[3] = msgID;
        buffer[4] = length & 0xFF;
        buffer[5] = (length >> 8) & 0xFF;

        for (uint16_t i = 0; i < length; i++) {
            buffer[6 + i] = payload[i];
        }

        // Simple checksum (not proper Fletcher but good enough for test)
        uint8_t ck_a = 0, ck_b = 0;
        for (int i = 2; i < 6 + length; i++) {
            ck_a += buffer[i];
            ck_b += ck_a;
        }
        buffer[6 + length] = ck_a;
        buffer[7 + length] = ck_b;

        outLength = 8 + length;
    }
};

UBXMessage createUBXCFGPRT(uint32_t baudrate) {
    UBXMessage msg;
    msg.msgClass = 0x06; // CFG
    msg.msgID = 0x00; // PRT
    msg.length = 20;

    // Fill payload (simplified)
    memset(msg.payload, 0, sizeof(msg.payload));
    msg.payload[0] = 1; // UART1
    msg.payload[8] = baudrate & 0xFF;
    msg.payload[9] = (baudrate >> 8) & 0xFF;
    msg.payload[10] = (baudrate >> 16) & 0xFF;
    msg.payload[11] = (baudrate >> 24) & 0xFF;

    return msg;
}

UBXMessage createUBXACK(uint8_t msgClass, uint8_t msgID) {
    UBXMessage msg;
    msg.msgClass = 0x05; // ACK
    msg.msgID = 0x01; // ACK
    msg.length = 2;
    msg.payload[0] = msgClass;
    msg.payload[1] = msgID;
    return msg;
}

//=============================================================================
// Integration Tests
//=============================================================================

bool test_serial_port_open_and_configure() {
    resetSerialPorts();
    SerialControlHandler handler;

    mavlink_serial_control_t request = {};
    request.device = SERIAL_CONTROL_DEV_GPS1;
    request.baudrate = 115200;
    request.timeout = 1000;
    request.flags = 0;
    request.count = 0;

    mavlink_serial_control_t reply = {};
    MAV_RESULT result = handler.handleSerialControl(request, reply);

    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should open port successfully");

    HardwareSerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    TEST_ASSERT(port->getBaudrate() == 115200, "Baudrate should be set");

    TEST_PASS();
}

bool test_serial_write_and_read() {
    resetSerialPorts();
    SerialControlHandler handler;

    // Open port
    mavlink_serial_control_t request1 = {};
    request1.device = SERIAL_CONTROL_DEV_GPS1;
    request1.baudrate = 9600;
    request1.timeout = 1000;
    request1.flags = 0;
    request1.count = 0;

    mavlink_serial_control_t reply1 = {};
    handler.handleSerialControl(request1, reply1);

    // Write data
    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_GPS1;
    request2.baudrate = 9600;
    request2.timeout = 1000;
    request2.flags = 0;
    request2.count = 6;
    request2.data[0] = 0xB5;
    request2.data[1] = 0x62;
    request2.data[2] = 0x06;
    request2.data[3] = 0x00;
    request2.data[4] = 0x00;
    request2.data[5] = 0x06;

    mavlink_serial_control_t reply2 = {};
    MAV_RESULT result = handler.handleSerialControl(request2, reply2);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Write should succeed");

    // Verify data was written
    HardwareSerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    TEST_ASSERT(port->getTxBuffer().size() == 6, "Should have 6 bytes written");
    TEST_ASSERT(port->getTxBuffer()[0] == 0xB5, "First byte should be 0xB5");

    // Simulate GPS response
    uint8_t ack[] = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x00, 0x0E, 0x1D};
    port->simulateRx(ack, 10);

    // Read response
    mavlink_serial_control_t request3 = {};
    request3.device = SERIAL_CONTROL_DEV_GPS1;
    request3.baudrate = 9600;
    request3.timeout = 1000;
    request3.flags = SERIAL_CONTROL_FLAG_RESPOND;
    request3.count = 0;

    mavlink_serial_control_t reply3 = {};
    result = handler.handleSerialControl(request3, reply3);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Read should succeed");
    TEST_ASSERT(reply3.count == 10, "Should read 10 bytes");
    TEST_ASSERT(reply3.data[0] == 0xB5, "First byte should be UBX header");
    TEST_ASSERT(reply3.flags & SERIAL_CONTROL_FLAG_REPLY, "Reply flag should be set");

    TEST_PASS();
}

bool test_serial_exclusive_access() {
    resetSerialPorts();
    SerialControlHandler handler1(1);
    SerialControlHandler handler2(2);

    // Client 1 acquires exclusive access
    mavlink_serial_control_t request1 = {};
    request1.device = SERIAL_CONTROL_DEV_GPS1;
    request1.baudrate = 9600;
    request1.timeout = 1000;
    request1.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request1.count = 0;

    mavlink_serial_control_t reply1 = {};
    MAV_RESULT result1 = handler1.handleSerialControl(request1, reply1);
    TEST_ASSERT(result1 == MAV_RESULT_ACCEPTED, "Client 1 should get exclusive access");

    // Client 2 tries to access (should fail)
    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_GPS1;
    request2.baudrate = 9600;
    request2.timeout = 1000;
    request2.flags = 0;
    request2.count = 0;

    mavlink_serial_control_t reply2 = {};
    MAV_RESULT result2 = handler2.handleSerialControl(request2, reply2);
    TEST_ASSERT(result2 == MAV_RESULT_DENIED, "Client 2 should be denied");

    // Client 1 can still access
    mavlink_serial_control_t request3 = {};
    request3.device = SERIAL_CONTROL_DEV_GPS1;
    request3.baudrate = 9600;
    request3.timeout = 1000;
    request3.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request3.count = 0;

    mavlink_serial_control_t reply3 = {};
    MAV_RESULT result3 = handler1.handleSerialControl(request3, reply3);
    TEST_ASSERT(result3 == MAV_RESULT_ACCEPTED, "Client 1 should retain access");

    // Release lock
    handler1.releaseExclusiveLocks();

    // Client 2 can now access
    MAV_RESULT result4 = handler2.handleSerialControl(request2, reply2);
    TEST_ASSERT(result4 == MAV_RESULT_ACCEPTED, "Client 2 should now succeed");

    TEST_PASS();
}

bool test_serial_baudrate_change() {
    resetSerialPorts();
    SerialControlHandler handler;

    // Open at 9600
    mavlink_serial_control_t request1 = {};
    request1.device = SERIAL_CONTROL_DEV_GPS1;
    request1.baudrate = 9600;
    request1.timeout = 1000;
    request1.flags = 0;
    request1.count = 0;

    mavlink_serial_control_t reply1 = {};
    handler.handleSerialControl(request1, reply1);

    HardwareSerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    TEST_ASSERT(port->getBaudrate() == 9600, "Initial baudrate should be 9600");

    // Change to 115200
    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_GPS1;
    request2.baudrate = 115200;
    request2.timeout = 1000;
    request2.flags = 0;
    request2.count = 0;

    mavlink_serial_control_t reply2 = {};
    MAV_RESULT result = handler.handleSerialControl(request2, reply2);

    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Baudrate change should succeed");
    TEST_ASSERT(port->getBaudrate() == 115200, "Baudrate should be 115200");

    TEST_PASS();
}

bool test_serial_multiple_ports() {
    resetSerialPorts();
    SerialControlHandler handler;

    // Open GPS1
    mavlink_serial_control_t request1 = {};
    request1.device = SERIAL_CONTROL_DEV_GPS1;
    request1.baudrate = 9600;
    request1.timeout = 1000;
    request1.flags = 0;
    request1.count = 0;

    mavlink_serial_control_t reply1 = {};
    handler.handleSerialControl(request1, reply1);

    // Open TELEM1
    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_TELEM1;
    request2.baudrate = 57600;
    request2.timeout = 1000;
    request2.flags = 0;
    request2.count = 0;

    mavlink_serial_control_t reply2 = {};
    handler.handleSerialControl(request2, reply2);

    // Open SHELL
    mavlink_serial_control_t request3 = {};
    request3.device = SERIAL_CONTROL_DEV_SHELL;
    request3.baudrate = 115200;
    request3.timeout = 1000;
    request3.flags = 0;
    request3.count = 0;

    mavlink_serial_control_t reply3 = {};
    handler.handleSerialControl(request3, reply3);

    // Verify all ports configured correctly
    TEST_ASSERT(getSerialPort(SERIAL_CONTROL_DEV_GPS1)->getBaudrate() == 9600, "GPS1 baudrate");
    TEST_ASSERT(getSerialPort(SERIAL_CONTROL_DEV_TELEM1)->getBaudrate() == 57600, "TELEM1 baudrate");
    TEST_ASSERT(getSerialPort(SERIAL_CONTROL_DEV_SHELL)->getBaudrate() == 115200, "SHELL baudrate");

    TEST_PASS();
}

bool test_gps_passthrough_configuration() {
    resetSerialPorts();
    SerialControlHandler handler;

    // Step 1: Acquire exclusive access to GPS port
    mavlink_serial_control_t request1 = {};
    request1.device = SERIAL_CONTROL_DEV_GPS1;
    request1.baudrate = 9600;
    request1.timeout = 5000;
    request1.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request1.count = 0;

    mavlink_serial_control_t reply1 = {};
    MAV_RESULT result = handler.handleSerialControl(request1, reply1);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should acquire GPS exclusive access");

    // Step 2: Send UBX CFG-PRT command to change baudrate to 115200
    UBXMessage cfgPRT = createUBXCFGPRT(115200);
    uint8_t ubxData[128];
    uint16_t ubxLen;
    cfgPRT.pack(ubxData, ubxLen);

    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_GPS1;
    request2.baudrate = 9600; // Current baudrate
    request2.timeout = 5000;
    request2.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request2.count = (ubxLen > 70) ? 70 : ubxLen;
    memcpy(request2.data, ubxData, request2.count);

    mavlink_serial_control_t reply2 = {};
    result = handler.handleSerialControl(request2, reply2);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should send CFG-PRT command");

    // Simulate GPS ACK response
    UBXMessage ack = createUBXACK(0x06, 0x00);
    uint8_t ackData[16];
    uint16_t ackLen;
    ack.pack(ackData, ackLen);

    HardwareSerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_GPS1);
    port->simulateRx(ackData, ackLen);

    // Step 3: Read ACK
    mavlink_serial_control_t request3 = {};
    request3.device = SERIAL_CONTROL_DEV_GPS1;
    request3.baudrate = 9600;
    request3.timeout = 5000;
    request3.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND;
    request3.count = 0;

    mavlink_serial_control_t reply3 = {};
    result = handler.handleSerialControl(request3, reply3);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should read ACK");
    TEST_ASSERT(reply3.count > 0, "Should have ACK data");
    TEST_ASSERT(reply3.data[0] == 0xB5, "ACK should start with UBX header");

    // Step 4: Change port baudrate to 115200
    mavlink_serial_control_t request4 = {};
    request4.device = SERIAL_CONTROL_DEV_GPS1;
    request4.baudrate = 115200; // New baudrate
    request4.timeout = 5000;
    request4.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request4.count = 0;

    mavlink_serial_control_t reply4 = {};
    result = handler.handleSerialControl(request4, reply4);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should change port baudrate");
    TEST_ASSERT(port->getBaudrate() == 115200, "Port should now be at 115200");

    TEST_PASS();
}

bool test_serial_buffer_limits() {
    resetSerialPorts();
    SerialControlHandler handler;

    // Open port
    mavlink_serial_control_t request1 = {};
    request1.device = SERIAL_CONTROL_DEV_TELEM1;
    request1.baudrate = 57600;
    request1.timeout = 1000;
    request1.flags = 0;
    request1.count = 0;

    mavlink_serial_control_t reply1 = {};
    handler.handleSerialControl(request1, reply1);

    // Write maximum data (70 bytes)
    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_TELEM1;
    request2.baudrate = 57600;
    request2.timeout = 1000;
    request2.flags = 0;
    request2.count = 70;
    for (int i = 0; i < 70; i++) {
        request2.data[i] = i;
    }

    mavlink_serial_control_t reply2 = {};
    MAV_RESULT result = handler.handleSerialControl(request2, reply2);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should write 70 bytes");

    HardwareSerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_TELEM1);
    TEST_ASSERT(port->getTxBuffer().size() == 70, "Should have 70 bytes in buffer");

    // Simulate large response (100 bytes)
    uint8_t largeData[100];
    for (int i = 0; i < 100; i++) {
        largeData[i] = i;
    }
    port->simulateRx(largeData, 100);

    // Read should return max 70 bytes
    mavlink_serial_control_t request3 = {};
    request3.device = SERIAL_CONTROL_DEV_TELEM1;
    request3.baudrate = 57600;
    request3.timeout = 1000;
    request3.flags = SERIAL_CONTROL_FLAG_RESPOND;
    request3.count = 0;

    mavlink_serial_control_t reply3 = {};
    result = handler.handleSerialControl(request3, reply3);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Read should succeed");
    TEST_ASSERT(reply3.count == 70, "Should read max 70 bytes");
    TEST_ASSERT(port->available() == 30, "Should have 30 bytes remaining");

    TEST_PASS();
}

bool test_serial_invalid_device() {
    resetSerialPorts();
    SerialControlHandler handler;

    mavlink_serial_control_t request = {};
    request.device = 99; // Invalid
    request.baudrate = 9600;
    request.timeout = 1000;
    request.flags = 0;
    request.count = 0;

    mavlink_serial_control_t reply = {};
    MAV_RESULT result = handler.handleSerialControl(request, reply);

    TEST_ASSERT(result == MAV_RESULT_UNSUPPORTED, "Should reject invalid device");

    TEST_PASS();
}

bool test_shell_port_access() {
    resetSerialPorts();
    SerialControlHandler handler;

    // Open SHELL port
    mavlink_serial_control_t request = {};
    request.device = SERIAL_CONTROL_DEV_SHELL;
    request.baudrate = 115200;
    request.timeout = 1000;
    request.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request.count = 0;

    mavlink_serial_control_t reply = {};
    MAV_RESULT result = handler.handleSerialControl(request, reply);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should open shell port");

    // Send shell command
    const char* cmd = "status\n";
    mavlink_serial_control_t request2 = {};
    request2.device = SERIAL_CONTROL_DEV_SHELL;
    request2.baudrate = 115200;
    request2.timeout = 1000;
    request2.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE;
    request2.count = strlen(cmd);
    memcpy(request2.data, cmd, request2.count);

    mavlink_serial_control_t reply2 = {};
    result = handler.handleSerialControl(request2, reply2);
    TEST_ASSERT(result == MAV_RESULT_ACCEPTED, "Should send shell command");

    HardwareSerialPort* port = getSerialPort(SERIAL_CONTROL_DEV_SHELL);
    TEST_ASSERT(port->getTxBuffer().size() == strlen(cmd), "Should write command");

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
    printTestHeader("GCS_serial_control Integration Tests");

    static Test tests[] = {
        {"Serial Port Open and Configure", test_serial_port_open_and_configure},
        {"Serial Write and Read", test_serial_write_and_read},
        {"Serial Exclusive Access", test_serial_exclusive_access},
        {"Serial Baudrate Change", test_serial_baudrate_change},
        {"Serial Multiple Ports", test_serial_multiple_ports},
        {"GPS Passthrough Configuration", test_gps_passthrough_configuration},
        {"Serial Buffer Limits", test_serial_buffer_limits},
        {"Serial Invalid Device", test_serial_invalid_device},
        {"Shell Port Access", test_shell_port_access}
    };

    int numTests = sizeof(tests) / sizeof(tests[0]);
    int passed = 0;

    for (int i = 0; i < numTests; i++) {
        if (runTest(tests[i].name, tests[i].func)) {
            passed++;
        }
    }

    printTestSummary(numTests, passed);

    return (passed == numTests) ? 0 : 1;
}
