# GCS_MAVLink.h - Deep Dive Analysis

## File Location
`libraries/GCS_MAVLink/GCS_MAVLink.h`

## Purpose
This is the **LOW-LEVEL MAVLink protocol integration header**. It bridges the gap between the MAVLink protocol library and ArduPilot's UART hardware abstraction layer.

---

## Critical Configurations

### 1. Buffer Management (Lines 18-24)

```cpp
#if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
// allow 8 telemetry ports, allowing for extra networking or CAN ports
#define MAVLINK_COMM_NUM_BUFFERS 8
#else
// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5
#endif
```

**What This Means:**
- **Large Flight Controllers** (>1MB flash): Support 8 simultaneous MAVLink connections
  - Example: Pixhawk 4, Cube Orange, H7 processors
- **Small Flight Controllers** (<1MB flash): Support 5 connections
  - Example: Pixhawk 1, older F4 processors

**Why It Matters:**
Each buffer consumes memory for:
- Incoming message buffer (~300 bytes)
- Outgoing message buffer (~300 bytes)
- Channel status structures (~100 bytes)
- **Total per channel: ~700 bytes**

8 channels = 5.6KB of RAM just for buffers!

---

### 2. MAVLink Send Macros (Lines 13-16)

```cpp
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)
#define MAVLINK_START_UART_SEND(chan, size) comm_send_lock(chan, size)
#define MAVLINK_END_UART_SEND(chan, size) comm_send_unlock(chan)
```

**How MAVLink Library Uses These:**

When MAVLink library wants to send a message:
```cpp
// Inside mavlink_msg_heartbeat_send():
MAVLINK_START_UART_SEND(chan, msg_length);  // Lock the channel
MAVLINK_SEND_UART_BYTES(chan, packet, length); // Write bytes
MAVLINK_END_UART_SEND(chan, msg_length);    // Unlock the channel
```

**Thread Safety:**
These macros provide **semaphore-based locking** to prevent:
- Two threads sending at the same time
- Interleaved messages (corrupted packets)
- Race conditions on the UART hardware

---

### 3. Channel Buffer Access (Lines 26-27)

```cpp
#define MAVLINK_GET_CHANNEL_BUFFER 1
#define MAVLINK_GET_CHANNEL_STATUS 1
```

**Enables these functions:**
```cpp
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
```

**Used by message parsing:**
```cpp
// In update_receive():
mavlink_message_t* msg = mavlink_get_channel_buffer(chan);
mavlink_status_t* status = mavlink_get_channel_status(chan);

// Parse incoming byte:
if (mavlink_parse_char(chan, byte, msg, status)) {
    // Complete message received!
    handle_message(*msg);
}
```

---

## Global Variables (Lines 47-51)

### 1. Port Mapping
```cpp
extern AP_HAL::UARTDriver *mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
```

**Array Maps:**
```
Index 0 (MAVLINK_COMM_0) → Serial0 (usually USB)
Index 1 (MAVLINK_COMM_1) → Serial1 (Telem1 port)
Index 2 (MAVLINK_COMM_2) → Serial2 (Telem2 port)
Index 3 (MAVLINK_COMM_3) → Serial3 (GPS2 or Telem3)
...
```

**Example Usage:**
```cpp
// In GCS_MAVLink::init():
mavlink_comm_port[chan] = _port;  // Associate channel with UART
```

### 2. Alternative Protocol Flag
```cpp
extern bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];
```

**Purpose:** Allow non-MAVLink protocols to temporarily take over a port

**Use Case: BLHeli Pass-Through**
```cpp
// When configuring ESCs:
gcs_alternative_active[chan] = true;  // Disable MAVLink on this port
// ... send BLHeli configuration commands ...
gcs_alternative_active[chan] = false; // Re-enable MAVLink
```

### 3. System Identity
```cpp
extern mavlink_system_t mavlink_system;
```

**Structure:**
```cpp
mavlink_system_t {
    uint8_t sysid;    // This vehicle's system ID (default: 1)
    uint8_t compid;   // This vehicle's component ID (usually 1 = autopilot)
}
```

**Set During Initialization:**
```cpp
// In GCS::init():
mavlink_system.sysid = sysid_this_mav();  // From MAV_SYSID parameter
mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1;
```

---

## Key Functions

### 1. Channel Validation (Lines 56-59)
```cpp
static inline bool valid_channel(mavlink_channel_t chan)
{
    return static_cast<int>(chan) < MAVLINK_COMM_NUM_BUFFERS;
}
```

**Prevents:**
```cpp
// Crash scenario without validation:
mavlink_channel_t chan = (mavlink_channel_t)99;  // Invalid!
mavlink_comm_port[chan]->write(...);  // SEGFAULT - array out of bounds
```

**Safe Usage:**
```cpp
if (valid_channel(chan)) {
    mavlink_comm_port[chan]->write(...);  // Safe!
}
```

---

### 2. Buffer Management (Lines 61-62)
```cpp
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
```

**Implementation (from GCS_MAVLink.cpp):**
```cpp
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) return nullptr;
    return link->channel_buffer();  // Returns &_channel_buffer
}
```

**Why Separate Buffers Per Channel?**
Parallel processing! Multiple channels can receive messages simultaneously:
```
USB:     [Parsing byte 5 of PARAM_SET]
Telem1:  [Parsing byte 12 of COMMAND_LONG]
Telem2:  [Complete HEARTBEAT message ready]
```

---

### 3. Transmit Space Check (Line 70)
```cpp
uint16_t comm_get_txspace(mavlink_channel_t chan);
```

**Critical for Flow Control:**
```cpp
// Before sending a large message:
if (comm_get_txspace(chan) >= message_size) {
    send_message();  // Safe to send
} else {
    // Buffer full! Defer sending to avoid blocking
    queue_for_later();
}
```

**Real-World Example:**
```cpp
// Sending a large mission (100 waypoints)
for (int i = 0; i < 100; i++) {
    while (comm_get_txspace(chan) < 100) {
        delay_microseconds(100);  // Wait for space
    }
    send_waypoint(i);
}
```

---

### 4. Multi-threading Locks (Lines 83-85)
```cpp
void comm_send_lock(mavlink_channel_t chan, uint16_t size);
void comm_send_unlock(mavlink_channel_t chan);
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan);
```

**Scenario Requiring Locks:**
```
Thread 1: Sending ATTITUDE message
Thread 2: Sending GPS_RAW message
Both writing to same UART simultaneously → Corrupted packets!
```

**Safe Implementation:**
```cpp
// Thread 1:
comm_send_lock(chan, ATTITUDE_msg_size);
  write_attitude_bytes();
comm_send_unlock(chan);

// Thread 2 (waits for Thread 1 to finish):
comm_send_lock(chan, GPS_RAW_msg_size);
  write_gps_bytes();
comm_send_unlock(chan);
```

---

## MAVLink Version Selection (Lines 40-44, 79)

```cpp
#include "include/mavlink/v2.0/all/version.h"
#define MAVLINK_MAX_PAYLOAD_LEN 255
#include "include/mavlink/v2.0/all/mavlink.h"
```

**MAVLink 2 vs MAVLink 1:**

| Feature | MAVLink 1 | MAVLink 2 |
|---------|-----------|-----------|
| Max Payload | 255 bytes | 255 bytes |
| Packet Overhead | 8 bytes | 14 bytes (with signing) |
| Signing Support | No | Yes |
| Message Extensions | No | Yes |
| Backwards Compatible | - | Yes (can talk to v1) |

**Runtime Detection:**
```cpp
// In GCS_MAVLINK::init():
if (protocol == SerialProtocol_MAVLink2) {
    // Use MAVLink 2 features
} else {
    // Force MAVLink 1 mode
    _channel_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}
```

---

## Compiler Warnings Management (Lines 33-38, 74-80, 87)

```cpp
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
// ... include MAVLink headers ...
#pragma GCC diagnostic pop
```

**Why Needed?**

MAVLink uses **packed structures**:
```cpp
typedef struct __mavlink_heartbeat_t {
    uint32_t custom_mode;  // Offset: 0
    uint8_t type;          // Offset: 4 (misaligned!)
    uint8_t autopilot;     // Offset: 5
    uint8_t base_mode;     // Offset: 6
    uint8_t system_status; // Offset: 7
    uint8_t mavlink_version; // Offset: 8
} __attribute__((__packed__)) mavlink_heartbeat_t;
```

**Problem:**
```cpp
mavlink_heartbeat_t msg;
uint32_t* ptr = &msg.custom_mode;  // WARNING: may be misaligned!
```

**Solution:** Suppress warnings since MAVLink library handles alignment internally.

---

## Communication Flow Example

### Sending a Message

```cpp
// High-level call:
GCS_MAVLINK::send_heartbeat();

↓ [Step 1: Check space]
if (!HAVE_PAYLOAD_SPACE(chan, HEARTBEAT)) return;

↓ [Step 2: Lock channel]
comm_send_lock(chan, packet_size);

↓ [Step 3: MAVLink library encodes message]
mavlink_msg_heartbeat_pack(
    mavlink_system.sysid,
    mavlink_system.compid,
    &msg,
    type, autopilot, base_mode, custom_mode, system_status
);

↓ [Step 4: Send bytes]
MAVLINK_SEND_UART_BYTES(chan, msg_buf, msg_len);
    └→ comm_send_buffer(chan, buf, len)
        └→ mavlink_comm_port[chan]->write(buf, len)
            └→ Hardware UART transmission

↓ [Step 5: Unlock channel]
comm_send_unlock(chan);
```

### Receiving a Message

```cpp
// Hardware interrupt fires when byte arrives:
UART_IRQ_Handler() {
    uint8_t byte = read_uart_register();

    ↓ [Call ArduPilot receive handler]
    GCS_MAVLINK::update_receive();

        ↓ [Get buffers]
        mavlink_message_t* msg = mavlink_get_channel_buffer(chan);
        mavlink_status_t* status = mavlink_get_channel_status(chan);

        ↓ [Parse byte]
        if (mavlink_parse_char(chan, byte, msg, status)) {
            // Complete message assembled!

            ↓ [Route to handler]
            handle_message(msg);
                └→ switch(msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        handle_heartbeat(msg);
                        break;
                    case MAVLINK_MSG_ID_PARAM_SET:
                        handle_param_set(msg);
                        break;
                    ...
                }
        }
}
```

---

## Memory Layout

### Per-Channel Memory Allocation

```
Channel 0 (USB):
├─ mavlink_message_t _channel_buffer     [300 bytes]
├─ mavlink_status_t _channel_status      [100 bytes]
├─ HAL_Semaphore chan_lock               [20 bytes]
└─ AP_HAL::UARTDriver* port pointer      [8 bytes]
                                Total:    ~428 bytes

Channel 1 (Telem1): [same structure]     ~428 bytes
Channel 2 (Telem2): [same structure]     ~428 bytes
...

Total for 8 channels:                    ~3.4 KB
```

---

## Testing Examples

### Test 1: Channel Validation
```cpp
void test_channel_validation() {
    // Valid channels
    assert(valid_channel(MAVLINK_COMM_0) == true);
    assert(valid_channel(MAVLINK_COMM_1) == true);

    // Invalid channels
    assert(valid_channel((mavlink_channel_t)99) == false);
    assert(valid_channel((mavlink_channel_t)-1) == false);
}
```

### Test 2: Buffer Space Check
```cpp
void test_transmit_space() {
    mavlink_channel_t chan = MAVLINK_COMM_0;

    uint16_t space = comm_get_txspace(chan);
    printf("Available TX space: %u bytes\n", space);

    // Should have at least 256 bytes for a message
    assert(space >= 256);
}
```

### Test 3: Thread Safety
```cpp
void test_concurrent_send() {
    mavlink_channel_t chan = MAVLINK_COMM_0;

    // Thread 1
    std::thread t1([chan]() {
        for (int i = 0; i < 100; i++) {
            comm_send_lock(chan, 50);
            // Simulate sending
            hal.scheduler->delay(1);
            comm_send_unlock(chan);
        }
    });

    // Thread 2
    std::thread t2([chan]() {
        for (int i = 0; i < 100; i++) {
            comm_send_lock(chan, 50);
            // Simulate sending
            hal.scheduler->delay(1);
            comm_send_unlock(chan);
        }
    });

    t1.join();
    t2.join();
    // If no crashes/corruption, locks are working!
}
```

---

## Common Issues & Solutions

### Issue 1: "No UART on channel X"
**Symptom:** Messages not being sent
**Cause:** `mavlink_comm_port[chan]` is nullptr
**Solution:** Check serial port configuration in parameters (SERIALn_PROTOCOL)

### Issue 2: "Messages getting corrupted"
**Symptom:** QGC shows parse errors
**Cause:** Missing lock when sending from multiple threads
**Solution:** Always use comm_send_lock/unlock

### Issue 3: "Out of memory"
**Symptom:** Firmware won't compile with 8 channels
**Cause:** Small flight controller (<1MB flash)
**Solution:** Reduce MAVLINK_COMM_NUM_BUFFERS to 5

---

## Key Takeaways

1. **GCS_MAVLink.h is the hardware abstraction layer** - it connects protocol to UART
2. **Separate buffers per channel** enable parallel communication
3. **Thread safety is critical** for reliable communication
4. **Buffer management prevents packet loss** via flow control
5. **MAVLink 2 is preferred** but can fall back to MAVLink 1
6. **Memory is precious** - buffer count depends on flash size

This file is **foundational** - all MAVLink communication flows through these primitives!
