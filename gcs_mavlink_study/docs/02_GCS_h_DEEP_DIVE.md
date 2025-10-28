# GCS.h - Deep Dive Analysis

## File Location
`libraries/GCS_MAVLink/GCS.h`

## Purpose
This is the **MASTER CLASS DEFINITION** for all Ground Control Station communication. It defines two critical classes:
- **GCS_MAVLINK**: Per-channel communication handler (one instance per telemetry port)
- **GCS**: Singleton coordinator that manages all GCS_MAVLINK instances

---

## Class Hierarchy

```
┌─────────────────────────────────────┐
│     GCS (Singleton Manager)         │
│  - Manages all channels             │
│  - Broadcasts messages              │
│  - Sensor status coordination       │
└────────────┬────────────────────────┘
             │ has array of
             ▼
┌─────────────────────────────────────┐
│  GCS_MAVLINK[0..7]                  │  ← One per port
│  - Channel-specific handling        │
│  - Message send/receive             │
│  - Stream rate management           │
└─────────────────────────────────────┘
             │ uses
             ▼
┌─────────────────────────────────────┐
│  MissionItemProtocol                │
│  - Waypoint upload/download         │
│  - Rally point handling             │
│  - Fence management                 │
└─────────────────────────────────────┘
```

---

## Part 1: GCS_MAVLINK Class (Lines 174-1085)

### Core Structure

```cpp
class GCS_MAVLINK {
public:
    GCS_MAVLINK(AP_HAL::UARTDriver &uart);  // Constructor
    bool init(uint8_t instance);            // Initialize channel
    void update_receive(uint32_t max_time_us=1000);  // Process incoming
    void update_send();                     // Send queued messages

private:
    AP_HAL::UARTDriver *_port;              // Hardware UART
    mavlink_channel_t chan;                 // Channel ID (COMM_0, COMM_1, etc.)
    mavlink_message_t _channel_buffer;      // RX message buffer
    mavlink_status_t _channel_status;       // Parser state
};
```

---

### Message Sending System

#### 1. Payload Space Checking (Lines 41-81)

**The Problem:**
```cpp
// BAD: Send without checking space
send_heartbeat();  // What if TX buffer is full?
send_attitude();   // Messages could be lost!
```

**The Solution: CHECK_PAYLOAD_SIZE Macro**

```cpp
#define PAYLOAD_SIZE(chan, id) \
    (GCS_MAVLINK::packet_overhead_chan(chan) + MAVLINK_MSG_ID_##id##_LEN)

#define HAVE_PAYLOAD_SPACE(_chan, id) \
    (comm_get_txspace(_chan) >= PAYLOAD_SIZE(_chan, id) ? \
        true : (gcs_out_of_space_to_send(_chan), false))

#define CHECK_PAYLOAD_SIZE(id) \
    if (!check_payload_size(MAVLINK_MSG_ID_##id##_LEN)) return false
```

**Real-World Usage:**
```cpp
void GCS_MAVLINK::send_heartbeat() const {
    CHECK_PAYLOAD_SIZE(HEARTBEAT);  // Expands to space check

    mavlink_msg_heartbeat_send(
        chan,
        mavlink_type,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode(),
        custom_mode(),
        system_status()
    );
}
```

**What CHECK_PAYLOAD_SIZE Actually Does:**

```cpp
// Expansion of CHECK_PAYLOAD_SIZE(HEARTBEAT):
if (!check_payload_size(MAVLINK_MSG_ID_HEARTBEAT_LEN)) {
    return false;  // Not enough space, abort send
}

// check_payload_size implementation:
bool GCS_MAVLINK::check_payload_size(uint16_t max_payload_len) {
    uint16_t required = packet_overhead() + max_payload_len;
    if (txspace() < required) {
        gcs_out_of_space_to_send(chan);  // Track missed sends
        return false;
    }
    return true;
}
```

**Packet Overhead Calculation:**
```cpp
// MAVLink 1:  8 bytes (header) + 2 bytes (checksum) = 10 bytes
// MAVLink 2: 10 bytes (header) + 2 bytes (checksum) +
//            13 bytes (signature if enabled) = 25 bytes

HEARTBEAT message: 9 bytes payload
Total MAVLink 2 packet: 10 + 9 + 2 + 13 = 34 bytes
```

---

#### 2. Message Streams (Lines 268-285)

```cpp
enum streams : uint8_t {
    STREAM_RAW_SENSORS,      // IMU, baro, mag raw data
    STREAM_EXTENDED_STATUS,  // System status, battery, GPS
    STREAM_RC_CHANNELS,      // RC input values
    STREAM_RAW_CONTROLLER,   // PID outputs
    STREAM_POSITION,         // GPS position
    STREAM_EXTRA1,           // Attitude (roll/pitch/yaw)
    STREAM_EXTRA2,           // VFR_HUD (velocity, heading)
    STREAM_EXTRA3,           // AHRS, HWSTATUS, WIND
    STREAM_PARAMS,           // Parameter streaming
    STREAM_ADSB,             // ADS-B traffic
    NUM_STREAMS
};
```

**Stream Rate Configuration:**
```cpp
// Per-channel parameters (visible in QGC/Mission Planner):
SR0_RAW_SENS  = 2   // 2 Hz on channel 0 (USB)
SR0_EXT_STAT  = 2   // 2 Hz
SR0_POSITION  = 3   // 3 Hz
SR0_EXTRA1    = 10  // 10 Hz (attitude updates)

SR1_RAW_SENS  = 1   // 1 Hz on channel 1 (Telem1)
SR1_EXT_STAT  = 1   // Slower on telemetry radio
SR1_POSITION  = 2
SR1_EXTRA1    = 4   // Lower bandwidth link
```

**How Streams Work:**

```cpp
// Simplified update_send() logic:
void GCS_MAVLINK::update_send() {
    uint32_t now = AP_HAL::millis();

    // Check each stream
    for (uint8_t i = 0; i < NUM_STREAMS; i++) {
        uint16_t rate_hz = streamRates[i];
        if (rate_hz == 0) continue;  // Stream disabled

        uint16_t interval_ms = 1000 / rate_hz;

        if (now - last_sent_ms[i] >= interval_ms) {
            send_stream_messages(i);
            last_sent_ms[i] = now;
        }
    }
}
```

**Stream-to-Message Mapping:**
```cpp
// In vehicle-specific code (e.g., Copter):
const GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    {STREAM_RAW_SENSORS,
     {MSG_RAW_IMU, MSG_SCALED_IMU, MSG_SCALED_PRESSURE}, 3},

    {STREAM_EXTENDED_STATUS,
     {MSG_SYS_STATUS, MSG_POWER_STATUS, MSG_MEMINFO, MSG_GPS_RAW}, 4},

    {STREAM_POSITION,
     {MSG_LOCATION, MSG_LOCAL_POSITION}, 2},

    {STREAM_EXTRA1,
     {MSG_ATTITUDE, MSG_SIMSTATE, MSG_AHRS2}, 3},
};
```

---

#### 3. Deferred Message Queue (Lines 852-895)

**The Problem:**
Sending messages immediately can:
- Block the main loop
- Overflow the UART buffer
- Cause timing jitter

**The Solution: Deferred Message System**

```cpp
// Special high-priority messages
struct deferred_message_t {
    const ap_message id;      // MSG_HEARTBEAT, MSG_NEXT_PARAM, etc.
    uint16_t interval_ms;     // Send every N milliseconds
    uint16_t last_sent_ms;    // Timestamp of last send
} deferred_message[3] = {
    { MSG_HEARTBEAT, },       // Always highest priority
    { MSG_NEXT_PARAM, },      // Parameter streaming
    { MSG_HIGH_LATENCY2, },   // Low-bandwidth mode
};

// Regular messages organized in buckets
struct deferred_message_bucket_t {
    Bitmask<MSG_LAST> ap_message_ids;  // Which messages in this bucket
    uint16_t interval_ms;              // Bucket send rate
    uint16_t last_sent_ms;
};
deferred_message_bucket_t deferred_message_bucket[10];
```

**How It Works:**

```cpp
// 1. Queueing a message
void GCS_MAVLINK::send_message(enum ap_message id) {
    pushed_ap_message_ids.set(id);  // Mark as needing send
}

// 2. Sending from queue (in update_send)
void GCS_MAVLINK::update_send() {
    // Send high-priority deferred messages first
    for (int i = 0; i < 3; i++) {
        if (should_send_deferred_message(i)) {
            do_try_send_message(deferred_message[i].id);
        }
    }

    // Then send from buckets
    if (sending_bucket_id != no_bucket_to_send) {
        send_bucket_messages();
    }
}
```

**Bucket System Example:**

```
Bucket 0 (100ms interval):  MSG_HEARTBEAT, MSG_SYS_STATUS
Bucket 1 (50ms interval):   MSG_ATTITUDE, MSG_GPS_RAW
Bucket 2 (200ms interval):  MSG_RC_CHANNELS, MSG_BATTERY_STATUS
Bucket 3 (500ms interval):  MSG_MEMINFO, MSG_HWSTATUS
...
```

**Benefits:**
- **Predictable timing**: Messages sent at regular intervals
- **Priority system**: Critical messages (heartbeat) never blocked
- **Bandwidth management**: Automatically scales with link speed

---

### Message Reception System

#### 1. update_receive() (Line 191)

```cpp
void GCS_MAVLINK::update_receive(uint32_t max_time_us) {
    uint32_t start_us = AP_HAL::micros();

    // Process incoming bytes for up to max_time_us microseconds
    while ((AP_HAL::micros() - start_us) < max_time_us) {

        // Read one byte from UART
        int16_t byte = _port->read();
        if (byte < 0) break;  // No more data

        // Get parsing buffers
        mavlink_message_t* msg = &_channel_buffer;
        mavlink_status_t* status = &_channel_status;

        // Parse the byte
        if (mavlink_parse_char(chan, byte, msg, status)) {
            // Complete message received!
            packetReceived(*status, *msg);
        }
    }
}
```

**Key Features:**
- **Time-limited**: Won't block main loop
- **Byte-by-byte parsing**: Handles fragmented data
- **State machine**: MAVLink library tracks partial messages

**Parsing State Machine:**
```
IDLE → STX → LEN → SEQ → SYSID → COMPID → MSGID → PAYLOAD → CRC → COMPLETE
```

---

#### 2. packetReceived() (Line 232)

```cpp
void GCS_MAVLINK::packetReceived(const mavlink_status_t &status,
                                 const mavlink_message_t &msg) {
    // 1. Update routing table
    routing.check_and_forward(*this, msg);

    // 2. Check if message is for us
    if (!accept_packet(status, msg)) {
        return;  // Wrong target system/component
    }

    // 3. Route to specific handler
    handle_message(msg);
}
```

---

#### 3. handle_message() (Line 614)

```cpp
void GCS_MAVLINK::handle_message(const mavlink_message_t &msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handle_heartbeat(msg);
            break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handle_param_request_list(msg);
            break;

        case MAVLINK_MSG_ID_PARAM_SET:
            handle_param_set(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_LONG:
            handle_command_long(msg);
            break;

        case MAVLINK_MSG_ID_COMMAND_INT:
            handle_command_int(msg);
            break;

        case MAVLINK_MSG_ID_MISSION_COUNT:
            handle_mission_count(msg);
            break;

        // ... 100+ message types ...
    }
}
```

---

### Parameter Handling (Lines 514-595)

#### Parameter Streaming (Lines 514-520)
```cpp
AP_Param *_queued_parameter;           // Current parameter being sent
enum ap_var_type _queued_parameter_type;  // Parameter data type
AP_Param::ParamToken _queued_parameter_token;  // Iterator
uint16_t _queued_parameter_index;      // Current index (0..count-1)
uint16_t _queued_parameter_count;      // Total parameter count
```

**Parameter Download Flow:**
```
1. GCS sends PARAM_REQUEST_LIST
2. Vehicle sets _queued_parameter = first parameter
3. update_send() calls queued_param_send() periodically
4. Sends PARAM_VALUE for current parameter
5. Advances to next parameter
6. Repeat until all sent
```

**Bandwidth Management:**
```cpp
void GCS_MAVLINK::queued_param_send() {
    // Use at most 30% of bandwidth for parameters
    uint32_t link_bw = _port->bw_in_bytes_per_second();
    uint32_t bytes_allowed = link_bw * time_delta / 3333;

    // Calculate how many parameters can fit
    uint32_t count = bytes_allowed / PARAM_VALUE_size;

    // Send up to 'count' parameters
    while (count-- && _queued_parameter != nullptr) {
        send_parameter(_queued_parameter);
        _queued_parameter = AP_Param::next(...);
    }
}
```

---

### Mission/Waypoint Protocol (Lines 564-582)

**Mission Item Protocol Integration:**
```cpp
// Three protocol handlers (defined in GCS class)
static MissionItemProtocol *missionitemprotocols[3];

// Indexed by MAV_MISSION_TYPE:
// [0] = MAV_MISSION_TYPE_MISSION   → Waypoints
// [1] = MAV_MISSION_TYPE_FENCE     → Geofence
// [2] = MAV_MISSION_TYPE_RALLY     → Rally points
```

**Message Routing:**
```cpp
void GCS_MAVLINK::handle_mission_count(const mavlink_message_t &msg) {
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(&msg, &packet);

    // Get appropriate protocol handler
    MissionItemProtocol *prot = gcs().get_prot_for_mission_type(
        (MAV_MISSION_TYPE)packet.mission_type
    );

    // Forward to handler
    if (prot != nullptr) {
        prot->handle_mission_count(*this, packet, msg);
    }
}
```

---

### Advanced Features

#### 1. Flow Control (Lines 287-289)
```cpp
bool is_high_bandwidth() {
    return chan == MAVLINK_COMM_0;  // USB is fast
}

bool have_flow_control();  // RTS/CTS or USB
```

**Impact on Behavior:**
```cpp
// High bandwidth (USB):
- Send parameters faster
- Use higher stream rates
- Less aggressive throttling

// Low bandwidth (radio):
- Throttle parameter sends
- Lower stream rates
- More careful buffer management
```

---

#### 2. Private Channels (Lines 424-437)
```cpp
static void set_channel_private(mavlink_channel_t chan);
static bool is_private(mavlink_channel_t _chan);
```

**Use Case: Companion Computer**
```cpp
// Telem2 connected to Raspberry Pi running DroneKit
set_channel_private(MAVLINK_COMM_2);

// Results:
// ✓ Heartbeats sent to Pi
// ✓ Pi can send commands
// ✗ Broadcasts NOT forwarded to Pi
// ✗ Messages from Pi NOT forwarded to GCS
```

**Why?**
- Prevents message loops
- Reduces bandwidth on GCS link
- Isolates companion computer traffic

---

#### 3. Signing (Lines 470-473, 1010-1021)

```cpp
#if AP_MAVLINK_SIGNING_ENABLED
    mavlink_signing_t signing;
    static mavlink_signing_streams_t signing_streams;

    void load_signing_key(void);
    bool signing_enabled(void) const;
    static void update_signing_timestamp(uint64_t timestamp_usec);
#endif
```

**Signing Process:**
```
1. Load secret key from EEPROM
2. For each outgoing message:
   - Append timestamp
   - Calculate SHA-256 HMAC
   - Append 6-byte signature
3. For each incoming message:
   - Extract signature
   - Recalculate HMAC
   - Compare signatures
   - Reject if mismatch
```

**Security Benefits:**
- Prevents command injection
- Detects message tampering
- Replay attack protection (timestamp)

---

## Part 2: GCS Class (Lines 1089-1375)

### Singleton Pattern

```cpp
class GCS {
public:
    GCS() {
        if (_singleton == nullptr) {
            _singleton = this;
        } else {
            AP_HAL::panic("GCS must be singleton");
        }
    }

    static GCS *get_singleton() { return _singleton; }

private:
    static GCS *_singleton;
};

// Global accessor
GCS &gcs() {
    return *GCS::get_singleton();
}
```

**Usage:**
```cpp
// Anywhere in ArduPilot code:
gcs().send_text(MAV_SEVERITY_INFO, "Taking off!");
gcs().send_message(MSG_ATTITUDE);
```

---

### Channel Management (Lines 1163-1166, 1291-1293)

```cpp
private:
    uint8_t _num_gcs;                        // Number of active channels
    GCS_MAVLINK *_chan[MAVLINK_COMM_NUM_BUFFERS];  // Channel array

public:
    virtual GCS_MAVLINK *chan(const uint8_t ofs) = 0;  // Get channel
    uint8_t num_gcs() const { return _num_gcs; }
```

**Channel Access:**
```cpp
// Get channel 0 (USB)
GCS_MAVLINK *usb = gcs().chan(0);
if (usb != nullptr && usb->is_active()) {
    usb->send_message(MSG_HEARTBEAT);
}

// Iterate all channels
for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
    GCS_MAVLINK *link = gcs().chan(i);
    if (link->is_active()) {
        link->send_message(MSG_GPS_RAW);
    }
}
```

---

### Broadcasting (Lines 1156, 1167-1168)

```cpp
void send_to_active_channels(uint32_t msgid, const char *pkt);
void send_message(enum ap_message id);
void send_text(MAV_SEVERITY severity, const char *fmt, ...);
```

**Broadcast Logic:**
```cpp
void GCS::send_message(enum ap_message id) {
    for (uint8_t i = 0; i < _num_gcs; i++) {
        GCS_MAVLINK *link = chan(i);
        if (link->is_private()) continue;      // Skip private channels
        if (!link->is_active()) continue;      // Skip inactive
        if (link->is_high_latency_link) continue;  // Skip HL2

        link->send_message(id);  // Send to this channel
    }
}
```

---

### StatusText Queue (Lines 1118-1138)

```cpp
struct statustext_t {
    mavlink_statustext_t msg;      // MAVLink message
    uint16_t entry_created_ms;     // Timestamp
    uint8_t bitmask;               // Which channels need it
};

class StatusTextQueue : public ObjectArray<statustext_t> {
    HAL_Semaphore _sem;  // Thread-safe!
};

StatusTextQueue _statustext_queue{_status_capacity};  // 7 or 30 slots
```

**Thread-Safe Text Sending:**
```cpp
// Can be called from ANY thread!
void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...) {
    WITH_SEMAPHORE(_statustext_queue.semaphore());

    // Format message
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // Add to queue
    statustext_t entry;
    entry.msg.severity = severity;
    strncpy((char*)entry.msg.text, buffer, sizeof(entry.msg.text));
    entry.bitmask = statustext_send_channel_mask();

    _statustext_queue.push(entry);
}
```

**Queue Processing:**
```cpp
void GCS::update_send() {
    // In main loop, periodically:
    service_statustext();
}

void GCS::service_statustext() {
    statustext_t *entry = _statustext_queue.front();
    if (entry == nullptr) return;

    // Try to send to channels in bitmask
    for (uint8_t i = 0; i < num_gcs(); i++) {
        if (entry->bitmask & (1<<i)) {
            if (chan(i)->send_statustext(entry->msg)) {
                entry->bitmask &= ~(1<<i);  // Sent successfully
            }
        }
    }

    // Remove if sent to all channels
    if (entry->bitmask == 0) {
        _statustext_queue.pop();
    }
}
```

---

### Sensor Status Flags (Lines 1131-1148, 1285-1289)

```cpp
HAL_Semaphore control_sensors_sem;
uint32_t control_sensors_present;   // Sensors that exist
uint32_t control_sensors_enabled;   // Sensors that are active
uint32_t control_sensors_health;    // Sensors that are healthy

void get_sensor_status_flags(uint32_t &present,
                              uint32_t &enabled,
                              uint32_t &health);
```

**Bitmask Flags (from MAVLink spec):**
```cpp
MAV_SYS_STATUS_SENSOR_3D_GYRO           = 0x01
MAV_SYS_STATUS_SENSOR_3D_ACCEL          = 0x02
MAV_SYS_STATUS_SENSOR_3D_MAG            = 0x04
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 0x08
MAV_SYS_STATUS_SENSOR_GPS               = 0x20
MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW      = 0x40
MAV_SYS_STATUS_SENSOR_VISION_POSITION   = 0x80
MAV_SYS_STATUS_SENSOR_BATTERY           = 0x200
MAV_SYS_STATUS_AHRS                     = 0x400
MAV_SYS_STATUS_GEOFENCE                 = 0x1000
```

**Example in SYS_STATUS Message:**
```cpp
void GCS_MAVLINK::send_sys_status() {
    uint32_t present, enabled, health;
    gcs().get_sensor_status_flags(present, enabled, health);

    mavlink_msg_sys_status_send(
        chan,
        present,   // 0x0000062F (gyro, accel, mag, baro, GPS present)
        enabled,   // 0x0000062F (all enabled)
        health,    // 0x0000042F (GPS unhealthy - no fix)
        load,      // CPU load
        voltage_battery,
        current_battery,
        battery_remaining,
        drop_rate_comm,
        errors_comm
    );
}
```

**QGC Displays:**
```
Green checkmark:  (present & enabled & health) == true
Yellow warning:   (present & enabled) but not health
Red X:           not present or not enabled
```

---

### Vehicle Type Information (Lines 1114-1116)

```cpp
virtual uint32_t custom_mode() const = 0;  // Flight mode
virtual MAV_TYPE frame_type() const = 0;   // Vehicle type
virtual const char* frame_string() const { return nullptr; }
```

**Implemented by Vehicle:**
```cpp
// In Copter/GCS_Mavlink.cpp:
uint32_t GCS_Copter::custom_mode() const {
    return (uint32_t)copter.control_mode;
}

MAV_TYPE GCS_Copter::frame_type() const {
    return MAV_TYPE_QUADROTOR;  // or HEXAROTOR, OCTOROTOR, etc.
}
```

**Sent in HEARTBEAT:**
```cpp
mavlink_msg_heartbeat_send(
    chan,
    MAV_TYPE_QUADROTOR,        // type
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    base_mode,
    17,  // custom_mode (GUIDED = 17)
    MAV_STATE_ACTIVE
);
```

---

## Key Macros Explained

### GCS_MAVLINK_CHAN_METHOD_DEFINITIONS (Lines 87-100)

**Code Generation Macro:**
```cpp
#define GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(subclass_name) \
    subclass_name *chan(const uint8_t ofs) override {      \
        if (ofs >= _num_gcs) return nullptr;               \
        return (subclass_name *)_chan[ofs];                \
    }                                                       \
    const subclass_name *chan(const uint8_t ofs) const override { \
        if (ofs >= _num_gcs) return nullptr;               \
        return (subclass_name *)_chan[ofs];                \
    }
```

**Used in Vehicle-Specific GCS:**
```cpp
class GCS_Copter : public GCS {
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Copter)
};

// Expands to:
class GCS_Copter : public GCS {
    GCS_MAVLINK_Copter *chan(const uint8_t ofs) override {
        if (ofs >= _num_gcs) return nullptr;
        return (GCS_MAVLINK_Copter *)_chan[ofs];
    }
};
```

**Benefits:**
- Avoids code duplication
- Type-safe channel access
- Vehicle-specific features accessible

---

### GCS_SEND_TEXT Macro (Lines 1380-1401)

```cpp
#if !defined(HAL_BUILD_AP_PERIPH)
    #define GCS_SEND_TEXT(severity, format, args...) \
        gcs().send_text(severity, format, ##args)
#else
    // Peripheral boards use CAN instead
    #define GCS_SEND_TEXT(severity, format, args...) \
        can_printf_severity(severity, format, ##args)
#endif
```

**Usage Anywhere in Code:**
```cpp
GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Waypoint %d reached", wp_num);
GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Low battery: %d%%", battery_pct);
GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GPS lost!");
```

---

## Memory Management

### Per-GCS_MAVLINK Instance (~2KB each)

```
├─ _channel_buffer (mavlink_message_t)      300 bytes
├─ _channel_status (mavlink_status_t)       100 bytes
├─ streamRates[NUM_STREAMS] (AP_Int16[])    20 bytes
├─ deferred_message_bucket[10]              200 bytes
├─ pushed_ap_message_ids (Bitmask)          16 bytes
├─ various counters and flags               50 bytes
├─ parameter streaming state                50 bytes
├─ lag_correction (JitterCorrection)        100 bytes
└─ Other members                            ~1200 bytes
                                    Total:  ~2036 bytes
```

### Global GCS Singleton (~1KB)

```
├─ _statustext_queue (7-30 entries)         420-1740 bytes
├─ _chan[] array (8 pointers)               64 bytes
├─ control_sensors (3x uint32_t)            12 bytes
├─ parameters (sysid, options, etc.)        20 bytes
└─ Other                                    ~100 bytes
                                    Total:  ~616-1936 bytes
```

### Total Memory (8 channels)
```
8 × GCS_MAVLINK instances:  ~16 KB
1 × GCS singleton:          ~1.5 KB
                   Total:   ~17.5 KB
```

---

## Testing Framework

### Test: Channel Initialization
```cpp
void test_channel_init() {
    GCS_MAVLINK link(serial_port);
    bool success = link.init(0);  // Channel 0

    assert(success == true);
    assert(link.get_chan() == MAVLINK_COMM_0);
    assert(link.txspace() > 0);
}
```

### Test: Message Queueing
```cpp
void test_message_queue() {
    gcs().send_message(MSG_HEARTBEAT);
    gcs().send_message(MSG_ATTITUDE);
    gcs().send_message(MSG_GPS_RAW);

    // Messages should be queued
    assert(link.has_queued_messages() == true);

    // Process queue
    link.update_send();

    // Verify sent
    assert(get_sent_message_count() == 3);
}
```

### Test: StatusText Thread Safety
```cpp
void test_statustext_threading() {
    // Launch 10 threads, each sending 100 messages
    std::vector<std::thread> threads;
    for (int t = 0; t < 10; t++) {
        threads.push_back(std::thread([t]() {
            for (int i = 0; i < 100; i++) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Thread %d msg %d", t, i);
            }
        }));
    }

    // Wait for all to complete
    for (auto &thread : threads) {
        thread.join();
    }

    // Should have 1000 messages queued (no crashes/corruption)
    assert(gcs().statustext_queue_count() <= 1000);
}
```

---

## Common Patterns

### Pattern 1: Send Message to All Channels
```cpp
// Broadcast
gcs().send_message(MSG_ATTITUDE);

// Or manual:
for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
    GCS_MAVLINK *link = gcs().chan(i);
    if (link && link->is_active()) {
        link->send_message(MSG_ATTITUDE);
    }
}
```

### Pattern 2: Send to Specific Channel
```cpp
GCS_MAVLINK *usb = gcs().chan(0);
if (usb && usb->is_active()) {
    usb->send_text(MAV_SEVERITY_INFO, "Debug: x=%f", x);
}
```

### Pattern 3: Check Sensor Health
```cpp
uint32_t present, enabled, health;
gcs().get_sensor_status_flags(present, enabled, health);

if (!(health & MAV_SYS_STATUS_SENSOR_GPS)) {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS unhealthy!");
}
```

---

## Key Takeaways

1. **GCS_MAVLINK** = per-channel handler, **GCS** = global coordinator
2. **Deferred message system** prevents blocking and manages bandwidth
3. **Stream rates** control telemetry frequency per channel
4. **Thread-safe statustext** allows text from any thread
5. **Sensor status flags** provide health monitoring for GCS display
6. **Payload space checks** prevent buffer overflow
7. **Private channels** isolate companion computer traffic
8. **Message signing** provides security against attacks

This architecture enables **robust, scalable communication** with multiple ground stations and companion computers simultaneously!
