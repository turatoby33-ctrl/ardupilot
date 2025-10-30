# Complete Guide: Implementing Custom MAVLink Messages in ArduPilot

## Table of Contents
1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Step-by-Step Implementation](#step-by-step-implementation)
4. [Example: Custom Sensor Data Message](#example-custom-sensor-data-message)
5. [Testing Your Custom Message](#testing-your-custom-message)
6. [Troubleshooting](#troubleshooting)

---

## Overview

### What is a Custom MAVLink Message?

MAVLink messages are the **communication packets** sent between ArduPilot and ground control stations. While ArduPilot supports 300+ standard messages, you might need a custom message for:

- **Custom sensor data** (e.g., special payload sensors)
- **Vehicle-specific telemetry** (e.g., agricultural sprayer status)
- **Custom commands** (e.g., triggering specialized actuators)
- **Debug information** (e.g., algorithm internal states)

### Architecture

```
┌─────────────────────────────────────────┐
│  1. Define Message in XML               │  ← MAVLink message definition
│     (ardupilot.xml)                     │
├─────────────────────────────────────────┤
│  2. Generate C Headers                  │  ← pymavlink generates code
│     (mavlink_msg_custom.h)              │
├─────────────────────────────────────────┤
│  3. Add to ap_message enum              │  ← ArduPilot message catalog
│     (ap_message.h)                      │
├─────────────────────────────────────────┤
│  4. Implement Sending Function          │  ← ArduPilot sends the message
│     (GCS_Common.cpp or vehicle-specific)│
├─────────────────────────────────────────┤
│  5. Map to Stream                       │  ← Control send rate
│     (vehicle GCS_Mavlink.cpp)           │
├─────────────────────────────────────────┤
│  6. (Optional) Implement Handler        │  ← Receive from GCS
│     (GCS_Common.cpp or vehicle-specific)│
└─────────────────────────────────────────┘
```

---

## Prerequisites

### Required Tools

1. **Python 3** with pymavlink:
   ```bash
   pip3 install pymavlink
   ```

2. **ArduPilot source code**:
   ```bash
   git clone https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   git submodule update --init --recursive
   ```

3. **Text editor** (VSCode, Sublime, etc.)

4. **Build environment** (already set up if you can compile ArduPilot)

---

## Step-by-Step Implementation

### Step 1: Define Your Message in XML

**Location:** `modules/mavlink/message_definitions/v1.0/ardupilot.xml`

**Example:** Adding a custom sensor data message

```xml
<!-- Open ardupilot.xml and find the <messages> section -->
<messages>
    <!-- Add your custom message before the closing </messages> tag -->

    <!-- Custom sensor telemetry message -->
    <message id="11000" name="CUSTOM_SENSOR_DATA">
        <description>Custom sensor data from specialized payload</description>
        <field type="uint64_t" name="time_usec" units="us">
            Timestamp (UNIX Epoch time or time since system boot)
        </field>
        <field type="float" name="temperature" units="degC">
            Temperature reading from sensor
        </field>
        <field type="float" name="pressure" units="Pa">
            Pressure reading from sensor
        </field>
        <field type="float" name="humidity" units="%">
            Relative humidity percentage
        </field>
        <field type="uint16_t" name="sensor_status">
            Sensor status bitmask: bit 0: valid, bit 1: calibrated
        </field>
        <field type="uint8_t" name="sensor_id">
            Sensor ID (0-255)
        </field>
    </message>

</messages>
```

**Important Notes:**
- **Message ID**: Use range 11000-11999 for custom messages (avoids conflicts)
- **Field Types**: uint8_t, uint16_t, uint32_t, uint64_t, int8_t, int16_t, int32_t, int64_t, float, double, char[N]
- **Units**: Always specify units for clarity
- **Name Convention**: UPPERCASE_WITH_UNDERSCORES

---

### Step 2: Generate MAVLink Headers

```bash
cd ardupilot

# This regenerates all MAVLink headers including your new message
./modules/mavlink/pymavlink/tools/mavgen.py \
    --lang=C \
    --wire-protocol=2.0 \
    --output=libraries/GCS_MAVLink/include/mavlink/v2.0 \
    modules/mavlink/message_definitions/v1.0/ardupilot.xml
```

**What This Creates:**
```
libraries/GCS_MAVLink/include/mavlink/v2.0/ardupilotmega/
├── mavlink_msg_custom_sensor_data.h  ← YOUR NEW MESSAGE!
├── testsuite.h                        ← Updated test suite
└── version.h                          ← Version updated
```

**Verify Generation:**
```bash
ls -l libraries/GCS_MAVLink/include/mavlink/v2.0/ardupilotmega/mavlink_msg_custom_sensor_data.h
```

---

### Step 3: Add Message to ArduPilot's Message Enum

**Location:** `libraries/GCS_MAVLink/ap_message.h`

```cpp
enum ap_message : uint8_t {
    MSG_HEARTBEAT                      =   0,
    // ... existing messages ...
    MSG_AVAILABLE_MODES_MONITOR        =  99,
#if AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
    MSG_FLIGHT_INFORMATION             = 100,
#endif

    // ===== ADD YOUR CUSTOM MESSAGE HERE =====
    MSG_CUSTOM_SENSOR_DATA             = 101,  // ← ADD THIS LINE

    MSG_LAST // MSG_LAST must be the last entry in this enum
};
```

**Important:** Assign next available number, keep MSG_LAST at the end!

---

### Step 4: Implement the Sending Function

**Location:** `libraries/GCS_MAVLink/GCS_Common.cpp` (or vehicle-specific file)

```cpp
// Add this function to GCS_Common.cpp

#if AP_CUSTOM_SENSOR_ENABLED  // Add feature flag
void GCS_MAVLINK::send_custom_sensor_data()
{
    // Check if we have space in the transmit buffer
    CHECK_PAYLOAD_SIZE(CUSTOM_SENSOR_DATA);

    // Get sensor data (replace with your actual sensor reading)
    const AP_CustomSensor *sensor = AP_CustomSensor::get_singleton();
    if (sensor == nullptr || !sensor->healthy()) {
        return;  // Sensor not available
    }

    float temperature, pressure, humidity;
    uint16_t status;
    uint8_t sensor_id;

    // Read sensor data
    if (!sensor->get_data(temperature, pressure, humidity, status, sensor_id)) {
        return;  // Failed to read
    }

    // Send the MAVLink message
    mavlink_msg_custom_sensor_data_send(
        chan,                        // Channel to send on
        AP_HAL::micros64(),          // Timestamp
        temperature,                 // Temperature
        pressure,                    // Pressure
        humidity,                    // Humidity
        status,                      // Status bitmask
        sensor_id                    // Sensor ID
    );
}
#endif  // AP_CUSTOM_SENSOR_ENABLED
```

---

### Step 5: Add to try_send_message()

**Location:** `libraries/GCS_MAVLink/GCS_Common.cpp`

Find the `try_send_message()` function and add your message case:

```cpp
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    switch (id) {
        // ... existing cases ...

#if AP_CUSTOM_SENSOR_ENABLED
        case MSG_CUSTOM_SENSOR_DATA:
            send_custom_sensor_data();
            break;
#endif

        // ... more cases ...
    }
    return true;
}
```

---

### Step 6: Map to a Stream (Optional but Recommended)

**Location:** Vehicle-specific file (e.g., `ArduCopter/GCS_Mavlink.cpp`)

```cpp
// Find the stream_entries definition
const GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    // ... existing streams ...

    // Add to STREAM_EXTRA3 (or create custom stream)
    MAV_STREAM_ENTRY(EXTRA3),
    // ... other entries ...
};

// Then in the vehicle's GCS class, define what goes in EXTRA3:
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_CUSTOM_SENSOR_DATA,  // ← ADD YOUR MESSAGE HERE
};
```

**Alternative: Create Custom Stream**

If you want dedicated rate control:

```cpp
// In GCS.h, add new stream:
enum streams : uint8_t {
    STREAM_RAW_SENSORS,
    STREAM_EXTENDED_STATUS,
    // ... existing streams ...
    STREAM_CUSTOM_SENSORS,  // ← ADD THIS
    NUM_STREAMS
};

// In vehicle GCS_Mavlink.cpp:
static const ap_message STREAM_CUSTOM_SENSORS_msgs[] = {
    MSG_CUSTOM_SENSOR_DATA,
};

const GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    // ... existing entries ...
    MAV_STREAM_ENTRY(CUSTOM_SENSORS),
};
```

This creates a new parameter: `SRx_CUSTOM_SENSORS` for rate control!

---

### Step 7: (Optional) Implement Message Reception Handler

If you want to **receive** this message from GCS:

**Location:** `libraries/GCS_MAVLink/GCS_Common.cpp`

```cpp
void GCS_MAVLINK::handle_custom_sensor_data(const mavlink_message_t &msg)
{
    mavlink_custom_sensor_data_t packet;
    mavlink_msg_custom_sensor_data_decode(&msg, &packet);

    // Process the received data
    hal.console->printf("Received custom sensor data:\n");
    hal.console->printf("  Temperature: %.2f C\n", packet.temperature);
    hal.console->printf("  Pressure: %.2f Pa\n", packet.pressure);
    hal.console->printf("  Humidity: %.2f %%\n", packet.humidity);
    hal.console->printf("  Status: 0x%04X\n", packet.sensor_status);

    // Forward to sensor driver or process as needed
    AP_CustomSensor *sensor = AP_CustomSensor::get_singleton();
    if (sensor != nullptr) {
        sensor->set_calibration_data(packet.temperature, packet.pressure);
    }
}
```

**Add to handle_message():**

```cpp
void GCS_MAVLINK::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        // ... existing cases ...

        case MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA:
            handle_custom_sensor_data(msg);
            break;

        // ... more cases ...
    }
}
```

---

## Example: Complete Custom Sensor Data Message

### File Structure

```
ardupilot/
├── modules/mavlink/message_definitions/v1.0/ardupilot.xml  [Modified]
├── libraries/
│   ├── GCS_MAVLink/
│   │   ├── ap_message.h                                    [Modified]
│   │   ├── GCS_Common.cpp                                  [Modified]
│   │   └── include/mavlink/v2.0/ardupilotmega/
│   │       └── mavlink_msg_custom_sensor_data.h            [Generated]
│   └── AP_CustomSensor/                                    [New Library]
│       ├── AP_CustomSensor.h
│       └── AP_CustomSensor.cpp
└── ArduCopter/
    └── GCS_Mavlink.cpp                                     [Modified]
```

### Complete Implementation Files

#### 1. XML Definition (ardupilot.xml)

```xml
<message id="11000" name="CUSTOM_SENSOR_DATA">
    <description>Custom environmental sensor telemetry</description>
    <field type="uint64_t" name="time_usec" units="us">Timestamp</field>
    <field type="float" name="temperature" units="degC">Temperature</field>
    <field type="float" name="pressure" units="Pa">Pressure</field>
    <field type="float" name="humidity" units="%">Humidity</field>
    <field type="uint16_t" name="sensor_status">Status bitmask</field>
    <field type="uint8_t" name="sensor_id">Sensor ID</field>
</message>
```

#### 2. Sender Function (GCS_Common.cpp)

```cpp
void GCS_MAVLINK::send_custom_sensor_data()
{
    CHECK_PAYLOAD_SIZE(CUSTOM_SENSOR_DATA);

    // Example: read from custom sensor library
    const AP_CustomSensor *sensor = AP_CustomSensor::get_singleton();
    if (sensor == nullptr) {
        return;
    }

    mavlink_msg_custom_sensor_data_send(
        chan,
        AP_HAL::micros64(),
        sensor->get_temperature(),
        sensor->get_pressure(),
        sensor->get_humidity(),
        sensor->get_status(),
        sensor->get_id()
    );
}
```

#### 3. Stream Mapping (ArduCopter/GCS_Mavlink.cpp)

```cpp
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_CUSTOM_SENSOR_DATA,  // Added
};
```

---

## Testing Your Custom Message

### Method 1: MAVLink Inspector in QGroundControl

1. **Build and upload** modified firmware
2. **Connect QGC** to vehicle
3. **Open MAVLink Inspector**:
   - Widgets → MAVLink Inspector
4. **Find your message**:
   - Should appear as "CUSTOM_SENSOR_DATA (11000)"
5. **Verify fields**:
   - Check temperature, pressure, humidity values
   - Confirm update rate

### Method 2: mavproxy

```bash
# Connect to vehicle
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

# Set stream rate
MAVPARAM> set SR0_EXTRA3 10  # 10 Hz

# Monitor messages
MAVPARAM> module load messagestats
MAVPARAM> messagestats

# Should see CUSTOM_SENSOR_DATA in the list with packet count
```

### Method 3: Python Script

```python
#!/usr/bin/env python3
from pymavlink import mavutil

# Connect to vehicle
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()
print("Connected!")

# Request message stream
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
    10,  # 10 Hz
    1    # Start
)

# Receive messages
while True:
    msg = master.recv_match(type='CUSTOM_SENSOR_DATA', blocking=True)
    if msg:
        print(f"Temperature: {msg.temperature:.2f} C")
        print(f"Pressure: {msg.pressure:.2f} Pa")
        print(f"Humidity: {msg.humidity:.2f} %")
        print(f"Status: 0x{msg.sensor_status:04X}")
        print("---")
```

### Method 4: SITL Testing

```bash
# Start SITL
sim_vehicle.py -v ArduCopter --console --map

# In MAVProxy console:
STABILIZE> module load messagestats
STABILIZE> set SR0_EXTRA3 5
STABILIZE> messagestats

# Should show CUSTOM_SENSOR_DATA being sent
```

---

## Advanced Topics

### Adding Parameters for Your Message

```cpp
// In your custom sensor library (AP_CustomSensor.cpp)
const AP_Param::GroupInfo AP_CustomSensor::var_info[] = {
    // Enable/disable custom sensor
    AP_GROUPINFO("_ENABLE", 1, AP_CustomSensor, _enabled, 0),

    // Sensor update rate (Hz)
    AP_GROUPINFO("_RATE", 2, AP_CustomSensor, _rate_hz, 10),

    // Sensor ID
    AP_GROUPINFO("_ID", 3, AP_CustomSensor, _sensor_id, 0),

    AP_GROUPEND
};
```

This creates parameters:
- `CSENS_ENABLE`
- `CSENS_RATE`
- `CSENS_ID`

### Conditional Compilation

```cpp
// In AP_CustomSensor_config.h
#ifndef HAL_CUSTOM_SENSOR_ENABLED
#define HAL_CUSTOM_SENSOR_ENABLED 1
#endif

// In GCS_Common.cpp
#if HAL_CUSTOM_SENSOR_ENABLED
void GCS_MAVLINK::send_custom_sensor_data() {
    // Implementation
}
#endif
```

### Message Extensions (MAVLink 2)

```xml
<!-- In ardupilot.xml -->
<message id="11000" name="CUSTOM_SENSOR_DATA">
    <description>Custom sensor data</description>
    <!-- Original fields -->
    <field type="float" name="temperature" units="degC">Temperature</field>

    <!-- Extension fields (only in MAVLink 2) -->
    <extensions/>
    <field type="float" name="temperature_raw" units="degC">
        Raw temperature before filtering
    </field>
</message>
```

Extension fields:
- Only sent if MAVLink 2 is used
- Backwards compatible with MAVLink 1
- Save bandwidth when not needed

---

## Troubleshooting

### Issue 1: "Message ID Already in Use"

**Error:**
```
Error: Message ID 11000 already defined
```

**Solution:**
Choose a different ID in the range 11000-11999.

### Issue 2: "Message Not Appearing in QGC"

**Checklist:**
1. ✓ Message added to ap_message.h?
2. ✓ send_XXX() function implemented?
3. ✓ Added to try_send_message() switch?
4. ✓ Mapped to a stream?
5. ✓ Stream rate > 0? (Check SRx parameters)
6. ✓ Firmware actually rebuilt and uploaded?

**Debug:**
```cpp
// Add debug output in send function
void GCS_MAVLINK::send_custom_sensor_data() {
    hal.console->printf("Sending custom sensor data\n");
    // ... rest of function
}
```

### Issue 3: "Compilation Errors"

**Common Causes:**
- Forgot to regenerate MAVLink headers
- Typo in message name
- Feature flag not defined

**Fix:**
```bash
# Regenerate headers
cd ardupilot
./modules/mavlink/pymavlink/tools/mavgen.py --lang=C --wire-protocol=2.0 \
    --output=libraries/GCS_MAVLink/include/mavlink/v2.0 \
    modules/mavlink/message_definitions/v1.0/ardupilot.xml

# Clean build
./waf clean
./waf configure --board=<your_board>
./waf copter
```

### Issue 4: "Message Sent Too Fast/Slow"

**Solution:**
Adjust stream rate parameter:

```
SR0_EXTRA3 = 10   # 10 Hz
SR1_EXTRA3 = 2    # 2 Hz on telemetry radio
```

Or create dedicated stream (see Step 6 alternative).

---

## Best Practices

### 1. Use Appropriate Data Types

```cpp
// Good: Efficient packing
uint8_t  sensor_id;       // 1 byte
uint16_t sensor_status;   // 2 bytes
float    temperature;     // 4 bytes

// Avoid: Wasteful
uint64_t sensor_id;       // 8 bytes for 0-255?
double   temperature;     // 8 bytes vs 4 bytes float
```

### 2. Include Timestamps

Always include a timestamp field for data correlation:

```xml
<field type="uint64_t" name="time_usec" units="us">Timestamp</field>
```

### 3. Use Status Bitmasks

```cpp
// Define bits
#define SENSOR_STATUS_VALID      (1<<0)
#define SENSOR_STATUS_CALIBRATED (1<<1)
#define SENSOR_STATUS_ERROR      (1<<7)

// Set status
uint16_t status = 0;
if (sensor_valid) status |= SENSOR_STATUS_VALID;
if (calibrated) status |= SENSOR_STATUS_CALIBRATED;
```

### 4. Document Units

Always specify units in XML and comments:

```xml
<field type="float" name="pressure" units="Pa">Pressure</field>  <!-- NOT hPa or bar! -->
```

### 5. Test Bandwidth Impact

```python
# Calculate bytes per second
message_size = 34  # bytes (MAVLink 2 overhead + payload)
rate_hz = 10       # messages per second
bandwidth = message_size * rate_hz  # = 340 bytes/sec

# Check against link capacity
# Typical 57600 baud ≈ 5760 bytes/sec effective
# Your message uses: 340/5760 = 5.9% of bandwidth
```

---

## Summary Checklist

- [ ] Define message in ardupilot.xml
- [ ] Generate MAVLink headers
- [ ] Add to ap_message enum
- [ ] Implement send_XXX() function
- [ ] Add to try_send_message() switch
- [ ] Map to stream or create new stream
- [ ] (Optional) Implement receiver handler
- [ ] Test in SITL
- [ ] Test with real hardware
- [ ] Verify in QGC MAVLink Inspector
- [ ] Document your message
- [ ] Check bandwidth impact
- [ ] Commit and push changes

---

## Next Steps

1. **Start Simple**: Begin with a basic message (2-3 fields)
2. **Test Thoroughly**: Use SITL before real hardware
3. **Iterate**: Add fields as needed
4. **Share**: Consider contributing useful messages to ArduPilot

## Resources

- **MAVLink Documentation**: https://mavlink.io/en/
- **ArduPilot Wiki**: https://ardupilot.org/dev/
- **pymavlink**: https://github.com/ArduPilot/pymavlink
- **Message Definitions**: https://github.com/ArduPilot/mavlink/tree/master/message_definitions/v1.0

---

**Congratulations!** You now have a complete guide to implementing custom MAVLink messages in ArduPilot. This opens up endless possibilities for custom telemetry, commands, and vehicle-specific features!
