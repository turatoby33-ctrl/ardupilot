# GCS_MAVLink Study Repository

Complete deep-dive analysis and practical examples for understanding ArduPilot's MAVLink communication system.

## 📁 Repository Structure

```
gcs_mavlink_study/
├── README.md                           ← You are here
├── docs/                               ← Deep-dive documentation
│   ├── 01_GCS_MAVLink_h_DEEP_DIVE.md  ← Low-level protocol layer
│   ├── 02_GCS_h_DEEP_DIVE.md          ← Class architecture
│   └── 03_ROUTING_SYSTEM_DEEP_DIVE.md ← Message routing
├── examples/                           ← Practical code examples
│   ├── 01_message_sending_example.cpp  ← How to send messages
│   └── 02_route_learning_simulation.cpp ← Routing demonstrations
├── custom_messages/                    ← Custom message implementation
│   └── CUSTOM_MAVLINK_MESSAGE_GUIDE.md ← Complete tutorial
├── tests/                              ← Test code (future)
└── diagrams/                           ← Architecture diagrams (future)
```

---

## 🎯 What This Repository Contains

### 1. Deep-Dive Documentation

Comprehensive analysis of each component with:
- **Line-by-line code explanations**
- **Real-world examples**
- **Architecture diagrams**
- **Memory layouts**
- **Performance considerations**
- **Common issues and solutions**

### 2. Practical Examples

Working code examples that demonstrate:
- ✅ Message sending (payload checks, broadcasting, etc.)
- ✅ Routing system simulation
- ✅ Thread-safe operations
- ✅ Parameter streaming
- ✅ Device discovery
- ✅ Performance measurement

### 3. Custom Message Tutorial

Step-by-step guide to adding your own MAVLink messages:
- XML message definition
- Header generation
- ArduPilot integration
- Testing procedures
- Troubleshooting guide

---

## 📚 Learning Path

### For Beginners

**Start Here:**

1. **Read:** `docs/01_GCS_MAVLink_h_DEEP_DIVE.md`
   - Understand the low-level communication layer
   - Learn about channels and buffers
   - See how bytes are sent/received

2. **Read:** `docs/02_GCS_h_DEEP_DIVE.md`
   - Understand the GCS_MAVLINK class
   - Learn about message streams
   - See the deferred message queue

3. **Run:** `examples/01_message_sending_example.cpp`
   - See practical sending patterns
   - Understand payload space checks
   - Learn broadcasting vs targeted sending

### For Intermediate Users

**After completing beginner path:**

4. **Read:** `docs/03_ROUTING_SYSTEM_DEEP_DIVE.md`
   - Understand multi-GCS setups
   - Learn route learning
   - Master message forwarding

5. **Run:** `examples/02_route_learning_simulation.cpp`
   - Simulate complex routing scenarios
   - Understand private channels
   - Practice device discovery

### For Advanced Users

**Ready to customize:**

6. **Follow:** `custom_messages/CUSTOM_MAVLINK_MESSAGE_GUIDE.md`
   - Add your own messages
   - Integrate custom sensors
   - Build specialized features

---

## 🔑 Key Concepts Covered

### Communication Architecture

```
┌──────────────────────────────────┐
│   QGroundControl / GCS           │
│   (sysid=255, compid=190)        │
└────────────┬─────────────────────┘
             │ MAVLink Messages
             │ (UART/USB)
┌────────────▼─────────────────────┐
│   GCS_MAVLink Layer              │
│   ┌──────────────────────────┐   │
│   │ Message Routing          │   │
│   │ - Learn routes           │   │
│   │ - Forward messages       │   │
│   │ - Prevent loops          │   │
│   └──────────────────────────┘   │
│   ┌──────────────────────────┐   │
│   │ Send/Receive Queue       │   │
│   │ - Deferred messages      │   │
│   │ - Stream rates           │   │
│   │ - Bandwidth management   │   │
│   └──────────────────────────┘   │
└──────────────────────────────────┘
             │
┌────────────▼─────────────────────┐
│   ArduPilot Core                 │
│   - Flight control               │
│   - Navigation                   │
│   - Sensor fusion                │
└──────────────────────────────────┘
```

### Message Flow

**Sending:**
```cpp
// Step 1: Queue message
gcs().send_message(MSG_ATTITUDE);

// Step 2: Check payload space
if (HAVE_PAYLOAD_SPACE(chan, ATTITUDE)) {

    // Step 3: Encode MAVLink packet
    mavlink_msg_attitude_pack(...);

    // Step 4: Send to UART
    comm_send_buffer(chan, bytes, length);
}
```

**Receiving:**
```cpp
// Step 1: Byte arrives from UART
uint8_t byte = uart->read();

// Step 2: Parse into message
if (mavlink_parse_char(chan, byte, &msg, &status)) {

    // Step 3: Learn route
    routing.learn_route(link, msg);

    // Step 4: Forward if needed
    routing.check_and_forward(link, msg);

    // Step 5: Handle message
    handle_message(msg);
}
```

### Stream System

```
Stream Rates (SRx_* parameters):
├─ STREAM_RAW_SENSORS    → SR0_RAW_SENS  (IMU, Baro)
├─ STREAM_EXTENDED_STATUS → SR0_EXT_STAT  (Battery, GPS)
├─ STREAM_POSITION       → SR0_POSITION  (GPS data)
├─ STREAM_EXTRA1         → SR0_EXTRA1    (Attitude)
├─ STREAM_EXTRA2         → SR0_EXTRA2    (VFR_HUD)
└─ STREAM_EXTRA3         → SR0_EXTRA3    (AHRS, Wind)

Example: SR0_EXTRA1 = 10  → Attitude sent at 10 Hz on Channel 0
```

---

## 📖 Documentation Guide

### File Naming Convention

- `XX_FILENAME_DEEP_DIVE.md` - In-depth analysis
- `XX_description_example.cpp` - Working code examples
- `*_GUIDE.md` - Step-by-step tutorials

### Documentation Structure

Each deep-dive document follows this pattern:

1. **Overview** - What the file does
2. **Architecture** - How it fits into the system
3. **Core Functions** - Detailed code analysis
4. **Examples** - Real-world usage
5. **Memory Layout** - RAM/Flash usage
6. **Testing** - How to verify behavior
7. **Troubleshooting** - Common issues
8. **Key Takeaways** - Summary

---

## 🚀 Quick Start

### Compile and Run Examples

```bash
cd ardupilot

# Copy example files to a test location
cp gcs_mavlink_study/examples/*.cpp libraries/GCS_MAVLink/examples/

# Or create a test in your vehicle directory
# e.g., ArduCopter/test_gcs.cpp

# Build for your board
./waf configure --board=CubeOrange
./waf copter

# Upload
./waf --targets bin/arducopter --upload
```

### View Documentation

```bash
# Use any Markdown viewer or IDE
code gcs_mavlink_study/docs/

# Or in terminal with pandoc
cd gcs_mavlink_study/docs
pandoc 01_GCS_MAVLink_h_DEEP_DIVE.md | lynx -stdin
```

---

## 🧪 Testing Your Understanding

### Challenge 1: Send a Custom Message

**Task:** Send vehicle's internal temperature every 5 seconds

**Hints:**
1. Use `send_named_float("TEMP", temperature)`
2. Or create custom message (see custom_messages/)
3. Map to STREAM_EXTRA3
4. Set `SR0_EXTRA3 = 0.2` (1/5 = 0.2 Hz)

**Solution:** See `examples/01_message_sending_example.cpp`, Example 5

---

### Challenge 2: Setup Multi-GCS

**Task:** Configure QGC on USB + Mission Planner on Telem1

**Expected Behavior:**
- Both GCS receive telemetry
- Commands from either GCS work
- Parameter changes visible on both
- No message loops

**Verify:**
```bash
# Check routing table
module load messagestats
messagestats
# Should show messages on COMM_0 and COMM_1
```

**Solution:** See `docs/03_ROUTING_SYSTEM_DEEP_DIVE.md`, Complete Flow Example

---

### Challenge 3: Implement Custom Message

**Task:** Add temperature + humidity sensor telemetry

**Steps:**
1. Define in ardupilot.xml (ID 11000)
2. Generate headers
3. Add MSG_CUSTOM_TEMP_HUMIDITY to ap_message.h
4. Implement send function
5. Map to stream
6. Test in QGC

**Solution:** See `custom_messages/CUSTOM_MAVLINK_MESSAGE_GUIDE.md`

---

## 📊 Performance Metrics

### Typical Message Sizes (MAVLink 2)

| Message | Payload | Total | Frequency | Bandwidth |
|---------|---------|-------|-----------|-----------|
| HEARTBEAT | 9 bytes | 23 bytes | 1 Hz | 23 B/s |
| ATTITUDE | 28 bytes | 42 bytes | 10 Hz | 420 B/s |
| GPS_RAW_INT | 30 bytes | 44 bytes | 5 Hz | 220 B/s |
| SYS_STATUS | 31 bytes | 45 bytes | 2 Hz | 90 B/s |
| RC_CHANNELS | 42 bytes | 56 bytes | 5 Hz | 280 B/s |
| **Total** | - | - | - | **~1033 B/s** |

### Link Capacities

| Link Type | Baud | Effective | Typical Usage |
|-----------|------|-----------|---------------|
| USB | 115200 | ~11 KB/s | 10-20% |
| Telem (57k) | 57600 | ~5.7 KB/s | 20-30% |
| Telem (38k) | 38400 | ~3.8 KB/s | 30-40% |
| Radio (57k) | 57600 | ~2-4 KB/s | 40-60% |

**Note:** Keep total bandwidth usage < 60% to avoid buffer overflow

---

## 🔍 Debugging Tips

### Enable MAVLink Debug Output

```cpp
// In MAVLink_routing.cpp
#define ROUTING_DEBUG 1

// Outputs:
// "route learned: sysid=255 compid=190 chan=0"
// "fwd msg 76 from chan 0 on chan 1"
```

### Monitor Message Rates

```bash
# In MAVProxy:
module load messagestats
messagestats

# Output:
# HEARTBEAT: 60 packets/min (1 Hz)
# ATTITUDE: 600 packets/min (10 Hz)
```

### Check Buffer Usage

```cpp
// Add to your code:
hal.console->printf("TX space: %u bytes\n", comm_get_txspace(chan));
hal.console->printf("Out of space count: %u\n", link->out_of_space_count);
```

---

## 📝 Contributing

This is a study repository. Feel free to:

- ✅ Add more examples
- ✅ Create diagrams
- ✅ Write additional documentation
- ✅ Share your custom message implementations
- ✅ Report issues or unclear explanations

### Adding Your Examples

```bash
cd gcs_mavlink_study/examples
cp template.cpp 03_your_example.cpp
# Edit and document your example
```

---

## 📚 Reference Materials

### Official Documentation

- [ArduPilot Developer Wiki](https://ardupilot.org/dev/)
- [MAVLink Protocol](https://mavlink.io/en/)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/)

### Source Code Locations

```
ardupilot/
├── libraries/GCS_MAVLink/          ← Main MAVLink library
│   ├── GCS.h/cpp                   ← GCS class
│   ├── GCS_Common.cpp              ← Message sending
│   ├── GCS_Param.cpp               ← Parameter handling
│   ├── MAVLink_routing.h/cpp       ← Message routing
│   └── include/mavlink/            ← Generated headers
├── ArduCopter/GCS_Mavlink.cpp      ← Copter-specific
├── ArduPlane/GCS_Mavlink.cpp       ← Plane-specific
└── Rover/GCS_Mavlink.cpp           ← Rover-specific
```

### Message Definitions

```
modules/mavlink/message_definitions/v1.0/
├── common.xml          ← Standard MAVLink messages
├── ardupilotmega.xml   ← ArduPilot extensions
└── ardupilot.xml       ← ArduPilot-specific (custom here)
```

---

## 🎓 Learning Outcomes

After studying this repository, you will understand:

✅ **Architecture**
- GCS_MAVLINK class structure
- Multi-channel management
- Stream system design

✅ **Communication Flow**
- How messages are sent
- How messages are received
- Buffer management
- Thread safety

✅ **Routing System**
- Route learning algorithm
- Message forwarding logic
- Private channel isolation
- Device discovery

✅ **Customization**
- Adding custom messages
- Creating custom streams
- Implementing handlers
- Testing and debugging

✅ **Best Practices**
- Payload space checking
- Bandwidth management
- Error handling
- Performance optimization

---

## ❓ FAQ

**Q: Where should I start?**
A: Begin with `docs/01_GCS_MAVLink_h_DEEP_DIVE.md`, then work through the learning path above.

**Q: Can I use these examples in my project?**
A: Yes! All examples are based on ArduPilot's GPL license. Adapt freely.

**Q: How do I test without hardware?**
A: Use SITL (Software In The Loop):
```bash
sim_vehicle.py -v ArduCopter --console --map
```

**Q: My custom message isn't showing in QGC. What's wrong?**
A: Follow the troubleshooting section in `custom_messages/CUSTOM_MAVLINK_MESSAGE_GUIDE.md`

**Q: How much RAM does the GCS system use?**
A: Approximately 17-20 KB for 8 channels. See memory layout sections in docs.

**Q: Can I have more than 8 channels?**
A: Increase `MAVLINK_COMM_NUM_BUFFERS`, but ensure you have enough RAM.

---

## 🏆 Credits

- **ArduPilot Development Team** - For the amazing flight controller software
- **MAVLink Project** - For the robust communication protocol
- **QGroundControl Team** - For the excellent ground control station

---

## 📞 Support

- **ArduPilot Discuss**: https://discuss.ardupilot.org/
- **Discord**: https://ardupilot.org/discord
- **GitHub Issues**: https://github.com/ArduPilot/ardupilot/issues

---

## 📄 License

This study material follows ArduPilot's GPLv3 license.

All code examples are derived from ArduPilot source code and maintain the same license.

---

**Happy Learning! 🚁📡**

Last Updated: 2025-10-26
Version: 1.0
