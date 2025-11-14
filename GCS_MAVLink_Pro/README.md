# GCS_MAVLink_Pro Documentation

Complete documentation for understanding, integrating with, and implementing custom GCS MAVLink modules in ArduPilot.

## 📚 Documentation Index

This directory contains comprehensive documentation on the ArduPilot GCS MAVLink system:

### 1. [ARCHITECTURE_DOCUMENTATION.md](ARCHITECTURE_DOCUMENTATION.md)
**Complete architectural analysis** - Read this first!

**Contents**:
- Overview of the GCS MAVLink system design
- Detailed analysis of each component file (GCS.h, GCS_Common.cpp, etc.)
- Class hierarchy and relationships
- Message flow diagrams
- Integration guide for existing projects
- Complete implementation guide for new vehicle types

**When to use**: When you want to understand the full architecture, how components connect, and detailed implementation patterns.

---

### 2. [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
**Quick lookup and common tasks**

**Contents**:
- File location summary
- Class method quick reference
- Common task examples (send text, handle messages, add commands)
- Stream configuration guide
- Important macros
- Debugging tips
- Performance tips
- Quick setup checklist

**When to use**: When you know what you want to do and need a quick code example or reminder.

---

### 3. [EXAMPLE_IMPLEMENTATION.md](EXAMPLE_IMPLEMENTATION.md)
**Complete working example**

**Contents**:
- Full example of a custom "SimpleRover" vehicle implementation
- All required files with complete code
- Detailed comments explaining each part
- Build configuration
- Testing instructions
- Key takeaways

**When to use**: When you want to see a complete, working implementation from start to finish.

---

## 🚀 Quick Start Guide

### For Integration (Using Existing GCS)

If you just want to send/receive custom messages in an existing vehicle:

1. Read **ARCHITECTURE_DOCUMENTATION.md** sections:
   - "Component Analysis" → "GCS_MAVLink_Copter.h/cpp"
   - "Integration Guide"

2. Check **QUICK_REFERENCE.md** for examples:
   - Task 4: Handle Custom MAVLink Message
   - Task 5: Handle Custom MAVLink Command
   - Task 6: Send Custom MAVLink Message Periodically

3. Add your code to the appropriate vehicle files (e.g., `ArduCopter/GCS_MAVLink_Copter.cpp`)

### For New Vehicle Implementation

If you're creating a new vehicle type:

1. Read **ARCHITECTURE_DOCUMENTATION.md**:
   - Full "Overview" section
   - "Class Hierarchy & Relationships"
   - Complete "Implementation Guide"

2. Study **EXAMPLE_IMPLEMENTATION.md**:
   - See the complete SimpleRover example
   - Copy the pattern for your vehicle

3. Use **QUICK_REFERENCE.md** for:
   - Quick lookups during implementation
   - Debugging when things don't work
   - Performance optimization tips

---

## 📖 Document Relationships

```
Start Here
    ↓
┌───────────────────────────────┐
│  ARCHITECTURE_DOCUMENTATION   │ ← Comprehensive overview
│  • What each file does        │   Read this first!
│  • How they connect           │
│  • Design patterns            │
└───────────────┬───────────────┘
                ↓
    ┌───────────┴──────────┐
    ↓                      ↓
┌────────────────┐  ┌─────────────────┐
│ QUICK_REFERENCE│  │ EXAMPLE_IMPL    │
│ • Code snippets│  │ • Working code  │
│ • How-tos      │  │ • Complete demo │
│ • Lookups      │  │ • Build & test  │
└────────────────┘  └─────────────────┘
         ↓                   ↓
         └──────┬────────────┘
                ↓
        Your Implementation
```

---

## 🎯 Common Use Cases

### Use Case 1: "I want to add a custom command to ArduCopter"

**Path**:
1. Quick Reference → Task 5 (Handle Custom Command)
2. Modify `ArduCopter/GCS_MAVLink_Copter.cpp`
3. Add handler in `handle_command_int_packet()`

**Time**: 15 minutes

---

### Use Case 2: "I want to understand how messages are sent"

**Path**:
1. Architecture → "Message Flow" section
2. Architecture → Component Analysis → "GCS_Common.cpp"
3. Example → See `send_wheel_encoder_data()`

**Time**: 30 minutes

---

### Use Case 3: "I'm building a custom ground vehicle"

**Path**:
1. Architecture → Read overview and all component analysis
2. Example → Study complete SimpleRover implementation
3. Copy the pattern, modify for your vehicle
4. Quick Reference → Use as needed during development

**Time**: 2-4 hours for basic implementation

---

### Use Case 4: "I need to debug why my GCS isn't receiving messages"

**Path**:
1. Quick Reference → "Debugging" section
2. Quick Reference → "Common Gotchas"
3. Architecture → "Message Flow" for understanding

**Time**: 30 minutes

---

### Use Case 5: "I want to send periodic telemetry data"

**Path**:
1. Quick Reference → Task 6 (Send Custom Message Periodically)
2. Quick Reference → "Message Streaming Configuration"
3. Example → See `send_wheel_encoder_data()`

**Time**: 20 minutes

---

## 📋 File Overview Summary

### Core ArduPilot Files (Don't Modify)

```
libraries/GCS_MAVLink/
├── GCS_MAVLink.h       - Protocol headers, low-level macros
├── GCS_MAVLink.cpp     - Communication primitives
├── GCS.h               - Base class definitions (GCS, GCS_MAVLINK)
├── GCS.cpp             - GCS global manager implementation
└── GCS_Common.cpp      - Shared message/command handlers (huge!)
```

**Purpose**: Core framework that all vehicles use. Handles 90% of MAVLink protocol automatically.

### Vehicle-Specific Files (Customize These)

```
ArduCopter/             (or your vehicle directory)
├── GCS_Copter.h        - Vehicle GCS manager class
├── GCS_Copter.cpp      - Vehicle GCS manager implementation
├── GCS_MAVLink_Copter.h    - Vehicle channel class
└── GCS_MAVLink_Copter.cpp  - Vehicle channel implementation
```

**Purpose**: Vehicle-specific customizations. Override virtual functions, add custom handlers.

---

## 🔑 Key Concepts

### 1. Two-Level Architecture

```
GCS (Global Manager)
  ├── Manages all channels
  ├── Global operations (send_text to all, etc.)
  └── One instance per vehicle

GCS_MAVLINK (Channel)
  ├── One instance per serial port
  ├── Handles one MAVLink connection
  ├── Message parsing/sending
  └── Multiple instances (typically 2-5)
```

### 2. Inheritance-Based Customization

```
GCS (base) ──────────────> GCS_Copter (vehicle-specific)
GCS_MAVLINK (base) ──────> GCS_MAVLINK_Copter (vehicle-specific)
```

Override virtual methods to customize behavior.

### 3. Message Streams

Messages grouped into streams with configurable rates:
- STREAM_RAW_SENSORS (IMU, baro)
- STREAM_POSITION (GPS, position)
- STREAM_EXTRA1 (attitude)
- STREAM_EXTRA2 (VFR_HUD)
- etc.

Set via SR parameters (SR1_EXTRA1, etc.)

### 4. Virtual Function Pattern

```cpp
// Base class defines interface
class GCS_MAVLINK {
    virtual void send_nav_controller_output() = 0;  // Pure virtual
    virtual void handle_message(...) { }             // Can override
};

// Vehicle implements
class GCS_MAVLINK_Copter : public GCS_MAVLINK {
    void send_nav_controller_output() override { /* copter-specific */ }
    void handle_message(...) override { /* add custom messages */ }
};
```

---

## 🛠️ Development Workflow

### Typical Development Cycle

1. **Understand** (Architecture doc)
   - Read component analysis
   - Understand message flow
   - Review class hierarchy

2. **Plan** (Architecture doc)
   - Decide what to customize
   - Identify which methods to override
   - List required messages/commands

3. **Implement** (Example + Quick Reference)
   - Copy pattern from example
   - Use quick reference for syntax
   - Add custom handlers

4. **Test** (Example)
   - Build and upload
   - Connect GCS
   - Verify messages
   - Test commands

5. **Debug** (Quick Reference)
   - Enable debug output
   - Check channel status
   - Monitor message rates
   - Fix issues

6. **Optimize** (Quick Reference)
   - Review performance tips
   - Adjust stream rates
   - Batch operations

---

## 📊 What Each Document Covers

| Topic | Architecture | Quick Ref | Example |
|-------|-------------|-----------|---------|
| **Conceptual Understanding** | ✓✓✓ | ✓ | ✓ |
| **File-by-File Analysis** | ✓✓✓ | ✓ | - |
| **Class Relationships** | ✓✓✓ | ✓ | ✓ |
| **Message Flow** | ✓✓✓ | ✓ | - |
| **Integration Guide** | ✓✓✓ | - | - |
| **Implementation Guide** | ✓✓✓ | - | ✓✓✓ |
| **Code Examples** | ✓ | ✓✓✓ | ✓✓✓ |
| **Quick Lookup** | - | ✓✓✓ | - |
| **Complete Working Code** | - | - | ✓✓✓ |
| **Debugging** | ✓ | ✓✓✓ | - |
| **Performance** | ✓ | ✓✓✓ | - |
| **Build/Test** | ✓ | - | ✓✓✓ |

✓✓✓ = Primary focus
✓ = Covered but not main focus
\- = Not covered

---

## 🎓 Learning Paths

### Path 1: Beginner (Just Want to Add a Feature)

**Goal**: Add a custom message or command to existing vehicle

**Steps**:
1. Read ARCHITECTURE_DOCUMENTATION intro (10 min)
2. Skim class hierarchy section (5 min)
3. Go to QUICK_REFERENCE → Find your task (5 min)
4. Copy example code, modify for your needs (30 min)
5. Test (15 min)

**Total Time**: ~1 hour

---

### Path 2: Intermediate (Understanding the System)

**Goal**: Understand how GCS system works

**Steps**:
1. Read ARCHITECTURE_DOCUMENTATION fully (1 hour)
2. Study EXAMPLE_IMPLEMENTATION (30 min)
3. Trace message flow in actual code (30 min)
4. Experiment with QUICK_REFERENCE examples (30 min)

**Total Time**: ~2.5 hours

---

### Path 3: Advanced (Implementing New Vehicle)

**Goal**: Create complete GCS implementation for new vehicle type

**Steps**:
1. Read ARCHITECTURE_DOCUMENTATION fully (1 hour)
2. Study EXAMPLE_IMPLEMENTATION in detail (1 hour)
3. Plan your implementation (30 min)
4. Create skeleton classes (30 min)
5. Implement required methods (2 hours)
6. Add custom handlers (1 hour)
7. Test and debug (1 hour)
8. Use QUICK_REFERENCE throughout

**Total Time**: ~7 hours for basic implementation

---

## ⚡ Quick Answers

**Q: Where do I start?**
A: ARCHITECTURE_DOCUMENTATION.md - read the overview section

**Q: I need a code example NOW**
A: QUICK_REFERENCE.md - jump to the "Common Tasks" section

**Q: How do I implement a new vehicle?**
A: EXAMPLE_IMPLEMENTATION.md - follow the SimpleRover pattern

**Q: What does GCS_Common.cpp do?**
A: ARCHITECTURE_DOCUMENTATION.md → Component Analysis → GCS_Common.cpp

**Q: How do I send a message every 100ms?**
A: QUICK_REFERENCE.md → Task 6

**Q: Why aren't my messages being sent?**
A: QUICK_REFERENCE.md → Debugging section

**Q: What methods must I implement?**
A: ARCHITECTURE_DOCUMENTATION.md → Implementation Guide → Required overrides

**Q: How do streams work?**
A: ARCHITECTURE_DOCUMENTATION.md → Message Flow → Stream-Based Messages

---

## 🔧 Tools and Resources

### External Resources

- **MAVLink Protocol**: https://mavlink.io
- **ArduPilot Dev Wiki**: https://ardupilot.org/dev/
- **MAVProxy GCS**: https://ardupilot.org/mavproxy/
- **Mission Planner**: https://ardupilot.org/planner/
- **QGroundControl**: http://qgroundcontrol.com/

### Internal Files

- **MAVLink Definitions**: `modules/mavlink/message_definitions/v1.0/`
- **Common Messages**: `modules/mavlink/message_definitions/v1.0/common.xml`
- **ArduPilot Messages**: `modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`

### Example Vehicles

Study these for real-world implementations:
- **ArduCopter**: Multi-rotor UAVs
- **ArduPlane**: Fixed-wing aircraft
- **ArduRover**: Ground vehicles
- **ArduSub**: Underwater ROVs
- **Blimp**: Lighter-than-air vehicles

---

## 📝 Document Maintenance

These documents were generated on **2025-11-14** based on ArduPilot commit **1ba82e4**.

If you're reading this far in the future:
- Core architecture is stable (rarely changes)
- Specific method signatures may have changed
- New messages/commands may have been added
- Check git history for major GCS refactors

---

## 🤝 Contributing

Found an error or want to improve these docs?

1. Check the actual source files to verify accuracy
2. Update the relevant markdown file
3. Keep examples working and tested
4. Maintain consistent formatting

---

## 💡 Tips for Success

1. **Start Small**: Add one custom message before trying to implement a full vehicle
2. **Copy Existing Patterns**: Look at how ArduCopter does it, copy the pattern
3. **Use the Base Class**: Don't reimplement what GCS_Common already provides
4. **Always Call Base Class**: In overridden methods, call parent implementation
5. **Check for nullptr**: Before using any pointer, verify it's not null
6. **Respect Timing**: GCS checks available time before sending, don't block
7. **Test Incrementally**: Build and test after each small change
8. **Read Compiler Errors**: They often point to missing overrides
9. **Use Debug Output**: Enable GCS_DEBUG_SEND_MESSAGE_TIMINGS during development
10. **Study Working Code**: ArduCopter is the most complete reference

---

## 📞 Getting Help

If you're stuck:

1. **Check these docs first** - Likely answered in QUICK_REFERENCE
2. **Study working code** - Look at ArduCopter implementation
3. **Enable debugging** - Use GCS debug output to see what's happening
4. **ArduPilot Forums** - https://discuss.ardupilot.org
5. **ArduPilot Discord** - Active development community

---

## 🎉 You're Ready!

You now have everything you need to:
- ✓ Understand the GCS MAVLink architecture
- ✓ Integrate custom features into existing vehicles
- ✓ Implement GCS for new vehicle types
- ✓ Debug GCS communication issues
- ✓ Optimize message performance

Pick the document that matches your current goal and dive in!

**Happy Coding! 🚁**
