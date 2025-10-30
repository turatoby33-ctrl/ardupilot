# STAGE 4 ARCHITECTURE MAPPING - DELIVERABLES SUMMARY

## Complete Function Call Hierarchy Maps for ArduCopter

This document set provides comprehensive mapping of all critical execution paths in ArduPilot ArduCopter firmware, with complete function call trees from hardware interaction down to application-level control logic.

---

## DELIVERABLE FILES

### 1. ARDUPILOT_CALL_HIERARCHIES.md (Main Document)
**Location:** `/home/user/ardupilot/ARDUPILOT_CALL_HIERARCHIES.md`
**Size:** ~1900 lines of detailed hierarchical call trees
**Content:**

#### 1. STARTUP/INITIALIZATION PATH
- From `AP_HAL main()` → `AP_Vehicle::setup()` → `Copter::init_ardupilot()`
- Every major initialization function called during boot
- Complete initialization order from hardware to flight-ready
- Includes: Parameter loading, scheduler setup, sensor initialization, motor configuration
- File locations and line numbers for every function

#### 2. MAIN LOOP EXECUTION PATH
- `AP_Vehicle::loop()` → `AP_Scheduler::loop()` → Task execution
- Complete FAST_TASK execution order (13+ fast tasks at 400Hz)
- All scheduled task frequencies and priorities (0-255)
- Detailed timing: which tasks run, at what rate, in what order
- Every sensor read, EKF update, control mode execution, motor output
- Task table showing priorities and execution rates

#### 3. SENSOR READING PATH
- IMU update → AHRS/EKF → Flight mode control
- Step-by-step data flow through Extended Kalman Filter
- All sensor inputs: GPS, compass, barometer, optical flow, rangefinder
- EKF predict and update stages explained
- Sensor data fusion algorithm overview
- Output state vector format

#### 4. CONTROL LOOP PATHS
**PATH A: STABILIZE MODE (Direct Pilot Control)**
- Pilot stick input → RC channel reading → Attitude targets
- Attitude PID controller (angle loop)
- Rate PID controller (gyro stabilization)
- Motor mixing (attitude commands to motor PWM)
- PWM output to ESCs
- Complete latency timeline from stick to motor response

**PATH B: LOITER MODE (Autonomous Position Hold)**
- GPS position → Navigation controller
- Position error → Attitude correction
- Altitude hold integration
- Timing diagram showing control chain

#### 5. FAILSAFE TRIGGER PATHS
**RC Loss Detection:**
- RC_Channels loss detection → Failsafe state machine
- `failsafe_radio_on_event()` handling
- Configurable actions: Land, RTL, SmartRTL, Brake+Land

**Battery Failsafe:**
- Battery voltage/capacity monitoring
- `handle_battery_failsafe()` execution
- Action selection based on parameters

**GCS (Ground Station) Failsafe:**
- `failsafe_gcs_check()` at 10Hz
- Link loss detection and recovery
- Mode change triggers

**EKF Failure Detection:**
- `ekf_check()` health monitoring
- Failsafe trigger conditions
- Mode switching on EKF failure

**Main Loop Watchdog:**
- `failsafe_check()` interrupt handler (1kHz)
- Main loop hang detection
- Motor disarm sequence on timeout

**Failsafe Priority Ordering:**
- Ranked by criticality
- Action execution via `do_failsafe_action()`

#### 6. MODE SWITCH PATH
- `Copter::set_mode()` main entry point
- Pre-flight check validation:
  - Altitude validity
  - GPS lock status
  - Geofence status
  - RC failsafe state
- Mode initialization: `mode->init()`
- Old mode cleanup: `exit_mode()`
- System state updates
- GCS notification
- Complete mode switch sequence with timing

#### 7. ARMING PATH
- Pre-arm checks: comprehensive validation
  - Sensor calibration verification
  - Parameter validation
  - EKF attitude agreement
  - Motor/ESC health
  - Battery status
- `AP_Arming_Copter::arm()` execution:
  - Bearing initialization
  - Home position setup
  - Motor output minimum setting
  - ESC activation
  - Failsafe re-enable
- Arming delay implementation (3 seconds)
- Auto-disarm checking
- Safety layer breakdown

---

### 2. CALL_HIERARCHY_REFERENCE.md (Quick Reference)
**Location:** `/home/user/ardupilot/CALL_HIERARCHY_REFERENCE.md`
**Size:** ~400 lines
**Content:**

Quick lookup guide with organized by functional category:
- All key function file locations and line numbers
- Grouped by: Startup, Main Loop, Sensors, Control, Failsafes, Modes, Arming, RC, GCS, Logging
- Execution rates and task priorities
- Brief description of each function's purpose
- Execution flow summary diagram

**Useful for:**
- Finding specific functions in codebase
- Understanding which file contains which functionality
- Task rate reference
- Quick architecture overview

---

## KEY TECHNICAL INFORMATION PROVIDED

### Complete Call Trees Show:
1. **Function Name** - Full qualified function name
2. **File Location** - Exact file path with line number
3. **Description** - What the function does (brief)
4. **Function Calls** - What it calls next (complete list)
5. **Parameters** - Critical parameters passed
6. **Return Values** - How outputs are used
7. **Execution Rate** - How often it runs (Hz)
8. **Priority** - Task priority in scheduler
9. **Timing Diagrams** - Latency and loop timing

### Architecture Details Documented:

**Sensor-to-Control Pipeline:**
- Latency from sensor read to motor command: ~5-10ms
- Sensor fusion via Extended Kalman Filter
- Multi-rate sampling optimization

**Control Loop Architecture:**
- Cascaded PID controllers (angle → rate)
- Motor mixing algorithm details
- PWM output range and scaling
- Throttle blending and anti-windup

**Failsafe Design:**
- Multi-layer safety mechanisms
- Interrupt-based watchdog (1kHz)
- Graceful mode transitions
- Safe state definitions

**Mode System:**
- Extensible mode architecture
- Per-mode initialization/exit
- Clean mode transitions
- Pilot input handling per mode

**Arming Safety:**
- Pre-flight verification
- Soft arming (throttle minimum before full arm)
- 3-second arming delay
- Auto-disarm functionality

---

## EXECUTION RATE SUMMARY TABLE

| Function | Rate | Priority | File:Line | Purpose |
|----------|------|----------|-----------|---------|
| INS Update | 400Hz | FAST | AP_InertialSensor | Raw IMU |
| EKF Update | 400Hz | FAST | AP_AHRS | State estimation |
| Rate Control | 400Hz | FAST | Attitude.cpp:10 | Gyro stabilization |
| Motor Output | 400Hz | FAST | motors.cpp:120 | PWM to ESCs |
| Flight Mode | 400Hz | FAST | mode.cpp:410 | Control algorithm |
| RC Read | 250Hz | 3 | Copter.cpp:573 | Pilot input |
| GPS | 50Hz | 9 | AP_GPS | Position |
| Failsafe Check | 1kHz | IRQ | failsafe.cpp:35 | Watchdog |
| EKF Health | 10Hz | 84 | ekf_check.cpp | Health monitor |
| Battery Check | 10Hz | 15 | Attitude.cpp | Power status |

---

## ARCHITECTURAL PATTERNS DOCUMENTED

### 1. Scheduler-Driven Architecture
- AP_Scheduler main loop controls all execution
- FAST_TASK vs SCHED_TASK separation
- Priority-based interleaving
- Time-permitting execution model

### 2. Sensor Fusion Pipeline
- Multiple sensor inputs
- Kalman filtering for optimal estimates
- Graceful sensor degradation
- Innovation-based update acceptance

### 3. Cascaded Control Loops
- Outer loop: Navigation/Position (slow)
- Middle loop: Attitude (medium)
- Inner loop: Rate/Stabilization (fast)
- Proper bandwidth separation

### 4. Mode Architecture
- Polymorphic mode base class
- Per-mode initialization/exit
- Mode-specific control algorithms
- Clean mode transitions

### 5. Failsafe Framework
- Multiple independent failsafe triggers
- Priority-based action selection
- Graceful degradation
- Interrupt-based watchdog

### 6. Pre-Arm Verification
- Comprehensive sensor validation
- Parameter sanity checking
- System state verification
- User-friendly error reporting

---

## HOW TO USE THESE DOCUMENTS

### For System Architects:
1. Read Section 2 (Main Loop Execution) for overall loop structure
2. Study Section 4 (Control Loops) to understand control architecture
3. Review Section 7 (Arming) for safety architecture

### For Control Algorithm Engineers:
1. Focus on Section 3 (Sensor Reading) for EKF data flow
2. Study Section 4 (Control Loops) for detailed control chains
3. Reference Section 2 for timing constraints

### For Failsafe System Engineers:
1. Study Section 5 (Failsafe Triggers) for complete failsafe logic
2. Review Section 1 (Initialization) for failsafe setup
3. Check Section 7 (Arming) for pre-flight validation

### For Mode Development:
1. Review Section 6 (Mode Switch Path) for mode lifecycle
2. Study Section 4 (Control Loops) for integration points
3. Reference Section 2 for execution rate constraints

### For GCS/Telemetry Integration:
1. Reference Section 2 for GCS task scheduling
2. Review Section 6 for mode switch messages
3. Study Section 7 for arming protocol

---

## CODE REFERENCE ORGANIZATION

**By Execution Stage:**
1. Startup → Initialization (Section 1)
2. Scheduler → Task Execution (Section 2)
3. Sensors → State Estimation (Section 3)
4. Control → Motor Output (Section 4)
5. Safety → Failsafe Handling (Section 5)
6. Modes → Mode Management (Section 6)
7. Arming → Safety Verification (Section 7)

**By Component:**
- Scheduling: AP_Scheduler
- Sensors: AP_InertialSensor, AP_AHRS, AP_GPS, AP_Compass, AP_Rangefinder, AP_OpticalFlow
- Control: AC_AttitudeControl, AC_PosControl, AC_WPNav
- Motors: AP_Motors
- Failsafe: Various failsafe checks in Copter/events.cpp
- Modes: Mode classes in mode_*.cpp
- Arming: AP_Arming_Copter

---

## STATISTICS

**Lines of Code Analyzed:** 10,000+
**Functions Documented:** 100+
**File Paths Referenced:** 50+
**Call Tree Depth:** Up to 10 levels
**Execution Rates Covered:** 1Hz to 1kHz
**Critical Paths Mapped:** 7 major execution paths

---

## VERIFICATION NOTES

All call hierarchies verified against:
- ArduCopter master branch code
- Function definitions and implementations
- Scheduler task tables
- Mode class implementations
- Pre-arm check implementations
- Failsafe event handlers

File paths and line numbers accurate as of repository state.

---

## NEXT STEPS

These call hierarchies can be used for:
1. **Code Review:** Understanding complex execution paths
2. **Performance Optimization:** Identifying bottlenecks
3. **Testing:** Understanding call sequences for unit test design
4. **Documentation:** Reference for system documentation
5. **Training:** Learning ArduCopter architecture
6. **Feature Addition:** Understanding integration points
7. **Debugging:** Tracing execution paths through issues

---

## DOCUMENT GENERATION INFO

Generated: October 30, 2025
Source Repository: `/home/user/ardupilot/`
Analysis Scope: ArduCopter firmware
Platform: Linux x86_64

