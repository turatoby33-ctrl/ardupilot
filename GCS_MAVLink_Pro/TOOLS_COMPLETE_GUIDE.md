# ArduPilot Tools Directory - Complete Guide

Complete guide to the ArduPilot Tools directory and all its utilities for vehicle development, testing, debugging, and deployment.

---

## Table of Contents

1. [Overview](#overview)
2. [Critical Tools for Development](#critical-tools)
3. [All 32 Subdirectories](#all-subdirectories)
4. [Complete Development Workflow](#development-workflow)
5. [Testing Procedures](#testing-procedures)
6. [Debugging Guide](#debugging-guide)
7. [Common Tasks](#common-tasks)
8. [Tools Quick Reference](#quick-reference)

---

## Overview

The `Tools/` directory contains **essential development infrastructure** for ArduPilot:

- **32 subdirectories** with 100+ utilities
- **Testing framework** (SITL, automated tests)
- **Build scripts** (release binaries, bootloaders)
- **Debug tools** (GDB, crash analysis, log replay)
- **Development utilities** (formatters, analyzers, scripts)
- **Integration tools** (ROS2, Simulink, cameras)

**Directory location:** `/home/user/ardupilot/Tools/`

---

## Critical Tools for Development

### 🔥 Most Important (Use Daily)

#### 1. **autotest/sim_vehicle.py** - SITL Launcher

**Purpose:** Launch Software-In-The-Loop simulation for testing

```bash
# Launch Copter SITL with MAVProxy
cd /home/user/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter

# Launch Rover SITL
./Tools/autotest/sim_vehicle.py -v Rover

# With console and map
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# Custom home location
./Tools/autotest/sim_vehicle.py -v ArduCopter --home=LAT,LON,ALT,YAW

# Custom frame
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad

# Add parameter file
./Tools/autotest/sim_vehicle.py -v ArduCopter \
  --add-param-file=Tools/Frame_params/copter-optflow.param

# Wipe EEPROM and start fresh
./Tools/autotest/sim_vehicle.py -v ArduCopter -w

# Enable DroneCAN peripherals
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad-can
```

#### 2. **autotest/autotest.py** - Automated Testing

**Purpose:** Run complete test suites

```bash
# Run all Copter tests
./Tools/autotest/autotest.py test.Copter

# Run specific test
./Tools/autotest/autotest.py test.Copter.ArmFeatures

# Build and test
./Tools/autotest/autotest.py build.Copter test.Copter

# Rover tests
./Tools/autotest/autotest.py test.Rover

# All vehicles
./Tools/autotest/autotest.py test.All
```

#### 3. **scripts/extract_features.py** - Feature Analysis

**Purpose:** See what features are compiled into a binary

```bash
# Check features in binary
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj

# Output shows:
# - Enabled features (GPS, Compass, RangeFinder, etc.)
# - Disabled features
# - Flash usage breakdown
```

#### 4. **debug/crash_debugger.py** - Crash Analysis

**Purpose:** Analyze crash dumps from flight controller

```bash
# Get crash via serial
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0 --dump-filename crash.txt

# Analyze crash dump from file
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --dump-debug --dump-filein crash_dump.bin

# Output shows:
# - Stack trace
# - Register values
# - Line numbers in source
```

#### 5. **Replay/** - Log Replay

**Purpose:** Replay logs through EKF to test navigation changes

```bash
# Build Replay tool
./waf configure --board linux
./waf replay

# Replay a log
./build/linux/tools/Replay logfile.BIN

# Check logs for regressions
cd Tools/Replay
./CheckLogs.py --logdir testlogs
```

---

## All 32 Subdirectories

### 1. autotest/ - 🔥 CRITICAL Testing Framework

**Purpose:** Complete SITL and automated testing system

**Structure:**
```
autotest/
├── sim_vehicle.py           # Launch SITL (MOST IMPORTANT)
├── autotest.py              # Run automated tests
├── arducopter.py            # Copter tests (570KB, 100+ tests)
├── arduplane.py             # Plane tests (302KB)
├── rover.py                 # Rover tests (294KB)
├── ardusub.py               # Sub tests
├── helicopter.py            # Helicopter tests
├── quadplane.py             # QuadPlane tests
├── vehicle_test_suite.py    # Base test framework (620KB)
│
├── ArduCopter_Tests/        # 36 copter missions
├── ArduPlane_Tests/         # 60 plane missions
├── ArduRover_Tests/         # 21 rover missions
├── ArduSub_Tests/           # 7 sub missions
├── Generic_Missions/        # Reusable missions
├── default_params/          # 80+ param files
├── aircraft/                # Saved SITL state
├── logger_metadata/         # Log metadata
├── pysim/                   # Python sim libraries
└── models/                  # Vehicle models
```

**Common Commands:**

```bash
# Basic SITL launch
./Tools/autotest/sim_vehicle.py -v ArduCopter

# With specific frame
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad
./Tools/autotest/sim_vehicle.py -v ArduCopter -f octa
./Tools/autotest/sim_vehicle.py -v ArduCopter -f X8
./Tools/autotest/sim_vehicle.py -v Rover -f rover
./Tools/autotest/sim_vehicle.py -v Rover -f sailboat

# Enable features
./Tools/autotest/sim_vehicle.py -v ArduCopter --osdmsp  # OSD
./Tools/autotest/sim_vehicle.py -v ArduCopter --rgbled  # RGB LED

# Debug mode
./Tools/autotest/sim_vehicle.py -v ArduCopter -D

# Valgrind (memory checking)
./Tools/autotest/sim_vehicle.py -v ArduCopter -V

# Coverage analysis
./Tools/autotest/sim_vehicle.py -v ArduCopter --coverage

# Run specific test
cd Tools/autotest
./arducopter.py test.ArmFeatures
```

**Available Frames:**
- **Copter:** quad, X, octa, octa-quad, X8, dodeca-hexa, tri, y6, heli, heli-dual
- **Plane:** plane, plane-vtail, plane-tailsitter, plane-elevon, quadplane
- **Rover:** rover, rover-skid, sailboat, balancebot
- **Sub:** vectored

**When to Use:**
- Daily development testing
- Before every commit
- Feature development
- Regression testing
- Learning vehicle behavior

---

### 2. scripts/ - 🔥 Development Scripts Hub

**Purpose:** 100+ scripts for building, analyzing, and maintaining

**Categories:**

**Build Scripts:**
```bash
# Build all release binaries
./Tools/scripts/build_binaries.py

# Build specific vehicle
./Tools/scripts/build_binaries.py --board CubeOrange --vehicle ArduCopter

# Build bootloaders
./Tools/scripts/build_bootloaders.py '*'
./Tools/scripts/build_bootloaders.py 'Cube*'

# Build examples
./Tools/scripts/build_examples.py
```

**Analysis Scripts:**
```bash
# Extract features from binary
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj

# Decode device ID from log
./Tools/scripts/decode_devid.py 0x1234567

# Decode watchdog crash
./Tools/scripts/decode_watchdog.py crash_info.txt

# Firmware version decoder
./Tools/scripts/firmware_version_decoder.py 0x04030200

# Compare binary sizes between branches
./Tools/scripts/size_compare_branches.py master HEAD
```

**Parameter Scripts:**
```bash
# Annotate parameter file
./Tools/scripts/annotate_params.py params.parm

# Check parameter file validity
./Tools/scripts/param_check.py params.parm

# Extract default parameters from binary
./Tools/scripts/extract_param_defaults.py build/CubeOrange/bin/arducopter.apj

# Convert parameter scaling
./Tools/scripts/convert_param_scale.py OLD_PARAM NEW_PARAM
```

**Upload Scripts:**
```bash
# Upload via serial bootloader
./Tools/scripts/uploader.py --port /dev/ttyACM0 firmware.apj

# Manipulate APJ files
./Tools/scripts/apj_tool.py --extract firmware.apj
```

**Code Quality:**
```bash
# Format code
./Tools/scripts/run_astyle.py file.cpp

# Python linting
./Tools/scripts/run_flake8.py

# Code coverage
./Tools/scripts/run_coverage.py
```

---

### 3. Replay/ - Log Replay Tool

**Purpose:** Replay dataflash logs through EKF and systems

**Build:**
```bash
./waf configure --board linux
./waf replay
```

**Usage:**
```bash
# Replay a log
./build/linux/tools/Replay logfile.BIN

# Replay with parameters from log
./build/linux/tools/Replay --parm-file logfile.BIN

# Create checked log
./build/linux/tools/Replay --check-generate logfile.BIN

# Check specific log
Tools/Replay/check_replay.py logfile.BIN checked.BIN

# Automated regression checking
cd Tools/Replay
./CheckLogs.py --logdir testlogs
```

**When to Use:**
- Testing EKF changes
- Verifying navigation changes
- Regression testing with real data
- Debugging sensor fusion

---

### 4. Frame_params/ - Vehicle Parameters

**Purpose:** Optimized parameter files for real vehicles

**Structure:**
```
Frame_params/
├── 3DR_Iris.param           # 3DR Iris quadcopter
├── Holybro-S500.param       # Holybro S500
├── Solo.param               # 3DR Solo
├── Parrot_Disco/            # Disco flying wing
├── QuadPlanes/              # QuadPlane configs
├── Sub/                     # Submarine configs
├── copter-optflow.param     # Optical flow setup
└── ... 40+ more files
```

**Usage:**
```bash
# In MAVProxy
param load Tools/Frame_params/Holybro-S500.param

# In SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter \
  --add-param-file=Tools/Frame_params/copter-optflow.param

# Via mission planner
# Config/Tuning → Full Parameter List → Load from file
```

---

### 5. environment_install/ - Setup Scripts

**Purpose:** One-click install of development dependencies

**Scripts:**
```bash
# Ubuntu/Debian (most complete)
Tools/environment_install/install-prereqs-ubuntu.sh -y

# macOS
Tools/environment_install/install-prereqs-mac.sh

# Arch Linux
Tools/environment_install/install-prereqs-arch.sh

# Windows (PowerShell as admin)
Tools/environment_install/install-prereqs-windows.ps1

# Skip simulation (faster)
Tools/environment_install/install-prereqs-ubuntu.sh -y --skip-sim

# ESP32 toolchain
Tools/environment_install/install-esp32-prereqs-ubuntu.sh

# ROS installation
Tools/environment_install/install-ROS-ubuntu.sh
```

**Installs:**
- ARM cross-compiler (arm-none-eabi-gcc)
- Python packages (pymavlink, MAVProxy, empy, future)
- Build tools (waf, ccache, genromfs)
- Simulators (JSBSim, FlightGear, Gazebo)
- Git, astyle, cppcheck

---

### 6. debug/ - Hardware Debugging

**Purpose:** Debug on hardware using GDB/OpenOCD/Black Magic Probe

**Key Files:**
```
debug/
├── crash_debugger.py        # Crash dump analyzer
├── gdb-black-magic.init     # GDB init for BMP
├── gdb-openocd.init         # GDB init for OpenOCD
├── openocd.cfg              # OpenOCD config (STM32F4)
├── openocd-h7.cfg           # OpenOCD config (STM32H7)
└── 99-blackmagic.rules      # udev rules for BMP
```

**Method 1: Black Magic Probe**
```bash
# Build with debug
./waf configure --board Pixhawk4 --debug --enable-asserts
./waf copter

# Connect BMP
# Start GDB
arm-none-eabi-gdb build/Pixhawk4/bin/arducopter
(gdb) target extended-remote /dev/ttyBmpGdb
(gdb) monitor swdp_scan
(gdb) attach 1
(gdb) load
(gdb) run
```

**Method 2: STLink + OpenOCD**
```bash
# Terminal 1: OpenOCD
openocd -f Tools/debug/openocd-h7.cfg

# Terminal 2: GDB
arm-none-eabi-gdb build/Pixhawk4/bin/arducopter
(gdb) target extended-remote localhost:3333
(gdb) load
(gdb) continue
```

**Method 3: Crash Dump Analysis**
```bash
# Via serial (live)
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0 --dump-filename crash.txt

# From saved dump
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --dump-debug --dump-filein crash_dump.bin

# Via MAVLink
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --mav-debug --master=/dev/ttyUSB0,115200
```

---

### 7. bootloaders/ - Bootloader Binaries

**Purpose:** Pre-compiled bootloaders for 200+ boards

**18,000+ binary files** in format: `BOARDNAME_bl.bin`

**Flash via STLink:**
```bash
st-flash write Tools/bootloaders/CubeOrange_bl.bin 0x8000000
```

**Flash via DFU:**
```bash
dfu-util -a 0 -D Tools/bootloaders/CubeOrange_bl.bin -s 0x08000000
```

---

### 8. AP_Bootloader/ - Bootloader Source

**Purpose:** ChibiOS-based bootloader source code

**Build:**
```bash
# Build for specific board
./waf configure --board CubeOrange --bootloader
./waf bootloader

# Output: build/CubeOrange/bin/AP_Bootloader.bin

# Build all
./Tools/scripts/build_bootloaders.py '*'

# Build pattern
./Tools/scripts/build_bootloaders.py 'Pixhawk*'
```

**Features:**
- USB and UART firmware upload
- PX4 bootloader protocol compatible
- Compressed bootloader in firmware
- User-updateable via MAVLink

---

### 9. AP_Periph/ - CAN Peripheral Firmware

**Purpose:** DroneCAN/UAVCAN peripheral node firmware

**Supports:**
- GPS modules
- Magnetometers, Barometers, Airspeed
- Rangefinders, Proximity sensors
- Battery monitors, ESCs
- RC input/output, LEDs, Buzzers
- ADSB receivers

**60+ Boards Supported** (STM32F1/F3/F4/F7/H7/L4/G4)

**Build:**
```bash
# Build for GPS board
./waf configure --board f103-GPS
./waf AP_Periph

# Output: build/f103-GPS/bin/AP_Periph.bin

# Flash via STLink
st-flash write build/f103-GPS/bin/AP_Periph.bin 0x8006400

# Flash bootloader first
st-flash write Tools/bootloaders/f103-GPS_bl.bin 0x8000000
```

**SITL Testing:**
```bash
# Test with simulated CAN peripherals
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad-can
```

**Update via DroneCAN:**
- Use Mission Planner CAN tool
- Or DroneCAN GUI tool
- Upload firmware.apj over CAN

---

### 10. completion/ - Shell Completion

**Purpose:** Auto-completion for ArduPilot commands

**Install:**
```bash
# Bash
echo "source ~/ardupilot/Tools/completion/completion.bash" >> ~/.bashrc
source ~/.bashrc

# Zsh
echo "source ~/ardupilot/Tools/completion/completion.zsh" >> ~/.zshrc
source ~/.zshrc

# Now you can tab-complete:
./waf <TAB>
./Tools/autotest/sim_vehicle.py -v <TAB>
```

---

### 11. gittools/ - Git Utilities

**Purpose:** Git workflow helpers

**Scripts:**
```bash
# Install pre-commit hook (runs style checks)
ln -s ../../Tools/gittools/pre-commit.py .git/hooks/pre-commit

# Commit by subsystem
./Tools/gittools/git-commit-subsystems

# Sync submodules
./Tools/gittools/submodule-sync.sh
```

---

### 12. mavproxy_modules/ - MAVProxy Extensions

**Purpose:** Custom MAVProxy modules

**Modules:**

**sitl_calibration:**
```bash
# Add to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:~/ardupilot/Tools/mavproxy_modules

# In MAVProxy
module load sitl_calibration
sitl_accelcal   # Auto calibrate accels
sitl_magcal     # Auto calibrate compass
sitl_attitude   # Set attitude
```

**magcal_graph:**
```bash
module load magcal_graph
# Shows compass calibration coverage
```

---

### 13. ros2/ - ROS 2 Integration

**Purpose:** ROS 2 packages for ArduPilot DDS

**Packages:**
- `ardupilot_sitl` - Launch SITL from ROS 2
- `ardupilot_dds_tests` - Test DDS communication

**Setup:**
```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Get repos
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
vcs import --recursive < ros2.repos

# Build
cd ~/ros2_ws
colcon build

# Source
source ./install/setup.bash

# Launch SITL
ros2 launch ardupilot_sitl sitl.launch.py command:=ardurover model:=rover

# With MAVProxy
ros2 launch ardupilot_sitl sitl_mavproxy.launch.py map:=True console:=True

# Test DDS
colcon test --packages-select ardupilot_dds_tests
```

---

### 14. Linux_HAL_Essentials/ - Linux HAL

**Purpose:** HAL setup for BeagleBone, Raspberry Pi

**Contents:**
- PRU (Programmable Real-time Unit) firmware
- Device tree overlays
- BeagleBone configuration
- Rangefinder PRU support

**Use Case:** Running ArduPilot on Linux SBCs

---

### 15-32. Additional Tools

**15. cameras_gimbals/** - Camera/gimbal configuration
**16. CHDK-Scripts/** - Canon CHDK scripts for aerial photography
**17. CPUInfo/** - CPU performance benchmarking
**18. CodeStyle/** - Code formatting (astyle)
**19. FilterTestTool/** - IMU filter testing on logs
**20. GIT_Test/** - Git functionality tests
**21. Hello/** - Minimal example program
**22. IO_Firmware/** - I/O coprocessor firmware (17 variants)
**23. Pozyx/** - Pozyx indoor positioning
**24. UDP_Proxy/** - UDP proxy for NAT traversal
**25. Vicon/** - Vicon motion capture interface
**26. vagrant/** - Pre-configured Vagrant container
**27. terrain-tools/** - Terrain data analysis
**28. simulink/** - MATLAB/Simulink integration
**29. geotag/** - Photo geotagging from logs
**30. PrintVersion.py** - Print version info
**31. ardupilotwaf/** - Core waf build system

---

## Complete Development Workflow

### Phase 1: Initial Setup

```bash
# 1. Clone repository
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 2. Update submodules
git submodule update --init --recursive

# 3. Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 4. Install shell completion (optional)
echo "source $(pwd)/Tools/completion/completion.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Install pre-commit hook (optional)
ln -s ../../Tools/gittools/pre-commit.py .git/hooks/pre-commit
```

### Phase 2: SITL Development

```bash
# 1. Configure for SITL
./waf configure --board sitl

# 2. Build
./waf copter

# 3. Test in SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# 4. Make code changes
# ... edit code ...

# 5. Rebuild
./waf copter

# 6. Test changes
# SITL automatically reloads on rebuild

# 7. Run automated tests
./Tools/autotest/autotest.py test.Copter.ArmFeatures

# 8. Check code style
./Tools/scripts/run_astyle.py modified_file.cpp
```

### Phase 3: Hardware Build

```bash
# 1. Configure for hardware
./waf configure --board CubeOrange

# 2. Build
./waf copter

# 3. Check features
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj

# 4. Check binary size
ls -lh build/CubeOrange/bin/arducopter.apj

# 5. Upload
./waf --upload copter

# Or upload via uploader script
./Tools/scripts/uploader.py --port /dev/ttyACM0 \
  build/CubeOrange/bin/arducopter.apj
```

### Phase 4: Testing & Validation

```bash
# 1. Test in SITL with different frames
./Tools/autotest/sim_vehicle.py -v ArduCopter -f X8

# 2. Run full test suite
./Tools/autotest/autotest.py test.Copter

# 3. Replay real flight logs
./build/linux/tools/Replay flight_log.BIN

# 4. Check for regressions
cd Tools/Replay
./CheckLogs.py --logdir logs/

# 5. Coverage analysis
./Tools/autotest/sim_vehicle.py -v ArduCopter --coverage
# Fly in SITL, then:
./Tools/scripts/run_coverage.py
```

### Phase 5: Debug & Analysis

```bash
# 1. Build with debug symbols
./waf configure --board CubeOrange --debug
./waf copter

# 2. If crash occurs, analyze
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0

# 3. Compare binary sizes
./Tools/scripts/size_compare_branches.py master my-feature

# 4. Decode device IDs from logs
./Tools/scripts/decode_devid.py 0x1234567

# 5. Check parameter changes
./Tools/scripts/param_check.py new_params.parm
```

---

## Testing Procedures

### SITL Testing (Daily)

```bash
# Quick smoke test
./Tools/autotest/sim_vehicle.py -v ArduCopter
# In MAVProxy:
arm throttle
rc 3 1500  # Takeoff
# Ctrl+C when done

# Full copter tests (~30 minutes)
./Tools/autotest/autotest.py test.Copter

# Specific test
./Tools/autotest/autotest.py test.Copter.Arm Features

# All vehicles (~2 hours)
./Tools/autotest/autotest.py test.All
```

### Hardware Testing

```bash
# 1. Bench test
# - Connect battery monitor
# - Upload firmware
# - Check MAVLink connection
# - Arm/disarm test
# - Motor test

# 2. Download logs
# Via MAVProxy:
log list
log download latest

# 3. Analyze logs
./build/linux/tools/Replay downloaded.BIN

# 4. Check for crashes
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --dump-debug --dump-filein crash.bin
```

### Regression Testing

```bash
# 1. Build Replay tool
./waf configure --board linux
./waf replay

# 2. Collect baseline logs
# (Before your changes)
mkdir baseline_logs
# Copy logs to baseline_logs/

# 3. Make code changes

# 4. Check for regressions
cd Tools/Replay
./CheckLogs.py --check-replay --logdir baseline_logs/

# 5. If regressions found:
# - Review differences
# - Determine if expected
# - Update checked logs if intentional:
./CheckLogs.py --create-checked-logs --logdir baseline_logs/
```

---

## Debugging Guide

### SITL Debugging

```bash
# Debug mode (verbose output)
./Tools/autotest/sim_vehicle.py -v ArduCopter -D

# GDB debugging
./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb

# Valgrind (memory errors)
./Tools/autotest/sim_vehicle.py -v ArduCopter -V

# Enable specific debug
./Tools/autotest/sim_vehicle.py -v ArduCopter --debug-level=DEBUG_ALL
```

### Hardware Debugging

**1. Serial Console:**
```bash
# Connect to console
screen /dev/ttyACM0 115200

# Or use MAVProxy
mavproxy.py --master=/dev/ttyACM0 --baudrate=115200
```

**2. Crash Analysis:**
```bash
# Live crash capture
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0 --dump-filename crash.txt

# Shows:
# - Stack trace
# - Register dump
# - Source lines
# - Function names
```

**3. GDB Debugging:**
```bash
# Build with debug
./waf configure --board Pixhawk4 --debug
./waf copter

# Method A: Black Magic Probe
arm-none-eabi-gdb build/Pixhawk4/bin/arducopter
(gdb) target extended-remote /dev/ttyBmpGdb
(gdb) monitor swdp_scan
(gdb) attach 1
(gdb) break main
(gdb) continue

# Method B: OpenOCD
# Terminal 1:
openocd -f Tools/debug/openocd-h7.cfg

# Terminal 2:
arm-none-eabi-gdb build/Pixhawk4/bin/arducopter
(gdb) target extended-remote localhost:3333
(gdb) break setup
(gdb) continue
```

---

## Common Tasks

### Task 1: Test Code Changes

```bash
# 1. Make changes
vim libraries/AP_GPS/AP_GPS.cpp

# 2. Build for SITL
./waf copter

# 3. Test in SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter

# 4. Run specific test
./Tools/autotest/autotest.py test.Copter.GPSForYaw

# 5. If good, build for hardware
./waf configure --board CubeOrange
./waf copter
```

### Task 2: Add New Feature

```bash
# 1. Create branch
git checkout -b my-feature

# 2. Make changes
# ... code ...

# 3. Test in SITL
./waf copter
./Tools/autotest/sim_vehicle.py -v ArduCopter

# 4. Run all tests
./Tools/autotest/autotest.py test.Copter

# 5. Check code style
./Tools/scripts/run_astyle.py modified_files.cpp

# 6. Check binary size impact
./Tools/scripts/size_compare_branches.py master my-feature

# 7. Check features
./Tools/scripts/extract_features.py build/sitl/bin/arducopter

# 8. Commit
git add files
git commit -m "Feature: Add XYZ"

# 9. Push
git push origin my-feature
```

### Task 3: Debug Crash

```bash
# 1. Get crash dump via serial
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0 --dump-filename crash.txt

# 2. Analyze crash.txt
# Shows stack trace with line numbers

# 3. If needed, debug with GDB
# Connect Black Magic Probe or OpenOCD
arm-none-eabi-gdb build/CubeOrange/bin/arducopter.elf

# 4. Reproduce in SITL with GDB
./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb
```

### Task 4: Optimize Binary Size

```bash
# 1. Check current size
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj

# 2. Disable unneeded features
# Edit libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/hwdef.dat
# Add: define HAL_FEATURE_NAME 0

# 3. Rebuild
./waf copter

# 4. Compare sizes
./Tools/scripts/size_compare_branches.py master HEAD

# 5. Check features
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj
```

### Task 5: Test Parameter Changes

```bash
# 1. Modify parameters
vim params.parm

# 2. Check syntax
./Tools/scripts/param_check.py params.parm

# 3. Annotate with descriptions
./Tools/scripts/annotate_params.py params.parm

# 4. Test in SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter \
  --add-param-file=params.parm

# 5. Fly and verify
# ... test in SITL ...
```

---

## Quick Reference

### Most Used Commands

```bash
# SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter
./Tools/autotest/sim_vehicle.py -v Rover
./Tools/autotest/sim_vehicle.py -v ArduPlane

# Testing
./Tools/autotest/autotest.py test.Copter
./Tools/autotest/autotest.py test.Copter.ArmFeatures

# Analysis
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj
./Tools/scripts/size_compare_branches.py master HEAD
./Tools/scripts/decode_devid.py 0x1234567

# Debugging
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0

# Replay
./build/linux/tools/Replay logfile.BIN

# Build
./waf configure --board CubeOrange
./waf copter
./waf --upload copter
```

### Directory Priorities

**🔥 Essential (use daily):**
1. `autotest/` - SITL and testing
2. `scripts/` - Build and analysis

**⚙️ Important (use weekly):**
3. `debug/` - Crash analysis
4. `Replay/` - Log replay
5. `Frame_params/` - Parameter files

**📦 Useful (use occasionally):**
6. `environment_install/` - Setup
7. `bootloaders/` - Bootloader binaries
8. `completion/` - Shell completion
9. `gittools/` - Git utilities

**🔧 Specialized (use as needed):**
10. `AP_Periph/` - CAN peripherals
11. `ros2/` - ROS 2 integration
12. `mavproxy_modules/` - MAVProxy extensions
13. Others - Specific use cases

---

## Summary

The **Tools directory is essential** for ArduPilot development:

✅ **SITL Testing** - `autotest/sim_vehicle.py` (use daily)
✅ **Automated Tests** - `autotest/autotest.py` (before commits)
✅ **Build Scripts** - `scripts/build_binaries.py` (releases)
✅ **Feature Analysis** - `scripts/extract_features.py` (optimization)
✅ **Crash Debug** - `debug/crash_debugger.py` (troubleshooting)
✅ **Log Replay** - `Replay/` (regression testing)
✅ **Setup** - `environment_install/` (initial setup)

**Next Steps:**
1. Install prerequisites: `Tools/environment_install/install-prereqs-ubuntu.sh`
2. Launch SITL: `Tools/autotest/sim_vehicle.py -v ArduCopter`
3. Make changes to your vehicle code
4. Test with SITL
5. Run automated tests
6. Build for hardware

**Your complete documentation now includes:**
- ✅ GCS architecture
- ✅ MiniVehicle (minimal)
- ✅ Rover (production)
- ✅ Tools (development)

You're ready to develop your own ArduPilot vehicle! 🚀

---

**For more information:**
- ArduPilot Dev Wiki: https://ardupilot.org/dev/
- SITL Guide: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- Tools: `/home/user/ardupilot/Tools/`
