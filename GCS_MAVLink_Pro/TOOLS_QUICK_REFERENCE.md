# Tools Quick Reference

Fast reference for ArduPilot Tools directory - essential commands for daily development.

---

## 🔥 Daily Use Commands

### Launch SITL

```bash
# Basic launch
cd /home/user/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter

# With console and map
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map

# Other vehicles
./Tools/autotest/sim_vehicle.py -v Rover
./Tools/autotest/sim_vehicle.py -v ArduPlane
./Tools/autotest/sim_vehicle.py -v ArduSub

# Wipe EEPROM (fresh start)
./Tools/autotest/sim_vehicle.py -v ArduCopter -w

# Custom frame
./Tools/autotest/sim_vehicle.py -v ArduCopter -f X8
./Tools/autotest/sim_vehicle.py -v Rover -f sailboat

# Debug mode
./Tools/autotest/sim_vehicle.py -v ArduCopter -D
./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb
```

### Run Tests

```bash
# All Copter tests
./Tools/autotest/autotest.py test.Copter

# Specific test
./Tools/autotest/autotest.py test.Copter.ArmFeatures

# All vehicles
./Tools/autotest/autotest.py test.All

# Rover tests
./Tools/autotest/autotest.py test.Rover
```

---

## ⚙️ Build Commands

### Basic Build

```bash
# Configure for SITL
./waf configure --board sitl

# Build
./waf copter

# Configure for hardware
./waf configure --board CubeOrange

# Build and upload
./waf copter --upload
```

### Build Scripts

```bash
# Build all release binaries
./Tools/scripts/build_binaries.py

# Build specific board
./Tools/scripts/build_binaries.py --board CubeOrange --vehicle ArduCopter

# Build bootloaders
./Tools/scripts/build_bootloaders.py '*'
```

---

## 🔍 Analysis Commands

### Feature Analysis

```bash
# Check what's in binary
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj

# Compare binary sizes
./Tools/scripts/size_compare_branches.py master HEAD

# Decode device ID
./Tools/scripts/decode_devid.py 0x1234567

# Firmware version
./Tools/scripts/firmware_version_decoder.py 0x04030200
```

### Parameter Tools

```bash
# Check parameter file
./Tools/scripts/param_check.py params.parm

# Annotate with descriptions
./Tools/scripts/annotate_params.py params.parm

# Extract defaults from binary
./Tools/scripts/extract_param_defaults.py build/CubeOrange/bin/arducopter.apj
```

---

## 🐛 Debugging Commands

### Crash Analysis

```bash
# Get crash via serial (live)
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0 --dump-filename crash.txt

# Analyze saved dump
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --dump-debug --dump-filein crash_dump.bin
```

### GDB Debugging

```bash
# Build with debug
./waf configure --board Pixhawk4 --debug
./waf copter

# SITL with GDB
./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb

# Hardware with Black Magic Probe
arm-none-eabi-gdb build/Pixhawk4/bin/arducopter
(gdb) target extended-remote /dev/ttyBmpGdb
(gdb) monitor swdp_scan
(gdb) attach 1
```

### Log Replay

```bash
# Build Replay
./waf configure --board linux
./waf replay

# Replay log
./build/linux/tools/Replay logfile.BIN

# Check for regressions
cd Tools/Replay
./CheckLogs.py --logdir testlogs
```

---

## 📦 Setup Commands

### Initial Setup

```bash
# Clone
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Submodules
git submodule update --init --recursive

# Install prereqs (Ubuntu)
Tools/environment_install/install-prereqs-ubuntu.sh -y

# macOS
Tools/environment_install/install-prereqs-mac.sh

# Windows (PowerShell as admin)
Tools/environment_install/install-prereqs-windows.ps1
```

### Shell Completion

```bash
# Bash
echo "source $(pwd)/Tools/completion/completion.bash" >> ~/.bashrc
source ~/.bashrc

# Zsh
echo "source $(pwd)/Tools/completion/completion.zsh" >> ~/.zshrc
source ~/.zshrc
```

### Git Hooks

```bash
# Install pre-commit hook
ln -s ../../Tools/gittools/pre-commit.py .git/hooks/pre-commit
```

---

## 📋 Common Workflows

### Workflow 1: Test Code Change

```bash
# 1. Make changes
vim libraries/AP_GPS/AP_GPS.cpp

# 2. Build
./waf copter

# 3. Test in SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter

# 4. Run tests
./Tools/autotest/autotest.py test.Copter.GPSForYaw
```

### Workflow 2: Hardware Deploy

```bash
# 1. Configure
./waf configure --board CubeOrange

# 2. Build
./waf copter

# 3. Check features
./Tools/scripts/extract_features.py build/CubeOrange/bin/arducopter.apj

# 4. Upload
./waf --upload copter

# Or via uploader
./Tools/scripts/uploader.py --port /dev/ttyACM0 \
  build/CubeOrange/bin/arducopter.apj
```

### Workflow 3: Debug Crash

```bash
# 1. Get crash dump
./Tools/debug/crash_debugger.py build/CubeOrange/bin/arducopter.elf \
  --ser-debug --ser-port /dev/ttyUSB0

# 2. Analyze (shows stack trace with line numbers)

# 3. Fix code

# 4. Test in SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb
```

### Workflow 4: Add Feature

```bash
# 1. Branch
git checkout -b my-feature

# 2. Code changes
# ... edit ...

# 3. Format
./Tools/scripts/run_astyle.py modified_file.cpp

# 4. Test
./waf copter
./Tools/autotest/autotest.py test.Copter

# 5. Check size
./Tools/scripts/size_compare_branches.py master my-feature

# 6. Commit
git add files
git commit -m "Feature: XYZ"

# 7. Push
git push origin my-feature
```

---

## 🎯 Quick Directory Map

```
Tools/
├── autotest/            # 🔥 SITL & testing (use daily)
│   ├── sim_vehicle.py   # Launch SITL
│   └── autotest.py      # Run tests
│
├── scripts/             # 🔥 Build & analysis (use daily)
│   ├── build_binaries.py
│   ├── extract_features.py
│   └── size_compare_branches.py
│
├── debug/               # 🐛 Debugging (as needed)
│   └── crash_debugger.py
│
├── Replay/              # 📊 Log replay (testing)
│
├── Frame_params/        # 📦 Vehicle configs
│
├── environment_install/ # ⚙️ Setup (once)
│
├── bootloaders/         # Bootloader binaries
├── AP_Bootloader/       # Bootloader source
├── AP_Periph/           # CAN peripheral firmware
│
├── completion/          # Shell completion
├── gittools/            # Git utilities
├── mavproxy_modules/    # MAVProxy extensions
└── ros2/                # ROS 2 integration
```

---

## 💡 Pro Tips

### Tip 1: Fast SITL Iteration

```bash
# Keep SITL running, rebuild in another terminal
# SITL auto-reloads on rebuild

# Terminal 1:
./Tools/autotest/sim_vehicle.py -v ArduCopter

# Terminal 2:
vim code.cpp
./waf copter  # SITL reloads automatically
```

### Tip 2: Parameter Files

```bash
# Load params in SITL
./Tools/autotest/sim_vehicle.py -v ArduCopter \
  --add-param-file=Tools/Frame_params/Holybro-S500.param

# Load in MAVProxy
param load Tools/Frame_params/Holybro-S500.param
```

### Tip 3: SITL Frames

```bash
# Copter frames
-f quad        # Standard quad
-f X8          # X8 octocopter
-f octa        # Octocopter
-f tri         # Tricopter
-f y6          # Y6
-f heli        # Helicopter

# Rover frames
-f rover       # Regular rover
-f rover-skid  # Skid steering
-f sailboat    # Sailboat
-f balancebot  # Balance bot

# Plane frames
-f plane       # Standard plane
-f quadplane   # QuadPlane
```

### Tip 4: Debug Levels

```bash
# SITL debug output
./Tools/autotest/sim_vehicle.py -v ArduCopter -D

# Valgrind (memory check)
./Tools/autotest/sim_vehicle.py -v ArduCopter -V

# Coverage analysis
./Tools/autotest/sim_vehicle.py -v ArduCopter --coverage
```

### Tip 5: Quick Tests

```bash
# Run just one test (fast)
./Tools/autotest/autotest.py test.Copter.ArmFeatures

# List available tests
cd Tools/autotest
./arducopter.py --list-subtests
```

---

## 📚 Essential Docs

- **Full Guide:** `TOOLS_COMPLETE_GUIDE.md`
- **Rover Analysis:** `ROVER_COMPLETE_ANALYSIS.md`
- **MiniVehicle:** `MiniVehicle/BUILD_AND_RUN.md`
- **ArduPilot Wiki:** https://ardupilot.org/dev/

---

## 🚀 Quick Start

```bash
# 1. Setup (once)
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 2. Build for SITL
./waf configure --board sitl
./waf copter

# 3. Test
./Tools/autotest/sim_vehicle.py -v ArduCopter

# 4. Develop
# - Edit code
# - Run: ./waf copter (in another terminal)
# - SITL reloads automatically
# - Test changes

# 5. Test thoroughly
./Tools/autotest/autotest.py test.Copter

# 6. Build for hardware
./waf configure --board CubeOrange
./waf copter --upload
```

**You're ready to develop!** 🎉
