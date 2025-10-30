# CustomCopter - C++ CMake Flight Controller

A clean, modern C++ implementation of a quadcopter flight controller based on ArduPilot ArduCopter architecture.

## Project Overview

CustomCopter is a complete rewrite of ArduCopter using modern C++17 and CMake, designed to replicate all functionality without external dependencies on the ArduPilot libraries.

### Goals
- **Clean Architecture**: Six-layer design (HAL → Drivers → Fusion → Control → Modes → Application)
- **No Script Dependencies**: Pure C++ with CMake (no WAF, no Python build scripts)
- **Platform Independence**: HAL interface allows easy porting to any platform
- **Full Feature Parity**: Implement all 29 flight modes from ArduCopter
- **Modern C++**: C++17 standard with clear, maintainable code

## Project Status

### ✅ Completed (Stage 5.1-5.4)
- [x] Project structure and CMake build system
- [x] Core data structures (Vector3f, Quaternion, Location, etc.)
- [x] HAL interface definitions (GPIO, PWM, UART, I2C, SPI, Scheduler)
- [x] PID controller implementation (full P+I+D+FF with filtering)

### 🚧 In Progress (Stage 5.5-5.10)
- [ ] Task scheduler (400Hz main loop)
- [ ] IMU driver and sensor interfaces
- [ ] AHRS/attitude estimation
- [ ] Attitude controller (3-axis PID cascade)
- [ ] Motor mixing (quad X frame)
- [ ] STABILIZE flight mode
- [ ] Main Copter class

### 📋 Planned (Stage 5.11+)
- [ ] Additional sensors (GPS, Barometer, Compass, Rangefinder)
- [ ] EKF state estimation
- [ ] Position controller
- [ ] Flight modes (ALT_HOLD, LOITER, RTL, LAND, AUTO)
- [ ] Safety systems (arming, failsafe, landing detection)
- [ ] MAVLink telemetry
- [ ] Mission execution

## Architecture

### Six-Layer Design

```
Layer 6: Application (Missions, Parameters, Logging, GCS)
Layer 5: Flight Modes (29 modes: STABILIZE, LOITER, AUTO, etc.)
Layer 4: Control (Attitude, Position, Navigation, Motors)
Layer 3: Sensor Fusion (AHRS, EKF, Inertial Navigation)
Layer 2: Drivers (IMU, GPS, Compass, Barometer, etc.)
Layer 1: HAL (Hardware Abstraction Layer)
```

### Directory Structure

```
CustomCopter/
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
├── include/                    # Public headers
│   ├── DataTypes.h            # Core data structures
│   ├── HAL.h                  # Hardware abstraction interfaces
│   ├── PID.h                  # PID controller
│   ├── AttitudeControl.h      # Attitude controller
│   ├── Motors.h               # Motor mixing
│   └── ...
├── src/                        # Implementation
│   ├── core/                  # Core system
│   │   ├── Copter.cpp         # Main copter class
│   │   └── Scheduler.cpp      # Task scheduler
│   ├── hal/                   # HAL implementations
│   │   ├── HAL_Linux.cpp      # Linux implementation
│   │   └── ...
│   ├── control/               # Control systems
│   │   ├── PID.cpp            # PID controller
│   │   ├── AttitudeControl.cpp
│   │   ├── PositionControl.cpp
│   │   └── Motors.cpp
│   ├── modes/                 # Flight modes
│   │   ├── Mode.cpp           # Base mode class
│   │   ├── ModeStabilize.cpp
│   │   ├── ModeAltHold.cpp
│   │   └── ...
│   ├── sensors/               # Sensor drivers
│   │   ├── IMU.cpp
│   │   ├── GPS.cpp
│   │   ├── Compass.cpp
│   │   └── Barometer.cpp
│   ├── safety/                # Safety systems
│   │   ├── Arming.cpp
│   │   ├── Failsafe.cpp
│   │   └── LandingDetector.cpp
│   └── main.cpp               # Entry point
├── tests/                      # Unit tests
└── build/                      # Build output (gitignored)
```

## Building

### Requirements
- CMake 3.15+
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- pthread library (for threading)

### Build Steps

```bash
cd CustomCopter
mkdir build && cd build
cmake ..
make -j4
```

### Build Options

```bash
# Debug build (with debug symbols)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build (optimized)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

## Usage

```bash
# Run the flight controller
./customcopter

# Run with options (TBD)
./customcopter --config config.json
```

## Key Features

### Core Data Structures
- **Vector3f/2f**: 3D and 2D vector math with operators
- **Quaternion**: Attitude representation with Euler angle conversion
- **Location**: GPS position (lat/lon/alt) with distance/bearing calculations
- **PID**: Full PID controller with filtering and anti-windup

### Hardware Abstraction Layer (HAL)
- **GPIO**: Digital I/O pins
- **PWM**: Motor control output (1000-2000µs pulses)
- **UART**: Serial communication (GPS, telemetry)
- **I2C**: Sensor bus (IMU, compass, barometer)
- **SPI**: High-speed sensor bus
- **Scheduler**: Timing and threading primitives

### Control Systems
- **Rate Controller**: 3-axis PID (roll/pitch/yaw rates)
- **Attitude Controller**: Angle-to-rate conversion with feedforward
- **Position Controller**: XY position and Z altitude control
- **Motor Mixing**: Quad X frame (extensible to other frames)

### Flight Modes (Planned)
1. **STABILIZE** - Manual attitude control
2. **ALT_HOLD** - Altitude hold with manual horizontal
3. **LOITER** - GPS position hold
4. **RTL** - Return to launch
5. **LAND** - Autonomous landing
6. **AUTO** - Waypoint mission execution
7. And 23 more modes...

## Development Workflow

### Phase 1: Foundation ✅
- [x] CMake build system
- [x] Core data structures
- [x] HAL interfaces
- [x] PID controller

### Phase 2: Basic Flight 🚧
- [ ] Scheduler (400Hz loop)
- [ ] IMU driver
- [ ] AHRS (attitude estimation)
- [ ] Attitude controller
- [ ] Motor mixing
- [ ] STABILIZE mode

### Phase 3: Altitude Control
- [ ] Barometer driver
- [ ] Altitude controller
- [ ] ALT_HOLD mode

### Phase 4: Position Control
- [ ] GPS driver
- [ ] Compass driver
- [ ] EKF (sensor fusion)
- [ ] Position controller
- [ ] LOITER mode

### Phase 5: Autonomous Flight
- [ ] Waypoint navigation
- [ ] RTL mode
- [ ] LAND mode
- [ ] AUTO mode

### Phase 6: Safety & Robustness
- [ ] Arming checks
- [ ] Failsafe logic
- [ ] Landing detection
- [ ] Crash detection

### Phase 7: Advanced Features
- [ ] All 29 flight modes
- [ ] MAVLink telemetry
- [ ] Mission planning
- [ ] Parameter management

## Testing

```bash
# Run unit tests
cd build
ctest

# Run specific test
./tests/test_pid
./tests/test_attitude
```

## Documentation

- **Architecture docs**: See `/CustomCopter/STAGE1-4_*.md` for complete analysis
- **API docs**: Generate with Doxygen (TBD)
- **User manual**: TBD

## Performance Targets

- **Main loop rate**: 400 Hz (2.5ms cycle time)
- **Rate controller**: 400 Hz
- **Attitude controller**: 400 Hz
- **Position controller**: 100 Hz
- **Navigation updates**: 10-50 Hz
- **CPU usage**: < 50% on target hardware
- **Memory usage**: < 128 MB

## Hardware Targets

### Primary Target
- **Linux SBC**: Raspberry Pi 4, BeagleBone, NVIDIA Jetson
- **IMU**: MPU6000, MPU9250, ICM-20689
- **GPS**: uBlox M8N/M9N
- **Barometer**: MS5611, BMP280
- **Compass**: HMC5883L, QMC5883L

### Future Targets
- STM32 microcontrollers
- ESP32
- Custom hardware

## License

TBD - Following ArduPilot licensing (GPLv3)

## Credits

Based on ArduPilot ArduCopter architecture:
- Original authors: Jason Short, Randy Mackay, and ArduPilot team
- Complete rewrite for educational and development purposes
- Architecture analysis from ArduCopter v4.7.0-dev

## Contributing

This is a personal learning/development project. Contributions welcome after initial implementation is complete.

## Contact

TBD

---

**Status**: Stage 5 in progress - Foundation complete, building core flight systems
**Last Updated**: 2025
