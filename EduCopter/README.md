# EduCopter - Educational Quadcopter Flight Controller

EduCopter is a simplified but feature-rich quadcopter flight controller designed for educational purposes. It implements the core functionality of a modern flight controller with a clean, understandable architecture inspired by ArduPilot.

## Features

### Core Systems
- **PID Control**: Cascaded attitude and rate controllers
- **AHRS/EKF**: Complementary filter-based attitude estimation
- **Flight Modes**: Stabilize, Altitude Hold, Land
- **Motor Control**: Quad-X and Quad-Plus frame support
- **Arming System**: Pre-flight safety checks
- **Parameter System**: Tunable flight parameters
- **Task Scheduler**: Multi-rate task execution (400Hz main loop)
- **Logger**: CSV-based flight data logging
- **MAVLink**: Basic QGroundControl integration

### SITL (Software In The Loop)
- Physics-based quadcopter simulation
- Realistic sensor simulation (IMU, Baro, GPS, Compass)
- Sensor noise modeling
- No hardware required for testing

## Architecture

EduCopter follows a modular architecture similar to ArduPilot:

```
EduCopter/
├── Libraries/
│   ├── AP_HAL/              # Hardware Abstraction Layer
│   ├── AP_HAL_SITL/         # SITL implementation
│   ├── AP_Math/             # Math utilities (vectors, quaternions)
│   ├── AP_Param/            # Parameter system
│   ├── AP_Scheduler/        # Task scheduler
│   ├── AP_InertialSensor/   # IMU interface
│   ├── AP_Baro/             # Barometer interface
│   ├── AP_GPS/              # GPS interface
│   ├── AP_Compass/          # Compass interface
│   ├── AP_AHRS/             # Attitude estimation
│   ├── AC_PID/              # PID controllers
│   ├── AC_AttitudeControl/  # Attitude control system
│   ├── AP_Motors/           # Motor mixing
│   ├── AP_Arming/           # Arming checks
│   ├── GCS_MAVLink/         # MAVLink communication
│   ├── AP_Logger/           # Data logging
│   └── SITL/                # Physics simulation
├── mode*.cpp/h              # Flight mode implementations
├── EduCopter.cpp/h          # Main vehicle class
├── config.h                 # Configuration parameters
└── main_sitl.cpp            # SITL entry point
```

## Building and Running

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt-get install build-essential cmake g++

# macOS
brew install cmake
```

### Build

```bash
cd EduCopter
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run SITL

Option 1 - Using the launch script:
```bash
cd EduCopter
./Tools/sitl_run.sh
```

Option 2 - Direct execution:
```bash
cd EduCopter/build
./educopter_sitl
```

## Flight Modes

### Stabilize Mode
- Manual attitude control
- Pilot controls roll, pitch, yaw angles
- Manual throttle control
- Default flight mode

### Altitude Hold Mode
- Manual attitude control
- Automatic altitude hold
- Throttle stick controls climb/descent rate
- Barometer-based altitude control

### Land Mode
- Autonomous landing
- Gradual descent
- Ground detection
- Automatic disarm on landing

## Control Architecture

### Cascaded Control Loops

```
RC Input
   ↓
Angle Controller (P-only)
   ↓
Rate Controller (PID)
   ↓
Motor Mixer
   ↓
Motors
```

### Loop Rates
- **Fast Loop (400Hz)**: IMU update, rate controller, motor output
- **RC Loop (50Hz)**: RC input reading
- **Sensor Loops**: Baro (20Hz), GPS (10Hz), Compass (10Hz)

## PID Tuning

Default PID values (in `AC_AttitudeControl.cpp`):

```cpp
// Rate PIDs (inner loop)
Roll Rate:  P=0.15, I=0.1, D=0.004
Pitch Rate: P=0.15, I=0.1, D=0.004
Yaw Rate:   P=0.2,  I=0.02, D=0.0

// Angle P gains (outer loop)
Roll/Pitch: P=4.5
Yaw:        P=4.5
```

To tune PIDs:
1. Edit `Libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`
2. Rebuild and test in SITL
3. Check logs for stability

## Logging

Flight logs are saved as CSV files in the build directory:

```
educopter_YYYYMMDD_HHMMSS.csv
```

Log data includes:
- Attitude (roll, pitch, yaw)
- Rates (angular velocities)
- IMU data (accel, gyro)
- Control outputs
- Position and velocity
- Mode changes
- Events (arm/disarm)

## Code Structure and Extensibility

### Adding a New Flight Mode

1. Create header file (e.g., `mode_newmode.h`):
```cpp
#pragma once
#include "mode.h"

class ModeNewMode : public Mode {
public:
    ModeNewMode(EduCopter& copter) : Mode(copter) {}
    bool init(bool ignore_checks) override;
    void run() override;
    Number mode_number() const override { return NEW_MODE; }
    const char* name() const override { return "NEW_MODE"; }
};
```

2. Implement in `mode_newmode.cpp`
3. Add to `mode.h` enum
4. Register in `EduCopter.h` and `EduCopter.cpp`

### Adding a New Sensor

1. Create HAL interface in `AP_HAL/AP_HAL.h`
2. Create sensor class in `Libraries/AP_SensorName/`
3. Implement SITL version in `AP_HAL_SITL/`
4. Add to physics simulation in `SITL/SITL_Physics.cpp`

### Modifying Physics

Edit `Libraries/SITL/SITL_Physics.cpp`:

- **Vehicle parameters** (mass, arm length, etc.): Constructor
- **Motor dynamics**: `calculate_forces()` and `calculate_moments()`
- **Aerodynamics**: `calculate_forces()`
- **Sensor noise**: `add_noise_*()` methods

## Understanding the Code

### Main Loop Flow

```cpp
void EduCopter::loop() {
    scheduler.run()        // Run scheduled tasks
      ↓
    fast_loop()           // 400Hz
      ↓ update_sensors()
      ↓ update_ahrs()
      ↓ flightmode->run()
      ↓ motors_output()

    mavlink.update()      // MAVLink telemetry
}
```

### Flight Mode Execution

```cpp
void ModeStabilize::run() {
    1. Get pilot input (RC channels)
    2. Convert to desired angles/rates
    3. Call attitude controller
    4. Get motor outputs
    5. Set motors
}
```

### Attitude Control

```cpp
// Outer loop: Angle → Rate
angle_error = target_angle - current_angle
target_rate = angle_error * P_angle

// Inner loop: Rate → Motor output
rate_error = target_rate - current_rate
output = rate_error * P + integral * I + derivative * D
```

## Key Files to Study

### For Control Theory
- `Libraries/AC_PID/AC_PID.cpp` - PID implementation
- `Libraries/AC_AttitudeControl/AC_AttitudeControl.cpp` - Attitude control
- `Libraries/AP_Motors/AP_Motors.cpp` - Motor mixing

### For State Estimation
- `Libraries/AP_AHRS/AP_AHRS.cpp` - AHRS/EKF implementation
- `Libraries/AP_InertialSensor/AP_InertialSensor.cpp` - IMU processing

### For Physics
- `Libraries/SITL/SITL_Physics.cpp` - Flight dynamics
- Motor forces, moments, integration

### For System Architecture
- `EduCopter.cpp` - Main vehicle class
- `Libraries/AP_Scheduler/AP_Scheduler.cpp` - Task scheduling

## QGroundControl Integration

EduCopter implements basic MAVLink for QGroundControl compatibility:

1. Start EduCopter SITL
2. Open QGroundControl
3. It should auto-connect via UDP (future enhancement)
4. View attitude, position, and telemetry

Currently implemented MAVLink messages:
- HEARTBEAT
- ATTITUDE
- GLOBAL_POSITION_INT
- VFR_HUD

## Performance Metrics

On a typical laptop:
- Main loop: ~400Hz
- Physics simulation: ~1000Hz
- CPU usage: <5%
- Memory: ~10MB

## Future Enhancements

Potential areas for expansion:

1. **More Flight Modes**
   - RTL (Return to Launch)
   - Auto (waypoint navigation)
   - Loiter (position hold)

2. **Advanced Control**
   - Full EKF2 implementation
   - Position controller
   - Velocity controller

3. **More Sensors**
   - Rangefinder
   - Optical flow
   - Airspeed sensor

4. **Communication**
   - Full MAVLink implementation
   - UDP/Serial communication
   - Mission planning

5. **Safety Features**
   - Failsafes (RC loss, battery, GPS)
   - Geofencing
   - Return to home

6. **Tuning Tools**
   - Auto-tune
   - Real-time parameter adjustment
   - PID visualization

## Troubleshooting

### Build Issues

**CMake not found:**
```bash
sudo apt-get install cmake
```

**Compiler errors:**
```bash
sudo apt-get install build-essential g++
```

### Runtime Issues

**Simulation doesn't start:**
- Check that executable was built successfully
- Run from build directory
- Check for error messages in console

**Poor flight performance:**
- PIDs may need tuning for your modifications
- Check sensor noise levels
- Verify physics parameters

## Learning Resources

### Control Theory
- PID controller basics
- Cascaded control loops
- Feed-forward control

### State Estimation
- Complementary filters
- Extended Kalman Filters (EKF)
- Sensor fusion

### Quadcopter Dynamics
- Motor thrust and moments
- Frame configurations
- Stability analysis

### Software Architecture
- Hardware abstraction layers
- Task scheduling
- Real-time systems

## Contributing

This is an educational project. Feel free to:
- Experiment with the code
- Add new features
- Improve documentation
- Share modifications

## License

Educational use. Based on concepts from ArduPilot (GPL v3).

## Credits

Inspired by:
- **ArduPilot**: https://ardupilot.org
- **PX4**: https://px4.io
- **Betaflight**: https://betaflight.com

## Contact and Support

This is an educational demonstration project. For learning and experimentation.

---

**Happy Flying! 🚁**
