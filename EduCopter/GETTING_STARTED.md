# Getting Started with EduCopter

## Quick Start

### 1. Build the Project

```bash
cd EduCopter
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 2. Run SITL

```bash
# From build directory
./educopter_sitl

# Or use the launch script
cd ..
./Tools/sitl_run.sh
```

### 3. Watch It Fly!

The simulation will:
1. Initialize all systems (~2 seconds)
2. Calibrate sensors
3. Start in STABILIZE mode
4. Auto-arm after 3 seconds (throttle low + yaw right)
5. Increase throttle to take off
6. Perform basic maneuvers
7. Create a flight log CSV file

### 4. View the Logs

```bash
cd build
ls -l educopter_*.csv
```

Open the CSV file in Excel, LibreOffice, or any plotting tool to analyze:
- Attitude (roll, pitch, yaw)
- Angular rates
- Position and velocity
- Motor outputs
- Control commands

## Understanding the Output

### Console Output

```
Status: Loop rate: 397.2 Hz | Pos: [0.00, 0.00, 1.23] m | Vel: [0.00, 0.00, -0.15] m/s | Armed: YES
```

- **Loop rate**: Main control loop frequency (target: 400Hz)
- **Pos**: Position in NED frame [North, East, Down] meters
- **Vel**: Velocity in NED frame [North, East, Down] m/s
- **Armed**: Whether motors are armed

### Flight Log Format

CSV file with columns:
```
time_us,type,data
1234567,ATT,0.015,0.023,1.234    # Attitude: roll, pitch, yaw (rad)
1234567,RATE,0.001,0.002,0.003   # Rates: roll_rate, pitch_rate, yaw_rate (rad/s)
1234567,IMU,0.12,0.15,9.81,...   # IMU: accel_xyz, gyro_xyz
1234567,MOT,0.45,0.46,0.44,0.45  # Motors: PWM outputs (0-1)
1234567,POS,1.2,0.3,-5.4         # Position: NED (m)
1234567,VEL,0.1,0.0,-0.5         # Velocity: NED (m/s)
```

## Modifying Flight Behavior

### Change PIDs

Edit `Libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`:

```cpp
// Line ~12-14
_pid_rate_roll(0.15f, 0.1f, 0.004f, 0.5f, 20.0f),  // P, I, D, Imax, filt_hz
```

Rebuild and test:
```bash
cd build
make
./educopter_sitl
```

### Change Flight Mode

Edit `main_sitl.cpp` to change initial mode or add mode switches.

### Modify RC Inputs

Edit `main_sitl.cpp` in the `rc_input_thread()` function:

```cpp
// Line ~40-45
sitl_hal->set_rc_input(2, 1600);  // Set throttle to 60%
```

### Adjust Physics

Edit `Libraries/SITL/SITL_Physics.cpp`:

```cpp
// Line ~18-22
mass(1.5f),                 // Change mass (kg)
arm_length(0.225f),         // Change arm length (m)
thrust_coefficient(12.0f),  // Change motor thrust
```

## Common Experiments

### 1. Step Response Test

Modify `main_sitl.cpp` to apply step inputs and observe response:

```cpp
// After arming
if (loop_count == 500) {
    sitl_hal->set_rc_input(0, 1700);  // 20% roll input
}
if (loop_count == 700) {
    sitl_hal->set_rc_input(0, 1500);  // Back to center
}
```

### 2. Stability Testing

Reduce PID gains to see instability:

```cpp
_pid_rate_roll(0.05f, 0.01f, 0.0f, 0.5f, 20.0f),  // Very low gains
```

### 3. Different Frame Types

Edit `EduCopter.cpp` line ~15:

```cpp
motors(AP_Motors::FRAME_QUAD_PLUS),  // Try Quad-Plus instead of Quad-X
```

### 4. Altitude Hold Mode

Modify initial mode in `EduCopter.cpp`:

```cpp
set_mode(Mode::ALT_HOLD, 0);  // Start in altitude hold
```

## Troubleshooting

### Build Fails

**Missing compiler:**
```bash
sudo apt-get install build-essential g++ cmake
```

**Permission denied:**
```bash
chmod +x Tools/sitl_run.sh
```

### Runtime Issues

**Low loop rate (<300Hz):**
- Your computer may be slow
- Reduce physics update rate in `main_sitl.cpp`
- Disable logging temporarily

**Unstable flight:**
- PIDs may need tuning
- Check sensor calibration
- Verify physics parameters match your configuration

**Crashes immediately:**
- Initial altitude may be too low
- Throttle curve may be wrong
- Check motor mixing

## Next Steps

1. **Study the code**: Start with `EduCopter.cpp` and follow the control flow
2. **Implement features**: Add RTL mode, position hold, or waypoint navigation
3. **Tune for performance**: Optimize PIDs for fast response
4. **Add sensors**: Implement rangefinder or optical flow
5. **Connect QGroundControl**: Complete MAVLink implementation

## Learning Path

### Week 1: Understanding the Architecture
- Read `EduCopter.cpp` - main vehicle class
- Study `mode_stabilize.cpp` - simplest flight mode
- Understand `AP_Motors.cpp` - motor mixing

### Week 2: Control Theory
- Study `AC_PID.cpp` - PID implementation
- Analyze `AC_AttitudeControl.cpp` - cascaded control
- Experiment with PID tuning

### Week 3: State Estimation
- Read `AP_AHRS.cpp` - attitude estimation
- Understand sensor fusion
- Study `SITL_Physics.cpp` - dynamics

### Week 4: Adding Features
- Implement a new flight mode
- Add a sensor
- Improve MAVLink support

## Resources

### ArduPilot Documentation
- https://ardupilot.org/dev/
- Study: Parameter system, MAVLink, EKF

### Control Theory
- PID tuning guides
- Cascaded control loops
- Feed-forward control

### Quadcopter Dynamics
- Motor thrust modeling
- Frame configurations
- Stability analysis

Happy learning! 🚁✨
