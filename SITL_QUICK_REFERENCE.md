# AP_HAL_SITL Quick Reference Guide

## Key Files Overview

| File | Purpose |
|------|---------|
| `HAL_SITL_Class.h/cpp` | Main HAL implementation entry point |
| `SITL_State.h/cpp` | Core simulation state management |
| `SITL_State_common.h/cpp` | Base simulation state and sensor data |
| `SITL_cmdline.cpp` | Command-line parsing and vehicle model selection |
| `Scheduler.h/cpp` | Timing and threading simulation |
| `UARTDriver.h/cpp` | Serial port simulation via TCP/UDP sockets |
| `RCOutput.h/cpp` | PWM output simulation |
| `RCInput.h/cpp` | RC input simulation |
| `AnalogIn.h/cpp` | ADC/analog input simulation |
| `I2CDevice.h/cpp` | I2C bus simulation |
| `SPIDevice.h/cpp` | SPI bus simulation |
| `CANSocketIface.h/cpp` | CAN bus simulation |
| `Storage.h/cpp` | EEPROM/Flash/FRAM simulation |
| `GPIO.h/cpp` | GPIO simulation |
| `Util.h/cpp` | Utility functions |

## Class Hierarchy

```
AP_HAL::HAL (abstract)
  └─ HAL_SITL (SITL implementation)

SITL_State_Common (abstract base)
  └─ SITL_State (main SITL state manager)

SITL::Aircraft (physics base)
  ├─ Multicopter (quad, hex, octa, etc.)
  ├─ Helicopter
  ├─ Plane
  ├─ Rover
  ├─ And 20+ more vehicle types
```

## Main Simulation Loop

```
wait_clock(time_us)
  ├─ _fdm_input_step()
  │   └─ _fdm_input_local()
  │       ├─ _simulator_servos(input)      [Read PWM outputs]
  │       ├─ sitl_model->update_model()    [Run physics]
  │       ├─ multicast_state_send()        [Share state]
  │       ├─ sim_update()                  [Update sensors]
  │       └─ stop_clock()                  [Update time]
  └─ Scheduler::timer_event()              [Run timer procs]
```

## Network Configuration

**Multicast for Multi-Vehicle:**
- IP: `239.255.145.51`
- State Port: `20721`
- Servo Port: `20722 + instance`

**Serial (TCP by default):**
- Base Port: `5760` (default, configurable)
- Instance 0: ports 5760-5769
- Instance 1: ports 5770-5779 (offset by -10)

## Command-Line Examples

```bash
# Simple quadrotor
sim_vehicle.py -M quad

# Quadrotor at specific location
sim_vehicle.py -M quad --home -35.3632,149.1652,30,353

# Fast simulation (10x speedup)
sim_vehicle.py -M quad -s 10

# With FlightGear visualization
sim_vehicle.py -M quad --enable-fgview

# Multi-vehicle (instance 1 of 2)
sim_vehicle.py -M quad -I 1

# Airplane model
sim_vehicle.py -M plane

# External simulator (JSBSim)
sim_vehicle.py -M jsbsim
```

## Sensor Simulation Sources

| Sensor | Source | Files |
|--------|--------|-------|
| **GPS** | `SITL::GPS` | SITL/SIM_GPS.* |
| **IMU** | `sitl_fdm` accel/gyro | Via I2C/SPI |
| **Compass** | Earth field + offset | SITL/SIM_Compass.* |
| **Barometer** | Altitude via atmosphere | SITL/SIM_Baro.* |
| **Airspeed** | `sitl_fdm.airspeed` | AnalogIn pin 1-2 |
| **Rangefinder** | Altitude AGL | AnalogIn pin 0 |
| **Battery** | Simulated discharge | AnalogIn pins 12-15 |
| **Gimbal** | Serial device | SITL/SIM_SoloGimbal.* |
| **ADSB** | Serial device | SITL/SIM_ADSB.* |
| **Vicon** | Serial device | SITL/SIM_Vicon.* |

## Key Data Structures

**PWM Output:**
```cpp
_sitlState->pwm_output[channel]  // 1000-2000 microseconds
```

**FDM State:**
```cpp
_sitl->state.latitude, longitude, altitude
_sitl->state.rollDeg, pitchDeg, yawDeg
_sitl->state.xAccel, yAccel, zAccel
_sitl->state.rollRate, pitchRate, yawRate
_sitl->state.airspeed, battery_voltage, battery_current
```

**Simulated Voltages:**
```cpp
_sitlState->sonar_pin_voltage           // Rangefinder (pin 0)
_sitlState->airspeed_pin_voltage[0-1]   // Airspeed (pins 1-2)
_sitlState->voltage_pin_voltage         // Battery voltage (pin 13)
_sitlState->current_pin_voltage         // Battery current (pin 12)
```

## Common Simulation Parameters

Accessible via `SIM_*` parameters:

```
SIM_SPEEDUP           # Simulation speedup (1.0 = real-time)
SIM_RATE_HZ           # Simulation frame rate
SIM_ENGINE_FAIL       # Bitmask of motors to fail
SIM_ENGINE_MUL        # Engine throttle multiplier
SIM_MAG_NOISE         # Compass noise magnitude
SIM_SONAR_SCALE       # Rangefinder meters per volt
SIM_WIND_*            # Wind simulation parameters
SIM_BARO_DRIFT        # Barometer drift rate
SIM_GPS_DELAY         # GPS delay in milliseconds
```

## Multi-Vehicle Simulation

**Master Instance (runs FDM):**
```bash
sim_vehicle.py -M quad -I 0
```

**Slave Instance (reads shared state):**
```bash
sim_vehicle.py -M quad -I 1
```

Both receive state via multicast UDP and can send servo updates back.

## Storage Configuration

Three storage backends can be enabled:
- **POSIX**: File-based (`eeprom.bin`)
- **Flash**: Simulated flash memory
- **FRAM**: Simulated FRAM memory

Enable via command-line:
```cpp
--set-storage-posix-enabled 1
--set-storage-flash-enabled 1
--set-storage-fram-enabled 1
```

## Serial Port Mapping

Default configuration:
```cpp
SERIAL0 → tcp:0:wait    // MAVLink telemetry
SERIAL1 → tcp:2         // Auxiliary
SERIAL2 → tcp:3         // Auxiliary
SERIAL3 → GPS1          // Primary GPS
SERIAL4 → GPS2          // Secondary GPS
SERIAL5 → tcp:5         // Auxiliary
SERIAL6 → tcp:6         // Auxiliary
SERIAL7 → tcp:7         // Auxiliary
SERIAL8 → tcp:8         // Auxiliary
```

Override with:
```bash
--serial0 tcp:9000
--serial3 sim:gps:0
```

## Vehicle Models

**Copter:**
- quad, +, x, bfxrev, bfx, djix, cwx, hexa, hexax, octa, tri, y6, deca, singlecopter, coaxcopter

**Plane:**
- plane, quadplane, glider, firefly

**Ground:**
- rover, balancebot, sailboat, motorboat

**Aerial:**
- heli, heli-dual, heli-compound, tracker, balloon, blimp

**Underwater:**
- vectored (6DOF submarine)

**External:**
- jsbsim, crrcsim, gazebo, xplane, airsim, webots

## Important Limitations

1. **Time is Simulated**: Cannot use wall-clock time; must use `hal.micros()`
2. **No Real Peripherals**: I2C/SPI/CAN are simulated
3. **CPU Bound**: Speedup limited by CPU (10x typical max on modern systems)
4. **Thread Safety**: Some SITL components not thread-safe (design limitation)
5. **Network Bound**: TCP/UDP performance affects simulation speed

## Debugging Tips

```cpp
// Enable floating-point exception catching
UBSAN_LOG_PATH=/tmp/ubsan.log sim_vehicle.py -M quad

// Run with GDB
gdb --args ./bin/arducopter -M quad

// Verbose output
SIM_DEBUG=1 sim_vehicle.py -M quad

// Slow simulation for debugging
sim_vehicle.py -M quad -s 0.1 --rate 100
```

## Integration Points

**Autopilot reads sensors from:**
- ADC pins (AnalogIn) → sonar, airspeed, battery
- I2C/SPI buses → IMU, compass, barometer
- UART → GPS, gimbal, ADSB

**Autopilot writes control to:**
- PWM outputs (RCOutput) → Motor/servo commands
- UART → Gimbal, relay commands
- CAN → DroneCAN devices

**Simulation updates from:**
- Physics model (`sitl_model->update_model()`)
- Sensor simulators (`sim_update()`)
- Time control (`stop_clock()`)

---

For detailed information, see: `AP_HAL_SITL_ANALYSIS.md`
