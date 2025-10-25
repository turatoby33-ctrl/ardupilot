# QGroundControl Integration

EduCopter now supports basic MAVLink communication over UDP for QGroundControl connectivity.

## Setup Instructions

### 1. Install QGroundControl

**Linux:**
```bash
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

**macOS:**
```bash
brew install --cask qgroundcontrol
```

**Windows:**
Download from: https://qgroundcontrol.com

### 2. Start EduCopter SITL

```bash
cd EduCopter/build
./educopter_sitl
```

You should see:
```
GCS_MAVLink: Initializing MAVLink communication
GCS_MAVLink: System ID: 1, Component ID: 1
GCS_MAVLink: UDP socket created on port 14550
GCS_MAVLink: Waiting for QGroundControl connection...
GCS_MAVLink: QGC should auto-detect on UDP port 14550
GCS_MAVLink: Ready for QGroundControl
```

### 3. Configure QGroundControl

QGroundControl should auto-detect EduCopter on localhost:14550. If not:

1. Open QGroundControl
2. Go to **Application Settings** (Q icon top-left)
3. Select **Comm Links**
4. Click **Add** to create a new connection
5. Configure:
   - **Name**: EduCopter SITL
   - **Type**: UDP
   - **Port**: 14550
   - **Server Address**: 127.0.0.1
6. Click **OK**
7. Click **Connect**

## What You'll See

### Telemetry Data

QGroundControl will display:
- **Attitude**: Real-time roll, pitch, yaw (artificial horizon)
- **Altitude**: Barometric altitude
- **Position**: GPS coordinates (lat/lon)
- **Velocity**: Ground speed and climb rate
- **Status**: Armed/disarmed state
- **Flight Mode**: Current mode (displayed as number)

### MAVLink Messages

EduCopter sends these MAVLink v1 messages:

| Message | Rate | Contents |
|---------|------|----------|
| HEARTBEAT | 1 Hz | System status, armed state |
| ATTITUDE | 4 Hz | Roll, pitch, yaw, rates |
| GLOBAL_POSITION_INT | 4 Hz | Lat, lon, alt, velocity |
| VFR_HUD | 4 Hz | Airspeed, groundspeed, heading, throttle, alt, climb |

## Limitations

### Current Implementation
- **MAVLink v1 only** (not v2)
- **Simplified CRC** (zeros instead of proper checksum)
- **Basic messages only** (no commands, parameters, missions)
- **No incoming message parsing**
- **UDP only** (no serial or TCP)

### What Works
✅ Attitude display
✅ Altitude display
✅ Position display (if GPS active)
✅ Velocity display
✅ Armed/disarmed status
✅ Heartbeat

### What Doesn't Work
❌ Arming from QGC
❌ Mode changes from QGC
❌ Parameter adjustment
❌ Mission planning
❌ Waypoint upload
❌ RC override
❌ Command execution

## Troubleshooting

### QGC Doesn't Connect

1. **Check port 14550 is not in use:**
   ```bash
   sudo netstat -tulpn | grep 14550
   ```

2. **Check firewall:**
   ```bash
   sudo ufw allow 14550/udp
   ```

3. **Verify EduCopter is sending:**
   Monitor UDP traffic:
   ```bash
   sudo tcpdump -i lo udp port 14550 -X
   ```

4. **Check QGC console:**
   - In QGC, open **Analyze Tools** > **MAVLink Inspector**
   - Look for messages from System ID 1

### Connection Drops

- EduCopter sends heartbeat every 1 second
- QGC expects heartbeat every ~5 seconds
- If you don't see "Connected" in QGC after 5 seconds, restart both

### Incorrect Data

- Position may be incorrect if GPS hasn't initialized
- Altitude is relative to takeoff point
- MAVLink CRCs are simplified (not validated)

## Testing Without QGC

You can verify MAVLink packets with `nc` (netcat):

```bash
# In one terminal, start EduCopter
cd EduCopter/build
./educopter_sitl

# In another terminal, listen on port 14550
nc -u -l 14550
```

You should see binary MAVLink packets.

Or use MAVProxy:
```bash
pip install MAVProxy
mavproxy.py --master=udp:127.0.0.1:14550
```

## Improving MAVLink Support

To add full MAVLink support, you would need to:

1. **Use official MAVLink library**
   ```bash
   git clone https://github.com/mavlink/mavlink
   ```

2. **Implement proper CRC calculation**
   - Use `mavlink_finalize_message_chan()`
   - Add CRC extra bytes per message

3. **Add incoming message parsing**
   - Parse COMMAND_LONG for arming
   - Parse SET_MODE for mode changes
   - Parse PARAM_REQUEST_LIST/SET

4. **Add more messages**
   - SYS_STATUS (battery, sensors)
   - GPS_RAW_INT (GPS details)
   - RC_CHANNELS (RC inputs)
   - SERVO_OUTPUT_RAW (motor outputs)

5. **Implement MAVLink v2**
   - Signing support
   - Message extensions
   - Better compatibility

## Example: Full MAVLink Integration

See `Libraries/GCS_MAVLink/GCS_MAVLink.cpp` for current implementation.

To add command handling:

```cpp
void GCS_MAVLink::handle_message() {
    // Receive UDP packet
    uint8_t buf[300];
    ssize_t received = recv(_udp_socket, buf, sizeof(buf), 0);

    if (received > 0) {
        // Parse MAVLink message
        if (buf[0] == 0xFE) {  // MAVLink v1
            uint8_t msg_id = buf[5];

            if (msg_id == MAVLINK_MSG_ID_COMMAND_LONG) {
                // Handle arm/disarm commands
                // Parse payload and execute
            }
        }
    }
}
```

## Network Configuration

### Localhost (Default)
- Address: 127.0.0.1
- Best for testing on same machine
- No network required

### LAN (For Multiple Machines)
Edit `GCS_MAVLink.cpp` line 297:
```cpp
addr.sin_addr.s_addr = inet_addr("192.168.1.255");  // Broadcast on LAN
```

Then rebuild:
```bash
cd build
make
```

### Different Port
Edit `GCS_MAVLink.cpp` line 25:
```cpp
_udp_port(14551),  // Use port 14551 instead
```

## QGroundControl Tips

### Best View for Testing
1. **Fly View**: See artificial horizon and gauges
2. **Analyze Tools** > **MAVLink Inspector**: See raw messages
3. **Application Settings** > **General**: Enable "Show MAVLink packet loss"

### Useful QGC Settings
- **Application Settings** > **General** > **Units**: Choose metric/imperial
- **Vehicle Setup** > **Parameters**: Won't work (not implemented)
- **Plan View**: Won't work (mission upload not implemented)

## Summary

**Current Status**: ✅ Basic telemetry working
- Real-time attitude visualization
- Position and altitude display
- Velocity and heading
- Armed status indication

**Future Work**: Additional features require full MAVLink library integration

---

For questions or improvements, see the main README.md.

🚁 **Happy Flying with QGroundControl!** 🚁
