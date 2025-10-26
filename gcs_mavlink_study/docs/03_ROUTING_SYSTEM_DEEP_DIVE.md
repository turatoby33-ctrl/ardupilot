# MAVLink Routing System - Deep Dive

## Files
- `MAVLink_routing.h`
- `MAVLink_routing.cpp`

## Purpose
The routing system is the **TRAFFIC CONTROLLER** for MAVLink messages. It:
- Learns which devices are on which channels
- Forwards messages to appropriate destinations
- Prevents message loops
- Enables multi-GCS and companion computer setups

---

## The Routing Problem

### Scenario: Multiple Devices
```
               ┌─────────────────┐
               │  Flight         │
               │  Controller     │
               │  (sysid=1)      │
               └────┬──┬──┬──┬───┘
                    │  │  │  │
        ┌───────────┘  │  │  └──────────┐
        │              │  │             │
   ┌────▼────┐    ┌───▼──▼───┐    ┌────▼────┐
   │ QGC     │    │  RasPi   │    │ Mission │
   │ sysid   │    │ sysid    │    │ Planner │
   │ =255    │    │ =100     │    │ sysid   │
   │ compid  │    │ compid   │    │ =254    │
   │ =190    │    │ =191     │    │ compid  │
   └─────────┘    └──────────┘    └─────────┘

Channel 0       Channel 1         Channel 2
(USB)           (Telem1)          (Telem2)
```

### Questions the Router Must Answer:
1. **Where did this message come from?**
   - Which channel?
   - What is the sender's sysid/compid?

2. **Who should receive this message?**
   - Process locally?
   - Forward to other channels?
   - Which channels?

3. **How to prevent loops?**
   - Don't forward back to sender
   - Don't forward private channel messages

---

## Route Learning

### Route Structure (Lines 56-61 in routing.h)

```cpp
struct route {
    uint8_t sysid;           // System ID (e.g., 255 for GCS)
    uint8_t compid;          // Component ID (e.g., 190 for QGC)
    mavlink_channel_t channel;  // Which channel (COMM_0, COMM_1, etc.)
    uint8_t mavtype;         // MAV_TYPE (GCS, onboard computer, etc.)
} routes[MAVLINK_MAX_ROUTES];  // Max 20 routes
```

### Learning Process (routing.cpp:37-200)

```cpp
void MAVLink_routing::learn_route(GCS_MAVLINK &link,
                                  const mavlink_message_t &msg) {
    // Extract sender info
    uint8_t sysid = msg.sysid;
    uint8_t compid = msg.compid;
    mavlink_channel_t chan = link.get_chan();

    // Check if we already know this route
    for (uint8_t i = 0; i < num_routes; i++) {
        if (routes[i].sysid == sysid &&
            routes[i].compid == compid) {

            // Update channel if changed
            if (routes[i].channel != chan) {
                routes[i].channel = chan;
            }
            return;  // Already learned
        }
    }

    // New route! Add to table
    if (num_routes < MAVLINK_MAX_ROUTES) {
        routes[num_routes].sysid = sysid;
        routes[num_routes].compid = compid;
        routes[num_routes].channel = chan;
        routes[num_routes].mavtype = get_mavtype(msg);
        num_routes++;
    }
}
```

### Example: Learning Routes

**Initial state:**
```
routes[] = empty
```

**Message 1 arrives on Channel 0:**
```
HEARTBEAT from sysid=255, compid=190 (QGroundControl)
```

**Route learned:**
```cpp
routes[0] = {
    sysid:   255,
    compid:  190,
    channel: MAVLINK_COMM_0,
    mavtype: MAV_TYPE_GCS
}
num_routes = 1
```

**Message 2 arrives on Channel 1:**
```
HEARTBEAT from sysid=100, compid=191 (Companion Computer)
```

**Route learned:**
```cpp
routes[1] = {
    sysid:   100,
    compid:  191,
    channel: MAVLINK_COMM_1,
    mavtype: MAV_TYPE_ONBOARD_CONTROLLER
}
num_routes = 2
```

---

## Message Forwarding Logic

### The Core Function: check_and_forward() (routing.cpp:97-240)

```cpp
bool MAVLink_routing::check_and_forward(GCS_MAVLINK &in_link,
                                       const mavlink_message_t &msg) {
    // Step 1: Prevent loopback of our own messages
    if (msg.sysid == mavlink_system.sysid &&
        msg.compid == mavlink_system.compid) {
        return false;  // Ignore our own messages
    }

    // Step 2: Learn the route
    learn_route(in_link, msg);

    // Step 3: Extract target system/component
    int16_t target_system = -1;
    int16_t target_component = -1;
    get_targets(msg, target_system, target_component);

    // Step 4: Determine if for us
    bool broadcast_system = (target_system == 0 || target_system == -1);
    bool broadcast_component = (target_component == 0 || target_component == -1);
    bool match_system = broadcast_system ||
                       (target_system == mavlink_system.sysid);
    bool match_component = match_system &&
                          (broadcast_component ||
                           target_component == mavlink_system.compid);
    bool process_locally = match_system && match_component;

    // Step 5: Don't forward from private channels
    if (in_link.is_private()) {
        return process_locally;
    }

    // Step 6: Forward to appropriate channels
    for (uint8_t i = 0; i < num_routes; i++) {
        if (should_forward_to_route(i, msg, target_system,
                                    target_component, &in_link)) {
            GCS_MAVLINK *out_link = gcs().chan(routes[i].channel);
            if (out_link && &in_link != out_link) {
                out_link->send_message(msg);  // Forward!
            }
        }
    }

    return process_locally;
}
```

---

## Target Extraction (routing.cpp:250-350)

### get_targets() Function

Many MAVLink messages have target fields:
```cpp
typedef struct __mavlink_param_set_t {
    float param_value;
    uint8_t target_system;     // ← Who should receive this
    uint8_t target_component;  // ← Specific component
    char param_id[16];
    uint8_t param_type;
} mavlink_param_set_t;
```

**Extraction Logic:**
```cpp
void MAVLink_routing::get_targets(const mavlink_message_t &msg,
                                  int16_t &sysid, int16_t &compid) {
    sysid = -1;   // Default: no target (broadcast)
    compid = -1;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(&msg, &packet);
            sysid = packet.target_system;
            compid = packet.target_component;
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(&msg, &packet);
            sysid = packet.target_system;
            compid = packet.target_component;
            break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
            mavlink_mission_item_int_t packet;
            mavlink_msg_mission_item_int_decode(&msg, &packet);
            sysid = packet.target_system;
            compid = packet.target_component;
            break;
        }

        // ... 50+ message types with targets ...

        default:
            // No target fields - broadcast message
            break;
    }
}
```

---

## Forwarding Decision Tree

```
                    Message Arrives
                          │
                          ▼
              ┌───────────────────────┐
              │  Is it from us?       │───Yes──► Discard
              └───────────┬───────────┘
                          │ No
                          ▼
              ┌───────────────────────┐
              │  Learn Route          │
              └───────────┬───────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Extract Target       │
              │  sysid/compid         │
              └───────────┬───────────┘
                          │
                   ┌──────┴──────┐
                   │             │
             No Target      Has Target
            (broadcast)         │
                   │            │
                   └──────┬─────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Is target us?        │
              └───────────┬───────────┘
                          │
                   ┌──────┴──────┐
                   │             │
                  Yes            No
                   │             │
                   │             ▼
                   │    ┌────────────────┐
                   │    │ Forward to     │
                   │    │ known route    │
                   │    └────────────────┘
                   │
                   ▼
              ┌───────────────────────┐
              │  Process Locally      │
              └───────────────────────┘
```

---

## Forwarding Examples

### Example 1: Parameter Request from QGC

**Setup:**
```
Channel 0: QGC (sysid=255)
Channel 1: Mission Planner (sysid=254)
Flight Controller: sysid=1
```

**Message Arrives:**
```
Channel: 0 (USB)
Message: PARAM_REQUEST_LIST
From: sysid=255, compid=190
Target: sysid=1, compid=1
```

**Router Decision:**
1. **Not from us** ✓ (we are sysid=1, message from 255)
2. **Learn route**: routes[0] = {255, 190, COMM_0}
3. **Extract target**: sysid=1, compid=1
4. **Is target us?** ✓ (we are sysid=1, compid=1)
5. **Process locally** ✓
6. **Forward?** NO (targeted specifically at us)
7. **Result:** Process parameter request, don't forward

---

### Example 2: Broadcast Message

**Message Arrives:**
```
Channel: 0 (USB)
Message: HEARTBEAT
From: sysid=255, compid=190
Target: none (broadcast)
```

**Router Decision:**
1. **Not from us** ✓
2. **Learn route**: Update routes[0]
3. **No target** → broadcast
4. **Process locally** ✓
5. **Forward to all other channels** ✓
6. **Result:**
   - Process locally (update GCS seen time)
   - Forward to Channel 1 (Mission Planner sees it)

---

### Example 3: Message for Companion Computer

**Setup:**
```
Channel 0: QGC (sysid=255)
Channel 1: RasPi (sysid=100)
```

**Message Arrives:**
```
Channel: 0 (USB)
Message: COMMAND_LONG (MAV_CMD_DO_SET_MODE)
From: sysid=255, compid=190
Target: sysid=100, compid=191
```

**Router Decision:**
1. **Not from us** ✓ (we are sysid=1)
2. **Learn route**: routes[0] = {255, 190, COMM_0}
3. **Extract target**: sysid=100, compid=191
4. **Is target us?** ✗ (we are sysid=1, target is 100)
5. **Process locally?** ✗
6. **Find route for sysid=100**: routes[1] = {100, 191, COMM_1}
7. **Forward to Channel 1** ✓
8. **Result:** Forward only, don't process

---

## Special Message Handling

### Heartbeat (routing.cpp:128-132)

```cpp
if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    handle_heartbeat(in_link, msg);
    return true;  // Always process locally
}
```

**Why Special?**
- Used for route learning
- Updates "last seen" timestamps
- Triggers GCS failsafe logic
- Should not be forwarded (too much traffic)

---

### Radio Status (routing.cpp:119-122)

```cpp
if (msg.msgid == MAVLINK_MSG_ID_RADIO ||
    msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
    return true;  // Process locally only
}
```

**Why Special?**
- Contains link-specific metrics (RSSI, noise)
- Not relevant to other channels
- Would confuse GCS if forwarded

---

## Private Channel Handling

### Private Channel Flag (routing.cpp:125-132)

```cpp
const bool from_private_channel = in_link.is_private();

if (from_private_channel) {
    // Don't forward anything from private channels
    return process_locally;
}
```

### Setting a Channel Private

```cpp
// In vehicle initialization:
GCS_MAVLINK::set_channel_private(MAVLINK_COMM_2);
```

**Effect:**
```
Channel 2 Messages:
├─ Heartbeats → Process locally only
├─ Commands to FC → Process locally only
├─ Telemetry from FC → Sent to Channel 2
└─ Messages from other channels → NOT forwarded to Channel 2
```

**Use Case: Companion Computer**
```
Channel 0: QGC (public)
Channel 1: Mission Planner (public)
Channel 2: RasPi (PRIVATE)

QGC sends PARAM_SET:
  ✓ Processed by FC
  ✓ Forwarded to Mission Planner
  ✗ NOT forwarded to RasPi (private)

RasPi sends COMMAND_LONG:
  ✓ Processed by FC
  ✗ NOT forwarded to QGC
  ✗ NOT forwarded to Mission Planner
```

---

## send_to_components() (routing.h:38, routing.cpp:400-450)

### Purpose
Send a message to all known **components** on this vehicle (not GCS).

```cpp
void MAVLink_routing::send_to_components(uint32_t msgid,
                                        const char *pkt,
                                        uint8_t pkt_len) {
    for (uint8_t i = 0; i < num_routes; i++) {
        // Skip if not our sysid
        if (routes[i].sysid != mavlink_system.sysid) {
            continue;
        }

        // Skip if it's us
        if (routes[i].compid == mavlink_system.compid) {
            continue;
        }

        // Send to this component
        GCS_MAVLINK *out_link = gcs().chan(routes[i].channel);
        if (out_link) {
            out_link->send_message(msgid, pkt);
        }
    }
}
```

### Example: Camera Trigger

**Scenario:**
```
sysid=1, compid=1: Autopilot
sysid=1, compid=100: Camera (on Telem2)
sysid=1, compid=101: Gimbal (on Telem2)
```

**Code:**
```cpp
// Trigger all cameras on this vehicle
mavlink_command_long_t cmd;
cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
cmd.target_system = mavlink_system.sysid;  // Our sysid
cmd.target_component = MAV_COMP_ID_ALL;    // All components

GCS_MAVLINK::send_to_components(
    MAVLINK_MSG_ID_COMMAND_LONG,
    (const char*)&cmd,
    sizeof(cmd)
);
```

**Result:**
- Sent to compid=100 (camera)
- Sent to compid=101 (gimbal)
- NOT sent to compid=1 (ourselves)
- NOT sent to GCS (sysid=255)

---

## Finding Devices

### find_by_mavtype() (routing.h:44)

```cpp
bool MAVLink_routing::find_by_mavtype(uint8_t mavtype,
                                      uint8_t &sysid,
                                      uint8_t &compid,
                                      mavlink_channel_t &channel) {
    for (uint8_t i = 0; i < num_routes; i++) {
        if (routes[i].mavtype == mavtype) {
            sysid = routes[i].sysid;
            compid = routes[i].compid;
            channel = routes[i].channel;
            return true;  // Found!
        }
    }
    return false;  // Not found
}
```

### Example: Find Gimbal

```cpp
uint8_t gimbal_sysid, gimbal_compid;
mavlink_channel_t gimbal_chan;

if (routing.find_by_mavtype(MAV_TYPE_GIMBAL,
                           gimbal_sysid,
                           gimbal_compid,
                           gimbal_chan)) {
    // Found gimbal! Send command to it
    GCS_MAVLINK *link = gcs().chan(gimbal_chan);
    link->send_gimbal_command(...);
} else {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "No gimbal found");
}
```

---

## Route Table Management

### Maximum Routes (routing.h:10)

```cpp
#define MAVLINK_MAX_ROUTES 20
```

**Why 20?**
Typical setup:
```
1. QGroundControl
2. Mission Planner (backup GCS)
3. Companion computer
4. Camera
5. Gimbal
6. Servo controller
7-20. Reserve for future devices
```

### Route Overflow

```cpp
if (num_routes >= MAVLINK_MAX_ROUTES) {
    // Table full! Cannot learn new route
    // Oldest routes might need manual clearing
}
```

**Mitigation:**
- Heartbeats refresh routes (prevent stale entries)
- Rebooting vehicle clears table
- Routes with same sysid/compid update channel (don't duplicate)

---

## Routing Masks

### no_route_mask (routing.h:64)

```cpp
uint8_t no_route_mask;  // Bitmask of channels to never forward to
```

**Setting:**
```cpp
// Disable routing to/from channel 2
routing.no_route_mask |= (1U << 2);
```

**Effect:**
```
Channel 2:
  ✓ Can receive direct telemetry
  ✗ Will NOT receive forwarded messages
  ✗ Messages from Ch2 will NOT be forwarded
```

**Use Case:**
Point-to-point protocols that shouldn't mix with MAVLink traffic.

---

## Complete Flow Example

### Multi-GCS Setup

**Configuration:**
```
Channel 0 (USB): QGC sysid=255
Channel 1 (Telem1): Mission Planner sysid=254
Channel 2 (Telem2): RasPi sysid=100 (PRIVATE)
```

**Scenario 1: QGC Changes Parameter**

```
1. QGC sends: PARAM_SET(target=1, param=WPNAV_SPEED, value=500)
   Channel: 0
   │
2. Router receives on Channel 0
   │
3. Learn route: routes[0] = {255, 190, COMM_0}
   │
4. Extract target: sysid=1 (us!)
   │
5. Process locally: ✓ Change parameter
   │
6. Forward decision:
   - Target is us specifically
   - Don't forward to other GCS
   │
7. Send PARAM_VALUE acknowledgment to Channel 0 only
```

**Scenario 2: FC Sends GPS Telemetry**

```
1. FC generates: GPS_RAW_INT (no target = broadcast)
   │
2. Router prepares to send
   │
3. Iterate channels:
   - Channel 0 (QGC): ✓ Send
   - Channel 1 (MP):  ✓ Send
   - Channel 2 (RasPi): ✓ Send (it's our telemetry)
   │
4. All GCS and companion get GPS data
```

**Scenario 3: QGC Sends Command to RasPi**

```
1. QGC sends: COMMAND_LONG(target_sys=100, cmd=123)
   Channel: 0
   │
2. Router receives
   │
3. Learn route (already known)
   │
4. Extract target: sysid=100 (NOT us, we're sysid=1)
   │
5. Process locally? ✗ (not for us)
   │
6. Find route for sysid=100: routes[2] = {100, 191, COMM_2}
   │
7. Forward to Channel 2 only
   │
8. RasPi receives command
```

---

## Performance Considerations

### Route Lookup Complexity
```
Linear search: O(n) where n ≤ 20
Per message: ~20 comparisons worst case
At 100 msg/sec: 2000 comparisons/sec (negligible on modern CPU)
```

### Memory Usage
```
struct route = 4 bytes × 20 routes = 80 bytes
Plus overhead: ~100 bytes total
```

### Optimization Opportunity
```cpp
// Could use hash map for O(1) lookup:
std::unordered_map<uint32_t, route> route_map;
uint32_t key = (sysid << 8) | compid;

// But 20 routes is small enough that linear search is fine
```

---

## Debugging Routing

### Enable Debug Output (routing.cpp:33)

```cpp
#define ROUTING_DEBUG 1

// Outputs:
"fwd msg 76 from chan 0 on chan 1 sysid=255 compid=190"
"route learned: sysid=100 compid=191 chan=1 type=18"
```

### Print Routing Table

```cpp
void MAVLink_routing::print_routes() {
    printf("Routing table (%d routes):\n", num_routes);
    for (uint8_t i = 0; i < num_routes; i++) {
        printf("  [%d] sysid=%d compid=%d chan=%d type=%d\n",
               i,
               routes[i].sysid,
               routes[i].compid,
               routes[i].channel,
               routes[i].mavtype);
    }
}
```

---

## Key Takeaways

1. **Automatic route learning** - no manual configuration needed
2. **Intelligent forwarding** - messages go only where needed
3. **Private channel support** - isolate companion computer traffic
4. **Prevents loops** - won't forward back to sender
5. **Component discovery** - find devices by MAV_TYPE
6. **Broadcast support** - messages with no target go everywhere
7. **Lightweight** - only 80 bytes RAM, O(n) with small n
8. **Essential for multi-GCS** - enables complex ground station setups

The routing system is what makes **multi-GCS operations** and **companion computers** practical and reliable in ArduPilot!
