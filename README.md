# f450_drone_ws

ROS 2 workspace for F450 drone demos:
- `src/f450_bridges`: MAVROS-based trajectory visualization (RViz2)
- More packages to come

## run telemetry
ros2 run mavros mavros_node --ros-args   -p fcu_url:=udp://127.0.0.1:14660@0.0.0.0:14661?mav10=1   -p system_id:=255 -p component_id:=190   -p target_system_id:=1 -p target_component_id:=1


# Problem 1 - communication problem: Mavros not connecting to the ESP
Telemetry data are sent by an ESP32 board connected to the Pixhawk APM 2.8 (FCU).
while Mission Planner worked correctly, Ros2 Mavlink Bridge (MAVROS) was not initializing correctly the ROS2 topics. Wireshark and `tcpdump` revealed that packets were being **broadcast**, not **unicast**.

---

## ⚙️ Hardware and Software Setup

| Component | Role |
|------------|------|
| **Pixhawk APM 2.8** | Flight Control Unit (FCU) running ArduCopter 3.2.1 |
| **ESP8266 Wi-Fi Bridge (v1.2.8)** | UART ↔ UDP MAVLink converter |
| **Ubuntu 22.04 Laptop** | Running ROS 2 Humble and MAVROS |
| **Mission Planner** | Reference GCS (for baseline testing) |

---

- **Mission Planner:** OK  
- **MAVROS:** `connected: false`  
- **UDP packets:** seen from `192.168.4.1 → 192.168.4.255:14550`  
- **No response:** from MAVROS → Pixhawk  

Root cause:  
> The ESP8266 bridge **only broadcasted** MAVLink packets and didn’t accept **unicast** replies.  
> MAVROS expects a unicast reply for its initial heartbeat and version handshake — which never arrived.

---

## ✅ Solution Overview

We introduced a **bidirectional UDP relay** using `socat`, which forces MAVLink traffic to flow through broadcast in both directions.

ESP → 14550 → socket Relay (RX) → 14660 →  MAVROS → 14661 → socket relay (TX) → 14550  

Socket relay (RX) converts UDP packet type from **BROADCAST** to **UNICAST**.
Socket relay (TX) is optional.                                     

### commands (in the right order)

**Terminal A**
**1** check WIFI connection with the ESP
`ping 192.168.4.1`

**2** Start MAVROS first (so it binds @0.0.0.0:14661 and listens on 127.0.0.1:14660):
`ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14660@0.0.0.0:14661?mav10=1`

**Terminal B**
**3** Verify sockets are up (you should see 14661 “UNCONN” and 14660 “UNCONN”):
`sudo lsof -nP -iUDP | egrep ':(14660|14661)\b'`

**Terminal B**
**4** Start RX relay (broadcast from ESP (14550) → MAVROS loopback (14660)):
`sudo socat -u UDP-RECV:14550,so-broadcast,reuseaddr,bind=0.0.0.0 UDP:127.0.0.1:14660`

**Terminal C**
**5** Start TX relay (MAVROS loopback → broadcast to ESP):
`sudo socat -u UDP-RECV:14661,bind=127.0.0.1 UDP-SENDTO:192.168.4.255:14550,broadcast,bind=192.168.4.2:0` 

# Problem 2 - no telemetry data sent, ArduPilot's Default Behavior

**Symptom** run the command:
`sudo tcpdump -i wlp3s0 -n udp port 14550`

If it returns :

tcpdump: verbose output suppressed, use -v[v]... for full protocol decode
listening on wlp3s0, link-type EN10MB (Ethernet), snapshot length 262144 bytes \
10:18:10.018342 IP 192.168.4.1.14555 > 192.168.4.255.14550: UDP, length 17
10:18:11.041897 IP 192.168.4.1.14555 > 192.168.4.255.14550: UDP, length 17
10:18:12.066378 IP 192.168.4.1.14555 > 192.168.4.255.14550: UDP, length 17

then it means that the PC is receiving only hearbeats from the ESC not telemetry data.

**Root cause** ArduPilot (and most autopilots) don't automatically stream all telemetry data. Here's why:
Before the command:

Limited bandwidth conservation: ArduPilot assumes the telemetry link might be slow (like 57600 baud radio), so it only sends minimal data by default - just heartbeats to maintain connection
Your APM 2.8 was only sending:

HEARTBEAT messages (17 bytes, every ~2 seconds)
Basic system status to show "I'm alive and in STABILIZE mode"

No sensor data was streaming: No IMU, GPS, attitude, RC channels, etc.

This is why you see:

✅ /mavros/state topic working (derived from heartbeat)
❌ /mavros/imu/data not publishing
❌ /mavros/heartbeat not publishing (ironically, because MAVRos wasn't getting enough data to republish it)

## ✅ Solution Overview

Call a Mavros service that will enable telemetry data publishing from the FCU: 
`ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"`

This sent a **MAVLink REQUEST_DATA_STREAM** message to ArduPilot FCU with:
- `stream_id: 0` = **ALL streams** (sensors, GPS, attitude, RC, etc.)
- `message_rate: 10` = Send at **10 Hz** (10 times per second)
- `on_off: true` = **Enable** streaming

### After the command:
ArduPilot started flooding the telemetry with:
- **RAW_IMU** (accelerometer, gyro, magnetometer)
- **SCALED_PRESSURE** (barometer)
- **ATTITUDE** (roll, pitch, yaw)
- **GLOBAL_POSITION_INT** (GPS)
- **VFR_HUD** (airspeed, altitude, climb rate)
- **SYS_STATUS** (battery, sensors)
- **RC_CHANNELS** (radio control inputs)
- And many more...

This is why you suddenly saw packets of 19, 38, 56, 61, 64, 72 bytes - each different size represents a different MAVLink message type with sensor data.

## Why This Isn't Automatic

**Design philosophy:**
1. **Bandwidth-constrained links**: Traditional telemetry radios (433 MHz, 915 MHz) are slow - autopilots can't just spam data
2. **Ground station driven**: The ground control station (Mission Planner, QGroundControl, MAVRos) should **request what it needs**
3. **Power efficiency**: On battery-powered systems, unnecessary transmissions waste power

**In Mission Planner**, this happens automatically when you connect - it requests all streams. But with **MAVRos**, you need to either:
- Call the service manually (as you did)
- Configure stream rates in ArduPilot parameters (SR0_*, SR1_*, etc.) to stream by default
- Add the service call to your launch file so it runs automatically

## The Stream ID Breakdown

stream_id: 0  → ALL streams
stream_id: 1  → RAW_SENSORS (IMU, barometer, magnetometer)
stream_id: 2  → EXTENDED_STATUS (mode, battery, failsafes)
stream_id: 3  → RC_CHANNELS (radio inputs)
stream_id: 6  → POSITION (GPS, altitude)
stream_id: 10 → EXTRA1 (attitude, PID)
stream_id: 11 → EXTRA2 (VFR_HUD)
stream_id: 12 → EXTRA3 (AHRS, wind)

# Problem 3: Pitch inverted in sign, NED vs ENU Coordinates

Root cause:
ArduPilot uses NED (North-East-Down):

Nose up = negative pitch
Nose down = positive pitch
Roll right = positive roll

ROS2 uses ENU (East-North-Up) by convention:

Nose up = positive pitch
Nose down = negative pitch
Roll right = positive roll



# ESP32 Bridge Code - Data Flow & Protocol Explanation
What This Code Does:
It's a TRANSPARENT SERIAL ↔ UDP BRIDGE
Think of it as a "dumb pipe" - it doesn't understand, parse, or modify MAVLink at all.

## Data Flow:
### Direction 1: APM → Laptop
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
APM 2.8 (Serial TX) 
    ↓ [MAVLink v1 bytes: 0xFE 0x09 0x89...]
ESP32 UART RX (GPIO16) receives bytes
    ↓ [stores in buffer]
ESP32 reads from Serial2.available()
    ↓ [doesn't parse - just raw bytes]
ESP32 sends via UDP (WiFi)
    ↓ [broadcasts to 192.168.4.255:14550 or unicast if GCS known]
Laptop receives UDP packet
    ↓ [same bytes: 0xFE 0x09 0x89...]
MAVRos parses MAVLink v1 packets
    ↓ [converts to ROS topics]
ROS topics: /mavros/imu/data, /mavros/state, etc.


### Direction 2: Laptop → APM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MAVRos generates MAVLink v2 command
    ↓ [MAVLink v2 bytes: 0xFD 0x09...]
Laptop sends UDP packet to ESP32:14550
    ↓ [ESP32 learns laptop IP from this packet]
ESP32 receives UDP packet
    ↓ [doesn't parse - just raw bytes]
ESP32 writes to Serial2 (GPIO17)
    ↓ [sends bytes to APM UART RX]
APM 2.8 receives on Serial RX
    ↓ [APM parses the packet]
APM processes command (if it understands it)


## Protocol & Dialect Handling:
CRITICAL POINT: The ESP32 code is PROTOCOL-AGNOSTIC
┌─────────────────────────────────────────┐
│  ESP32 Bridge Code                      │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━    │
│  Does NOT know about:                   │
│  ❌ MAVLink v1 vs v2                    │
│  ❌ Packet structure                    │
│  ❌ Message IDs                         │
│  ❌ Dialects (common/ardupilotmega)     │
│  ❌ Checksums                           │
│                                         │
│  Only knows:                            │
│  ✅ Read bytes from UART                │
│  ✅ Send bytes to UDP                   │
│  ✅ Read bytes from UDP                 │
│  ✅ Send bytes to UART                  │
│                                         │
│  It's a TRANSPARENT BYTE PIPE           │
└─────────────────────────────────────────┘
### What Actually Handles Protocol/Dialect:
APM 2.8 Firmware:
  • Generates MAVLink v1 packets
  • Uses ardupilotmega dialect
  • Sends: HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS, etc.

       ↕ [raw bytes through ESP32]

### MAVRos on Laptop:
  • Receives packets (v1 or v2 - it handles both)
  • Parses based on packet structure
  • Converts to ROS2 topics
  • Understands ardupilotmega dialect messages

### Discovery Process:
Initial State:
  ESP32 doesn't know laptop IP
  → Broadcasts MAVLink packets to 192.168.4.255

Laptop Sends First Packet:
  MAVRos → UDP packet to ESP32
  ESP32 learns: "Aha! Laptop is at 192.168.4.2:14550"

Locked State:
  ESP32 → unicast to 192.168.4.2:14550 (faster, more reliable)

### Mental Picture
✅ APM generates MAVLink v1 (with ardupilotmega dialect messages)
✅ ESP32 doesn't care - just moves bytes
✅ MAVRos receives MAVLink v1 packets over UDP
✅ MAVRos parses v1 (backward compatible with v2 parser)
✅ MAVRos publishes /mavros/imu/data with roll/pitch/yaw
✅ You subscribe to ROS topics for trajectory building