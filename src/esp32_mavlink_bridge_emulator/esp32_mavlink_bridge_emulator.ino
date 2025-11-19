#include <WiFi.h>
#include <WiFiUdp.h>

// ============ CONFIGURATION - EDIT THESE ============
// Access Point mode - ESP32 creates WiFi hotspot
const char* ap_ssid = "DRONE_LINK";        // Change to your preferred WiFi name
const char* ap_password = "12345678";      // Change password (min 8 characters)

#define UDP_PORT 14550       // MAVLink ground station port

// MAVLink System Configuration (emulating APM2.8)
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1
#define MAVLINK_TYPE 2  // MAV_TYPE_QUADROTOR (use 1 for fixed wing)

// ====================================================

WiFiUDP udp;
IPAddress remoteIP;
uint16_t remotePort = 0;
bool hasRemoteIP = false;

uint8_t buf[512];

// Timing variables
unsigned long lastHeartbeat = 0;
unsigned long lastAttitude = 0;
unsigned long pauseStartTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;  // 1 Hz (1000ms)
const unsigned long ATTITUDE_INTERVAL = 250;    // 4 Hz 

// Pause durations (in milliseconds)
const unsigned long PAUSE_ROLL_TO_PITCH = 2500;   // 0.5s
const unsigned long PAUSE_PITCH_TO_YAW = 2500;    // 0.5s
const unsigned long PAUSE_YAW_TO_ROLL = 3500;    // 1s

// State machine
enum DemoState {
  STATE_ROLL_UP,
  STATE_ROLL_DOWN,
  STATE_PAUSE_ROLL_PITCH,
  STATE_PITCH_UP,
  STATE_PITCH_DOWN,
  STATE_PAUSE_PITCH_YAW,
  STATE_YAW_UP,
  STATE_YAW_DOWN,
  STATE_PAUSE_YAW_ROLL
};

DemoState currentState = STATE_ROLL_UP;

// Pitch simulation
float currentPitch = 0.0;  // In radians
float pitchDegrees = 0;      // In degrees (0-10)

// Roll simulation
float currentRoll = 0.0;  // In radians
float rollDegrees = 0;      // In degrees (0-10)

// Yaw simulation
float currentYaw = 0.0;  // In radians
float yawDegrees = 0;      // In degrees (0-10)

// MAVLink sequence number
uint8_t mavlink_seq = 0;

// MAVLink message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_ATTITUDE 30

// MAVLink protocol constants
#define MAVLINK_STX 0xFE
#define MAVLINK_CRC_EXTRA_HEARTBEAT 50
#define MAVLINK_CRC_EXTRA_ATTITUDE 39

// CRC calculation (X.25)
uint16_t crc_calculate(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    uint8_t tmp = data[i] ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
  }
  return crc;
}

void crc_accumulate(uint8_t data, uint16_t *crc) {
  uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
  tmp ^= (tmp << 4);
  *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// Send MAVLink HEARTBEAT message
void sendHeartbeat() {
  uint8_t packet[17];  // Header(6) + payload(9) + checksum(2)
  uint8_t idx = 0;
  
  // Header
  packet[idx++] = MAVLINK_STX;           // STX
  packet[idx++] = 9;                     // Payload length
  packet[idx++] = mavlink_seq++;         // Sequence
  packet[idx++] = MAVLINK_SYSTEM_ID;     // System ID
  packet[idx++] = MAVLINK_COMPONENT_ID;  // Component ID
  packet[idx++] = MAVLINK_MSG_ID_HEARTBEAT; // Message ID
  
  // Payload (9 bytes)
  uint32_t custom_mode = 0;
  packet[idx++] = custom_mode & 0xFF;
  packet[idx++] = (custom_mode >> 8) & 0xFF;
  packet[idx++] = (custom_mode >> 16) & 0xFF;
  packet[idx++] = (custom_mode >> 24) & 0xFF;
  
  packet[idx++] = MAVLINK_TYPE;          // type (MAV_TYPE)
  packet[idx++] = 3;                     // autopilot (MAV_AUTOPILOT_ARDUPILOTMEGA)
  packet[idx++] = 192;                   // base_mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
  packet[idx++] = 3;                     // system_status (MAV_STATE_STANDBY)
  packet[idx++] = 3;                     // mavlink_version
  
  // Calculate CRC
  uint16_t crc = 0xFFFF;
  for (int i = 1; i < idx; i++) {
    crc_accumulate(packet[i], &crc);
  }
  crc_accumulate(MAVLINK_CRC_EXTRA_HEARTBEAT, &crc);
  
  packet[idx++] = crc & 0xFF;
  packet[idx++] = (crc >> 8) & 0xFF;
  
  // Send packet
  if (hasRemoteIP) {
    udp.beginPacket(remoteIP, remotePort);
    udp.write(packet, idx);
    udp.endPacket();
  } else {
    // Broadcast
    IPAddress broadcastIP(192, 168, 4, 255);
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write(packet, idx);
    udp.endPacket();
  }
  
  Serial.print("Sent HEARTBEAT");
  Serial.println();
}

// Send MAVLink ATTITUDE message
void sendAttitude() {
  uint8_t packet[36];  // Header(6) + payload(28) + checksum(2)
  uint8_t idx = 0;
  
  // Header
  packet[idx++] = MAVLINK_STX;           // STX
  packet[idx++] = 28;                    // Payload length
  packet[idx++] = mavlink_seq++;         // Sequence
  packet[idx++] = MAVLINK_SYSTEM_ID;     // System ID
  packet[idx++] = MAVLINK_COMPONENT_ID;  // Component ID
  packet[idx++] = MAVLINK_MSG_ID_ATTITUDE; // Message ID
  
  // Payload (28 bytes)
  // time_boot_ms (uint32_t)
  uint32_t time_boot_ms = millis();
  memcpy(&packet[idx], &time_boot_ms, 4);
  idx += 4;
  
  // roll (float, radians)
  float roll = 0.0f;
  memcpy(&packet[idx], &currentRoll, 4);
  idx += 4;
  
  // pitch (float, radians)
  memcpy(&packet[idx], &currentPitch, 4);
  idx += 4;
  
  // yaw (float, radians)
  float yaw = 0.0f;
  memcpy(&packet[idx], &currentYaw, 4);
  idx += 4;
  
  // rollspeed (float, rad/s)
  float rollspeed = 0.0f;
  memcpy(&packet[idx], &rollspeed, 4);
  idx += 4;
  
  // pitchspeed (float, rad/s)
  float pitchspeed = 0.0f;
  memcpy(&packet[idx], &pitchspeed, 4);
  idx += 4;
  
  // yawspeed (float, rad/s)
  float yawspeed = 0.0f;
  memcpy(&packet[idx], &yawspeed, 4);
  idx += 4;
  
  // Calculate CRC
  uint16_t crc = 0xFFFF;
  for (int i = 1; i < idx; i++) {
    crc_accumulate(packet[i], &crc);
  }
  crc_accumulate(MAVLINK_CRC_EXTRA_ATTITUDE, &crc);
  
  packet[idx++] = crc & 0xFF;
  packet[idx++] = (crc >> 8) & 0xFF;
  
  // Send packet
  if (hasRemoteIP) {
    udp.beginPacket(remoteIP, remotePort);
    udp.write(packet, idx);
    udp.endPacket();
  } else {
    // Broadcast
    IPAddress broadcastIP(192, 168, 4, 255);
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write(packet, idx);
    udp.endPacket();
  }
  
Serial.print("ATTITUDE: roll=");
Serial.print(rollDegrees);
Serial.print(" pitch=");
Serial.print(pitchDegrees);
Serial.print(" yaw=");
Serial.print(yawDegrees);
Serial.print(" state=");
Serial.println(currentState);
}

void setup() {
  // Debug serial (USB)
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=================================");
  Serial.println("ESP32 MAVLink Emulator - APM2.8");
  Serial.println("=================================");
  
  // Create WiFi Access Point
  Serial.println("\nStarting Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.println("✓ Access Point started");
  Serial.printf("  SSID: %s\n", ap_ssid);
  Serial.printf("  Password: %s\n", ap_password);
  Serial.printf("  IP address: %s\n", apIP.toString().c_str());
  
  // Start UDP
  udp.begin(UDP_PORT);
  Serial.printf("✓ UDP listening on port %d\n", UDP_PORT);
  
  Serial.println("\n=================================");
  Serial.println("Emulating APM2.8 MAVLink traffic");
  Serial.println("Connect your laptop to WiFi:");
  Serial.printf("  Network: %s\n", ap_ssid);
  Serial.println("=================================\n");
  
  // Initialize timing
  lastHeartbeat = millis();
  lastAttitude = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send HEARTBEAT at 1 Hz
  if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = currentTime;
  }
  
  if (currentTime - lastAttitude >= ATTITUDE_INTERVAL) {
    sendAttitude();
    lastAttitude = currentTime;
  }

  switch (currentState) {
    case STATE_ROLL_UP:
      rollDegrees += 0.1;
      if (rollDegrees >= 15) {
        currentState = STATE_ROLL_DOWN;
        Serial.println(">> Roll down");
      }
      break;
      
    case STATE_ROLL_DOWN:
      rollDegrees -= 0.1;
      if (rollDegrees <= 0) {
        rollDegrees = 0;
        pauseStartTime = currentTime;
        currentState = STATE_PAUSE_ROLL_PITCH;
        Serial.println(">> Pause before Pitch");
      }
      break;
      
    case STATE_PAUSE_ROLL_PITCH:
      if (currentTime - pauseStartTime >= PAUSE_ROLL_TO_PITCH) {
        currentState = STATE_PITCH_UP;
        Serial.println(">> Pitch up");
      }
      break;
      
    case STATE_PITCH_UP:
      pitchDegrees += 0.1;
      if (pitchDegrees >= 15) {
        currentState = STATE_PITCH_DOWN;
        Serial.println(">> Pitch down");
      }
      break;
      
    case STATE_PITCH_DOWN:
      pitchDegrees -= 0.1;
      if (pitchDegrees <= 0) {
        pitchDegrees = 0;
        pauseStartTime = currentTime;
        currentState = STATE_PAUSE_PITCH_YAW;
        Serial.println(">> Pause before Yaw");
      }
      break;
      
    case STATE_PAUSE_PITCH_YAW:
      if (currentTime - pauseStartTime >= PAUSE_PITCH_TO_YAW) {
        currentState = STATE_YAW_UP;
        Serial.println(">> Yaw up");
      }
      break;
      
    case STATE_YAW_UP:
      yawDegrees += 0.5;
      if (yawDegrees >= 90) {
        currentState = STATE_YAW_DOWN;
        Serial.println(">> Yaw down");
      }
      break;
      
    case STATE_YAW_DOWN:
      yawDegrees -= 0.5;
      if (yawDegrees <= 0) {
        yawDegrees = 0;
        pauseStartTime = currentTime;
        currentState = STATE_PAUSE_YAW_ROLL;
        Serial.println(">> Pause before next cycle");
      }
      break;
      
    case STATE_PAUSE_YAW_ROLL:
      if (currentTime - pauseStartTime >= PAUSE_YAW_TO_ROLL) {
        currentState = STATE_ROLL_UP;
        Serial.println(">> Roll up (new cycle)");
      }
      break;
  }

  // Convert to radians
  currentPitch = pitchDegrees * 0.0174533f;
  currentRoll = rollDegrees * 0.0174533f;
  currentYaw = yawDegrees * 0.0174533f;
  

  // Listen for incoming UDP packets (ground station commands)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Learn remote IP from first packet (discovery)
    if (!hasRemoteIP) {
      remoteIP = udp.remoteIP();
      remotePort = udp.remotePort();
      hasRemoteIP = true;
      
      Serial.println("\n>>> Ground Station Connected <<<");
      Serial.print("  IP: ");
      Serial.print(remoteIP);
      Serial.print(":");
      Serial.println(remotePort);
      Serial.println();
    }
    
    // Read and discard (we're just emulating, not responding to commands)
    int len = udp.read(buf, sizeof(buf));
    Serial.printf("Received %d bytes from ground station\n", len);
  }
  
  // Small delay to prevent watchdog timer issues
  delay(10);
}