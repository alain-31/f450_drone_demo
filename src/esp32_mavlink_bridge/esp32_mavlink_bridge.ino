#include <WiFi.h>
#include <WiFiUdp.h>

// ============ CONFIGURATION ============
const char* ap_ssid = "DRONE_LINK";
const char* ap_password = "12345678";

#define UDP_PORT 14550
#define UART_BAUD 57600
#define RX_PIN 16
#define TX_PIN 17
// =======================================

WiFiUDP udp;
IPAddress remoteIP;
uint16_t remotePort = 0;
bool hasRemoteIP = false;

// Packet buffer
uint8_t packetBuffer[280];
uint16_t bufferIndex = 0;
uint16_t expectedPacketLength = 0;

uint8_t udpBuffer[280];

// Statistics
unsigned long bytesReceived = 0;
unsigned long packetsCompleted = 0;
unsigned long packetsSent = 0;
unsigned long bytesFromGCS = 0;
unsigned long packetsToAPM = 0;
unsigned long lastStatsTime = 0;

// Debug
unsigned long invalidStarts = 0;
unsigned long lastDebugTime = 0;

void sendPacket() {
  if (bufferIndex == 0) return;
  
  // Debug first few packets
  if (packetsSent < 5) {
    Serial.printf("Sending packet #%lu: length=%d, expected=%d\n", 
                  packetsSent + 1, bufferIndex, expectedPacketLength);
    Serial.print("  First bytes: ");
    for (int i = 0; i < min(10, (int)bufferIndex); i++) {
      Serial.printf("%02X ", packetBuffer[i]);
    }
    Serial.println();
  }
  
  if (hasRemoteIP) {
    udp.beginPacket(remoteIP, remotePort);
    udp.write(packetBuffer, bufferIndex);
    udp.endPacket();
  } else {
    IPAddress broadcastIP(192, 168, 4, 255);
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write(packetBuffer, bufferIndex);
    udp.endPacket();
  }
  
  packetsSent++;
  bufferIndex = 0;
  expectedPacketLength = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=================================");
  Serial.println("ESP32 MAVLink Bridge - DEBUG");
  Serial.println("=================================");
  
  Serial2.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.printf("✓ UART: %d baud\n", UART_BAUD);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.printf("✓ WiFi: %s\n", WiFi.softAPIP().toString().c_str());
  
  udp.begin(UDP_PORT);
  Serial.printf("✓ UDP: port %d\n\n", UDP_PORT);
  Serial.println("Waiting for MAVLink packets...\n");
  
  lastStatsTime = millis();
  lastDebugTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // ======== UART → UDP ========
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    bytesReceived++;
    
    // Looking for packet start
    if (bufferIndex == 0) {
      if (byte == 0xFE) {  // MAVLink v1 magic byte
        packetBuffer[bufferIndex++] = byte;
        expectedPacketLength = 0;
        
        // Debug first packet start
        if (packetsCompleted == 0) {
          Serial.println("First 0xFE detected!");
        }
      } else {
        invalidStarts++;
        // Show first few invalid bytes
        if (invalidStarts <= 10) {
          Serial.printf("Ignoring non-MAVLink byte: 0x%02X\n", byte);
        }
      }
      continue;
    }
    
    // Add byte to buffer
    packetBuffer[bufferIndex++] = byte;
    
    // After magic (0) and length (1), calculate expected length
    if (bufferIndex == 2) {
      uint8_t payloadLength = packetBuffer[1];
      expectedPacketLength = 6 + payloadLength + 2;
      
      // Debug first few packets
      if (packetsCompleted < 5) {
        Serial.printf("Packet detected: payload_len=%d, total_len=%d\n", 
                      payloadLength, expectedPacketLength);
      }
      
      // Sanity check
      if (expectedPacketLength > sizeof(packetBuffer) || expectedPacketLength < 8) {
        Serial.printf("ERROR: Invalid length %d (payload=%d), resetting\n", 
                      expectedPacketLength, payloadLength);
        bufferIndex = 0;
        expectedPacketLength = 0;
        continue;
      }
    }
    
    // Check if packet is complete
    if (expectedPacketLength > 0 && bufferIndex >= expectedPacketLength) {
      packetsCompleted++;
      sendPacket();
    }
    
    // Safety
    if (bufferIndex >= sizeof(packetBuffer)) {
      Serial.println("ERROR: Buffer overflow!");
      bufferIndex = 0;
      expectedPacketLength = 0;
    }
  }
  
  // ======== UDP → UART ========
  int packetSize = udp.parsePacket();
  if (packetSize) {
    if (!hasRemoteIP) {
      remoteIP = udp.remoteIP();
      remotePort = udp.remotePort();
      hasRemoteIP = true;
      Serial.println("\n>>> GCS Connected <<<");
      Serial.printf("  %s:%d\n\n", remoteIP.toString().c_str(), remotePort);
    }
    
    int len = udp.read(udpBuffer, sizeof(udpBuffer));
    if (len > 0) {
      Serial2.write(udpBuffer, len);
      bytesFromGCS += len;
      packetsToAPM++;
    }
  }
  
  // ======== Debug output every 2s ========
  if (currentTime - lastDebugTime >= 2000) {
    Serial.printf("[%lu] Bytes: %lu, Complete pkts: %lu, Sent: %lu, Buffer: %d/%d\n",
                  currentTime/1000, bytesReceived, packetsCompleted, packetsSent,
                  bufferIndex, expectedPacketLength);
    lastDebugTime = currentTime;
  }
  
  // ======== Statistics every 5s ========
  if (currentTime - lastStatsTime >= 5000) {
    Serial.println("\n--- Stats (5s) ---");
    Serial.printf("Bytes received: %lu\n", bytesReceived);
    Serial.printf("Packets completed: %lu\n", packetsCompleted);
    Serial.printf("UDP packets sent: %lu\n", packetsSent);
    Serial.printf("Invalid starts: %lu\n", invalidStarts);
    Serial.printf("GCS→APM: %lu bytes, %lu pkts\n", bytesFromGCS, packetsToAPM);
    Serial.printf("Status: %s\n", hasRemoteIP ? remoteIP.toString().c_str() : "Broadcasting");
    Serial.println("------------------\n");
    
    bytesReceived = 0;
    packetsCompleted = 0;
    packetsSent = 0;
    invalidStarts = 0;
    bytesFromGCS = 0;
    packetsToAPM = 0;
    lastStatsTime = currentTime;
  }
  
  delay(1);
}