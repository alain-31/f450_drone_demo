#include <WiFi.h>
#include <WiFiUdp.h>

// ============ CONFIGURATION - EDIT THESE ============
// Access Point mode - ESP32 creates WiFi hotspot
const char* ap_ssid = "DRONE_LINK";        // Change to your preferred WiFi name
const char* ap_password = "12345678";      // Change password (min 8 characters)

#define UART_BAUD 57600      // Match APM baud rate (usually 57600 or 115200)
#define UDP_PORT 14550       // MAVLink ground station port
#define REMOTE_UDP_PORT 14555 // MAVLink remote port

// ESP32 UART2 pins - Connect through level shifter to APM
#define RXD2 16  // ESP32 RX ← APM TX (through level shifter)
#define TXD2 17  // ESP32 TX → APM RX (through level shifter)
// ====================================================

WiFiUDP udp;
IPAddress remoteIP;
uint16_t remotePort = 0;
bool hasRemoteIP = false;

uint8_t buf[512];

void setup() {
  // Debug serial (USB)
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=================================");
  Serial.println("ESP32 MAVLink WiFi Bridge - AP Mode");
  Serial.println("=================================");
  
  // Initialize UART2 for APM communication
  Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("✓ UART2 initialized");
  Serial.printf("  Baud rate: %d\n", UART_BAUD);
  Serial.printf("  RX pin: GPIO%d\n", RXD2);
  Serial.printf("  TX pin: GPIO%d\n", TXD2);
  
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
  Serial.println("Ready for MAVLink traffic!");
  Serial.println("Connect your laptop to WiFi:");
  Serial.printf("  Network: %s\n", ap_ssid);
  Serial.println("=================================\n");
}

void loop() {
  // ===== APM → UART → ESP32 =====
  if (Serial2.available()) {
    int len = Serial2.available();
    if (len > sizeof(buf)) len = sizeof(buf);
    
    int bytesRead = Serial2.readBytes(buf, len);
    
    // ===== ESP32 → UDP → Ground Station =====
    if (hasRemoteIP) {
      // Send to known ground station (unicast)
      udp.beginPacket(remoteIP, remotePort);
      udp.write(buf, bytesRead);
      udp.endPacket();
    } else {
      // Broadcast discovery - send to subnet broadcast
      IPAddress broadcastIP(192, 168, 4, 255);
      udp.beginPacket(broadcastIP, UDP_PORT);
      udp.write(buf, bytesRead);
      udp.endPacket();
    }
  }
  
  // ===== ESP32 <- UDP <- Ground Station =====
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
    
    // ===== APM <- UART <- ESP32 =====
    // Read packet and forward to APM via UART
    int len = udp.read(buf, sizeof(buf));
    if (len > 0) {
      Serial2.write(buf, len);
    }
  }
  
  // Small delay to prevent watchdog timer issues
  delay(1);
}