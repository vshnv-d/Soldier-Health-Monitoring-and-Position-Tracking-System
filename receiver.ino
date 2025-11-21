#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <LoRa.h>
#include <base64.h>

// WiFi credentials
const char* ssid = "Samsung S20 FE 5G";
const char* password = "Vedanti9";

// Twilio credentials
const char* account_sid = "AC435487ffe2437f2839b067ab6d839472";
const char* auth_token = "e60944d123067f3ba092942cf0357d07";
const char* from_number = "+17166412805";  // Include country code
const char* to_number = "+918669955660";   // Include country code

// Pin mapping for LoRa module
#define SS 15
#define RST 16
#define DIO0 2
#define LORA_FREQ 433E6

// Timing control
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 30000; // 30 seconds

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // LoRa initialization
  Serial.println("Initializing LoRa Receiver...");
  LoRa.setPins(SS, RST, DIO0);
  
  // Set basic LoRa parameters
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  
  if (!LoRa.begin(433E6)) {  // Use explicit frequency
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  Serial.println("LoRa Receiver Ready");
  Serial.println("Listening on 433MHz...");
}

void sendTwilioSMS(String message) {
  WiFiClientSecure client;
  client.setInsecure(); // For development. Use proper certificate verification in production

  const char* host = "api.twilio.com";
  const int httpsPort = 443;

  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection to Twilio failed!");
    return;
  }

  // Create the POST data
  String postData = "To=" + urlEncode(String(to_number)) +
                   "&From=" + urlEncode(String(from_number)) +
                   "&Body=" + urlEncode(message);

  // Create authorization header
  String auth = base64::encode(String(account_sid) + ":" + String(auth_token));

  // Create the HTTP request
  String request = "POST /2010-04-01/Accounts/" + String(account_sid) + "/Messages.json HTTP/1.1\r\n";
  request += "Host: api.twilio.com\r\n";
  request += "Authorization: Basic " + auth + "\r\n";
  request += "Content-Type: application/x-www-form-urlencoded\r\n";
  request += "Content-Length: " + String(postData.length()) + "\r\n";
  request += "Connection: close\r\n\r\n";
  request += postData;

  // Send the request
  client.print(request);
  
  // Wait for response
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      break;
    }
  }

  Serial.println("SMS sent successfully!");
}

String urlEncode(String str) {
  String encodedString = "";
  char c;
  char code0;
  char code1;

  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (isAlphaNumeric(c)) {
      encodedString += c;
    } else if (c == ' ') {
      encodedString += '+';
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
    }
  }
  return encodedString;
}

void processAndSendData(String firstPacket, String secondPacket) {
  // Check if this is an SOS alert
  bool isSOS = (firstPacket.indexOf("SOS!") != -1);
  
  // Extract GPS data if available
  String gpsData = "";
  int gpsIndex = firstPacket.indexOf("GPS:");
  if (gpsIndex != -1) {
    gpsData = firstPacket.substring(gpsIndex);
  }
  
  // For SOS alerts, send both messages immediately
  if (isSOS) {
    // First message: Soldier info and location
    String sosMessage1 = "🚨 EMERGENCY ALERT 🚨\n";
    sosMessage1 += firstPacket;
    if (gpsData != "" && gpsData.indexOf("NO LOCK") == -1) {
      sosMessage1 += "\n\n📍 Location:\n";
      sosMessage1 += "https://maps.google.com/maps?q=";
      sosMessage1 += gpsData.substring(4); // Remove "GPS:" prefix
    }
    sendTwilioSMS(sosMessage1);
    delay(1000); // Wait 1 second between messages
    
    // Second message: Health parameters
    String sosMessage2 = "🚨 VITAL SIGNS 🚨\n" + secondPacket;
    sendTwilioSMS(sosMessage2);
    lastSendTime = millis(); // Update last send time
  }
  // For regular updates, check the interval
  else if (millis() - lastSendTime >= sendInterval) {
    // First message: Soldier info and location
    String statusMessage1 = "📍 SOLDIER INFO 📍\n";
    statusMessage1 += firstPacket;
    if (gpsData != "" && gpsData.indexOf("NO LOCK") == -1) {
      statusMessage1 += "\n\n📍 Location:\n";
      statusMessage1 += "https://maps.google.com/maps?q=";
      statusMessage1 += gpsData.substring(4); // Remove "GPS:" prefix
    }
    sendTwilioSMS(statusMessage1);
    delay(1000); // Wait 1 second between messages
    
    // Second message: Health parameters
    String statusMessage2 = "📊 VITAL SIGNS 📊\n" + secondPacket;
    sendTwilioSMS(statusMessage2);
    lastSendTime = millis(); // Update last send time
  }
}

void loop() {
  static String firstPacket = "";
  static unsigned long firstPacketTime = 0;
  const unsigned long PACKET_TIMEOUT = 2000; // 2 seconds timeout for second packet
  
  // Check for incoming packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";
    
    Serial.println("\nReceived packet!");
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
    Serial.print("SNR: ");
    Serial.println(LoRa.packetSnr());
    
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    
    Serial.println("Received data: ");
    Serial.println(receivedData);
    
    // If this is the first packet or timeout occurred
    if (firstPacket == "" || (millis() - firstPacketTime > PACKET_TIMEOUT)) {
      firstPacket = receivedData;
      firstPacketTime = millis();
      Serial.println("Stored first packet, waiting for second...");
    } else {
      // This is the second packet, process both packets
      Serial.println("Received second packet, processing both...");
      processAndSendData(firstPacket, receivedData);
      firstPacket = ""; // Reset for next pair
    }
  } else {
    // Print status every 10 seconds
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 10000) {
      Serial.println("Waiting for packets...");
      if (firstPacket != "") {
        Serial.println("Still waiting for second packet...");
      }
      lastStatusPrint = millis();
    }
  }
}