#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADXL345_U.h>

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MAX30102 sensor
MAX30105 particleSensor;

// GPS module
SoftwareSerial gpsSerial(6, 7);  // GPS TX -> 6, RX -> 7
TinyGPSPlus gps;

// LoRa setup
#define SS_PIN 10
#define RST_PIN 9
#define DIO0_PIN 2

// Temperature sensor
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// UV sensor
#define UV_SENSOR_PIN A0

// Motion sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Push button
#define BUTTON_PIN 3
volatile bool sosButtonPressed = false;
unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 300;

// Timing
uint32_t startTime;
uint32_t lastBeat = 0;
uint32_t lastUpdate = 0;
uint16_t secondsRemaining = 60;

// HR calculation
uint8_t beatCount = 0;
float bpmSum = 0;
float lastBPM = 0;

// Restart LCD safely
void restartLCD() {
  Wire.endTransmission();
  delay(100);
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

// Get body temperature
float getTemperature() {
  tempSensor.requestTemperatures();
  return tempSensor.getTempCByIndex(0);
}

// Get UV index
float getUVIndex() {
  int uvLevel = analogRead(UV_SENSOR_PIN);
  float outputVoltage = 3.3 * uvLevel / 1024.0;
  return outputVoltage / 0.1;
}

// Get motion status
String getMotionStatus() {
  sensors_event_t event;
  accel.getEvent(&event);
  float accelMagnitude = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
  return (accelMagnitude > 10.5) ? "Moving" : "Still";
}

// Debounced button press
void sosButtonInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonPressTime > debounceDelay) {
    sosButtonPressed = true;
    lastButtonPressTime = currentTime;
  }
}

// Update LCD with timer
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Timer: ");
  uint8_t min = secondsRemaining / 60;
  uint8_t sec = secondsRemaining % 60;
  if (min < 10) lcd.print("0");
  lcd.print(min);
  lcd.print(":");
  if (sec < 10) lcd.print("0");
  lcd.print(sec);
  lcd.print("     ");
}


// Calculate average BPM
float getAverageBPM() {
  return (beatCount > 0) ? bpmSum / beatCount : 0;
}

// Estimate SpO2
float estimateSpO2() {
  float red = particleSensor.getRed();
  float ir = particleSensor.getIR();
  if (red > 30000 && ir > 30000) {
    float ratio = red / ir;
    float spo2 = 110.0 - (17.0 * ratio);
    if (spo2 >= 90 && spo2 <= 100) return spo2;
  }
  return 0;
}

// Send data via LoRa and serial
void sendData(bool isSOS) {
  // Get sensor readings
  float avgBPM = getAverageBPM();
  float spo2 = estimateSpO2();
  float temp = getTemperature();
  float uv = getUVIndex();
  String motion = getMotionStatus();
  
  // UV Index interpretation
  String uvStatus;
  if (uv < 3) {
    uvStatus = "Low";
  } else if (uv < 6) {
    uvStatus = "Moderate";
  } else if (uv < 8) {
    uvStatus = "High";
  } else if (uv < 11) {
    uvStatus = "Very High";
  } else {
    uvStatus = "Extreme";
  }

  // First packet: Soldier info and SOS status
  LoRa.beginPacket();
  if (isSOS) {
    LoRa.println("SOS! EMERGENCY ALERT");
  }
  LoRa.println("Name: Ashish Kumar");
  LoRa.println("Age: 28");
  LoRa.println("Battalion: 7th");
  LoRa.println("Regiment: Rajput");
  if (gps.location.isValid()) {
    LoRa.print("GPS: ");
    LoRa.print(gps.location.lat(), 6);
    LoRa.print(",");
    LoRa.println(gps.location.lng(), 6);
  } else {
    LoRa.println("GPS: NO LOCK");
  }
  LoRa.endPacket();
  
  // Print first packet to Serial
  Serial.println("\n=== SENDING FIRST PACKET ===");
  if (isSOS) {
    Serial.println("SOS! EMERGENCY ALERT");
  }
  Serial.println("Name: Ashish Kumar");
  Serial.println("Age: 28");
  Serial.println("Battalion: 7th");
  Serial.println("Regiment: Rajput");
  if (gps.location.isValid()) {
    Serial.print("GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("GPS: NO LOCK");
  }
  Serial.println("==========================");
  
  delay(100); // Small delay between packets

  // Second packet: Health parameters
  LoRa.beginPacket();
  LoRa.print("Temperature: ");
  LoRa.print(temp, 1);
  LoRa.println("°C");
  
  LoRa.print("Heart Rate: ");
  if (avgBPM > 0) {
    LoRa.print(avgBPM, 0);
    LoRa.println(" BPM");
  } else {
    LoRa.println("NA");
  }
  
  LoRa.print("SpO2: ");
  if (spo2 > 0) {
    LoRa.print(spo2, 0);
    LoRa.println("%");
  } else {
    LoRa.println("NA");
  }
  
  LoRa.print("Motion: ");
  LoRa.println(motion);
  
  LoRa.print("UV Index: ");
  LoRa.print(uv, 1);
  LoRa.print(" (");
  LoRa.print(uvStatus);
  LoRa.println(")");
  LoRa.endPacket();
  
  // Print second packet to Serial
  Serial.println("\n=== SENDING SECOND PACKET ===");
  Serial.print("Temperature: ");
  Serial.print(temp, 1);
  Serial.println("°C");
  
  Serial.print("Heart Rate: ");
  if (avgBPM > 0) {
    Serial.print(avgBPM, 0);
    Serial.println(" BPM");
  } else {
    Serial.println("NA");
  }
  
  Serial.print("SpO2: ");
  if (spo2 > 0) {
    Serial.print(spo2, 0);
    Serial.println("%");
  } else {
    Serial.println("NA");
  }
  
  Serial.print("Motion: ");
  Serial.println(motion);
  
  Serial.print("UV Index: ");
  Serial.print(uv, 1);
  Serial.print(" (");
  Serial.print(uvStatus);
  Serial.print(")");
  Serial.println("\n============================");

  // Reset timer and counters for next cycle
  if (!isSOS) {  // Only reset for routine updates, SOS is handled in loop
    startTime = millis();
    lastUpdate = millis();
    secondsRemaining = 60;  // Reset to 1 minute
    beatCount = 0;
    bpmSum = 0;
    lastBPM = 0;
  }
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.print("Initializing...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), sosButtonInterrupt, FALLING);

  tempSensor.begin();
  accel.begin();
  accel.setRange(ADXL345_RANGE_16_G);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    lcd.setCursor(0, 1);
    lcd.print("HR sensor error!");
    delay(1000);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x3F);
  particleSensor.setPulseAmplitudeIR(0x3F);

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    lcd.setCursor(0, 1);
    lcd.print("LoRa error!");
    delay(1000);
  }
  
  // Set LoRa parameters to match receiver
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);

  lcd.clear();
  lcd.print("System ready");
  lcd.setCursor(0, 1);
  lcd.print("Place finger");
  delay(3000);
  lcd.clear();

  startTime = millis();
  lastUpdate = millis();
  updateLCD();
}

void loop() {
  // Check for SOS button press first
  if (sosButtonPressed) {
    sosButtonPressed = false;  // Clear flag immediately
    
    // Show sending message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SENDING SOS");
    lcd.setCursor(0, 1);
    lcd.print("ALERT...");
    delay(500);  // Brief delay to show message
    
    // Send SOS data immediately
    sendData(true);
    
    // Show confirmation
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SOS ALERT");
    lcd.setCursor(0, 1);
    lcd.print("SENT TO BASE!");
    delay(2000);  // Show confirmation longer
    
    // Reset timer and continue normal operation
    startTime = millis();
    lastUpdate = millis();
    secondsRemaining = 60;  // Reset to 1 minute
    beatCount = 0;
    bpmSum = 0;
    lastBPM = 0;
    
    // Return to normal display
    lcd.clear();
    updateLCD();
    return;
  }

  // Heart rate detection
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    uint32_t now = millis();
    if (lastBeat != 0) {
      float interval = (now - lastBeat) / 1000.0;
      float bpm = 60.0 / interval;
      if (bpm > 40 && bpm < 130 && (lastBPM == 0 || abs(bpm - lastBPM) <= 25)) {
        bpmSum += bpm;
        beatCount++;
        lastBPM = bpm;
      }
    }
    lastBeat = now;
  }

  // GPS data reading
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Update timer display every second
  if (millis() - lastUpdate >= 1000) {
    if (secondsRemaining > 0) {
      secondsRemaining--;
      lastUpdate = millis();
      updateLCD();
    }
  }

  // Regular data update when timer reaches zero
  if (secondsRemaining == 0) {
    sendData(false);
    // Timer will be reset in sendData function
  }
}