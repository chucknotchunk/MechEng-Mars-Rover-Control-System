#include <Wire.h>

// Subsystem addresses
#define PANEL_SUBSYSTEM_ADDR 0x0A
#define DRILL_SUBSYSTEM_ADDR 0x0B
#define BATTERY_SUBSYSTEM_ADDR 0X0C

// Dummy values for battery status
float currentPowerDraw = 1.5;        // Example current draw in Amps
float currentPowerLevel = 75.0;      // Example power level in percentage
float currentCoreTemperature = 35.5; // Example core temperature in Celsius

void setup() {
  // Wire.begin(BATTERY_SUBSYSTEM_ADDR);  // Start I2C as slave with address 0x04
  Wire.onReceive(receiveEvent);        // Register event for receiving data
  // Wire.onRequest(requestEvent);        // Register event for sending data
  // Serial.begin(9600);                  // Start serial communication for debugging
}

void loop() {
  // Update battery status values periodically, just for testing
  currentPowerDraw += 0.1;
  currentPowerLevel -= 0.1;
  currentCoreTemperature += 0.05;

  if (currentPowerLevel < 0) currentPowerLevel = 100.0;
  if (currentCoreTemperature > 40.0) currentCoreTemperature = 35.0;

    // Print the received data to the serial monitor
  Serial.print("Current Power Draw: ");
  Serial.print(currentPowerDraw);
  Serial.println(" A");

  Serial.print("Current Power Level: ");
  Serial.print(currentPowerLevel);
  Serial.println(" %");

  Serial.print("Current Core Temperature: ");
  Serial.print(currentCoreTemperature);
  Serial.println(" °C");

  delay(100); // Simulate periodic updates
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    byte command = Wire.read();  // Read the command byte
    handleCommand(command);      // Handle the command
  }
}

void handleCommand(byte command) {
}

void requestEvent() {
  // Buffer to hold the float values as bytes
  byte buffer[12];
  floatToBytes(currentPowerDraw, buffer, 0);
  floatToBytes(currentPowerLevel, buffer, 4);
  floatToBytes(currentCoreTemperature, buffer, 8);

  // Send the buffer to the master
  Wire.write(buffer, 12);
}

// Helper function to convert float to bytes
void floatToBytes(float value, byte* bytes, int offset) {
  memcpy(bytes + offset, &value, sizeof(float));
}