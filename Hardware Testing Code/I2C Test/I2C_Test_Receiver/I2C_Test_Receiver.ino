#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"

volatile bool roverActive;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool sendPWMCommands = false;

// Subsystem addresses
#define PANEL_SUBSYSTEM_ADDR 0x0A
#define DRILL_SUBSYSTEM_ADDR 0x0B
#define BATTERY_SUBSYSTEM_ADDR 0X0C
#define MOTION_SUBSYSTEM_ADDR 0X0D

// Dummy values for battery status
float currentPowerDraw = 1.5;         // Example current draw in Amps
float currentPowerLevel = 75.0;       // Example power level in percentage
float currentCoreTemperature = 35.5;  // Example core temperature in Celsius

// Buffered PID outputs
unsigned int bufferedPWM[16];

void setup() {
  Wire.begin(BATTERY_SUBSYSTEM_ADDR);  // Start I2C as slave with address 0x04
  Wire.setClock(400000);
  Wire.onReceive(receiveEvent);        // Register event for receiving data
  Wire.onRequest(requestEvent);  // Register event for sending data
  Serial.begin(9600);            // Start serial communication for debugging

  pwm.begin();                           // Start communication with PCA9685 PWM driver
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685

  pwm_map_innit();
}

void loop() {
  // Update battery status values periodically, just for testing

  currentPowerDraw += 0.1;
  currentPowerLevel -= 0.1;
  currentCoreTemperature += 0.05;

  if (currentPowerLevel < 0) currentPowerLevel = 100.0;
  if (currentCoreTemperature > 40.0) currentCoreTemperature = 35.0;

  // // Print the received data to the serial monitor
  // Serial.print("Current Power Draw: ");
  // Serial.print(currentPowerDraw);
  // Serial.println(" A");

  // Serial.print("Current Power Level: ");
  // Serial.print(currentPowerLevel);
  // Serial.println(" %");

  // Serial.print("Current Core Temperature: ");
  // Serial.print(currentCoreTemperature);
  // Serial.println(" °C");

  delay(1);  // Simulate periodic updates

  //pwm_map();
  // Check for serial input
  if (Serial.available() > 0) {
    int input = Serial.parseInt();  // Read the integer input from the serial

    // Check the input value
    if (input == 1) {
      sendPWMCommands = true;
      Serial.println("Sending PWM commands.");
    } else if (input == 2) {
      sendPWMCommands = false;
      Serial.println("Stopping PWM commands.");
      // Optionally set all PWM outputs to 0
      for (int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);
      }
    }
  }

  // Send PWM commands if enabled
  if (sendPWMCommands) {
    for (int i = 0; i < 16; i++) {
      pwm.setPWM(i, 0, 2000);
    }
  }
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

void receiveEvent(int howMany) {
  String command = "";
  while (Wire.available()) {
    char c = Wire.read();
    command += c;
  }
  processCommand(command);
}

void processCommand(String command) {
  command.trim();  // Remove any leading/trailing whitespace

  float commandDistance = 0.0;
  float commandAngle = 0.0;
  float commandRadius = 0.0;
  bool validCommand = false;

  if (command.equals("S")) {
    roverActive = false;
    Serial.println("Rover stopped.");
    validCommand = true;
  } else if (command.equals("R")) {
    roverActive = true;
    Serial.println("Rover Resumed.");
    validCommand = true;
  } else if (command.startsWith("M")) {
    commandDistance = command.substring(1).toFloat();
    // moveStraight(commandDistance);
    validCommand = true;
  } else if (command.startsWith("T")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      commandAngle = command.substring(1, commaIndex).toFloat();
      commandRadius = command.substring(commaIndex + 1).toFloat();
      if (commandRadius > 0) {
        // turnByRadius(commandAngle, commandRadius);
        validCommand = true;
      } else {
        Serial.println("Invalid radius. Please enter a positive number for radius.");
      }
    } else {
      Serial.println("Invalid turn command format. Please use 'Tangle,radius'");
    }
  }

  if (!validCommand) {
    Serial.println("Invalid command. Please enter 'M' followed by distance or 'T' followed by angle and radius.");
  }
}