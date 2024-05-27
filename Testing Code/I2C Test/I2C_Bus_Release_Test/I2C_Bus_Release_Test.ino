#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"

#define MOTION_SUBSYSTEM_ADDR 0X0D
#include <Wire.h>

volatile bool roverActive;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool sendPWMCommands = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685

  pwm_map_innit();

  Wire.begin(MOTION_SUBSYSTEM_ADDR);  // Start I2C as slave with address
  Wire.onReceive(receiveEvent);       // Register the receive event handler
  //Wire.onRequest(requestEvent);     // Register event for sending data
  Wire.setClock(400000);
}

void loop() {
  pwm_map();
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