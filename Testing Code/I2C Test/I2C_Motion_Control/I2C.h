#ifndef _I2C_H
#define _I2C_H

#define MOTION_SUBSYSTEM_ADDR 0X0D
#include <Wire.h>
#include "motion.h"

extern volatile bool roverActive;

// Forward declarations
void receiveEvent(int howMany);
void processCommand(String command);

void i2cInit(void) {
  Wire.begin(MOTION_SUBSYSTEM_ADDR);  // Start I2C as slave with address
  Wire.onReceive(receiveEvent);       // Register the receive event handler
  //Wire.onRequest(requestEvent);     // Register event for sending data
  Wire.setClock(400000);
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
    moveStraight(commandDistance);
    validCommand = true;
  } else if (command.startsWith("T")) {
    int commaIndex = command.indexOf(',');
    if (commaIndex != -1) {
      commandAngle = command.substring(1, commaIndex).toFloat();
      commandRadius = command.substring(commaIndex + 1).toFloat();
      if (commandRadius > 0) {
        turnByRadius(commandAngle, commandRadius);
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

#endif
