#ifndef _SERIAL_H
#define _SERIAL_H

#include "motion.h"

extern volatile bool state;

void serial_innit() {
  Serial.begin(9600);  // Start serial for output
}

void serial_input() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the entire string until newline
    command.trim();                                 // Remove any leading/trailing whitespace

    float commandDistance = 0.0;
    float commandAngle = 0.0;
    float commandRadius = 0.0;
    bool validCommand = false;

    if (command.equals("stop")) {
      state = false;  // Set state to false
      Serial.println("Rover stopped.");
      validCommand = true;
    } else if (command.equals("resume")) {
      state = true;  // Set state to true
      Serial.println("Rover Resumed.");
      validCommand = true;
    } else if (command.startsWith("move")) {
      // Extract distance value for moving straight command
      commandDistance = command.substring(5).toFloat();  // Skip "move "
      moveStraight(commandDistance);
      validCommand = true;
    } else if (command.startsWith("turn")) {
      // Expected command format: "turn angle radius"
      int firstSpaceIndex = command.indexOf(' ');
      int secondSpaceIndex = command.indexOf(' ', firstSpaceIndex + 1);

      if (firstSpaceIndex != -1 && secondSpaceIndex != -1) {
        commandAngle = command.substring(firstSpaceIndex + 1, secondSpaceIndex).toFloat();
        commandRadius = command.substring(secondSpaceIndex + 1).toFloat();

        if (commandRadius > 0) {
          turnByRadius(commandAngle, commandRadius);
          validCommand = true;
        } else {
          Serial.println("Invalid radius. Please enter a positive number for radius.");
        }
      } else {
        Serial.println("Invalid turn command format. Please use 'turn angle radius'");
      }
    }

    if (!validCommand) {
      // Invalid command entered
      Serial.println("Invalid command. Please enter 'move' followed by distance or 'turn' followed by angle and radius");
    }
  }
}

#endif
