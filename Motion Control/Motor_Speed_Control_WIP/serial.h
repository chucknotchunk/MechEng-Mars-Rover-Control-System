#ifndef _SERIAL_H
#define _SERIAL_H

#include "motion.h"

void serial_innit(){
    Serial.begin(9600); // Start serial for output
}

void serial_input(){
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read the entire string until newline
        command.trim(); // Remove any leading/trailing whitespace

        float commandDistance = 0.0;
        float commandAngle = 0.0;
        float commandRadius = 0.0;
        bool validCommand = false;

        if (command.startsWith("move")) {
            // Extract distance value for moving straight command
            commandDistance = command.substring(4).toFloat();
            moveStraight(commandDistance);
            validCommand = true;
        } else if (command.startsWith("turn")) {
            // Extract distance value for backward command and negate it
            commandAngle = command.substring(4).toFloat();
            validCommand = true;
        }

        if (validCommand) {

        } else {
            // Invalid command entered
            Serial.println("Invalid command. Please enter 'move' or 'turn' followed by distance/angle");
        }
    }
}

#endif
