#ifndef _SERIAL_H
#define _SERIAL_H

#include "logic.h"

void comm_init() {
  Wire.begin();  // Initialize I2C as master
  Wire.setClock(400000);
  // Initialize serial communication
  Serial.begin(9600);  // Start serial communication
}

void serial_input() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equals("deploy panel")) {
      handleDeployPanel();
    } else if (command.equals("retract panel")) {
      handleRetractPanel();
    } else if (command.equals("start drill")) {
      handleStartDrilling();
    } else if (command.equals("stop drill")) {
      handleStopDrilling();
    } else if (command.equals("stop move")) {  // Corrected missing closing parenthesis
      handleStopDriving();
    } else if (command.equals("resume move")) {  // Corrected missing closing parenthesis
      handleResumeDriving();
    } else if (command.startsWith("move") || command.startsWith("turn")) {  // Corrected logic
      // Handle move and turn commands here
      handleMoveOrTurnCommand(command);
    }
  }
}

#endif