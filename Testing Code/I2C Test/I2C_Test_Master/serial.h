#ifndef _SERIAL_H
#define _SERIAL_H

#include "logic.h"

void serial_init() {
  Serial.begin(9600);  // Start serial communication
}

void serial_input() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equals("Deploy Panel")) {
      handleDeployPanel();
    } else if (command.equals("Retract Panel")) {
      handleRetractPanel();
    } else if (command.equals("Start Drilling")) {
      handleStartDrilling();
    } else if (command.equals("Stop Drilling")) {
      handleStopDrilling();
    }
  }
}

#endif