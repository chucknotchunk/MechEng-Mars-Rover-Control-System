#ifndef _FEEDBACK_H
#define _FEEDBACK_H

#include "helper.h"
// Define global variables for panel feedback
extern volatile bool panelStatus;  // Retracted

// Define global variables for simple collection feedback
extern volatile bool drillStatus;  // Stopped
extern volatile bool armStatus;    // Retracted

// Define global variables for battery feedback
extern volatile float currentPowerDraw;        // in Amps
extern volatile float currentPowerLevel;       // in percentage
extern volatile float currentCoreTemperature;  // in Celsius

// Poll status form the panel deployment system
void requestPanelState(byte slaveAddr) {
  Wire.requestFrom(slaveAddr, 1);  // Request 1 byte from the slave
  if (Wire.available()) {
    panelStatus = Wire.read();
  }
}

// Poll status from sample collection system
void requestSamplingState(byte slaveAddr) {
  Wire.requestFrom(slaveAddr, 2);  // Request 2 bytes from the slave
  if (Wire.available() == 2) {
    byte DrillState = Wire.read();
    byte ArmState = Wire.read();
    // Update the global variables based on the received statuses
    drillStatus = DrillState;
    armStatus = ArmState;
  }
}

void requestBatteryState(byte slaveAddr) {
  Wire.requestFrom(slaveAddr, 12);  // Request 12 bytes from the slave
  delay(10);                        // Small delay to ensure data is available

  Serial.print("Bytes available: ");
  Serial.println(Wire.available());

  if (Wire.available() == 12) {
    byte rawData[12];
    for (int i = 0; i < 12; i++) {
      rawData[i] = Wire.read();
    }

    // Convert byte array to floats
    currentPowerDraw = bytesToFloat(rawData, 0);
    currentPowerLevel = bytesToFloat(rawData, 4);
    currentCoreTemperature = bytesToFloat(rawData, 8);
  }
}

#endif