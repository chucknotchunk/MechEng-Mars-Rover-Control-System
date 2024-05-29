#ifndef _LOGIC_H
#define _LOGIC_H

#include "command.h"

// Forward declaration
bool isBatterySufficient();

// Command constants
extern const int PANEL_DEPLOY_COMMAND;
extern const int PANEL_RETRACT_COMMAND;
extern const int DEACTIVATE_MAGNET_COMMAND;
extern const int DRILL_DEPLOY_COMMAND;
extern const int DRILL_RETRACT_COMMAND;

// Subsystem addresses
extern const int PANEL_SUBSYSTEM_ADDR;
extern const int DRILL_SUBSYSTEM_ADDR;
extern const int BATTERY_SUBSYSTEM_ADDR;
extern const int MOTION_SUBSYSTEM_ADDR;

// Define global variables for panel feedback
extern volatile bool panelStatus;  // Retracted

// Define global variables for sample collection feedback
extern volatile bool drillStatus;  // Stopped
extern volatile bool armStatus;    // Retracted

// Define global variables for motion control feedback
extern volatile bool roverMoving;  // Not moving

// Define global variables for battery feedback
extern volatile float currentPowerDraw;        // in Amps
extern volatile float currentPowerLevel;       // in percentage
extern volatile float currentCoreTemperature;  // in Celsius

void handleDeployPanel() {
  // Check if Rover is driving
  if (roverMoving) {
    Serial.println("Rover is moving. Cannot deploy panel.");
    return;  // Exit function
  }
  // Check if Battery has enough power to deploy panel
  if (!isBatterySufficient()) {
    Serial.println("Battery level is too low to deploy panel.");
    return;  // Exit function
  }
  sendPanelCommand(PANEL_SUBSYSTEM_ADDR, DEACTIVATE_MAGNET_COMMAND);  // Deactivate magnet
  sendPanelCommand(PANEL_SUBSYSTEM_ADDR, PANEL_DEPLOY_COMMAND);       // Deploy panel
}

void handleRetractPanel() {
  // Check if Battery has enough power to retract panel
  if (!isBatterySufficient()) {
    Serial.println("Battery level is too low to retract panel.");
    return;
  }
  sendPanelCommand(PANEL_SUBSYSTEM_ADDR, PANEL_RETRACT_COMMAND);  // Retract panel
}

void handleStartDrilling() {
  if (roverMoving) {
    Serial.println("Rover is moving. Cannot start drilling.");
    return;  // Exit function
  }
  if (!isBatterySufficient()) {
    Serial.println("Battery level is too low to start drilling.");
    return;
  }
  // If panel already deployed, no need to deactivate the magnet
  if (!panelStatus) {
    sendPanelCommand(PANEL_SUBSYSTEM_ADDR, DEACTIVATE_MAGNET_COMMAND);  // Deactivate magnet
  }
  sendSamplingCommand(DRILL_SUBSYSTEM_ADDR, DRILL_DEPLOY_COMMAND);  // Start drilling
}

void handleStopDrilling() {
  sendSamplingCommand(DRILL_SUBSYSTEM_ADDR, DRILL_RETRACT_COMMAND);  // Stop drilling
}

void handleStartDriving() {
}

void handleStopDriving() {
}

void handleResumeDriving() {
  
}

bool isBatterySufficient() {
  // Check if the current power level is above a certain threshold
  return currentPowerLevel > 20.0;  // Example threshold of 20%
}

#endif