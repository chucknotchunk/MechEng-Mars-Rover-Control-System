#ifndef _LOGIC_H
#define _LOGIC_H

#include "command.h"
#include "helper.h"

// Forward declaration
bool isBatterySufficient();
bool isValidAngle(float angle);
bool isValidRadius(float radius);
bool isValidDistance(float distance);
void handleDeployPanel();
void handleRetractPanel();
void handleStartDrilling();
void handleStopDrilling();
void handleMoveOrTurnCommand(String command);
void handleStopDriving();
void handleResumeDriving();
void handleRoverStatus();
void sys_init();
void moveRover(float distance);
void turnRover(float angle, float radius);

// Forward declarations for subsystem status request functions
void requestBatteryState(byte slaveAddr);
void requestPanelState(byte slaveAddr);
void requestSamplingState(byte slaveAddr);

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
volatile bool movingComm = true;   // Moving command status, allow sending command

// Define global variables for battery feedback
extern volatile float currentPowerDraw;        // in Amps
extern volatile float currentPowerLevel;       // in percentage
extern volatile float currentCoreTemperature;  // in Celsius

// Define the pins for interrupt and output
extern const int interruptPin;
extern const int outputPin;

// Motion control interrupt flag set up
extern volatile bool interruptFlag;

void handleDeployPanel() {
  // Check if panel is already deployed
  if (panelStatus) {
    Serial.println("Panel is already deployed.");
    return;  // Exit function
  }
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
  delay(100);                                                         // Add a delay to ensure the command is processed
  sendPanelCommand(PANEL_SUBSYSTEM_ADDR, PANEL_DEPLOY_COMMAND);       // Deploy panel
  Serial.println("Panel deployment command sent.");
}

void handleRetractPanel() {
  // Check if panel is already retracted
  if (!panelStatus) {
    Serial.println("Panel has yet to deployed.");
    return;  // Exit function
  }
  // Check if Battery has enough power to retract panel
  if (!isBatterySufficient()) {
    Serial.println("Battery level is too low to retract panel.");
    return;
  }
  sendPanelCommand(PANEL_SUBSYSTEM_ADDR, PANEL_RETRACT_COMMAND);  // Retract panel
  Serial.println("Panel retraction command sent.");
}

void handleStartDrilling() {
  // Check if Rover is drilling
  if (drillStatus) {
    Serial.println("Drilling is already initiated.");
    return;  // Exit function
  }
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
  delay(100);                                                       // Add a delay to ensure the command is processed
  sendSamplingCommand(DRILL_SUBSYSTEM_ADDR, DRILL_DEPLOY_COMMAND);  // Start drilling
  Serial.println("Start drilling command sent.");
}

void handleStopDrilling() {
  // Check if Rover is not drilling
  if (!drillStatus) {
    Serial.println("Drilling has yet to started.");
    return;  // Exit function
  }
  sendSamplingCommand(DRILL_SUBSYSTEM_ADDR, DRILL_RETRACT_COMMAND);  // Stop drilling
  Serial.println("Emergency stop drilling command sent.");
}

void handleMoveOrTurnCommand(String command) {
  // Perform checks before processing move commands
  if (!movingComm) {
    Serial.println("Rover Paused.");
    return;  // Exit function
  }
  if (roverMoving) {
    Serial.println("Rover is moving.");
    return;  // Exit function
  }
  if (!isBatterySufficient()) {
    Serial.println("Battery level is too low to move.");
    return;
  }
  if (panelStatus) {
    Serial.println("Panel is deployed. Cannot move Rover.");
    return;  // Exit function
  }
  if (armStatus) {
    Serial.println("Arm is deployed. Cannot move Rover.");
    return;  // Exit function
  }

  // Identify commands
  if (command.startsWith("move")) {
    // Extract distance value from the command
    float distance = command.substring(5).toFloat();  // Skip "move "
    if (isValidDistance(distance)) {
      moveRover(distance);
    } else {
      Serial.println("Invalid move command. Distance out of range.");
    }
  } else if (command.startsWith("turn")) {
    // Extract angle and radius values from the command
    int firstSpaceIndex = command.indexOf(' ');
    int secondSpaceIndex = command.indexOf(' ', firstSpaceIndex + 1);
    if (firstSpaceIndex != -1 && secondSpaceIndex != -1) {
      float angle = command.substring(firstSpaceIndex + 1, secondSpaceIndex).toFloat();
      float radius = command.substring(secondSpaceIndex + 1).toFloat();
      if (isValidAngle(angle) && isValidRadius(radius)) {
        turnRover(angle, radius);
      } else {
        Serial.println("Invalid turn command. Angle or radius out of range.");
      }
    } else {
      Serial.println("Invalid turn command format. Please use 'turn angle radius'.");
    }
  } else {
    Serial.println("Invalid command. Use 'move distance' or 'turn angle radius'.");
  }
}

bool isValidDistance(float distance) {
  // Define the valid range for distance
  return abs(distance) <= 10;
}

bool isValidAngle(float angle) {
  // Define the valid range for angle
  return angle >= -180 && angle <= 180;
}

bool isValidRadius(float radius) {
  // Define the valid range for radius
  return radius >= 0 && radius <= 10;
}

void moveRover(float distance) {
  // Format the command string as "Mdistance"
  String command = "M" + String(distance, 2);
  // Send the command to the motion subsystem
  sendMotionCommand(MOTION_SUBSYSTEM_ADDR, command);
  roverMoving = true;
  // Print the command for debugging purposes
  Serial.print("Moving rover by distance: ");
  Serial.println(distance);
}


void turnRover(float angle, float radius) {
  // Format the command string as "Tangle,radius"
  String command = "T" + String(angle, 2) + "," + String(radius, 2);
  // Send the command to the motion subsystem
  sendMotionCommand(MOTION_SUBSYSTEM_ADDR, command);
  roverMoving = true;
  // Print the command for debugging purposes
  Serial.print("Turning rover by angle: ");
  Serial.print(angle);
  Serial.print(" with radius: ");
  Serial.println(radius);
}

void handleStopDriving() {
  // Inform the slave stop driving
  digitalWrite(outputPin, HIGH);
  // Add a short delay to ensure the master detects the high signal
  delay(10);
  // Reset the pin to low to avoid keeping it high
  digitalWrite(outputPin, LOW);
  movingComm = false;
  Serial.println("Stop Rover moving command sent.");
}

void handleResumeDriving() {
  sendMotionCommand(MOTION_SUBSYSTEM_ADDR, "R");
  movingComm = true;
  Serial.println("Resume Rover moving command sent.");
}

bool isBatterySufficient() {
  // Check if the current power level is above a certain threshold
  return currentPowerLevel > 20.0;  // Example threshold of 20%
}

void handleRoverStatus() {
  if (interruptFlag) {
    roverMoving = false;
    Serial.println("Rover stopped moving.");
    // Reset the interrupt flag after handling it
    interruptFlag = false;
  }
}

void sys_init() {
  // Configure the pins
  pinMode(interruptPin, INPUT);
  pinMode(outputPin, OUTPUT);

  // Initialize Serial communication
  Serial.begin(9600);

  // Poll Subsystem Status
  //requestBatteryState(BATTERY_SUBSYSTEM_ADDR);
  Serial.println("Rover Initializing.");
  delay(100);

  // Check Panel Status
  Serial.println("Checking Panel Status.");
  requestPanelState(PANEL_SUBSYSTEM_ADDR);
  delay(100);
  if (panelStatus) {
    Serial.println("Panel is deployed. Retracting panel.");
    sendPanelCommand(PANEL_SUBSYSTEM_ADDR, PANEL_RETRACT_COMMAND);
    delay(100);
  } else {
    Serial.println("Panel is already retracted.");
  }

  // Check Drill Status
  Serial.println("Checking Drill Status.");
  requestSamplingState(DRILL_SUBSYSTEM_ADDR);
  delay(100);
  if (drillStatus) {
    Serial.println("Drill is deployed. Retracting drill.");
    sendSamplingCommand(DRILL_SUBSYSTEM_ADDR, DRILL_RETRACT_COMMAND);
    delay(100);
  } else {
    Serial.println("Drill is already retracted.");
  }

  // Wait until both panel and drill are retracted
  while (panelStatus || drillStatus) {
    // Check Panel Status
    requestPanelState(PANEL_SUBSYSTEM_ADDR);
    delay(100);
    if (!panelStatus) {
      Serial.println("Panel retracted.");
    }

    // Check Drill Status
    requestSamplingState(DRILL_SUBSYSTEM_ADDR);
    delay(100);
    if (!drillStatus) {
      Serial.println("Drill retracted.");
    }
  }

  sendMotionCommand(MOTION_SUBSYSTEM_ADDR, "R");
  movingComm = true;

  Serial.println("Initialization Complete.");
}

#endif