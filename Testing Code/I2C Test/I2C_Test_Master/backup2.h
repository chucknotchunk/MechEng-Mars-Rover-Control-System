#include <Wire.h>
#include "serial.h"
#include "helper.h"
#include "feedback.h"
#include "command.h"
#include "logic.h"

// Command constants
const int PANEL_DEPLOY_COMMAND = 0x01;
const int PANEL_RETRACT_COMMAND = 0x02;
const int DEACTIVATE_MAGNET_COMMAND = 0x03;
const int DRILL_DEPLOY_COMMAND = 0x04;
const int DRILL_RETRACT_COMMAND = 0x05;

// Subsystem addresses
const int PANEL_SUBSYSTEM_ADDR = 0x0A;
const int DRILL_SUBSYSTEM_ADDR = 0x0B;
const int BATTERY_SUBSYSTEM_ADDR = 0x0C;
const int MOTION_SUBSYSTEM_ADDR = 0x0D;

// Define the pins for Motionc Control related interrupt and output
const int interruptPin = 3;  // Triggered when Rover stopped
const int outputPin = 2;     // Set to High when emergency stop


const float batteryOverl = 10.0;  // Define battery power overload threshold
const float batteryLow = 20.0;    // Define battery level low threshold
const float batteryHot = 50.0;    // Define battery core termperature overheat threshold

// Define global variables for panel feedback
volatile bool panelStatus = false;  // Retracted

// Define global variables for sample collection feedback
volatile bool drillStatus = false;  // Stopped
volatile bool armStatus = false;    // Retracted

// Define global variables for motion control feedback
volatile bool roverMoving = false;  // Rover status, Not moving

// Define global variables for battery feedback
volatile float currentPowerDraw = 0.0;        // in Amps
volatile float currentPowerLevel = 21.0;      // in percentage
volatile float currentCoreTemperature = 0.0;  // in Celsius

// Motion control interrupt flag set up
volatile bool interruptFlag = false;

void setup() {
  comm_init();  // Initialize communication system
  sys_init();   // Initialize control system
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);   //  Attach interrupt to the rising edge
}

void loop() {

  serial_input();
  // delay(10);  // Delay to avoid excessive I2C bus usage
  if (!roverMoving) {
    // Only request status updates from the slaves when Rover is not moving (I2C Bus is free)
    pollSubsystemState();
  }

  handleRoverStatus();
}

// Function to poll status from eac subsystem
void pollSubsystemState() {
  requestPanelState(PANEL_SUBSYSTEM_ADDR);
  requestSamplingState(DRILL_SUBSYSTEM_ADDR);
  //requestBatteryState(BATTERY_SUBSYSTEM_ADDR);
}

void handleInterrupt() {
  // Set the flag to indicate the interrupt occurred, when las command motion is completed
  interruptFlag = true;
}