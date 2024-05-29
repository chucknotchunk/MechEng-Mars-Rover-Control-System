#include <Wire.h>
#include "serial.h"
#include "helper.h"
#include "feedback.h"
#include "command.h"

#define PANEL_DEPLOY_COMMAND 0x01
#define PANEL_RETRACT_COMMAND 0x02
#define DEACTIVATE_MAGNET_COMMAND 0x03
#define DRILL_DEPLOY_COMMAND 0x04
#define DRILL_RETRACT_COMMAND 0x05

// Subsystem addresses
#define PANEL_SUBSYSTEM_ADDR 0x0A
#define DRILL_SUBSYSTEM_ADDR 0x0B
#define BATTERY_SUBSYSTEM_ADDR 0X0C
#define MOTION_SUBSYSTEM_ADDR 0X0D

// Define the pins for interrupt and output
#define interruptPin 3
#define outputPin 2

// Define global variables for panel feedback
volatile bool panelStatus = false;  // Retracted

// Define global variables for simple collection feedback
volatile bool drillStatus = false;  // Stopped
volatile bool armStatus = false;    // Retracted

// Define global variables for battery feedback
volatile float currentPowerDraw = 0.0;        // in Amps
volatile float currentPowerLevel = 0.0;       // in percentage
volatile float currentCoreTemperature = 0.0;  // in Celsius

// Motion control interrupt flag set up
volatile bool interruptFlag = false;

void setup() {
  Wire.begin();  // Initialize I2C as master
  Wire.setClock(400000);
  serial_init();  // Initialize serial communication

  // Configure the pins
  pinMode(interruptPin, INPUT);
  pinMode(outputPin, OUTPUT);

  // Attach interrupt to the pin
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
}

void loop() {
  // // Request battery status from the slave
  // pollSubsystemState();
  // // requestBatteryState(BATTERY_SUBSYSTEM_ADDR);

  // // Print the received data to the serial monitor
  // Serial.print("Current Power Draw: ");
  // Serial.print(currentPowerDraw);
  // Serial.println(" A");

  // Serial.print("Current Power Level: ");
  // Serial.print(currentPowerLevel);
  // Serial.println(" %");

  // Serial.print("Current Core Temperature: ");
  // Serial.print(currentCoreTemperature);
  // Serial.println(" °C");

  // delay(10);  // Delay to avoid excessive I2C bus usage

  // Example: sending commands from serial input for testing purposes
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    sendMotionCommand(MOTION_SUBSYSTEM_ADDR, command);
  }
}

// Function to poll status from eac subsystem
void pollSubsystemState() {
  requestPanelState(PANEL_SUBSYSTEM_ADDR);
  requestSamplingState(DRILL_SUBSYSTEM_ADDR);
  requestBatteryState(BATTERY_SUBSYSTEM_ADDR);
}

void handleInterrupt() {
  // Set the flag to indicate the interrupt occurred
  interruptFlag = true;
}