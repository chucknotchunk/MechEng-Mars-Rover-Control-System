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

// Define global variables for panel feedback
volatile bool panelStatus = false;  // Retracted

// Define global variables for simple collection feedback
volatile bool drillStatus = false;  // Stopped
volatile bool armStatus = false;    // Retracted

// Define global variables for battery feedback
volatile float currentPowerDraw = 0.0;        // in Amps
volatile float currentPowerLevel = 0.0;       // in percentage
volatile float currentCoreTemperature = 0.0;  // in Celsius

void setup() {
  Wire.begin();   // Initialize I2C as master
  serial_init();  // Initialize serial communication
}

void loop() {
  // Request battery status from the slave
  requestBatteryState(BATTERY_SUBSYSTEM_ADDR);

  // Print the received data to the serial monitor
  Serial.print("Current Power Draw: ");
  Serial.print(currentPowerDraw);
  Serial.println(" A");

  Serial.print("Current Power Level: ");
  Serial.print(currentPowerLevel);
  Serial.println(" %");

  Serial.print("Current Core Temperature: ");
  Serial.print(currentCoreTemperature);
  Serial.println(" °C");

  delay(100); // Delay to avoid excessive I2C bus usage
}

// Function to poll status from eac subsystem
void pollSubsystemState() {
  requestPanelState(PANEL_SUBSYSTEM_ADDR);
  requestSamplingState(DRILL_SUBSYSTEM_ADDR);
  requestBatteryState(BATTERY_SUBSYSTEM_ADDR);
}
