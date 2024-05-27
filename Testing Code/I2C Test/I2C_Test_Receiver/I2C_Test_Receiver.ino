#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Subsystem addresses
#define PANEL_SUBSYSTEM_ADDR 0x0A
#define DRILL_SUBSYSTEM_ADDR 0x0B
#define BATTERY_SUBSYSTEM_ADDR 0X0C

// Dummy values for battery status
float currentPowerDraw = 1.5;         // Example current draw in Amps
float currentPowerLevel = 75.0;       // Example power level in percentage
float currentCoreTemperature = 35.5;  // Example core temperature in Celsius

// Buffered PID outputs
unsigned int bufferedPWM[16];

void setup() {
  Wire.begin(BATTERY_SUBSYSTEM_ADDR);  // Start I2C as slave with address 0x04
  Wire.setClock(400000);
  // Wire.onReceive(receiveEvent);        // Register event for receiving data
  Wire.onRequest(requestEvent);  // Register event for sending data
  Serial.begin(9600);            // Start serial communication for debugging

  pwm.begin();                           // Start communication with PCA9685 PWM driver
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685

  pwm_map_innit();
}

void loop() {
  // Update battery status values periodically, just for testing

  currentPowerDraw += 0.1;
  currentPowerLevel -= 0.1;
  currentCoreTemperature += 0.05;

  if (currentPowerLevel < 0) currentPowerLevel = 100.0;
  if (currentCoreTemperature > 40.0) currentCoreTemperature = 35.0;

  // Print the received data to the serial monitor
  // Serial.print("Current Power Draw: ");
  // Serial.print(currentPowerDraw);
  // Serial.println(" A");

  // Serial.print("Current Power Level: ");
  // Serial.print(currentPowerLevel);
  // Serial.println(" %");

  // Serial.print("Current Core Temperature: ");
  // Serial.print(currentCoreTemperature);
  // Serial.println(" °C");
 
  // bufferPWMOutputs();
  // updatePWM();


  // pwm_map();

  delay(100);  // Simulate periodic updates
}

// void bufferPWMOutputs() {
//   static int phase = 0;
//   phase = (phase + 8) % 4096;  // Cycle through the full range of PWM values

//   for (uint8_t i = 0; i < 16; i++) {
//     // Create a waveform pattern for each channel
//     bufferedPWM[i] = (phase + (i * 256)) % 4096;  // Example pattern
//   }
// }

// // Function to update PWM signals
// void updatePWM() {
//   for (uint8_t i = 0; i < 16; i++) {
//     pwm.setPWM(i, 0, bufferedPWM[i]);  // Set PWM from buffer
//   }
// }

// void receiveEvent(int howMany) {
//   while (Wire.available()) {
//     byte command = Wire.read();  // Read the command byte
//     handleCommand(command);      // Handle the command
//   }
// }

// void handleCommand(byte command) {
// }

void requestEvent() {
  // Buffer to hold the float values as bytes
  byte buffer[12];
  floatToBytes(currentPowerDraw, buffer, 0);
  floatToBytes(currentPowerLevel, buffer, 4);
  floatToBytes(currentCoreTemperature, buffer, 8);

  // Send the buffer to the master
  Wire.write(buffer, 12);

}

// Helper function to convert float to bytes
void floatToBytes(float value, byte* bytes, int offset) {
  memcpy(bytes + offset, &value, sizeof(float));
}