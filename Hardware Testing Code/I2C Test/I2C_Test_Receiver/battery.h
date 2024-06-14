// Subsystem addresses
#define BATTERY_SUBSYSTEM_ADDR 0X0C

// Dummy values for battery status
volatile float currentPowerDraw = 1.5;         // Example current draw in Amps
volatile float currentPowerLevel = 75.0;       // Example power level in percentage
volatile float currentCoreTemperature = 35.5;  // Example core temperature in Celsius

void setup() {
  Wire.begin(BATTERY_SUBSYSTEM_ADDR);  // Start I2C as slave with address 0x04
  Wire.onRequest(requestEvent);        // Register event for sending data
  Wire.setClock(400000);
}

void loop() {
  // Main loop does nothing, just waiting for I2C commands
}

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