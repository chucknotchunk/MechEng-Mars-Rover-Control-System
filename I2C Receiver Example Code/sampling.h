// Subsystem addresses
#define DRILL_SUBSYSTEM_ADDR 0x0B

#define DRILL_DEPLOY_COMMAND 0x04
#define DRILL_RETRACT_COMMAND 0x05

// Define global variables for simple collection feedback
volatile bool drillStatus = false;  // Stopped
volatile bool armStatus = false;    // Retracted

void setup() {
  Wire.begin(SLAVE_ADDRESS);     // Initialize as an I2C slave with the given address
  Wire.onRequest(requestEvent);  // Register the request event handler
  Wire.onReceive(receiveEvent);  // Register the receive event handler
  Serial.begin(9600);            // Start serial communication for debugging
}

void loop() {
  // Main loop can update drillStatus and armStatus as needed
  // For example, based on some sensor readings or other logic
}

// This function is called when the master requests data
void requestEvent() {
  Wire.write(drillStatus);  // Send the drill status to the master
  Wire.write(armStatus);    // Send the arm status to the master
}

// This function is called when data is received from the master
void receiveEvent(int howMany) {
  if (howMany > 0) {
    byte command = Wire.read();  // Read the incoming byte as command
    handleCommand(command);      // Process the command
  }
}

// Function to handle the received command
void handleCommand(byte command) {
  switch (command) {
    case DRILL_DEPLOY_COMMAND:
      // Add your command-specific code here
      break;
    case DRILL_RETRACT_COMMAND:
      // Add your command-specific code here
      break;
  }
}
