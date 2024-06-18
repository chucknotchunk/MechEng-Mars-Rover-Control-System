// Subsystem addresses
#define PANEL_SUBSYSTEM_ADDR 0x0A

#define PANEL_DEPLOY_COMMAND 0x01
#define PANEL_RETRACT_COMMAND 0x02
#define DEACTIVATE_MAGNET_COMMAND 0x03

// Define global variables for panel feedback
volatile bool panelStatus = false;  // Retracted

void setup() {
  Wire.begin(SLAVE_ADDRESS);     // Initialize as an I2C slave with the given address
  Wire.onReceive(receiveEvent);  // Register the receive event handler
  Wire.onRequest(requestEvent);  // Register event for sending data
  Wire.setClock(400000);
}

void loop() {
  // Main loop does nothing, just waiting for I2C commands
}

// This function is called when data is received over I2C
void receiveEvent(int howMany) {
  while (Wire.available()) {
    byte command = Wire.read();  // Read the incoming byte as command
    handleCommand(command);      // Process the command
  }
}

// Function to handle the received command
void handleCommand(byte command) {
  switch (command) {
    case PANEL_DEPLOY_COMMAND:
      // Add your command-specific code here
      break;
    case PANEL_RETRACT_COMMAND:
      // Add your command-specific code here
      break;
    case DEACTIVATE_MAGNET_COMMAND:
      // Add your command-specific code here
      break;
  }
}

void requestEvent() {
  Wire.write(panelStatus);  // Send the panel status to the master
}
