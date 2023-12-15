#ifndef _SERIAL_H
#define _SERIAL_H

int LastInput = 0;
float TotalDistance = 0.0;

void serial_innit(){
    Serial.begin(9600); // Start serial for output
}
/*
int serial_input(){
    if (Serial.available() > 0) {
    int SerialInput = Serial.parseInt(); // Read the next integer from serial input
    if (Serial.read() != '\n') { // Check if a newline character follows the integer
      // If not a newline, the input is not valid or complete
      SerialInput = LastInput; // Keep the last valid value
    }
    if (SerialInput >= -4095 && SerialInput <= 4095) {
      LastInput = SerialInput; // Update LastInput with the new value
    }
  }

  return LastInput;
}
*/

float serial_input(){
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read the entire string until newline
        command.trim(); // Remove any leading/trailing whitespace

        float commandDistance = 0.0;
        bool validCommand = false;

        if (command.startsWith("forward")) {
            // Extract distance value for forward command
            commandDistance = command.substring(7).toFloat();
            validCommand = true;
        } else if (command.startsWith("backward")) {
            // Extract distance value for backward command and negate it
            commandDistance = -command.substring(8).toFloat();
            validCommand = true;
        }

        if (validCommand) {
            // Update the total distance
            TotalDistance += commandDistance;
        } else {
            // Invalid command entered
            Serial.println("Invalid command. Please enter 'forward' or 'backward' followed by distance");
        }
    }

    return TotalDistance;
}

#endif
