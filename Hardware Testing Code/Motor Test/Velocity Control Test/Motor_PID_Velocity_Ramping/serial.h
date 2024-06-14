#ifndef _SERIAL_H
#define _SERIAL_H

int LastInput = 0;

void serial_innit(){
    Serial.begin(9600);           // start serial for output
}

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

#endif