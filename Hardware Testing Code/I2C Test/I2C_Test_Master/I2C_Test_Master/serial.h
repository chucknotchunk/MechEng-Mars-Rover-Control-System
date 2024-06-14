#ifndef _SERIAL_H
#define _SERIAL_H

void serial_init() {
  Serial.begin(9600);  // Start serial communication
}

#endif