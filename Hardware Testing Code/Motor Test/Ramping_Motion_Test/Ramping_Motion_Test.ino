#include "serial.h"
#include <Derivs_Limiter.h>

// Define number of motors
const int NMOTORS = 2;

Derivs_Limiter DL[NMOTORS];

float velLimit[NMOTORS] = { 150, 50 };
float accLimit[NMOTORS] = { 50, 10 };

void setup() {
  // put your setup code here, to run once:
  serial_innit();
  for (int i = 0; i < NMOTORS; i++) {
    DL[i] = Derivs_Limiter(velLimit[i], accLimit[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  int SerialInput = serial_input();
  float limit = limiter.calc(SerialInput);

  Serial.print(SerialInput);
  Serial.print(" ");
  Serial.print(limiter.getPosition());
  //Serial.print(",");
  //Serial.print(limiter.getVelocity());
  //Serial.print(",");
  //Serial.print(limiter.getAcceleration());
  Serial.println();
}
