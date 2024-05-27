#ifndef _PWM_MAP_H
#define _PWM_MAP_H

void pwm_map_innit() {
  pinMode(4, INPUT);  // Set the pin for PWM mapping input
  // pinMode(9, OUTPUT);  // Set the pin for PWM mapping input
}

void pwm_map() {

  unsigned long highPulse = pulseIn(4, HIGH);
  unsigned long lowPulse = pulseIn(4, LOW);
  unsigned long period = highPulse + lowPulse;
  double dutyCycle = (double(highPulse) / period) * 100;

  // analogWrite(9, 2.25 * dutyCycle);  // Map duty cycle to analogWrite value

  // Print the results
  Serial.print("Duty Cycle: ");
  Serial.print(highPulse);
  Serial.println("%");
}

#endif