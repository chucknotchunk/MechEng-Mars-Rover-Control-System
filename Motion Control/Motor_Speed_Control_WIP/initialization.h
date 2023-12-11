#ifndef _INIT_H
#define _INIT_H

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void initialization(void){
  pwm_map_innit();
  serial_innit();

  pwm.begin();                  // Enable communication with PCA9685
  pwm.setOscillatorFrequency(27000000);  // Set onboard oscillator to 27MHz
  pwm.setPWMFreq(1600);         // Set PWM frequency to 1.6kHz (max value)
}

#endif 