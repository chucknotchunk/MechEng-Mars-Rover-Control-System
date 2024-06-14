#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
  pwm.begin();                           // Start communication with PCA9685 PWM driver
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685
}

void loop() {
  // put your main code here, to run repeatedly:
   pwm.setPWM(4, 0, 2000);
   pwm.setPWM(5, 0, 0);
}
