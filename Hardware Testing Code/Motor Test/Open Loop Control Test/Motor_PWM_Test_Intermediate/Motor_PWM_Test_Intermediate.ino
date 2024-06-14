#include <Adafruit_PWMServoDriver.h>

// Define number of motors
const int NMOTORS = 6;

// Define forward/reverse level pwm pins
const int RPWM[] = {4, 6, 8, 10, 12, 14};
const int LPWM[] = {5, 7, 9, 11, 13, 15};

int LastInput = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(9600);                    // Initialize serial communication
  pwm.begin();                           // Start communication with PCA9685 PWM driver
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685
}

void loop() {
  setMotor(serial_input(), NMOTORS);
}

void setMotor(int motorVelocity, int numMotors) {
  for (int k = 0; k < numMotors; k++) {
    // Determine direction based on the sign of motorVelocity
    int dir = (motorVelocity > 0) ? 1 : (motorVelocity < 0) ? -1
                                                            : 0;
    int absPwmVal = abs(motorVelocity);

    if (dir == 1) {
      // Turn one way
      pwm.setPWM(RPWM[k], 0, min(absPwmVal, 4095));
      pwm.setPWM(LPWM[k], 0, 0);
    } else if (dir == -1) {
      // Turn the other way
      pwm.setPWM(LPWM[k], 0, min(absPwmVal, 4095));
      pwm.setPWM(RPWM[k], 0, 0);
    } else {
      // Or don't turn
      pwm.setPWM(RPWM[k], 0, 0);
      pwm.setPWM(LPWM[k], 0, 0);
    }
  }
}

int serial_input() {
  if (Serial.available() > 0) {
    int SerialInput = Serial.parseInt();  // Read the next integer from serial input
    if (Serial.read() != '\n') {          // Check if a newline character follows the integer
      // If not a newline, the input is not valid or complete
      SerialInput = LastInput;  // Keep the last valid value
    }
    if (SerialInput >= -4095 && SerialInput <= 4095) {
      LastInput = SerialInput;  // Update LastInput with the new value
    }
  }

  return LastInput;
}