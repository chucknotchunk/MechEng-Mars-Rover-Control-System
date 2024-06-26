#ifndef _MOTOR_H
#define _MOTOR_H

// Declare external variables for global access across different files
extern const int NMOTORS;  // Number of motors in the system
extern const int RPWM[];
extern const int LPWM[];
extern const int IN1[];
extern const int IN2[];

void setMotor(volatile float motorVelocity[], int numMotors) {
  for (int k = 0; k < numMotors; k++) {
    // Determine direction based on the sign of motorVelocity
    int dir = (motorVelocity[k] > 0) ? 1 : (motorVelocity[k] < 0) ? -1
                                                                  : 0;
    int absPwmVal = abs(motorVelocity[k]);

    if (dir == 1) {
      // Turn one way
      pwm.setPWM(RPWM[k], 0, min(absPwmVal, 4095));

    } else if (dir == -1) {
      // Turn the other way
      pwm.setPWM(LPWM[k], 0, min(absPwmVal, 4095));
    } else {
      // Or don't turn
      pwm.setPWM(RPWM[k], 0, 0);
      pwm.setPWM(LPWM[k], 0, 0);
    }
  }
}

#endif