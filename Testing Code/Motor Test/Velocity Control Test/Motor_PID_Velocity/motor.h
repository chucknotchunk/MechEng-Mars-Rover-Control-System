#ifndef _MOTOR_H
#define _MOTOR_H

// Declare external variables for global access across different files
extern const int RPWM[];
extern const int LPWM[];

void setMotor(int motorIndex, int motorVelocity) {

    // Determine direction based on the sign of motorVelocity
    int dir = (motorVelocity > 0) ? 1 : (motorVelocity < 0) ? -1
                                                            : 0;
    int absPwmVal = abs(motorVelocity);

    if (dir == 1) {
      // Turn one way
      pwm.setPWM(RPWM[motorIndex], 0, min(absPwmVal, 4095));
      pwm.setPWM(LPWM[motorIndex], 0, 0);
    } else if (dir == -1) {
      // Turn the other way
      pwm.setPWM(LPWM[motorIndex], 0, min(absPwmVal, 4095));
      pwm.setPWM(RPWM[motorIndex], 0, 0);
    } else {
      // Or don't turn
      pwm.setPWM(RPWM[motorIndex], 0, 0);
      pwm.setPWM(LPWM[motorIndex], 0, 0);
    }

}


#endif