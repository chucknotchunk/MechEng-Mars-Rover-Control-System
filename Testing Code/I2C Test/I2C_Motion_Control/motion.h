#ifndef _MOTION_H
#define _MOTION_H

#include <math.h>
#include "helper.h"

extern volatile float motorTargetPos[];

extern volatile float velLimit[];
extern volatile float accLimit[];

extern const int dirMotor[];

extern const uint8_t  NMOTORS;
extern const float axleTrack;
extern const float wheelBase;

void moveStraight(float distance) {
  for (int i = 0; i < NMOTORS; i++) {
    // Set the velocity and acceleration of position change
    accLimit[i] = 0.1;
    velLimit[i] = 0.1;
    // new tareget = old target + position change
    motorTargetPos[i] += dirMotor[i] * distance;
  }
}

void turnByRadius(int angle, float radius) {
  // Motor position increment for each motion
  float motorIncrementPos[] = { 0, 0, 0, 0, 0, 0 };
  float accMax = 0.1;    // Linear acceleration limit of the wheel
  float tbRatio = 0.25;  // Parabolic blend time to total time ratio

  // Constants for the geometric calculations
  float leftInnerRadius = radius - axleTrack / 2;
  float leftOuterRadius = radius + axleTrack / 2;
  float turnArcLeftInner = abs(angle) * DEG_TO_RAD * leftInnerRadius;
  float turnArcLeftOuter = abs(angle) * DEG_TO_RAD * mySign(leftInnerRadius) * sqrt(pow(leftInnerRadius, 2) + pow(wheelBase / 2, 2));
  float turnArcRightInner = abs(angle) * DEG_TO_RAD * leftOuterRadius;
  float turnArcRightOuter = abs(angle) * DEG_TO_RAD * mySign(leftOuterRadius) * sqrt(pow(leftOuterRadius, 2) + pow(wheelBase / 2, 2));

  // Update motor target positions based on the turn direction
  if (angle > 0) {
    // Left turn
    motorIncrementPos[0] = dirMotor[0] * turnArcLeftOuter;
    motorIncrementPos[2] = dirMotor[2] * turnArcLeftInner;
    motorIncrementPos[4] = dirMotor[4] * turnArcLeftOuter;
    motorIncrementPos[1] = dirMotor[1] * turnArcRightOuter;
    motorIncrementPos[3] = dirMotor[3] * turnArcRightInner;
    motorIncrementPos[5] = dirMotor[5] * turnArcRightOuter;
  } else if (angle < 0) {
    // Right turn
    motorIncrementPos[0] = dirMotor[0] * turnArcRightOuter;
    motorIncrementPos[2] = dirMotor[2] * turnArcRightInner;
    motorIncrementPos[4] = dirMotor[4] * turnArcRightOuter;
    motorIncrementPos[1] = dirMotor[1] * turnArcLeftOuter;
    motorIncrementPos[3] = dirMotor[3] * turnArcLeftInner;
    motorIncrementPos[5] = dirMotor[5] * turnArcLeftOuter;
  }

  float maxDistance = findMaxValue(motorIncrementPos, NMOTORS);
  // Calculate total steering time based on blend time to total time ratio and acceleration limit
  float tf = sqrt(maxDistance / accMax / (tbRatio - pow(tbRatio, 2)));
  float th = tf / 2;  // Half time of the total motion
  float tb = th - sqrt(pow(accMax, 2) * pow(th, 2) - accMax * maxDistance) / accMax;

  // Calculate the required velocity and acceleration for each wheel ensuring synchronized movements (linear function with parabolic blend)
  for (int k = 0; k < NMOTORS; k++) {
    accLimit[k] = motorIncrementPos[k] / (2 * th * tb - pow(tb, 2));
    velLimit[k] = accLimit[k] * tb;
    motorIncrementPos[k] += motorIncrementPos[k];  // Update motor target position
  }
}

#endif