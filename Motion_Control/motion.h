#ifndef _MOTION_H
#define _MOTION_H

#include <math.h>

extern volatile float motorTargetPos[];
extern const int dirMotor[];

extern const int NMOTORS;
extern const float axleTrack;
extern const float wheelBase;

void moveStraight(float distance) {
  for (int i = 0; i < NMOTORS; i++) {
    // new tareget = old target + position change
    motorTargetPos[i] = motorTargetPos[i] + dirMotor[i] * distance;
  }
}

void turnByRadius(int angle, float radius) {
  // Constants for the geometric calculations
  float rightInnerRadius = radius - axleTrack / 2;
  float rightOuterRadius = radius + axleTrack / 2;
  float turnArcRightInner = angle * DEG_TO_RAD * rightInnerRadius;
  float turnArcRightOuter = angle * DEG_TO_RAD * sqrt(pow(rightInnerRadius, 2) + pow(wheelBase / 2, 2));
  float turnArcLeftInner = angle * DEG_TO_RAD * rightOuterRadius;
  float turnArcLeftOuter = angle * DEG_TO_RAD * sqrt(pow(rightOuterRadius, 2) + pow(wheelBase / 2, 2));

  // Update motor positions based on the turn direction
  if (angle > 0) {
    // Right turn
    motorTargetPos[0] = motorTargetPos[0] + dirMotor[0] * turnArcRightOuter;
    motorTargetPos[2] = motorTargetPos[2] + dirMotor[2] * turnArcRightInner;
    motorTargetPos[4] = motorTargetPos[4] + dirMotor[4] * turnArcRightOuter;
    motorTargetPos[1] = motorTargetPos[1] + dirMotor[1] * turnArcLeftOuter;
    motorTargetPos[3] = motorTargetPos[3] + dirMotor[3] * turnArcLeftInner;
    motorTargetPos[5] = motorTargetPos[5] + dirMotor[5] * turnArcLeftOuter;
  } else if (angle < 0) {
    // Left turn
    motorTargetPos[0] = motorTargetPos[0] + dirMotor[0] * turnArcLeftOuter;
    motorTargetPos[2] = motorTargetPos[2] + dirMotor[2] * turnArcLeftInner;
    motorTargetPos[4] = motorTargetPos[4] + dirMotor[4] * turnArcLeftOuter;
    motorTargetPos[1] = motorTargetPos[1] + dirMotor[1] * turnArcRightOuter;
    motorTargetPos[3] = motorTargetPos[3] + dirMotor[3] * turnArcRightInner;
    motorTargetPos[5] = motorTargetPos[5] + dirMotor[5] * turnArcRightOuter;
  }
}

#endif