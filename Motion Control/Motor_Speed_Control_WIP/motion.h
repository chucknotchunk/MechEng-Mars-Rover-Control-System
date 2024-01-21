#ifndef _MOTION_H
#define _MOTION_H

# include <math.h>

extern volatile float motorTargetPos[];

extern const int NMOTORS;
extern const float axleTrack;
extern const float wheelBase;

void moveStraight(float distance) {
  for (int i = 0; i < NMOTORS; i++){
    // new tareget = old target + position change
    motorTargetPos[i] = motorTargetPos[i] + distance;
  }
}

void turnByRadius(float radius, int angle) {
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
        motorTargetPos[3] = motorTargetPos[3] + turnArcRightOuter;
        motorTargetPos[4] = motorTargetPos[4] + turnArcRightInner;
        motorTargetPos[5] = motorTargetPos[5] + turnArcRightOuter;
        motorTargetPos[0] = motorTargetPos[0] + turnArcLeftOuter;
        motorTargetPos[1] = motorTargetPos[1] + turnArcLeftInner;
        motorTargetPos[2] = motorTargetPos[2] + turnArcLeftOuter;
    } else if (angle < 0) {
        // Left turn
        motorTargetPos[3] = motorTargetPos[3] + turnArcLeftOuter;
        motorTargetPos[4] = motorTargetPos[4] + turnArcLeftInner;
        motorTargetPos[5] = motorTargetPos[5] + turnArcLeftOuter;
        motorTargetPos[0] = motorTargetPos[0] + turnArcRightOuter;
        motorTargetPos[1] = motorTargetPos[1] + turnArcRightInner;
        motorTargetPos[2] = motorTargetPos[2] + turnArcRightOuter;
    }
}

#endif 