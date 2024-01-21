# include <math.h>

float motor_pos[] = {0, 0, 0, 0, 0, 0};

const float DEG_TO_RAD = M_PI / 180.0; // Conversion factor from degrees to radians

extern const int NMOTORS;
extern const float axleTrack;
extern const float wheelBase;

void moveStraight(float distance) {
  for (int i = 0; i < NMOTORS; i++){
    // new tareget = old target + position change
    motor_pos[i] = motor_pos[i] + distance;
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
        motor_pos[3] = motor_pos[3] + turnArcRightOuter;
        motor_pos[4] = motor_pos[4] + turnArcRightInner;
        motor_pos[5] = motor_pos[5] + turnArcRightOuter;
        motor_pos[0] = motor_pos[0] + turnArcLeftOuter;
        motor_pos[1] = motor_pos[1] + turnArcLeftInner;
        motor_pos[2] = motor_pos[2] + turnArcLeftOuter;
    } else if (angle < 0) {
        // Left turn
        motor_pos[3] = motor_pos[3] + turnArcLeftOuter;
        motor_pos[4] = motor_pos[4] + turnArcLeftInner;
        motor_pos[5] = motor_pos[5] + turnArcLeftOuter;
        motor_pos[0] = motor_pos[0] + turnArcRightOuter;
        motor_pos[1] = motor_pos[1] + turnArcRightInner;
        motor_pos[2] = motor_pos[2] + turnArcRightOuter;
    }
}

/*
void driveMotors(int i){

  // Convert target distance to motor target speed
  float targetV_motor0 = max(-20,min(20,pid2.calculate(targetCounts, currentCounts, deltaT)));
  motorVelocity[0] = (int)pid1.calculate(TargetV, rpmFilt[0], deltaT);
  
  // Set the motor velocity
  setMotor(motorVelocity, NMOTORS);

}
*/