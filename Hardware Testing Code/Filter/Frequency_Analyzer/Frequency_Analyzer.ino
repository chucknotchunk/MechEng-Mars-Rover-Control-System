// Define libraries
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"
#include "serial.h"
#include "filter.h"
#include "PID.h"
#include "initialization.h"
#include "motor.h"
#include "motion.h"

// Define encoder pulses per revolution
#define pulsePerRev 11

// Define gear ratio
#define gearRatio 103
#define wheelRadius 0.1  // Assuming wheel radius of 0.1m

// Define wheel RPM limit
#define rpmLimit 20

// Define Axle track and wheelbase
const float axleTrack = 1.2;
const float wheelBase = 1;

// Define number of motors
const int NMOTORS = 1;

// Define encoder pins
const int ENCA[] = { 2 };
const int ENCB[] = { 3 };

// Define forward/reverse level pwm pins
const int RPWM[] = { 0 };
const int LPWM[] = { 1 };

// For demo motor driver only
const int IN1[] = { 1 };  // Motor driver direction pin
const int IN2[] = { 2 };  // Motor driver direction pin

// Global variables
volatile bool state = true;
volatile int pos_i[NMOTORS];
volatile int posPrev[NMOTORS];
volatile float velocity[NMOTORS];
volatile float rpm[NMOTORS];
volatile float rpmFilt[NMOTORS];
volatile float motorVelocity[NMOTORS];
volatile float motorTargetV[NMOTORS];
volatile float deltaT;
volatile float targetDistance = 0.0;
volatile float motorTargetPos[] = { 0, 0, 0, 0, 0, 0 };

// Initialize serial input
int SerialInput = 0;

// Declare an array of filters, one for each motor
LowPassFilter filter[NMOTORS];

// Initialize an array of PIDController for motor distance control
PIDController PID_distance[NMOTORS] = {
  PIDController(70, 0, 0),  // PID for motor0
                            /*
    PIDController(0.07, 0.01, 0.03), // PID for motor1
    */
};

// Initialize an array of PIDController for motor velocity control
PIDController PID_velocity[NMOTORS] = {
  PIDController(450, 350, 20),  // PID for motor0
                                /*
    PIDController(450, 350, 20), // PID for motor1
    */
};

void setup() {
  initialization();
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder<0>, RISING);  // Attach interrupt for encoders of each wheel
}

void loop() {
  // Get deltaT from the function
  calculateDeltaTime();

  // Map the pwm input, just for the demo setup
  pwm_map();

  // calculate the velocity for each motor
  calculatevelocity(deltaT);

  // Drive the motors
  driveMotors(state);

  // Debugging output
  Serial.print(rpm[0]);
  Serial.print(" ");
  Serial.print(rpmFilt[0]);
  Serial.println();
  //delay(1);
}

template<int j>
void readEncoder() {
  int b = digitalRead(ENCB[j]);  // Read encoder B when ENCA rises
  pos_i[j] += (b > 0) ? 1 : -1;  // Increment or decrement the position count based on the state of encoder pin B
}

float convertDistanceToCounts(float distance) {
  // Convert the target distance to encoder counts
  return distance / (wheelRadius * 2 * 3.14) * pulsePerRev * gearRatio;
}

float convertCountsToDistance(long counts) {
  // Calculate the distance based on encoder counts
  return (float)counts / (pulsePerRev * gearRatio) * (wheelRadius * 2 * M_PI);
}

void calculatevelocity(float deltaT) {
  int pos[NMOTORS];  // Temporary array to store current positions for each motor
  // Ensure atomic access to pos_i array to prevent data corruption
  noInterrupts();  // disable interrupts temporarily while reading
  for (int k = 0; k < NMOTORS; k++) {
    pos[k] = pos_i[k];  // Copy the current position from the shared pos_i array
  }
  interrupts();  // turn interrupts back on
  // Compute the velocity for each motor
  for (int k = 0; k < NMOTORS; k++) {
    velocity[k] = (pos[k] - posPrev[k]) / deltaT;           // Calculate the velocity as the difference in position divided by the time interval
    posPrev[k] = pos[k];                                    // Update the previous position for the next calculation
    rpm[k] = velocity[k] / pulsePerRev / gearRatio * 60.0;  // Convert count/s to RPM
    rpmFilt[k] = filter[k].update(rpm[k]);                  // Update the filtered RPM value using the low pass filter
  }
}

void driveMotors(bool state) {
  for (int k = 0; k < NMOTORS; k++) {
    if (state == true) {
      // Convert target distance to motor target speed
      //motorTargetV[k] = constrain(PID_distance[k].calculate(motorTargetPos[k], convertCountsToDistance(pos_i[k]), deltaT), -rpmLimit, rpmLimit);
      // PID control for motor speed
      // motorVelocity[k] = PID_velocity[0].calculate(motorTargetV[k], rpmFilt[k], deltaT);
      motorVelocity[k] = 4095;
    } else if (state == false) {
      motorVelocity[k] = 0;
    }
    // Set the motor velocity
    setMotor(motorVelocity, NMOTORS);
  }
}