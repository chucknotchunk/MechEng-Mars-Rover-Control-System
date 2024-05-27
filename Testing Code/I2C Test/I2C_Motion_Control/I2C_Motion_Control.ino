// Define libraries
#include <math.h>
#include <Wire.h>                     // Library for I2C communication
#include <Adafruit_PWMServoDriver.h>  // Library for PWM breakout board control
#include <Derivs_Limiter.h>           // Library for derivatives limiting
#include "serial.h"
#include "filter.h"
#include "PID.h"
#include "initialization.h"
#include "motor.h"
#include "motion.h"


// Define encoder pulses per revolution
#define pulsePerRev 16

// Define gear ratio
#define gearRatio 55
#define wheelRadius 0.1  // Wheel radius of 0.1m

// Define wheel RPM limit
#define rpmLimit 50

// Define how close the current position should be to the target to consider it complete
#define POSITION_TOLERANCE 0.05

// Define Axle track and wheelbase
const float axleTrack = 1.2;
const float wheelBase = 1;

// Define number of motors
const int NMOTORS = 6;

// Define encoder pins
const int ENCA[] = { 2, 4, 6, 8, 10, 12 };
const int ENCB[] = { 3, 5, 7, 9, 11, 13 };

// Define forward/reverse level pwm pins
const int RPWM[] = { 4, 6, 8, 10, 12, 14 };
const int LPWM[] = { 5, 7, 9, 11, 13, 15 };

// Define directional multiplier for motor
const int dirMotor[] = { 1, 1, 1, 1, -1, -1 };

// Global variables
volatile bool roverActive = true;
volatile long pos_i[NMOTORS];
volatile int posPrev[NMOTORS];
volatile float velocity[NMOTORS];
volatile float rpm[NMOTORS];
volatile float rpmFilt[NMOTORS];
volatile float motorVelocity[NMOTORS];
volatile float motorTargetV[NMOTORS];
volatile float deltaT;
volatile float targetDistance = 0.0;
volatile float motorTargetPos[] = { 0, 0, 0, 0, 0, 0 };

// Initialize arrays for wheel linear acceleration and velocity limit
volatile float velLimit[NMOTORS] = { 0, 0, 0, 0, 0, 0 };
volatile float accLimit[NMOTORS] = { 0, 0, 0, 0, 0, 0 };

Derivs_Limiter limiter[NMOTORS];

// Initialize serial input
int SerialInput = 0;

// Declare an array of filters, one for each motor
LowPassFilter filter[NMOTORS];

// Initialize an array of PIDController for motor distance control
PIDController PID_distance[NMOTORS] = {
  PIDController(70, 0, 0),  // PID for motor0
  PIDController(70, 0, 0),  // PID for motor1
  PIDController(70, 0, 0),  // PID for motor2
  PIDController(70, 0, 0),  // PID for motor3
  PIDController(70, 0, 0),  // PID for motor4
  PIDController(70, 0, 0),  // PID for motor5
};

// Initialize an array of PIDController for motor velocity control
PIDController PID_velocity[NMOTORS] = {
  PIDController(400, 120, 0),  // PID for motor0
  PIDController(400, 120, 0),  // PID for motor1
  PIDController(400, 120, 0),  // PID for motor2
  PIDController(400, 120, 0),  // PID for motor3
  PIDController(400, 120, 0),  // PID for motor4
  PIDController(400, 120, 0),  // PID for motor5

};

void setup() {
  initialization();

  // set the velocity and acceleration limiter for each motor
  // for (int i = 0; i < NMOTORS; i++) {
  //   limiter[i] = Derivs_Limiter(velLimit[i], accLimit[i]);
  // }

  // attach external interrupt for encoder of each motor
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder<0>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder<1>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), readEncoder<2>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[3]), readEncoder<3>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[4]), readEncoder<4>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[5]), readEncoder<5>, RISING);  // Attach interrupt for encoders of each wheel
}

void loop() {
  // Calculate deltaT
  calculateDeltaTime();

  // Calculate the velocity for each motor
  calculatevelocity();
  // Take serial command
  serial_input();
  // Drive the motors
  driveMotors(roverActive);

  // Debugging output
  Serial.print(motorTargetPos[0]);
  Serial.print(" ");
  Serial.print(convertCountsToDistance(pos_i[0]));
  Serial.println();
  //delay(1);
}

template<int j>
void readEncoder() {
  int b = digitalRead(ENCB[j]);  // Read encoder B when ENCA rises
  pos_i[j] += (b > 0) ? 1 : -1;  // Increment or decrement the position count based on the roverActive  of encoder pin B
}

float convertDistanceToCounts(float distance) {
  // Convert the target distance to encoder counts
  return distance / (wheelRadius * 2 * 3.14) * pulsePerRev * gearRatio;
}

float convertCountsToDistance(long counts) {
  // Calculate the distance based on encoder counts
  return (float)counts / (pulsePerRev * gearRatio) * (wheelRadius * 2 * M_PI);
}

void calculatevelocity() {
  long pos[NMOTORS];  // Temporary array to store current positions for each motor
  // Ensure atomic access to pos_i array to prevent data corruption
  noInterrupts();  // disable interrupts temporarily while reading
  for (int k = 0; k < NMOTORS; k++) {
    pos[k] = pos_i[k];  // Copy the current position from the shared pos_i array
  }
  interrupts();  // turn interrupts back on

  // Compute the velocity for each motor
  for (int k = 0; k < NMOTORS; k++) {
    velocity[k] = (pos[k] - posPrev[k]) / deltaT;           // Calculate the velocity as counting pulses per time interval
    posPrev[k] = pos[k];                                    // Update the previous position for the next calculation
    rpm[k] = velocity[k] / pulsePerRev / gearRatio * 60.0;  // Convert count/s to RPM
    rpmFilt[k] = filter[k].update(rpm[k]);                  // Update the filtered RPM value using the low pass filter
  }
}

void driveMotors(bool roverActive) {
  for (int k = 0; k < NMOTORS; k++) {
    if (roverActive == true) {
      // set the velocity and acceleration limiter for each motor
      limiter[k] = Derivs_Limiter(velLimit[k], accLimit[k]);
      // Convert target distance to motor target speed, limiting the velocity and acceleration of target position change
      motorTargetV[k] = constrain(PID_distance[k].calculate(limiter[k].calc(motorTargetPos[k]), convertCountsToDistance(pos_i[k]), deltaT), -rpmLimit, rpmLimit);
      // PID control for motor speed
      motorVelocity[k] = PID_velocity[k].calculate(motorTargetV[k], rpmFilt[k], deltaT);
    } else if (roverActive == false) {
      motorVelocity[k] = 0;
    }
    // Set the motor velocity
    setMotor(k, motorVelocity[k]);
  }
}

// Function to check if last commanded motion is completed
bool isCommandComplete() {
  for (int k = 0; k < NMOTORS; k++) {
    if (abs(convertCountsToDistance(pos_i[k]) - motorTargetPos[k]) > POSITION_TOLERANCE) {
      return false;
    }
  }
  return true;
}