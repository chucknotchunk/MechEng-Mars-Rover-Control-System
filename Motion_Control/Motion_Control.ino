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
#include "I2C.h"

// Define encoder pulses per revolution
#define pulsePerRev 16

// Define gear ratio
#define gearRatio 55
#define wheelRadius 0.1  // Wheel radius of 0.1m

// Define wheel RPM limit
#define rpmLimit 50

// Define how close the current position should be to the target to consider it complete
#define positionTolerance 0.05

// Define the pins for interrupt and output
// Arduino Nano BLE: Pin 0 Interrupt, Pin 1 Output
// Arduino UNO Testing: Pin 2 Interrupt, Pin 3 Output
#define interruptPin 0
#define outputPin 1

// Define Axle track and wheelbase
const float axleTrack = 1.2;
const float wheelBase = 1;

// Define number of motors
const uint8_t NMOTORS = 6;

// Define encoder pins
const uint8_t ENCA[] = { 2, 4, 6, 8, 10, 12 };
const uint8_t ENCB[] = { 3, 5, 7, 9, 11, 13 };

// Define forward/reverse level pwm pins
const uint8_t RPWM[] = { 4, 6, 8, 10, 12, 14 };
const uint8_t LPWM[] = { 5, 7, 9, 11, 13, 15 };

// Define directional multiplier for motor
const int dirMotor[] = { 1, 1, 1, 1, -1, -1 };

// Define a flag to indicate the completion of a command
bool commandCompleteFlag = false;

// Global variables
volatile bool roverMoving = false;
volatile bool interruptFlag = false;
volatile bool pwmSetupDone = true;
volatile long pos_i[NMOTORS];
volatile long posPrev[NMOTORS];
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

  pwm.begin();                           // Start communication with PCA9685 PWM driver
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685

  // Set all motors to stopped
  for (int i = 0; i < NMOTORS; i++) {
    pwm.setPWM(RPWM[i], 0, 0);
    pwm.setPWM(LPWM[i], 0, 0);
  }

  i2cInit();  // Initialize I2C communication

  // Configure the pins
  pinMode(interruptPin, INPUT);
  pinMode(outputPin, OUTPUT);

  // attach external interrupt for encoder of each motor
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder<0>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder<1>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), readEncoder<2>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[3]), readEncoder<3>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[4]), readEncoder<4>, RISING);  // Attach interrupt for encoders of each wheel
  attachInterrupt(digitalPinToInterrupt(ENCA[5]), readEncoder<5>, RISING);  // Attach interrupt for encoders of each wheel

  // Attach interrupt to the pin for emergency stop
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
}

void loop() {
  // Calculate deltaT
  calculateDeltaTime();

  // Calculate the velocity for each motor
  calculatevelocity();
  // Take serial command
  //serial_input();

  // Check if the command is complete or interrupted
  // bool commandComplete = interruptFlag;  // For testing
  bool commandComplete = isCommandComplete() || interruptFlag;

  if (commandComplete) {
    if (!commandCompleteFlag) {
      roverMoving = false;
      Serial.println("Rover stopped.");
      // Inform the central controller that the last command is completed / Motors stopped
      digitalWrite(outputPin, HIGH);
      // Add a short delay to ensure the master detects the high signal
      delay(10);
      // Reset the pin to low to avoid keeping it high
      digitalWrite(outputPin, LOW);
      // Set the flag to indicate the command completion has been handled
      commandCompleteFlag = true;
      // Reset the target position to the current position
      resetTargetPositionToCurrent();
      i2cInit();  // Initialize I2C communication
      pwmSetupDone = false;
    }
  } else {
    if (commandCompleteFlag) {
      // Reset the flag to indicate we are back to normal operation
      commandCompleteFlag = false;
    }
    digitalWrite(outputPin, LOW);
  }

  if (roverMoving && !pwmSetupDone) {
    pwm.begin();                           // Start communication with PCA9685 PWM driver
    pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
    pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685
    pwmSetupDone = true;                   // Update the flag to indicate PWM setup is done
  }

  if (roverMoving) {
    // Drive the motors
    driveMotors(roverMoving);
  }
}

template<int j>
void readEncoder() {
  int b = digitalRead(ENCB[j]);  // Read encoder B when ENCA rises
  pos_i[j] += (b > 0) ? 1 : -1;  // Increment or decrement the position count based on the roverMoving  of encoder pin B
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

void driveMotors(bool state) {
  for (int k = 0; k < NMOTORS; k++) {
    if (state == true) {
      // set the velocity and acceleration limiter for each motor
      limiter[k] = Derivs_Limiter(velLimit[k], accLimit[k]);
      // Convert target distance to motor target speed, limiting the velocity and acceleration of target position change
      motorTargetV[k] = constrain(PID_distance[k].calculate(limiter[k].calc(motorTargetPos[k]), convertCountsToDistance(pos_i[k]), deltaT), -rpmLimit, rpmLimit);
      // PID control for motor speed
      motorVelocity[k] = PID_velocity[k].calculate(motorTargetV[k], rpmFilt[k], deltaT);
    } else if (state == false) {
      motorVelocity[k] = 0;
    }
    // Set the motor velocity
    setMotor(k, motorVelocity[k]);
  }
}

// Function to check if last commanded motion is completed
bool isCommandComplete() {
  for (int k = 0; k < NMOTORS; k++) {
    if (abs(convertCountsToDistance(pos_i[k]) - motorTargetPos[k]) > positionTolerance) {
      return false;
    }
  }
  return true;
}

// Function to reset the target position to the current position
void resetTargetPositionToCurrent() {
  for (int k = 0; k < NMOTORS; k++) {
    motorTargetPos[k] = convertCountsToDistance(pos_i[k]);
  }
}

void handleInterrupt() {
  // Set the flag to indicate the interrupt occurred
  interruptFlag = true;
  // Serial.println(interruptFlag);
}