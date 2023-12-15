// Define libraries
#include <Wire.h>
#include <util/atomic.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"
#include "serial.h"
#include "filter.h"
#include "PID.h"
#include "initialization.h"
#include "motor.h"

// Define encoder pulses per revolution
#define pulsePerRev 11

// Define gear ratio
#define gearRatio 103
#define wheelRadius 10 // Assuming wheel radius of 10 cm

// Define number of motors 
const int NMOTORS = 1; 

// Define encoder pins
const int ENCA[] = {2};
const int ENCB[] = {3};

// Define forward/reverse level pwm pins
const int RPWM[] = {0};

// const int LPWM[] = {};
const int IN1[] = {1}; // Motor driver direction pin
const int IN2[] = {2}; // Motor driver direction pin

// Global variables
volatile int pos_i[NMOTORS];
volatile int posPrev[NMOTORS];
volatile float velocity[NMOTORS];
volatile float rpm[NMOTORS];
volatile float rpmFilt[NMOTORS];
volatile float motorVelocity[NMOTORS];
volatile float deltaT;
float targetDistance = 0.0;
volatile long targetCounts = 0;
volatile long currentCounts = 0;

// Initialize serial input 
int SerialInput = 0;

// Declare an array of filters, one for each motor
LowPassFilter filter[NMOTORS];

// PID controller for motor1
PIDController pid1(450, 350, 20);

// PID controller for distance control
PIDController pid2(0.07, 0.01, 0.03);

void setup() {
  initialization();
  setupEncoders();
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder<0>, RISING);
} 

void loop() {
  // Get deltaT from the function
  float deltaT = calculateDeltaTime();
  
  // Map the pwm input
  pwm_map(); 
  
  // calculate the velocity for each motor
  calculatevelocity(deltaT);

  // Read current encoder counts
  currentCounts = readEncoderCounts();
  
  // Set a target from serial input
  SerialInput = serial_input();
  //float TargetV = SerialInput;
  setTargetDistance(SerialInput);
  
  // Compute the control signal motorVelocity as input  
  float TargetV = max(-20,min(20,pid2.calculate(targetCounts, currentCounts, deltaT)));
  motorVelocity[0] = (int)pid1.calculate(TargetV, rpmFilt[0], deltaT);
  
  // Set the motor velocity
  setMotor(motorVelocity, NMOTORS);
  
  // Serial print data for debugging
  /*
  Serial.print(TargetV);
  Serial.print(" ");
  Serial.print(rpmFilt[0]);
  Serial.println();
  delay(1);
  */

  // Debugging output
  //Serial.print("Target: ");
  Serial.print(SerialInput);
  //Serial.print(" Current: ");
  Serial.print(" ");
  Serial.print(convertCountsToDistance(currentCounts));
  Serial.println();
  delay(1);
}

template <int j>
void readEncoder() {
  int b = digitalRead(ENCB[j]); // Read encoder B when ENCA rises
  pos_i[j] += (b > 0) ? 1 : -1; // Increment or decrement the position count based on the state of encoder pin B
}

void setTargetDistance(float distance) {
  targetDistance = distance;
  targetCounts = convertDistanceToCounts(targetDistance);
}

long readEncoderCounts() {
  // Returns the current encoder count
  return pos_i[0];
}

float convertDistanceToCounts(float distance) {
  // Convert the target distance to encoder counts
  return 100 * distance / (wheelRadius * 2 * 3.14) * pulsePerRev * gearRatio;
}

float convertCountsToDistance(long counts) {
    // Calculate the distance based on encoder counts
    return (float)counts / (pulsePerRev * gearRatio) * (wheelRadius * 2 * 3.14) / 100;
}

void calculatevelocity(float deltaT){
  int pos[NMOTORS]; // Temporary array to store current positions for each motor
  // Ensure atomic access to pos_i array to prevent data corruption
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = pos_i[k]; // Copy the current position from the shared pos_i array
    }
  }

  // Compute the velocity for each motor
  for (int k = 0; k < NMOTORS; k++){
    velocity[k] = (pos[k] - posPrev[k]) / deltaT; // Calculate the velocity as the difference in position divided by the time interval
    posPrev[k] = pos[k]; // Update the previous position for the next calculation
    rpm[k] = velocity[k] / pulsePerRev / gearRatio * 60.0; // Convert count/s to RPM
    rpmFilt[k] = filter[k].update(rpm[k]); // Update the filtered RPM value using the low pass filter
  }
}