// Define libraries
#include <Wire.h>
#include <util/atomic.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"
#include "serial.h"
#include "filter.h"
#include "PID.h"
#include "initialization.h"
// Define encoder pulses per revolution
#define pulsePerRev 11
// Define gear ratio
#define gearRatio 103

// Define number of motors 
const int NMOTORS = 1; 
// Define encoder pins
const int ENCA[] = {2};
const int ENCB[] = {3};
// Define forward/reverse level pwm pins
const int RPWM[] = {0};
// Const int LPWM[] = {};
const int IN1[] = {1}; // Motor driver direction pin
const int IN2[] = {2}; // Motor driver direction pin
// Declare global variables 
volatile int pos_i[NMOTORS];
volatile int posPrev[NMOTORS];
volatile float velocity[NMOTORS];
volatile float rpm[NMOTORS];
volatile float rpmFilt[NMOTORS];
volatile float motorVelocity[NMOTORS];
volatile float deltaT;
// Initialize serial input 
int SerialInput = 0;
// Declare an array of filters, one for each motor
LowPassFilter filter[NMOTORS];
// PID controller for motor1
PIDController pid1(450, 300, 20);

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
  // Set a target from serial input
  SerialInput = serial_input();
  float TargetV = SerialInput;
  // Compute the control signal motorVelocity as inpput  
  motorVelocity[0] = (int)pid1.calculate(TargetV, rpmFilt[0], deltaT);
  // Set the motor velocity
  setMotor();
  
  // Serial print data for debugging
  Serial.print(TargetV);
  Serial.print(" ");
  Serial.print(rpmFilt[0]);
  Serial.println();
  delay(1);
}

void setMotor() {
  for(int k = 0; k < NMOTORS; k++){
    // Determine direction based on the sign of motorVelocity
    int dir = (motorVelocity[k] > 0) ? 1 : (motorVelocity[k] < 0) ? -1 : 0;
    int absPwmVal = abs(motorVelocity[k]);

    if (dir == 1) { 
        // Turn one way
        pwm.setPWM(RPWM[k], 0, min(absPwmVal, 4095));
        pwm.setPin(IN1[k], 0, 0);
        pwm.setPin(IN2[k], 0, 4096);
    } else if (dir == -1) {
        // Turn the other way
        pwm.setPWM(RPWM[k], 0, min(absPwmVal, 4095));
        pwm.setPin(IN1[k], 0, 4096);
        pwm.setPin(IN2[k], 0, 0);
    } else {
        // Or don't turn
        pwm.setPin(IN1[k], 0, 0);
        pwm.setPin(IN2[k], 0, 0); 
    }
  }
}

template <int j>
void readEncoder() {
  int b = digitalRead(ENCB[j]); // Read encoder B when ENCA rises
  pos_i[j] += (b > 0) ? 1 : -1; // Increment or decrement the position count based on the state of encoder pin B
}

float calculateDeltaTime(){
  static unsigned long prevT = 0; // Static variable to hold time of the last call
  unsigned long currT = micros(); // Get the current time
  deltaT = ((float) (currT - prevT)) / 1.0e6; // Calculate the time difference in seconds
  prevT = currT; // Update prevT for the next call
  return deltaT; // Return the calculated deltaT
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