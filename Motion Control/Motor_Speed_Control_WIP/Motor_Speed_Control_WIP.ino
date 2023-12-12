#include <Wire.h>
#include <util/atomic.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"
#include "serial.h"
#include "filter.h"
#include "PID.h"
#include "initialization.h"


const int NMOTORS = 1; // define numver of motors 

const int ENCA[] = {2};
const int ENCB[] = {3};
const int RPWM[] = {0};
//const int LPWM[] = {};
const int IN1[] = {1}; // motor driver direction pin
const int IN2[] = {2}; // motor driver direction pin

volatile int pos_i[NMOTORS];
volatile int posPrev[NMOTORS];
volatile float velocity[NMOTORS];
volatile float rpm[NMOTORS];
volatile float rpmFilt[NMOTORS];
volatile float motorVelocity[NMOTORS];
volatile float deltaT;


int SerialInput = 0;  // Variable to store the last PWM value

LowPassFilter filter1; // Filter for the first signal
PIDController pid1(12.5, 5, 2); // PID controller for process 1

void setup() {

  initialization();

  setupEncoders();

  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder<0>, RISING);
} 

void loop() {
  float deltaT = calculateDeltaTime(); // get deltaT from the function

  pwm_map(); // map the pwm input

  calculatevelocity(deltaT);

  // Set a target
  SerialInput = serial_input();
  float TargetV = SerialInput;

  // Compute the control signal u  
  motorVelocity[0] = pid1.calculate(SerialInput, rpmFilt[0], deltaT);
  
  // Set the motor speed and direction
  setMotor((motorVelocity[0] < 0) ? -1 : 1, min((int)fabs(motorVelocity[0] * 20), 4095));

  Serial.print(TargetV);
  Serial.print(" ");
  Serial.print(rpmFilt[0]);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal) {
  pwm.setPWM(0, 0, pwmVal);
  if (dir == 1) { 
    // Turn one way
    pwm.setPin(1, 0, 0);
    pwm.setPin(2, 0, 4096);
  } else if (dir == -1) {
    // Turn the other way
    pwm.setPin(1, 0, 4096);
    pwm.setPin(2, 0, 0);
  } else {
    // Or don't turn
    pwm.setPin(1, 0, 0);
    pwm.setPin(2, 0, 0);
  }
}

template <int j>
void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB[j]);
  pos_i[j] += (b > 0) ? 1 : -1;
}

float calculateDeltaTime(){
  static unsigned long prevT = 0; // static variable to hold time of the last call
  unsigned long currT = micros(); // get the current time
  deltaT = ((float) (currT - prevT)) / 1.0e6; // calculate the time difference in seconds
  prevT = currT; // update prevT for the next call
  return deltaT; // return the calculated deltaT
}

void calculatevelocity(float deltaT){
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = pos_i[k];
      }
  }

  // Compute velocity
  for (int k = 0; k < NMOTORS; k++){
    long currT = micros();
    velocity[k] = (pos[k] - posPrev[k]) / deltaT;
    posPrev[k] = pos[k];

    // Convert count/s to RPM
    rpm[k] = velocity[k] / 600.0 * 60.0;

    rpmFilt[k] = filter1.update(rpm[k]);
    }
}
