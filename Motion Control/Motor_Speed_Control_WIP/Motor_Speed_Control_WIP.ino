#include <Wire.h>
#include <util/atomic.h>
#include <Adafruit_PWMServoDriver.h>
#include "pwm_map.h"
#include "serial.h"
#include "filter.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pins
#define ENCA 2 // encoder input
#define ENCB 3 // encoder input

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;

float eintegral = 0;
int SerialInput = 0;  // Variable to store the last PWM value

LowPassFilter filter1; // Filter for the first signal

void setup() {
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  pwm.begin();                  // Enable communication with PCA9685
  pwm.setOscillatorFrequency(27000000);  // Set onboard oscillator to 27MHz
  pwm.setPWMFreq(1600);         // Set PWM frequency to 1.6kHz (max value)

  pwm_map_innit();
  serial_innit();

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {


  pwm_map(); // map the pwm input

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1 / 600.0 * 60.0;

  // Low-pass filter (25 Hz cutoff)

  v1Filt = filter1.update(v1);

  // Set a target
  SerialInput = serial_input();
  float vt = SerialInput;

  // Compute the control signal u
  float kp = 10;
  float ki = 10;
  float e = vt - v1Filt;
  eintegral = eintegral + e * deltaT;
  
  float u = kp * e + ki * eintegral;

  // Set the motor speed and direction
  setMotor((u < 0) ? -1 : 1, min((int)fabs(u * 20), 4095));

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
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

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  pos_i += (b > 0) ? 1 : -1;
}
