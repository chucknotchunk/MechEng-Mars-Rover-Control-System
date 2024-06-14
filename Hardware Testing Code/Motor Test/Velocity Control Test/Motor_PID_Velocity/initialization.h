#ifndef _INIT_H
#define _INIT_H

// Declare external variables for global access across different files
extern const int NMOTORS;  // Number of motors in the system
extern const int ENCA[];   // Array of pins for encoder A for each motor
extern const int ENCB[];   // Array of pins for encoder B for each motor
extern volatile float deltaT;

// Create an instance of the Adafruit PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setupEncoders();

// Function to initialize system components
void initialization(void) {
  //pwm_map_innit();  // Initialize the PWM map
  serial_innit();  // Initialize serial communication

  pwm.begin();                           // Start communication with PCA9685 PWM driver
  pwm.setOscillatorFrequency(27000000);  // Set the onboard oscillator of PCA9685 to 27 MHz
  pwm.setPWMFreq(1600);                  // Set the PWM frequency to 1.6 kHz, the maximum value for PCA9685
  setupEncoders();
}

// Function to setup encoders for all motors
void setupEncoders() {
  // Loop through all motors based on NMOTORS
  for (int k = 0; k < NMOTORS; k++) {
    // Set the encoder pins as input
    pinMode(ENCA[k], INPUT);  // Setup pin for encoder A of motor k
    pinMode(ENCB[k], INPUT);  // Setup pin for encoder B of motor k
  }
}

float calculateDeltaTime() {
  static unsigned long prevT = 0;             // Static variable to hold time of the last call
  unsigned long currT = micros();             // Get the current time
  deltaT = ((float)(currT - prevT)) / 1.0e6;  // Calculate the time difference in seconds
  prevT = currT;                              // Update prevT for the next call
}

#endif
