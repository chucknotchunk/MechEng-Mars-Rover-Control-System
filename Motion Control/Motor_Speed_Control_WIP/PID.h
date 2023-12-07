#ifndef _PID_H
#define _PID_H

class PIDController {
  public:
    PIDController(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      integral = 0.0;
      previous_error = 0.0;
    }

    float calculate(float setpoint, float measured_value, float dt) {
      // Calculate error
      float error = setpoint - measured_value;

      // Proportional term
      float P_out = kp * error;

      // Integral term
      integral += error * dt;
      float I_out = ki * integral;

      // Derivative term
      float derivative = (error - previous_error) / dt;
      float D_out = kd * derivative;

      // Calculate total output
      float output = P_out + I_out + D_out;

      // Remember some variables for next time
      previous_error = error;

      return output;
    }

  private:
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float integral;
    float previous_error;
};

#endif