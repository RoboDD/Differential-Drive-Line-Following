// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

class PID_c {
 public:
  float K_P;
  float K_I;
  float K_D;
  float Frequency;
  float last_error;
  // float last_i_error;
  float pid_out;
  float error;
  float error_derivative;
  float error_integral;

  // Constructor, must exist.
  PID_c() {}

  void init(float k_p, float k_i, float k_d, float frequency) {

    K_P = k_p;
    K_I = k_i;
    K_D = k_d;
    Frequency = frequency;

  }

  float calulate_PID(float value_des, float value_cur) {

    error = value_des - value_cur;
    error_derivative = (last_error - error) / Frequency;
    error_integral += error;

    pid_out = K_P * error + K_I * error_integral + K_D * error_derivative;

    last_error = error;

    return pid_out;
  }
};

#endif
