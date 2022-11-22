// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

// TODO: Class to contain generic PID algorithm.
/*! 12312312
 */
class PID_c {
 public:
  float K_P;
  float K_I;
  float K_D;
  float Frequency;
  float last_d_error;
  float last_i_error;
  float pid_out = 0;
  float error = 0;
  float error_derivative = 0;
  float error_integral = 0;

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
    error_derivative = error / Frequency;
    error_integral += error;

    pid_out = K_P * error + K_I * error_integral + K_D * error_derivative;

    last_d_error = error_derivative;
    last_i_error += error_integral;

    return pid_out;
  }
};

#endif
