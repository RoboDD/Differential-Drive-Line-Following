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

  // Constructor, must exist.
  PID_c() {}

  void init(float k_p, float k_i, float k_d) {
    K_P = k_p;
    K_I = k_i;
    K_D = k_d;
  }

  float calulate_PID(float value_des, float value_cur, float time_duration) {
    float pid_out = 0;
    float error = 0;
    float error_derivative = 0;
    float error_integral = 0;

    error = value_des - value_cur;
    error_derivative = error / time_duration;
    error_integral += error;

    pid_out = K_P * error + K_I * error_integral + K_D * error_derivative;

    return pid_out;
  }
};

#endif
