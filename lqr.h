// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LQR_H
#define _LQR_H

class LQR_c {
 public:
  float K_1;
  float K_2;
  float K_3;
  float Frequency;
  float last_error;
  // float last_i_error;
  float lqr_out;
  float error;
  float error_derivative;
  float error_integral;

  // Constructor, must exist.
  LQR_c() {}

  void init(float k_1, float k_2, float k_3, float frequency) {

    K_1 = k_1;
    K_2 = k_2;
    K_3 = k_3;
    Frequency = frequency;

  }

  float calulate_LQR(float speed_difference_sp, float value_cur) {

    error = 0 - value_cur;
    error_derivative = (last_error - error) / Frequency;

    lqr_out = K_1 * error + K_2 * error_derivative + K_3 * speed_difference_sp;

    last_error = error;

    return lqr_out;
  }
};

#endif
