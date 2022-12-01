// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#define DEBUG_MODE false

// Class to track robot position.
class Kinematics_c {
 public:
  int last_count_left;
  int last_count_right;
  unsigned long start_time;
  unsigned long duration;
  double omega_left;
  double omega_right;

  // Constructor, must exist.
  Kinematics_c() {}

  // Use this function to update
  // your kinematics
  void update() {
    start_time = micros();
    last_count_left = count_left;
    last_count_right = count_right;
    delay(100);
    duration = micros() - start_time;
    last_count_left = count_left - last_count_left;
    last_count_right = count_right - last_count_right;

    omega_left = last_count_left / duration;
    omega_right = last_count_right / duration;

    if (DEBUG_MODE == true) {
      Serial.print("L-omega: ");
      Serial.print(omega_left);
      Serial.print("R-omega: ");
      Serial.println(omega_right);
    }
  }
};

#endif
