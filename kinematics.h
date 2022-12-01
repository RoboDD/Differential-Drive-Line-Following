// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#define DEBUG_MODE true

// Class to track robot position.
class Kinematics_c {
 public:
  float x_dot;
  float y_dot;
  float theta_dot;

  float last_world_x=0;
  float last_world_y=0;
  float last_theta=0;

  float R = 0.016; // Radius of wheel in meters
  float L = 0.96; // Distance of two wheels in meters

  // Constructor, must exist.
  Kinematics_c() {}


  // Use this function to update
  // your kinematics
  float get_theta(float left_whl_spd, float right_whl_spd, float interval) {

    x_dot = R / 2 * (left_whl_spd + right_whl_spd) * cos(theta_dot);
    y_dot = R / 2 * (left_whl_spd + right_whl_spd) * sin(theta_dot);
    // theta_dot = R / L * (left_whl_spd - right_whl_spd);
    theta_dot = R / L * (left_whl_spd - right_whl_spd);

    last_world_x += x_dot * interval;
    last_world_y += y_dot * interval;
    last_theta += theta_dot * interval * 2;    
    // last_theta += (atan2(last_world_y, last_world_x)* 4068) / 71; //

    if (DEBUG_MODE == true) {
      Serial.print("X: ");
      Serial.print(last_world_x);
      Serial.print("Y: ");
      Serial.println(last_world_y);
      Serial.print("Theta: ");
      Serial.println(last_theta);
    }

  return last_theta;
  }
};

#endif
