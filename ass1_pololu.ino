#include "encoders.h"
#include "kinematics.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"

Motors_c motors;
LineSensors_c linesensors;

//!  Runs once.
/**
 * @brief
 * todo 123
 * ! fwef
 * *erwe
 * todo
 * TODO 123
 *
 */
void setup() {
  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  motors.init();
  linesensors.init(true);
}

void loop() {
  int data[3];
  float dir;
  float pwm;

  // linesensors.read_data();

  dir = linesensors.get_norm_error();

  pwm = 44 * dir;

  motors.set_chasis_power(30 + pwm, 30 - pwm);

  delay(10);

  // motors.set_chasis_power(30,30);
  // delay(1000);
  // motors.set_chasis_power(-30, -30);
  // delay(1000);
  // motors.stop_motors();
  // delay(2000);
}
