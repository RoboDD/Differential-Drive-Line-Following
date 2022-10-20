#include "encoders.h"
#include "kinematics.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"

Motors_c motors;
LineSensors_c linesensors;

void setup() {
  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  motors.init();
  linesensors.init(true);
  setupEncoder0();
  setupEncoder1();
}

void loop() {
  int data[3];
  float dir;
  float pwm;

  dir = linesensors.get_norm_error();

  pwm = 50 * dir;


  motors.set_chasis_power(30 + pwm, 30 - pwm);

  // delay(100);
  // delay(1000);
  linesensors.check_on_line();

}
