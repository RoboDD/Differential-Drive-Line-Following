#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

// Replace the ? with correct pin numbers
// https://www.pololu.com/docs/0J83/5.9

//
Motors_c motors;
LineSensors_c linesensors;


// Runs once.
void setup() {

  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  motors.init();
  linesensors.init(true);

}

// Repeats.
void loop() {

  linesensors.read_data();
  delay(200);

  // motors.set_chasis_power(30,30);
  // delay(1000);
  // motors.set_chasis_power(-30, -30);
  // delay(1000);
  // motors.stop_motors();
  // delay(2000);


 
}

