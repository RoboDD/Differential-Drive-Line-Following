#include "encoders.h"
#include "kinematics.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"

Motors_c motors;
LineSensors_c linesensors;

unsigned long endtime = 0;


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

  float dir;
  float pwm;
  float k_p = 50;
  int conor;
  bool online;

  dir = linesensors.get_line_follow_error();
  conor = linesensors.check_on_cornor();
  online = linesensors.check_on_line();

  switch(conor){
    case 0:
      pwm = k_p * dir; //simple P-controller
      motors.set_chasis_power(30 + pwm, 30 - pwm);
      delay(10);
      break;
    case 1:// turn left
      // motors.set_chasis_power(15, 15);
      delay(50);
      motors.set_chasis_power(-30, 30);
      delay(100);
      break;
    case 2:// turn right
      // motors.set_chasis_power(15, 15);
      delay(50);
      motors.set_chasis_power(30, -30);
      delay(100);
      break;
    default:
      break;
  }

  // if (conor == 0){
  
  //   pwm = k_p * dir; //simple P-controller

  //   motors.set_chasis_power(30 + pwm, 30 - pwm);
  //   delay(10);
 
  // }


  // // on the left conor
  // if (conor == 1){
  //     motors.set_chasis_power(30, -30);
  //     delay(100);
  // }
  // // on the right conor
  // if (conor == 2){
  //     motors.set_chasis_power(-30, 30);
  //     delay(100);
  // }

  

}
