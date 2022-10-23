#include "encoders.h"
#include "kinematics.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"

#define TASK_INIT 0
#define TASK_JOIN_LINE 1
#define TASK_FOLLOW_LINE 2
#define TASK_GAP 3
#define TASK_RETURN_HOME 4

Motors_c motors;
LineSensors_c linesensors;
Kinematics_c kinematics;

unsigned long endtime = 0;

int OFFLINE_COUNT = 0;

float dir;
float pwm;
float k_p = 50;
int conor;
bool online;
int current_task;

void setup() {
  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  motors.init();
  linesensors.init(true);
  setupEncoder0();
  setupEncoder1();

  current_task = TASK_INIT;
}

void loop() {

  // kinematics.update();


  do_line_follow();

  

}

void do_line_follow(){
    //Line Following
  dir = linesensors.get_line_follow_error();
  conor = linesensors.check_on_cornor();
  online = linesensors.check_on_line();

  switch(conor){
    case 0:// straight
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
}
