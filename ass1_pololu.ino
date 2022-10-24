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

float error_line;
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

  // init motors
  motors.init();

  //init line sensors
  linesensors.init(true);

  // init encoders
  setupEncoder0();
  setupEncoder1();

  // init task
  current_task = TASK_INIT;
}

void loop() {

  // kinematics.update();

  online = linesensors.check_on_line();

  do_line_follow();

  // stupid logic to stop car
  if (count_left > 1750){ // 20cm,168
    if (online == false) {
      motors.stop_motors();
      delay(10000);
    }
  }
  
  // Serial.println(current_task);

  

}


//Line Following
void do_line_follow(){
  
  error_line = linesensors.get_line_follow_error();
  conor = linesensors.check_on_cornor();
  
  switch(conor){
    case 0:// straight
      pwm = k_p * error_line; //simple P-controller
      motors.set_chasis_power(30 + pwm, 30 - pwm);
      delay(10);
      break;
    case 1:// 90 degree, turn left
      delay(50);
      motors.set_chasis_power(-30, 30);
      delay(100);
      break;
    case 2:// 90 degree, turn right
      delay(50);
      motors.set_chasis_power(30, -30);
      delay(100);
      break;
    case 3:// at T-shape, ture right first
      delay(50);
      motors.set_chasis_power(30, -30);
      delay(100);
      break;
    default:
      break;
  }
}
