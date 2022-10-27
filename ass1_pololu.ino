#include "encoders.h"
#include "kinematics.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"

#define TASK_INIT 0
// #define TASK_JOIN_LINE 1
#define TASK_FOLLOW_LINE 1
// #define TASK_GAP 2
#define TASK_REACH_END 2
#define TASK_RETURN_HOME 3
#define TASK_STOP_AT_HOME 4

#define BUZZ_PIN 6

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
  // init task
  current_task = TASK_INIT;

  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Buzzer
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);

  // init motors
  motors.init();

  //init line sensors
  linesensors.init(true);

  // init encoders
  setupEncoder0();
  setupEncoder1();

  // finish all init
  current_task += 1; // go to task 2 "folow line"
  beep();
  
}

void loop() {
  
  // beep();
  // delay(2000);
  online = linesensors.check_on_line();




  switch(current_task){
    case TASK_INIT:
      Serial.println("System is initialing!");
      break;
    case TASK_FOLLOW_LINE:
      do_line_follow();
      // stupid logic to stop car
      if (count_left > 1750){ // 20cm,168
        if (online == false) {
          current_task +=1;
        }
      }
      break;
    case TASK_REACH_END:
      do_stop_and_wait();
      current_task +=1;
      break;
    case TASK_RETURN_HOME:
      delay(50);
      motors.set_chasis_power(-30, 30);
      delay(1000);
      motors.set_chasis_power(30, 30);
      delay(10000);
      current_task +=1;
      break;
    case TASK_STOP_AT_HOME:
      do_stop_and_wait();
      break;
    default:
      break;
  }

  Serial.println(current_task);

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

void do_stop_and_wait(){
  motors.stop_motors();
  beep();
  delay(50000);
}

void beep(){
    for(int i=0;i<100;i++){
      digitalWrite(BUZZ_PIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(BUZZ_PIN, LOW);
      delayMicroseconds(500);
    }

}

