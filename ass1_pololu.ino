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
#define RED_LED_PIN 17
#define YELLOW_LED_PIN 13

#define TYPE_HEADING_CONTROL 0 // 0 for pid, 1 for lqr

Motors_c motors;
LineSensors_c linesensors;
Kinematics_c kinematics;

unsigned long endtime = 0;

int OFFLINE_COUNT = 0;

float error_line;
float speed_sp;
float k_p = 50; // heading kp
float speed_kp = 0.85;//speed kp

int conor;
bool online;
double speed_left;
double speed_right;
double v_forward = 50;
double spd_error_left;
double spd_error_right;
double pwm_sp_left;
double pwm_sp_right;

// Task scheduler parameter
unsigned long prev_time_line_sensor = millis();
unsigned long prev_time_encoder = millis();
unsigned long prev_time_heading_control = millis();
unsigned long prev_time_motor_speed_control = millis();

unsigned long interval_line_sensor = 10; // 50 Hz for read linesensor data
unsigned long interval_encoder = 10; // 50 Hz for read linesensor data
unsigned long interval_heading_control = 50; // 20 Hz for heading controller
unsigned long interval_motor_speed_control = 10; // 100 Hz for motor speed controller


void setup() {
  // Start serial, send debug text.
  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  // Buzzer
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);

  // LED
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  // init motors
  motors.init();

  // init line sensors
  linesensors.init(true);

  // init encoders
  setupEncoder0();
  setupEncoder1();


  beep();
}

void loop() {

  unsigned long current_time = millis();

  // Task 1: Read line sensor data
  if (current_time - prev_time_line_sensor > interval_line_sensor){

    // start task 1
    error_line = linesensors.get_line_follow_error();
    conor = linesensors.check_on_cornor();
    // end task 1

    prev_time_line_sensor = current_time;
  }

  // Task 2: Read encoder data
  if (current_time - prev_time_encoder > interval_encoder){

    speed_left = get_wheel_speed_left();
    speed_right = get_wheel_speed_right();

    prev_time_encoder = current_time;
  }

  // Task 3: heading controller
  if (current_time - prev_time_heading_control > interval_heading_control){

    switch(TYPE_HEADING_CONTROL){
      case 0:// use pid controller
        speed_sp = k_p * error_line;  // simple P-controller
        // speed_sp=0;
        break;
      case 1://use lqr controller

        break;
    }



    prev_time_heading_control = current_time;
  }

  // Task 4: set motor pwm
  if (current_time - prev_time_motor_speed_control > interval_motor_speed_control){

    // start task 4
    // pwm = k_p * error_line;  // simple P-controller
    // motors.set_chasis_power(30, 30);
    // motors.set_chasis_power(30 + pwm, 30 - pwm);
    // do_line_follow();
    spd_error_left =  v_forward - speed_left + speed_sp;
    spd_error_right =  v_forward - speed_right - speed_sp;

    pwm_sp_left = speed_kp * spd_error_left;
    pwm_sp_right = speed_kp * spd_error_right;
    // Serial.print("speed left: ");
    // Serial.println(pwm_sp_left);
    // Serial.print("speed right: ");
    // Serial.println(pwm_sp_right);
    motors.set_chasis_power(pwm_sp_left, pwm_sp_right);

    // last_spd_error

    // end task 4

    prev_time_motor_speed_control = current_time;
  }

}

// Line Following
void do_line_follow() {
  // error_line = linesensors.get_line_follow_error();
  // conor = linesensors.check_on_cornor();

  switch (conor) {
    case 0:                    // straight
      speed_sp = k_p * error_line;  // simple P-controller
      motors.set_chasis_power(30 + speed_sp, 30 - speed_sp);
      delay(10);
      break;
    case 1:  // 90 degree, turn left
      delay(50);
      motors.set_chasis_power(-30, 30);
      delay(100);
      break;
    case 2:  // 90 degree, turn right
      delay(50);
      motors.set_chasis_power(30, -30);
      delay(100);
      break;
    case 3:  // at T-shape, ture right first
      delay(50);
      motors.set_chasis_power(30, -30);
      delay(100);
      break;
    default:
      break;
  }
}



void do_stop_and_wait() {
  motors.stop_motors();
  beep();
  delay(20000);
}


void beep() {
  for (int i = 0; i < 100; i++) {
    digitalWrite(BUZZ_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(BUZZ_PIN, LOW);
    delayMicroseconds(500);
  }
}

double get_wheel_speed_left(){

    double last_count_left = count_left;
    // double last_count_right = count_right;
    
    unsigned long start_time = millis();

    delay(20);

    unsigned long end_time = millis();    
    unsigned long delta_t = end_time - start_time;

    // double count_diff_right =  count_right - last_count_right;
    // double speed_right = 100*count_diff_right / delta_t;

    double count_diff_left =  count_left - last_count_left ;
    double speed_left = 100*count_diff_left / delta_t;
    // Serial.print("speed left: ");
    // Serial.println(speed_left);   

    return speed_left;    

}
double get_wheel_speed_right(){

    // double last_count_left = count_left;
    double last_count_right = count_right;
    
    unsigned long start_time = millis();

    delay(20);

    unsigned long end_time = millis();    
    unsigned long delta_t = end_time - start_time;

    double count_diff_right =  count_right - last_count_right;
    double speed_right = 100*count_diff_right / delta_t;
 
    // Serial.print("speed right: ");
    // Serial.println(speed_right);
    return speed_right;

}




