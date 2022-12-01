#include "encoders.h"
#include "kinematics.h"
#include "linesensors.h"
#include "motors.h"
#include "pid.h"
#include "lqr.h"

#define TYPE_HEADING_CONTROL 0 // 0 for pid, 1 for lqr

#define BUZZ_PIN 6
#define RED_LED_PIN 17
#define YELLOW_LED_PIN 13


// Construct Instances
Motors_c motors;
LineSensors_c linesensors;
Kinematics_c kinematics;
PID_c heading_pid;
PID_c wheel_spd_left_pid;
PID_c wheel_spd_right_pid;
LQR_c heading_lqr;

// Parameters for FSM
#define TASK_INIT 0
#define TASK_FOLLOW_LINE 1
#define TASK_REACH_END 2
#define TASK_COMPLETED 3
#define TASK_SAVE_DATA 4


// Parameters for encoder
unsigned long endtime = 0; // for encoder
float theta = 0; // Kinematics

// Parameters for LPF
double alpha = 0.3; // lpf parameter for encoder velocity estimation
double alpha_eline = 0.7; //lpf parameter for eline
double lpf_l = 0;
double lpf_r = 0;
double lpf_eline = 0;
double lpf_theta = 0;

// Parameters for Heading Control
float error_line;
float speed_difference_sp;
float speed_difference_ac;
// int conor;
// bool online;
float record[20][10];
long record_index = 0;
int record_index_tem;

// Parameters for Wheel Speed Control
double speed_left;
double speed_right;
double v_forward = 40;
double pwm_sp_left;
double pwm_sp_right;


// Task scheduler parameter
unsigned long current_time;
unsigned long prev_time_line_sensor = millis();
unsigned long prev_time_encoder = millis();
unsigned long prev_time_heading_control = millis();
unsigned long prev_time_motor_speed_control = millis();
unsigned long prev_time_save_data = millis();

unsigned long interval_line_sensor = 10; // 100 Hz for read linesensor data 10
unsigned long interval_encoder = 10; // 100 Hz for read linesensor data
unsigned long interval_heading_control = 50; // 20 Hz for heading controller
unsigned long interval_motor_speed_control = 10; // 100 Hz for motor speed controller
unsigned long interval_save_data = 20; // 50 Hz for record data

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////





void setup() {
  // Start serial, send debug text.
  Serial.begin(115200);
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

  // init PID controllers
  
  switch(TYPE_HEADING_CONTROL){
  case 0:// use pid heading controller

    heading_pid.init(46.4, 0, 30, 50); //50ms=20Hz

    break;
  case 1://use lqr heading controller

    // heading_lqr.init(54, -17, -0.04, 50); //50ms=20Hz
    heading_lqr.init(60,  0,    -2.5, 50); //50ms=20Hz

    break;
}
  wheel_spd_left_pid.init(0.57, 0, 0.4, 10); //10ms=50Hz
  wheel_spd_right_pid.init(0.57, 0, 0.4, 10); //10ms=50Hz

  // beep(); // finish init
}


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void loop() {
  if (record_index < 100){
    current_time = millis();

    // Task 1: Read line sensor data
    if (current_time - prev_time_line_sensor > interval_line_sensor){

      // start task 1
      // Serial.println( "Normal" );
      error_line = linesensors.get_line_follow_error();
     
      // conor = linesensors.check_on_cornor();
      lpf_eline = ( lpf_eline * (1 - alpha_eline ) ) + ( error_line * alpha_eline ); //low pass filter
      // record_index_tem = record_index/10;
      // record[record_index_tem][record_index-(10*record_index_tem)] = lpf_eline;
      // record_index = record_index + 1;
      // end task 1

      prev_time_line_sensor = current_time;
    }
    // END Task 1

    // Task 2: Read encoder data
    if (current_time - prev_time_encoder > interval_encoder){

      // start task 2
      speed_left = get_wheel_speed_left();
      speed_right = get_wheel_speed_right();
      
      lpf_l = ( lpf_l * (1 - alpha ) ) + ( speed_left * alpha ); //low pass filter
      lpf_r = ( lpf_r * (1 - alpha ) ) + ( speed_right * alpha ); //low pass filter

      theta = kinematics.get_theta(lpf_l, lpf_r, 10);
      // lpf_theta = ( lpf_theta * (1 - alpha ) ) + ( theta * alpha );

      record_index_tem = record_index/10;
      record[record_index_tem][record_index-(10*record_index_tem)] = theta;
      record_index = record_index + 1;
      // end task 2

      prev_time_encoder = current_time;
    }
    // END Task 2

    // Task 3: heading controller: input: filtered eline, output: speed difference setpoint
    if (current_time - prev_time_heading_control > interval_heading_control){

      // start task 3
      switch(TYPE_HEADING_CONTROL){
        case 0:// use pid heading controller

          speed_difference_sp = heading_pid.calulate_PID(0,lpf_eline); // Full PID

          break;
        case 1://use lqr heading controller

          speed_difference_ac = heading_lqr.calulate_LQR(speed_difference_sp, lpf_eline);

          break;
      }
      // end task 3
      prev_time_heading_control = current_time;
    }
    // END Task 3

    // Task 4: set motor pwm, input: speed difference setpoint, output: pwm.
    if (current_time - prev_time_motor_speed_control > interval_motor_speed_control){
      speed_difference_sp = speed_difference_sp + speed_difference_ac * 0.3;
      // start task 4
      pwm_sp_left = wheel_spd_left_pid.calulate_PID(v_forward + speed_difference_sp , lpf_l);
      pwm_sp_right = wheel_spd_left_pid.calulate_PID(v_forward - speed_difference_sp , lpf_r);

      motors.set_chasis_power(pwm_sp_left, pwm_sp_right);
      // end task 4

      prev_time_motor_speed_control = current_time;
    }
    // END Task 4

    // Task 5: record data
    if (current_time - prev_time_save_data > interval_save_data){

      // start task 5

      //TODO

      // end task 5
      prev_time_save_data = current_time;
    }
    // END Task 5

  }
  else{
    motors.stop_motors();
    Serial.println( "stop???????");
    for (int i = 0; i<20; i++){
      for (int j = 0; j<10; j++){
        Serial.print( record[i][j]);
        Serial.print( ", " );
        delay(1);
      }
      Serial.print( "\n" ); // new row
    }

    
  }

  


}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void beep() {
  for (int i = 0; i < 100; i++) {
    digitalWrite(BUZZ_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(BUZZ_PIN, LOW);
    delayMicroseconds(500);
  }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

double get_wheel_speed_left(){

    double last_count_left = count_left;

    unsigned long start_time = millis();

    delay(20);

    unsigned long end_time = millis();    
    unsigned long delta_t = end_time - start_time;

    double count_diff_left =  count_left - last_count_left ;
    double speed_left = 100*count_diff_left / delta_t;

    // Serial.print("speed left: ");
    // Serial.println(speed_left);   

    return speed_left;    
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

double get_wheel_speed_right(){

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

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////


