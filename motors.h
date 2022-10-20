// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// set motor pin
#define L_PWM_PIN 9
#define L_DIR_PIN 16
#define R_PWM_PIN 10
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

// Class to operate the motor(s).
class Motors_c {
 public:
  // int MOTOR_PIN
  // int DIR_PIN
  // static const int L_PWM_PIN = 9  ;
  // static const int L_DIR_PIN = 16 ;
  // static const int R_PWM_PIN = 10 ;
  // static const int R_DIR_PIN = 15 ;

  // static const int FWD =  0 ;
  // static const int REV =  1 ;

  // Constructor, must exist.
  Motors_c() {}

  // Use this function to
  // initialise the pins and
  // state of your motor(s).
  void init() { set_chasis_power(0, 0); }

  // set chasis power for differential drive
  void set_chasis_power(float left_pwr_des, float right_pwr_des) {
    // left_pwr_des = left_pwr_des;
    // right_pwr_des = right_pwr_des;

    set_motor_pwm(L_PWM_PIN, L_DIR_PIN, left_pwr_des);
    set_motor_pwm(R_PWM_PIN, R_DIR_PIN, right_pwr_des);
  }

  // stop all motor
  void stop_motors() {
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
  }

 private:
  // set pwm of a single motor
  // -255 < motor_pwr < 255
  //  REV ------------- FWD
  void set_motor_pwm(int motor_pin, int dir_pin, float motor_pwr) {
    // set direction
    if (motor_pwr < 0) {
      digitalWrite(dir_pin, REV);
    }
    if (motor_pwr >= 0) {
      digitalWrite(dir_pin, FWD);
    }

    // set pwr
    if (motor_pwr < 0) {
      motor_pwr = -motor_pwr;
      analogWrite(motor_pin, motor_pwr);
    }
    if (motor_pwr >= 0) {
      analogWrite(motor_pin, motor_pwr);
    }
  }
};

#endif
