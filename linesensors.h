// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

// define pin of left, middle and right line sensors

#define LR_LINESENSOR_PIN 12 // left-rear
#define L_LINESENSOR_PIN A0
#define M_LINESENSOR_PIN A2
#define R_LINESENSOR_PIN A3
#define RR_LINESENSOR_PIN A4 // right-rear

#define EMIT_PIN 11
#define NUM_OF_LINESENSOR 5
#define BLACK_THRESHOLD 1400

#define ON_LINE true
#define OFF_LINE false

// #define NO_CORNOR 0
// #define L_CORNOR 1
// #define R_CORNOR 2

#define DEBUG_MODE true

// Class to operate the linesensor(s).
class LineSensors_c {
public:
  int line_sensors_pins[NUM_OF_LINESENSOR] = { L_LINESENSOR_PIN, M_LINESENSOR_PIN,
                               R_LINESENSOR_PIN, LR_LINESENSOR_PIN,
                               RR_LINESENSOR_PIN };
  int line_sensors_data[NUM_OF_LINESENSOR];

  float line_sensors_data_norm[NUM_OF_LINESENSOR];
  int which;

  unsigned long start_time;
  unsigned long elapsed_time;
  unsigned long dt;
  unsigned long timeout = 5000;

  // Constructor, must exist.
  LineSensors_c() {}

  void init(bool use_emit) {
    pinMode(L_LINESENSOR_PIN, INPUT);
    pinMode(M_LINESENSOR_PIN, INPUT);
    pinMode(R_LINESENSOR_PIN, INPUT);

    if (use_emit == true) {
      enable_emit();
    }
    if (use_emit == false) {
      disable_emit();
    }
  }

  // check in a sensor is ON_LINE or OFF_LINE
  bool check_on_line() {
    if (line_sensors_data[1] > BLACK_THRESHOLD) {
      if (DEBUG_MODE == true) {
        Serial.print("ON THE LINE!");
      }
      return ON_LINE;
    }
    if (line_sensors_data[1] < BLACK_THRESHOLD) {
      if (DEBUG_MODE == true) {
        Serial.print("OFF THE LINE!");
      }
      return OFF_LINE;
    }
  }


  // check 90 degree cornor using line sensor on most left and most right
  int check_on_cornor() {
    if (line_sensors_data[3] > BLACK_THRESHOLD) {
      if (DEBUG_MODE == true) {
        Serial.print("LEFT 90 CORNOR!");
      }
      // int result = 1;
      return 1;
    }

    if (line_sensors_data[4] > BLACK_THRESHOLD) {
      if (DEBUG_MODE == true) {
        Serial.print("RIGHT 90 CORNOR!");
      }
      // int result = 2;
      return 2;
    }

    if (line_sensors_data[3] < BLACK_THRESHOLD && line_sensors_data[4] < BLACK_THRESHOLD) {
      if (DEBUG_MODE == true) {
        Serial.print("NO CORNOR!");        
      }
      // int result = 0;                         
      return 0;
      
    }
    

    

  }



  float get_line_follow_error() {
    // read_data();
    read_data_parallel();

    float w_left;
    float w_right;
    float e_line;
    float sum;
    float avg;

    sum = line_sensors_data[0] + line_sensors_data[1] + line_sensors_data[2];
    avg = sum / 3;

    line_sensors_data_norm[0] = line_sensors_data[0] / sum;
    line_sensors_data_norm[1] = line_sensors_data[1] / sum;
    line_sensors_data_norm[2] = line_sensors_data[2] / sum;

    w_left = line_sensors_data_norm[0] + (line_sensors_data_norm[1] * 0.5);
    w_right = line_sensors_data_norm[2] + (line_sensors_data_norm[1] * 0.5);

    e_line = w_left - w_right;

    Serial.print("Normalised error signal=");
    Serial.print(e_line);
    Serial.print("!");

    return e_line;
  }

  void read_data_parallel() {
    int remaining = NUM_OF_LINESENSOR;

    charge_capcitors();

    // Start timing
    start_time = micros();

    while (remaining > 0) {
      for (which = 0; which < NUM_OF_LINESENSOR; which++) {
        elapsed_time = micros() - start_time;
      
        // wait the capacitor to decay
        if (digitalRead(line_sensors_pins[which]) == LOW) {
          if (line_sensors_data[which] == 0) {
            line_sensors_data[which] = elapsed_time;
            remaining = remaining - 1;
          }
        }
      }

      // Here, we make a final check to see if the
      // whole read operation is taking too long.
      if (elapsed_time >= timeout) {
        // Here, you may need to set an appropriate
        // sensor_read[ which ] value to indicate a
        // timeout.  An appropriate initial value of
        // sensor_read[] could mean this step isn't
        // necessary.

        // Set remaining to 0 to force end of while()
        remaining = 0;
      }
    }

    if (DEBUG_MODE == true) {
      Serial.print("LS_Left=");
      Serial.print(line_sensors_data[0]);
      Serial.print(",");

      Serial.print("LS_Middle=");
      Serial.print(line_sensors_data[1]);
      Serial.print(",");

      Serial.print("LS_Right=");
      Serial.print(line_sensors_data[2]);

      Serial.print("LS_Left_rear=");
      Serial.print(line_sensors_data[3]);

      Serial.print("LS_Right_rear=");
      Serial.println(line_sensors_data[4]);
    }
  }

  void enable_emit() {
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);
  }

  void disable_emit() {
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, LOW);
  }

private:
  void charge_capcitors() {
    // Charge each capcitor
    for (int which = 0; which < NUM_OF_LINESENSOR; which++) {
      // Charge capacitor by setting input pin
      // temporarily to output and HIGH
      pinMode(line_sensors_pins[which], OUTPUT);
      digitalWrite(line_sensors_pins[which], HIGH);

      // Tiny delay for capacitor to charge.
      delayMicroseconds(10);

      //  Turn input pin back to an input
      pinMode(line_sensors_pins[which], INPUT);

      line_sensors_data[which] = 0;
    }
  }
};

#endif