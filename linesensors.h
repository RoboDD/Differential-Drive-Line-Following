// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

// define pin of left, middle and right line sensors
# define L_LINESENSOR_PIN A0 
# define M_LINESENSOR_PIN A2
# define R_LINESENSOR_PIN A3

# define EMIT_PIN 11
# define NUM_OF_LINESENSOR 3


// Class to operate the linesensor(s).
class LineSensors_c {
  public:

    int line_sensors_pins[3] = { L_LINESENSOR_PIN , M_LINESENSOR_PIN , R_LINESENSOR_PIN};
    int line_sensors_data[3];
    int which;

    unsigned long start_time;
    unsigned long elapsed_time;
    unsigned long dt;
    unsigned long timeout = 5000;

    // Constructor, must exist.
    LineSensor_c() {

    } 

    void init(bool use_emit){

      pinMode( L_LINESENSOR_PIN , INPUT );
      pinMode( M_LINESENSOR_PIN , INPUT );
      pinMode( R_LINESENSOR_PIN , INPUT );

      if (use_emit == true) {

        enable_emit();

      }
      if (use_emit == false) {

        disable_emit();

      }

    }

    void read_data(){


      

      // digitalRead( L_LINESENSOR_PIN );
      // digitalRead( M_LINESENSOR_PIN );
      // digitalRead( R_LINESENSOR_PIN );

      // while( digitalRead( L_LINESENSOR_PIN ) == HIGH ) {
      // // Do nothing, waiting for LS_LEFT_PINT to
      // // go to LOW state.
      // }

      int remaining = NUM_OF_LINESENSOR;

      while ( remaining > 0 ) {
        
        for( which = 0 ; which < NUM_OF_LINESENSOR; which++ ) {
          dt = 0;
          start_time = micros();

          // Charge capacitor by setting input pin
          // temporarily to output and HIGH
          pinMode( line_sensors_pins[ which ] , OUTPUT );
          digitalWrite( line_sensors_pins[ which ] , HIGH );

          // Tiny delay for capacitor to charge.
          delayMicroseconds(10);

          //  Turn input pin back to an input
          pinMode( line_sensors_pins[ which ] , INPUT );

          while( digitalRead( line_sensors_pins[ which ] ) == HIGH ) {

          }
          
          elapsed_time = micros();
          dt = elapsed_time - start_time;
          line_sensors_data[ which ] = dt;

          remaining = remaining - 1;

        }

        // Here, we make a final check to see if the 
        // whole read operation is taking too long.
        if( elapsed_time >= timeout ) {

            // Here, you may need to set an appropriate
            // sensor_read[ which ] value to indicate a
            // timeout.  An appropriate initial value of
            // sensor_read[] could mean this step isn't 
            // necessary.

            // Set remaining to 0 to force end of while()
            remaining = 0;
        }

      }
      

      // unsigned long value1 = line_sensors_data[0];
      // unsigned long value2 = line_sensors_data[1];
      // unsigned long value3 = line_sensors_data[2];


      Serial.print("LS_Left=");
      Serial.print( line_sensors_data[0] );
      Serial.print(",");

      Serial.print("LS_Middle=");
      Serial.print( line_sensors_data[1] );
      Serial.print(",");

      Serial.print("LS_Right=");
      Serial.println( line_sensors_data[2] );
      // Serial.println("\n");


    }

    void enable_emit(){

      pinMode( EMIT_PIN , OUTPUT );
      digitalWrite( EMIT_PIN , HIGH );

    }

    void disable_emit(){

      pinMode( EMIT_PIN , OUTPUT );
      digitalWrite( EMIT_PIN , LOW );
      
    }

    

};



#endif
