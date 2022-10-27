#ifndef _ENCODERS_H
#define _ENCODERS_H

#define ENCODER_0_A_PIN 7
#define ENCODER_0_B_PIN 23
#define ENCODER_1_A_PIN 26
// #define ENCODER_1_B_PIN Non-standard pin!

#define DEBUG_MODE false
// Volatile Global variables used by Encoder ISR.
volatile long count_right;  // used by encoder to count the rotation
volatile byte state_right;
volatile long count_left;
volatile byte state_left;

// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR(INT6_vect) {
  // We know that the ISR is only called when a pin changes.
  // We also know only 1 pin can change at a time.
  // The XOR(AB) signal change from "Channel A" triggers ISR.

  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  boolean right_B = digitalRead(ENCODER_0_B_PIN);  // normal B state
  boolean right_A = digitalRead(ENCODER_0_A_PIN);  // XOR(AB)

  // Software XOR (^) logically infers
  // the true value of A given the state of B
  right_A = right_A ^ right_B;

  // Shift our (new) current readings into bit positions
  // 2 and 3 in the state variable (current state)
  // State: (bit3)  (bit2)  (bit1)   (bit0)
  // State:  new B   new A   old B   old A
  state_right = state_right | (right_B << 3);
  state_right = state_right | (right_A << 2);

  // Handle which transition we have registered.
  // Complete this if statement as necessary.
  // Refer to the labsheet.
  if (state_right == 0) {
  } else if (state_right == 1) {
    count_right = count_right -1;   // backwards
    
  } else if (state_right == 2) {
    count_right = count_right + 1;  // forwards
    

  }  // Continue this if statement as necessary.

  // Shift the current readings (bits 3 and 2) down
  // into position 1 and 0 (to become prior readings)
  // This bumps bits 1 and 0 off to the right, "deleting"
  // them for the next ISR call.
  state_right = state_right >> 2;
  if (DEBUG_MODE == true){
    Serial.print("state_right: ");
    Serial.print(state_right);
    Serial.print("count_right: ");
    Serial.println(count_right);
  }

  
}

// This ISR handles just Encoder 1
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR(PCINT0_vect) {
  // First, Read in the new state of the encoder pins.

  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  // It is more portable to use the PINE2 keyword.
  boolean left_B = PINE & (1 << PINE2);
  // boolean e1_B = PINE & B00000100;  // Does same as above.

  // Standard read from the other pin.
  boolean left_A = digitalRead(ENCODER_1_A_PIN);  // 26 the same as A8

  left_A = left_A ^ left_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:
  //
  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  state_left = state_left | (left_B << 3);
  state_left = state_left | (left_A << 2);

  // Handle which transition we have registered.
  // Complete this if statement as necessary.
  // Refer to the labsheet.
  if (state_left == 0) {
  } else if (state_left == 1) {
    count_left = count_left -1;   // backwards
  } else if (state_left == 2) {
    count_left = count_left +1;   // forewards
  }  // Continue this if statement as necessary.

  // Shift the current readings (bits 3 and 2) down
  // into position 1 and 0 (to become prior readings)
  // This bumps bits 1 and 0 off to the right, "deleting"
  // them for the next ISR call.
  state_left = state_left >> 2;

  if (DEBUG_MODE == true){
    Serial.print("state_left: ");
    Serial.print(state_left);
    Serial.print("count_left: ");
    Serial.println(count_left);
  }
}

/*
   This setup routine enables interrupts for
   encoder1.  The interrupt is automatically
   triggered when one of the encoder pin changes.
   This is really convenient!  It means we don't
   have to check the encoder manually.
*/
void setupEncoder0() {
  count_right = 0;

  // Setup pins for right encoder
  pinMode(ENCODER_0_A_PIN, INPUT);
  pinMode(ENCODER_0_B_PIN, INPUT);

  // initialise the recorded state of e0 encoder.
  state_right = 0;

  // Get initial state of encoder pins A + B
  boolean right_A = digitalRead(ENCODER_0_A_PIN);
  boolean right_B = digitalRead(ENCODER_0_B_PIN);
  right_A = right_A ^ right_B;

  // Shift values into correct place in state.
  // Bits 1 and 0  are prior states.
  state_right = state_right | (right_B << 1);
  state_right = state_right | (right_A << 0);

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1 << INT6);
  // EIMSK = EIMSK & B1011111; // Same as above.

  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= (1 << ISC60);  // using header file names, push 1 to bit ISC60
  // EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= (1 << INTF6);
  // EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= (1 << INT6);
  // EIMSK |= B01000000; // Same as above
}

void setupEncoder1() {
  count_left = 0;

  // Setting up left encoder:
  // The Romi board uses the pin PE2 (port E, pin 2) which is
  // very unconventional.  It doesn't have a standard
  // arduino alias (like d6, or a5, for example).
  // We set it up here with direct register access
  // Writing a 0 to a DDR sets as input
  // DDRE = Data Direction Register (Port)E
  // We want pin PE2, which means bit 2 (counting from 0)
  // PE Register bits [ 7  6  5  4  3  2  1  0 ]
  // Binary mask      [ 1  1  1  1  1  0  1  1 ]
  //
  // By performing an & here, the 0 sets low, all 1's preserve
  // any previous state.
  DDRE = DDRE & ~(1 << DDE6);
  // DDRE = DDRE & B11111011; // Same as above.

  // We need to enable the pull up resistor for the pin
  // To do this, once a pin is set to input (as above)
  // You write a 1 to the bit in the output register
  PORTE = PORTE | (1 << PORTE2);
  // PORTE = PORTE | 0B00000100;

  // Encoder0 uses conventional pin 26
  pinMode(ENCODER_1_A_PIN, INPUT);
  digitalWrite(ENCODER_1_A_PIN, HIGH);  // Encoder 1 xor

  // initialise the recorded state of e1 encoder.
  state_left = 0;

  // Get initial state of encoder.
  boolean left_B = PINE & (1 << PINE2);
  // boolean e1_B = PINE & B00000100;  // Does same as above.

  // Standard read from the other pin.
  boolean left_A = digitalRead(ENCODER_1_A_PIN);  // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  left_A = left_A ^ left_B;

  // Shift values into correct place in state.
  // Bits 1 and 0  are prior states.
  state_left = state_left | (left_B << 1);
  state_left = state_left | (left_A << 0);

  // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
  // pin-change interrupts.
  // Note, this register will normally create an interrupt a change to any pins
  // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
  // When we set these registers, the compiler will now look for a routine
  // called ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0
  // seems like a mismatch to PCINT4, however there is only the one vector
  // servicing a change to all PCINT0->7 pins. See Manual 11.1.5 Pin Change
  // Interrupt Control Register - PCICR

  // Page 91, 11.1.5, Pin Change Interrupt Control Register
  // Disable interrupt first
  PCICR = PCICR & ~(1 << PCIE0);
  // PCICR &= B11111110;  // Same as above

  // 11.1.7 Pin Change Mask Register 0 – PCMSK0
  PCMSK0 |= (1 << PCINT4);

  // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
  PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

  // Enable
  PCICR |= (1 << PCIE0);
}

#endif
