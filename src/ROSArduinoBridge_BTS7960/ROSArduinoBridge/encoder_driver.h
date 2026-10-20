/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  // ---- RIGHT SIDE (right-front + right-rear motors), on PORTD ----
  // These must stay PORTD pins (D0-D7) - handled by the PCINT2 ISR.
  #define RIGHT_FRONT_ENC_PIN_A PD2  //pin D2
  #define RIGHT_FRONT_ENC_PIN_B PD3  //pin D3
  #define RIGHT_REAR_ENC_PIN_A  PD4  //pin D4
  #define RIGHT_REAR_ENC_PIN_B  PD7  //pin D7

  // ---- LEFT SIDE (left-front + left-rear motors) ----
  // Left-rear is on PORTC pins A5/A4 (handled by PCINT1 ISR).
  // Left-front is connected to A7 (Phase A) and A6 (Phase B).
  // On ATmega328P (Arduino Nano), A7 and A6 are ADC-only pins sampled via analogRead polling.
  #define LEFT_FRONT_ENC_PIN_A  A7  // pin A7 (Phase A)
  #define LEFT_FRONT_ENC_PIN_B  A6  // pin A6 (Phase B)
  #define LEFT_REAR_ENC_PIN_A   PC5 // pin A5
  #define LEFT_REAR_ENC_PIN_B   PC4 // pin A4
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void updateLeftFrontEncoder();

// Individual per-motor encoder readouts (raw counts)
long readRightFrontEncoder();
long readRightRearEncoder();
long readLeftFrontEncoder();
long readLeftRearEncoder();

// Combined per-side encoder readouts (what readEncoder(LEFT/RIGHT) returns)
long readRightSideEncoder();
long readLeftSideEncoder();

