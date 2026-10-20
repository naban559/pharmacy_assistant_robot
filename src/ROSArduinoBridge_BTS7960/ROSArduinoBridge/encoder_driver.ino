/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  // Four independent encoder counters - one per motor.
  volatile long rightFrontEncoder = 0L;
  volatile long rightRearEncoder  = 0L;
  volatile long leftFrontEncoder  = 0L;
  volatile long leftRearEncoder   = 0L;

  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

  // If the front/rear encoders on a side have different resolutions
  // (counts per revolution), set these so the two are normalized before
  // being averaged together. Left at 1.0 (equal resolution) by default.
  #define RIGHT_FRONT_CPR 1.0
  #define RIGHT_REAR_CPR  1.0
  #define LEFT_FRONT_CPR  1.0
  #define LEFT_REAR_CPR   1.0

  /* Interrupt routine for the RIGHT side: decodes BOTH the right-front
     (D2/D3) and right-rear (D4/D7) encoders, since all four pins live
     in PORTD and share this one pin-change vector. Each encoder keeps
     its own 2-bit history so the two quadrature decodes don't interfere
     with each other. */
  ISR (PCINT2_vect) {
    static uint8_t enc_last_rf = 0;
    static uint8_t enc_last_rr = 0;
    uint8_t pind = PIND;

    uint8_t rf_state = ((pind >> 2) & 0x01) | (((pind >> 3) & 0x01) << 1); // D2=A, D3=B
    enc_last_rf = ((enc_last_rf << 2) | rf_state) & 0x0F;
    rightFrontEncoder += ENC_STATES[enc_last_rf];

    uint8_t rr_state = ((pind >> 4) & 0x01) | (((pind >> 7) & 0x01) << 1); // D4=A, D7=B
    enc_last_rr = ((enc_last_rr << 2) | rr_state) & 0x0F;
    rightRearEncoder += ENC_STATES[enc_last_rr];
  }

  /* Interrupt routine for LEFT-REAR encoder (A5/A4 on PORTC) */
  ISR (PCINT1_vect) {
    static uint8_t enc_last_lr = 0;
    uint8_t pinc = PINC;

    uint8_t lr_state = ((pinc >> 5) & 0x01) | (((pinc >> 4) & 0x01) << 1); // A5=A, A4=B
    enc_last_lr = ((enc_last_lr << 2) | lr_state) & 0x0F;
    leftRearEncoder += ENC_STATES[enc_last_lr];
  }

  /* Polling function for LEFT-FRONT encoder on ADC-only pins A7 (Phase A) & A6 (Phase B) */
  void updateLeftFrontEncoder() {
    static uint8_t enc_last_lf = 0;
    uint8_t a = (analogRead(A7) > 500) ? 1 : 0;
    uint8_t b = (analogRead(A6) > 500) ? 1 : 0;
    uint8_t lf_state = a | (b << 1);
    if ((enc_last_lf & 0x03) != lf_state) {
      enc_last_lf = ((enc_last_lf << 2) | lf_state) & 0x0F;
      leftFrontEncoder += ENC_STATES[enc_last_lf];
    }
  }

  /* Raw per-motor readouts - Forward physical motion returns positive counts for all 4 wheels */
  long readRightFrontEncoder() { return rightFrontEncoder; }
  long readRightRearEncoder()  { return rightRearEncoder; }
  long readLeftFrontEncoder()  { updateLeftFrontEncoder(); return -leftFrontEncoder; }
  long readLeftRearEncoder()   { return -leftRearEncoder; }

  /* Combine front+rear on each side into one side measurement.
     Normalizes each encoder's count by its counts-per-revolution before
     averaging, so this stays correct even if the two resolutions differ
     (with equal resolutions, the CPR terms cancel out to a plain average). */
  long readRightSideEncoder() {
    double rf = readRightFrontEncoder() / RIGHT_FRONT_CPR;
    double rr = readRightRearEncoder()  / RIGHT_REAR_CPR;
    return (long)round((rf + rr) / 2.0);
  }

  long readLeftSideEncoder() {
    double lf = readLeftFrontEncoder() / LEFT_FRONT_CPR;
    double lr = readLeftRearEncoder()  / LEFT_REAR_CPR;
    return (long)round((lf + lr) / 2.0);
  }

  /* Wrap the encoder reading function - unchanged interface, now backed
     by the combined per-side measurement instead of a single encoder. */
  long readEncoder(int i) {
    if (i == LEFT) return readLeftSideEncoder();
    else return readRightSideEncoder();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      leftFrontEncoder = 0L;
      leftRearEncoder = 0L;
      return;
    } else { 
      rightFrontEncoder = 0L;
      rightRearEncoder = 0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
