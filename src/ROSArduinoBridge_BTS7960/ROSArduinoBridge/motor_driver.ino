/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined BTS7960_MOTOR_DRIVER
  /* BTS7960 uses two PWM inputs per side:
       - Forward: RPWM = speed, LPWM = 0
       - Reverse: RPWM = 0,     LPWM = speed
       - Stop:    RPWM = 0,     LPWM = 0
     Each side's driver feeds both motors on that side (wired in parallel). */

  #ifndef RIGHT_MOTOR_SCALE
    #define RIGHT_MOTOR_SCALE 1.0  // Tune this (e.g. 1.05 - 1.20) if right side is physically weaker
  #endif

  void initMotorController() {
    pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
    pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
    pinMode(LEFT_MOTOR_RPWM, OUTPUT);
    pinMode(LEFT_MOTOR_LPWM, OUTPUT);

    #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
      // Synchronize Timer1 PWM frequency (Pins 9 & 10, Right side) to 976 Hz
      // to match Timer0 PWM frequency (Pins 5 & 6, Left side) for equal power delivery.
      TCCR1B = (TCCR1B & 0b11111000) | 0x02;
    #endif

    #ifdef RIGHT_MOTOR_R_EN
      pinMode(RIGHT_MOTOR_R_EN, OUTPUT);
      pinMode(RIGHT_MOTOR_L_EN, OUTPUT);
      pinMode(LEFT_MOTOR_R_EN, OUTPUT);
      pinMode(LEFT_MOTOR_L_EN, OUTPUT);
      digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
      digitalWrite(RIGHT_MOTOR_L_EN, HIGH);
      digitalWrite(LEFT_MOTOR_R_EN, HIGH);
      digitalWrite(LEFT_MOTOR_L_EN, HIGH);
    #endif

    // Start with motors stopped
    analogWrite(RIGHT_MOTOR_RPWM, 0);
    analogWrite(RIGHT_MOTOR_LPWM, 0);
    analogWrite(LEFT_MOTOR_RPWM, 0);
    analogWrite(LEFT_MOTOR_LPWM, 0);
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (i == RIGHT) {
      spd = (int)round(spd * RIGHT_MOTOR_SCALE);
    }

    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;

    if (i == LEFT) {
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_RPWM, spd); analogWrite(LEFT_MOTOR_LPWM, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_LPWM, spd); analogWrite(LEFT_MOTOR_RPWM, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_RPWM, spd); analogWrite(RIGHT_MOTOR_LPWM, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_LPWM, spd); analogWrite(RIGHT_MOTOR_RPWM, 0); }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
