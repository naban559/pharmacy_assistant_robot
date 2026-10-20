/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

#ifdef BTS7960_MOTOR_DRIVER
  /* Right side BTS7960 drives BOTH the right-front and right-rear motors,
     which are wired in parallel to this single driver. */
  #define RIGHT_MOTOR_RPWM 10
  #define RIGHT_MOTOR_LPWM 9

  /* Left side BTS7960 drives BOTH the left-front and left-rear motors,
     which are wired in parallel to this single driver. */
  #define LEFT_MOTOR_RPWM  5
  #define LEFT_MOTOR_LPWM  6

  /* R_EN/L_EN on each BTS7960 module are assumed tied HIGH (5V) in
     hardware, as is common practice when no spare Arduino pins are
     used for them. If your modules have EN wired to Arduino pins
     instead, define these and initMotorController() will drive them. */
  //#define RIGHT_MOTOR_R_EN 7
  //#define RIGHT_MOTOR_L_EN 8
  //#define LEFT_MOTOR_R_EN  11
  //#define LEFT_MOTOR_L_EN  12
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
