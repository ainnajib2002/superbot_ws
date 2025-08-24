/***************************************************************
   Motor driver function definitions - by James Nugen
 ***************************************************************/

#ifdef BTS7960_MOTOR_DRIVER
  // Pin untuk BTS7960 ESP32 (silakan sesuaikan dengan wiring kamu)
  #define RIGHT_MOTOR_RPWM    4
  #define RIGHT_MOTOR_LPWM    0
  #define RIGHT_MOTOR_REN     2
  #define RIGHT_MOTOR_LEN     15

  #define LEFT_MOTOR_RPWM     27 
  #define LEFT_MOTOR_LPWM     14  
  #define LEFT_MOTOR_REN      19 
  #define LEFT_MOTOR_LEN      18 

  // Channel PWM ESP32
  #define CH_RPWM_R 0
  #define CH_LPWM_R 1
  #define CH_RPWM_L 2
  #define CH_LPWM_L 3
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
