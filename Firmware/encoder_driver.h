/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A 32 //pin 
  #define LEFT_ENC_PIN_B 33  //pin 
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A 25  //pin 
  #define RIGHT_ENC_PIN_B 26   //pin
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

