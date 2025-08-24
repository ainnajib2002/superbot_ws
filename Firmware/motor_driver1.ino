#ifdef USE_BASE
#define BTS7960_MOTOR_DRIVER  // Pilih driver motor BTS7960

#ifdef POLOLU_VNH5019
  #include "DualVNH5019MotorShield.h"
  DualVNH5019MotorShield drive;
  
  void initMotorController() { drive.init(); }
  
  void setMotorSpeed(int i, int spd, bool allowRamp = true) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined POLOLU_MC33926
  #include "DualMC33926MotorShield.h"
  DualMC33926MotorShield drive;
  
  void initMotorController() { drive.init(); }
  
  void setMotorSpeed(int i, int spd, bool allowRamp = true) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd, bool allowRamp = true) {
    unsigned char reverse = 0;
    if (spd < 0) {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
    else {
      if (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined BTS7960_MOTOR_DRIVER
  #include "motor_driver.h"

  // Variabel global untuk menyimpan kecepatan sebelumnya
  int lastLeftSpeed = 0;
  int lastRightSpeed = 0;

  void initMotorController() {
    const int PWM_FREQ = 20000;
    const int PWM_RES  = 8;

    // RIGHT motor
    ledcSetup(CH_RPWM_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(RIGHT_MOTOR_RPWM, CH_RPWM_R);
    ledcSetup(CH_LPWM_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(RIGHT_MOTOR_LPWM, CH_LPWM_R);
    pinMode(RIGHT_MOTOR_REN, OUTPUT);
    pinMode(RIGHT_MOTOR_LEN, OUTPUT);
    digitalWrite(RIGHT_MOTOR_REN, HIGH);
    digitalWrite(RIGHT_MOTOR_LEN, HIGH);

    // LEFT motor
    ledcSetup(CH_RPWM_L, PWM_FREQ, PWM_RES);
    ledcAttachPin(LEFT_MOTOR_RPWM, CH_RPWM_L);
    ledcSetup(CH_LPWM_L, PWM_FREQ, PWM_RES);
    ledcAttachPin(LEFT_MOTOR_LPWM, CH_LPWM_L);
    pinMode(LEFT_MOTOR_REN, OUTPUT);
    pinMode(LEFT_MOTOR_LEN, OUTPUT);
    digitalWrite(LEFT_MOTOR_REN, HIGH);
    digitalWrite(LEFT_MOTOR_LEN, HIGH);
  }

  // ======== Tambahkan deklarasi fungsi rampMotorToStop di sini ========
  void rampMotorToStop(int i, int currentSpd);

  // Fungsi untuk mengatur kecepatan motor
  void setMotorSpeed(int i, int spd, bool allowRamp = true) {
    int pwmVal = constrain(abs(spd), 0, 255);
    bool forward = spd >= 0;

    if (allowRamp) {
      if (i == LEFT && spd == 0 && lastLeftSpeed != 0) {
        rampMotorToStop(LEFT, lastLeftSpeed);
        lastLeftSpeed = 0;
        return;
      }
      if (i == RIGHT && spd == 0 && lastRightSpeed != 0) {
        rampMotorToStop(RIGHT, lastRightSpeed);
        lastRightSpeed = 0;
        return;
      }
    }

    // Motor jalan normal
    if (i == LEFT) {
      ledcWrite(CH_RPWM_L, forward ? pwmVal : 0);
      ledcWrite(CH_LPWM_L, forward ? 0 : pwmVal);
      lastLeftSpeed = spd;
    } else if (i == RIGHT) {
      ledcWrite(CH_RPWM_R, forward ? pwmVal : 0);
      ledcWrite(CH_LPWM_R, forward ? 0 : pwmVal);
      lastRightSpeed = spd;
    }
  }

  // Fungsi ramp down (smooth berhenti)
  void rampMotorToStop(int i, int currentSpd) {
    int step = 50;        // Nilai step pengurangan speed
    int delayTime = 10;  // Delay antar step (ms)
    int spd = currentSpd;

    if (currentSpd > 0) {
      while (spd > 0) {
        spd -= step;
        if (spd < 0) spd = 0;
        setMotorSpeed(i, spd, false);  // False = tidak perlu ramp recursive
        delay(delayTime);
      }
    } else if (currentSpd < 0) {
      while (spd < 0) {
        spd += step;
        if (spd > 0) spd = 0;
        setMotorSpeed(i, spd, false);
        delay(delayTime);
      }
    }
  }

  // // Fungsi set kecepatan kedua motor
  // void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  //   setMotorSpeed(LEFT, leftSpeed);
  //   setMotorSpeed(RIGHT, rightSpeed);
  // }

// Fungsi set kecepatan kedua motor (dengan opsi ramping)
void setMotorSpeeds(int leftSpeed, int rightSpeed, bool allowRamp) {
  setMotorSpeed(LEFT, leftSpeed, allowRamp);
  setMotorSpeed(RIGHT, rightSpeed, allowRamp);
}

// Fungsi kompatibilitas (default ramp = true)
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeeds(leftSpeed, rightSpeed, true);
}

#else
  #error A motor driver must be selected!
#endif

#endif