#ifdef USE_BASE

#ifdef ROBOGAIA
  #include "MegaEncoderCounter.h"

  MegaEncoderCounter encoders = MegaEncoderCounter(4);

  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  void resetEncoder(int i) {
    if (i == LEFT) encoders.YAxisReset();
    else encoders.XAxisReset();
  }

#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

  // Simpan status sebelumnya untuk masing-masing encoder
  volatile uint8_t left_enc_last = 0;
  volatile uint8_t right_enc_last = 0;

  // Fungsi pembacaan 2 bit pin
  inline uint8_t readEncoderBits(int pinA, int pinB) {
    return (digitalRead(pinA) << 1) | digitalRead(pinB);
  }

  void leftEncoderISR() {
    left_enc_last <<= 2;
    left_enc_last |= readEncoderBits(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
    left_enc_pos += ENC_STATES[left_enc_last & 0x0F];
  }

  void rightEncoderISR() {
    right_enc_last <<= 2;
    right_enc_last |= readEncoderBits(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
    right_enc_pos += ENC_STATES[right_enc_last & 0x0F];
  }

  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  void resetEncoder(int i) {
    if (i == LEFT) {
      left_enc_pos = 0L;
    } else {
      right_enc_pos = 0L;
    }
  }

#else
  #error An encoder driver must be selected!
#endif

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
