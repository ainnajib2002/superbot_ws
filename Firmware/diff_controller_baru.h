// /* Functions and type-defs for PID control dengan Adaptive Tuning */Jangan dipake!!
// /* PID setpoint info For a Motor */
// typedef struct {
//   double TargetTicksPerFrame;    // Target speed dalam ticks per frame
//   long Encoder;                  // Pembacaan encoder saat ini
//   long PrevEnc;                  // Pembacaan encoder sebelumnya
//   int PrevInput;                 // Input sebelumnya untuk menghitung turunan
//   double ITerm;                  // Term integral
//   long output;                   // Output motor terakhir
  
//   /* Parameter Adaptive Tuning */
//   unsigned long lastTime;        // Waktu terakhir perhitungan PID
//   int SampleTime;                // Waktu sampling dalam ms
//   double kp;                     // Gain proporsional
//   double ki;                     // Gain integral yang disesuaikan
//   double kd;                     // Gain turunan yang disesuaikan
//   int Ko;                        // Faktor skala output
// } SetPointInfo;

// SetPointInfo leftPID, rightPID;

// /* Parameter PID default */
// #define DEFAULT_KP 80
// #define DEFAULT_KI 50
// #define DEFAULT_KD 10
// #define DEFAULT_KO 50
// #define DEFAULT_SAMPLE_TIME 1000 // 1 detik

// unsigned char moving = 0; // Status pergerakan robot

// /* Fungsi untuk mengatur parameter PID dengan penyesuaian waktu sampling */
// void SetTunings(SetPointInfo*p, double Kp, double Ki, double Kd) {
//   double SampleTimeInSec = ((double)p->SampleTime)/1000;
//   p->kp = Kp;
//   p->ki = Ki * SampleTimeInSec;  // Menyesuaikan Ki dengan waktu sampling
//   p->kd = Kd / SampleTimeInSec;  // Menyesuaikan Kd dengan waktu sampling
// }

// /* Fungsi untuk mengubah waktu sampling */
// void SetSampleTime(SetPointInfo *p, int NewSampleTime) {
//   if (NewSampleTime > 0) {
//     double ratio = (double)NewSampleTime / (double)p->SampleTime;
//     p->ki *= ratio;
//     p->kd /= ratio;
//     p->SampleTime = (unsigned long)NewSampleTime;
//   }
// }

// /* Reset parameter PID dan inisialisasi ulang */
// void resetPID() {
//   // Reset PID kiri
//   leftPID.TargetTicksPerFrame = 0.0;
//   leftPID.Encoder = readEncoder(LEFT);
//   leftPID.PrevEnc = leftPID.Encoder;
//   leftPID.PrevInput = 0;
//   leftPID.ITerm = 0;
//   leftPID.output = 0;
//   leftPID.SampleTime = DEFAULT_SAMPLE_TIME;
//   leftPID.Ko = 30;
//   SetTunings(&leftPID, 80, 15, 25);
//   leftPID.lastTime = millis();

//   // Reset PID kanan
//   rightPID.TargetTicksPerFrame = 0.0;
//   rightPID.Encoder = readEncoder(RIGHT);
//   rightPID.PrevEnc = rightPID.Encoder;
//   rightPID.PrevInput = 0;
//   rightPID.ITerm = 0;
//   rightPID.output = 0;
//   rightPID.SampleTime = DEFAULT_SAMPLE_TIME;
//   rightPID.Ko = 30;
//   SetTunings(&rightPID, 80, 15, 25);
//   rightPID.lastTime = millis();
// }

// /* Perhitungan PID dengan adaptive tuning */
// void doPID(SetPointInfo *p) {
//   unsigned long now = millis();
//   unsigned long timeChange = now - p->lastTime;
  
//   if (timeChange >= p->SampleTime) {
//     // Hitung input sebagai selisih encoder
//     int input = p->Encoder - p->PrevEnc;
//     double error = p->TargetTicksPerFrame - input;
//     double dInput = input - p->PrevInput;

//     // Hitung komponen PID
//     double pTerm = p->kp * error;
//     double dTerm = -p->kd * dInput;
    
//     // Hitung output sementara
//     long output = (pTerm + p->ITerm + dTerm) / p->Ko;

//     // Anti-windup dan clamping output
//     if (output > MAX_PWM) {
//       output = MAX_PWM;
//     } else if (output < -MAX_PWM) {
//       output = -MAX_PWM;
//     } else {
//       // Jika tidak jenuh, update term integral
//       p->ITerm += p->ki * error;
//     }

//     // Update state
//     p->output = output;
//     p->PrevEnc = p->Encoder;
//     p->PrevInput = input;
//     p->lastTime = now;
//   }
// }

// /* Update PID dan pembacaan encoder */
// void updatePID() {
//   // Baca nilai encoder terbaru
//   leftPID.Encoder = readEncoder(LEFT);
//   rightPID.Encoder = readEncoder(RIGHT);

//   if (!moving) {
//     if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
//     return;
//   }

//   // Lakukan perhitungan PID untuk kedua motor
//   doPID(&leftPID);
//   doPID(&rightPID);

//   // Update kecepatan motor
//   setMotorSpeeds(leftPID.output, rightPID.output);
// }