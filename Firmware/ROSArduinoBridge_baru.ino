// // JANGAN DIPAKE
// // Konfigurasi Awal (ESP32)
// // *********************************************************************

// #define USE_BASE            // Aktifkan pengendali dasar (base controller)
// #undef USE_SERVOS          // Nonaktifkan penggunaan servo PWM

// // Pilih jenis motor driver dan encoder
// #ifdef USE_BASE
//   #define ARDUINO_ENC_COUNTER   // Gunakan encoder langsung ke pin ESP32
//   #define BTS7960_MOTOR_DRIVER  // Gunakan driver motor BTS7960
// #endif

// #define BAUDRATE       115200   // Baudrate komunikasi serial
// #define MAX_PWM        255      // Nilai maksimum PWM

// #include "Arduino.h"
// #include "commands.h"         // Daftar perintah serial
// #include "sensors.h"          // Fungsi sensor

// #ifdef USE_SERVOS
//   #include <ESP32Servo.h>
//   #include "servos.h"
// #endif

// #ifdef USE_BASE
//   #include "motor_driver.h"      // Fungsi pengendali motor
//   #include "encoder_driver.h"    // Fungsi pembacaan encoder
//   #include "diff_controller_baru.h"   // Fungsi pengendali PID

//   #define PID_RATE        30     // Frekuensi update PID (Hz)
//   const int PID_INTERVAL = 1000 / PID_RATE;
//   unsigned long nextPID = PID_INTERVAL;

//   #define AUTO_STOP_INTERVAL 2000
//   long lastMotorCommand = AUTO_STOP_INTERVAL;
// #endif

// // PWM parameters untuk ESP32
// static const int PWM_CHANNEL = 0;
// static const int PWM_FREQ    = 5000;
// static const int PWM_RES     = 8;

// // *********************************************************************
// // Variabel Global
// // *********************************************************************

// int arg = 0;
// char chr;
// char cmd;
// char argv1[16];
// char argv2[16];
// long arg1;
// long arg2;
// int idx = 0;

// // *********************************************************************
// // Reset parameter perintah
// // *********************************************************************

// void resetCommand() {
//   cmd = 0;
//   memset(argv1, 0, sizeof(argv1));
//   memset(argv2, 0, sizeof(argv2));
//   arg1 = 0;
//   arg2 = 0;
//   arg = 0;
//   idx = 0;
// }

// // *********************************************************************
// // Eksekusi perintah dari serial
// // *********************************************************************

// int runCommand() {
//     // Cek apakah cmd 0, jika ya, keluar dari fungsi
//   if (cmd == 0) return 0;  // Tambahkan ini di awal fungsi runCommand()

//   int i = 0;
//   char *p = argv1;
//   char *str;
//   int pid_args[4];

//   arg1 = atol(argv1);
//   arg2 = atol(argv2);

//   switch (cmd) {
//     case GET_BAUDRATE:
//       Serial.println(BAUDRATE);
//       break;

//     case ANALOG_READ:
//       Serial.println(analogRead(arg1));
//       break;

//     case DIGITAL_READ:
//       Serial.println(digitalRead(arg1));
//       break;

//     case ANALOG_WRITE: {
//       // Gunakan LEDC untuk PWM di ESP32
//       ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
//       ledcAttachPin(arg1, PWM_CHANNEL);
//       ledcWrite(PWM_CHANNEL, arg2);
//       Serial.println("OK");
//       break;
//     }

//     case DIGITAL_WRITE:
//       digitalWrite(arg1, arg2 ? HIGH : LOW);
//       Serial.println("OK");
//       break;

//     case PIN_MODE:
//       pinMode(arg1, arg2 ? OUTPUT : INPUT);
//       Serial.println("OK");
//       break;

//     case PING:
//       Serial.println(Ping(arg1));
//       break;

// #ifdef USE_SERVOS
//     case SERVO_WRITE:
//       servos[arg1].write(arg2);
//       Serial.println("OK");
//       break;

//     case SERVO_READ:
//       Serial.println(servos[arg1].read());
//       break;
// #endif

// #ifdef USE_BASE
//     case READ_ENCODERS:
//       Serial.print(readEncoder(LEFT));
//       Serial.print(" ");
//       Serial.println(readEncoder(RIGHT));
//       break;

//     case RESET_ENCODERS:
//       resetEncoders();
//       resetPID();
//       Serial.println("OK");
//       break;

//     case MOTOR_SPEEDS:
//       lastMotorCommand = millis();
//       if (arg1 == 0 && arg2 == 0) {
//         setMotorSpeeds(0, 0);
//         resetPID();
//         moving = 0;
//       } else {
//         moving = 1;
//       }
//       leftPID.TargetTicksPerFrame  = arg1;
//       rightPID.TargetTicksPerFrame = arg2;
//       Serial.println("OK");
//       break;

//     case MOTOR_RAW_PWM:
//       lastMotorCommand = millis();
//       resetPID();
//       moving = 0;
//       setMotorSpeeds(arg1, arg2);
//       Serial.println("OK");
//       break;

//     // case UPDATE_PID:
//     //   while ((str = strtok_r(p, ":", &p)) != NULL && i < 4) {
//     //     pid_args[i++] = atoi(str);
//     //   }
//     //   if (i == 4) {
//     //     Kp = pid_args[0];
//     //     Kd = pid_args[1];
//     //     Ki = pid_args[2];
//     //     Ko = pid_args[3];
//     //     Serial.println("OK");
//     //   } else {
//     //     Serial.println("Invalid PID Params");
//     //   }
//     //   break;

//     case UPDATE_PID:
//       while ((str = strtok_r(p, ":", &p)) != NULL && i < 4) {
//         pid_args[i++] = atoi(str);
//       }
//       if (i == 4) {
//     // Update parameter PID untuk kedua motor
//         SetTunings(&leftPID, pid_args[0], pid_args[2], pid_args[1]);  // Kp, Ki, Kd
//         SetTunings(&rightPID, pid_args[0], pid_args[2], pid_args[1]);
//         leftPID.Ko = pid_args[3];
//         rightPID.Ko = pid_args[3];
//         Serial.println("OK");
//       } else {
//         Serial.println("Invalid PID Params");
//       }
//       break;
// #endif

//     default:
//       Serial.println("Invalid Command");
//       break;
//   }
//   return 1;
// }

// // *********************************************************************
// // Fungsi setup (dijalankan sekali saat startup)
// // *********************************************************************

// void setup() {
//   Serial.begin(BAUDRATE);
//   // Tunggu hingga serial siap (opsional)
//   while (!Serial) { delay(10); }

// #ifdef USE_BASE
//   #ifdef ARDUINO_ENC_COUNTER
//     pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
//     pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
//     pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
//     pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

//     attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderISR, CHANGE);
//   #endif

//   initMotorController();
//   resetPID();
// #endif

// #ifdef USE_SERVOS
//   for (int i = 0; i < N_SERVOS; i++) {
//     servos[i].attach(servoPins[i]);
//   }
// #endif
// }

// // *********************************************************************
// // Fungsi loop utama (berjalan terus menerus)
// // *********************************************************************

// void loop() {
//   while (Serial.available() > 0) {
//     chr = Serial.read();

//     if (chr == '\r' || chr == '\n') {
//       if (arg == 1) argv1[idx] = '\0';
//       else if (arg == 2) argv2[idx] = '\0';
//       runCommand();
//       resetCommand();
//     } else if (chr == ' ') {
//       if (arg == 0) arg = 1;
//       else if (arg == 1) {
//         argv1[idx] = '\0';
//         arg = 2;
//         idx = 0;
//       }
//     } else {
//       if (arg == 0) cmd = chr;
//       else if (arg == 1 && idx < sizeof(argv1)-1) argv1[idx++] = chr;
//       else if (arg == 2 && idx < sizeof(argv2)-1) argv2[idx++] = chr;
//     }
//   }

// #ifdef USE_BASE
//   if (millis() > nextPID) {
//     updatePID();
//     nextPID += PID_INTERVAL;
//   }

//   if (millis() - lastMotorCommand > AUTO_STOP_INTERVAL) {
//     setMotorSpeeds(0, 0);
//     moving = 0;
//   }
// #endif

// #ifdef USE_SERVOS
//   // Tidak perlu sweep manual, library ESP32Servo handle
// #endif
// }

