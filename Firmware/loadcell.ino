#ifdef USE_LOADCELL

#include <HX711.h>

// Pin HX711
const int DT_PIN = 13;
const int SCK_PIN = 5;

// Faktor kalibrasi (ubah sesuai kebutuhan)
float calibration_factor = -28000.0+1.015;

HX711 scale;

void setup_loadcell() {
  scale.begin(DT_PIN, SCK_PIN);
  scale.set_scale();
  scale.tare();
  delay(2000);
  scale.set_scale(calibration_factor);
}

void loop_loadcell() {
  // Bisa dikosongkan atau aktif jika ingin tampil periodik
}

float readLoadcell() {
  return abs(scale.get_units(5));  // nilai positif
}

void resetLoadcell() {
  scale.tare();
}

// Fungsi tambahan untuk set kalibrasi dari luar modul ini
void setCalibrationFactor(float factor) {
  calibration_factor = factor;
  scale.set_scale(calibration_factor);
}

// Fungsi tambahan untuk baca nilai mentah (raw)
long readRawLoadcell() {
  return scale.read_average(5);  // rata-rata 5 pembacaan
}


#endif