#pragma once
#include "Arduino.h"
extern int g_pca_us[16];
extern bool g_pca_suelto[16];
struct Adafruit_PWMServoDriver {
  Adafruit_PWMServoDriver(uint8_t = 0x40) {}
  void begin() {}
  void setOscillatorFrequency(uint32_t) {}
  void setPWMFreq(float) {}
  void setPWM(int ch, int on, int off) { if (off >= 4096) g_pca_suelto[ch] = true; }
  void writeMicroseconds(int ch, int us) { g_pca_us[ch] = us; g_pca_suelto[ch] = false; }
};
