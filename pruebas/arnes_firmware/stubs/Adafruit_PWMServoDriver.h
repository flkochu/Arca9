#pragma once
#include "Arduino.h"
#include "Wire.h"

// Como la librería de Adafruit, pero sobre el PCA9685 de mentiras de Wire.h (mismas cuentas).
struct Adafruit_PWMServoDriver {
  uint32_t osc = 25000000;
  Adafruit_PWMServoDriver(uint8_t = 0x40) {}
  bool begin() { g_pca.reg[0x00] = 0x01; return true; }
  void setOscillatorFrequency(uint32_t f) { osc = f; }
  uint32_t getOscillatorFrequency() { return osc; }
  void setPWMFreq(float freq) {                    // Adafruit_PWMServoDriver.cpp, setPWMFreq()
    float prescaleval = ((osc / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < 3) prescaleval = 3;
    if (prescaleval > 255) prescaleval = 255;
    g_pca.reg[0xFE] = (uint8_t)prescaleval;
    g_pca.reg[0x00] |= 0x20;                       // auto-incremento
    g_pca.transacciones += 4;
  }
  uint8_t readPrescale() { g_pca.transacciones++; g_pca.lecturas++; return g_pca.reg[0xFE]; }
  uint8_t setPWM(uint8_t num, uint16_t on, uint16_t off) {
    g_pca.transacciones++;
    if (g_pca.fallar > 0) { g_pca.fallar--; return 1; }
    uint8_t b = 0x06 + 4 * num;
    g_pca.reg[b] = on & 0xFF; g_pca.reg[b + 1] = on >> 8;
    g_pca.reg[b + 2] = off & 0xFF; g_pca.reg[b + 3] = off >> 8;
    return 0;
  }
  void writeMicroseconds(uint8_t num, uint16_t Microseconds) {   // igual que la librería
    double pulse = Microseconds;
    double pulselength = 1000000;
    uint16_t prescale = readPrescale();
    prescale += 1;
    pulselength *= prescale;
    pulselength /= osc;
    pulse /= pulselength;
    setPWM(num, 0, pulse);
  }
};
