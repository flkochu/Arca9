#pragma once
#include "Arduino.h"
struct TwoWire {
  uint8_t addr = 0;
  void begin(int = -1, int = -1) {}
  void beginTransmission(uint8_t a) { addr = a; }
  uint8_t endTransmission() { return addr == 0x40 ? 0 : 2; }
};
extern TwoWire Wire;
