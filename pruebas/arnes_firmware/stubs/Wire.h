#pragma once
#include "Arduino.h"
#include <vector>

// Un PCA9685 de mentiras, a nivel de registros, para contar las transacciones de I²C y revisar
// exactamente qué ticks quedan en cada canal.
struct PcaSim {
  uint8_t reg[256] = {0};
  long transacciones = 0;      // cada beginTransmission/endTransmission o setPWM o lectura
  long lecturas = 0;           // lecturas de registros (readPrescale)
  long escrituras_por_wire = 0;
  int fallar = 0;              // cuántas transacciones siguientes fallan (prueba de errores)
  uint8_t codigo_falla = 4;    // lo que regresa endTransmission() al fallar (4 = otra falla, 5 = tiempo)
  uint64_t us_falla = 0;       // lo que tarda en fallar (con 5: el timeout de Wire)
  uint32_t reloj_i2c = 0;
};
extern PcaSim g_pca;

struct TwoWire {
  uint8_t addr = 0;
  std::vector<uint8_t> buf;
  void begin(int = -1, int = -1) {}
  void setClock(uint32_t hz) { g_pca.reloj_i2c = hz; }
  void beginTransmission(uint8_t a) { addr = a; buf.clear(); }
  size_t write(uint8_t b) { buf.push_back(b); return 1; }
  uint8_t endTransmission(bool = true) {
    if (addr != 0x40) return 2;                    // nadie contesta en esa dirección
    if (buf.empty()) return 0;                     // solo preguntar si está (setup)
    g_pca.transacciones++;
    g_pca.escrituras_por_wire++;
    if (g_pca.fallar > 0) { g_pca.fallar--; g_us += g_pca.us_falla; return g_pca.codigo_falla; }
    // lo que tarda en el cable: dirección + datos, 9 bits por byte (50 bytes a 100 kHz = 4.5 ms)
    if (g_pca.reloj_i2c) g_us += (uint64_t)(1 + buf.size()) * 9 * 1000000 / g_pca.reloj_i2c;
    uint8_t r = buf[0];                            // auto-incremento: un registro tras otro
    for (size_t i = 1; i < buf.size(); i++) g_pca.reg[(uint8_t)(r + i - 1)] = buf[i];
    return 0;
  }
};
extern TwoWire Wire;
