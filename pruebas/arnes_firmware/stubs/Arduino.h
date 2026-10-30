#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <cstdarg>
#include <deque>
#include <string>
#include <functional>
using std::sqrt;
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define SERIAL_8N1 0x800001c
enum { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db };
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

extern uint64_t g_us;
extern int g_board;                  // 0 = principal, 1 = auxiliar
void hook_otra();                    // corre la otra placa mientras esta espera
inline unsigned long micros() { g_us += 1; return (unsigned long)g_us; }
inline unsigned long millis() { g_us += 1; return (unsigned long)(g_us / 1000); }
inline void delayMicroseconds(unsigned us) { g_us += us; if (g_board == 0) hook_otra(); }
inline void delay(unsigned ms) { g_us += (uint64_t)ms * 1000; if (g_board == 0) hook_otra(); }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v);
inline void analogSetPinAttenuation(int, int) {}

struct SerialStub {
  std::deque<uint8_t> rx;
  size_t cap = 256;
  unsigned long perdidos = 0;
  SerialStub *peer = nullptr;
  std::function<void(const char *, size_t)> sink;
  std::function<void()> al_vaciarse;
  void begin(long, int = 0, int = -1, int = -1) {}
  void setRxBufferSize(size_t n) { cap = n; }
  int available() { if (rx.empty() && al_vaciarse) al_vaciarse(); return (int)rx.size(); }
  int read() { if (rx.empty()) return -1; int c = rx.front(); rx.pop_front(); return c; }
  void entregar(const char *s, size_t n) {
    if (peer) { for (size_t i = 0; i < n; i++) { if (peer->rx.size() < peer->cap) peer->rx.push_back((uint8_t)s[i]); else peer->perdidos++; } }
    else if (sink) sink(s, n);
  }
  size_t print(const char *s) { entregar(s, strlen(s)); return strlen(s); }
  size_t println(const char *s) { print(s); print("\n"); return strlen(s) + 1; }
  size_t println() { print("\n"); return 1; }
  size_t printf(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    char b[512]; va_list ap; va_start(ap, fmt); int n = vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    entregar(b, (size_t)n); return (size_t)n;
  }
};
