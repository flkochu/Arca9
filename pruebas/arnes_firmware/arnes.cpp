// Arnés: compila esp32_principal.ino y esp32_aux.ino en la compu, los une por un UART virtual
// y revisa el protocolo. Con --pty expone el USB de la principal en un pseudo-terminal para
// probar eficiencia_servos.py de punta a punta.
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <random>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

uint64_t g_us = 0;
int g_board = 0;
TwoWire Wire;
int g_pca_us[16] = {0};
bool g_pca_suelto[16] = {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true};
inline void digitalWrite(int, int) {}
std::mt19937 rng(3);
std::normal_distribution<double> ruido(0.0, 1.5);
double pos_us[12];

double mv_canal(int ch) {
  if (!g_pca_suelto[ch] && g_pca_us[ch] > 0) pos_us[ch] = g_pca_us[ch];
  double us = pos_us[ch];
  double tope = (ch == 5) ? 2300 : 2470;                   // un servo con tope antes
  if (us > tope) us = tope;
  double cable = 3000 - 1.404 * (us - 500);
  return 0.6 * cable + ruido(rng);
}

namespace principal {
SerialStub Serial, Serial2;
int analogReadMilliVolts(int pin) {
  int ch = pin == 32 ? 0 : pin == 33 ? 1 : pin == 35 ? 2 : -1;
  return ch < 0 ? 0 : (int)lround(mv_canal(ch));
}
#include "../../firmware/esp32_principal/esp32_principal.ino"
}  // namespace principal

namespace aux {
SerialStub Serial, Serial2;
int analogReadMilliVolts(int pin) {
  const int pins[9] = {32, 33, 35, 34, 36, 39, 25, 26, 27};
  for (int k = 0; k < 9; k++)
    if (pins[k] == pin) return (int)lround(mv_canal(3 + k));
  return 0;
}
#include "../../firmware/esp32_aux/esp32_aux.ino"
}  // namespace aux

bool aux_vivo = true;
void hook_otra() {
  if (!aux_vivo || g_board != 0) return;
  g_board = 1;
  aux::loop();
  g_board = 0;
}

std::string salida;                // lo que la principal manda por USB
void mandar(const std::string &s) { for (char c : s) principal::Serial.rx.push_back((uint8_t)c); }

std::string correr(const std::string &cmd, int vueltas = 300) {
  salida.clear();
  mandar(cmd + "\n");
  for (int i = 0; i < vueltas; i++) {
    principal::loop();
    g_us += 200;
    hook_otra();
  }
  return salida;
}

int fallas = 0;
void revisa(bool ok, const char *que, const std::string &detalle = "") {
  printf("%s  %s%s%s\n", ok ? "OK  " : "FALLA", que, detalle.empty() ? "" : "  ->  ", detalle.c_str());
  if (!ok) fallas++;
}
std::string sin_nl(std::string s) { while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back(); return s; }

int main(int argc, char **argv) {
  for (int i = 0; i < 12; i++) pos_us[i] = 1500;
  principal::Serial.sink = [](const char *s, size_t n) { salida.append(s, n); };
  principal::Serial2.peer = &aux::Serial2;
  aux::Serial2.peer = &principal::Serial2;
  aux::Serial.sink = [](const char *, size_t) {};
  principal::Serial2.al_vaciarse = [] { hook_otra(); };

  g_board = 1; aux::setup(); g_board = 0;
  principal::setup();

  if (argc > 1 && std::string(argv[1]) == "--pty") {
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(m); unlockpt(m);
    struct termios t; tcgetattr(m, &t); cfmakeraw(&t); tcsetattr(m, TCSANOW, &t);
    fcntl(m, F_SETFL, O_NONBLOCK);
    printf("%s\n", ptsname(m)); fflush(stdout);
    principal::Serial.sink = [m](const char *s, size_t n) { ssize_t r = write(m, s, n); (void)r; };
    char b[256];
    for (;;) {
      ssize_t n = read(m, b, sizeof b);
      for (ssize_t i = 0; i < n; i++) principal::Serial.rx.push_back((uint8_t)b[i]);
      principal::loop();
      g_us += 200;
      hook_otra();
      if (n <= 0) usleep(100);
    }
  }

  std::string arranque = salida;
  revisa(arranque.find("OK1-RETRO listo PCA=ok FB=12 AUX=ok") != std::string::npos, "arranque", sin_nl(arranque));
  std::string r = correr("?");
  revisa(r.find("PCA=ok FB=12 AUX=ok DESC=0") != std::string::npos, "? con la auxiliar", sin_nl(r));
  r = correr("P 4 2500");
  revisa(r == "OK P 4 2360\n", "P recorta a 2360", sin_nl(r));
  r = correr("P 0 1500");
  r = correr("R 0 50");
  { int ch, n; float m, s; int a, b;
    bool ok = sscanf(r.c_str(), "R %d %d %f %f %d %d", &ch, &n, &m, &s, &a, &b) == 6 && ch == 0 && n == 50 && fabs(m - 957.6) < 3;
    revisa(ok, "R local (FL, canal 0)", sin_nl(r)); }
  correr("P 7 1000");
  r = correr("R 7 50");
  { int ch, n; float m, s; int a, b;
    bool ok = sscanf(r.c_str(), "R %d %d %f %f %d %d", &ch, &n, &m, &s, &a, &b) == 6 && ch == 7 && n == 50 && fabs(m - 1378.8) < 3;
    revisa(ok, "R por la auxiliar (RL cadera, canal 7)", sin_nl(r)); }
  r = correr("V");
  { std::vector<int> v; const char *p = r.c_str() + 1; char *fin;
    for (;;) { long x = strtol(p, &fin, 10); if (fin == p) break; v.push_back((int)x); p = fin; }
    bool ok = v.size() == 12 && abs(v[0] - 958) < 8 && abs(v[7] - 1379) < 8 && abs(v[4] - 0.6 * (3000 - 1.404 * 1860)) < 8;
    revisa(ok, "V da los 12", sin_nl(r)); }
  correr("P 5 2360");
  r = correr("R 5 20");
  { int ch, n; float m, s; int a, b;
    bool ok = sscanf(r.c_str(), "R %d %d %f %f %d %d", &ch, &n, &m, &s, &a, &b) == 6 && fabs(m - 0.6 * (3000 - 1.404 * 1800)) < 3;
    revisa(ok, "servo con tope (canal 5) se queda en su tope", sin_nl(r)); }
  mandar("");
  for (const char *c = "A 1 2 3 4 5 6 7 8 9 999\n"; *c; c++) principal::Serial2.rx.push_back((uint8_t)*c);
  r = correr("?");
  revisa(r.find("DESC=1") != std::string::npos, "línea A con suma mala se descarta", sin_nl(r));
  r = correr("R 12 5");
  revisa(r.rfind("E R", 0) == 0, "R de un canal sin verde da error", sin_nl(r));
  r = correr("X");
  revisa(r.rfind("E comando", 0) == 0, "comando desconocido", sin_nl(r));

  // la ESP32 local lee mucho tiempo mientras la auxiliar sigue mandando: se llena el buffer
  r = correr("R 1 1000", 50);
  revisa(r.rfind("R 1 1000", 0) == 0, "R local largo (1000 lecturas)", sin_nl(r));
  r = correr("R 9 20");
  revisa(r.rfind("R 9 20", 0) == 0, "R por la auxiliar después de llenarse el buffer",
         sin_nl(r) + "  (bytes perdidos: " + std::to_string(principal::Serial2.perdidos) + ")");
  r = correr("?");
  revisa(r.find("AUX=ok") != std::string::npos, "la auxiliar sigue bien", sin_nl(r));

  // se desconecta la auxiliar
  aux_vivo = false;
  for (int i = 0; i < 300; i++) { principal::loop(); g_us += 1000; }   // 300 ms sin la auxiliar
  r = correr("?");
  revisa(r.find("FB=3 AUX=NO") != std::string::npos, "? sin la auxiliar", sin_nl(r));
  r = correr("R 7 10");
  revisa(r.rfind("E R: la auxiliar no manda datos", 0) == 0, "R sin la auxiliar da error claro", sin_nl(r));
  r = correr("R 2 10");
  revisa(r.rfind("R 2 10", 0) == 0, "FL sigue funcionando sin la auxiliar", sin_nl(r));
  aux_vivo = true;
  r = correr("R 10 10");
  revisa(r.rfind("R 10 10", 0) == 0, "se reconecta la auxiliar", sin_nl(r));
  r = correr("D");
  bool sueltos = true;
  for (int i = 0; i < 16; i++) sueltos = sueltos && g_pca_suelto[i];
  revisa(r == "OK D\n" && sueltos, "D suelta los 16 canales", sin_nl(r));
  r = correr("P 3 1500\r");
  revisa(r == "OK P 3 1500\n", "acepta CR+LF (Both NL & CR)", sin_nl(r));
  printf("\n%s (%d fallas)\n", fallas ? "HAY FALLAS" : "TODO BIEN", fallas);
  return fallas ? 1 : 0;
}
