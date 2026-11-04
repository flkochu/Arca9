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
    principal::Serial.sink = [m](const char *s, size_t n) {   // como el USB real: espera, no tira bytes
      while (n > 0) {
        ssize_t r = write(m, s, n);
        if (r > 0) { s += r; n -= (size_t)r; } else usleep(200);
      }
    };
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

  // ---------------------------------------------------------------- protocolo v2: rampas (G)
  r = correr("?");
  { size_t q = r.find("PROTO="); int v = q == std::string::npos ? 0 : atoi(r.c_str() + q + 6);
    revisa(v >= 2, "? dice PROTO=2 o mayor (G)", sin_nl(r)); }
  r = correr("G 0 819 2097 2556 1 200");
  revisa(r.rfind("E G: primero manda P 0 819", 0) == 0, "G sin P previo no da saltos", sin_nl(r));
  correr("P 0 819");
  r = correr("G 0 819 2400 2556 1 200");
  revisa(r.rfind("E G: us_a y us_b", 0) == 0, "G no acepta pulsos arriba de 2360", sin_nl(r));
  r = correr("G 0 819 2097 50 10 5000");
  revisa(r.rfind("E G: muy larga", 0) == 0, "G muy larga da error", sin_nl(r));
  r = correr("G 0 819 2097 9000 1 200");
  revisa(r.rfind("E G: vel", 0) == 0, "G con velocidad fuera de rango", sin_nl(r));

  struct Rampa { int ok = 0, ch = -1, nM = -1, nC = -1, lm = 0, lc = 0, fin = 0, usMin = 99999, usMax = 0,
                 mvMin = 99999, tMax = 0; std::string error; };
  auto leer_rampa = [](const std::string &s) {
    Rampa R;
    size_t i = 0;
    while (i < s.size()) {
      size_t j = s.find('\n', i);
      if (j == std::string::npos) j = s.size();
      std::string l = s.substr(i, j - i);
      i = j + 1;
      int a, b, c;
      if (l.rfind("OK G", 0) == 0) R.ok = 1;
      else if (l.rfind("E ", 0) == 0) R.error = l;
      else if (sscanf(l.c_str(), "G DATOS %d %d %d", &a, &b, &c) == 3) { R.ch = a; R.nM = b; R.nC = c; }
      else if (sscanf(l.c_str(), "M %d %d", &a, &b) == 2) { R.lm++; R.mvMin = std::min(R.mvMin, b); R.tMax = std::max(R.tMax, a); }
      else if (sscanf(l.c_str(), "C %d %d", &a, &b) == 2) { R.lc++; R.usMin = std::min(R.usMin, b); R.usMax = std::max(R.usMax, b); }
      else if (l.rfind("G FIN", 0) == 0) R.fin = 1;
    }
    return R;
  };
  // FL: 0.2 s + (0.5 s ida + 0.2 s + 0.5 s vuelta + 0.2 s) = 1.6 s
  r = correr("G 0 819 2097 2556 1 200");
  { Rampa R = leer_rampa(r);
    bool ok = R.ok && R.fin && R.ch == 0 && R.nM == R.lm && R.nC == R.lc && R.nM > 750 && R.nM < 830 &&
              R.nC >= 79 && R.nC <= 81 && R.usMin == 819 && R.usMax == 2097 && g_pca_us[0] == 819 &&
              abs(R.mvMin - (int)lround(0.6 * (3000 - 1.404 * 1597))) < 8 && R.tMax > 15800 && R.tMax <= 16000;
    char d[200];
    snprintf(d, sizeof d, "M=%d/%d C=%d/%d us %d-%d mvMin=%d tMax=%d (0.1 ms) %s", R.lm, R.nM, R.lc, R.nC,
             R.usMin, R.usMax, R.mvMin, R.tMax, R.error.c_str());
    revisa(ok, "G en FL (canal 0): datos completos y termina en us_a", d); }
  // Auxiliar, 3 vueltas
  correr("P 7 819");
  r = correr("G 7 819 2097 2556 3 200");
  { Rampa R = leer_rampa(r);
    double total_ms = 200 + 3 * (2 * 500 + 2 * 200);           // 4.4 s
    bool ok = R.ok && R.fin && R.ch == 7 && R.nM == R.lm && R.nC == R.lc && R.nM > total_ms / 2 * 0.9 &&
              R.usMax == 2097 && R.usMin == 819 && g_pca_us[7] == 819;
    char d[200];
    snprintf(d, sizeof d, "M=%d C=%d us %d-%d %s", R.lm, R.lc, R.usMin, R.usMax, R.error.c_str());
    revisa(ok, "G por la auxiliar (canal 7), 3 vueltas", d); }
  r = correr("?");
  revisa(r.find("AUX=ok") != std::string::npos, "la auxiliar sigue bien después de G", sin_nl(r));
  // En el límite: nunca más de 2360
  correr("P 4 1500");
  r = correr("G 4 1500 2360 3000 1 100");
  { Rampa R = leer_rampa(r);
    revisa(R.fin && R.usMax == 2360, "G llega al límite y no pasa de 2360", "usMax=" + std::to_string(R.usMax)); }
  correr("P 4 1500");
  r = correr("G 4 1500 2360 3000 1 0");
  { Rampa R = leer_rampa(r);
    revisa(R.fin && R.usMax <= 2360 && R.usMax > 2300, "G sin espera tampoco pasa de 2360",
           "usMax=" + std::to_string(R.usMax)); }
  // Se detiene si llega algo por USB
  correr("P 3 819");
  salida.clear();
  mandar("G 3 819 2097 639 1 200\nD\n");
  for (int i = 0; i < 300; i++) { principal::loop(); g_us += 200; hook_otra(); }
  revisa(salida.find("E G: detenida") != std::string::npos && salida.find("OK D") != std::string::npos &&
         salida.find("G DATOS") == std::string::npos, "G se detiene si llega un comando y luego lo atiende",
         sin_nl(salida.substr(0, 120)));
  r = correr("G 3 819 2097 639 1 200");
  revisa(r.rfind("E G: primero manda P", 0) == 0, "después de D, G vuelve a pedir P", sin_nl(r));

  // ---------------------------------------------------------------- protocolo v3: S y T
  r = correr("?");
  revisa(r.find("PROTO=3") != std::string::npos, "? dice PROTO=3", sin_nl(r));
  revisa(r.find("RESET=encendido SEG=") != std::string::npos, "? dice por qué arrancó (RESET) y cuánto lleva (SEG)", sin_nl(r));
  auto numeros = [](const std::string &s, size_t desde) {
    std::vector<long> v; const char *p = s.c_str() + desde; char *fin;
    for (;;) { long x = strtol(p, &fin, 10); if (fin == p) break; v.push_back(x); p = fin; }
    return v;
  };
  r = correr("S 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 2000 2500");
  { auto v = numeros(r, 1);
    bool ok = r.rfind("S ", 0) == 0 && v.size() == 12 && g_pca_us[0] == 1000 && g_pca_us[10] == 2000 &&
              g_pca_us[11] == 2360 && abs(v[0] - (int)lround(0.6 * (3000 - 1.404 * 500))) < 8;
    revisa(ok, "S mueve los 12 a la vez, recorta a 2360 y regresa 12 lecturas", sin_nl(r)); }
  r = correr("S -1 -1 0 -1 -1 -1 -1 -1 -1 -1 -1 1500");
  revisa(r.rfind("S ", 0) == 0 && g_pca_us[0] == 1000 && g_pca_suelto[2] && g_pca_us[11] == 1500,
         "S: -1 no cambia y 0 suelta ese canal", sin_nl(r));
  r = correr("S 1500 1500 1500");
  revisa(r.rfind("E S:", 0) == 0, "S con menos de 12 pulsos da error", sin_nl(r));

  // T: canales 0, 1 y 2, 100 puntos (2 s)
  correr("P 0 1000"); correr("P 1 1200"); correr("P 2 1400");
  r = correr("T INICIO 7 100");
  revisa(r == "OK T INICIO 7 3 100\n", "T INICIO con máscara 7", sin_nl(r));
  salida.clear();
  for (int i = 0; i < 100; i++) {
    char l[64];
    snprintf(l, sizeof l, "K %d %d %d\n", 1000 + 5 * i, 1200 - 3 * i, 1400);
    mandar(l);
    for (int k = 0; k < 4; k++) { principal::loop(); g_us += 200; hook_otra(); }
  }
  revisa(salida == "OK K 32\nOK K 64\nOK K 96\nOK K 100\n", "K: confirma cada 32 puntos y el último", sin_nl(salida));
  r = correr("T CORRER");
  { size_t a = r.find("T DATOS");
    bool ok = r.rfind("OK T CORRER 3 100 2000", 0) == 0 && a != std::string::npos && r.find("T FIN") != std::string::npos;
    int nc = 0, nf = 0, np = 0, filasBien = 0, cBien = 0;
    if (ok) sscanf(r.c_str() + a, "T DATOS %d %d %d", &nc, &nf, &np);
    size_t i = 0;
    while (i < r.size()) {
      size_t j = r.find('\n', i); if (j == std::string::npos) j = r.size();
      std::string l = r.substr(i, j - i); i = j + 1;
      if (l.rfind("M ", 0) == 0 && numeros(l, 1).size() == 4) filasBien++;
      if (l.rfind("C ", 0) == 0 && numeros(l, 1).size() == 4) cBien++;
    }
    ok = ok && nc == 3 && np == 100 && nf > 950 && nf < 1010 && filasBien == nf && cBien == 100 &&
         g_pca_us[0] == 1495 && g_pca_us[1] == 903 && g_pca_us[2] == 1400;
    char d[160]; snprintf(d, sizeof d, "canales=%d filas=%d/%d puntos=%d/%d final=%d %d %d", nc, filasBien, nf, cBien, np,
                          g_pca_us[0], g_pca_us[1], g_pca_us[2]);
    revisa(ok, "T CORRER: 3 servos a la vez, datos completos, termina en el último punto", d); }
  r = correr("T CORRER");
  revisa(r.rfind("E T: primero manda P 0 1000", 0) == 0, "T no da saltos (el servo ya no está en el primer punto)", sin_nl(r));
  r = correr("T INICIO 7 5");
  correr("K 1000 1200 1400");
  r = correr("K 1000 2400 1400");
  revisa(r.rfind("E K: punto 1, canal 1: 2400 us", 0) == 0, "K fuera de 500-2360 cancela la trayectoria", sin_nl(r));
  r = correr("T CORRER");
  revisa(r.rfind("E T: faltan puntos", 0) == 0, "después de cancelar no se puede correr", sin_nl(r));

  // Los 12: 8 s no caben grabando; sin grabar sí
  correr("S 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500");
  correr("T INICIO 4095 400");
  salida.clear();
  for (int i = 0; i < 400; i++) {
    std::string l = "K";
    for (int c = 0; c < 12; c++) l += " " + std::to_string(1500 + (i % 50));
    mandar(l + "\n");
    for (int k = 0; k < 4; k++) { principal::loop(); g_us += 200; hook_otra(); }
  }
  revisa(salida.find("OK K 400") != std::string::npos, "K con los 12 canales (líneas largas)", sin_nl(salida.substr(salida.size() > 40 ? salida.size() - 40 : 0)));
  r = correr("T CORRER");
  revisa(r.rfind("E T: muy larga para grabar 12 canales", 0) == 0, "T con 12 canales y 8 s no cabe grabando", sin_nl(r));
  r = correr("T CORRER 0");
  revisa(r.find("T FIN 400") != std::string::npos && g_pca_us[0] == 1549 && g_pca_us[11] == 1549,
         "T CORRER 0: los 12 sin grabar", sin_nl(r));
  // Los 12 grabando, 2 s: los de la auxiliar también llegan
  correr("S 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500");
  correr("T INICIO 4095 100");
  for (int i = 0; i < 100; i++) {
    std::string l = "K";
    for (int c = 0; c < 12; c++) l += " " + std::to_string(1500 + 4 * i);
    mandar(l + "\n");
    for (int k = 0; k < 4; k++) { principal::loop(); g_us += 200; hook_otra(); }
  }
  r = correr("T CORRER");
  { size_t a = r.find("T DATOS");
    int nc = 0, nf = 0, np = 0, sinLectura = 0, filas = 0;
    if (a != std::string::npos) sscanf(r.c_str() + a, "T DATOS %d %d %d", &nc, &nf, &np);
    size_t i = 0;
    while (i < r.size()) {
      size_t j = r.find('\n', i); if (j == std::string::npos) j = r.size();
      std::string l = r.substr(i, j - i); i = j + 1;
      if (l.rfind("M ", 0) == 0) { auto v = numeros(l, 1); filas++; for (size_t k = 1; k < v.size(); k++) if (v[k] < 0) sinLectura++; }
    }
    char d[120]; snprintf(d, sizeof d, "canales=%d filas=%d/%d sin lectura=%d", nc, filas, nf, sinLectura);
    revisa(nc == 12 && filas == nf && nf > 950 && sinLectura == 0 && r.find("T FIN") != std::string::npos,
           "T con los 12 grabando (FL y auxiliar)", d); }
  // T se detiene si llega algo por USB
  correr("P 0 1000"); correr("P 1 1200"); correr("P 2 1400");
  correr("T INICIO 7 50");
  for (int i = 0; i < 50; i++) { mandar("K 1000 1200 1400\n"); for (int k = 0; k < 4; k++) { principal::loop(); g_us += 200; hook_otra(); } }
  salida.clear();
  mandar("T CORRER\nD\n");
  for (int i = 0; i < 300; i++) { principal::loop(); g_us += 200; hook_otra(); }
  revisa(salida.find("E T: detenida") != std::string::npos && salida.find("OK D") != std::string::npos &&
         salida.find("T DATOS") == std::string::npos, "T se detiene si llega un comando y luego lo atiende",
         sin_nl(salida.substr(0, 110)));
  printf("\n%s (%d fallas)\n", fallas ? "HAY FALLAS" : "TODO BIEN", fallas);
  return fallas ? 1 : 0;
}
