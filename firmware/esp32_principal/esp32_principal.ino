/*
  Arca9 — ESP32 PRINCIPAL (protocolo v3)
  --------------------------------------
  Va junto con esp32_aux/esp32_aux.ino (en la ESP32 auxiliar) y con los scripts de pc/
  (eficiencia_servos.py, angulo_real.py, ruido.py, rampas.py). La compu solo se conecta a esta
  ESP32, por USB.

  Esta ESP32 mueve los 12 servos con el PCA9685 y lee los cables verdes de la pata FL. Los de
  FR, RL y RR los lee la auxiliar y se los manda por UART; aquí se juntan las 12 lecturas.

  Conexiones (las del mapa de cableado):
    PCA9685   SDA -> GPIO 21, SCL -> GPIO 22, VCC -> 3V3, GND -> GND, OE -> GPIO 25 (opcional)
              V+ (bornera azul) -> fuente de 6 V de los servos, con su GND
              canales 0-2 FL, 3-5 FR, 6-8 RL, 9-11 RR (abducción, cadera, rodilla)
    Verde FL  canal 0 -> GPIO 32, canal 1 -> GPIO 33, canal 2 -> GPIO 35, cada uno con divisor:
                 verde ──[100 kΩ]──┬── GPIO
                                   ├──[150 kΩ]── GND
                                   └──[10 nF]─── GND
    Auxiliar  GPIO 17 (TX2) -> GPIO 16 (RX2) de la auxiliar
              GPIO 16 (RX2) <- GPIO 17 (TX2) de la auxiliar
              GND con GND. VIN -> VIN de la auxiliar (o la auxiliar con su USB, no los dos).
    GND común: fuente de los servos, PCA9685 y las dos ESP32.

  Protocolo (115200 baud, una línea por comando; sirve "New Line" o "Both NL & CR"):
    P canal us   mueve el servo a ese pulso (500 a 2360 µs)
                 responde "OK P canal us" con el pulso que sí se aplicó
    R canal n    lee n veces el cable verde de ese canal, con 0.5 ms entre lecturas
                 responde "R canal n media_mV sigma_mV min_mV max_mV"
    D            suelta todos los servos; responde "OK D"
    V            una lectura de los 12 cables verdes: "V mv0 mv1 ... mv11" (-1 = sin lectura)
    ?            responde "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=3 RESET=encendido SEG=12"
                 (RESET = por qué arrancó la última vez; SEG = segundos desde entonces)
                 (FB = canales con retroalimentación; DESC = líneas de la auxiliar descartadas;
                 PROTO = versión del protocolo)

  Protocolo v2 (2 nov 2026): agrega G; P, R, D, V y ? no cambian.
    G canal us_a us_b vel vueltas espera_ms
                 rampa de un servo, generada aquí: espera en us_a, va a us_b a vel µs/s, espera,
                 regresa a us_a, espera; eso `vueltas` veces. El pulso se actualiza cada 20 ms y
                 el cable verde se graba cada ~2 ms (FL: mediana de 5 aquí; FR, RL y RR: cada
                 línea de la auxiliar). Antes hay que mandar "P canal us_a" (no da saltos).
                 Responde "OK G ..." al empezar. Al terminar manda los datos:
                   G DATOS canal n_muestras n_comandos
                   M t mv        n_muestras líneas; t en décimas de ms desde el inicio
                   C t us        n_comandos líneas; el pulso aplicado y cuándo
                   G FIN canal
                 Si llega cualquier cosa por USB mientras corre, se detiene, el servo se queda
                 con el último pulso, responde "E G: detenida ..." y no manda datos.

  Protocolo v3 (3 nov 2026): agrega S y T (varios servos a la vez); lo de v1 y v2 no cambia.
    S us0 us1 ... us11
                 los 12 pulsos a la vez (-1 = no cambiar ese canal, 0 = soltarlo); cada uno pasa
                 por el límite de 2360 µs. Responde "S mv0 ... mv11", las 12 lecturas justo
                 después (igual que V). No hace rampas: quien lo use manda cambios chicos.
    T INICIO mascara n     prepara una trayectoria de n puntos (uno cada 20 ms, hasta 1500 = 30 s)
                           para los canales de la máscara (bit i = canal i; 7 = canales 0, 1 y 2;
                           4095 = los 12). Responde "OK T INICIO mascara n_canales n".
    K us_a us_b ...        un punto: un pulso por canal de la máscara, en orden. Fuera de 500–2360
                           da error y se cancela la trayectoria. Cada 32 puntos (y en el último)
                           responde "OK K cuantos": la compu espera eso antes de seguir mandando.
    T CORRER [grabar]      la ejecuta. El primer punto debe ser el pulso actual de cada canal (no da
                           saltos). Con grabar = 1 (por defecto) graba los cables verdes de esos
                           canales cada ~2 ms y al final manda:
                             T DATOS n_canales n_filas n_puntos canal_a canal_b ...
                             M t mv_a mv_b ...     n_filas líneas; t en décimas de ms
                             C t us_a us_b ...     n_puntos líneas; cuándo se aplicó cada punto
                             T FIN
                           Con grabar = 0 solo responde "T FIN n_puntos" al terminar. Igual que G,
                           cualquier cosa que llegue por USB la detiene y los servos se quedan donde
                           iban. Grabando caben ~16 s con 3 canales y ~5.7 s con los 12.

  Los mV son los del pin, después del divisor. La compu los pasa a grados al calibrar, así que
  no importa qué divisor uses mientras el pin quede entre ~0.15 y ~2.45 V.

  Librería: "Adafruit PWM Servo Driver Library". Placa: ESP32 Dev Module.
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_system.h>

const int N = 16;                      // canales del PCA9685
const int N_SERVOS = 12;
const uint8_t PCA_ADDR = 0x40;
const uint32_t PCA_OSC_HZ = 27000000;  // el oscilador de los PCA genéricos varía (25-27 MHz)
const int PIN_SDA = 21, PIN_SCL = 22;
const int PIN_OE = 25;                 // OE del PCA9685: LOW = salidas encendidas
const int PIN_LED = 2;
const int US_MIN = 500;

// Límite de pulso por canal (µs). En la prueba de eficiencia del 29 oct los 12 FS5115M-FB
// llegaron a su tope en ~168° (2367 µs; tres de ellos en 170°). El límite queda 7 µs antes,
// en 2360 µs (167.4°), para que ningún servo empuje contra su tope (docs/bitacora/01).
int US_MAX_CH[N] = {2360, 2360, 2360, 2360, 2360, 2360, 2360, 2360,
                    2360, 2360, 2360, 2360, 2360, 2360, 2360, 2360};

// Cable verde de cada servo: pin de esta ESP32 (pata FL) o índice en la auxiliar (FR, RL, RR).
const int FB_LOCAL[N_SERVOS] = {32, 33, 35,  -1, -1, -1,  -1, -1, -1,  -1, -1, -1};
const int FB_AUX[N_SERVOS]   = {-1, -1, -1,   0,  1,  2,   3,  4,  5,   6,  7,  8};
const int N_AUX = 9;
const int PIN_AUX_RX = 16, PIN_AUX_TX = 17;
const long AUX_BAUD = 921600;
const unsigned long AUX_VIEJO_MS = 100;   // sin datos de la auxiliar por más de esto = desconectada

Adafruit_PWMServoDriver pca(PCA_ADDR);
bool pcaOk = false;
int ultimoUs[N] = {0};                 // último pulso aplicado a cada canal (0 = suelto)

char linea[128];                       // comando de la compu (S y K llevan hasta 12 números)
int largo = 0;

int auxMv[N_AUX];                      // última lectura de cada verde de la auxiliar
unsigned long auxT = 0;
unsigned long auxTus = 0;              // micros() de la última línea buena (para las rampas)
bool auxNuevo = false;                 // llegó una línea buena que la rampa no ha grabado
bool auxVisto = false;
unsigned long auxDescartadas = 0;
char auxBuf[160];
int auxLargo = 0;

// ------------------------------------------------------------------ servos

void soltarTodos() {
  if (!pcaOk) return;
  for (int i = 0; i < N; i++) {
    pca.setPWM(i, 0, 4096);                                // salida apagada: servo sin fuerza
    ultimoUs[i] = 0;
  }
}

// ------------------------------------------------------------------ auxiliar

bool auxOk() { return auxVisto && (millis() - auxT) < AUX_VIEJO_MS; }

// "A v0 v1 ... v8 c", con c = (v0 + ... + v8) % 1000
void auxProcesarA(const char *s) {
  int v[N_AUX];
  const char *p = s + 1;
  char *fin;
  long suma = 0;
  for (int i = 0; i < N_AUX; i++) {
    long x = strtol(p, &fin, 10);
    if (fin == p) { auxDescartadas++; return; }
    v[i] = (int)x;
    suma += x;
    p = fin;
  }
  long c = strtol(p, &fin, 10);
  if (fin == p || c != suma % 1000) { auxDescartadas++; return; }
  for (int i = 0; i < N_AUX; i++) auxMv[i] = v[i];
  auxT = millis();
  auxTus = micros();
  auxNuevo = true;
  auxVisto = true;
}

// Junta los bytes que manda la auxiliar. Regresa true cuando hay una línea completa en `out`.
bool auxSiguienteLinea(char *out, size_t len) {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      auxBuf[auxLargo] = 0;
      bool hay = auxLargo > 0;
      if (hay) { strncpy(out, auxBuf, len - 1); out[len - 1] = 0; }
      auxLargo = 0;
      if (hay) return true;
      continue;
    }
    if (auxLargo < (int)sizeof(auxBuf) - 1) auxBuf[auxLargo++] = c;
    else { auxLargo = 0; auxDescartadas++; }                // línea demasiado larga: basura
  }
  return false;
}

void auxPoll() {
  char l[160];
  while (auxSiguienteLinea(l, sizeof l))
    if (l[0] == 'A') auxProcesarA(l);
}

// Le pide a la auxiliar "R k n" y espera su respuesta (mientras, sigue tomando las líneas A).
bool auxPedirR(int k, int n, char *resp, size_t len) {
  Serial2.printf("R %d %d\n", k, n);
  unsigned long t0 = millis(), lim = 300 + (unsigned long)n * 2;
  char l[160];
  while (millis() - t0 < lim) {
    if (auxSiguienteLinea(l, sizeof l)) {
      if (l[0] == 'R') {
        int kk = -1;
        if (sscanf(l + 1, "%d", &kk) == 1 && kk == k) {
          strncpy(resp, l, len - 1);
          resp[len - 1] = 0;
          return true;
        }
      } else if (l[0] == 'A') {
        auxProcesarA(l);
      } else if (l[0] == 'E') {
        return false;
      }
    } else {
      delayMicroseconds(50);
    }
  }
  return false;
}

// ------------------------------------------------------------------ lecturas

int canalesConRetro() {
  int n = 0;
  bool ok = auxOk();
  for (int i = 0; i < N_SERVOS; i++)
    if (FB_LOCAL[i] >= 0 || (FB_AUX[i] >= 0 && ok)) n++;
  return n;
}

int medianaLocal(int pin) {
  int v[5];
  for (int i = 0; i < 5; i++) v[i] = analogReadMilliVolts(pin);
  for (int i = 1; i < 5; i++)                              // ordenar 5 valores
    for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { int t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
  return v[2];
}

// n lecturas del pin, 0.5 ms entre ellas: media, desviación estándar, mínimo y máximo (mV)
void leerLocal(int ch, int n) {
  int pin = FB_LOCAL[ch];
  double suma = 0, suma2 = 0;
  int mn = 100000, mx = -1;
  for (int k = 0; k < n; k++) {
    int v = analogReadMilliVolts(pin);
    suma += v;
    suma2 += (double)v * v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    delayMicroseconds(500);
  }
  double media = suma / n;
  double var = suma2 / n - media * media;
  Serial.printf("R %d %d %.1f %.2f %d %d\n", ch, n, media, var > 0 ? sqrt(var) : 0.0, mn, mx);
}

void leerAux(int ch, int n) {
  auxPoll();
  if (!auxOk()) {
    Serial.println("E R: la auxiliar no manda datos (revisa TX/RX cruzados, GND y que tenga esp32_aux.ino)");
    return;
  }
  char r[160];
  if (!auxPedirR(FB_AUX[ch], n, r, sizeof r)) {
    Serial.println("E R: la auxiliar no contesto");
    return;
  }
  int k, nn, mn, mx;
  float media, sigma;
  if (sscanf(r, "R %d %d %f %f %d %d", &k, &nn, &media, &sigma, &mn, &mx) != 6) {
    Serial.println("E R: respuesta rara de la auxiliar");
    return;
  }
  Serial.printf("R %d %d %.1f %.2f %d %d\n", ch, nn, media, sigma, mn, mx);
}

void lecturas(const char *prefijo) {      // "V mv0 ... mv11" (o "S ...")
  auxPoll();
  bool ok = auxOk();
  Serial.print(prefijo);
  for (int i = 0; i < N_SERVOS; i++) {
    int v = -1;
    if (FB_LOCAL[i] >= 0) v = medianaLocal(FB_LOCAL[i]);
    else if (FB_AUX[i] >= 0 && ok) v = auxMv[FB_AUX[i]];
    Serial.printf(" %d", v);
  }
  Serial.println();
}

void mandarV() { lecturas("V"); }

// Por qué arrancó la ESP32. "encendido" también es al abrir el puerto USB o apretar EN;
// "voltaje_bajo" = el brownout la reinició (la alimentación cayó, casi siempre por los servos).
const char *razonReinicio() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "encendido";
    case ESP_RST_BROWNOUT: return "voltaje_bajo";
    case ESP_RST_PANIC: return "falla";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_SW: return "software";
    case ESP_RST_EXT: return "externo";
    default: return "otro";
  }
}

void mandarEstado() {
  auxPoll();
  Serial.printf("OK1-RETRO listo PCA=%s FB=%d AUX=%s DESC=%lu PROTO=3 RESET=%s SEG=%lu\n", pcaOk ? "ok" : "NO",
                canalesConRetro(), auxOk() ? "ok" : "NO", auxDescartadas, razonReinicio(), millis() / 1000);
}

// ------------------------------------------------------------------ rampas (G, protocolo v2)

const unsigned long RAMPA_DT_CMD_US = 20000;    // el servo recibe un pulso cada 20 ms (50 Hz)
const unsigned long RAMPA_DT_LOCAL_US = 2000;   // FL se lee cada 2 ms, como manda la auxiliar
const int RAMPA_MAX_MUESTRAS = 9000;            // 18 s a 2 ms
const int RAMPA_MAX_CMD = 1000;                 // 20 s a 20 ms
const long RAMPA_VEL_MIN = 50, RAMPA_VEL_MAX = 4000;   // µs/s (~5 a ~375 °/s)

struct MuestraRampa { uint32_t t; int16_t mv; };   // t en µs desde el inicio de la rampa
struct CmdRampa { uint32_t t; int16_t us; };
MuestraRampa *rampaM = nullptr;
CmdRampa *rampaC = nullptr;

// Memoria de grabación: se pide una sola vez y la comparten G y T (80 kB).
const size_t GRAB_BYTES = sizeof(MuestraRampa) * RAMPA_MAX_MUESTRAS + sizeof(CmdRampa) * RAMPA_MAX_CMD;
uint8_t *grab = nullptr;
bool pedirGrabacion() {
  if (!grab) grab = (uint8_t *)malloc(GRAB_BYTES);
  return grab != nullptr;
}

// Pulso que toca en el instante t (µs desde el inicio): espera en a, sube a b, espera, baja a a,
// espera; `vueltas` veces.
double rampaPulso(double t, double a, double b, double dur, double espera, int vueltas) {
  if (t < espera) return a;
  t -= espera;
  double periodo = 2 * dur + 2 * espera;
  long k = (long)(t / periodo);
  if (k >= vueltas) return a;
  double r = t - k * periodo;
  if (r < dur) return a + (b - a) * r / dur;
  r -= dur;
  if (r < espera) return b;
  r -= espera;
  if (r < dur) return b + (a - b) * r / dur;
  return a;
}

void rampa(char *p) {
  long v[6];
  char *fin;
  for (int i = 0; i < 6; i++) {
    v[i] = strtol(p, &fin, 10);
    if (fin == p) { Serial.println("E G: uso G canal us_a us_b vel_us_s vueltas espera_ms"); return; }
    p = fin;
  }
  int ch = (int)v[0];
  long a = v[1], b = v[2], vel = v[3], vueltas = v[4], esperaMs = v[5];
  if (ch < 0 || ch >= N_SERVOS) { Serial.println("E G: canal (0-11)"); return; }
  if (a < US_MIN || a > US_MAX_CH[ch] || b < US_MIN || b > US_MAX_CH[ch] || a == b) {
    Serial.printf("E G: us_a y us_b distintos y entre %d y %d\n", US_MIN, US_MAX_CH[ch]);
    return;
  }
  if (vel < RAMPA_VEL_MIN || vel > RAMPA_VEL_MAX) {
    Serial.printf("E G: vel entre %ld y %ld us/s\n", RAMPA_VEL_MIN, RAMPA_VEL_MAX);
    return;
  }
  if (vueltas < 1 || vueltas > 10 || esperaMs < 0 || esperaMs > 5000) {
    Serial.println("E G: vueltas 1-10 y espera_ms 0-5000");
    return;
  }
  if (!pcaOk) { Serial.println("E G: sin PCA9685"); return; }
  bool local = FB_LOCAL[ch] >= 0;
  auxPoll();
  if (!local && !auxOk()) { Serial.println("E G: la auxiliar no manda datos"); return; }
  if (ultimoUs[ch] != a) {                                  // nada de saltos: debe estar ya en a
    Serial.printf("E G: primero manda P %d %ld (el canal esta en %d)\n", ch, a, ultimoUs[ch]);
    return;
  }
  double dur = (double)labs(b - a) * 1e6 / vel;              // µs de cada rampa
  double espera = esperaMs * 1000.0;
  double total = espera + vueltas * (2 * dur + 2 * espera);
  long nEst = (long)(total / RAMPA_DT_LOCAL_US) + 200;       // + margen por la auxiliar
  long cEst = (long)(total / RAMPA_DT_CMD_US) + 5;
  if (nEst > RAMPA_MAX_MUESTRAS || cEst > RAMPA_MAX_CMD) {
    Serial.printf("E G: muy larga (%.1f s; maximo ~%.0f s)\n", total / 1e6,
                  (RAMPA_MAX_MUESTRAS - 200) * RAMPA_DT_LOCAL_US / 1e6);
    return;
  }
  if (!pedirGrabacion()) { Serial.println("E G: sin memoria"); return; }
  rampaM = (MuestraRampa *)grab;
  rampaC = (CmdRampa *)(grab + sizeof(MuestraRampa) * RAMPA_MAX_MUESTRAS);

  Serial.printf("OK G %d %ld %ld %ld %ld %ld %.0f\n", ch, a, b, vel, vueltas, esperaMs, total / 1000.0);
  int nM = 0, nC = 0;
  bool detenida = false;
  const char *razon = "";
  unsigned long t0 = micros();
  unsigned long sigCmd = 0, sigLocal = 0;
  auxNuevo = false;
  while (true) {
    unsigned long t = micros() - t0;
    if (t >= (unsigned long)total) break;
    if (Serial.available()) { detenida = true; razon = "llego un comando por USB"; break; }
    if (t >= sigCmd) {
      long us = lround(rampaPulso(t, a, b, dur, espera, (int)vueltas));
      us = constrain(us, (long)US_MIN, (long)US_MAX_CH[ch]);  // el límite duro manda siempre
      pca.writeMicroseconds(ch, (int)us);
      ultimoUs[ch] = (int)us;
      if (nC < RAMPA_MAX_CMD) rampaC[nC++] = {(uint32_t)t, (int16_t)us};
      sigCmd += RAMPA_DT_CMD_US;
      if (!local && !auxOk()) { detenida = true; razon = "la auxiliar dejo de mandar datos"; break; }
    }
    if (local) {
      if (t >= sigLocal) {
        int mv = medianaLocal(FB_LOCAL[ch]);
        if (nM < RAMPA_MAX_MUESTRAS) rampaM[nM++] = {(uint32_t)(micros() - t0), (int16_t)mv};
        sigLocal += RAMPA_DT_LOCAL_US;
      }
      auxPoll();                                             // que no se llene el buffer
    } else {
      auxPoll();
      if (auxNuevo) {
        auxNuevo = false;
        if (nM < RAMPA_MAX_MUESTRAS) rampaM[nM++] = {(uint32_t)(auxTus - t0), (int16_t)auxMv[FB_AUX[ch]]};
      }
    }
  }
  if (detenida) {
    Serial.printf("E G: detenida (%s); el canal %d se queda en %d us\n", razon, ch, ultimoUs[ch]);
    return;
  }
  pca.writeMicroseconds(ch, (int)a);                         // termina exacto en a
  ultimoUs[ch] = (int)a;

  Serial.printf("G DATOS %d %d %d\n", ch, nM, nC);
  for (int i = 0; i < nM; i++) {
    Serial.printf("M %lu %d\n", (unsigned long)(rampaM[i].t / 100), rampaM[i].mv);
    if ((i & 63) == 0) auxPoll();                            // mientras tanto, vaciar la auxiliar
  }
  for (int i = 0; i < nC; i++) Serial.printf("C %lu %d\n", (unsigned long)(rampaC[i].t / 100), rampaC[i].us);
  Serial.printf("G FIN %d\n", ch);
}

// ------------------------------------------------------------------ varios servos a la vez (S y T, v3)

const int TRAY_MAX_PUNTOS = 1500;               // un punto cada 20 ms: hasta 30 s
const unsigned long TRAY_DT_US = 20000;
const unsigned long TRAY_DT_MUESTRA_US = 2000;
int16_t *trayUs = nullptr;                      // TRAY_MAX_PUNTOS x N_SERVOS pulsos
uint32_t *trayTc = nullptr;                     // cuándo se aplicó cada punto (µs)
int trayCh[N_SERVOS];
int trayNch = 0, trayN = 0, trayCargados = 0;

// S us0 ... us11: los 12 a la vez (-1 = no cambiar, 0 = soltar). Responde con las 12 lecturas.
void ponerTodos(char *p) {
  long v[N_SERVOS];
  char *fin;
  for (int i = 0; i < N_SERVOS; i++) {
    v[i] = strtol(p, &fin, 10);
    if (fin == p || v[i] < -1) {
      Serial.println("E S: son 12 pulsos (us0 ... us11; -1 = no cambiar, 0 = soltar)");
      return;
    }
    p = fin;
  }
  if (!pcaOk) { Serial.println("E S: sin PCA9685"); return; }
  for (int i = 0; i < N_SERVOS; i++) {
    if (v[i] == -1) continue;
    if (v[i] == 0) { pca.setPWM(i, 0, 4096); ultimoUs[i] = 0; continue; }
    long us = constrain(v[i], (long)US_MIN, (long)US_MAX_CH[i]);   // el límite duro manda siempre
    pca.writeMicroseconds(i, (int)us);
    ultimoUs[i] = (int)us;
  }
  lecturas("S");
}

void trayInicio(char *p) {
  char *fin;
  long mascara = strtol(p, &fin, 10);
  bool ok = fin != p;
  p = fin;
  long n = strtol(p, &fin, 10);
  ok = ok && fin != p;
  trayN = 0;
  if (!ok || mascara <= 0 || mascara >= (1L << N_SERVOS) || n < 1 || n > TRAY_MAX_PUNTOS) {
    Serial.printf("E T: usa T INICIO mascara n (mascara 1-4095, n 1-%d)\n", TRAY_MAX_PUNTOS);
    return;
  }
  if (!trayUs) trayUs = (int16_t *)malloc(sizeof(int16_t) * TRAY_MAX_PUNTOS * N_SERVOS);
  if (!trayTc) trayTc = (uint32_t *)malloc(sizeof(uint32_t) * TRAY_MAX_PUNTOS);
  if (!trayUs || !trayTc) { Serial.println("E T: sin memoria"); return; }
  trayNch = 0;
  for (int i = 0; i < N_SERVOS; i++)
    if (mascara & (1L << i)) trayCh[trayNch++] = i;
  trayN = (int)n;
  trayCargados = 0;
  Serial.printf("OK T INICIO %ld %d %d\n", mascara, trayNch, trayN);
}

void trayPunto(char *p) {
  if (trayN == 0 || trayCargados >= trayN) { Serial.println("E K: primero T INICIO (o ya estan todos los puntos)"); return; }
  int16_t *fila = trayUs + (size_t)trayCargados * N_SERVOS;
  char *fin;
  for (int j = 0; j < trayNch; j++) {
    long us = strtol(p, &fin, 10);
    int ch = trayCh[j];
    if (fin == p) {
      Serial.printf("E K: el punto %d necesita %d pulsos; se cancela la trayectoria\n", trayCargados, trayNch);
      trayN = 0;
      return;
    }
    if (us < US_MIN || us > US_MAX_CH[ch]) {          // en una trayectoria, fuera de rango es un error
      Serial.printf("E K: punto %d, canal %d: %ld us fuera de %d-%d; se cancela la trayectoria\n",
                    trayCargados, ch, us, US_MIN, US_MAX_CH[ch]);
      trayN = 0;
      return;
    }
    fila[j] = (int16_t)us;
    p = fin;
  }
  trayCargados++;
  if (trayCargados % 32 == 0 || trayCargados == trayN) Serial.printf("OK K %d\n", trayCargados);
}

void trayCorrer(char *p) {
  char *fin;
  long g = strtol(p, &fin, 10);
  bool grabar = (fin == p) ? true : (g != 0);
  if (trayN == 0 || trayCargados != trayN) {
    Serial.printf("E T: faltan puntos (%d de %d)\n", trayCargados, trayN);
    return;
  }
  if (!pcaOk) { Serial.println("E T: sin PCA9685"); return; }
  bool usaAux = false;
  for (int j = 0; j < trayNch; j++)
    if (FB_AUX[trayCh[j]] >= 0) usaAux = true;
  auxPoll();
  if (usaAux && !auxOk()) { Serial.println("E T: la auxiliar no manda datos"); return; }
  for (int j = 0; j < trayNch; j++) {                      // nada de saltos
    int ch = trayCh[j];
    if (ultimoUs[ch] != trayUs[j]) {
      Serial.printf("E T: primero manda P %d %d (el canal esta en %d)\n", ch, trayUs[j], ultimoUs[ch]);
      return;
    }
  }
  unsigned long total = (unsigned long)trayN * TRAY_DT_US;
  size_t filaBytes = 4 + 2 * (size_t)trayNch;
  long maxFilas = 0;
  if (grabar) {
    if (!pedirGrabacion()) { Serial.println("E T: sin memoria"); return; }
    maxFilas = (long)(GRAB_BYTES / filaBytes);
    long necesarias = (long)(total / TRAY_DT_MUESTRA_US) + 10;
    if (necesarias > maxFilas) {
      Serial.printf("E T: muy larga para grabar %d canales (maximo ~%.1f s; o T CORRER 0 sin grabar)\n",
                    trayNch, (maxFilas - 10) * TRAY_DT_MUESTRA_US / 1e6);
      return;
    }
  }
  Serial.printf("OK T CORRER %d %d %lu\n", trayNch, trayN, total / 1000);
  long nF = 0;
  int k = 0;
  bool detenida = false;
  const char *razon = "";
  unsigned long t0 = micros(), sigM = 0;
  while (true) {
    unsigned long t = micros() - t0;
    if (t >= total) break;
    if (Serial.available()) { detenida = true; razon = "llego un comando por USB"; break; }
    if (k < trayN && t >= (unsigned long)k * TRAY_DT_US) {
      int16_t *fila = trayUs + (size_t)k * N_SERVOS;
      for (int j = 0; j < trayNch; j++) {
        pca.writeMicroseconds(trayCh[j], fila[j]);
        ultimoUs[trayCh[j]] = fila[j];
      }
      trayTc[k++] = (uint32_t)t;
      if (usaAux && !auxOk()) { detenida = true; razon = "la auxiliar dejo de mandar datos"; break; }
    }
    if (grabar && t >= sigM) {
      if (nF < maxFilas) {
        uint8_t *q = grab + (size_t)nF * filaBytes;
        uint32_t tm = (uint32_t)(micros() - t0);
        memcpy(q, &tm, 4);                                  // memcpy: sin accesos desalineados
        bool auxBien = auxOk();
        for (int j = 0; j < trayNch; j++) {
          int ch = trayCh[j];
          int16_t mv = FB_LOCAL[ch] >= 0 ? (int16_t)medianaLocal(FB_LOCAL[ch])
                                         : (int16_t)(auxBien ? auxMv[FB_AUX[ch]] : -1);
          memcpy(q + 4 + 2 * j, &mv, 2);
        }
        nF++;
      }
      sigM += TRAY_DT_MUESTRA_US;
    }
    auxPoll();
  }
  if (detenida) {
    Serial.printf("E T: detenida en el punto %d de %d (%s); los servos se quedan donde iban\n", k, trayN, razon);
    return;
  }
  if (!grabar) { Serial.printf("T FIN %d\n", k); return; }

  Serial.printf("T DATOS %d %ld %d", trayNch, nF, trayN);
  for (int j = 0; j < trayNch; j++) Serial.printf(" %d", trayCh[j]);
  Serial.println();
  for (long i = 0; i < nF; i++) {
    uint8_t *q = grab + (size_t)i * filaBytes;
    uint32_t tm;
    memcpy(&tm, q, 4);
    Serial.printf("M %lu", (unsigned long)(tm / 100));
    for (int j = 0; j < trayNch; j++) {
      int16_t mv;
      memcpy(&mv, q + 4 + 2 * j, 2);
      Serial.printf(" %d", mv);
    }
    Serial.println();
    if ((i & 63) == 0) auxPoll();
  }
  for (int i = 0; i < trayN; i++) {
    Serial.printf("C %lu", (unsigned long)(trayTc[i] / 100));
    int16_t *fila = trayUs + (size_t)i * N_SERVOS;
    for (int j = 0; j < trayNch; j++) Serial.printf(" %d", fila[j]);
    Serial.println();
    if ((i & 63) == 0) auxPoll();
  }
  Serial.println("T FIN");
}

// ------------------------------------------------------------------ comandos

void atender(char *s) {
  char c = s[0];
  char *p = s + 1, *fin;
  if (c == 'P') {
    long ch = strtol(p, &fin, 10);
    if (fin == p || ch < 0 || ch >= N) { Serial.println("E P: canal"); return; }
    p = fin;
    long us = strtol(p, &fin, 10);
    if (fin == p) { Serial.println("E P: falta us"); return; }
    us = constrain(us, (long)US_MIN, (long)US_MAX_CH[ch]);
    if (pcaOk) pca.writeMicroseconds((int)ch, (int)us);
    ultimoUs[ch] = (int)us;
    Serial.printf("OK P %ld %ld\n", ch, us);              // pulso que sí se aplicó
  } else if (c == 'R') {
    long ch = strtol(p, &fin, 10);
    if (fin == p || ch < 0 || ch >= N_SERVOS) { Serial.println("E R: canal sin retroalimentacion"); return; }
    p = fin;
    long n = strtol(p, &fin, 10);
    if (fin == p) n = 64;
    n = constrain(n, 1L, 1000L);
    if (FB_LOCAL[ch] >= 0) leerLocal((int)ch, (int)n);
    else if (FB_AUX[ch] >= 0) leerAux((int)ch, (int)n);
    else Serial.println("E R: canal sin retroalimentacion");
  } else if (c == 'D') {
    soltarTodos();
    Serial.println("OK D");
  } else if (c == 'V') {
    mandarV();
  } else if (c == 'G') {
    rampa(p);
  } else if (c == 'S') {
    ponerTodos(p);
  } else if (c == 'K') {
    trayPunto(p);
  } else if (c == 'T') {
    while (*p == ' ') p++;
    if (strncmp(p, "INICIO", 6) == 0) trayInicio(p + 6);
    else if (strncmp(p, "CORRER", 6) == 0) trayCorrer(p + 6);
    else Serial.println("E T: usa T INICIO mascara n, luego K ..., luego T CORRER");
  } else if (c == '?') {
    mandarEstado();
  } else if (c != 0) {
    Serial.println("E comando desconocido");
  }
}

void setup() {
  Serial.setRxBufferSize(1024);                            // caben varias líneas K seguidas
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH);                              // salidas apagadas mientras arranca
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = (Wire.endTransmission() == 0);
  if (pcaOk) {
    pca.begin();
    pca.setOscillatorFrequency(PCA_OSC_HZ);
    pca.setPWMFreq(50);
    soltarTodos();
  }
  digitalWrite(PIN_OE, LOW);                               // salidas encendidas (canales sueltos)
  for (int i = 0; i < N_SERVOS; i++)
    if (FB_LOCAL[i] >= 0) analogSetPinAttenuation(FB_LOCAL[i], ADC_11db);   // ~0.15-2.45 V
  Serial2.setRxBufferSize(4096);
  Serial2.begin(AUX_BAUD, SERIAL_8N1, PIN_AUX_RX, PIN_AUX_TX);
  delay(100);
  mandarEstado();
}

void loop() {
  auxPoll();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linea[largo] = 0;
      atender(linea);
      largo = 0;
    } else if (largo < (int)sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }
  // LED: fijo si la auxiliar manda datos, parpadeando si no
  digitalWrite(PIN_LED, auxOk() ? HIGH : ((millis() / 250) % 2 ? HIGH : LOW));
}
