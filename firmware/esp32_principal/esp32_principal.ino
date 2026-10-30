/*
  Eficiencia de los 12 servos con retroalimentación — ESP32 PRINCIPAL
  -------------------------------------------------------------------
  Va junto con esp32_aux/esp32_aux.ino (en la ESP32 auxiliar) y con eficiencia_servos.py
  (en la compu). La compu solo se conecta a esta ESP32, por USB.

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
    ?            responde "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0"
                 (FB = canales con retroalimentación; DESC = líneas de la auxiliar descartadas)

  Los mV son los del pin, después del divisor. La compu los pasa a grados al calibrar, así que
  no importa qué divisor uses mientras el pin quede entre ~0.15 y ~2.45 V.

  Librería: "Adafruit PWM Servo Driver Library". Placa: ESP32 Dev Module.
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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

char linea[48];                        // comando de la compu
int largo = 0;

int auxMv[N_AUX];                      // última lectura de cada verde de la auxiliar
unsigned long auxT = 0;
bool auxVisto = false;
unsigned long auxDescartadas = 0;
char auxBuf[160];
int auxLargo = 0;

// ------------------------------------------------------------------ servos

void soltarTodos() {
  if (!pcaOk) return;
  for (int i = 0; i < N; i++) pca.setPWM(i, 0, 4096);    // salida apagada: servo sin fuerza
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

void mandarV() {
  auxPoll();
  bool ok = auxOk();
  Serial.print("V");
  for (int i = 0; i < N_SERVOS; i++) {
    int v = -1;
    if (FB_LOCAL[i] >= 0) v = medianaLocal(FB_LOCAL[i]);
    else if (FB_AUX[i] >= 0 && ok) v = auxMv[FB_AUX[i]];
    Serial.printf(" %d", v);
  }
  Serial.println();
}

void mandarEstado() {
  auxPoll();
  Serial.printf("OK1-RETRO listo PCA=%s FB=%d AUX=%s DESC=%lu\n", pcaOk ? "ok" : "NO",
                canalesConRetro(), auxOk() ? "ok" : "NO", auxDescartadas);
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
  } else if (c == '?') {
    mandarEstado();
  } else if (c != 0) {
    Serial.println("E comando desconocido");
  }
}

void setup() {
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
