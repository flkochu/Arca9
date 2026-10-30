/*
  Eficiencia de los 12 servos con retroalimentación — ESP32 AUXILIAR
  ------------------------------------------------------------------
  Lee los cables verdes de las patas FR, RL y RR (9 servos) y se los manda a la ESP32
  principal por UART. La principal (esp32_principal.ino o perro_servos.ino) junta estas
  lecturas con las de la pata FL; la compu solo habla con la principal.

  Conexiones (las del mapa de cableado). Cada verde con su divisor:
      verde ──[100 kΩ]──┬── GPIO
                        ├──[150 kΩ]── GND
                        └──[10 nF]─── GND
    FR (canales 3, 4, 5)   abducción -> GPIO 32, cadera -> GPIO 33, rodilla -> GPIO 35
    RL (canales 6, 7, 8)   abducción -> GPIO 34, cadera -> GPIO 36, rodilla -> GPIO 39
    RR (canales 9, 10, 11) abducción -> GPIO 25, cadera -> GPIO 26, rodilla -> GPIO 27
    UART   GPIO 17 (TX2) -> GPIO 16 (RX2) de la principal
           GPIO 16 (RX2) <- GPIO 17 (TX2) de la principal
    GND con la principal. VIN desde la VIN de la principal, o su propio USB (no los dos).
  (25, 26 y 27 son del ADC2: funcionan porque esta ESP32 no usa WiFi.)

  Lo que manda a la principal (921600 baud):
    "A v0 v1 ... v8 c" cada 2 ms: mediana de 5 lecturas de cada verde (mV del pin), con
    c = (v0 + ... + v8) % 1000 para que la principal descarte líneas dañadas.
    "R k n media sigma min max" cuando la principal le pide "R k n" (k = 0..8 en el orden de
    arriba): n lecturas con 0.5 ms entre ellas.
  Por su USB (115200 baud) escribe cada segundo "AUX mV: FR-abd ... RR-rod ...", para revisar
  sin la principal. El LED parpadea una vez por segundo mientras corre.

  Placa: ESP32 Dev Module. No necesita librerías.
*/

const int N_AUX = 9;
const int PINS[N_AUX] = {32, 33, 35,  34, 36, 39,  25, 26, 27};
const char *NOMBRES[N_AUX] = {"FR-abd", "FR-cad", "FR-rod", "RL-abd", "RL-cad", "RL-rod",
                              "RR-abd", "RR-cad", "RR-rod"};
const int PIN_RX = 16, PIN_TX = 17;
const long BAUD = 921600;
const unsigned long PERIODO_US = 2000;   // 500 líneas por segundo
const int PIN_LED = 2;

int ultimo[N_AUX];
unsigned long tEnvio = 0, tUsb = 0;
bool led = false;
char linea[48];
int largo = 0;

int mediana5(int pin) {
  int v[5];
  for (int i = 0; i < 5; i++) v[i] = analogReadMilliVolts(pin);
  for (int i = 1; i < 5; i++)
    for (int j = i; j > 0 && v[j - 1] > v[j]; j--) { int t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
  return v[2];
}

void responderR(int k, int n) {
  if (k < 0 || k >= N_AUX) { Serial2.printf("E R %d\n", k); return; }
  int pin = PINS[k];
  double suma = 0, suma2 = 0;
  int mn = 100000, mx = -1;
  for (int i = 0; i < n; i++) {
    int v = analogReadMilliVolts(pin);
    suma += v;
    suma2 += (double)v * v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    delayMicroseconds(500);
  }
  double media = suma / n;
  double var = suma2 / n - media * media;
  Serial2.printf("R %d %d %.1f %.2f %d %d\n", k, n, media, var > 0 ? sqrt(var) : 0.0, mn, mx);
}

void atender(char *s) {
  if (s[0] != 'R') return;
  char *p = s + 1, *fin;
  long k = strtol(p, &fin, 10);
  if (fin == p) return;
  p = fin;
  long n = strtol(p, &fin, 10);
  if (fin == p) n = 64;
  responderR((int)k, (int)constrain(n, 1L, 1000L));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  for (int i = 0; i < N_AUX; i++) analogSetPinAttenuation(PINS[i], ADC_11db);   // ~0.15-2.45 V
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  Serial.println("AUX lista: verdes de FR (32 33 35), RL (34 36 39) y RR (25 26 27)");
}

void loop() {
  while (Serial2.available()) {                            // pedidos de la principal
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linea[largo] = 0;
      atender(linea);
      largo = 0;
    } else if (largo < (int)sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }

  unsigned long ahora = micros();
  if (ahora - tEnvio >= PERIODO_US) {
    tEnvio = ahora;
    char buf[96];
    int p = snprintf(buf, sizeof buf, "A");
    long suma = 0;
    for (int i = 0; i < N_AUX; i++) {
      ultimo[i] = mediana5(PINS[i]);
      suma += ultimo[i];
      p += snprintf(buf + p, sizeof buf - p, " %d", ultimo[i]);
    }
    snprintf(buf + p, sizeof buf - p, " %ld\n", suma % 1000);
    Serial2.print(buf);
  }

  if (millis() - tUsb >= 1000) {
    tUsb = millis();
    Serial.print("AUX mV:");
    for (int i = 0; i < N_AUX; i++) Serial.printf("  %s %d", NOMBRES[i], ultimo[i]);
    Serial.println();
    led = !led;
    digitalWrite(PIN_LED, led ? HIGH : LOW);
  }
}
