# Hardware de Arca9

## Partes

| Parte | Detalle |
|---|---|
| Servos | 12 × FEETECH FS5115M-FB (Pololu 3443). 15.5 kg·cm y 300 °/s a 6 V; picos de más de 3 A cada uno |
| PCA9685 | 16 canales PWM a 50 Hz, I²C en la dirección 0x40 |
| ESP32 principal | ESP32 DevKit. Maneja el PCA9685, lee los verdes de FL y se conecta a la compu por USB |
| ESP32 auxiliar | ESP32 DevKit. Lee los verdes de FR, RL y RR |
| IMU | MPU6050 en 0x68, en el mismo bus I²C (todavía sin usar) |
| Fuente de servos | 5.99 V en la prueba 01 |

## Canales de los servos

| Pata | Abducción | Cadera | Rodilla |
|---|---|---|---|
| FL (delantera izquierda) | 0 | 1 | 2 |
| FR (delantera derecha) | 3 | 4 | 5 |
| RL (trasera izquierda) | 6 | 7 | 8 |
| RR (trasera derecha) | 9 | 10 | 11 |

## Conexiones

### ESP32 principal

| Pin | Va a |
|---|---|
| USB | Compu |
| GPIO 21 (SDA) | SDA del PCA9685 y de la IMU |
| GPIO 22 (SCL) | SCL del PCA9685 y de la IMU |
| 3V3 | VCC del PCA9685 y de la IMU |
| GPIO 25 | OE del PCA9685 (opcional; en LOW las salidas están encendidas) |
| GPIO 32 · 33 · 35 | Verde de FL abducción · cadera · rodilla, cada uno con su divisor |
| GPIO 17 (TX2) | GPIO 16 (RX2) de la auxiliar |
| GPIO 16 (RX2) | GPIO 17 (TX2) de la auxiliar |
| VIN | VIN de la auxiliar (o la auxiliar con su propio USB, no los dos) |
| GND | Tierra común |

### ESP32 auxiliar

| Pin | Va a |
|---|---|
| GPIO 32 · 33 · 35 | Verde de FR abducción · cadera · rodilla |
| GPIO 34 · 36 · 39 | Verde de RL abducción · cadera · rodilla |
| GPIO 25 · 26 · 27 | Verde de RR abducción · cadera · rodilla (ADC2: funciona porque no se usa WiFi) |
| GPIO 17 (TX2) / 16 (RX2) | Cruzados con la principal |
| GND | Tierra común |

### PCA9685

| Terminal | Va a |
|---|---|
| VCC · GND | 3V3 y GND de la principal (VCC alimenta solo el chip) |
| SDA · SCL | GPIO 21 y 22 de la principal |
| OE | GPIO 25 de la principal (opcional) |
| V+ (bornera azul) | Fuente de 6 V de los servos, con su GND |
| Canales 0–11 | Señal de cada servo: café a GND, rojo a V+, naranja a PWM |

**Tierra común:** la fuente de los servos, el PCA9685 y las dos ESP32 comparten GND.

## Divisor de cada cable verde

```
verde ──[100 kΩ]──┬── GPIO
                  ├──[150 kΩ]── GND
                  └──[10 nF]─── GND
```

| Cálculo | Resultado |
|---|---|
| Fracción que llega al pin: 150 / (100 + 150) | 0.60 |
| 0° (cable a ~3.1 V) | 1.86 V en el pin (medido: 1.84–1.89 V) |
| 168° (cable a ~0.31 V) | 0.19 V en el pin (medido: 0.18–0.19 V) |
| Lo que el ADC del ESP32 lee bien (11 dB) | 0.15–2.45 V |
| Resistencia que ve el pin: 100 k ∥ 150 k | 60 kΩ |
| Filtro: 60 kΩ × 10 nF | 0.6 ms (corte en 265 Hz) |
| Corriente que toma del cursor: 3.1 V / 250 kΩ | 12 µA |

- Las resistencias y el capacitor van pegados a la ESP32.
- La tierra del divisor va al GND de esa misma ESP32.
- El firmware reporta los mV del pin. La calibración los convierte a grados, así que no hace
  falta corregirlos por el divisor.

## Límites

- **Pulso:** de 500 a 2360 µs en todos los canales (`US_MAX_CH`). Los 12 servos llegan a su
  tope en ~2367 µs (168°), según la [prueba 01](bitacora/01_eficiencia_servos.md).
- **Convención de ángulo:** 500 µs = 0°, 1500 µs = 90°, 2500 µs = 180°, o sea 11.11 µs por grado.

## Protocolo de la ESP32 principal

USB a 115200 baud, una línea por comando. Sirven "New Line" y "Both NL & CR".

| Comando | Respuesta |
|---|---|
| `P canal us` | `OK P canal us`, con el pulso que sí se aplicó |
| `R canal n` | `R canal n media_mV sigma_mV min_mV max_mV`: n lecturas, 0.5 ms entre ellas |
| `D` | `OK D`: suelta todos los servos |
| `V` | `V mv0 … mv11`: una lectura de los 12 (-1 = sin lectura) |
| `?` | `OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0` |

Entre las ESP32 (921600 baud), la auxiliar manda `A v0 … v8 c` cada 2 ms, con
c = (v0 + … + v8) % 1000, y contesta `R k n` con `R k n media sigma min max`.

## Errores que ya pasaron

| Síntoma | Causa |
|---|---|
| `PCA=NO` | SDA y SCL invertidos, o 3V3 conectado a V+ en lugar de VCC |
| `FB=3 AUX=NO` y `E comando desconocido` cada segundo | Los cables entre las ESP32 estaban en TX0/RX0 (GPIO 1 y 3) en lugar de 16 y 17 |
| El servo zumba en 2500 µs | El servo ya llegó a su tope: no se le debe mandar más de 2360 µs |
| Arduino: `No space left on device` o `Failed to extract python313.dll` | Disco C: lleno; borra `%LOCALAPPDATA%\Temp\arduino` |
| `no se pudo abrir COM6` | El Monitor Serie de Arduino sigue abierto |
