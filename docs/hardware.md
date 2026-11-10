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
- **Convención de ángulo (pruebas 01 y 02):** 500 µs = 0°, 1500 µs = 90°, 2500 µs = 180°, o sea
  11.11 µs por grado.
- **Grados reales:** según la [prueba 02](bitacora/02_angulo_real.md), el eje gira 1.043 veces el
  ángulo de la convención: **10.65 µs por grado real**. El límite de 2360 µs son 174.6° reales.
  El firmware no cambia (sigue en µs); la cuenta se hace solo en `pc/escala.py`, que lee el factor
  de `datos/2026-11-02_angulo_real/escala.json`. Si se cambia el PCA9685, hay que repetir la
  prueba 02.

## Protocolo de la ESP32 principal (versión 3)

USB a 115200 baud, una línea por comando. Sirven "New Line" y "Both NL & CR".

**Versiones.**
- v1 (29 oct 2026): `P`, `R`, `D`, `V` y `?`.
- **v2 (2 nov 2026): agrega `G`** (rampa de un servo).
- **v3 (3 nov 2026): agrega `S` y `T`** (varios servos a la vez, hasta los 12), y `?` dice
  `PROTO=3`.

Cada versión solo agrega comandos, así que los scripts viejos siguen funcionando. Un script que
necesita un comando nuevo revisa primero el número de `PROTO` en `?`.

**Diagnóstico en `?` (9 nov 2026, `DIAG=1`).** Al final de la respuesta de `?` van campos nuevos;
`PROTO` sigue en 3 porque ningún comando cambia, y `DIAG` dice la versión de estos campos. Todos
los scripts buscan sus campos por nombre, así que los ignoran. Los `MAX` son lo peor desde el `?`
anterior (se reinician al contestar), en µs:

```
OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=3 RESET=encendido SEG=12 DIAG=1 NS=5321 SMAX=5410 LINMAX=5620 VUELTAMAX=310 I2CERR=0 I2CCOD=0 I2CMAX=4520
```

| Campo | Qué es |
|---|---|
| `NS` | `S` que entendió desde que arrancó (uno dañado no cuenta). El puente los compara con los que mandó |
| `SMAX` | El `S` más lento: de tener la línea completa a terminar de contestar (I²C + lecturas) |
| `LINMAX` | La línea más lenta en llegar completa, del primer carácter al fin de línea (~5.4 ms un `S` a 115200) |
| `VUELTAMAX` | La vuelta de `loop()` más larga sin contar los comandos (`G` y `T` tardan segundos a propósito) |
| `I2CERR` | Escrituras de I²C al PCA9685 que fallaron desde que arrancó |
| `I2CCOD` | Código de `Wire.endTransmission()` de la última falla: 2 o 3 = el PCA9685 no contestó, 4 = otra falla, 5 = se acabó el tiempo; 0 = ninguna |
| `I2CMAX` | La escritura de I²C más lenta (12 canales = 50 bytes a 100 kHz ≈ 4.5 ms) |

| Comando | Respuesta |
|---|---|
| `P canal us` | `OK P canal us`, con el pulso que sí se aplicó |
| `R canal n` | `R canal n media_mV sigma_mV min_mV max_mV`: n lecturas, 0.5 ms entre ellas |
| `D` | `OK D`: suelta todos los servos |
| `V` | `V mv0 … mv11`: una lectura de los 12 (-1 = sin lectura) |
| `?` | `OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=3 RESET=encendido SEG=12`. `RESET` = por qué arrancó la última vez (`encendido` también es al abrir el puerto o apretar EN; `voltaje_bajo` = se cayó la alimentación); `SEG` = segundos desde entonces |
| `G canal us_a us_b vel vueltas espera_ms` | Rampa de un servo hecha por la ESP32 (v2). Ver abajo |
| `S us0 … us11` | `S mv0 … mv11`: los 12 pulsos a la vez y las 12 lecturas (v3). Ver abajo |
| `T INICIO mascara n` · `K …` · `T CORRER [grabar]` | Trayectoria de varios servos a la vez, con grabación (v3). Ver abajo |

### `G`: rampa de un servo (v2)

La ESP32 hace la rampa ella sola. Espera `espera_ms` en `us_a`, va a `us_b` a `vel` µs/s, espera,
regresa a `us_a` y espera; eso `vueltas` veces.

- **Pulso:** se actualiza cada 20 ms (el servo no ve cambios más rápidos) y siempre pasa por el
  límite de 2360 µs.
- **Cable verde:** se graba cada ~2 ms. En FL es la mediana de 5 lecturas de la principal; en FR,
  RL y RR, cada línea `A` de la auxiliar, con la hora en que llegó (~0.6 ms después de medirse).
- **Límites:** canal 0–11; `us_a` ≠ `us_b`, los dos entre 500 y 2360; `vel` de 50 a 4000 µs/s;
  `vueltas` de 1 a 10; `espera_ms` de 0 a 5000; en total, hasta ~17.6 s (9000 muestras).
- **Sin saltos:** antes hay que mandar `P canal us_a`. Si el canal está en otro pulso, contesta
  `E G: primero manda P …`.
- **Para detenerla:** cualquier cosa que llegue por USB la detiene. El servo se queda con el último
  pulso, contesta `E G: detenida …`, no manda datos y luego atiende lo que llegó (por ejemplo, `D`).

Respuesta:

```
OK G canal us_a us_b vel vueltas espera_ms total_ms     al empezar
G DATOS canal n_muestras n_comandos                     al terminar
M t mv            (n_muestras líneas; t en décimas de ms desde el inicio)
C t us            (n_comandos líneas; cada pulso aplicado)
G FIN canal
```

A 115200 baud los datos tardan ~0.55 s por cada segundo de rampa (~10 s para una de 15.5 s).

### `S`: los 12 a la vez (v3)

`S us0 us1 … us11`: un pulso por canal, en orden; `-1` = no cambiar ese canal, `0` = soltarlo.
Cada pulso pasa por el límite de 2360 µs. Responde `S mv0 … mv11`, las 12 lecturas justo después,
igual que `V`. Es para un control en tiempo real desde la compu (por ejemplo, el puente con
Agmen): una línea de ida y una de vuelta, ~11 ms a 115200 baud. **No hace rampas:** quien lo use
tiene que mandar cambios chicos.

**Desde el 9 de noviembre (firmware, el protocolo no cambia):** los pulsos que cambiaron se
escriben al PCA9685 en una sola transacción de I²C, y los que no cambiaron no se vuelven a
escribir. Antes cada pulso usaba `writeMicroseconds()` de Adafruit, que en cada llamada lee el
PRESCALE por I²C: con los 12 servos moviéndose, un `S` tardaba 42 ms en vez de 11 (nota 07). Los
ticks se calculan con la misma cuenta de Adafruit, así que los pulsos son idénticos a los de antes
(el arnés lo revisa de 500 a 2360 µs).

### `T`: trayectoria de varios servos (v3)

1. `T INICIO mascara n`: prepara n puntos (uno cada 20 ms, hasta 1500 = 30 s) para los canales de
   la máscara: bit i = canal i (7 = canales 0, 1 y 2; 4095 = los 12). Responde
   `OK T INICIO mascara n_canales n`.
2. `K us_a us_b …`: un punto, con un pulso por canal de la máscara en orden. Un pulso fuera de
   500–2360 da error y cancela la trayectoria (en una trayectoria es un error, no se recorta).
   Cada 32 puntos, y en el último, responde `OK K cuantos`; la compu espera eso antes de mandar
   más.
3. `T CORRER [grabar]`: la ejecuta. El primer punto debe ser el pulso actual de cada canal (no da
   saltos). Al terminar, los servos se quedan en el último punto.
   - Con `grabar` = 1 (por defecto) graba el cable verde de esos canales cada ~2 ms: FL con la
     mediana de 5, y FR, RL y RR con la última línea de la auxiliar. Caben ~16 s con 3 canales y
     ~5.7 s con los 12.
   - Con `grabar` = 0 no graba y solo responde `T FIN n_puntos`.
   - Igual que `G`, cualquier cosa que llegue por USB la detiene y los servos se quedan donde
     iban.

Respuesta grabando:

```
OK T CORRER n_canales n_puntos total_ms
T DATOS n_canales n_filas n_puntos canal_a canal_b ...
M t mv_a mv_b ...       (n_filas líneas; t en décimas de ms)
C t us_a us_b ...       (n_puntos líneas; cuándo se aplicó cada punto)
T FIN
```

`G` y `T` comparten la misma memoria de grabación (80 kB).

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
| `la trayectoria no terminó a tiempo` y luego `no contestó a 'V'` (3 nov, prueba 05) | La ESP32 se reinició o Windows perdió el USB a media corrida. Con `?`: si `SEG` es chico, se reinició, y `RESET` dice por qué |
| El puente baja a ~24 Hz cuando el perro se mueve y `S tarda 42 ms` (9 nov, nota 07) | `writeMicroseconds()` de Adafruit leía el PRESCALE por I²C en cada pulso. Arreglado en el firmware del 9 nov: carga `esp32_principal` otra vez |
| El puente a ~35 Hz con `peor 100–270 ms` y errores que suben (9 nov, nota 07) | Mira las líneas `compu:` y `ESP32:` que el puente imprime cada 10 s (firmware con `DIAG=1`): dicen de qué lado se perdió el tiempo |
