# Arca9

Arca9 es un perro robot cuadrúpedo de 12 servos.

Adán López Lozano · Ingeniería Mecatrónica, Universidad Iberoamericana CDMX.

Este repositorio empieza de cero el 29 de octubre de 2026, con la prueba de eficiencia de los
12 servos con retroalimentación. De aquí en adelante, todo el trabajo del perro se agrega aquí.

## Estado

| | |
|---|---|
| Hecho | Dos ESP32 leen el cable de retroalimentación de los 12 servos |
| Hecho | Prueba 01: eficiencia de los 12 servos. Error de 0.19–0.25° con la tabla de búsqueda; tope en 168° |
| Hecho | Prueba 02: ángulo real con el transportador. El eje gira 4.3 % más que la convención: 10.65 µs por grado real (`pc/escala.py`) |
| Pospuesta | Prueba 03: ruido de la ESP32 auxiliar (`pc/ruido.py` está listo) |
| Hecho | Prueba 04: rampas. Atraso de ~100 ms a todas las velocidades; sin saturación hasta 240 °/s (`pc/rampas.py`, comando `G`) |
| Hecho | Prueba 05: marcha en el aire con la pata FL. Con el comando adelantado 100 ms, el error del pie baja de 1–2 cm a 0.5 cm (`pc/marcha.py`, comando `T`) |
| Hecho en simulación | 06: puente con Agmen. El panel web ve los 12 ángulos a 50 Hz y maneja al perro; poses parado, sentado y echado (`pc/agmen/`, comando `S`). Falta la primera prueba con el perro |

## Hardware

| Parte | Qué es |
|---|---|
| Servos | 12 × FEETECH FS5115M-FB (Pololu), con cable verde de retroalimentación |
| Control de servos | ESP32 principal + PCA9685 por I²C |
| Retroalimentación | La ESP32 principal lee la pata FL. Una ESP32 auxiliar lee FR, RL y RR y le manda las lecturas por UART |
| Divisor de cada cable verde | 100 kΩ en serie, 150 kΩ a GND y 10 nF a GND |
| IMU | MPU6050 (todavía no se usa en este repositorio) |

Las conexiones pin por pin están en [docs/hardware.md](docs/hardware.md).

## Estructura

```
firmware/esp32_principal/   PCA9685, cables verdes de FL y enlace con la auxiliar; USB a la compu
firmware/esp32_aux/         cables verdes de FR, RL y RR
pc/eficiencia_servos.py     prueba 01, eficiencia: mide, calcula y grafica
pc/angulo_real.py           prueba 02, ángulo real con el transportador impreso
pc/ruido.py                 prueba 03, ruido de los cables verdes con el servo quieto
pc/rampas.py                prueba 04, rampas: velocidad contra error (comando G)
pc/marcha.py                prueba 05, un paso en el aire con la pata FL (comando T)
pc/pata.py                  la pata: convención de ángulos, cinemática y límites
pc/escala.py                µs <-> grados reales y mV -> grados reales (lo usan los scripts nuevos)
pc/arca9_serial.py          comandos de la ESP32 principal (P, R, V, D, ?) para los scripts nuevos
pc/agmen/                   06, puente con Agmen: arca9.agmen, puente.py (comando S), poses.py,
                            manejo.py (las flechas del panel) y robot.toml para agmen run
pruebas/arnes_firmware/     corre los dos firmwares en la compu para probarlos sin las placas
pruebas/puente_agmen/       pruebas del puente sin el perro: ESP32 falsa, agmen run y el arnés
impresiones/                piezas para imprimir, cada una con su generador y sus STL
datos/                      corridas reales, una carpeta por prueba
docs/hardware.md            conexiones, circuito y protocolo
docs/bitacora/              una nota por prueba: objetivo, método, cálculos y resultados
```

## Cómo empezar

1. Instala lo que necesita Python:
   ```
   pip install -r pc/requirements.txt
   ```
2. Sube `firmware/esp32_aux/esp32_aux.ino` a la ESP32 auxiliar y
   `firmware/esp32_principal/esp32_principal.ino` a la principal (Arduino IDE, placa
   "ESP32 Dev Module", librería "Adafruit PWM Servo Driver Library").
3. Cierra el Monitor Serie y revisa que lleguen los 12 cables verdes:
   ```
   python pc/eficiencia_servos.py --port COM6 --revisar
   ```
4. Corre la prueba de eficiencia:
   ```
   python pc/eficiencia_servos.py --port COM6 --voltaje 5.99
   ```

Para practicar sin el hardware: `python pc/eficiencia_servos.py --simulado`.

### Con Agmen (panel web)

Todo el perro con un solo comando, y el panel en el navegador para verlo y manejarlo
([nota 06](docs/bitacora/06_puente_agmen.md)):

```
agmen run pc/agmen/robot_simulado.toml      # sin el perro
agmen run pc/agmen/robot.toml               # con el perro (COM6), colgado y despacio la primera vez
```

## Bitácora

| # | Prueba | Fecha | Resultado |
|---|---|---|---|
| [01](docs/bitacora/01_eficiencia_servos.md) | Eficiencia de los 12 servos con retroalimentación | 29 oct 2026 | 0.19–0.25° RMS con la tabla; histéresis de 0.2° en promedio; tope en 168° |
| [02](docs/bitacora/02_angulo_real.md) | Ángulo real del eje, con transportador | 2 nov 2026 | El eje gira 1.043 × la convención: 10.65 µs por grado real |
| [03](docs/bitacora/03_ruido_auxiliar.md) | Ruido de la ESP32 auxiliar, antes y después de su tierra | pospuesta | — |
| [04](docs/bitacora/04_rampas.md) | Rampas: velocidad contra error, los 12 servos | 2 nov 2026 | Atraso de ~100 ms a todas las velocidades; error = velocidad × 0.1 s; sin saturación hasta 240 °/s |
| [05](docs/bitacora/05_marcha_aire.md) | Marcha en el aire con la pata FL | 3 nov 2026 | Adelanto de 100 ms: error del pie de 1–2 cm a 0.5 cm a todas las velocidades; queda un error fijo de ~0.5 cm |
| [06](docs/bitacora/06_puente_agmen.md) | Puente entre Arca9 y Agmen | 5 nov 2026 (simulación) | 50 Hz, `S` en ~11 ms; el panel le gana a `poses.py` sin saltos. Falta probarlo con el perro |

## Créditos

- Piezas mecánicas: SpotMicro de KDY0523 (Thingiverse 3445283) y el pie v2 de Printables
  (modelo 145053). Sus archivos no están en este repositorio; antes de subirlos, revisa sus
  licencias.
- Licencia de este repositorio: por definir.
