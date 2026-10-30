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
| Listo para probar | Prueba 02: ángulo real del eje con el transportador impreso (`impresiones/transportador_servo/`) |
| Sigue | Mejorar la tierra de la ESP32 auxiliar |
| Sigue | Rampas (velocidad contra error) y marcha en el aire con una pata |

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
pc/eficiencia_servos.py     prueba de eficiencia: mide, calcula y grafica
pruebas/arnes_firmware/     corre los dos firmwares en la compu para probarlos sin las placas
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

## Bitácora

| # | Prueba | Fecha | Resultado |
|---|---|---|---|
| [01](docs/bitacora/01_eficiencia_servos.md) | Eficiencia de los 12 servos con retroalimentación | 29 oct 2026 | 0.19–0.25° RMS con la tabla; histéresis de 0.2° en promedio; tope en 168° |
| [02](docs/bitacora/02_angulo_real.md) | Ángulo real del eje, con transportador | pendiente | — |

## Créditos

- Piezas mecánicas: SpotMicro de KDY0523 (Thingiverse 3445283) y el pie v2 de Printables
  (modelo 145053). Sus archivos no están en este repositorio; antes de subirlos, revisa sus
  licencias.
- Licencia de este repositorio: por definir.
