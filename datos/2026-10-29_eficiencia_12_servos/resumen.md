# Eficiencia de los servos con retroalimentación

- Fecha: 20261029_181221
- Servos: FEETECH FS5115M-FB · divisor 100k/150k · fuente 5.99 V
- Rango pedido: 0° a 175°; pasadas de 30, 20, 10, 5, 2°; subiendo y bajando
- Por punto: 0.8 s de espera y 200 lecturas del ADC (0.5 ms entre ellas)

## Todos los servos

| Canal | Servo | Rango útil (°) | mV/° | Error RMS recta (°) | Error RMS tabla (°) | Error máx tabla (°) | ≤ 1° tabla (%) | Histéresis máx (°) | Ruido σ (°) |
|---|---|---|---|---|---|---|---|---|---|
| 0 | FL abducción | 0–168 | 10.0 | 0.32 | 0.22 | 0.61 | 100 | 0.87 | 0.30 |
| 1 | FL cadera | 0–168 | 9.9 | 0.32 | 0.25 | 0.87 | 100 | 1.01 | 0.33 |
| 2 | FL rodilla | 0–168 | 10.1 | 0.33 | 0.21 | 0.61 | 100 | 0.81 | 0.34 |
| 3 | FR abducción | 0–168 | 10.1 | 0.30 | 0.22 | 0.54 | 100 | 0.77 | 0.63 |
| 4 | FR cadera | 0–168 | 10.1 | 0.31 | 0.19 | 0.58 | 100 | 0.74 | 0.64 |
| 5 | FR rodilla | 0–168 | 10.1 | 0.32 | 0.21 | 0.60 | 100 | 0.70 | 0.71 |
| 6 | RL abducción | 0–170 | 10.0 | 0.39 | 0.21 | 1.01 | 99 | 0.76 | 0.61 |
| 7 | RL cadera | 0–168 | 10.1 | 0.32 | 0.20 | 0.50 | 100 | 0.71 | 0.64 |
| 8 | RL rodilla | 0–170 | 9.9 | 0.37 | 0.21 | 0.75 | 100 | 0.71 | 0.64 |
| 9 | RR abducción | 0–168 | 9.9 | 0.36 | 0.24 | 0.66 | 100 | 0.84 | 0.60 |
| 10 | RR cadera | 0–170 | 9.9 | 0.40 | 0.24 | 1.02 | 99 | 0.80 | 0.56 |
| 11 | RR rodilla | 0–168 | 10.1 | 0.32 | 0.22 | 0.56 | 100 | 0.73 | 0.42 |

- Error RMS con la recta: de 0.30° a 0.40° (el peor: canal 10, RR cadera).
- Error RMS con la tabla: de 0.19° a 0.25°.
- Histéresis máxima: hasta 1.01°. Ruido: hasta 0.71°.
- Servos que no recorren todo lo pedido: 0 (0–168°, 500–2367 µs), 1 (0–168°, 500–2367 µs), 2 (0–168°, 500–2367 µs), 3 (0–168°, 500–2367 µs), 4 (0–168°, 500–2367 µs), 5 (0–168°, 500–2367 µs), 6 (0–170°, 500–2389 µs), 7 (0–168°, 500–2367 µs), 8 (0–170°, 500–2389 µs), 9 (0–168°, 500–2367 µs), 10 (0–170°, 500–2389 µs), 11 (0–168°, 500–2367 µs). Pon ese rango en US_MAX_CH del firmware.

## Cómo leerla

- **Error**: ángulo medido con la retroalimentación menos el pedido. Con la recta incluye que el potenciómetro no es perfectamente lineal; la tabla de búsqueda lo corrige.
- **Histéresis**: diferencia entre llegar subiendo y llegar bajando al mismo ángulo (banda muerta del servo).
- **Ruido**: cuánto se mueve la lectura con el servo quieto.
- **Rango útil**: dónde el servo sigue el comando; fuera de él llegó a su tope.
- El ángulo pedido usa la convención 500 µs = 0°, 2500 µs = 180°. Que el giro real del eje coincida se comprueba aparte, con transportador.

## Canal 0 · FL abducción

- Rango útil: 0° a 168° (500–2367 µs), 191–1867 mV en el pin
- Recta: ángulo = -0.10018·mV +186.87  (9.98 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.32°, máx 0.94°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.22°, máx 0.61°, 100 % dentro de ±1°
- Histéresis: máx 0.87°, media 0.24°
- Ruido con el servo quieto: σ 2.98 mV = 0.299°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99997, se separa hasta 0.06°, error RMS 0.27°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.07°, error RMS 0.28°
  - de 10 en 10°: 34 puntos, R² 0.99997, se separa hasta 0.03°, error RMS 0.27°
  - de 5 en 5°: 68 puntos, R² 0.99996, se separa hasta 0.03°, error RMS 0.31°
  - de 2 en 2°: 170 puntos, R² 0.99995, se separa hasta 0.02°, error RMS 0.34°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 1 · FL cadera

- Rango útil: 0° a 168° (500–2367 µs), 192–1858 mV en el pin
- Recta: ángulo = -0.10071·mV +186.94  (9.93 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.32°, máx 0.98°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.25°, máx 0.87°, 100 % dentro de ±1°
- Histéresis: máx 1.01°, media 0.20°
- Ruido con el servo quieto: σ 3.28 mV = 0.330°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99998, se separa hasta 0.13°, error RMS 0.22°
  - de 20 en 20°: 18 puntos, R² 0.99998, se separa hasta 0.09°, error RMS 0.27°
  - de 10 en 10°: 34 puntos, R² 0.99997, se separa hasta 0.11°, error RMS 0.29°
  - de 5 en 5°: 68 puntos, R² 0.99995, se separa hasta 0.01°, error RMS 0.33°
  - de 2 en 2°: 170 puntos, R² 0.99995, se separa hasta 0.04°, error RMS 0.34°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 2 · FL rodilla

- Rango útil: 0° a 168° (500–2367 µs), 193–1874 mV en el pin
- Recta: ángulo = -0.09948·mV +186.32  (10.05 mV por grado), R² = 0.99995
- Error con la recta: RMS 0.33°, máx 1.06°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.21°, máx 0.61°, 100 % dentro de ±1°
- Histéresis: máx 0.81°, media 0.21°
- Ruido con el servo quieto: σ 3.38 mV = 0.336°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.13°, error RMS 0.16°
  - de 20 en 20°: 18 puntos, R² 0.99996, se separa hasta 0.09°, error RMS 0.32°
  - de 10 en 10°: 34 puntos, R² 0.99997, se separa hasta 0.13°, error RMS 0.29°
  - de 5 en 5°: 68 puntos, R² 0.99996, se separa hasta 0.08°, error RMS 0.31°
  - de 2 en 2°: 170 puntos, R² 0.99995, se separa hasta 0.04°, error RMS 0.36°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 3 · FR abducción

- Rango útil: 0° a 168° (500–2367 µs), 185–1888 mV en el pin
- Recta: ángulo = -0.09853·mV +185.73  (10.15 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.30°, máx 0.98°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.22°, máx 0.54°, 100 % dentro de ±1°
- Histéresis: máx 0.77°, media 0.20°
- Ruido con el servo quieto: σ 6.40 mV = 0.631°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.16°, error RMS 0.23°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.16°, error RMS 0.32°
  - de 10 en 10°: 34 puntos, R² 0.99997, se separa hasta 0.03°, error RMS 0.29°
  - de 5 en 5°: 68 puntos, R² 0.99996, se separa hasta 0.03°, error RMS 0.31°
  - de 2 en 2°: 170 puntos, R² 0.99996, se separa hasta 0.02°, error RMS 0.31°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 4 · FR cadera

- Rango útil: 0° a 168° (500–2367 µs), 182–1870 mV en el pin
- Recta: ángulo = -0.09935·mV +185.62  (10.07 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.31°, máx 0.98°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.19°, máx 0.58°, 100 % dentro de ±1°
- Histéresis: máx 0.74°, media 0.19°
- Ruido con el servo quieto: σ 6.42 mV = 0.637°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99998, se separa hasta 0.16°, error RMS 0.25°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.10°, error RMS 0.27°
  - de 10 en 10°: 34 puntos, R² 0.99997, se separa hasta 0.03°, error RMS 0.26°
  - de 5 en 5°: 68 puntos, R² 0.99997, se separa hasta 0.01°, error RMS 0.28°
  - de 2 en 2°: 170 puntos, R² 0.99995, se separa hasta 0.03°, error RMS 0.34°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 5 · FR rodilla

- Rango útil: 0° a 168° (500–2367 µs), 187–1888 mV en el pin
- Recta: ángulo = -0.09860·mV +185.95  (10.14 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.32°, máx 0.95°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.21°, máx 0.60°, 100 % dentro de ±1°
- Histéresis: máx 0.70°, media 0.24°
- Ruido con el servo quieto: σ 7.20 mV = 0.709°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.27°, error RMS 0.20°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.13°, error RMS 0.29°
  - de 10 en 10°: 34 puntos, R² 0.99996, se separa hasta 0.07°, error RMS 0.30°
  - de 5 en 5°: 68 puntos, R² 0.99997, se separa hasta 0.04°, error RMS 0.28°
  - de 2 en 2°: 170 puntos, R² 0.99995, se separa hasta 0.03°, error RMS 0.35°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 6 · RL abducción

- Rango útil: 0° a 170° (500–2389 µs), 180–1867 mV en el pin
- Recta: ángulo = -0.09992·mV +186.37  (10.01 mV por grado), R² = 0.99994
- Error con la recta: RMS 0.39°, máx 1.93°, 98 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.21°, máx 1.01°, 99 % dentro de ±1°
- Histéresis: máx 0.76°, media 0.19°
- Ruido con el servo quieto: σ 6.13 mV = 0.612°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.25°, error RMS 0.21°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.18°, error RMS 0.29°
  - de 10 en 10°: 36 puntos, R² 0.99992, se separa hasta 0.22°, error RMS 0.48°
  - de 5 en 5°: 70 puntos, R² 0.99993, se separa hasta 0.12°, error RMS 0.42°
  - de 2 en 2°: 172 puntos, R² 0.99994, se separa hasta 0.07°, error RMS 0.38°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 172, 174, 175°.

## Canal 7 · RL cadera

- Rango útil: 0° a 168° (500–2367 µs), 187–1879 mV en el pin
- Recta: ángulo = -0.09898·mV +185.88  (10.10 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.32°, máx 0.93°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.20°, máx 0.50°, 100 % dentro de ±1°
- Histéresis: máx 0.71°, media 0.21°
- Ruido con el servo quieto: σ 6.44 mV = 0.637°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.22°, error RMS 0.17°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.05°, error RMS 0.28°
  - de 10 en 10°: 34 puntos, R² 0.99997, se separa hasta 0.11°, error RMS 0.27°
  - de 5 en 5°: 68 puntos, R² 0.99995, se separa hasta 0.05°, error RMS 0.34°
  - de 2 en 2°: 170 puntos, R² 0.99995, se separa hasta 0.02°, error RMS 0.34°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 8 · RL rodilla

- Rango útil: 0° a 170° (500–2389 µs), 179–1859 mV en el pin
- Recta: ángulo = -0.10060·mV +186.93  (9.94 mV por grado), R² = 0.99995
- Error con la recta: RMS 0.37°, máx 1.51°, 98 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.21°, máx 0.75°, 100 % dentro de ±1°
- Histéresis: máx 0.71°, media 0.20°
- Ruido con el servo quieto: σ 6.35 mV = 0.638°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99997, se separa hasta 0.13°, error RMS 0.28°
  - de 20 en 20°: 18 puntos, R² 0.99996, se separa hasta 0.13°, error RMS 0.33°
  - de 10 en 10°: 36 puntos, R² 0.99994, se separa hasta 0.29°, error RMS 0.43°
  - de 5 en 5°: 70 puntos, R² 0.99994, se separa hasta 0.07°, error RMS 0.39°
  - de 2 en 2°: 172 puntos, R² 0.99995, se separa hasta 0.04°, error RMS 0.36°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 172, 174, 175°.

## Canal 9 · RR abducción

- Rango útil: 0° a 168° (500–2367 µs), 181–1840 mV en el pin
- Recta: ángulo = -0.10086·mV +185.42  (9.91 mV por grado), R² = 0.99995
- Error con la recta: RMS 0.36°, máx 1.07°, 99 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.24°, máx 0.66°, 100 % dentro de ±1°
- Histéresis: máx 0.84°, media 0.31°
- Ruido con el servo quieto: σ 5.96 mV = 0.601°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99998, se separa hasta 0.14°, error RMS 0.23°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.08°, error RMS 0.30°
  - de 10 en 10°: 34 puntos, R² 0.99996, se separa hasta 0.06°, error RMS 0.31°
  - de 5 en 5°: 68 puntos, R² 0.99996, se separa hasta 0.06°, error RMS 0.32°
  - de 2 en 2°: 170 puntos, R² 0.99994, se separa hasta 0.05°, error RMS 0.39°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.

## Canal 10 · RR cadera

- Rango útil: 0° a 170° (500–2389 µs), 180–1858 mV en el pin
- Recta: ángulo = -0.10051·mV +186.66  (9.95 mV por grado), R² = 0.99994
- Error con la recta: RMS 0.40°, máx 1.83°, 98 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.24°, máx 1.02°, 99 % dentro de ±1°
- Histéresis: máx 0.80°, media 0.22°
- Ruido con el servo quieto: σ 5.53 mV = 0.555°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.16°, error RMS 0.16°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.19°, error RMS 0.31°
  - de 10 en 10°: 36 puntos, R² 0.99992, se separa hasta 0.23°, error RMS 0.49°
  - de 5 en 5°: 70 puntos, R² 0.99992, se separa hasta 0.06°, error RMS 0.44°
  - de 2 en 2°: 172 puntos, R² 0.99994, se separa hasta 0.10°, error RMS 0.39°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 172, 174, 175°.

## Canal 11 · RR rodilla

- Rango útil: 0° a 168° (500–2367 µs), 185–1878 mV en el pin
- Recta: ángulo = -0.09912·mV +186.08  (10.09 mV por grado), R² = 0.99996
- Error con la recta: RMS 0.32°, máx 0.89°, 100 % de los puntos dentro de ±1°
- Error con la tabla (de 2 en 2°, probada con otras pasadas): RMS 0.22°, máx 0.56°, 100 % dentro de ±1°
- Histéresis: máx 0.73°, media 0.22°
- Ruido con el servo quieto: σ 4.23 mV = 0.419°
- Por pasada (recta de cada una contra la de todos los datos):
  - de 30 en 30°: 12 puntos, R² 0.99999, se separa hasta 0.15°, error RMS 0.20°
  - de 20 en 20°: 18 puntos, R² 0.99997, se separa hasta 0.18°, error RMS 0.29°
  - de 10 en 10°: 34 puntos, R² 0.99996, se separa hasta 0.10°, error RMS 0.33°
  - de 5 en 5°: 68 puntos, R² 0.99995, se separa hasta 0.04°, error RMS 0.33°
  - de 2 en 2°: 170 puntos, R² 0.99996, se separa hasta 0.06°, error RMS 0.32°
- ⚠ Fuera del rango útil (llegó a su tope o no siguió el comando): 170, 172, 174, 175°.
