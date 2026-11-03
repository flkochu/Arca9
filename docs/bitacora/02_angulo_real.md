# 02 · Ángulo real del eje, con transportador

| | |
|---|---|
| Fecha | 2 de noviembre de 2026 |
| Servos | Los 12 (canales 0–11), uno a la vez con el transportador |
| Fuente | ____ V (anotar) |
| Firmware | `firmware/esp32_principal` (límite de 2360 µs) |
| Método | A mano con el Monitor Serie: una lectura subiendo en 500, 1500 y 2360 µs por servo |
| Script | `pc/angulo_real.py` (para repetirla con 5 puntos, subiendo y bajando) |
| Herramienta | [`impresiones/transportador_servo/`](../../impresiones/transportador_servo/): disco sobre el horn + indicador fijo |
| Datos | [`datos/2026-11-02_angulo_real/`](../../datos/2026-11-02_angulo_real/): `giro_real.csv` y `escala.json` |
| Resultado | El eje gira **4.3 % más** que la convención: **10.65 µs por grado real**, no 11.11 |

## Por qué

En la [prueba 01](01_eficiencia_servos.md) el "ángulo pedido" usa la convención
500 µs = 0° y 2500 µs = 180° (11.11 µs por grado). La prueba midió qué tan bien sigue el cable
verde a ese ángulo, pero no si el eje gira de verdad esos grados. Esta prueba lo comprueba con
una escala física.

## Procedimiento

1. Imprime y arma el transportador como dice su README: indicador por abajo del servo, disco
   atornillado encima del horn.
2. Sujeta el servo por el cuerpo, en una prensa o con la mano firme, sin tocar el disco.
3. Practica una vez sin hardware y luego revisa el cable verde con el servo suelto (cierra
   antes el Monitor Serie):
   ```
   cd pc
   python angulo_real.py --simulado
   python angulo_real.py --port COM6 --revisar
   ```
   En `--revisar`, al girar el disco a mano tienen que cambiar los mV (0° ≈ 1860 mV y
   168° ≈ 190 mV). Sal con Ctrl+C.
4. Corre la prueba:
   ```
   python angulo_real.py --port COM6 --voltaje 5.99
   ```
   El script recorre los pulsos de la tabla subiendo y luego bajando, despacio (~60 °/s). En
   cada punto espera 2 s, hace `R 0 200` y te pide la marca que queda en la cuchilla. Escríbela
   y da Enter (`s` salta el punto y `q` termina). El ángulo por retroalimentación aparece
   **después** de que escribes la marca, para que no te influya. Al final manda `D`.
5. Copia la carpeta de `pc/resultados_angulo_real/` a `datos/AAAA-MM-DD_angulo_real/` sin
   editarla, y pega la tabla de su `resumen.md` en "Resultados".

Si te equivocaste al escribir una marca, corrígela en `mediciones.csv` (columna `marca`) antes de
copiar la carpeta a `datos/` y rehaz los cálculos:
```
python angulo_real.py --reanalizar ultima
```

Sin el script, la prueba también se puede hacer a mano en el Monitor Serie (115200): en cada
renglón manda `P 0 <pulso>`, espera 2 s, anota la marca y manda `R 0 200`. Al final, `D`.

| Pulso (µs) | Ángulo pedido | Lectura subiendo (°) | Lectura bajando (°) | Giro real (°) | Cable verde (mV) | Ángulo por retroalimentación (°) |
|---|---|---|---|---|---|---|
| 500 | 0 | | | 0 (referencia) | | |
| 1000 | 45 | | | | | |
| 1500 | 90 | | | | | |
| 2000 | 135 | | | | | |
| 2360 | 167.4 | | | | | |

## Cálculos

- **Giro real:** lectura − lectura en 500 µs. Si los números bajan al girar, resta al revés; si
  la resta pasa por el 0, súmale 360.
- **Ángulo por retroalimentación (canal 0):**
  θ = −0.10018 × mV + 186.87, con la recta de la prueba 01. Con la tabla de búsqueda de
  `datos/2026-10-29_eficiencia_12_servos/canal0/tabla_busqueda.csv` sale más exacto.
- **Escala real del servo:** giro real en 2360 µs / 1860 µs, en grados por µs. Si coincide con la
  convención, da 0.0900 °/µs (= 167.4° / 1860 µs).
- **Error de la convención:** giro real − ángulo pedido, en cada renglón.

El script hace estos cálculos solo. Además:
- Detecta si los números del disco suben o bajan al girar, y corrige el paso por 0/360.
- El giro real de cada renglón es el promedio de subiendo y bajando.
- La escala real sale de la recta con todos los puntos (giro real contra pulso − 500), no solo
  del extremo. Así un error de lectura en un punto pesa menos. También reporta la del extremo.
- Toma la tabla de búsqueda del `calibracion_servos.json` más reciente de `datos/`.

**Ojo:** la retroalimentación se calibró contra la convención (prueba 01). Si la convención
está mal, la retroalimentación se equivoca igual. Por eso esta prueba, con una escala física, es
la que confirma la escala de todo lo demás.

## Qué esperar

Si la convención es buena, el giro real queda dentro de ±1° del ángulo pedido. La lectura del
transportador tiene ~±0.5° de incertidumbre, por las marcas de 2°.

Si el giro real sale proporcionalmente distinto (por ejemplo, 92° en lugar de 90°), la escala
del servo no es 11.11 µs por grado.

## Resultados

Giro real medido con el transportador (una lectura subiendo; la marca en 500 µs es el 0):

| Servo | 500 µs (0°) | 1500 µs (90°) | 2360 µs (167.4°) | Factor |
|---|---|---|---|---|
| 0 · FL abducción | 0 | 92 | 176 | 1.045 |
| 1 · FL cadera | 0 | 92 | 172 | 1.026 |
| 2 · FL rodilla | 0 | 93 | 176 | 1.047 |
| 3 · FR abducción | 0 | 94 | 175 | 1.045 |
| 4 · FR cadera | 0 | 95 | 176 | 1.052 |
| 5 · FR rodilla | 0 | 94 | 175 | 1.045 |
| 6 · RL abducción | 0 | 98 | 178 | 1.069 |
| 7 · RL cadera | 0 | 92 | 174 | 1.036 |
| 8 · RL rodilla | 0 | 94 | 176 | 1.050 |
| 9 · RR abducción | 0 | 92 | 173 | 1.031 |
| 10 · RR cadera | 0 | 93 | 175 | 1.043 |
| 11 · RR rodilla | 0 | 92 | 173 | 1.031 |
| **Promedio** | 0 | **93.4** | **174.9** | **1.043** |

**Factor** = giro real / ángulo de la convención, ajustado por mínimos cuadrados con los dos
puntos de cada servo (recta que pasa por el 0):
factor = (90 · g₁₅₀₀ + 167.4 · g₂₃₆₀) / (90² + 167.4²).

- **Factor de los 12:** 1.043 (desviación de 0.012 entre servos; error estándar del promedio,
  0.003).
- **Escala real:** 11.11 / 1.043 = **10.65 µs por grado real**, o 0.0939 °/µs.
- **El límite de 2360 µs** es 167.4° de la convención y **174.6° reales**.
- **El giro es lineal:** con el promedio de 2360 µs, una recta predice 94.0° en 1500 µs y se
  midieron 93.4°.
- **Dispersión entre servos:** ±2–3° a la mitad del recorrido (de 92° a 98°). Con una sola
  lectura a ojo y marcas de 2°, no alcanza para decir que un servo es distinto de otro. En la
  prueba 01 los 12 tenían la misma forma, recta dentro de 0.5°.

## Conclusiones

- La convención de 11.11 µs/° no es la de estos servos: con el mismo pulso, el eje gira 4.3 % más.
  En el extremo son ~7°, que para la cinemática sí importan.
- **Se usa un solo factor para los 12 (1.043).** Con esta medición no se distinguen
  diferencias reales entre servos, y un factor por servo metería el error de lectura en cada
  articulación.
- **Decisión (opción B): el firmware no se cambia.** Sigue hablando en µs y con su límite duro de
  2360 µs, que se encontró midiendo en esas mismas unidades y sigue siendo válido. El factor se
  aplica solo en la compu, en un solo lugar: `pc/escala.py`, que lo lee de
  `datos/2026-11-02_angulo_real/escala.json`.
- **La prueba 01 sigue valiendo.** La relación mV contra pulso está bien medida; sus "grados"
  son de la convención. `pc/escala.py` los pasa a grados reales con el factor.
- Causa probable: el oscilador del PCA9685 (el firmware supone 27 MHz). Si corre a ~25.9 MHz,
  todos los pulsos salen 4.3 % más largos, y por eso los 12 se desvían casi igual. No se midió;
  con la opción B no hace falta. **Si se cambia la placa PCA9685, hay que repetir esta prueba.**

## Pendiente

- [ ] Anotar el voltaje de la fuente en la tabla de arriba.
- [ ] (Opcional) Repetir con `pc/angulo_real.py` en uno o dos servos, con 5 puntos subiendo y
      bajando, para medir la histéresis del eje y confirmar el factor.
