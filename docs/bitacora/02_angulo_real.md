# 02 · Ángulo real del eje, con transportador

| | |
|---|---|
| Fecha | ____ |
| Servo | Canal 0 (FL abducción), sacado de la pata y sujeto por el cuerpo |
| Fuente | ____ V |
| Firmware | `firmware/esp32_principal` (límite de 2360 µs) |
| Herramienta | [`impresiones/transportador_servo/`](../../impresiones/transportador_servo/): disco sobre el horn + indicador fijo |

## Por qué

En la [prueba 01](01_eficiencia_servos.md) el "ángulo pedido" usa la convención
500 µs = 0° y 2500 µs = 180° (11.11 µs por grado). La prueba midió qué tan bien sigue el cable
verde a ese ángulo, pero no si el eje gira de verdad esos grados. Esta prueba lo comprueba con
una escala física.

## Procedimiento

1. Imprime y arma el transportador como dice su README: indicador por abajo del servo, disco
   atornillado encima del horn.
2. Sujeta el servo por el cuerpo, en una prensa o con la mano firme, sin tocar el disco.
3. En el Monitor Serie (115200), para cada renglón de la tabla:
   - manda `P 0 <pulso>`;
   - espera 2 s;
   - anota la marca que queda en la cuchilla;
   - manda `R 0 200` y anota la media (mV).
4. Haz la tabla subiendo y luego bajando. Al final, manda `D`.

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

## Qué esperar

Si la convención es buena, el giro real queda dentro de ±1° del ángulo pedido. La lectura del
transportador tiene ~±0.5° de incertidumbre, por las marcas de 2°.

Si el giro real sale proporcionalmente distinto (por ejemplo, 92° en lugar de 90°), la escala
del servo no es 11.11 µs por grado. En ese caso se corrige el factor en el firmware y en
`pc/eficiencia_servos.py`, y se vuelve a calibrar.

## Resultados

(llenar)

## Conclusiones

(llenar)
