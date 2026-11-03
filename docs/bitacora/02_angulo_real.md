# 02 · Ángulo real del eje, con transportador

| | |
|---|---|
| Fecha | ____ |
| Servo | Canal 0 (FL abducción), sacado de la pata y sujeto por el cuerpo |
| Fuente | ____ V |
| Firmware | `firmware/esp32_principal` (límite de 2360 µs) |
| Script | `pc/angulo_real.py` |
| Herramienta | [`impresiones/transportador_servo/`](../../impresiones/transportador_servo/): disco sobre el horn + indicador fijo |
| Datos | `datos/AAAA-MM-DD_angulo_real/` (llenar) |

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
del servo no es 11.11 µs por grado. En ese caso se corrige el factor en el firmware y en
`pc/eficiencia_servos.py`, y se vuelve a calibrar.

## Resultados

(llenar)

## Conclusiones

(llenar)
