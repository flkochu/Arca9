# 03 · Ruido de la ESP32 auxiliar, antes y después de mejorar su tierra

> **Pospuesta (2 nov 2026).** Se pasó directo a la [prueba 04](04_rampas.md). En las rampas el
> ruido (~0.3–0.6° por lectura) es chico comparado con los errores que se miden (varios grados).
> Hay que hacerla antes de usar las lecturas sueltas de FR, RL y RR para controlar la marcha.

| | |
|---|---|
| Fecha | ____ |
| Servos | Los 12, en el banco (fuera de las patas) |
| Fuente | ____ V |
| Firmware | `firmware/esp32_principal` + `firmware/esp32_aux` (sin cambios) |
| Script | `pc/ruido.py` |
| Cómo se alimenta la auxiliar (antes) | ☐ VIN desde la principal ☐ su propio USB (anotar) |
| Cómo va su GND (antes) | ____ (largo y calibre del cable, y a dónde llega) |
| Datos | `datos/AAAA-MM-DD_ruido/` (llenar) |

## Por qué

En la [prueba 01](01_eficiencia_servos.md) el ruido con el servo quieto fue de 3.0–3.4 mV en
FL, que lee la principal, y de 4.2–7.2 mV en FR, RL y RR, que lee la auxiliar: casi el doble. En
la calibración no pesó, porque cada punto promedió 200 lecturas. En la marcha sí va a pesar,
porque el puente va a usar lecturas sueltas (`V`).

Las dos placas usan el mismo divisor y el mismo ADC. Por eso lo más probable es que la diferencia
venga de la **tierra o la alimentación de la auxiliar**: si su GND está lejos de la tierra de la
fuente, o si por ese cable regresa parte de la corriente de los servos, la auxiliar ve el ruido
de esa corriente como voltaje en sus pines.

## Qué mide el script

- **σ con R**, como en la prueba 01: `R canal 200` cinco veces en cada canal. Es el ruido de una
  lectura suelta del ADC, comparable con la prueba 01.
- **σ con V**, lo que va a usar el puente: 500 lecturas `V` de los 12 (mediana de 5 en cada
  placa). Es el ruido que va a ver la marcha.
- Todo en **mV del pin**, la medida directa, y en **grados reales** (1 mV ≈ 0.10°, con el factor
  de la [prueba 02](02_angulo_real.md)).
- Por defecto los 12 se van uno por uno, despacio, a 1500 µs y se quedan con fuerza, como en la
  marcha. Con `--sueltos` no se mueven: se manda `D` y se mide así. **Si sueltos el ruido baja
  mucho, la culpa es de la corriente de los servos** que pasa por la tierra.

## Procedimiento

Cambia los cables siempre con **la fuente de los servos apagada y el USB desconectado**.

1. Practica sin hardware (cierra antes el Monitor Serie):
   ```
   cd pc
   python ruido.py --simulado
   ```
2. **Antes:** con el cableado de hoy, dos corridas:
   ```
   python ruido.py --port COM6 --etiqueta antes --voltaje 5.99
   python ruido.py --port COM6 --etiqueta antes_sueltos --sueltos --voltaje 5.99
   ```
3. **Cambio 1, la tierra:** un cable corto y grueso del GND de la auxiliar al punto donde se
   juntan el GND de la fuente y el del PCA9685. Déjalo anotado en la tabla de "Cambios".
4. **Después:**
   ```
   python ruido.py --port COM6 --etiqueta despues_gnd --nota "GND corto y grueso a la fuente" --voltaje 5.99
   ```
5. Compara (usa las carpetas que te imprimió cada corrida):
   ```
   python ruido.py --comparar resultados_ruido\<carpeta_antes> resultados_ruido\<carpeta_despues_gnd>
   ```
6. Si la auxiliar sigue arriba de 1.3 veces el ruido de la principal, prueba el **cambio 2, la
   alimentación**. Si la auxiliar tiene su propio USB, pásala a VIN desde la principal, o al
   revés, pero nunca las dos cosas a la vez (`docs/hardware.md`). Mide con
   `--etiqueta despues_alim` y compara otra vez.
7. Copia las carpetas que sirvieron de `pc/resultados_ruido/` a `datos/AAAA-MM-DD_ruido/`, sin
   editarlas, y pega aquí las tablas de `comparacion.md`.

Para probar cambios rápido: `python ruido.py --port COM6 --revisar` muestra en vivo la σ de las
últimas 100 lecturas `V` de cada canal. Ctrl+C sale y suelta los servos.

## Cambios

| # | Etiqueta | Qué se cambió |
|---|---|---|
| 0 | antes | nada |
| 1 | despues_gnd | |
| 2 | despues_alim | |

## Qué esperar

Si la tierra era el problema, σ R de la auxiliar baja a ~3 mV (como FL) y la razón
auxiliar/principal queda en 1.3 o menos. Si no baja con ninguno de los dos cambios, el siguiente
sospechoso es la alimentación de 3.3 V de la propia auxiliar, o los cables verdes largos de
RL y RR. Ahí se mide antes de cambiar nada más.

## Resultados

(llenar)

## Conclusiones

(llenar)
