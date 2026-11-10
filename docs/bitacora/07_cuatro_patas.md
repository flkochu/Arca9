# 07 · Las 4 patas medidas, con el sentido de cada servo

| | |
|---|---|
| Fecha | 8 y 9 de noviembre de 2026 |
| Perro | Armado completo (SpotMicro KDY0523), en el soporte, con las patas al aire |
| Herramienta | [`pc/ver_servos.py`](../../pc/ver_servos.py): los 12 ángulos en vivo con los servos sueltos, sin mandar ningún pulso |
| Datos | [`datos/2026-11-09_patas/`](../../datos/2026-11-09_patas/): `pata_fl.json`, `pata_fr.json`, `pata_rl.json`, `pata_rr.json` y las medidas tal cual |
| Código | `pc/pata.py` (signos por pata), `pc/agmen/perro.py`, `pc/agmen/poses.py`, `pc/agmen/robot.toml` (Agmen 0.13) |
| Pruebas | `python pruebas/puente_agmen/prueba_puente.py`: 0 fallas |
| Resultado | **Las 4 patas tienen límites, pata recta, pose de pie y sentido.** Las 3 poses (parado, sentado, echado) caben en las 4 con 8.6° o más de margen. Falta la primera prueba en el aire con las 4 |

## Por qué

El puente (nota 06) solo mueve una pata que tenga su `pata_xx.json`: sin límites medidos no se
mueve, a propósito. Hasta ahora solo estaba la FL, medida suelta el 3 de noviembre. Con el perro
armado hacía falta medir las otras 3, y volver a revisar la FL.

## Cómo se midió

Con el perro en el soporte y `python pc/ver_servos.py --port COM6`, que suelta los 12 servos (`D`)
y muestra lo que mide cada cable verde, en grados reales. Moviendo cada articulación con la mano:

1. **Cables.** Al mover cada articulación, el ◄ tiene que salir en su canal. Así salió que la RL
   tenía la abducción y la rodilla cruzadas (P6 ↔ P8). Se cambiaron los dos conectores en el
   PCA9685 para que quede como en `docs/hardware.md`, igual que las otras 3 patas.
2. **Límites.** Cada articulación hasta donde apenas choca. En la abducción hay dos: hacia arriba
   (afuera) y hacia adentro, donde **choca con la pata del otro lado**.
3. **Pata recta.** Toda la pata apuntando derecho hacia abajo, con la rodilla estirada. Es el cero
   de cada articulación.
4. **Pose de pie.** Pata vertical vista de frente, rodilla doblada hacia atrás. Solo sirve para
   revisar los signos: las poses salen de la cinemática.

## Lo medido (grados reales del servo)

Los límites se midieron dos veces (8 y 9 de noviembre). Donde no coinciden, se usa la medida más
estrecha. En la abducción, el lado de adentro es donde **choca con la pata del otro lado** (sola
llegaría más lejos).

| Pata | Canales | Recta (abd · cad · rod) | De pie | Abducción: afuera / choca (sola) | Cadera | Rodilla |
|---|---|---|---|---|---|---|
| FL | 0 · 1 · 2 | 85.1 · 90.1 · 148.8 | 85.3 · 131.1 · 68.9 | 36 (33) / 125 (160) | −4–172 | 15–175 |
| FR | 3 · 4 · 5 | 92.8 · 88.7 · 0.4 | 93.1 · 64.3 · 95.9 | 145 (151) / 58 (22) | −4–174 | −5–146 |
| RL | 6 · 7 · 8 | 85.2 · 98.8 · 170.9 | 85.2 · 127.1 · 73.4 | 143 (151) / 69.5 (40) | −4–174 | 33–176 |
| RR | 9 · 10 · 11 | 94.1 · 76.3 · 0.5 | 94.2 · 51.9 · 89.2 | 35 (29) / 128 (163) | −6–175 | −6–136 |

*Los valores negativos y los de 174–176 son los topes del propio servo: esa articulación no choca en
todo su recorrido. La placa nunca manda menos de 0° ni más de 174.6° (500 a 2360 µs). En la
abducción "afuera", entre paréntesis va la otra medida (se usa la más estrecha).*

## Lo que salió

### 1. Cada pata gira para su lado

Comparando la pata recta con la de pie, y los límites de la abducción:

| Pata | Al subir el ángulo… abducción | cadera | rodilla | Signos (abd, cad, rod) |
|---|---|---|---|---|
| FL | se mete | va hacia atrás | se estira | −1, −1, −1 |
| FR | se abre | va hacia adelante | se dobla | +1, +1, +1 |
| RL | se abre | va hacia atrás | se estira | +1, −1, −1 |
| RR | se mete | va hacia adelante | se dobla | −1, +1, +1 |

**Por qué:** el lado derecho es un espejo del izquierdo, así que cadera y rodilla giran al revés
entre FL y FR (y entre RL y RR). En la abducción, además, los servos de atrás miran hacia el lado
contrario que los de adelante, y eso la vuelve a invertir. Por eso FL y RR giran igual en la
abducción, y FR y RL al revés.

Para que el resto del código no tenga que saberlo, los **ángulos de articulación** significan lo
mismo en las 4 patas (0 = recta; + = se abre, el muslo va adelante, la rodilla se dobla). Cada
`pata_xx.json` dice su `signos`, y `pata.py` hace la cuenta:

    grado del servo = cero + signo · q

Con sus signos, las 4 poses de pie medidas salen con la rodilla doblada hacia atrás y el muslo
hacia atrás, como debe ser. Si un signo estuviera mal, `perro.py` no dejaría cargar esa pata
(revisa la pose de pie).

### 2. La rodilla FL se recorrió un diente al armar

| FL | Recta 3 nov (suelta) | Recta 9 nov (armada) | Diferencia |
|---|---|---|---|
| Abducción | 87.8 | 85.1 | −2.7 |
| Cadera | 86.6 | 90.1 | +3.5 |
| Rodilla | 162.6 | 148.8 | **−13.8** |

La rodilla doblada de pie cambió casi lo mismo (84.5 → 68.9) y su flexión quedó igual (78° y
80°). O sea, no es un error de lectura: **el brazo del servo quedó un diente corrido**. El eje tiene
25 dientes, 360 / 25 = 14.4° por diente, y 13.8 ≈ 14.4. Por eso:

- `datos/2026-11-03_pata_fl` queda para la prueba 05 (ese día la pata estaba así), y ahora se usa
  la FL nueva.
- Sus límites se volvieron a medir. El de la rodilla doblada salió en 15°, no en los 1.2° que se
  habían estimado corriendo el del 3 de noviembre: por eso un límite se mide, no se calcula.

### 3. Las rodillas FR, RR y RL quedan rectas cerca del tope del servo

La pata recta de FR (0.4°) y RR (0.5°) está cerca del tope del servo, y la de RL (170.9°) también.
La segunda medida de límites lo aclaró: esas rodillas todavía pasan unos grados más allá (FR −5°,
RR −6°, RL 176°). O sea, la pata recta se midió antes del tope, y su cero es bueno. Igual se revisa
en la prueba en el aire: en "parado", las 4 patas deben verse iguales.

### 4. Las poses caben en las 4 patas

Las poses son posiciones del pie (`poses.py`). Con los límites de arriba menos 5° de margen (y la
rodilla sin pasar de recta), las tres caben con 8.6° o más de sobra en cada servo:

| Pose | FL | FR | RL | RR |
|---|---|---|---|---|
| parado | 85.2 · 127.2 · 70.1 | 92.9 · 51.6 · 79.1 | 85.2 · 135.9 · 92.2 | 94.2 · 39.2 · 79.2 |
| sentado | 85.2 · 123.0 · 78.0 | 92.9 · 55.8 · 71.2 | 85.2 · 142.7 · 50.7 | 94.2 · 32.4 · 120.7 |
| echado | 85.2 · 134.0 · 28.6 | 92.9 · 44.8 · 120.6 | 85.2 · 142.7 · 50.7 | 94.2 · 32.4 · 120.7 |

Lo más justo es la rodilla FL echada: 28.6°, a 8.6° de su límite (15° + 5° de margen). Le sigue la
abducción RL: con la pata vertical (85.2°) queda a 10.7° de donde choca con la RR (69.5° + 5°).

## Lo que cambió en el código

- **`pc/pata.py`**: lee `signos` de cada `.json` (si no lo trae, −1 en las tres, como la FL del 3
  de noviembre). La rodilla no pasa de recta del lado que corresponde a su signo.
- **`pc/agmen/perro.py`**: carga las 4 patas. Ya no rechaza una pata que gire al revés que la FL,
  pero sí una cuyo signo no cuadre con su pose de pie.
- **Agmen 0.13** (de paso, `PENDIENTES_AGMEN.md` 1, 2, 4 y 7): `manejo.py` se borró; el panel tiene
  la tarjeta **Poses** (botones parado, sentado y echado → `Arca9Pose` → `poses.py`), y `poses.py`
  sabe quién manda por `/mux/mando`.
- **Nuevo en `PENDIENTES_AGMEN.md`, el 11:** un comentario de varias líneas justo antes de un
  `const` en un `.agmen` rompe el código que genera Agmen.

## Procedimiento: la primera prueba con las 4 patas, en el aire

Con el perro **en el soporte**, las patas sin tocar nada, el Monitor Serie de Arduino cerrado y la
fuente de 6 V prendida:

1. `python pc\ver_servos.py --port COM6 --revisar`: los 12 con lectura (ningún "—"). Ctrl+C.
2. `agmen run pc\agmen\robot.toml --record`. Se abre el panel. El puente arranca **sin mover
   nada** (`modo` 0, SUELTO).
3. ▶ en `poses`: las 4 patas van a **parado**, a 15 °/s. Mira que las 4 queden iguales: pie debajo
   de la cadera, rodilla hacia atrás, vistas de frente verticales.
4. En la tarjeta **Poses**: *sentado*, luego *echado* y luego *parado*. Con apretar medio segundo
   basta.
5. ■ en `poses`: el puente se queda quieto (watchdog). ■ en `puente`: suelta los servos (`D`).
   Ctrl+C.

Para todo antes (■ en `puente`) si algo choca, una pata se ve distinta a las otras o un servo
zumba o se calienta. La grabación queda en `.mcap` (Agmen 0.12): `agmen web ARCHIVO.mcap` la
reproduce en el panel.

### Resultados de la primera prueba en el aire (9 de noviembre, 17:02)

Grabación: `pc/agmen/grabaciones/agmen-2026-11-09_17-01-56.mcap` (590 s, 0 mensajes perdidos).
Analizada con `agmen.Recording`.

| Qué | Resultado |
|---|---|
| ESP32 | 0 errores, 0 reinicios, siempre conectada. `SEG` empezó en 1337 s: **abrir el puerto no la reinició** |
| Cables verdes | Las 12 lecturas creíbles todo el tiempo (ningún NaN) |
| Límites | Solo se recortó al arrancar (82–86 s): las rodillas colgaban rectas, fuera de su límite, y el puente las metió a su rango. Ningún servo bloqueado |
| Las 4 patas en "parado" | Pies medidos a 0.5 cm o menos de lo pedido (x 1.7–2.3, z 18.4–18.6 cm; pedido 2.0, 18.5): **las 4 quedan iguales**. Los signos y los ceros están bien |
| "Echado" | z 10.8–11.2 cm (pedido 11.0) |
| Error fijo, quieto | ~+0.6° (como el sesgo de la prueba 05) |
| Botón → `poses.py` empieza | 48–63 ms |
| Comando → el servo lo sigue | 130–235 ms (la mitad, ~190 ms). La prueba 04 midió ~100 ms del servo solo; el resto es el ciclo del puente |
| Duración de cada cambio de pose | 4–6 s, a propósito: `--vel 15` en `poses.py`, `--vel-max 20` en el puente y el perfil de jerk mínimo |
| **El comando `S`** | **10.7 ms sin mover nada, 42 ms moviendo** (mediana). Con 42 ms el puente baja de 50 a ~24 ciclos por segundo |

**Por qué `S` tarda 42 ms al mover.** El firmware pone cada pulso con `writeMicroseconds()` de la
librería de Adafruit, y esa función **lee el registro PRESCALE del PCA9685 por I²C en cada
llamada**, antes de escribir el pulso (`Adafruit_PWMServoDriver.cpp`: `readPrescale()` dentro de
`writeMicroseconds()`). Son 2 transacciones de I²C por servo, 24 por cada `S`. A 100 kHz, y con lo
que tarda cada transacción en el núcleo de Arduino de la ESP32, suman ~31 ms. Sin pulsos (todo
`-1`), el `S` solo lee y tarda 10.7 ms: lo que tardan los bytes a 115200 baud.

**Arreglo (firmware del 9 nov, el protocolo no cambia):** leer el PRESCALE una vez al arrancar y
calcular los ticks con la misma fórmula de Adafruit (salen los mismos pulsos), y mandar solo los
servos cuyo pulso cambió, todos en una transacción. El I²C se quedó en 100 kHz: con una sola
transacción ya alcanza (50 bytes = 4.5 ms) y no se cambia algo que no hacía falta.

### Después del arreglo: `S` más rápido, pero con pausas (9 nov, 17:42)

```
[puente] 37.2 Hz · S tarda 14.9 ms (peor 218.1) · modo SIGUIENDO · errores 8 · reinicios 0
[puente] 34.7 Hz · S tarda 14.2 ms (peor 144.1) · modo SIGUIENDO · errores 18 · reinicios 0
[puente] ⚠ every(20 ms): 20 ciclos saltados ... Tu función tardó 129.5 ms (y 209.5, 269.5 ms)
[puente] ⛔ watchdog: sin comandos desde hace 0.5 s → quieto     (una vez; luego ▶ siguiendo)
```

- **Lo bueno:** el `S` normal bajó de 42 a ~14–15 ms. El arreglo de I²C funciona.
- **Lo nuevo:** de vez en cuando un ciclo tarda 100–270 ms; el puente pierde ciclos (35–37 Hz en
  lugar de 50) y cuenta ~1 error por segundo (respuestas que no llegaron en 100 ms).
- **Es seguro:** una respuesta tarde se tira, con 3 seguidas el puente deja de mandar pulsos, y
  el watchdog dejó la pata quieta donde iba. No hubo reinicios.

**¿Dónde se pierde el tiempo?** Hay dos sospechosos: la ESP32 (una escritura de I²C que se
cuelga; el `Wire` de la ESP32 espera hasta 50 ms antes de rendirse) o la compu y el USB (Windows
o el driver detienen el puerto un rato). Una pista: un `S` que **sí** contestó tardó 218 ms, y el
puente solo espera 100 ms (+20 ms de la última lectura). Si la ESP32 hubiera tardado 218 ms en
contestar, el puente se habría rendido antes. Para que conteste "bien" en 218 ms, el puente tuvo
que estar detenido en otra parte (al escribir, o Windows no lo dejó correr). Otra: en la prueba 08
(17:54) se perdió una línea de 6584 en el USB, sin que la ESP32 hiciera nada raro. Pero no basta
con pistas: hay que medirlo en los dos lados.

**Diagnóstico (firmware y puente del 9 nov, después de las 17:42):**
- La ESP32 agrega a `?` sus propios tiempos (`DIAG=1`, ver `docs/hardware.md`): cuántos `S`
  entendió (`NS`), su `S` más lento (`SMAX`), la línea más lenta en llegarle (`LINMAX`), la vuelta
  de `loop()` más larga (`VUELTAMAX`) y las fallas de I²C (`I2CERR`, `I2CCOD`, `I2CMAX`).
- El puente mide lo suyo: cuánto tarda en **mandar** cada `S` y cuánto **espera** la respuesta. Cada
  10 s, después de la línea de Hz, imprime las dos partes y una conclusión:

```
[puente]    compu: mandar S peor 0.1 ms · esperar su respuesta peor 210.0 ms · '?' peor 9.0 ms
[puente]    ESP32: S peor 5.4 ms (I²C peor 4.5 ms) · línea peor 5.6 ms · vuelta peor 0.3 ms · I²C fallas 0 · S perdidos 0
[puente]    → la ESP32 contestó rápido (lo peor suyo: 6 ms de 210 ms): lo lento fue la compu o el USB
```

  `S perdidos` compara los `S` que mandó el puente con los que contó la ESP32: es exacto porque el
  `?` va detrás de los `S` en el mismo cable y la ESP32 los atiende en orden.
- El aviso del watchdog ahora dice si el puente estuvo trabado (todo el proceso detenido: la
  compu) o si dejaron de llegar los comandos (`poses.py` o el mux).

Con eso se decide el arreglo: si es la ESP32, el I²C (cables, resistencias de pull-up, un timeout
más corto); si es el USB, el cable, el puerto, el ahorro de energía de Windows o un hub.

### Con el diagnóstico (9 nov, 18:35): sin pausas

Grabación `pc/agmen/grabaciones/agmen-2026-11-09_18-35-25.mcap`: 2 min 20 s, 14 cambios de pose
(parado, echado, sentado), el mismo cable y el mismo puerto que a las 17:42.

| Qué | Resultado |
|---|---|
| Ciclos por segundo | **49.3–49.9 Hz** (a las 17:42: 35–37) |
| Errores, reinicios, `S` perdidos | **0, 0, 0** |
| `S`, lo normal / lo peor | 13.5–22 ms / 23–40 ms (a las 17:42 lo peor fue 218 ms) |
| Compu: mandar un `S` / esperar su respuesta, lo peor | 2.7–18 ms / 23–40 ms |
| ESP32: su `S` más lento / el I²C más lento | 11.8 ms / 4.3–4.7 ms, **0 fallas de I²C** |
| ESP32: la vuelta de `loop()` más larga | ~14 ms en cada informe |
| Watchdog | Una vez, al apagar `poses.py` a propósito: "el puente no estuvo trabado: dejaron de llegar" |

Las pausas de 100–270 ms **no aparecieron**, y nunca se llegó a los 50 ms que activan la
conclusión del informe. El firmware de las 17:42 y el de ahora atienden el `S` igual (el
diagnóstico solo agrega cuentas), así que lo de las 17:42 fue algo pasajero que no se puede saber
ya: probablemente la compu o el USB (el `S` que contestó en 218 ms apunta ahí). El diagnóstico se
queda puesto: si vuelve, el informe dirá de qué lado fue.

Lo que sí se ve:
- **Un ciclo saltado cada ~1 s** ("Tu función tardó 28–40 ms y el periodo es 20 ms"). Es el ciclo
  que manda el `S` y además el `?` de cada segundo: ~14 ms + ~15 ms no caben en 20 ms (la línea
  de `?` creció con el diagnóstico). Por eso 49.5 Hz y no 50. No hace daño: el paso de la rampa
  se calcula con el tiempo real (`dt`).
- **La vuelta de `loop()` de ~14 ms** (lo normal en el arnés es 0.2 ms). Fuera de los comandos,
  lo único que hace el `loop()` es leer la auxiliar byte por byte; la sospecha es que eso es lento
  en la ESP32 de verdad. Mientras tanto, un `S` que llega puede esperar hasta 14 ms. Hoy no
  estorba (todo cabe en los 100 ms del puente).
- En los intentos de la prueba 08 hubo una vuelta de 190 ms (`VUELTAMAX=189506`) y dos líneas
  dañadas por el USB (una muestra y, muy probablemente, un encabezado). Aquí no se repitió nada de
  eso.

**Después, la prueba de carga:** el perro colgado de una correa, bajándolo poco a poco hasta que
apoye en "parado". Ahí se ve cuánto se hunde cada rodilla (lo mandado contra lo que mide el cable
verde), la corriente de la fuente y si los servos se calientan. El piso va al final.

## Pendiente

- [x] Volver a medir los límites de la FL (P0, P1, P2) con `ver_servos.py` (y de paso los de las
      otras 9, 9 de noviembre).
- [ ] El peso del perro. Se anotó 785 g, pero no cuadra: solo los 12 servos ya pesan 720–744 g
      (60–62 g cada uno). Hay que volver a pesarlo.
- [x] La prueba en el aire con las 4: quedan iguales (resultados arriba).
- [x] Arreglar el `S` lento al mover: firmware del 9 de noviembre (una sola transacción de I²C).
      Medido: 14–15 ms (antes 42).
- [x] Las pausas de 100–270 ms después del arreglo: con el diagnóstico (18:35) no aparecieron;
      49.5 Hz, 0 errores. El diagnóstico se queda.
- [ ] La vuelta de `loop()` de ~14 ms en la ESP32: medir cuánto tarda leer la auxiliar.
- [x] Revisar si las rodillas FR, RR y RL quedan rectas en el tope de su servo: no, pasan unos
      grados más (punto 3).
- [ ] La prueba de carga, colgado.
