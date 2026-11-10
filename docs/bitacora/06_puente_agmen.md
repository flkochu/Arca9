# 06 · Puente entre Arca9 y Agmen

| | |
|---|---|
| Fecha | 5 de noviembre de 2026, en simulación. La primera prueba con el perro está pendiente (procedimiento abajo) |
| Perro | Sin el perro: una ESP32 falsa (`pc/agmen/esp32_falsa.py`) y el firmware de verdad en el arnés (`pruebas/arnes_firmware`) |
| Firmware | `firmware/esp32_principal` con el protocolo v3 (comando `S`), sin cambios |
| Agmen | 0.10.1 (`pip install agmen`) |
| Código | [`pc/agmen/`](../../pc/agmen/) |
| Pruebas | [`pruebas/puente_agmen/`](../../pruebas/puente_agmen/): 66 revisiones, 0 fallas |
| Resultado | **En simulación, el panel de Agmen ve los 12 ángulos a 50 Hz y maneja al perro.** Cada `S` de ida y vuelta tarda ~11 ms. El panel le gana a `poses.py` y lo suelta sin saltos (0.7° como máximo entre dos comandos seguidos) |

> **Actualización del 9 de noviembre (Agmen 0.13.0 y las 4 patas, nota 07).** Agmen resolvió los
> puntos 1, 2, 4 y 7 de `PENDIENTES_AGMEN.md`, así que:
> - `manejo.py` y la tarjeta Manejar del panel (las flechas) **ya no existen**. En su lugar, la
>   tarjeta **Poses** del panel (`[[web.command]]` en `robot.toml`) tiene los botones *parado*,
>   *sentado* y *echado*. Mandan un `Arca9Pose` a `/arca9/pose`: `poses.py` va a esa pose con su
>   interpolación suave y se queda ahí al soltar el botón. Los botones no llevan ángulos (no se
>   copia ninguna calibración a `robot.toml`).
> - `poses.py` sabe quién manda por `/mux/mando` (`MuxEstado`, Agmen 0.13). Ya no adivina con
>   `--mando-mayor` ni copia el `hold_ms` del mux. La entrada `panel` del mux queda libre para lo
>   que siga (deslizadores, marcha, política de aprendizaje por refuerzo).
> - `/arca9/articulaciones` y `/arca9/estado` tienen `max_gap_ms`: si dejan de llegar, el panel y
>   `agmen run` lo avisan.
> - El perro de verdad tiene las 4 patas medidas, y `robot_simulado.toml` usa esas mismas 4.
> - Pruebas: 74 revisiones, 0 fallas. Las secciones de abajo sobre `manejo.py` quedan como
>   historia.

## Por qué

Hasta la prueba 05, cada prueba era un script que hablaba solo con la ESP32. Para lo que sigue
(poses, marcha, después aprendizaje por refuerzo) hace falta que varios programas trabajen con el
perro a la vez, y verlo y controlarlo mientras tanto. Agmen ya hace eso: nodos que se hablan por
topics, un solo comando que los levanta (`agmen run`), un panel web y un `[[mux]]` que decide
quién manda.

El puente es la pieza que faltaba en medio: un nodo de Agmen que habla con la ESP32 principal. El
comando `S` se agregó en la prueba 05 justo para esto. Con una línea de ida y una de vuelta
manda los 12 pulsos y recibe las 12 lecturas.

## Cómo queda

```
panel (Poses) ──► /arca9/pose ──► poses.py ──► /arca9/comando/poses ─┐ prioridad 1
                                                                     ├─► mux "mando" ──► /arca9/comando ──► puente.py ──S──► ESP32
(libre)  ──────────────────────────────────► /arca9/comando/panel ─┘ prioridad 10        │
                                                                                         ├──► /arca9/articulaciones
                                     mux "mando" ──► /mux/mando (quién manda)            └──► /arca9/estado
```

| Archivo | Qué hace |
|---|---|
| `arca9.agmen` | Los mensajes (abajo). `agmen gen pc/agmen/arca9.agmen` los muestra |
| `puente.py` | El nodo que habla con la ESP32 por USB con `S`, a 50 Hz |
| `poses.py` | Parado, sentado y echado, con interpolación suave. Es el "modo autónomo" |
| ~~`manejo.py`~~ | Se borró el 9 de noviembre: lo reemplazan los botones Poses del panel (Agmen 0.13) |
| `perro.py` | Las patas medidas (`pc/pata.py`), sus canales y límites, y pies ↔ servos. Lo usan los otros tres |
| `esp32_falsa.py` | Se hace pasar por el puerto serie de la ESP32 (`--simulado` y las pruebas) |
| `robot.toml` · `robot_simulado.toml` | Para `agmen run`: con el perro (COM6) y sin el perro |
| `PENDIENTES_AGMEN.md` | Lo que le faltó o le sobró a Agmen |

## Los mensajes

| Mensaje | Topic | Campos |
|---|---|---|
| `Arca9Articulaciones` | `/arca9/articulaciones` (lo publica el puente, 50 Hz) | `f32[12] q`: grados reales de cada servo; NaN = sin lectura creíble |
| `Arca9Comando` | `/arca9/comando` (lo sigue el puente) | `f32[12] q`: grados reales objetivo; `bool[12] activo`: false = ese servo no se toca |
| `Arca9Estado` | `/arca9/estado` (lo publica el puente, 10 Hz) | `conectado`, `proto`, `reset`, `seg`, `aux`, `modo`, `reinicios`, `errores`, `ciclos`, `ciclo_ms`, `vel_max`, `limitado[12]`, `bloqueado[12]` |
| `Arca9Pose` | `/arca9/pose` (lo mandan los botones del panel) | `u8 pose`: `PARADO` (0), `SENTADO` (1) o `ECHADO` (2) |

Todos los ángulos van en **grados reales** (`pc/escala.py`: tabla de la prueba 01 y factor de la
prueba 02). Los canales son los de `docs/hardware.md`: FL 0–2, FR 3–5, RL 6–8, RR 9–11.

El panel muestra los enums como números (`PENDIENTES_AGMEN.md` #3):

| `modo` | | `reset` | |
|---|---|---|---|
| 0 | SUELTO: todavía no llega ningún comando | 0 | DESCONOCIDO |
| 1 | SIGUIENDO | 1 | ENCENDIDO (USB, abrir el puerto o EN) |
| 2 | QUIETO: watchdog | 2 | VOLTAJE_BAJO (brownout) |
| 3 | REINICIADA: la ESP32 se reinició | 3 | FALLA |
| 4 | SIN_ESP32: no contesta | 4–7 | WATCHDOG, SOFTWARE, EXTERNO, OTRO |

## El puente

En cada ciclo (50 Hz, `node.every(0.02)`), el puente calcula los 12 pulsos, manda un `S` y
convierte las 12 lecturas a grados reales. Usa la clase `ESP32` de `pc/arca9_serial.py` y le agrega
el ciclo de `S` (`ESP32Puente`). El timeout es de 100 ms en lugar de 1 s, una respuesta tardía no
se confunde con la del ciclo siguiente, y se da cuenta si la placa se reinicia.

`S` no hace rampas: manda el pulso tal cual. Por eso el puente se encarga de que cada cambio sea
chico:

| Protección | Cómo |
|---|---|
| Límites | Cada servo se recorta a los de su pata en `pc/pata.py` (los medidos menos 5°, y la rodilla sin pasar de recta). `estado.limitado` marca los que se recortaron |
| Patas sin medir | Un servo sin `datos/*/pata_<xx>.json` no tiene límites, así que no se mueve aunque el comando lo pida (`estado.bloqueado`). **Hoy solo está medida la FL**: con el perro, el puente solo mueve los canales 0, 1 y 2 |
| Velocidad | El pulso de cada servo avanza a lo más `--vel-max` °/s (60 por defecto, tope 120). A 50 Hz son 1.2° por ciclo. La prueba 04 llegó a 240 °/s sin saturar |
| Arranque | Cada servo empieza desde donde lo mide su cable verde, así que no da saltos. Se prenden uno por ciclo (los 12 en 0.24 s), para no pedirle a la fuente los 12 picos a la vez. Un servo sin lectura creíble (165–1950 mV) no se prende |
| Watchdog | Sin comandos en 0.5 s, se queda quieto donde va. No los suelta, porque un perro de pie se caería |
| Reinicio de la ESP32 | Se detecta por su línea de arranque (`OK1-RETRO` sin pedirla) o porque `SEG` regresó (un `?` cada segundo, por si la línea se perdió). Lo publica (`modo` 3, `reinicios`, `reset`), imprime el aviso `REINICIO` de `arca9_serial.py` y deja de mandar pulsos (al arrancar, la ESP32 suelta los servos). Para seguir: revisar la fuente y reiniciar el nodo puente (↻) |
| La ESP32 no contesta | Después de 3 `S` seguidos sin respuesta: `conectado` = false, `modo` 4, y ningún pulso nuevo hasta que vuelva. Al volver, primero pregunta `?` por si se reinició |
| Al cerrar | Ctrl+C, ■ en el panel o `agmen run` que termina: manda `D` |

Si el puerto USB desaparece, el puente lo publica y termina con error. No se reinicia solo
(`restart = "never"`): que lo vuelva a prender una persona.

## Las poses

`poses.py` lleva al perro de una pose a otra. Las poses son posiciones del pie (cm desde la
cadera, x hacia el frente, z hacia abajo) con la abducción en 0, como en la prueba 05:

| Pose | Delanteras | Traseras |
|---|---|---|
| parado | x = 2, z = 18.5 (la pose de pie de la prueba 05) | igual |
| sentado | x = 2, z = 19.5 | x = 5, z = 11 (dobladas) |
| echado | x = 5, z = 11 | x = 5, z = 11 |

Para echarse, el pie tiene que ir 5 cm adelante: con el pie en x = 2, z = 12, la cadera ya pasa de
su límite de 145°.
Las tres poses quedan a 14° o más de los límites. Antes de mover nada, `poses.py` revisa que cada
pose quepa en los límites de cada pata.

La interpolación va de articulación a articulación, con un perfil de jerk mínimo: empieza y
termina sin velocidad ni aceleración. La duración sale de la articulación que más se mueve, para
que ninguna pase de `--vel` (30 °/s por defecto, la mitad del `--vel-max` del puente: así el puente
no la recorta).

Empieza desde donde está el perro (espera la primera lectura de `/arca9/articulaciones`). Si el
panel toma el mando, publica lo mismo que el panel; medio segundo después de que el panel se calla,
interpola desde ahí. Así no hay salto cuando el mux le regresa el mando.

## Manejar desde el panel (hasta el 9 de noviembre)

El panel de Agmen solo sabe mandar `MotorCommand{left, right}`, el comando de un robot de ruedas
(`PENDIENTES_AGMEN.md` #1). `manejo.py` lo traduce:

| Tecla | Qué hace | Al 100 % del deslizador |
|---|---|---|
| ↑ (W) | Sube el cuerpo: los pies bajan | 3 cm/s |
| ↓ (S) | Baja el cuerpo | 3 cm/s |
| → (D) | Se inclina a la derecha: los pies de la derecha suben y los de la izquierda bajan | 2 cm/s, hasta ±3 cm |
| ← (A) | Se inclina a la izquierda | |

Cada vez que empiezas a manejar, parte de lo que el perro está siguiendo (la salida del mux). Al
soltar, el panel manda ALTO: el perro se queda donde va, y 0.3 s después el mux le regresa el mando
a `poses.py`. Cada pie se queda entre z = 10.5 y 21 cm, ningún servo sale de sus límites y ninguno
cambia más de 3° de un mensaje al siguiente.

## El mux

En `robot.toml`, un `[[mux]]` llamado "mando" junta las dos fuentes en `/arca9/comando`: el panel
(prioridad 10) y `poses.py` (prioridad 1), con `hold_ms = 300`. Mientras manejas, los comandos de
`poses.py` se descartan. El mux nunca inventa comandos: si nadie manda, el watchdog del puente lo
deja quieto.

## Sin el perro (`--simulado`)

`agmen run pc/agmen/robot_simulado.toml` corre todo con:

- **La ESP32 falsa.** Se hace pasar por el puerto serie, así que la clase `ESP32` de
  `arca9_serial.py` le escribe y le lee como al USB: el código que se prueba es el mismo que
  habla con el perro. Contesta `?`, `S`, `V`, `P` y `D` como el firmware. Cada servo va ~100 ms
  atrás, como en la prueba 04 (70 ms de tiempo muerto y 30 ms de primer orden, hasta 300 °/s), y
  las lecturas salen de la tabla de la prueba 01 al revés. Las respuestas tardan lo que tardarían
  a 115200 baud.
- **Las 4 patas copiadas de la FL**, solo en la simulación (el perro de verdad solo tiene medida la
  FL).

Para ensayar un reinicio, agrega `--reinicio-en 20` al `cmd` del puente en
`robot_simulado.toml`: la ESP32 falsa se reinicia por voltaje bajo a los 20 s.

## Resultados (en simulación)

`python pruebas/puente_agmen/prueba_puente.py`: **66 revisiones, 0 fallas** (las 4 partes están en
su [README](../../pruebas/puente_agmen/README.md)).

| Qué | Resultado |
|---|---|
| `/arca9/articulaciones` | 50.0 Hz (`agmen topic hz`: entre 11 y 30 ms entre mensajes) |
| `S` de ida y vuelta, con la ESP32 falsa | 11.1–11.8 ms; el peor, 13–20 ms |
| `S` con el firmware de verdad (arnés) | 149 ciclos en 3 s, sin errores. El arnés no corre en tiempo real: sirve para el protocolo, no para medir el USB |
| Llega a "parado" | A 0.3–0.4° de la pose |
| El panel le gana a `poses.py` | Con ↓ al 60 % durante 1.5 s, los pies suben de z = 18.5 a 15.7 cm; al soltar se queda en 15.7 |
| Mayor cambio entre dos comandos seguidos al tomar y soltar el mando | 0.7° |
| Las 4 flechas hasta los topes | 1.5°, con los pies entre 10.5 y 21.0 cm |
| Reinicio de la ESP32 | `modo` 3 y `reset` 2 en `/arca9/estado`, el aviso `REINICIO` en los logs, servos sueltos; con ↻ vuelve a `modo` 1 |
| Watchdog | Al apagar `poses.py` (■): `modo` 2 en 0.5 s |
| Al apagar el puente | `■ servos sueltos (D)` |

**Correcciones mientras se hacía:**

- **Un salto de 8.9° en `manejo.py`.** Al subir el cuerpo con ↑, las patas delanteras llegaron casi
  a estiradas (24 cm). Ahí, 1.5 mm de pie son varios grados de rodilla. El puente lo frenaba a su
  `--vel-max`, pero el comando no debía pedirlo. Ahora cada pie se queda entre 10.5 y 21 cm, y ningún
  servo cambia más de 3° por mensaje. Con eso bajó a 1.5°.
- **Avisos falsos al reiniciar el puente.** Con ↻ mientras `poses.py` sigue mandando, el comando
  llegaba antes de la primera lectura, y el puente avisaba "sin lectura creíble" en los 12 canales.
  Ahora espera la primera lectura.

## Procedimiento: la primera prueba con el perro

**Con el perro colgado y despacio.** Hoy solo la FL tiene límites medidos, así que el puente solo
mueve los canales 0, 1 y 2. Los otros 9 se quedan sueltos aunque tengan pata.

1. Instala lo que falta (agrega `agmen` y `pyserial`). El firmware ya tiene `PROTO=3` desde la
   prueba 05. Cierra el Monitor Serie.
   ```
   pip install -r pc/requirements.txt
   ```
2. Ensaya sin el perro y corre las pruebas:
   ```
   python pruebas/puente_agmen/prueba_puente.py
   agmen run pc/agmen/robot_simulado.toml
   ```
   En el panel, clic en `/arca9/articulaciones` y en `/arca9/estado` para graficarlos. Maneja con
   las flechas, apaga `poses` (■) y mira el watchdog, y reinicia el puente (↻). Para ensayar un
   reinicio de la ESP32, usa `--reinicio-en 20` (arriba).
3. **Monta el perro colgado** de un soporte, con las patas sin tocar nada (ni la mesa ni el
   soporte) en todo su recorrido. Fuente de 6 V (5.99 V), con su interruptor a la mano: **apagar la
   fuente de los servos es el paro de emergencia** (no el USB).
4. Revisa `pc/agmen/robot.toml`: el puerto (`--port COM6`), `--vel-max 20` en el puente y `--vel
   15` en poses.
5. Arráncalo:
   ```
   agmen run pc/agmen/robot.toml
   ```
   En los logs del puente debe salir `ESP32: OK1-RETRO ... PROTO=3`, las patas (la FL con sus
   límites y las otras "sin medir") y `listo`. `poses` empieza apagado: nada se mueve, `modo` 0.
6. Con los servos sueltos, mueve la FL con la mano. En `/arca9/articulaciones`, `q[0]`, `q[1]` y
   `q[2]` deben cambiar como en `rampas.py --revisar`: con la pata recta, ~88°, ~87° y ~163°.
7. Prende `poses` (▶). La FL va a "parado" a 15 °/s y luego sigue la secuencia (parado, sentado,
   parado, echado), con 3 s en cada pose. En el inspector (Mensaje) de `/arca9/estado`: `modo` 1,
   `errores` 0, `reinicios` 0 y `limitado` en false (en las gráficas casi no se ven:
   `PENDIENTES_AGMEN.md` #5). Cada 10 s, el puente dice en los logs a cuántos Hz va y cuánto tarda
   `S`.
8. Maneja con el deslizador del panel en 30 %. Mantén ↓: el pie de la FL sube. Suelta: se queda, y
   en 0.3 s `poses` retoma.
9. Apaga `poses` (■): el puente avisa el watchdog y la pata se queda donde va (`modo` 2). Apaga el
   puente (■): manda `D` y la pata se suelta.
10. Si algo se mueve raro, apaga la fuente de los servos. ■ en el puente o Ctrl+C en la terminal
    también los sueltan (`D`).
11. Si todo salió bien, sube poco a poco: `--vel-max 30` y `--vel 20`, y luego los valores por
    defecto (60 y 30).
12. Anota aquí los Hz, cuánto tarda `S` (y el peor), los errores, los reinicios y cualquier cosa
    rara.

**Qué esperar:** en la simulación, `S` tarda ~11 ms por los bytes a 115200 baud. Con el USB de
Windows puede tardar más. Si el puente dice que `S` tarda más de ~18 ms, o Agmen avisa
"ciclos saltados", usa `--hz 40`.

**Si la ESP32 se reinicia** (`modo` 3, `reset` 2): es casi siempre la fuente. Apaga `poses` (■),
revisa la fuente y la tierra común, reinicia el puente (↻) y prende `poses` otra vez. Así `poses`
empieza desde donde quedó la pata. Si reinicias el puente con `poses` prendido, la pata va de
donde quedó a lo que pide `poses` a la velocidad del puente, sin la interpolación suave.

## Conclusiones

- **En simulación, el puente funciona de punta a punta con Agmen.** `agmen run` levanta el robot, el
  panel ve los 12 ángulos y el estado a 50 Hz, y el mux le da el mando al panel sin saltos.
- **La última protección es el puente**, no los nodos. Aunque un nodo mande un salto, el pulso no
  avanza más de `--vel-max`, no sale de los límites de `pata.py` y no toca una pata sin medir.
- **Para manejar al perro completo falta medir las otras 3 patas.** Hoy el puente solo mueve la FL.
- **Agmen sirvió casi sin rodeos.** Lo que más falta es que el panel pueda mandar algo que no sea
  `MotorCommand` (`PENDIENTES_AGMEN.md`).

## Pendiente

- [ ] Primera prueba con el perro (colgado, FL, `--vel-max 20`) y anotar sus números aquí.
- [ ] ¿Cabe el `S` en 20 ms con el USB de Windows? Si no, `--hz 40`.
- [x] Medir FR, RL y RR como la FL, con el sentido de cada servo (nota 07, `datos/2026-11-09_patas`).
- [ ] Con las 4 patas: las poses en el suelo (con carga), empezando por echado → parado.
- [ ] (Opcional) Limitar también la aceleración en el puente.
- [ ] (Opcional) El adelanto de 100 ms de la prueba 05, cuando haya marcha.
