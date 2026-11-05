# Pendientes de Agmen (vistos al hacer el puente de Arca9)

Lo que le faltó o le sobró a Agmen 0.10.1 al hacer `pc/agmen/` (nota 06 de la bitácora). Aquí no
se cambió nada de Agmen: donde hizo falta, se rodeó desde este lado, y abajo dice cómo. Cada punto
dice cómo verlo.

| # | Qué | Tipo | Cómo se rodeó aquí |
|---|---|---|---|
| 1 | El panel solo manda `MotorCommand` | Falta | `manejo.py` traduce las flechas a `Arca9Comando` |
| 2 | Nadie sabe quién tiene el mando del mux | Falta | `poses.py` escucha la entrada del panel |
| 3 | Los enums se ven como números | Falta | Ninguno: `modo: 3` hay que leerlo en `arca9.agmen` |
| 4 | Un `f32` NaN rompe el JSON de Foxglove | Error | Ninguno: con el panel de Agmen no pasa |
| 5 | Un arreglo de 12 sale en una sola gráfica, y el estado mezcla escalas | Falta | Ninguno: `modo` y `errores` se leen en el inspector |
| 6 | `robot.toml` no tiene variables | Falta | Dos archivos: `robot.toml` y `robot_simulado.toml` |
| 7 | Dos `agmen run` en la misma máquina | Error | La prueba revisa que no haya otro hub antes de empezar |
| 8 | Un nodo de hardware no puede pedir reloj real | Falta | `robot.toml` no usa `[clock]` |
| 9 | El ejemplo `arca9.agmen` de Agmen ya no es el de Arca9 | Sobra | — |
| 10 | Sin hub, cada publicador repite el mismo aviso | Detalle | — |

## 1. El panel solo sabe manejar un robot de ruedas

`[web.teleop]` solo acepta `MotorCommand{f32 left;f32 right}` (`run.py`: "por ahora el único tipo
soportado es 'MotorCommand'"), y la tarjeta **Manejar** dice "izq / der" y "Suelta = el robot
frena". Para un perro hace falta mandar 12 ángulos, o elegir una pose.

- **Aquí:** `manejo.py` recibe el `MotorCommand` del panel y lo traduce: ↑/↓ suben y bajan el
  cuerpo, ←/→ lo inclinan. Funciona, pero las flechas no dicen eso en el panel.
- **No se puede elegir una pose desde el panel** (parado, sentado, echado). `poses.py` las toma de
  sus argumentos en `robot.toml`; desde el panel solo se prende (▶) o se apaga (■).
- **Lo que serviría:** mandar cualquier tipo con esquema desde el panel. Por ejemplo, deslizadores
  armados a partir del esquema (como las gráficas), o botones que publiquen un mensaje fijo
  declarado en `robot.toml`. Ya está en la tabla del RFC 0003 §8 ("cualquier tipo de comando con
  esquema"). También, poder cambiar los letreros de la tarjeta Manejar.

## 2. El mux no dice quién manda

El mux escribe en su log `manda 'panel' (prioridad 10)`, pero ningún nodo lo puede saber: la
salida no dice de qué entrada viene (el mux pone su propio `source_id` y renumera `seq`, a
propósito).

- **Aquí:** `poses.py` necesita saberlo para no dar un salto cuando el panel suelta el mando:
  mientras el panel manda, `poses.py` publica lo mismo que él y, al soltar, interpola desde ahí.
  Lo resuelve escuchando directo `/arca9/comando/panel`, la entrada de más prioridad
  (`--mando-mayor`), y copiando el `hold_ms` del mux. Si alguien cambia el `[[mux]]` en
  `robot.toml`, también tiene que cambiar los argumentos de `poses.py`.
- **Lo que serviría:** que el mux publique su estado en un topic (por ejemplo `/mux/mando`, con la
  entrada que manda), o que lo muestre el panel.

## 3. Los enums se ven como números

`Arca9Estado.modo` y `.reset` son enums en `arca9.agmen`, pero en el esquema viajan como `u8`. En
el panel, en `agmen topic echo` y en Foxglove se ven `modo: 3` y `reset: 2`, no `REINICIADA` y
`VOLTAJE_BAJO`:

```
agmen topic echo /arca9/estado --count 1
  reset: 1
  modo: 1
```

Fue a propósito (RFC 0005, D4: el enum viaja como su entero y el nombre vive solo en el código), y
el RFC deja para después lo que tendría que ir en el anuncio del esquema (unidades y rangos). Los
nombres de los enums entrarían en ese mismo cambio. Mientras, hay que leer los números en
`arca9.agmen`.

## 4. Un NaN rompe los mensajes que van a Foxglove

`/arca9/articulaciones` pone NaN en un servo sin lectura creíble (por ejemplo, con la auxiliar
desconectada, en 9 de los 12). El panel de Agmen lo aguanta: ese punto no se dibuja. Pero el puente
le manda a Foxglove `json.dumps(...)` sin `allow_nan=False`, y sale un JSON que no es JSON:

```python
>>> json.dumps(foxglove_row(Layout("agmen/1 Arca9Articulaciones{f32[12] q}"), {"q": [nan] + [1.0]*11}))
'{"q":[NaN,1.0,1.0,...'
```

Un parser estricto (el `JSON.parse` del navegador) lo rechaza completo, así que Foxglove pierde
todo el mensaje, no solo ese servo. **Lo que serviría:** cambiar NaN e infinitos por `null` en
`schema.jsonable` (lo usan Foxglove y `agmen echo --json`).

## 5. Doce valores en una sola gráfica, y escalas mezcladas

El panel grafica `q[0…11]` en una sola gráfica de 12 líneas. Para un perro se leería mejor por
pata (FL abducción, cadera, rodilla…), o eligiendo qué elementos ver.

En `/arca9/estado` pasa algo parecido: el panel junta los campos sueltos de 8 en 8, sin importar
su escala. `conectado · proto · reset · seg · modo · reinicios · errores · ciclos` quedan en una
misma gráfica. `ciclos` sube de 50 en 50 por segundo y aplasta a los demás: `modo` y `errores`, lo
que más importa, no se ven (solo en la leyenda y en el inspector).

**Lo que serviría:** elegir campos o elementos de un arreglo para graficar, o ponerles nombre;
separar las gráficas por escala; o mostrar los campos enteros y booleanos como números, no como
curvas. No hay vista del perro en 3D (es del nivel 9 del roadmap): por ahora "ver el perro" son
estas curvas.

## 6. `robot.toml` no tiene variables

Para cambiar solo `--port COM6` por `--simulado` hay que copiar el archivo completo. Por eso
`robot.toml` y `robot_simulado.toml` son casi iguales, y si se cambia uno hay que acordarse de
cambiar el otro. **Lo que serviría:** variables (`agmen run robot.toml --set puerto=COM6`), un
`include`, o perfiles dentro del mismo archivo.

## 7. Dos `agmen run` en la misma máquina

Si ya hay un robot corriendo, el segundo `agmen run` reusa su hub (lo avisa: "los [[topic]] de este
robot.toml no se revisan"). Pero su panel no puede escuchar en `[web] listen` (127.0.0.1:7400) y
truena con un traceback sin explicar:

```
Exception in thread panel-udp:
  ...
  File ".../agmen/panel/server.py", line 170, in udp_receiver
    receivers.append(RawSubscriber(listen))
OSError: [Errno 98] Address already in use
```

Después, los dos robots se mezclan, porque los topics se llaman igual (el `poses.py` de uno manda
al puente del otro). Pasó al correr las pruebas con otro robot simulado prendido. **Lo que
serviría:** un mensaje claro ("ya hay un robot en esta máquina: 'Arca9 (simulado)'"), y no
arrancar si los dos declaran los mismos topics. **Aquí:**
`pruebas/puente_agmen/prueba_puente.py` revisa que no haya otro hub antes de levantar el robot.

## 8. Un nodo de hardware no puede pedir el reloj real

`node.every(0.02)` sigue el reloj de Agmen. Si alguien pone `[clock] mode = "sim"` en el
`robot.toml` del perro, el ciclo de `S` seguiría al tiempo simulado: en pausa no se manda nada y el
watchdog del puente no corre. No es peligroso (el firmware se queda con el último pulso), pero no
tiene sentido. **Lo que serviría:** que un nodo diga que necesita el reloj real, y que `agmen run`
avise si el robot está en simulación. **Aquí:** `robot.toml` no tiene `[clock]`, y el puente mide el
watchdog con `time.monotonic()`.

## 9. El ejemplo `arca9.agmen` de Agmen ya no es el de Arca9

`examples/mensajes/arca9.agmen` (capítulo 11 del tutorial) tiene `Articulaciones{q, kp, modo}` y
un `Estado` con voltaje y patas de apoyo. Los de verdad ahora están en `pc/agmen/arca9.agmen`
(`Arca9Articulaciones{f32[12] q}`, `Arca9Comando`, `Arca9Estado`). El ejemplo puede quedarse como
ejemplo, pero ya no conviene llamarlo "los mensajes de Arca9". Se podría apuntar al de este
repositorio.

## 10. Sin hub, el mismo aviso tres veces

`python puente.py --simulado` sin `agmen run` imprime "no hay un agmen hub en esta máquina…" tres
veces al mismo tiempo, una por cada publicador y suscriptor. Con uno por nodo bastaría.
