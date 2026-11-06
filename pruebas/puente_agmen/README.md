# Pruebas del puente con Agmen

Prueban `pc/agmen/` (el puente entre Arca9 y Agmen) sin el perro, en cuatro partes:

1. **La lógica del puente**, con la ESP32 falsa (`pc/agmen/esp32_falsa.py`) y un reloj que avanza a
   mano. Se revisa línea por línea lo que sale por el "USB":
   - que sin comandos no mueva nada;
   - que los servos se prendan uno por ciclo, desde donde los mide su cable verde;
   - que ningún pulso pase de `--vel-max` ni de los límites de `pc/pata.py`;
   - que un servo sin pata medida o sin lectura creíble no se mueva;
   - el watchdog de 0.5 s;
   - los reinicios de la ESP32, con y sin su línea de arranque;
   - las respuestas que no llegan, que llegan tarde o que llegan dañadas;
   - un firmware sin `PROTO=3`;
   - y que al cerrar mande `D`.
2. **Las patas y las poses**: archivos de patas mal hechos (canales equivocados, una pata que gira
   al revés), que las tres poses queden a 5° o más de los límites, y que la interpolación de
   `poses.py` no pase de `--vel` y empiece y termine sin velocidad.
3. **El robot completo** con `agmen run pc/agmen/robot_simulado.toml` (procesos de verdad, hub, mux y
   panel):
   - `/arca9/articulaciones` a ~50 Hz;
   - el perro llega a "parado";
   - las flechas del panel (`/api/teleop`) le ganan a `poses.py` y, al soltar, `poses.py` retoma
     sin saltos;
   - el watchdog cuando se apaga `poses.py`;
   - `D` cuando se apaga el puente.
4. **El firmware de verdad**: compila `pruebas/arnes_firmware` y lo corre con `--pty`. El puente se
   conecta con pyserial al pseudo-terminal, como a COM6, y hace el ciclo de `S` contra el
   `esp32_principal.ino` de verdad. Solo corre en Linux o macOS con g++; en Windows se salta.

```
pip install -r pc/requirements.txt
python pruebas/puente_agmen/prueba_puente.py              # todo (~20 s)
python pruebas/puente_agmen/prueba_puente.py --rapido     # solo las partes 1 y 2 (~1 s)
```

Al final debe decir `TODO BIEN (0 fallas)`.

La parte 3 levanta su propio robot. Antes de correrla, cierra cualquier otro `agmen run`: dos
robots con los mismos topics se mezclan, y la prueba se niega a correr si ya hay un hub.
