# Arnés de los firmwares

Este arnés compila `esp32_principal.ino` y `esp32_aux.ino` en la compu, sin las placas. Usa
versiones de prueba de Arduino, Wire y PCA9685 (carpeta `stubs/`) y une las dos ESP32 con un
UART virtual. Luego revisa el protocolo:

- `?`, `P`, `R`, `V`, `D` y el recorte del pulso;
- que las lecturas de la auxiliar lleguen a la principal;
- que se descarten las líneas dañadas;
- que todo siga funcionando aunque se llene el buffer;
- qué pasa si se desconecta la auxiliar y si se vuelve a conectar;
- el comando `G` (protocolo v2): datos completos en FL y por la auxiliar, que no dé saltos, que
  nunca pase de 2360 µs, los errores y que se detenga si llega un comando;
- los comandos `S` y `T` (protocolo v3): los 12 a la vez, `-1` y `0` en `S`, trayectorias de 3 y
  de 12 canales, con y sin grabar, que no dé saltos, pulsos fuera de rango, el límite de memoria y
  que se detenga si llega un comando;
- el I²C del PCA9685 (9 nov 2026): el PCA9685 de `stubs/` guarda sus registros y cuenta las
  transacciones. Se revisa que los ticks sean idénticos a los de `writeMicroseconds()` de Adafruit
  de 500 a 2360 µs, que el PRESCALE se lea una sola vez, que `S` mande los canales que cambiaron
  en una transacción (y ninguna si nada cambió), que un `S` reescriba lo que falló la vez anterior,
  y que `T` use a lo más una transacción por punto. El I²C de `stubs/` tarda lo que tardaría el
  cable (9 bits por byte a 100 kHz);
- el diagnóstico de `?` (`DIAG=1`): que esté al final sin cambiar lo de antes, que `NS` cuente los
  `S` (y no los dañados), que los `MAX` se reinicien en cada `?`, que una escritura de I²C colgada
  (código 5, 50 ms) se vea en `I2CERR`, `I2CCOD`, `I2CMAX` y `SMAX`, que una línea que llega en dos
  pedazos se vea en `LINMAX` y no en `SMAX`, y que `VUELTAMAX` no cuente lo que tarda un `G`.

```
g++ -std=c++17 -Istubs -o arnes arnes.cpp
./arnes
```

Al final debe decir `TODO BIEN (0 fallas)`.

Con `./arnes --pty` (Linux o macOS), expone el USB de la principal en un pseudo-terminal, para
probar `pc/eficiencia_servos.py --port /dev/pts/N` de punta a punta.
