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
  que se detenga si llega un comando.

```
g++ -std=c++17 -Istubs -o arnes arnes.cpp
./arnes
```

Al final debe decir `TODO BIEN (0 fallas)`.

Con `./arnes --pty` (Linux o macOS), expone el USB de la principal en un pseudo-terminal, para
probar `pc/eficiencia_servos.py --port /dev/pts/N` de punta a punta.
