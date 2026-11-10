# 2026-11-09 · Las 4 patas medidas

Nota: [docs/bitacora/07_cuatro_patas.md](../../docs/bitacora/07_cuatro_patas.md).

Medido por Adán con el perro armado, en el soporte y con `pc/ver_servos.py` (servos sueltos,
grados reales). Los `pata_xx.json` se armaron con estas medidas tal cual.

## Abducción hacia arriba (afuera), del 8 de noviembre

Los otros dos números son donde estaban la cadera y la rodilla en ese momento.

```
FL 36 144 87
FR 151 67 69
RL 151 150 86
RR 29 21 86
```

## Abducción hacia adentro, hasta que chocan las patas

```
RL 69.5 154 86
RR 128 22 81
FL 125 80 87
FR 58 97 69
```

## Límites de cada articulación (después de cambiar P6 ↔ P8)

```
P5 146 0      FR rodilla
P4 176 0      FR cadera
P3 151 17     FR abducción
P8 32 172     RL rodilla
P7 176 0      RL cadera
P6 152 18     RL abducción
P11 136 0     RR rodilla
P10 176 0     RR cadera
P9 164 28     RR abducción
```

## Límites de las 12, otra vez (9 de noviembre, en la tarde)

```
P1 -4    172
P2 175 15
P0 33   160
P4 174 -4
P3 22 145
P5 -5 146
P7 174 -4
P8 176 33
P6 143 40
P10 175 -6
P11 136 -6
P9 163 35
```

En los `pata_xx.json` van estos. En la abducción, el lado de afuera es el más estrecho de las dos
medidas, y el de adentro es donde choca con la otra pata (lista de arriba).

## Pata recta y pose de pie, del 9 de noviembre

```
                       Totalmente recto              Pose de pie
FL  P0 P1 P2           85.1   90.1  148.8            85.3  131.1   68.9
FR  P3 P4 P5           92.8   88.7    0.4            93.1   64.3   95.9
RL  P6 P7 P8           85.2   98.8  170.9            85.2  127.1   73.4
RR  P9 P10 P11         94.1   76.3    0.5            94.2   51.9   89.2
```

Peso anotado: 785 (no cuadra con los 12 servos, ~730 g; hay que volver a pesarlo).
