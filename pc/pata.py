#!/usr/bin/env python3
"""
La pata de Arca9: ángulos de articulación, cinemática y límites
================================================================
Lee las medidas de una pata (datos/<fecha>_pata_fl/pata_fl.json) y convierte entre:

    grados del servo  <->  ángulo de la articulación  <->  posición del pie

Convención de las articulaciones (0 = la pata recta, apuntando derecho hacia abajo):
  q_abd   abducción: + = la pata se abre hacia afuera del cuerpo
  q_cad   cadera:    + = el muslo va hacia el frente del perro
  q_rod   rodilla:   + = se dobla (0 = recta). La rodilla queda hacia atrás, como en Spot.

Posición del pie, en cm, desde el eje de la cadera, en el plano de la pata:
  x = hacia el frente del perro, z = hacia abajo
  rodilla = (L1·sin q_cad, L1·cos q_cad)
  pie     = rodilla + (L2·sin(q_cad + q_rod), L2·cos(q_cad + q_rod))
Con la abducción en 0 la pata se mueve en un plano vertical, a `hombro` cm del eje de la
abducción; la cinemática de ese plano es exacta.

Límites: los medidos menos 5° de margen, y la rodilla sin pasar de recta (no se dobla al revés).

Uso:
    from pata import Pata
    p = Pata()                         # el pata_fl.json más reciente de datos/
    q = p.ik(0.0, 17.5)                # ángulos (q_cad, q_rod) para el pie en x=0, z=17.5 cm
    th = p.servo(0.0, *q)              # grados reales de los 3 servos (abd, cad, rod)
    p.dentro(th)                       # ¿dentro de los límites?

    python pata.py                     # muestra la pata y prueba la cinemática
"""
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(HERE)
MARGEN = 5.0                 # ° antes de donde choca
MARGEN_RECTA = 2.0           # ° antes de que la rodilla quede recta
ART = ("abd", "cad", "rod")


def buscar_pata(nombre="pata_fl.json"):
    cands = sorted(glob.glob(os.path.join(RAIZ, "datos", "*", nombre)))
    return cands[-1] if cands else None


class ErrorPata(ValueError):
    pass


class Pata:
    def __init__(self, ruta=None):
        ruta = ruta or buscar_pata()
        if ruta is None:
            raise FileNotFoundError("no encontré datos/*/pata_fl.json")
        with open(ruta, encoding="utf-8") as f:
            d = json.load(f)
        self.fuente = os.path.relpath(ruta, RAIZ).replace("\\", "/")
        self.nombre = d.get("pata", "FL")
        self.canales = [d["canales"][a] for a in ART]
        m = d["medidas_cm"]
        self.L1, self.L2, self.hombro = float(m["muslo"]), float(m["espinilla"]), float(m["hombro"])
        recta = d["pose_recta_grados"]
        de_pie = d["pose_de_pie_grados"]
        # Cero de cada articulación = la pata recta. En abducción se promedian las dos poses
        # (las dos tienen la pata vertical vista de frente).
        self.cero = {"abd": (recta["abd"] + de_pie["abd"]) / 2, "cad": recta["cad"], "rod": recta["rod"]}
        # Sentido: grado del servo = cero + signo · q. Al subir el servo: abd se mete (q_abd baja),
        # cad va hacia atrás (q_cad baja), rod se estira (q_rod baja). Los tres dan −1.
        self.signo = {"abd": -1.0, "cad": -1.0, "rod": -1.0}
        lim = d["limites_grados"]
        self.lim = {a: [float(lim[a][0]) + MARGEN, float(lim[a][1]) - MARGEN] for a in ART}
        # la rodilla no pasa de recta: q_rod >= MARGEN_RECTA
        self.lim["rod"][1] = min(self.lim["rod"][1], self.cero["rod"] - MARGEN_RECTA)
        self.de_pie_servo = [de_pie[a] for a in ART]
        self.de_pie_q = self.q(self.de_pie_servo)

    # ---- servo <-> articulación (grados)
    def q(self, th):
        """Grados del servo (abd, cad, rod) -> ángulos de articulación (q_abd, q_cad, q_rod)."""
        return tuple((th[i] - self.cero[a]) / self.signo[a] for i, a in enumerate(ART))

    def servo(self, q_abd, q_cad, q_rod):
        return (self.cero["abd"] + self.signo["abd"] * q_abd,
                self.cero["cad"] + self.signo["cad"] * q_cad,
                self.cero["rod"] + self.signo["rod"] * q_rod)

    def dentro(self, th):
        return all(self.lim[a][0] <= th[i] <= self.lim[a][1] for i, a in enumerate(ART))

    def fuera(self, th):
        """Lista de (articulación, grado, [min, max]) fuera de los límites."""
        return [(a, th[i], self.lim[a]) for i, a in enumerate(ART) if not self.lim[a][0] <= th[i] <= self.lim[a][1]]

    # ---- cinemática del plano de la pata (cm, grados)
    def fk(self, q_cad, q_rod):
        a, b = math.radians(q_cad), math.radians(q_cad + q_rod)
        return (self.L1 * math.sin(a) + self.L2 * math.sin(b), self.L1 * math.cos(a) + self.L2 * math.cos(b))

    def rodilla(self, q_cad):
        a = math.radians(q_cad)
        return (self.L1 * math.sin(a), self.L1 * math.cos(a))

    def ik(self, x, z):
        """Pie en (x, z) cm -> (q_cad, q_rod) en grados, con la rodilla hacia atrás (q_rod > 0)."""
        d2 = x * x + z * z
        c = (d2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if not -1.0 <= c <= 1.0:
            raise ErrorPata(f"el pie en ({x:.1f}, {z:.1f}) cm no se alcanza (de {abs(self.L1 - self.L2):.1f} "
                            f"a {self.L1 + self.L2:.1f} cm de la cadera)")
        q_rod = math.acos(c)
        q_cad = math.atan2(x, z) - math.atan2(self.L2 * math.sin(q_rod), self.L1 + self.L2 * math.cos(q_rod))
        return math.degrees(q_cad), math.degrees(q_rod)


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    p = Pata()
    print(f"Pata {p.nombre}: {p.fuente}")
    print(f"  muslo {p.L1:g} cm · espinilla {p.L2:g} cm · hombro {p.hombro:g} cm · canales {p.canales}")
    print("\n  articulación   cero (servo)   límites del script (servo)")
    for a in ART:
        print(f"  {a:<12}   {p.cero[a]:8.2f}°      {p.lim[a][0]:6.1f}° a {p.lim[a][1]:6.1f}°")
    qa, qc, qr = p.de_pie_q
    x, z = p.fk(qc, qr)
    print(f"\n  Pose de pie: q_abd {qa:+.1f}°, q_cad {qc:+.1f}°, q_rod {qr:+.1f}°")
    print(f"  -> pie en x = {x:+.1f} cm (adelante de la cadera), z = {z:.1f} cm (abajo)")
    qc2, qr2 = p.ik(x, z)
    print(f"  Cinemática inversa de ese punto: q_cad {qc2:+.1f}°, q_rod {qr2:+.1f}° "
          f"(error {abs(qc2 - qc) + abs(qr2 - qr):.2e}°)")


if __name__ == "__main__":
    main()
