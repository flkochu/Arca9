"""
El perro completo para los nodos de Agmen: patas, canales, límites y pies
==========================================================================
Lo usan puente.py, poses.py y manejo.py. No calcula nada nuevo: junta una `Pata` de pc/pata.py
por cada pata medida y la escala de pc/escala.py.

- Las patas se leen de datos/*/pata_<fl|fr|rl|rr>.json (el más reciente de cada una). Hoy solo
  está medida la FL (prueba 05). Una pata sin su archivo no tiene límites, así que el puente no
  la mueve.
- Con simulado=True, las patas que faltan se arman como copias de la FL en sus canales. Solo
  sirve para la simulación: en el perro de verdad cada pata se mide.
- Los límites son los de pata.py: los medidos menos 5°, y la rodilla sin pasar de recta. Están
  en grados reales del servo.

Pies: en cm desde el eje de la cadera, en el plano de la pata (x hacia el frente, z hacia abajo),
como en pata.py. La abducción se deja como está (en las poses, q_abd = 0: la pata vertical vista
de frente).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PC = os.path.dirname(HERE)
if PC not in sys.path:
    sys.path.insert(0, PC)
from pata import Pata, ErrorPata, ART, buscar_pata          # noqa: E402
from escala import N_SERVOS, MV_MIN, MV_MAX                 # noqa: E402

PATAS = ("fl", "fr", "rl", "rr")
NOMBRES = {"fl": "FL (delantera izquierda)", "fr": "FR (delantera derecha)",
           "rl": "RL (trasera izquierda)", "rr": "RR (trasera derecha)"}
CANALES = {"fl": (0, 1, 2), "fr": (3, 4, 5), "rl": (6, 7, 8), "rr": (9, 10, 11)}   # docs/hardware.md
DELANTERAS = ("fl", "fr")
IZQUIERDAS = ("fl", "rl")
MENSAJES = os.path.join(HERE, "arca9.agmen")
NOMBRE_ART = {"abd": "abducción", "cad": "cadera", "rod": "rodilla"}


def cargar_agmen():
    """Importa agmen y carga arca9.agmen. Regresa (agmen, msgs)."""
    try:
        import agmen
    except ImportError:
        sys.exit("Falta Agmen: pip install agmen")
    if not hasattr(agmen, "Node"):
        # pc/agmen/ (esta carpeta) se puede importar como "agmen" si el de verdad no está instalado
        sys.exit("Falta Agmen (se encontró la carpeta pc/agmen/, no el paquete): pip install agmen")
    agmen.ensure_utf8_output()
    return agmen, agmen.load(MENSAJES)


def mv_creible(mv):
    """Con el cable verde suelto el ADC no baja de ~142 mV; en su tope el servo da ~180 mV."""
    return mv is not None and MV_MIN <= mv <= MV_MAX


class ErrorPerro(ValueError):
    pass


class Perro:
    def __init__(self, simulado=False, rutas=None):
        """`rutas` = {"fl": ruta, ...} para usar otros archivos (las pruebas); si no, los de datos/."""
        self.patas = {}
        self.simuladas = []
        usadas = {}
        rutas = rutas or {}
        for n in PATAS:
            ruta = rutas.get(n) or buscar_pata(f"pata_{n}.json")
            if ruta:
                p = Pata(ruta)
                self._revisar(n, p)
                self.patas[n] = p
                usadas[n] = ruta
        if simulado:
            if "fl" not in self.patas:
                raise ErrorPerro("no encontré datos/*/pata_fl.json: sin la FL no hay de dónde copiar las patas simuladas")
            for n in PATAS:
                if n not in self.patas:
                    p = Pata(usadas["fl"])
                    p.canales = list(CANALES[n])
                    p.nombre = n.upper()
                    self.patas[n] = p
                    self.simuladas.append(n)
        self.patas = {n: self.patas[n] for n in PATAS if n in self.patas}     # siempre en el mismo orden
        self.lim = {}                         # canal -> (min, max) en grados reales del servo
        self.de_canal = {}                    # canal -> (pata, articulación)
        for n, p in self.patas.items():
            for i, a in enumerate(ART):
                ch = p.canales[i]
                self.lim[ch] = tuple(p.lim[a])
                self.de_canal[ch] = (n, a)

    @staticmethod
    def _revisar(n, p):
        if tuple(p.canales) != CANALES[n]:
            raise ErrorPerro(f"{p.fuente} dice que la pata {n.upper()} usa los canales {p.canales}, pero en "
                             f"docs/hardware.md son {list(CANALES[n])}")
        for a in ART:
            lo, hi = p.lim[a]
            if not lo < hi:
                raise ErrorPerro(f"{p.fuente}: los límites de {NOMBRE_ART[a]} quedan vacíos ({lo:.1f}° a {hi:.1f}°)")
        q_abd, q_cad, q_rod = p.de_pie_q
        if q_rod <= 0:
            # pata.py supone que al subir el ángulo de la rodilla se estira (signo −1), como en la FL.
            raise ErrorPerro(f"{p.fuente}: con su pose de pie, la rodilla sale doblada al revés (q_rod = "
                             f"{q_rod:.1f}°). pata.py supone el mismo sentido que la FL en las 3 articulaciones; "
                             f"si esta pata gira al revés, primero hay que enseñárselo a pata.py")

    # ---- qué hay
    def canales(self):
        """Los canales que tienen pata (y límites), en orden."""
        return sorted(self.lim)

    def faltan(self):
        return [n for n in PATAS if n not in self.patas]

    def describir(self):
        lineas = []
        for n, p in self.patas.items():
            sim = " (SIMULADA: copia de la FL)" if n in self.simuladas else ""
            lims = ", ".join(f"{NOMBRE_ART[a]} {p.lim[a][0]:.0f}–{p.lim[a][1]:.0f}°" for a in ART)
            lineas.append(f"{NOMBRES[n]}, canales {p.canales}{sim}: {lims}")
        for n in self.faltan():
            lineas.append(f"{NOMBRES[n]}, canales {list(CANALES[n])}: sin medir (falta datos/*/pata_{n}.json): "
                          "no se mueve")
        return lineas

    def recortar(self, ch, g):
        lo, hi = self.lim[ch]
        return min(max(g, lo), hi)

    # ---- pies <-> 12 servos
    def servos_de_pata(self, n, x, z, q_abd=0.0):
        """Grados reales de los 3 servos de la pata `n` para el pie en (x, z). ErrorPata si no se
        alcanza o se sale de los límites."""
        p = self.patas[n]
        th = p.servo(q_abd, *p.ik(x, z))
        fuera = p.fuera(th)
        if fuera:
            raise ErrorPata(f"pata {n.upper()} con el pie en x = {x:.1f}, z = {z:.1f} cm: " + ", ".join(
                f"{NOMBRE_ART[a]} {g:.1f}° fuera de {lo:.1f}–{hi:.1f}°" for a, g, (lo, hi) in fuera))
        return th

    def comando_de_pies(self, pies, abd=None):
        """{pata: (x, z)} -> (q[12], activo[12]). `abd` = {pata: q_abd} (si no, 0). ErrorPata si
        algún pie no se alcanza o se sale de los límites."""
        q = [0.0] * N_SERVOS
        activo = [False] * N_SERVOS
        for n, (x, z) in pies.items():
            th = self.servos_de_pata(n, x, z, (abd or {}).get(n, 0.0))
            for ch, g in zip(self.patas[n].canales, th):
                q[ch] = float(g)
                activo[ch] = True
        return q, activo

    def pies_de_servos(self, q):
        """12 grados reales -> ({pata: (x, z)}, {pata: q_abd}). Las patas con algún ángulo que no sea
        un número no aparecen."""
        pies, abd = {}, {}
        for n, p in self.patas.items():
            th = [q[ch] for ch in p.canales]
            if not all(isinstance(g, (int, float)) and math.isfinite(g) for g in th):
                continue
            q_abd, q_cad, q_rod = p.q(th)
            pies[n] = p.fk(q_cad, q_rod)
            abd[n] = q_abd
        return pies, abd


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    for sim in (False, True):
        perro = Perro(simulado=sim)
        print("Simulado:" if sim else "Con el perro:")
        for l in perro.describir():
            print("  " + l)


if __name__ == "__main__":
    main()
