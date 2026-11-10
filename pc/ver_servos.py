#!/usr/bin/env python3
"""
Ver los 12 servos en vivo (sin mover nada)
===========================================
Muestra el ángulo que mide el cable verde de cada canal, 10 veces por segundo, con los servos
SUELTOS para que muevas las patas con la mano. Sirve para:

  1. Revisar que cada servo tenga su cable de señal y su cable verde en el MISMO canal: mueves una
     articulación con la mano y ves qué canal cambia (lo marca con ◄).
  2. Medir cada pata nueva como se midió la FL: sus límites, la pata recta y la pose de pie.
     Con Enter guardas una lectura de los 12 en datos/<fecha>_lecturas/lecturas.csv.

Nunca manda un pulso: al empezar suelta los 12 (comando D). Ten al perro en el soporte, con las
patas al aire: al soltarse, se cae.

Uso (desde la carpeta Arca9, con el venv activado y el Monitor Serie de Arduino cerrado):
    python pc/ver_servos.py --port COM6              # en vivo; Enter guarda, Ctrl+C sale
    python pc/ver_servos.py --port COM6 --revisar    # una lectura de los 12 y se sale
    python pc/ver_servos.py --simulado               # sin ESP32, para ver cómo se ve

Los nombres son los de docs/hardware.md (canal 6 = RL abducción, etc.). Si mueves la rodilla RL y
el ◄ sale en otro canal, el cable verde de esa rodilla está en ese otro canal.
"""
import argparse
import csv
import datetime
import math
import os
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from escala import MV_MAX, MV_MIN, N_SERVOS, Escala, Retro, relativa   # noqa: E402

RAIZ = os.path.dirname(HERE)
PATAS = [("FL", "delantera izquierda"), ("FR", "delantera derecha"),
         ("RL", "trasera izquierda"), ("RR", "trasera derecha")]
ART = ["abd", "cad", "rod"]
NOMBRE_ART = {"abd": "abducción", "cad": "cadera", "rod": "rodilla"}
SE_MUEVE = 3.0          # grados en el último segundo para marcarlo con ◄ (el ruido es ~0.3°)


def nombre(ch):
    return f"{PATAS[ch // 3][0]} {NOMBRE_ART[ART[ch % 3]]}"


class ESP32Simulada:
    """Sin hardware: los 12 quietos con ruido, y un canal a la vez que alguien "mueve con la mano"."""

    def __init__(self, esc, retro):
        import random
        self.r = random.Random(7)
        self.esc, self.retro = esc, retro
        self.t0 = time.monotonic()
        self.info = {"linea": "OK1-RETRO (SIMULADA)"}

    def _mv(self, ch, grados):
        c = self.retro.canales[ch]
        conv = self.esc.real_a_conv(grados)
        return int(round((conv - c["b"]) / c["a"] + self.r.gauss(0, 2)))

    def leer_todos(self):
        t = time.monotonic() - self.t0
        moviendo = int(t // 4) % N_SERVOS
        out = []
        for ch in range(N_SERVOS):
            g = 90.0 + ch
            if ch == moviendo:
                g += 30 * math.sin(2 * math.pi * (t % 4) / 4)
            out.append(self._mv(ch, g))
        return out

    def soltar(self):
        pass

    def cerrar(self):
        pass


def credible(mv):
    return mv is not None and MV_MIN <= mv <= MV_MAX


class Teclado:
    """¿Se apretó Enter? Sin bloquear (Windows: msvcrt; Linux/macOS: select)."""

    def __init__(self):
        try:
            import msvcrt
            self.msvcrt = msvcrt
        except ImportError:
            self.msvcrt = None

    def enter(self):
        if self.msvcrt:
            hubo = False
            while self.msvcrt.kbhit():
                if self.msvcrt.getwch() in "\r\n":
                    hubo = True
            return hubo
        import select
        if self.msvcrt is None and getattr(self, "fin", False):
            return False
        if select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.readline() == "":       # sin teclado (stdin cerrado): no hay Enter que esperar
                self.fin = True
                return False
            return True
        return False


def tabla(grados, mv, marcados):
    lineas = []
    for p, (pata, desc) in enumerate(PATAS):
        celdas = []
        for i, a in enumerate(ART):
            ch = 3 * p + i
            g = f"{grados[ch]:6.1f}°" if grados[ch] is not None else "   —   "
            marca = "◄" if ch in marcados else " "
            celdas.append(f"P{ch:<2} {NOMBRE_ART[a]:<9} {g}{marca}")
        lineas.append(f"{pata} {desc:<19} │ " + " │ ".join(celdas))
    return lineas


def main():
    ap = argparse.ArgumentParser(description="Ver los 12 servos en vivo, sueltos, sin mover nada")
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--simulado", action="store_true", help="sin ESP32")
    ap.add_argument("--revisar", action="store_true", help="una lectura de los 12 y se sale")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    os.system("")                         # Windows: activa los códigos para reescribir la pantalla

    esc = Escala()
    retro = Retro(esc)
    if args.simulado:
        esp = ESP32Simulada(esc, retro)
    else:
        from arca9_serial import ESP32, ErrorESP32
        try:
            esp = ESP32(args.port)        # abrir el puerto reinicia la ESP32: arranca con todo suelto
        except ErrorESP32 as e:
            sys.exit(f"ERROR: {e}")
    print(f"ESP32: {esp.info['linea']}")
    esp.soltar()                          # D: los 12 sueltos, por si acaso
    print("Los 12 servos están SUELTOS: mueve las patas con la mano.\n")

    def leer():
        mv = esp.leer_todos()
        return [retro.grados(ch, m) if credible(m) else None for ch, m in enumerate(mv)], mv

    try:
        if args.revisar:
            grados, mv = leer()
            for l in tabla(grados, mv, set()):
                print(l)
            sin = [ch for ch in range(N_SERVOS) if grados[ch] is None]
            if sin:
                print(f"\n⚠ sin lectura creíble ({MV_MIN}–{MV_MAX} mV), ¿cable verde suelto?: "
                      + ", ".join(f"P{ch} ({nombre(ch)}: {mv[ch]} mV)" for ch in sin))
            return

        teclado = Teclado()
        historia = deque()
        csv_ruta, n_guardadas, ultimo = None, 0, None
        print("Enter = guardar una lectura · Ctrl+C = salir\n")
        alto = len(PATAS) + 2
        print("\n" * (alto - 1))
        while True:
            t = time.monotonic()
            grados, mv = leer()
            historia.append((t, grados))
            while historia and t - historia[0][0] > 1.0:
                historia.popleft()
            marcados = set()
            for ch in range(N_SERVOS):
                vals = [g[ch] for _, g in historia if g[ch] is not None]
                if len(vals) > 2 and max(vals) - min(vals) > SE_MUEVE:
                    marcados.add(ch)
            if marcados:
                ultimo = max(marcados, key=lambda ch: max(g[ch] or 0 for _, g in historia)
                             - min(g[ch] or 0 for _, g in historia))
            if teclado.enter():
                if csv_ruta is None:
                    carpeta = os.path.join(RAIZ, "datos", f"{datetime.date.today().isoformat()}_lecturas")
                    os.makedirs(carpeta, exist_ok=True)
                    csv_ruta = os.path.join(carpeta, "lecturas.csv")
                    nuevo = not os.path.exists(csv_ruta)
                    f = open(csv_ruta, "a", newline="", encoding="utf-8")
                    w = csv.writer(f)
                    if nuevo:
                        w.writerow(["n", "hora"] + [f"g{ch}" for ch in range(N_SERVOS)]
                                   + [f"mv{ch}" for ch in range(N_SERVOS)])
                n_guardadas += 1
                w.writerow([n_guardadas, datetime.datetime.now().strftime("%H:%M:%S")]
                           + ["" if g is None else round(g, 1) for g in grados]
                           + ["" if m is None else m for m in mv])
                f.flush()
            pie = (f"Último que se movió: P{ultimo} ({nombre(ultimo)} según docs/hardware.md)"
                   if ultimo is not None else "Mueve una articulación con la mano…")
            if n_guardadas:
                pie += f"   ·   guardadas: {n_guardadas} (en {relativa(csv_ruta)})"
            sys.stdout.write(f"\x1b[{alto}A")
            for l in tabla(grados, mv, marcados) + ["", pie]:
                sys.stdout.write("\x1b[2K" + l + "\n")
            sys.stdout.flush()
            time.sleep(max(0.0, 0.1 - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("\nListo. Los servos siguen sueltos.")
    finally:
        esp.cerrar()


if __name__ == "__main__":
    main()
