#!/usr/bin/env python3
"""
Poses simples de Arca9: parado, sentado y echado
=================================================
Un nodo de Agmen que publica Arca9Comando a 50 Hz para llevar al perro de una pose a otra con
suavidad. Es el "modo autónomo" del robot.toml: el panel le gana en el [[mux]].

Las poses son posiciones del pie (cm desde el eje de la cadera; x hacia el frente, z hacia abajo)
con la abducción en 0, como en la prueba 05. Cada pata sale de su cinemática inversa (pc/pata.py)
y se revisa contra sus límites antes de mover nada:

    parado    las 4 patas con el pie en x = 2, z = 18.5 (la pose de pie de la prueba 05)
    sentado   delanteras en x = 2, z = 19.5; traseras dobladas, con el pie en x = 5, z = 11
    echado    las 4 dobladas, con el pie en x = 5, z = 11

Interpolación: de articulación a articulación, con un perfil de jerk mínimo (empieza y termina sin
velocidad ni aceleración). La duración sale de la articulación que más se mueve, para que ninguna
pase de --vel °/s (por defecto 30, la mitad del --vel-max del puente: el puente no la recorta).

Empieza desde donde está el perro: espera la primera lectura de /arca9/articulaciones. Si el panel
toma el mando (llegan mensajes a --mando-mayor, la entrada del mux con más prioridad), publica lo
mismo que el panel para que al soltar no haya salto, y medio segundo después de que el panel se
calla vuelve a interpolar desde ahí hasta su pose.

Uso (normalmente lo lanza `agmen run robot.toml`):
    python poses.py --pose parado
    python poses.py --secuencia parado sentado parado echado --espera 3   # en ciclo
    python poses.py --simulado ...       # con las 4 patas simuladas (como puente.py --simulado)
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from perro import Perro, ErrorPerro, ErrorPata, cargar_agmen, DELANTERAS, N_SERVOS   # noqa: E402

POSES = {
    "parado": {"delantera": (2.0, 18.5), "trasera": (2.0, 18.5)},
    "sentado": {"delantera": (2.0, 19.5), "trasera": (5.0, 11.0)},
    "echado": {"delantera": (5.0, 11.0), "trasera": (5.0, 11.0)},
}
HZ = 50.0
VEL = 30.0                  # °/s pico de la articulación que más se mueve
DUR_MIN = 1.0               # s: ninguna transición más rápida que esto
HOLD_MANDO = 0.5            # s sin mensajes del panel para retomar (el mux suelta a los 0.3 s)


def comando_de_pose(perro, nombre):
    pies = {n: POSES[nombre]["delantera" if n in DELANTERAS else "trasera"] for n in perro.patas}
    return perro.comando_de_pies(pies)


def jerk_minimo(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10 - 15 * u + 6 * u * u)


class Transicion:
    """De `desde` a `hasta` (12 grados reales; solo los canales activos) con jerk mínimo."""

    def __init__(self, desde, hasta, activo, vel, t0):
        self.desde, self.hasta, self.activo, self.t0 = list(desde), list(hasta), list(activo), t0
        delta = max((abs(h - d) for d, h, a in zip(desde, hasta, activo) if a), default=0.0)
        self.T = max(DUR_MIN, 1.875 * delta / vel)        # pico de jerk mínimo = 1.875 · delta / T

    def q(self, t):
        s = jerk_minimo((t - self.t0) / self.T)
        return [d + (h - d) * s if a else 0.0 for d, h, a in zip(self.desde, self.hasta, self.activo)]

    def termino(self, t):
        return t - self.t0 >= self.T


def main():
    agmen, msgs = cargar_agmen()
    ap = argparse.ArgumentParser(description="Poses simples de Arca9 (parado, sentado, echado)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--pose", choices=sorted(POSES), help="ir a una pose y quedarse")
    g.add_argument("--secuencia", nargs="+", choices=sorted(POSES), metavar="POSE",
                   help="ir de una pose a otra, en ciclo (ej. parado sentado parado echado)")
    ap.add_argument("--espera", type=float, default=3.0, help="s quieto en cada pose de la secuencia (default 3)")
    ap.add_argument("--vel", type=float, default=VEL, help=f"°/s pico de cada articulación (default {VEL:g})")
    ap.add_argument("--destino", default="/arca9/comando/poses", help="a dónde publicar (la entrada del mux)")
    ap.add_argument("--mando-mayor", default="/arca9/comando/panel",
                    help="la entrada del mux que le gana a poses (para retomar sin salto); '' = ninguna")
    ap.add_argument("--simulado", action="store_true", help="con las 4 patas simuladas (copias de la FL)")
    a = ap.parse_args()
    secuencia = a.secuencia or [a.pose or "parado"]
    if not 0 < a.vel <= 120:
        ap.error("--vel debe estar entre 0 y 120 °/s")

    def log(t):
        print(f"[poses] {t}", flush=True)

    try:
        perro = Perro(simulado=a.simulado)
    except ErrorPerro as e:
        sys.exit(f"[poses] {e}")
    if not perro.patas:
        sys.exit("[poses] ninguna pata medida (datos/*/pata_*.json): no hay nada que mover")
    objetivos = {}
    for nombre in sorted(set(secuencia)):
        try:
            objetivos[nombre] = comando_de_pose(perro, nombre)
        except ErrorPata as e:
            sys.exit(f"[poses] la pose '{nombre}' no cabe en los límites: {e}")
    log(f"patas: {', '.join(n.upper() for n in perro.patas)}"
        + (f" (simuladas: {', '.join(n.upper() for n in perro.simuladas)})" if perro.simuladas else "")
        + (f" · sin medir, no se mueven: {', '.join(n.upper() for n in perro.faltan())}" if perro.faltan() else ""))
    log(f"{' → '.join(secuencia)}{' (en ciclo)' if len(secuencia) > 1 else ''} · hasta {a.vel:g} °/s · "
        f"publico en {a.destino}")

    node = agmen.Node("poses")
    pub = node.publisher(msgs.Arca9Comando, a.destino)
    st = {"medido": None, "tr": None, "i": 0, "llego": None, "mayor_q": None, "mayor_t": -1e9,
          "cedido": False, "esperando": 0.0}

    @node.on(msgs.Arca9Articulaciones, "/arca9/articulaciones")
    def articulaciones(s):
        st["medido"] = list(s.message.q)

    if a.mando_mayor:
        @node.on(msgs.Arca9Comando, a.mando_mayor)
        def mayor(s):
            st["mayor_q"] = (list(s.message.q), list(s.message.activo))
            st["mayor_t"] = time.monotonic()

    def empezar(desde, t):
        nombre = secuencia[st["i"]]
        q, activo = objetivos[nombre]
        st["tr"] = Transicion(desde, q, activo, a.vel, t)
        st["llego"] = None
        log(f"→ {nombre} en {st['tr'].T:.1f} s")

    @node.every(1.0 / HZ)
    def tick():
        t = time.monotonic()
        if t - st["mayor_t"] < HOLD_MANDO and st["mayor_q"]:
            # El panel tiene el mando: publico lo mismo que él, así al soltar no hay salto.
            if not st["cedido"]:
                st["cedido"] = True
                log("el panel tomó el mando: sigo donde él deje al perro")
            q, activo = st["mayor_q"]
            pub.publish(msgs.Arca9Comando(q=q, activo=activo))
            return
        if st["cedido"]:
            st["cedido"] = False
            q_panel, _ = st["mayor_q"]
            log("el panel soltó el mando: retomo desde ahí")
            empezar(q_panel, t)
        if st["tr"] is None:
            q0 = st["medido"]
            canales = perro.canales()
            if q0 is None or not all(math.isfinite(q0[ch]) for ch in canales):
                if t - st["esperando"] > 5.0:
                    st["esperando"] = t
                    log("esperando las lecturas de /arca9/articulaciones (¿está corriendo el puente?)"
                        if q0 is None else
                        "esperando lectura creíble de los canales " +
                        ", ".join(str(ch) for ch in canales if not math.isfinite(q0[ch])))
                return
            empezar([q0[ch] if ch in canales else 0.0 for ch in range(N_SERVOS)], t)
        tr = st["tr"]
        pub.publish(msgs.Arca9Comando(q=tr.q(t), activo=tr.activo))
        if tr.termino(t) and len(secuencia) > 1:
            if st["llego"] is None:
                st["llego"] = t
            elif t - st["llego"] >= a.espera:
                st["i"] = (st["i"] + 1) % len(secuencia)
                empezar(tr.hasta, t)

    node.spin()
    node.close()


if __name__ == "__main__":
    main()
