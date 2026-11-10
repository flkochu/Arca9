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

Empieza desde donde está el perro: espera la primera lectura de /arca9/articulaciones.

Los botones "parado / sentado / echado" del panel (la tarjeta "Poses", [[web.command]] en
robot.toml, Agmen 0.13) mandan un Arca9Pose a /arca9/pose: poses.py va a esa pose con suavidad,
desde donde va, y se queda ahí (deja la secuencia).

Si otra entrada del mux le gana (la del panel, /arca9/comando/panel), lo sabe por /mux/mando (el
MuxEstado de Agmen 0.13): mientras tanto publica lo mismo que sale del mux, para que al regresarle
el mando no haya salto, y retoma desde ahí hasta su pose.

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
NOMBRE_POSE = {0: "parado", 1: "sentado", 2: "echado"}          # Arca9Pose.pose (arca9.agmen)


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
    ap.add_argument("--pedidos", default="/arca9/pose", help="de dónde llegan los Arca9Pose (botones del panel)")
    ap.add_argument("--mux", default="/mux/mando", help="el MuxEstado del mux (quién manda); '' = no escuchar")
    ap.add_argument("--entrada", default="poses", help="el nombre de mi entrada en el [[mux]]")
    ap.add_argument("--salida", default="/arca9/comando", help="lo que sale del mux (para retomar sin salto)")
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
    for nombre in POSES:
        try:
            objetivos[nombre] = comando_de_pose(perro, nombre)
        except ErrorPata as e:
            if nombre in secuencia:
                sys.exit(f"[poses] la pose '{nombre}' no cabe en los límites: {e}")
            log(f"⚠ la pose '{nombre}' no cabe en los límites, su botón no hará nada: {e}")
    log(f"patas: {', '.join(n.upper() for n in perro.patas)}"
        + (f" (simuladas: {', '.join(n.upper() for n in perro.simuladas)})" if perro.simuladas else "")
        + (f" · sin medir, no se mueven: {', '.join(n.upper() for n in perro.faltan())}" if perro.faltan() else ""))
    log(f"{' → '.join(secuencia)}{' (en ciclo)' if len(secuencia) > 1 else ''} · hasta {a.vel:g} °/s · "
        f"publico en {a.destino}")

    node = agmen.Node("poses")
    pub = node.publisher(msgs.Arca9Comando, a.destino)
    st = {"medido": None, "tr": None, "i": 0, "llego": None, "salida": None, "manda": "",
          "cedido": False, "esperando": 0.0, "pedida": None}

    @node.on(msgs.Arca9Articulaciones, "/arca9/articulaciones")
    def articulaciones(s):
        st["medido"] = list(s.message.q)

    @node.on(msgs.Arca9Pose, a.pedidos)
    def pedido(s):
        nombre = NOMBRE_POSE.get(s.message.pose)
        if nombre is None:
            log(f"⚠ llegó la pose {s.message.pose}: no existe (0 = parado, 1 = sentado, 2 = echado)")
        elif nombre not in objetivos:
            log(f"⚠ la pose '{nombre}' no cabe en los límites: no la hago")
        else:
            st["pedida"] = nombre          # los botones mandan ~20 por segundo: se aplica en tick()

    if a.mux:
        from agmen.msgs import MuxEstado

        @node.on(MuxEstado, a.mux)
        def mando(s):
            st["manda"] = s.message.entrada

        @node.on(msgs.Arca9Comando, a.salida)
        def salida(s):
            st["salida"] = (list(s.message.q), list(s.message.activo))

    def empezar(desde, t):
        nombre = secuencia[st["i"]]
        q, activo = objetivos[nombre]
        st["tr"] = Transicion(desde, q, activo, a.vel, t)
        st["llego"] = None
        log(f"→ {nombre} en {st['tr'].T:.1f} s")

    def pedir(nombre, t):
        """Un botón del panel: de la secuencia a quedarse en `nombre`, desde donde va."""
        st["pedida"] = None
        if secuencia == [nombre] and st["tr"] is not None:
            return                          # ya va (o ya está) ahí
        log(f"el panel pidió '{nombre}'")
        secuencia[:] = [nombre]
        st["i"] = 0
        if st["tr"] is not None and not st["cedido"]:
            empezar(st["tr"].q(t), t)

    @node.every(1.0 / HZ)
    def tick():
        t = time.monotonic()
        if st["pedida"]:
            pedir(st["pedida"], t)
        if st["manda"] not in (a.entrada, "") and st["salida"]:
            # Otra entrada tiene el mando: publico lo que sale del mux, así al regresarme el mando
            # no hay salto.
            if not st["cedido"]:
                st["cedido"] = True
                log(f"'{st['manda']}' tomó el mando: sigo donde deje al perro")
            q, activo = st["salida"]
            pub.publish(msgs.Arca9Comando(q=q, activo=activo))
            return
        if st["cedido"] and st["manda"] == a.entrada:
            st["cedido"] = False
            q_ultimo, _ = st["salida"]
            log("me regresaron el mando: retomo desde ahí")
            empezar(q_ultimo, t)
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
