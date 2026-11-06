#!/usr/bin/env python3
"""
Manejar a Arca9 desde el panel de Agmen
=======================================
El panel de Agmen solo sabe mandar MotorCommand{left, right} (su sección "Manejar", pensada para
un robot de ruedas). Este nodo lo traduce a Arca9Comando y lo publica en la entrada del [[mux]]
que le gana a poses.py:

    panel ──MotorCommand──► /arca9/teleop ──► manejo.py ──Arca9Comando──► /arca9/comando/panel ──► mux

Mientras mantienes presionado:
    ↑ (W)  sube el cuerpo: los 4 pies bajan        ↓ (S)  baja el cuerpo
    → (D)  se inclina a la derecha: los pies de la derecha suben y los de la izquierda bajan
    ← (A)  se inclina a la izquierda
La rapidez es la del deslizador "velocidad" del panel: al 100 %, 3 cm/s de altura y 2 cm/s de
inclinación. Al soltar, el panel manda ALTO: el perro se queda donde va y, 0.3 s después, el mux
le regresa el mando a poses.py (que sigue desde ahí, sin salto).

Cada vez que empiezas a manejar, parte de lo que el perro está siguiendo en ese momento (la salida
del mux, /arca9/comando) o, si nadie le manda nada, de lo que mide (/arca9/articulaciones). Antes
de publicar revisa que cada pie quede entre z = 10.5 y 21 cm, que ningún servo salga de sus límites
y que ninguno cambie más de 3° de un mensaje al siguiente; si no, ese paso no se da.

Uso (normalmente lo lanza `agmen run robot.toml`):
    python manejo.py
    python manejo.py --simulado        # con las 4 patas simuladas (como puente.py --simulado)
"""
import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from perro import Perro, ErrorPerro, ErrorPata, cargar_agmen, IZQUIERDAS   # noqa: E402

VEL_ALTURA = 3.0            # cm/s con el panel al 100 %
VEL_INCLINA = 2.0           # cm/s
INCLINA_MAX = 3.0           # cm de diferencia, de cada lado
Z_MIN, Z_MAX = 10.5, 21.0   # cm: cada pie entre estas alturas. Más abajo de 21 cm la pata casi queda
                            # recta (24 cm), y ahí un milímetro de pie es mucho ángulo de rodilla
PASO_MAX = 3.0              # ° que puede cambiar un servo de un mensaje al siguiente
SESION_S = 0.5              # sin mensajes del panel por más de esto = la siguiente vez empieza otra vez
BASE_VIEJA_S = 1.0          # la salida del mux de hace más de esto ya no cuenta como "lo que sigue el perro"


def main():
    agmen, msgs = cargar_agmen()
    ap = argparse.ArgumentParser(description="Manejar a Arca9 desde el panel de Agmen (MotorCommand → Arca9Comando)")
    ap.add_argument("--entrada", default="/arca9/teleop", help="a dónde manda el panel ([web.teleop] target)")
    ap.add_argument("--destino", default="/arca9/comando/panel", help="a dónde publicar (la entrada del mux)")
    ap.add_argument("--salida-mux", default="/arca9/comando", help="lo que sigue el perro (la salida del mux)")
    ap.add_argument("--simulado", action="store_true", help="con las 4 patas simuladas (copias de la FL)")
    a = ap.parse_args()

    def log(t):
        print(f"[manejo] {t}", flush=True)

    try:
        perro = Perro(simulado=a.simulado)
    except ErrorPerro as e:
        sys.exit(f"[manejo] {e}")
    if not perro.patas:
        sys.exit("[manejo] ninguna pata medida (datos/*/pata_*.json): no hay nada que mover")
    log(f"patas: {', '.join(n.upper() for n in perro.patas)} · {a.entrada} (MotorCommand del panel) → {a.destino}")
    log("↑/↓ sube y baja el cuerpo · ←/→ lo inclina · al soltar se queda donde va")

    node = agmen.Node("manejo")
    pub = node.publisher(msgs.Arca9Comando, a.destino)
    st = {"mux": None, "mux_t": -1e9, "medido": None, "sesion": None, "t": -1e9}

    @node.on(msgs.Arca9Comando, a.salida_mux)
    def salida_mux(s):
        st["mux"], st["mux_t"] = (list(s.message.q), list(s.message.activo)), time.monotonic()

    @node.on(msgs.Arca9Articulaciones, "/arca9/articulaciones")
    def articulaciones(s):
        st["medido"] = list(s.message.q)

    def base():
        """Dónde están los pies ahora: lo que sigue el perro o, si no, lo que mide."""
        if st["mux"] and time.monotonic() - st["mux_t"] < BASE_VIEJA_S:
            q, activo = st["mux"]
            q = [g if act else math.nan for g, act in zip(q, activo)]
            de = "lo que sigue el perro (/arca9/comando)"
        elif st["medido"]:
            q, de = st["medido"], "lo que mide el perro (/arca9/articulaciones)"
        else:
            return None, None
        pies, abd = perro.pies_de_servos(q)
        if set(pies) != set(perro.patas):
            return None, None
        return {"pies": pies, "abd": abd, "altura": 0.0, "inclina": 0.0, "q": None, "limite": False}, de

    def pies_de(ses, altura, inclina):
        return {n: (x, z + altura + (inclina if n in IZQUIERDAS else -inclina)) for n, (x, z) in ses["pies"].items()}

    @node.on(msgs.MotorCommand, a.entrada)
    def panel(s):
        t = time.monotonic()
        l, r = float(s.message.left), float(s.message.right)
        ses = st["sesion"]
        if ses is not None and t - st["t"] > SESION_S:
            ses = st["sesion"] = None
        dt = min(max(t - st["t"], 0.0), 0.1)
        st["t"] = t
        if l == 0.0 and r == 0.0:                     # ALTO: se queda donde va
            if ses is not None and ses["q"] is not None:
                pub.publish(msgs.Arca9Comando(q=ses["q"][0], activo=ses["q"][1]))
            st["sesion"] = None
            return
        if ses is None:
            ses, de = base()
            if ses is None:
                log("⚠ todavía no sé dónde está el perro (no llega /arca9/comando ni /arca9/articulaciones con "
                    "las patas completas): no muevo nada")
                return
            st["sesion"] = ses
            dt = 0.05
            log(f"manejo desde el panel, partiendo de {de}")
        avance, giro = (l + r) / 2, (l - r) / 2
        altura = ses["altura"] + avance * VEL_ALTURA * dt
        inclina = max(-INCLINA_MAX, min(INCLINA_MAX, ses["inclina"] + giro * VEL_INCLINA * dt))
        try:
            pies = pies_de(ses, altura, inclina)
            for n, (x, z) in pies.items():
                if not Z_MIN <= z <= Z_MAX:
                    raise ErrorPata(f"el pie de la pata {n.upper()} quedaría a z = {z:.1f} cm (se maneja entre "
                                    f"{Z_MIN:g} y {Z_MAX:g} cm)")
            q = perro.comando_de_pies(pies, ses["abd"])
            if ses["q"] is not None:
                cambio = max(abs(g - h) for g, h, act in zip(q[0], ses["q"][0], q[1]) if act)
                if cambio > PASO_MAX:
                    raise ErrorPata(f"un servo cambiaría {cambio:.1f}° de golpe (máximo {PASO_MAX:g}°)")
            ses["altura"], ses["inclina"], ses["q"] = altura, inclina, q
            ses["limite"] = False
        except ErrorPata as e:
            if not ses["limite"]:
                ses["limite"] = True
                log(f"llegó al límite, no sigo por ahí: {e}")
            if ses["q"] is None:                       # todavía no hay un paso bueno: quedarse en la base
                try:
                    ses["q"] = perro.comando_de_pies(pies_de(ses, ses["altura"], ses["inclina"]), ses["abd"])
                except ErrorPata:
                    log("⚠ el perro está fuera de sus límites: desde aquí no lo manejo (que poses.py lo lleve "
                        "a una pose primero)")
                    st["sesion"] = None
                    return
        pub.publish(msgs.Arca9Comando(q=ses["q"][0], activo=ses["q"][1]))

    node.spin()
    node.close()


if __name__ == "__main__":
    main()
