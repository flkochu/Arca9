#!/usr/bin/env python3
"""
Pruebas del puente entre Arca9 y Agmen (pc/agmen/), sin el perro
=================================================================
Cuatro partes:

  1. La lógica del puente con la ESP32 falsa (pc/agmen/esp32_falsa.py) y un reloj que avanza a
     mano: límites, velocidad, arranque, watchdog, reinicios, respuestas que no llegan o llegan
     tarde o dañadas, y D al cerrar. Todo lo que sale por el "USB" se revisa línea por línea.
  2. Las patas y las poses: archivos de patas mal hechos, poses dentro de los límites y la
     interpolación de poses.py.
  3. El robot completo con `agmen run pc/agmen/robot_simulado.toml` (procesos de verdad, hub, mux
     y panel): 50 Hz, el perro llega a la pose, el panel le gana a poses.py sin saltos, watchdog
     al apagar poses y D al apagar el puente.
  4. Con el arnés del firmware (pruebas/arnes_firmware, solo Linux o macOS con g++): el puente
     habla por un pseudo-terminal con el esp32_principal.ino de verdad.

Uso, desde la raíz del repo:
    python pruebas/puente_agmen/prueba_puente.py              # todo (~1 min)
    python pruebas/puente_agmen/prueba_puente.py --rapido     # solo las partes 1 y 2 (~5 s)

Al final debe decir TODO BIEN (0 fallas).
"""
import argparse
import json
import math
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

RAIZ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AGMEN_DIR = os.path.join(RAIZ, "pc", "agmen")
sys.path.insert(0, AGMEN_DIR)

from perro import Perro, ErrorPerro, ErrorPata, cargar_agmen, PATAS, CANALES, N_SERVOS   # noqa: E402
from esp32_falsa import ESP32Falsa                                                     # noqa: E402
from puente import ESP32Puente, Puente, revisar_proto, Reinicio, VEL_TOPE              # noqa: E402
from escala import Escala, Retro                                                       # noqa: E402
from arca9_serial import REINICIO                                                      # noqa: E402
import poses                                                                           # noqa: E402

agmen, msgs = cargar_agmen()
esc = Escala()
retro = Retro(esc)

fallas = 0


def revisa(ok, que, detalle=""):
    global fallas
    print(f"{'OK  ' if ok else 'FALLA'}  {que}{'  ->  ' + detalle if detalle else ''}", flush=True)
    if not ok:
        fallas += 1


def seccion(t):
    print(f"\n---- {t}", flush=True)


class Reloj:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def avanzar(self, s):
        self.t += s


def armar(perro=None, inicio=None, aux=True, vel_max=60.0):
    reloj = Reloj()
    perro = perro or Perro(simulado=True)
    if inicio is None:
        inicio = [90.0] * N_SERVOS
        for p in perro.patas.values():
            for ch, g in zip(p.canales, p.de_pie_servo):
                inicio[ch] = g
    falsa = ESP32Falsa(esc, retro, inicio=inicio, reloj=reloj, latencia=False, aux=aux)
    esp = ESP32Puente(puerto_falso=falsa)
    logs = []
    p = Puente(esp, esc, retro, perro, vel_max=vel_max, reloj=reloj, log=logs.append)
    return falsa, esp, p, reloj, logs


def pulsos_s(falsa, desde=0):
    """Los pulsos de cada S que llegó a la ESP32 falsa (desde la línea `desde`)."""
    return [[int(x) for x in l.split()[1:]] for l in falsa.recibidas[desde:] if l.startswith("S ")]


def correr(p, reloj, n, cmd=None, dt=0.02):
    for _ in range(n):
        if cmd is not None:
            p.comando(*cmd)
        p.ciclo()
        reloj.avanzar(dt)


def comando_fl(perro, cad, rod, abd=87.85):
    q = [0.0] * N_SERVOS
    act = [False] * N_SERVOS
    for ch, g in zip(CANALES["fl"], (abd, cad, rod)):
        q[ch], act[ch] = g, True
    return q, act


# ============================================================================ 1. lógica
def parte_logica():
    seccion("1. lógica del puente con la ESP32 falsa")
    perro = Perro(simulado=True)

    falsa, esp, p, reloj, logs = armar(perro)
    revisa(esp.info.get("proto") == 3 and revisar_proto(esp.info) is None, "conecta y lee PROTO=3", esp.info["linea"])
    correr(p, reloj, 10)
    s = pulsos_s(falsa)
    revisa(len(s) == 10 and all(u == -1 for fila in s for u in fila) and p.modo == "SUELTO",
           "sin comandos no mueve nada (S con -1 en los 12)", f"{len(s)} S, modo {p.modo}")
    revisa(all(math.isfinite(g) for g in p.medido) and abs(p.medido[1] - perro.patas["fl"].de_pie_servo[1]) < 2,
           "publica las 12 lecturas en grados reales", " ".join(f"{g:.1f}" for g in p.medido[:3]))

    # Un comando que llega antes de la primera lectura (el puente se reinició con ↻ y poses ya manda)
    f0, e0, p0, r0, l0 = armar(perro)
    correr(p0, r0, 15, (list(perro.patas["fl"].de_pie_servo) * 4, [True] * N_SERVOS))
    revisa(not any("lectura creíble" in l for l in l0) and sum(u != -1 for u in pulsos_s(f0)[-1]) == 12,
           "un comando antes de la primera lectura espera a leer (sin avisos falsos)", "; ".join(l0)[:100])

    # Arranque: uno por ciclo, desde donde está
    n0 = len(falsa.recibidas)
    q_aqui = list(p.medido)
    cmd = (q_aqui, [True] * N_SERVOS)
    correr(p, reloj, 14, cmd)
    s = pulsos_s(falsa, n0)
    prendidos = [sum(u != -1 for u in fila) for fila in s]
    revisa(prendidos[:12] == list(range(1, 13)), "se prenden uno por ciclo", str(prendidos))
    primero = [next(fila[ch] for fila in s if fila[ch] != -1) for ch in range(N_SERVOS)]
    salto = max(abs(esc.us_a_real(u) - g) for u, g in zip(primero, q_aqui))
    revisa(salto < 1.0, "cada servo empieza desde donde lo mide su cable verde", f"{salto:.2f}° de diferencia")
    revisa(p.modo == "SIGUIENDO", "modo SIGUIENDO", p.modo)

    # Velocidad: un comando lejos
    n0 = len(falsa.recibidas)
    lejos = comando_fl(perro, cad=140.0, rod=40.0, abd=110.0)
    correr(p, reloj, 120, lejos)
    s = pulsos_s(falsa, n0)
    paso_max = max(abs(b[ch] - a[ch]) for a, b in zip(s, s[1:]) for ch in (0, 1, 2) if a[ch] != -1 and b[ch] != -1)
    tope_us = 60.0 * 0.02 * esc.us_por_grado + 1
    revisa(paso_max <= tope_us, "nunca más de --vel-max: 60 °/s = 1.2° por ciclo",
           f"paso máximo {paso_max} µs (tope {tope_us:.1f} µs)")
    final = [esc.us_a_real(u) for u in s[-1][:3]]
    revisa(max(abs(f - g) for f, g in zip(final, (110.0, 140.0, 40.0))) < 0.2, "llega al objetivo",
           " ".join(f"{g:.1f}" for g in final))
    revisa(all(fila[ch] == -1 for fila in s for ch in range(3, 12)),
           "los canales que no se piden siguen con su último pulso (-1)")

    # Límites
    n0 = len(falsa.recibidas)
    fuera = comando_fl(perro, cad=175.0, rod=2.0, abd=10.0)
    correr(p, reloj, 200, fuera)
    s = pulsos_s(falsa, n0)
    lim = perro.lim
    peor = max(max(esc.us_a_real(fila[ch]) - lim[ch][1], lim[ch][0] - esc.us_a_real(fila[ch]))
               for fila in s for ch in (0, 1, 2) if fila[ch] != -1)
    revisa(peor <= 0.1, "nunca sale de los límites de pata.py", f"lo más fuera: {peor:+.2f}°")
    final = [esc.us_a_real(u) for u in s[-1][:3]]
    revisa(abs(final[1] - lim[1][1]) < 0.1 and abs(final[2] - lim[2][0]) < 0.1 and abs(final[0] - lim[0][0]) < 0.1,
           "se queda en el límite", " ".join(f"{g:.1f}" for g in final))
    e = p.estado(msgs)
    revisa(e.limitado[:3] == [True, True, True] and not any(e.limitado[3:]), "estado.limitado marca los recortados",
           str(e.limitado[:4]))

    # Watchdog
    n0 = len(falsa.recibidas)
    vuelta = comando_fl(perro, cad=120.0, rod=90.0, abd=87.85)
    correr(p, reloj, 5, vuelta)
    correr(p, reloj, 30)                       # 0.6 s sin comandos
    s = pulsos_s(falsa, n0)
    activos_tarde = [any(u != -1 for u in fila) for fila in s[5 + 26:]]
    revisa(p.modo == "QUIETO" and not any(activos_tarde), "watchdog: sin comandos en 0.5 s se queda quieto",
           f"modo {p.modo}")
    revisa(any("watchdog" in l for l in logs), "watchdog: lo avisa en los logs")
    ultimo = [fila for fila in s if fila[1] != -1][-1]
    n0 = len(falsa.recibidas)
    correr(p, reloj, 3, vuelta)
    s2 = pulsos_s(falsa, n0)
    revisa(p.modo == "SIGUIENDO" and abs(s2[0][1] - ultimo[1]) <= tope_us,
           "al volver los comandos sigue desde donde se quedó (sin salto)", f"{ultimo[1]} → {s2[0][1]} µs")

    # Al cerrar: D
    p.cerrar()
    revisa(falsa.recibidas[-1] == "D" and any("(D)" in l for l in logs), "al cerrar manda D", falsa.recibidas[-1])

    # Patas sin medir: no se mueven
    solo_fl = Perro(simulado=False)
    falsa, esp, p, reloj, logs = armar(solo_fl, inicio=[100.0] * N_SERVOS)
    correr(p, reloj, 20, ([100.0] * N_SERVOS, [True] * N_SERVOS))
    s = pulsos_s(falsa)
    sin_pata = [ch for ch in range(N_SERVOS) if ch not in solo_fl.lim]
    revisa(sin_pata and all(fila[ch] == -1 for fila in s for ch in sin_pata) and any(fila[0] != -1 for fila in s),
           "un servo sin pata medida (sin límites) no se mueve", f"sin pata: canales {sin_pata}")
    e = p.estado(msgs)
    revisa(all(e.bloqueado[ch] for ch in sin_pata) and not any(e.bloqueado[:3]), "estado.bloqueado los marca")
    avisos = [l for l in logs if "no tiene límites" in l]
    revisa(len(avisos) == 3, "lo avisa una vez por pata", f"{len(avisos)} avisos")

    # Sin lectura creíble (auxiliar desconectada): no se prende
    falsa, esp, p, reloj, logs = armar(perro, aux=False)
    correr(p, reloj, 20, (list(perro.patas["fl"].de_pie_servo) * 4, [True] * N_SERVOS))
    s = pulsos_s(falsa)
    revisa(all(fila[ch] == -1 for fila in s for ch in range(3, 12)) and any(fila[1] != -1 for fila in s),
           "sin lectura creíble (AUX=NO) ese servo no se prende; FL sí",
           f"lecturas {['nan' if not math.isfinite(g) else round(g) for g in p.medido[2:5]]}")

    # Reinicio con aviso
    falsa, esp, p, reloj, logs = armar(perro)
    correr(p, reloj, 2)
    cmd = (list(p.medido), [True] * N_SERVOS)
    correr(p, reloj, 20, cmd)
    falsa.reiniciar("voltaje_bajo")
    n0 = len(falsa.recibidas)
    correr(p, reloj, 10, cmd)
    s = pulsos_s(falsa, n0)
    e = p.estado(msgs)
    revisa(p.modo == "REINICIADA" and p.reinicios == 1, "detecta el reinicio por la línea de arranque",
           f"modo {p.modo}, reinicios {p.reinicios}")
    revisa(e.modo == msgs.Modo.REINICIADA and e.reset == msgs.Reset.VOLTAJE_BAJO and e.reinicios == 1,
           "lo publica: modo REINICIADA y reset VOLTAJE_BAJO", f"{e.modo!r} {e.reset!r}")
    revisa(any(REINICIO in l for l in logs), "imprime el aviso REINICIO de arca9_serial.py")
    revisa(len(s) >= 9 and all(u == -1 for fila in s for u in fila),
           "después del reinicio no manda pulsos (los servos se quedan sueltos)")
    revisa(all(math.isfinite(g) for g in p.medido), "sigue publicando las lecturas")

    # Reinicio sin aviso (la línea de arranque se perdió): por SEG
    falsa, esp, p, reloj, logs = armar(perro)
    correr(p, reloj, 2)
    cmd = (list(p.medido), [True] * N_SERVOS)
    correr(p, reloj, 150, cmd)                 # 3 s: SEG sube
    falsa.reiniciar("watchdog", aviso=False)
    correr(p, reloj, 60, cmd)                  # 1.2 s: le toca un '?'
    revisa(p.modo == "REINICIADA" and any("SEG regresó" in l for l in logs),
           "detecta el reinicio sin línea de arranque (SEG regresó)",
           next((l for l in logs if "reinició" in l), "sin aviso"))

    # Respuesta tardía: no se confunde con la siguiente
    falsa, esp, p, reloj, logs = armar(perro)
    correr(p, reloj, 2)
    cmd = (list(p.medido), [True] * N_SERVOS)
    correr(p, reloj, 15, cmd)
    falsa.tardar(1, 0.3)
    correr(p, reloj, 1, cmd)
    reloj.avanzar(0.3)
    correr(p, reloj, 5, cmd)
    revisa(p.errores == 1 and esp.tardias == 1 and p.modo == "SIGUIENDO" and p.reinicios == 0
           and all(u != -1 for u in pulsos_s(falsa)[-1]),
           "una respuesta tardía cuenta como error y se tira", f"errores {p.errores}, tardías {esp.tardias}")

    # Un comando con NaN: ese servo no se mueve y se avisa
    falsa, esp, p, reloj, logs = armar(perro)
    correr(p, reloj, 2)
    q = list(p.medido)
    q[4] = math.nan
    correr(p, reloj, 15, (q, [True] * N_SERVOS))
    revisa(all(fila[4] == -1 for fila in pulsos_s(falsa)) and p.bloqueado[4] and any("NaN" in l for l in logs),
           "un comando con NaN: ese servo no se mueve y se avisa")

    # Sin respuesta: tras 3 seguidas, deja de mandar pulsos; al volver, sigue
    falsa, esp, p, reloj, logs = armar(perro)
    correr(p, reloj, 2)
    cmd = (list(p.medido), [True] * N_SERVOS)
    correr(p, reloj, 15, cmd)
    falsa.callar(3)
    correr(p, reloj, 3, cmd)
    e = p.estado(msgs)
    desconectado = not p.conectado and e.conectado is False and e.modo == msgs.Modo.SIN_ESP32
    n0 = len(falsa.recibidas)
    correr(p, reloj, 4, cmd)
    s = pulsos_s(falsa, n0)
    revisa(desconectado and all(u == -1 for u in s[0]) and p.conectado and all(u != -1 for u in s[-1]),
           "sin respuesta 3 veces: SIN_ESP32, conectado = false y no manda pulsos; al volver, sigue",
           next((l for l in logs if "no contesta" in l), ""))

    # Línea dañada
    falsa, esp, p, reloj, logs = armar(perro)
    correr(p, reloj, 3)
    falsa.danar(1)
    correr(p, reloj, 3)
    revisa(p.errores == 1 and p.modo == "SUELTO", "una respuesta dañada cuenta como error y sigue",
           f"errores {p.errores}")

    # Firmware viejo
    f2 = ESP32Falsa(esc, retro, latencia=False)
    f2.proto = 2
    esp2 = ESP32Puente(puerto_falso=f2)
    revisa(revisar_proto(esp2.info) is not None and "PROTO=3" in revisar_proto(esp2.info),
           "con PROTO=2 se niega (no hay comando S)")

    # Velocidad: el tope
    try:
        Puente(esp2, esc, retro, perro, vel_max=VEL_TOPE + 1)
        revisa(False, f"--vel-max arriba de {VEL_TOPE:g} °/s da error")
    except ValueError:
        revisa(True, f"--vel-max arriba de {VEL_TOPE:g} °/s da error")

    # El estado viaja
    e = p.estado(msgs)
    d = msgs.Arca9Estado.decode_payload(e.encode_payload())
    revisa(d.modo == msgs.Modo.SUELTO and d.proto == 3 and d.reset == msgs.Reset.ENCENDIDO and d.conectado,
           "Arca9Estado se codifica y decodifica", repr(d)[:90])


# ============================================================================ 2. patas y poses
def parte_poses():
    seccion("2. patas y poses")
    real = Perro(simulado=False)
    sim = Perro(simulado=True)
    revisa(list(real.patas) == ["fl"] and real.faltan() == ["fr", "rl", "rr"],
           "con el perro: solo la FL está medida (datos/2026-11-03_pata_fl)", str(list(real.patas)))
    revisa(list(sim.patas) == list(PATAS) and sim.simuladas == ["fr", "rl", "rr"] and sim.canales() == list(range(12)),
           "simulado: las 4 patas (FR, RL y RR copiadas de la FL)")

    with tempfile.TemporaryDirectory() as d:
        base = json.load(open(real.patas["fl"].fuente if os.path.isabs(real.patas["fl"].fuente)
                              else os.path.join(RAIZ, real.patas["fl"].fuente), encoding="utf-8"))
        malo = dict(base, canales={"abd": 0, "cad": 1, "rod": 2})
        ruta = os.path.join(d, "pata_fr.json")
        json.dump(malo, open(ruta, "w", encoding="utf-8"))
        try:
            Perro(rutas={"fr": ruta})
            revisa(False, "una pata FR con los canales de la FL da error")
        except ErrorPerro as e:
            revisa("canales" in str(e), "una pata FR con los canales de la FL da error", str(e)[:90])
        espejo = lambda d: {k: (180.0 - v if isinstance(v, (int, float)) else v) for k, v in d.items()}   # noqa: E731
        alreves = dict(base, canales={"abd": 3, "cad": 4, "rod": 5},
                       pose_recta_grados=espejo(base["pose_recta_grados"]),
                       pose_de_pie_grados=espejo(base["pose_de_pie_grados"]),
                       limites_grados={a: ([180.0 - v[1], 180.0 - v[0]] if isinstance(v, list) else v)
                                       for a, v in base["limites_grados"].items()})
        json.dump(alreves, open(ruta, "w", encoding="utf-8"))
        try:
            Perro(rutas={"fr": ruta})
            revisa(False, "una pata que gira al revés da error (pata.py supone el sentido de la FL)")
        except ErrorPerro as e:
            revisa("al revés" in str(e), "una pata que gira al revés da error (pata.py supone el sentido de la FL)",
                   str(e)[:90])
        bien = dict(base, pata="FR", canales={"abd": 3, "cad": 4, "rod": 5})
        json.dump(bien, open(ruta, "w", encoding="utf-8"))
        p2 = Perro(rutas={"fr": ruta})
        revisa(list(p2.patas) == ["fl", "fr"] and 4 in p2.lim, "una pata FR medida se agrega con sus canales")

    for nombre in poses.POSES:
        for perro, que in ((real, "FL"), (sim, "las 4")):
            q, act = poses.comando_de_pose(perro, nombre)
            margen = min(min(q[ch] - perro.lim[ch][0], perro.lim[ch][1] - q[ch]) for ch in perro.canales())
            revisa(margen >= 5.0 and sum(act) == len(perro.canales()),
                   f"pose '{nombre}' ({que}): dentro de los límites, a {margen:.1f}° o más")
    q_sent, _ = poses.comando_de_pose(sim, "sentado")
    revisa(q_sent[7] - q_sent[1] > 5 and q_sent[2] - q_sent[8] > 30, "sentado: traseras dobladas, delanteras de pie",
           f"cadera FL {q_sent[1]:.0f}° / RL {q_sent[7]:.0f}°, rodilla FL {q_sent[2]:.0f}° / RL {q_sent[8]:.0f}°")

    q0, act = poses.comando_de_pose(sim, "parado")
    q1, _ = poses.comando_de_pose(sim, "echado")
    tr = poses.Transicion(q0, q1, act, 30.0, 0.0)
    ts = [i * 0.002 for i in range(int(tr.T / 0.002) + 2)]
    qs = [tr.q(t) for t in ts]
    vel = max(abs(b[ch] - a[ch]) / 0.002 for a, b in zip(qs, qs[1:]) for ch in range(12))
    revisa(vel <= 30.0 * 1.01, "interpolación: ninguna articulación pasa de --vel", f"pico {vel:.1f} °/s en {tr.T:.2f} s")
    v0 = max(abs(qs[1][ch] - qs[0][ch]) / 0.002 for ch in range(12))
    v1 = max(abs(qs[-1][ch] - qs[-2][ch]) / 0.002 for ch in range(12))
    revisa(v0 < 0.5 and v1 < 0.5 and max(abs(a - b) for a, b in zip(qs[-1], q1)) < 1e-6,
           "interpolación: empieza y termina sin velocidad, y llega", f"{v0:.2f} y {v1:.2f} °/s")


# ============================================================================ 3. robot completo
def puerto_libre():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Robot:
    def __init__(self):
        self.http = puerto_libre()
        self.lineas = []
        env = dict(os.environ, NO_COLOR="1")
        self.proc = subprocess.Popen(
            [sys.executable, "-m", "agmen", "run", os.path.join(AGMEN_DIR, "robot_simulado.toml"), "--no-browser",
             "--http", f"127.0.0.1:{self.http}"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace", env=env,
            start_new_session=(os.name != "nt"),
            creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0))
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        for l in self.proc.stdout:
            self.lineas.append(l.rstrip("\n"))

    def log(self):
        return "\n".join(self.lineas)

    def api(self, ruta, cuerpo=None):
        url = f"http://127.0.0.1:{self.http}{ruta}"
        if cuerpo is None:
            return json.load(urllib.request.urlopen(url, timeout=2))
        req = urllib.request.Request(url, data=json.dumps(cuerpo).encode(), method="POST",
                                     headers={"X-Agmen": "1", "Content-Type": "application/json"})
        return json.load(urllib.request.urlopen(req, timeout=2))

    def cerrar(self):
        if self.proc.poll() is None:
            import signal
            if os.name == "nt":
                self.proc.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(15)
            except subprocess.TimeoutExpired:
                self.proc.kill()


class Escucha:
    """Junta en un hilo lo que llega a un topic: [(t, mensaje)]."""

    def __init__(self, tipo, topic):
        self.sub = agmen.Subscriber(tipo, topic)
        self.datos = []
        self.alto = False
        self.hilo = threading.Thread(target=self._loop, daemon=True)
        self.hilo.start()

    def _loop(self):
        while not self.alto:
            s = self.sub.receive(0.05)
            if s:
                self.datos.append((time.monotonic(), s.message))

    def ultimo(self):
        return self.datos[-1][1] if self.datos else None

    def entre(self, t0, t1):
        return [m for t, m in self.datos if t0 <= t <= t1]

    def parar(self):
        self.alto = True
        self.hilo.join(1)
        self.sub.close()


def esperar(cond, s, paso=0.1):
    lim = time.monotonic() + s
    while time.monotonic() < lim:
        if cond():
            return True
        time.sleep(paso)
    return cond()


def hub_ocupado():
    """¿Ya corre un agmen hub (otro agmen run) en esta máquina?"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", 7390))
        return False
    except OSError:
        return True
    finally:
        s.close()


def parte_robot():
    seccion("3. el robot completo: agmen run pc/agmen/robot_simulado.toml")
    if hub_ocupado():
        revisa(False, "no hay otro robot corriendo en esta máquina",
               "ya corre un agmen hub en 127.0.0.1:7390 (¿otro agmen run?). Ciérralo y vuelve a correr la prueba: "
               "dos robots con los mismos topics se mezclan")
        return
    sim = Perro(simulado=True)
    robot = Robot()
    q_esc = e_esc = c_esc = None
    try:
        nodos_ok = esperar(lambda: _nodos(robot) == {"hub", "mando", "puente", "poses", "manejo"}, 20)
        revisa(nodos_ok, "arrancan los 5 nodos (hub, mux 'mando', puente, poses y manejo)", str(_nodos(robot)))
        q_esc = Escucha(msgs.Arca9Articulaciones, "/arca9/articulaciones")
        e_esc = Escucha(msgs.Arca9Estado, "/arca9/estado")
        c_esc = Escucha(msgs.Arca9Comando, "/arca9/comando")
        llego = esperar(lambda: len(q_esc.datos) > 10, 15)
        revisa(llego, "llega /arca9/articulaciones")
        n0, t0 = len(q_esc.datos), time.monotonic()
        time.sleep(2.0)
        hz = (len(q_esc.datos) - n0) / (time.monotonic() - t0)
        revisa(40 <= hz <= 55, "/arca9/articulaciones a ~50 Hz", f"{hz:.1f} Hz")
        e = e_esc.ultimo()
        revisa(e is not None and e.conectado and e.proto == 3 and e.modo == msgs.Modo.SIGUIENDO and e.errores == 0,
               "/arca9/estado: conectado, PROTO=3, SIGUIENDO, sin errores", repr(e)[:110] if e else "no llegó")
        parado, _ = poses.comando_de_pose(sim, "parado")
        cerca = esperar(lambda: q_esc.ultimo() and max(abs(a - b) for a, b in zip(q_esc.ultimo().q, parado)) < 3, 10)
        dif = max(abs(a - b) for a, b in zip(q_esc.ultimo().q, parado)) if q_esc.ultimo() else math.nan
        revisa(cerca, "el perro llega a 'parado' (medido a menos de 3°)", f"{dif:.1f}°")

        # El panel le gana a poses.py
        def z_media(q):
            pies, _ = sim.pies_de_servos(q)
            return sum(z for _, z in pies.values()) / len(pies)
        t_antes = time.monotonic()
        z0 = z_media(c_esc.ultimo().q)
        for _ in range(30):                        # ↓ al 60 %, 1.5 s
            robot.api("/api/teleop", {"left": -0.6, "right": -0.6})
            time.sleep(0.05)
        t_suelta = time.monotonic()
        z1 = z_media(c_esc.ultimo().q)
        robot.api("/api/teleop", {"left": 0.0, "right": 0.0})
        time.sleep(0.25)
        z_quieto = z_media(c_esc.ultimo().q)
        revisa(z0 - z1 > 2.0 and "manda 'panel'" in robot.log(),
               "con las flechas del panel el cuerpo baja (el panel le gana a poses.py)",
               f"z de los pies {z0:.1f} → {z1:.1f} cm")
        revisa(abs(z_quieto - z1) < 0.2, "al soltar se queda donde va", f"{z_quieto:.1f} cm")
        regreso = esperar(lambda: robot.log().count("manda 'poses'") >= 2
                          and "el panel soltó el mando: retomo desde ahí" in robot.log(), 3)
        revisa(regreso,
               "al soltar, el mux le regresa el mando a poses.py y poses.py retoma desde ahí")
        time.sleep(1.5)
        cmds = [m.q for m in c_esc.entre(t_antes, time.monotonic())]
        salto = max(max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(cmds, cmds[1:]))
        revisa(salto < 3.5, "sin saltos en /arca9/comando al tomar y al soltar el mando",
               f"mayor cambio entre dos comandos seguidos: {salto:.2f}° ({len(cmds)} comandos)")

        # Watchdog: se apaga poses
        robot.api("/api/node/poses/stop", {})
        quieto = esperar(lambda: e_esc.ultimo() and e_esc.ultimo().modo == msgs.Modo.QUIETO, 4)
        revisa(quieto and "watchdog" in robot.log(), "al apagar poses.py, el puente se queda quieto (watchdog)",
               repr(e_esc.ultimo().modo) if e_esc.ultimo() else "")

        # D al apagar el puente
        robot.api("/api/node/puente/stop", {})
        soltó = esperar(lambda: "servos sueltos (D)" in robot.log(), 6)
        revisa(soltó, "al apagar el puente manda D (suelta los servos)")
    except Exception as ex:                        # noqa: BLE001 — que se vea qué pasó
        revisa(False, "el robot completo corre sin errores", f"{type(ex).__name__}: {ex}")
    finally:
        for x in (q_esc, e_esc, c_esc):
            if x:
                x.parar()
        robot.cerrar()
    errores = [l for l in robot.lineas if "Traceback" in l or "falló" in l]
    revisa(not errores, "ningún nodo tronó", errores[0] if errores else "")
    if fallas:
        print("\n  (log de agmen run, las últimas 40 líneas)")
        for l in robot.lineas[-40:]:
            print("   " + l)


def _nodos(robot):
    try:
        return {n["name"] for n in robot.api("/api/state")["nodes"] if n["state"] == "corriendo"}
    except Exception:                              # noqa: BLE001 — todavía no levanta el panel
        return set()


# ============================================================================ 4. arnés
def parte_arnes():
    seccion("4. con el firmware de verdad (pruebas/arnes_firmware --pty)")
    if os.name != "posix" or not shutil.which("g++"):
        print("  (se salta: necesita Linux o macOS con g++)")
        return
    arnes_dir = os.path.join(RAIZ, "pruebas", "arnes_firmware")
    with tempfile.TemporaryDirectory() as d:
        exe = os.path.join(d, "arnes")
        r = subprocess.run(["g++", "-std=c++17", "-Istubs", "-o", exe, "arnes.cpp"], cwd=arnes_dir,
                           capture_output=True, text=True)
        if r.returncode:
            revisa(False, "compila el arnés", r.stderr[-300:])
            return
        proc = subprocess.Popen([exe, "--pty"], stdout=subprocess.PIPE, text=True)
        try:
            pty = proc.stdout.readline().strip()
            revisa(pty.startswith("/dev/"), "el arnés expone la principal en un pseudo-terminal", pty)
            esp = ESP32Puente(pty)                         # pyserial de verdad, como con COM6
            revisa(esp.info.get("proto") == 3, "el puente se conecta (PROTO=3)", esp.info["linea"])
            perro = Perro(simulado=False)                  # solo la FL, como el perro hoy
            logs = []
            p = Puente(esp, esc, retro, perro, vel_max=60.0, log=logs.append)
            q, act = poses.comando_de_pose(perro, "parado")
            t_fin = time.monotonic() + 3.0
            while time.monotonic() < t_fin:
                t = time.monotonic()
                p.comando(q, act)
                p.ciclo()
                time.sleep(max(0.0, t + 0.02 - time.monotonic()))
            revisa(p.ciclos > 100 and p.errores == 0 and p.reinicios == 0,
                   "S a ~50 Hz con el firmware de verdad, sin errores",
                   f"{p.ciclos} ciclos en 3 s, S tarda {p.ciclo_ms:.1f} ms")
            dif = max(abs(p.medido[ch] - q[ch]) for ch in (0, 1, 2))
            revisa(dif < 6.0, "las lecturas de la FL llegan a la pose (el arnés no tiene el atraso del servo)",
                   " ".join(f"{p.medido[ch]:.1f}/{q[ch]:.1f}" for ch in (0, 1, 2)))
            revisa(all(math.isfinite(g) for g in p.medido), "llegan las 12 lecturas (FL y la auxiliar)")
            esp.comando("D", "OK D", 1.0)
            revisa(True, "el firmware contesta OK D")
            esp.cerrar()
        except Exception as ex:                    # noqa: BLE001
            revisa(False, "el puente habla con el firmware del arnés", f"{type(ex).__name__}: {ex}")
        finally:
            proc.kill()
            proc.wait()


def main():
    ap = argparse.ArgumentParser(description="Pruebas del puente entre Arca9 y Agmen")
    ap.add_argument("--rapido", action="store_true", help="solo la lógica y las poses (sin procesos ni red)")
    a = ap.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    print(f"Agmen {agmen.__version__} · Python {sys.version.split()[0]}")
    parte_logica()
    parte_poses()
    if not a.rapido:
        parte_robot()
        parte_arnes()
    print(f"\n{'HAY FALLAS' if fallas else 'TODO BIEN'} ({fallas} fallas)")
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
