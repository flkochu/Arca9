#!/usr/bin/env python3
"""
Prueba 05 · Marcha en el aire con una pata (FL) — Arca9
========================================================
El pie de la pata FL dibuja un paso en el aire. Los ángulos de las 3 articulaciones salen de la
cinemática inversa con las medidas de la pata (pc/pata.py). La ESP32 principal ejecuta la
trayectoria de los 3 servos a la vez con su propio reloj y graba sus cables verdes (comando T,
protocolo v3). La prueba compara dos formas de mandar el paso:

  - sin compensar: a cada instante se manda el ángulo que se quiere en ese instante;
  - con adelanto:  se manda el ángulo de 100 ms después, porque la prueba 04 midió que el servo va
                   ~100 ms atrás del pulso.

El paso (pie, en cm, desde el eje de la cadera; x = hacia el frente, z = hacia abajo):
  - apoyo (60 % del periodo): el pie va en línea recta de x0 + L/2 a x0 − L/2, a la altura z0;
  - vuelo (40 %): regresa al frente y sube h, con una curva suave (cicloide).
Por defecto x0 = 2 cm, z0 = 18.5 cm (casi la pose de pie), L = 6 cm, h = 3 cm, 4 pasos por
corrida, y periodos de 2.0, 1.2 y 0.8 s. Cada periodo se corre sin compensar y con adelanto.

Antes de mover nada, el script revisa que todo el paso quede dentro de los límites de la pata
(los medidos menos 5°, y la rodilla sin pasar de recta) y que ninguna articulación pase de
240 °/s (lo que se midió en la prueba 04).

Hardware: esp32_principal.ino con PROTO=3 (comando T) y la pata FL armada, colgando al aire.

Uso (en la carpeta de este archivo):
  python marcha.py --simulado                     # practicar sin hardware
  python marcha.py --port COM6 --revisar          # ángulos y posición del pie en vivo, servos sueltos
  python marcha.py --port COM6 --voltaje 5.99     # la prueba (~3 min)
  python marcha.py --port COM6 --periodos 2.0     # una corrida corta
  python marcha.py --reanalizar ultima

Qué se calcula en cada corrida (en los pasos, sin las esperas):
  - Error de cada articulación: medido − deseado (RMS y máximo, en grados).
  - Atraso de cada articulación: cuánto hay que recorrer en el tiempo el ángulo deseado para que
    se parezca más al medido (ms). Con adelanto debería quedar cerca de 0.
  - Error del pie: distancia entre donde está el pie (cinemática con los ángulos medidos) y
    donde debía estar (RMS y máximo, en cm). Y el largo y la altura del paso que sí hizo.

Resultados en resultados_marcha/AAAAMMDD_HHMMSS/:
  meta.json, corridas.csv    parámetros y estado de cada corrida
  muestras.csv               lecturas (corrida, t, mV de los 3)
  comandos.csv               pulsos aplicados (corrida, t, µs de los 3)
  metricas.csv, resumen.md   métricas por corrida y tabla resumen
  marcha.png                 error y atraso: sin compensar contra con adelanto
  angulos_<periodo>s.png     ángulo contra tiempo de cada articulación (deseado, mandado, medido)
  pie.png                    trayectoria del pie: deseada contra medida
"""
import argparse
import csv
import datetime
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from arca9_serial import ESP32, ErrorESP32
from escala import Escala, Retro, US_CERO, US_LIMITE, N_SERVOS, MV_MIN, MV_MAX
from pata import Pata, ErrorPata, ART

DT = 0.020                      # un punto de la trayectoria cada 20 ms (lo que ve el servo)
PERIODOS = [2.0, 1.2, 0.8]
X0, Z0, LARGO, ALTO, APOYO = 2.0, 18.5, 6.0, 3.0, 0.6
PASOS = 4
ESPERA = 0.5                    # s quieto antes y después de los pasos
ADELANTO = 0.100                # s (prueba 04: ~100 ms en FL)
VEL_MAX = 240.0                 # °/s, lo más rápido que se midió en la prueba 04
VEL_ACERCAR = 40.0              # °/s para llevar la pata al inicio
NOMBRES = {"abd": "abducción", "cad": "cadera", "rod": "rodilla"}

SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
AZUL, NARANJA, VERDE = "#2a78d6", "#eb6834", "#1baf7a"

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


def fmt(x, d=1):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    return f"{x:.{d}f}"


def guardar_json(ruta, obj):
    def limpio(x):
        if isinstance(x, dict):
            return {str(k): limpio(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [limpio(v) for v in x]
        if isinstance(x, (float, np.floating)):
            x = float(x)
            return round(x, 5) if math.isfinite(x) else None
        if isinstance(x, np.integer):
            return int(x)
        return x
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(limpio(obj), f, ensure_ascii=False, indent=2)


# ======================================================================= el paso

class Paso:
    def __init__(self, periodo, x0=X0, z0=Z0, largo=LARGO, alto=ALTO, apoyo=APOYO, pasos=PASOS, espera=ESPERA):
        self.T, self.x0, self.z0, self.L, self.h, self.d = periodo, x0, z0, largo, alto, apoyo
        self.n, self.espera = pasos, espera
        self.total = espera + pasos * periodo + espera

    def pie(self, t):
        """Posición deseada del pie (x, z) en cm al tiempo t (s)."""
        t = t - self.espera
        if t < 0 or t >= self.n * self.T:
            u = 0.0
        else:
            u = (t % self.T) / self.T
        if u < self.d:
            s = u / self.d
            return self.x0 + self.L / 2 - self.L * s, self.z0
        s = (u - self.d) / (1 - self.d)
        x = self.x0 - self.L / 2 + self.L * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        z = self.z0 - self.h * (1 - math.cos(2 * math.pi * s)) / 2
        return x, z

    def servos(self, pata, t):
        """Grados reales de los 3 servos (abd, cad, rod) para el pie deseado al tiempo t."""
        q_cad, q_rod = pata.ik(*self.pie(t))
        return pata.servo(0.0, q_cad, q_rod)


def plan(paso, pata, esc, adelanto):
    """Puntos de la trayectoria (cada 20 ms): grados deseados, grados mandados y µs mandados."""
    n = int(round(paso.total / DT))
    t = np.arange(n) * DT
    deseado = np.array([paso.servos(pata, ti) for ti in t])
    mandado = np.array([paso.servos(pata, min(ti + adelanto, paso.total)) for ti in t])
    us = np.array([[esc.real_a_us(g) for g in fila] for fila in mandado], dtype=int)
    return t, deseado, mandado, us


def revisar_plan(paso, pata):
    """Revisa límites y velocidades con un muestreo fino. Regresa (vel_max por articulación, problemas)."""
    tt = np.arange(0, paso.total, 0.002)
    try:
        th = np.array([paso.servos(pata, t) for t in tt])
    except ErrorPata as e:
        return None, [str(e)]
    problemas = []
    for i, a in enumerate(ART):
        lo, hi = pata.lim[a]
        if th[:, i].min() < lo - 1e-6 or th[:, i].max() > hi + 1e-6:
            problemas.append(f"{NOMBRES[a]}: el paso va de {th[:, i].min():.1f}° a {th[:, i].max():.1f}°, "
                             f"fuera de {lo:.1f}°–{hi:.1f}°")
    vel = np.abs(np.diff(th, axis=0)).max(axis=0) / 0.002
    for i, a in enumerate(ART):
        if vel[i] > VEL_MAX:
            problemas.append(f"{NOMBRES[a]}: necesita {vel[i]:.0f} °/s (máximo {VEL_MAX:.0f}); usa un periodo más largo")
    return vel, problemas


# ======================================================================= simulador

class ESP32Simulado:
    """3 servos con un tiempo muerto de 70 ms y un primer orden de 30 ms (≈ 100 ms de atraso,
    como en la prueba 04), velocidad máxima y ruido. Hace el comando T como el firmware."""

    def __init__(self, esc, retro, semilla=5):
        self.rng = np.random.default_rng(semilla)
        self.esc, self.retro = esc, retro
        self.pos = {ch: 90.0 + 20 * self.rng.random() for ch in range(N_SERVOS)}   # grados reales
        self.us = {ch: 0 for ch in range(N_SERVOS)}
        self.info = {"linea": "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=3 (SIMULADO)",
                     "pca": "ok", "fb": 12, "aux": "ok", "proto": 3}

    def _mv(self, ch):
        c = self.retro.canales[ch]
        conv = self.esc.real_a_conv(self.pos[ch])
        if len(c["mv"]) >= 2:                                   # inverso de la misma tabla que usa el análisis
            orden = np.argsort(c["ang"])
            base = float(np.interp(conv, np.array(c["ang"])[orden], np.array(c["mv"])[orden]))
        else:
            base = (conv - c["b"]) / c["a"]
        return int(round(base + self.rng.normal(0, 1.7)))

    def mover(self, ch, us):
        us = int(min(max(us, US_CERO), US_LIMITE))
        self.us[ch] = us
        self.pos[ch] = self.esc.us_a_real(us)
        return us

    def leer_todos(self):
        return [self._mv(ch) for ch in range(N_SERVOS)]

    def trayectoria(self, canales, puntos, grabar=True):
        for j, ch in enumerate(canales):
            if self.us[ch] != puntos[0][j]:
                raise ErrorESP32(f"la ESP32 detuvo la trayectoria: E T: primero manda P {ch} {puntos[0][j]}")
        muerto, tau, vmax, dt = 0.070, 0.030, 300.0, 0.0005
        total = len(puntos) * DT
        pend = []                                              # (t en que el servo "ve" el punto, objetivo)
        obj = {ch: self.pos[ch] for ch in canales}
        M, C = [], []
        k, t, sig_m = 0, 0.0, 0.0
        while t < total:
            if k < len(puntos) and t >= k * DT:
                C.append((round(t, 4), list(puntos[k])))
                pend.append((t + muerto, [self.esc.us_a_real(u) for u in puntos[k]]))
                k += 1
            while pend and pend[0][0] <= t:
                _, vals = pend.pop(0)
                obj = dict(zip(canales, vals))
            for ch in canales:
                v = max(-vmax, min(vmax, (obj[ch] - self.pos[ch]) / tau))
                self.pos[ch] += v * dt
            if t >= sig_m:
                M.append((round(t, 4), [self._mv(ch) for ch in canales]))
                sig_m += 0.002
            t += dt
        for j, ch in enumerate(canales):
            self.us[ch] = puntos[-1][j]
        return {"canales": canales, "M": M, "C": C, "total_s": total}

    def soltar(self):
        self.us = {ch: 0 for ch in range(N_SERVOS)}

    def esperar(self, s):
        pass

    def cerrar(self):
        pass


# ======================================================================= medición

def acercar(esp, canales, desde, hasta, esc, dt=0.03):
    """Lleva los 3 servos juntos, en línea recta, de `desde` a `hasta` (µs) a ~VEL_ACERCAR °/s."""
    salto = max(abs(h - d) for d, h in zip(desde, hasta)) / esc.us_por_grado
    n = max(1, int(math.ceil(salto / (VEL_ACERCAR * dt))))
    for k in range(1, n + 1):
        for ch, d, h in zip(canales, desde, hasta):
            esp.mover(ch, int(round(d + (h - d) * k / n)))
        esp.esperar(dt)


def medir(a, pata, esc, retro):
    corridas = [(T, modo) for T in a.periodos for modo in ("sin", "con")]
    pasos = {T: Paso(T, a.x0, a.z0, a.largo, a.alto, APOYO, a.pasos) for T in a.periodos}
    print("\n" + "=" * 78)
    print(f"PRUEBA 05 · MARCHA EN EL AIRE — pata {pata.nombre} (canales {pata.canales})")
    print("=" * 78)
    print(f"Paso: {a.largo:g} cm de largo, {a.alto:g} cm de alto, centrado en x = {a.x0:g} cm, z = {a.z0:g} cm · "
          f"{a.pasos} pasos por corrida · adelanto {a.adelanto_ms:g} ms")
    print("\n  periodo   cadera °     rodilla °    vel. máx (cad / rod)   duración")
    for T in a.periodos:
        vel, probl = revisar_plan(pasos[T], pata)
        if probl:
            sys.exit("No se puede hacer el paso de %.2f s:\n  - " % T + "\n  - ".join(probl))
        th = np.array([pasos[T].servos(pata, t) for t in np.arange(0, pasos[T].total, 0.01)])
        print(f"  {T:5.2f} s   {th[:, 1].min():5.1f}–{th[:, 1].max():5.1f}   {th[:, 2].min():5.1f}–{th[:, 2].max():5.1f}"
              f"     {vel[1]:5.0f} / {vel[2]:5.0f} °/s         {pasos[T].total:4.1f} s")
    print(f"\n  {len(corridas)} corridas: cada periodo sin compensar y con adelanto. Límites del script: "
          + ", ".join(f"{NOMBRES[x]} {pata.lim[x][0]:.0f}–{pata.lim[x][1]:.0f}°" for x in ART))

    esp = ESP32Simulado(esc, retro, a.semilla) if a.simulado else ESP32(a.port)
    print(f"\nESP32: {esp.info['linea']}")
    if esp.info.get("proto", 1) < 3:
        esp.cerrar()
        sys.exit("La principal no tiene el comando T (falta PROTO=3 en '?'). Carga el esp32_principal.ino nuevo.")
    v = esp.leer_todos()
    # Con el cable verde suelto el pin no lee 0: el ADC no baja de ~142 mV. En su tope el servo da ~180 mV.
    malos = [c for c in pata.canales if v[c] is None or not MV_MIN <= v[c] <= MV_MAX]
    if malos:
        esp.soltar()
        esp.cerrar()
        sys.exit("No se mueve nada: el cable verde de " + ", ".join(f"canal {c} da {v[c]} mV" for c in malos)
                 + f" (lo creíble es {MV_MIN}–{MV_MAX} mV). ¿Cable verde suelto o divisor desconectado?"
                 + " Revisa con --revisar.")
    actual_g = [retro.grados(c, v[c]) for c in pata.canales]
    fuera = pata.fuera(actual_g)
    if fuera:
        esp.soltar()
        esp.cerrar()
        sys.exit("La pata está fuera de los límites, así que no se mueve nada:\n  - "
                 + "\n  - ".join(f"{NOMBRES[x]} en {g:.1f}° (límites {lo:.1f}°–{hi:.1f}°)" for x, g, (lo, hi) in fuera)
                 + "\nCon los servos sueltos, ponla a mano en la pose de pie (pie bajo la cadera) y vuelve a correr."
                 + "\nPara verla en vivo: python marcha.py --port COM6 --revisar")
    if not a.simulado:
        print(f"\nLa pata está en {', '.join(f'{g:.0f}°' for g in actual_g)}. Primero va despacio a la pose de inicio.")
        print("Que cuelgue al aire, sin nada cerca. Ctrl+C detiene todo (y suelta los servos).")
        input("Enter para empezar (Ctrl+C para salir) ")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = os.path.join(a.salida, stamp + ("_sim" if a.simulado else ""))
    os.makedirs(carpeta, exist_ok=True)
    meta = {"fecha": stamp, "prueba": "05_marcha", "pata": pata.nombre, "canales": pata.canales,
            "periodos_s": a.periodos, "x0_cm": a.x0, "z0_cm": a.z0, "largo_cm": a.largo, "alto_cm": a.alto,
            "apoyo": APOYO, "pasos": a.pasos, "espera_s": ESPERA, "adelanto_ms": a.adelanto_ms, "dt_s": DT,
            "voltaje": a.voltaje, "simulado": a.simulado, "esp32": esp.info, "escala": esc.fuente,
            "calibracion": retro.fuente, "pata_json": pata.fuente}
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    f_m = open(os.path.join(carpeta, "muestras.csv"), "w", newline="", encoding="utf-8")
    f_c = open(os.path.join(carpeta, "comandos.csv"), "w", newline="", encoding="utf-8")
    f_k = open(os.path.join(carpeta, "corridas.csv"), "w", newline="", encoding="utf-8")
    w_m, w_c, w_k = csv.writer(f_m), csv.writer(f_c), csv.writer(f_k)
    w_m.writerow(["corrida", "t_s"] + [f"mv_{x}" for x in ART])
    w_c.writerow(["corrida", "t_s"] + [f"us_{x}" for x in ART])
    w_k.writerow(["corrida", "periodo_s", "modo", "adelanto_ms", "n_puntos", "n_muestras", "estado"])

    actual_us = [esc.real_a_us(g, recortar=True) for g in actual_g]
    cancelado = False
    try:
        for i, (T, modo) in enumerate(corridas):
            adel = a.adelanto_ms / 1000.0 if modo == "con" else 0.0
            _, _, _, us = plan(pasos[T], pata, esc, adel)
            inicio = [int(u) for u in us[0]]
            acercar(esp, pata.canales, actual_us, inicio, esc)
            esp.esperar(1.0)
            print(f"\n  corrida {i + 1}/{len(corridas)}: periodo {T:g} s, "
                  f"{'sin compensar' if modo == 'sin' else f'con adelanto de {a.adelanto_ms:g} ms'} ...", end="", flush=True)
            estado = "ok"
            try:
                r = esp.trayectoria(pata.canales, [list(map(int, f)) for f in us])
            except ErrorESP32 as e:
                estado = f"error: {e}"
                r = {"M": [], "C": []}
                print(" ERROR")
            for t, mv in r["M"]:
                w_m.writerow([i + 1, f"{t:.4f}"] + mv)
            for t, uu in r["C"]:
                w_c.writerow([i + 1, f"{t:.4f}"] + uu)
            w_k.writerow([i + 1, T, modo, a.adelanto_ms if modo == "con" else 0, len(us), len(r["M"]), estado])
            for f in (f_m, f_c, f_k):
                f.flush()
            if estado == "ok":
                actual_us = [int(u) for u in us[-1]]
                m = metricas_corrida(i + 1, T, modo, r["M"], pasos[T], pata, retro)
                print(f" error pie {fmt(m['pie_rms_cm'], 2)} cm RMS · atraso cad {fmt(m['atraso_cad_ms'], 0)} ms, "
                      f"rod {fmt(m['atraso_rod_ms'], 0)} ms")
            else:
                raise ErrorESP32(estado[len("error: "):])     # no se sabe cómo quedó la ESP32: se para todo
        # al final, de vuelta a la pose de pie
        acercar(esp, pata.canales, actual_us, [esc.real_a_us(g) for g in pata.de_pie_servo], esc)
    except KeyboardInterrupt:
        cancelado = True
        print("\n\nDetenido. Se analiza lo que se alcanzó a medir.")
    except ErrorESP32 as e:
        cancelado = True
        print(f"\n\nError: {e}. Se analiza lo que se alcanzó a medir.")
    finally:
        esp.soltar()
        esp.cerrar()
        for f in (f_m, f_c, f_k):
            f.close()
    meta["cancelado"] = cancelado
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    return carpeta


# ======================================================================= análisis

def series(M, paso, pata, retro):
    """De las lecturas a: t, grados medidos (n×3), grados deseados (n×3), pie medido y deseado (n×2)."""
    t = np.array([m[0] for m in M])
    mv = np.array([m[1] for m in M], float)
    med = np.column_stack([[retro.grados(ch, x) for x in mv[:, j]] for j, ch in enumerate(pata.canales)])
    des = np.array([paso.servos(pata, ti) for ti in t])
    q = np.array([pata.q(fila) for fila in med])
    pie_m = np.array([pata.fk(qc, qr) for _, qc, qr in q])
    pie_d = np.array([paso.pie(ti) for ti in t])
    return t, med, des, pie_m, pie_d


def atraso(t, med, paso, pata, j, ventana):
    """ms que hay que atrasar el deseado para parecerse más al medido (busca de −100 a 250 ms)."""
    mejor, mejor_e = float("nan"), float("inf")
    for lag in np.arange(-0.100, 0.2501, 0.002):
        d = np.array([paso.servos(pata, ti - lag)[j] for ti in t[ventana]])
        e = float(np.mean((med[ventana, j] - d) ** 2))
        if e < mejor_e:
            mejor, mejor_e = lag, e
    return mejor * 1000


def metricas_corrida(n, T, modo, M, paso, pata, retro):
    t, med, des, pie_m, pie_d = series(M, paso, pata, retro)
    ventana = (t >= paso.espera) & (t < paso.espera + paso.n * paso.T)
    r = {"corrida": n, "periodo_s": T, "modo": modo}
    for j, a in enumerate(ART):
        e = med[ventana, j] - des[ventana, j]
        r[f"rms_{a}"] = float(np.sqrt(np.mean(e ** 2)))
        r[f"max_{a}"] = float(np.max(np.abs(e)))
    for j, a in ((1, "cad"), (2, "rod")):
        r[f"atraso_{a}_ms"] = atraso(t, med, paso, pata, j, ventana)
    dist = np.hypot(*(pie_m[ventana] - pie_d[ventana]).T)
    r["pie_rms_cm"] = float(np.sqrt(np.mean(dist ** 2)))
    r["pie_max_cm"] = float(np.max(dist))
    # largo y altura del paso que sí hizo (promedio por paso)
    largos, altos = [], []
    for k in range(paso.n):
        s = (t >= paso.espera + k * paso.T) & (t < paso.espera + (k + 1) * paso.T)
        if s.sum() > 15:
            k9 = np.ones(9) / 9                               # media de 18 ms: que el ruido no infle el rango
            xs = np.convolve(pie_m[s, 0], k9, mode="valid")
            zs = np.convolve(pie_m[s, 1], k9, mode="valid")
            largos.append(float(np.ptp(xs)))
            altos.append(float(np.ptp(zs)))
    r["largo_cm"] = float(np.mean(largos)) if largos else float("nan")
    r["alto_cm"] = float(np.mean(altos)) if altos else float("nan")
    return r


def cargar(carpeta):
    corridas = []
    with open(os.path.join(carpeta, "corridas.csv"), newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            corridas.append(r)
    M = {}
    def leer(nombre):
        ruta = os.path.join(carpeta, nombre)
        with open(ruta, encoding="utf-8") as f:
            if sum(1 for _ in f) < 2:
                return np.zeros((0, 5))
        return np.genfromtxt(ruta, delimiter=",", skip_header=1, ndmin=2)
    arr = leer("muestras.csv")
    if arr.size:
        for n in np.unique(arr[:, 0]).astype(int):
            sel = arr[arr[:, 0] == n]
            M[n] = [(row[1], list(row[2:5])) for row in sel]
    C = {}
    arr = leer("comandos.csv")
    if arr.size:
        for n in np.unique(arr[:, 0]).astype(int):
            C[n] = arr[arr[:, 0] == n][:, 1:5]
    return corridas, M, C


def analizar(carpeta, pata, esc, retro):
    with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    corridas, M, C = cargar(carpeta)
    pasos = {}
    filas = []
    for k in corridas:
        n, T = int(k["corrida"]), float(k["periodo_s"])
        if k["estado"] != "ok" or n not in M:
            continue
        pasos[T] = Paso(T, meta["x0_cm"], meta["z0_cm"], meta["largo_cm"], meta["alto_cm"], meta["apoyo"],
                        meta["pasos"], meta["espera_s"])
        filas.append(metricas_corrida(n, T, k["modo"], M[n], pasos[T], pata, retro))
    if not filas:
        print("No hay corridas completas para analizar.")
        return
    with open(os.path.join(carpeta, "metricas.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0].keys()))
        w.writeheader()
        for r in filas:
            w.writerow({kk: (round(x, 3) if isinstance(x, float) else x) for kk, x in r.items()})
    escribir_md(carpeta, meta, filas)
    graficas(carpeta, meta, corridas, filas, M, C, pasos, pata, esc, retro)
    imprimir(filas, carpeta)


def escribir_md(carpeta, meta, filas):
    L = ["# 05 · Marcha en el aire — resumen", "", "| | |", "|---|---|",
         f"| Fecha | {meta.get('fecha')} |",
         f"| Pata | {meta.get('pata')} (canales {meta.get('canales')}) |",
         f"| Paso | {meta['largo_cm']:g} cm de largo, {meta['alto_cm']:g} cm de alto, centro en x = {meta['x0_cm']:g} cm, "
         f"z = {meta['z0_cm']:g} cm; {int(meta['apoyo'] * 100)} % de apoyo; {meta['pasos']} pasos por corrida |",
         f"| Adelanto | {meta['adelanto_ms']:g} ms |",
         f"| Fuente | {fmt(meta.get('voltaje'), 2) if meta.get('voltaje') else '—'} V |",
         f"| Simulado | {'sí' if meta.get('simulado') else 'no'} |", "",
         "| Periodo (s) | Modo | Pie RMS (cm) | Pie máx (cm) | Largo hecho (cm) | Alto hecho (cm) | "
         "Cadera RMS (°) | Rodilla RMS (°) | Abducción RMS (°) | Atraso cadera (ms) | Atraso rodilla (ms) |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in filas:
        L.append(f"| {r['periodo_s']:g} | {'sin compensar' if r['modo'] == 'sin' else 'con adelanto'} | "
                 f"{fmt(r['pie_rms_cm'], 2)} | {fmt(r['pie_max_cm'], 2)} | {fmt(r['largo_cm'], 1)} | {fmt(r['alto_cm'], 1)} | "
                 f"{fmt(r['rms_cad'])} | {fmt(r['rms_rod'])} | {fmt(r['rms_abd'])} | "
                 f"{fmt(r['atraso_cad_ms'], 0)} | {fmt(r['atraso_rod_ms'], 0)} |")
    L += ["", f"Lo pedido: {meta['largo_cm']:g} cm de largo y {meta['alto_cm']:g} cm de alto. "
          "El alto hecho incluye la diferencia de altura entre el apoyo y el punto más alto del vuelo.", ""]
    with open(os.path.join(carpeta, "resumen.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("AVISO: falta matplotlib; no se hicieron las gráficas.")
        return None
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": SURF, "axes.facecolor": SURF})
    return plt


def graficas(carpeta, meta, corridas, filas, M, C, pasos, pata, esc, retro):
    plt = _plt()
    if plt is None:
        return
    per = sorted({r["periodo_s"] for r in filas}, reverse=True)
    idx = {(float(k["periodo_s"]), k["modo"]): int(k["corrida"]) for k in corridas if k["estado"] == "ok"}
    met = {(r["periodo_s"], r["modo"]): r for r in filas}

    # 1) Resumen: error del pie y atraso, sin contra con
    fig, ejes = plt.subplots(1, 3, figsize=(11, 3.6))
    x = np.arange(len(per))
    for ax, campo, txt in ((ejes[0], "pie_rms_cm", "error del pie, RMS (cm)"),
                           (ejes[1], "atraso_cad_ms", "atraso de la cadera (ms)"),
                           (ejes[2], "atraso_rod_ms", "atraso de la rodilla (ms)")):
        for k, (modo, col, et) in enumerate((("sin", MUTED, "sin compensar"), ("con", AZUL, "con adelanto"))):
            ys = [met.get((T, modo), {}).get(campo, float("nan")) for T in per]
            ax.bar(x + (k - 0.5) * 0.38, ys, width=0.36, color=col, label=et, zorder=3)
        ax.set_xticks(x)
        ax.set_xticklabels([f"{T:g} s" for T in per])
        ax.set_title(txt, loc="left", fontsize=9.5, color=INK)
        ax.axhline(0, color=MUTED, lw=0.8)
        ax.grid(True, axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    ejes[0].legend(frameon=False, fontsize=8)
    fig.suptitle("Marcha en el aire, pata FL: sin compensar contra adelanto de "
                 f"{meta['adelanto_ms']:g} ms" + (" · SIMULADO" if meta.get("simulado") else ""),
                 x=0.02, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(carpeta, "marcha.png"), dpi=150)
    plt.close(fig)

    # 2) Ángulo contra tiempo, una figura por periodo: 3 articulaciones × (sin, con)
    for T in per:
        fig, ejes = plt.subplots(3, 2, figsize=(11, 7), sharex=True)
        for c, modo in enumerate(("sin", "con")):
            n = idx.get((T, modo))
            if n is None:
                continue
            t, med, des, _, _ = series(M[n], pasos[T], pata, retro)
            cc = C.get(n)
            for j, a in enumerate(ART):
                ax = ejes[j][c]
                ax.plot(t, des[:, j], color=INK, lw=1.2, ls="--", label="deseado")
                if cc is not None and modo == "con":
                    ax.step(cc[:, 0], [esc.us_a_real(u) for u in cc[:, j + 1]], where="post", color=MUTED, lw=0.9,
                            label="mandado (adelantado)")
                ax.plot(t, med[:, j], color=AZUL if modo == "con" else NARANJA, lw=1.1, label="medido")
                ax.grid(True, color=GRID, lw=0.5)
                if np.ptp(des[:, j]) < 4:                      # articulación quieta: ±5° para que el ruido no parezca movimiento
                    ax.set_ylim(np.mean(des[:, j]) - 5, np.mean(des[:, j]) + 5)
                if c == 0:
                    ax.set_ylabel(f"{NOMBRES[a]} (°)")
                if j == 0:
                    r = met.get((T, modo), {})
                    ax.set_title(f"{'sin compensar' if modo == 'sin' else 'con adelanto'} · pie {fmt(r.get('pie_rms_cm'), 2)} cm RMS",
                                 loc="left", fontsize=9.5, color=INK)
                    ax.legend(frameon=False, fontsize=7, loc="upper right")
            ejes[2][c].set_xlabel("tiempo (s)")
        fig.suptitle(f"Pata FL · periodo {T:g} s · ángulo de cada servo contra tiempo (grados reales)",
                     x=0.02, ha="left", fontsize=11, fontweight="bold", color=INK)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        fig.savefig(os.path.join(carpeta, f"angulos_{T:g}s.png"), dpi=140)
        plt.close(fig)

    # 3) Trayectoria del pie: deseada contra medida
    fig, ejes = plt.subplots(1, len(per), figsize=(3.6 * len(per), 3.8), squeeze=False)
    for i, T in enumerate(per):
        ax = ejes[0][i]
        tt = np.arange(0, pasos[T].T, 0.005)
        d = np.array([pasos[T].pie(pasos[T].espera + ti) for ti in tt])
        ax.plot(d[:, 0], d[:, 1], color=INK, lw=1.4, ls="--", label="deseado")
        for modo, col, et in (("sin", NARANJA, "sin compensar"), ("con", AZUL, "con adelanto")):
            n = idx.get((T, modo))
            if n is None:
                continue
            t, _, _, pie_m, _ = series(M[n], pasos[T], pata, retro)
            s = (t >= pasos[T].espera + pasos[T].T) & (t < pasos[T].espera + (pasos[T].n - 1) * pasos[T].T)
            ax.plot(pie_m[s, 0], pie_m[s, 1], color=col, lw=0.9, label=et)
        ax.invert_yaxis()
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(f"periodo {T:g} s", loc="left", fontsize=9.5, color=INK)
        ax.set_xlabel("x: hacia el frente (cm)")
        if i == 0:
            ax.set_ylabel("z: hacia abajo (cm)")
            ax.legend(frameon=False, fontsize=7)
        ax.grid(True, color=GRID, lw=0.5)
    fig.suptitle("Trayectoria del pie (desde la cadera), pasos 2 y 3", x=0.02, ha="left", fontsize=11,
                 fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(carpeta, "pie.png"), dpi=150)
    plt.close(fig)


def imprimir(filas, carpeta):
    print("\n" + "=" * 96)
    print("RESULTADOS")
    print("=" * 96)
    print("  periodo  modo            pie RMS cm  pie máx cm  largo cm  alto cm  cad RMS °  rod RMS °  atraso cad/rod ms")
    for r in filas:
        print(f"  {r['periodo_s']:6.2f}   {'sin compensar' if r['modo'] == 'sin' else 'con adelanto ':<14}  "
              f"{fmt(r['pie_rms_cm'], 2):>9}  {fmt(r['pie_max_cm'], 2):>10}  {fmt(r['largo_cm']):>8}  {fmt(r['alto_cm']):>7}  "
              f"{fmt(r['rms_cad']):>9}  {fmt(r['rms_rod']):>9}  {fmt(r['atraso_cad_ms'], 0):>8} / {fmt(r['atraso_rod_ms'], 0)}")
    print(f"\nResultados en {carpeta}")
    print("  resumen.md, marcha.png, pie.png, angulos_<periodo>s.png, metricas.csv")


# ======================================================================= revisar

def revisar(a, pata, retro):
    esp = ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    esp.soltar()
    print("\nServos sueltos. Mueve la pata con la mano. Ctrl+C para salir.\n")
    print("    servo (abd / cad / rod)        articulación (abd / cad / rod)      pie x, z (cm)")
    try:
        while True:
            try:
                v = esp.leer_todos()
                th = [retro.grados(c, v[c]) for c in pata.canales]
                q = pata.q(th)
                x, z = pata.fk(q[1], q[2])
                sueltos = [c for c in pata.canales if v[c] is None or not MV_MIN <= v[c] <= MV_MAX]
                if sueltos:
                    aviso = "  ¡cable verde suelto? canal " + ", ".join(str(c) for c in sueltos) + "!"
                else:
                    aviso = "" if pata.dentro(th) else "  ¡fuera de límites!"
                print(f"\r  {th[0]:6.1f} {th[1]:6.1f} {th[2]:6.1f}°        {q[0]:+6.1f} {q[1]:+6.1f} {q[2]:+6.1f}°"
                      f"           {x:+5.1f}, {z:5.1f}{aviso}".ljust(100), end="", flush=True)
            except ErrorESP32:
                pass
            time.sleep(0.15)
    except KeyboardInterrupt:
        print()
    finally:
        esp.cerrar()


# ======================================================================= principal

def main():
    ap = argparse.ArgumentParser(description="Prueba 05 de Arca9: marcha en el aire con la pata FL.")
    ap.add_argument("--port", help="puerto de la ESP32 principal (COM6, /dev/ttyUSB0)")
    ap.add_argument("--periodos", type=float, nargs="+", default=PERIODOS, help="s por paso, en orden")
    ap.add_argument("--pasos", type=int, default=PASOS, help="pasos por corrida")
    ap.add_argument("--largo", type=float, default=LARGO, help="cm")
    ap.add_argument("--alto", type=float, default=ALTO, help="cm")
    ap.add_argument("--x0", type=float, default=X0, help="centro del paso, cm hacia el frente de la cadera")
    ap.add_argument("--z0", type=float, default=Z0, help="altura del apoyo, cm debajo de la cadera")
    ap.add_argument("--adelanto-ms", type=float, default=ADELANTO * 1000, help="adelanto del modo compensado")
    ap.add_argument("--voltaje", type=float, help="voltaje de la fuente de los servos (para el registro)")
    ap.add_argument("--revisar", action="store_true", help="ángulos y pie en vivo, servos sueltos")
    ap.add_argument("--simulado", action="store_true", help="sin hardware")
    ap.add_argument("--semilla", type=int, default=5)
    ap.add_argument("--reanalizar", metavar="RUTA", help="carpeta de una corrida o 'ultima'")
    ap.add_argument("--salida", default=os.path.join(HERE, "resultados_marcha"))
    a = ap.parse_args()

    try:
        esc = Escala()
        retro = Retro(esc)
        pata = Pata()
    except (FileNotFoundError, ValueError, KeyError) as e:
        sys.exit(f"Error: {e}")
    if any(not 0.3 <= T <= 5 for T in a.periodos):
        ap.error("periodos entre 0.3 y 5 s")
    if not 1 <= a.pasos <= 10 or not 0 <= a.adelanto_ms <= 300:
        ap.error("--pasos de 1 a 10 y --adelanto-ms de 0 a 300")
    for T in a.periodos:
        dur = ESPERA + a.pasos * T + ESPERA
        if dur / DT > 1500 or dur > 15.5:
            ap.error(f"a {T:g} s por paso la corrida dura {dur:.1f} s; la ESP32 graba hasta ~15.5 s con 3 servos: "
                     "baja --pasos")

    if a.reanalizar:
        ruta = a.reanalizar
        if ruta == "ultima":
            cands = sorted(d for d in (os.listdir(a.salida) if os.path.isdir(a.salida) else [])
                           if os.path.isfile(os.path.join(a.salida, d, "corridas.csv")))
            if not cands:
                sys.exit(f"No hay corridas en {a.salida}")
            ruta = os.path.join(a.salida, cands[-1])
        print(f"Reanalizando {ruta}")
        analizar(ruta, pata, esc, retro)
        return
    if a.simulado and a.revisar:
        ap.error("--revisar necesita el hardware (--port)")
    if not a.simulado and not a.port:
        ap.error("falta --port (o usa --simulado o --reanalizar)")
    try:
        if a.revisar:
            revisar(a, pata, retro)
            return
        carpeta = medir(a, pata, esc, retro)
    except ErrorESP32 as e:
        sys.exit(f"Error: {e}")
    except KeyboardInterrupt:
        sys.exit("\nCancelado antes de empezar; no se movió nada.")
    analizar(carpeta, pata, esc, retro)


if __name__ == "__main__":
    main()
