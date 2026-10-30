#!/usr/bin/env python3
"""
Eficiencia de los 12 servos FS5115M-FB con retroalimentación — perro OK1
=========================================================================
Mueve cada servo a ángulos fijos, de 30 en 30°, de 20 en 20°, de 10 en 10°, de 5 en 5° y de
2 en 2°, subiendo y luego bajando. En cada punto lee el voltaje de su cable verde y al final
calcula qué tan bien sigue el ángulo pedido: error, histéresis, ruido y rango útil de cada
servo. Hace las gráficas de cada servo y las de los 12 juntos.

Hardware: ESP32 principal con esp32_principal/esp32_principal.ino (PCA9685 y verdes de FL) y
ESP32 auxiliar con esp32_aux/esp32_aux.ino (verdes de FR, RL y RR). La compu solo se conecta a
la principal. También funciona con perro_servos.ino + retro_aux.ino (mismos comandos P, R, D, ?).

Uso (en la carpeta de este archivo):
  python eficiencia_servos.py --port COM6 --revisar           # los 12 voltajes en vivo, para
                                                               # revisar el cableado
  python eficiencia_servos.py --port COM6 --voltaje 5.95      # los 12 servos, una pata a la vez
  python eficiencia_servos.py --port COM6 --canal 3 4 5       # solo la pata FR
  python eficiencia_servos.py --reanalizar ultima             # rehace tablas y gráficas sin hardware
  python eficiencia_servos.py --simulado                      # practicar sin hardware

Ángulo = convención del proyecto: 500 µs = 0°, 1500 µs = 90°, 2500 µs = 180° (11.1 µs por
grado). Los 12 FS5115M-FB llegan a su tope en ~168° (2367 µs, prueba del 29 oct), así que el
firmware recorta en 2360 µs (167.4°) y por defecto se mide de 0° a 166°.

CUIDADO: por defecto cada servo recorre 0–166°. Hazlo con los servos sueltos (solo con su horn)
o, si ya están en las patas, limita cada articulación con --rango CANAL:MIN:MAX para que no
choque (por ejemplo --rango 2:40:150).

Cómo se calcula (cada servo):
  - Recta: ángulo = a·mV + b, ajustada con los puntos de todas las pasadas dentro del rango útil.
  - Rango útil: desde el centro hacia los extremos, mientras el punto quede a menos de 2° de la
    recta. Donde el servo llega a su tope (o el firmware recorta el pulso) deja de seguirla.
  - Error: ángulo medido (con la recta o con la tabla) menos ángulo pedido.
  - Tabla de búsqueda: mV -> ángulo de la pasada más fina (2°); se prueba con las otras pasadas.
  - Histéresis: diferencia entre subir y bajar al mismo ángulo, en grados.
  - Ruido: desviación estándar de las lecturas con el servo quieto, en grados.

Resultados en resultados_eficiencia/AAAAMMDD_HHMMSS_<canales>/:
  plan.csv, meta.json        ángulos y pulsos que se mandan; parámetros de la corrida
  mediciones.csv             cada lectura (se guarda mientras mide: no se pierde si se corta)
  eficiencia_12_servos.png   resumen de los servos medidos: error, histéresis, ruido, rango
  error_12_servos.png        error contra ángulo de cada servo, en su lugar del perro
  resumen.md, resumen.csv    tabla de todos los servos
  calibracion_servos.json    recta y tabla de búsqueda de cada servo
  canal<c>/                  por servo: retroalimentacion.png, tabla.csv, tabla.md,
                             tabla_busqueda.csv, calibracion.json
"""
import argparse
import csv
import datetime
import json
import math
import os
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
US_MIN, US_ESCALA = 500, 2500            # 0° y 180° en la convención del proyecto
US_LIMITE = 2360                         # el firmware no deja pasar de aquí (tope medido: ~2367 µs)
LIMITE_TOPE = 2.0                        # ° fuera de la recta para decir que ya no la sigue
PASOS = [30, 20, 10, 5, 2]
PATAS = ["FL", "FR", "RL", "RR"]
ARTICULACIONES = ["abducción", "cadera", "rodilla"]
FLECHA = {"sube": "↑", "baja": "↓"}

# Paleta validada (azul, naranja, aqua, amarillo, magenta) y tintas
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
COLORES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
MARCAS = ["o", "s", "D", "^", "v"]       # la forma también distingue la pasada
BANDA = "#eeede8"                        # zona fuera del rango útil

COLUMNAS = ["grupo", "canal", "nombre", "pasada", "paso", "sentido", "angulo", "pulso_us",
            "pulso_aplicado_us", "media_mv", "sigma_mv", "min_mv", "max_mv", "n", "t_s"]


try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# ======================================================================= utilidades

def ang_a_us(ang):
    return int(round(US_MIN + ang * (US_ESCALA - US_MIN) / 180.0))


def us_a_ang(us):
    return (us - US_MIN) * 180.0 / (US_ESCALA - US_MIN)


def nombre_canal(ch):
    if 0 <= ch < 12:
        return f"{PATAS[ch // 3]} {ARTICULACIONES[ch % 3]}"
    return f"canal {ch}"


def nombre_corto(ch):
    if 0 <= ch < 12:
        return f"{PATAS[ch // 3]} {['abd', 'cad', 'rod'][ch % 3]}"
    return f"c{ch}"


def angulos_de(paso, lo, hi):
    """De lo a hi cada `paso`; siempre incluye los dos extremos."""
    n = int(math.floor((hi - lo) / paso + 1e-9))
    xs = [round(lo + i * paso, 3) for i in range(n + 1)]
    if hi - xs[-1] > 1e-6:
        xs.append(round(hi, 3))
    return xs


def armar_plan(pasos, lo, hi, solo_subida):
    plan = []
    for i, paso in enumerate(pasos):
        angs = angulos_de(paso, lo, hi)
        plan += [{"pasada": i + 1, "paso": paso, "sentido": "sube", "angulo": x} for x in angs]
        if not solo_subida:
            plan += [{"pasada": i + 1, "paso": paso, "sentido": "baja", "angulo": x} for x in angs[::-1]]
    return plan


def armar_grupos(canales, modo):
    if modo == "uno":
        return [[c] for c in canales]
    if modo == "todos":
        return [list(canales)]
    grupos = {}
    for c in canales:
        grupos.setdefault(c // 3, []).append(c)
    return [grupos[k] for k in sorted(grupos)]


def fmt(x, d=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    return f"{x:.{d}f}"


def flotante(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


# ======================================================================= ESP32 real

class ErrorESP32(RuntimeError):
    pass


class ESP32:
    """Habla con la ESP32 principal: P, R, D y ?."""

    def __init__(self, port, baud=115200):
        import serial
        try:
            self.s = serial.Serial(port, baud, timeout=0.02)
        except serial.SerialException as e:
            raise ErrorESP32(f"no se pudo abrir {port}: {e}\n  ¿Está abierto el Monitor Serie de "
                             "Arduino u otro programa con ese puerto? Ciérralo.")
        time.sleep(2.0)                    # la ESP32 se reinicia al abrir el puerto
        self.s.reset_input_buffer()
        self.buf = b""
        self.ruido = []
        self.info = self.estado()

    def _linea(self, limite):
        while True:
            if b"\n" in self.buf:
                l, self.buf = self.buf.split(b"\n", 1)
                return l.decode("utf-8", "replace").strip()
            if time.time() > limite:
                return None
            self.buf += self.s.read(max(1, self.s.in_waiting))

    def comando(self, txt, prefijo, timeout=1.0):
        self.s.write((txt + "\n").encode())
        limite = time.time() + timeout
        while True:
            l = self._linea(limite)
            if l is None:
                raise ErrorESP32(f"la ESP32 no contestó a '{txt}'")
            if l.startswith(prefijo):
                return l
            if l.startswith("E "):
                raise ErrorESP32(f"la ESP32 contestó a '{txt}': {l}")
            if l:
                self.ruido.append(l)

    def estado(self):
        for _ in range(3):
            try:
                l = self.comando("?", "OK1", 1.5)
                break
            except ErrorESP32:
                l = None
        if l is None:
            raise ErrorESP32("la ESP32 no contesta. Revisa el puerto, aprieta EN en la ESP32 y vuelve "
                             "a intentar.")
        info = {"linea": l}
        for k in ("PCA", "FB", "AUX", "DESC"):
            m = re.search(k + r"=(\w+)", l)
            if m:
                info[k.lower()] = int(m.group(1)) if m.group(1).isdigit() else m.group(1)
        return info

    def mover(self, ch, us):
        l = self.comando(f"P {ch} {us}", f"OK P {ch} ", 1.0)
        try:
            return int(l.split()[3])
        except (IndexError, ValueError):
            return us

    def leer(self, ch, n):
        l = self.comando(f"R {ch} {n}", f"R {ch} ", 1.0 + n * 0.003)
        p = l.split()
        return {"media_mv": float(p[3]), "sigma_mv": float(p[4]), "min_mv": float(p[5]),
                "max_mv": float(p[6]), "n": int(p[2])}

    def soltar(self):
        try:
            self.comando("D", "OK D", 1.0)
        except ErrorESP32:
            pass

    def esperar(self, s):
        time.sleep(s)

    def cerrar(self):
        try:
            self.s.close()
        except Exception:
            pass


# ======================================================================= servos simulados

class ServoSimulado:
    """FS5115M-FB con divisor 100k/150k: voltaje que baja con el ángulo, curvatura, banda muerta,
    ruido y tope."""

    def __init__(self, ch, rng):
        self.rng = rng
        self.ratio = 150.0 / 250.0
        self.v0 = 3110 + rng.normal(0, 30)                 # mV del cable a 0° (medido: 3.07–3.15 V)
        self.pend = -16.7 * (1 + rng.normal(0, 0.01))      # mV del cable por grado (medido: ~10 mV/° en el pin)
        self.curva = rng.normal(0, 0.0006)                 # mV/°² (no es una recta perfecta)
        self.banda = 0.25 + 0.2 * rng.random()             # ° de banda muerta
        self.ruido = 1.2 + 0.6 * rng.random()              # mV en el pin
        self.tope_hi = {6: 170.0, 8: 170.0, 10: 170.0}.get(ch, 168.3)   # topes medidos el 29 oct
        self.tope_lo = 0.0
        self.pos = 90.0

    def mover(self, ang):
        obj = min(max(ang, self.tope_lo), self.tope_hi)
        d = obj - self.pos
        if abs(d) > self.banda:
            self.pos = obj - math.copysign(self.banda * (0.4 + 0.6 * self.rng.random()), d)

    def mv(self):
        x = self.pos
        cable = self.v0 + self.pend * x + self.curva * (x - 90) ** 2
        return self.ratio * cable


class ESP32Simulado:
    def __init__(self, semilla=7):
        self.rng = np.random.default_rng(semilla)
        self.servos = {ch: ServoSimulado(ch, self.rng) for ch in range(12)}
        self.info = {"linea": "OK1-RETRO listo PCA=ok FB=12 AUX=ok (SIMULADO)", "pca": "ok",
                     "fb": 12, "aux": "ok"}
        self.ruido = []

    def mover(self, ch, us):
        us = int(min(max(us, US_MIN), US_LIMITE))
        if ch in self.servos:
            self.servos[ch].mover(us_a_ang(us))
        return us

    def leer(self, ch, n):
        if ch not in self.servos:
            raise ErrorESP32(f"E R: canal sin retroalimentacion ({ch})")
        s = self.servos[ch]
        x = s.mv() + self.rng.normal(0, s.ruido, n)
        x = np.round(x)
        return {"media_mv": float(np.mean(x)), "sigma_mv": float(np.std(x)), "min_mv": float(x.min()),
                "max_mv": float(x.max()), "n": n}

    def soltar(self):
        pass

    def esperar(self, s):
        pass

    def cerrar(self):
        pass


# ======================================================================= medición

def imprimir_plan(a, plan, grupos, rangos):
    print("\n" + "=" * 78)
    print("TABLA INICIAL — ángulos que se van a mandar")
    sentido = "subiendo" if a.solo_subida else "subiendo y luego bajando"
    print(f"De {a.min:g}° a {a.max:g}°, {sentido}")
    print("=" * 78)
    for i, paso in enumerate(a.pasos):
        angs = angulos_de(paso, a.min, a.max)
        print(f"\n  De {paso:g} en {paso:g}°  ({len(angs)} ángulos)")
        for j in range(0, len(angs), 14):
            trozo = angs[j:j + 14]
            print("    ángulo (°) " + "".join(f"{x:>6g}" for x in trozo))
            print("    pulso (µs) " + "".join(f"{min(ang_a_us(x), US_LIMITE):>6d}" for x in trozo))
    print(f"\n  {len(plan)} puntos por servo.")
    print("\n  Orden:")
    for g in grupos:
        print("    " + ", ".join(f"{c} ({nombre_canal(c)})" for c in g) + "  — juntos")
    esp = [c for c in rangos if rangos[c] != (a.min, a.max)]
    for c in esp:
        print(f"    canal {c}: solo de {rangos[c][0]:g}° a {rangos[c][1]:g}° (--rango)")
    t_punto = a.espera + 0.05 + len(grupos[0]) * (a.muestras * 0.0006 + 0.02)
    print(f"\n  Tiempo estimado: ~{len(plan) * len(grupos) * t_punto / 60:.0f} min "
          f"(~{len(plan) * t_punto / 60:.1f} min por grupo).")


def revisar_lecturas(esp, canales, n=10):
    """Una lectura de cada canal antes de empezar. Regresa (lecturas, problemas)."""
    lecturas, problemas = {}, []
    for ch in canales:
        try:
            r = esp.leer(ch, n)
            lecturas[ch] = r["media_mv"]
            if r["media_mv"] < 60:
                problemas.append(f"canal {ch} ({nombre_canal(ch)}): {r['media_mv']:.0f} mV — casi 0: "
                                 "cable verde suelto, divisor sin conectar o servo sin V+.")
            elif r["media_mv"] > 2500:
                problemas.append(f"canal {ch} ({nombre_canal(ch)}): {r['media_mv']:.0f} mV — más de lo "
                                 "que el ESP32 lee bien: ¿falta el divisor?")
        except ErrorESP32 as e:
            problemas.append(f"canal {ch} ({nombre_canal(ch)}): {e}")
    return lecturas, problemas


def mover_suave(esp, chs, desde, hasta, vel=90.0, dt=0.03):
    """Lleva varios servos de `desde` a `hasta` (grados) a ~vel °/s."""
    salto = max(abs(hasta[c] - desde[c]) for c in chs)
    pasos = max(1, int(math.ceil(salto / (vel * dt))))
    for k in range(1, pasos + 1):
        f = k / pasos
        for c in chs:
            esp.mover(c, ang_a_us(desde[c] + (hasta[c] - desde[c]) * f))
        esp.esperar(dt)


def medir(a):
    canales = a.canal
    rangos = {c: a.rangos.get(c, (a.min, a.max)) for c in canales}
    grupos = armar_grupos(canales, a.grupo)
    plan = armar_plan(a.pasos, a.min, a.max, a.solo_subida)
    imprimir_plan(a, plan, grupos, rangos)

    esp = ESP32Simulado() if a.simulado else ESP32(a.port)
    print(f"\nESP32: {esp.info['linea']}")
    if esp.info.get("pca") == "NO":
        esp.cerrar()
        sys.exit("El ESP32 no encuentra el PCA9685 (PCA=NO). Revisa SDA 21, SCL 22, VCC 3V3 y GND.")
    if isinstance(esp.info.get("fb"), int) and esp.info["fb"] < 12 and any(c >= 3 for c in canales):
        print(f"AVISO: el firmware dice FB={esp.info['fb']}: la auxiliar no está mandando datos. "
              "Revisa TX/RX cruzados (16 <-> 17), GND común y que tenga esp32_aux.ino.")

    lect, problemas = revisar_lecturas(esp, canales)
    if lect:
        print("\nLectura inicial (mV del pin):")
        for ch in canales:
            if ch in lect:
                print(f"  {ch:>2}  {nombre_canal(ch):<16} {lect[ch]:7.0f}")
    if problemas:
        print("\nProblemas antes de empezar:")
        for p in problemas:
            print("  - " + p)
        if not a.forzar:
            esp.cerrar()
            sys.exit("\nArréglalos y vuelve a correr (o usa --forzar para medir de todos modos). "
                     "Para ver los 12 en vivo: --revisar")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    etiqueta = "12_servos" if sorted(canales) == list(range(12)) else "canales_" + "_".join(map(str, canales))
    carpeta = os.path.join(a.salida, f"{stamp}_{etiqueta}" + ("_sim" if a.simulado else ""))
    os.makedirs(carpeta, exist_ok=True)

    with open(os.path.join(carpeta, "plan.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["pasada", "paso", "sentido", "angulo", "pulso_us"])
        for p in plan:
            w.writerow([p["pasada"], p["paso"], p["sentido"], p["angulo"], ang_a_us(p["angulo"])])
    meta = {
        "fecha": stamp, "canales": canales, "pasos": a.pasos, "min": a.min, "max": a.max,
        "rangos": {str(c): list(r) for c, r in rangos.items()}, "grupo": a.grupo,
        "solo_subida": a.solo_subida, "espera_s": a.espera, "espera_inicial_s": a.espera_inicial,
        "muestras": a.muestras, "voltaje": a.voltaje, "divisor": a.divisor, "simulado": a.simulado,
        "esp32": esp.info, "lectura_inicial_mv": {str(c): v for c, v in lect.items()},
    }
    guardar_json(os.path.join(carpeta, "meta.json"), meta)

    f = open(os.path.join(carpeta, "mediciones.csv"), "w", newline="", encoding="utf-8")
    w = csv.DictWriter(f, fieldnames=COLUMNAS)
    w.writeheader()
    t0 = time.time()
    total = len(plan) * len(grupos)
    hechos = 0
    fallas = 0
    cancelado = False
    print("\nMidiendo. Ctrl+C para detener (se guarda lo que se alcanzó a medir).")
    try:
        for gi, grupo in enumerate(grupos):
            nombre_g = ", ".join(f"{c} {nombre_corto(c)}" for c in grupo)
            print(f"\n— Grupo {gi + 1}/{len(grupos)}: {nombre_g}")
            centro = {c: min(max(90.0, rangos[c][0]), rangos[c][1]) for c in grupo}
            for c in grupo:
                esp.mover(c, ang_a_us(centro[c]))
            esp.esperar(a.espera_inicial)
            actual = dict(centro)
            pasada_ant = None
            for k, p in enumerate(plan):
                objetivo = {c: min(max(p["angulo"], rangos[c][0]), rangos[c][1]) for c in grupo}
                salto = max(abs(objetivo[c] - actual[c]) for c in grupo)
                if salto > 12:
                    mover_suave(esp, grupo, actual, objetivo)
                aplicado = {c: esp.mover(c, ang_a_us(objetivo[c])) for c in grupo}
                esp.esperar(a.espera_inicial if (k == 0 or salto > 35) else a.espera)
                actual = objetivo
                if p["pasada"] != pasada_ant:
                    pasada_ant = p["pasada"]
                    print(f"\n  pasada {p['pasada']}/{len(a.pasos)}: de {p['paso']:g} en {p['paso']:g}°")
                vals = []
                for c in grupo:
                    if not rangos[c][0] - 1e-9 <= p["angulo"] <= rangos[c][1] + 1e-9:
                        continue
                    r = None
                    for intento in range(3):
                        try:
                            r = esp.leer(c, a.muestras)
                            break
                        except ErrorESP32 as e:
                            if intento == 2:
                                fallas += 1
                                print(f"\n  AVISO: canal {c} sin lectura en {p['angulo']:g}°: {e}")
                    if r is None:
                        continue
                    fila = {"grupo": gi + 1, "canal": c, "nombre": nombre_canal(c), "pasada": p["pasada"],
                            "paso": p["paso"], "sentido": p["sentido"], "angulo": p["angulo"],
                            "pulso_us": ang_a_us(p["angulo"]), "pulso_aplicado_us": aplicado[c],
                            "media_mv": round(r["media_mv"], 2), "sigma_mv": round(r["sigma_mv"], 3),
                            "min_mv": r["min_mv"], "max_mv": r["max_mv"], "n": r["n"],
                            "t_s": round(time.time() - t0, 3)}
                    w.writerow(fila)
                    vals.append(f"{c}: {r['media_mv']:6.0f}")
                f.flush()
                hechos += 1
                trans = time.time() - t0
                falta = trans / hechos * (total - hechos) / 60 if hechos else 0
                linea = (f"  {FLECHA[p['sentido']]} {p['angulo']:6.1f}°  {ang_a_us(p['angulo']):5d} µs  | "
                         + "  ".join(vals) + f" mV  | {hechos}/{total}, faltan ~{falta:.0f} min")
                print("\r" + linea.ljust(110), end="", flush=True)
            print()
            mover_suave(esp, grupo, actual, centro)
            esp.soltar()
    except KeyboardInterrupt:
        cancelado = True
        print("\n\nDetenido. Se analiza lo que se alcanzó a medir.")
    finally:
        esp.soltar()
        esp.cerrar()
        f.close()
    if fallas:
        print(f"AVISO: {fallas} lecturas fallaron (están fuera de mediciones.csv).")
    meta["cancelado"] = cancelado
    meta["duracion_min"] = round((time.time() - t0) / 60, 1)
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    return carpeta


# ======================================================================= análisis

def cargar_mediciones(ruta):
    filas = []
    with open(ruta, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                filas.append({
                    "grupo": int(r["grupo"]), "canal": int(r["canal"]), "pasada": int(r["pasada"]),
                    "paso": float(r["paso"]), "sentido": r["sentido"], "angulo": float(r["angulo"]),
                    "pulso_us": int(float(r["pulso_us"])),
                    "pulso_aplicado_us": int(float(r["pulso_aplicado_us"] or r["pulso_us"])),
                    "media_mv": flotante(r["media_mv"]), "sigma_mv": flotante(r["sigma_mv"]),
                    "min_mv": flotante(r["min_mv"]), "max_mv": flotante(r["max_mv"]),
                })
            except (KeyError, ValueError):
                continue
    return [r for r in filas if math.isfinite(r["media_mv"])]


def ajuste(mv, ang):
    k1, k0 = np.polyfit(mv, ang, 1)
    pred = k1 * mv + k0
    ss_res = float(np.sum((ang - pred) ** 2))
    ss_tot = float(np.sum((ang - np.mean(ang)) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(k1), float(k0), r2


def rango_util(angs_u, media_por_ang, validos):
    """Crece desde el centro mientras el punto quede a menos de LIMITE_TOPE de la recta.
    Se detiene en dos puntos seguidos fuera (o en el último), para no cortar por un punto raro."""
    lo, hi = angs_u[0], angs_u[-1]
    centro = [x for x in angs_u if lo + 0.25 * (hi - lo) <= x <= lo + 0.75 * (hi - lo) and x in validos]
    if len(centro) < 3:
        centro = [x for x in angs_u if x in validos]
    util = centro
    for _ in range(4):
        mv = np.array([media_por_ang[x] for x in util])
        k1, k0, _ = ajuste(mv, np.array(util))
        res = {x: k1 * media_por_ang[x] + k0 - x for x in angs_u}
        bien = {x: (x in validos and abs(res[x]) <= LIMITE_TOPE) for x in angs_u}
        mid = min(angs_u, key=lambda x: abs(x - (lo + hi) / 2))
        i0 = angs_u.index(mid)
        nuevo = [mid]
        for paso in (1, -1):
            i = i0 + paso
            while 0 <= i < len(angs_u):
                if bien[angs_u[i]]:
                    nuevo.append(angs_u[i])
                elif 0 <= i + paso < len(angs_u) and bien[angs_u[i + paso]]:
                    pass                                   # un punto raro aislado: se salta
                else:
                    break
                i += paso
        nuevo = sorted(nuevo)
        if nuevo == util:
            break
        util = nuevo
    return min(util), max(util), res


def analizar_canal(filas_todas, ch):
    filas = [r for r in filas_todas if r["canal"] == ch]
    out = {"canal": ch, "nombre": nombre_canal(ch), "avisos": [], "filas": filas}
    if len(filas) < 6:
        out["avisos"].append("Muy pocas mediciones para analizar.")
        return out
    recortadas = [r for r in filas if r["pulso_aplicado_us"] != r["pulso_us"]]
    if recortadas:
        lim = sorted({r["pulso_aplicado_us"] for r in recortadas})
        out["avisos"].append(f"El firmware recortó el pulso en {len(recortadas)} mediciones (se aplicó "
                             f"{' / '.join(map(str, lim))} µs): esos ángulos no se pidieron de verdad.")
    angs_u = sorted({r["angulo"] for r in filas})
    media_por_ang = {x: float(np.mean([r["media_mv"] for r in filas if r["angulo"] == x])) for x in angs_u}
    validos = {r["angulo"] for r in filas if r["pulso_aplicado_us"] == r["pulso_us"]}
    lo, hi, _ = rango_util(angs_u, media_por_ang, validos)
    dentro = [r for r in filas if lo <= r["angulo"] <= hi and r["pulso_aplicado_us"] == r["pulso_us"]]
    fuera = sorted({r["angulo"] for r in filas} - {r["angulo"] for r in dentro})

    mv = np.array([r["media_mv"] for r in dentro])
    ang = np.array([r["angulo"] for r in dentro])
    k1, k0, r2 = ajuste(mv, ang)
    for r in filas:
        r["ang_recta"] = k1 * r["media_mv"] + k0
        r["err_recta"] = r["ang_recta"] - r["angulo"]
        r["util"] = lo <= r["angulo"] <= hi and r["pulso_aplicado_us"] == r["pulso_us"]
    err = np.array([r["err_recta"] for r in dentro])

    # por pasada
    por_pasada = []
    for p in sorted({r["pasada"] for r in dentro}):
        sub = [r for r in dentro if r["pasada"] == p]
        if len(sub) < 3:
            continue
        a1, a0, rr = ajuste(np.array([r["media_mv"] for r in sub]), np.array([r["angulo"] for r in sub]))
        malla = np.linspace(mv.min(), mv.max(), 50)
        sep = float(np.max(np.abs((a1 * malla + a0) - (k1 * malla + k0))))
        e = np.array([r["err_recta"] for r in sub])
        por_pasada.append({"pasada": p, "paso": sub[0]["paso"], "puntos": len(sub), "a": a1, "b": a0,
                           "r2": rr, "separacion_max": sep, "error_rms": float(np.sqrt(np.mean(e ** 2))),
                           "error_max": float(np.max(np.abs(e)))})

    # histéresis
    hist = []
    for p in sorted({r["pasada"] for r in dentro}):
        for x in sorted({r["angulo"] for r in dentro if r["pasada"] == p}):
            s = [r["media_mv"] for r in dentro if r["pasada"] == p and r["angulo"] == x and r["sentido"] == "sube"]
            b = [r["media_mv"] for r in dentro if r["pasada"] == p and r["angulo"] == x and r["sentido"] == "baja"]
            if s and b:
                hist.append({"pasada": p, "angulo": x, "grados": abs(np.mean(s) - np.mean(b)) * abs(k1)})
    hist_g = np.array([h["grados"] for h in hist]) if hist else np.array([])

    # ruido
    sig_mv = float(np.median([r["sigma_mv"] for r in dentro if math.isfinite(r["sigma_mv"])] or [float("nan")]))

    # tabla de búsqueda: pasada más fina
    tabla = None
    pasos_d = sorted({r["paso"] for r in dentro})
    if pasos_d:
        fino = pasos_d[0]
        sub = [r for r in dentro if r["paso"] == fino]
        xs = sorted({r["angulo"] for r in sub})
        mvs = [float(np.mean([r["media_mv"] for r in sub if r["angulo"] == x])) for x in xs]
        d = np.diff(mvs)
        monotona = len(xs) >= 3 and (np.all(d < 0) or np.all(d > 0))
        tabla = {"paso": fino, "angulo": xs, "mv": [round(v, 2) for v in mvs], "monotona": bool(monotona)}
        if monotona:
            orden = np.argsort(mvs)
            mv_s, ang_s = np.array(mvs)[orden], np.array(xs)[orden]
            prueba = [r for r in dentro if r["paso"] != fino] or sub
            for r in filas:
                r["ang_tabla"] = float(np.interp(r["media_mv"], mv_s, ang_s))
                r["err_tabla"] = r["ang_tabla"] - r["angulo"]
            et = np.array([r["err_tabla"] for r in prueba])
            tabla.update({"probada_con": "otras pasadas" if prueba is not sub else "la misma pasada",
                          "error_rms": float(np.sqrt(np.mean(et ** 2))), "error_max": float(np.max(np.abs(et))),
                          "dentro_1g": float(np.mean(np.abs(et) <= 1.0) * 100)})
        else:
            out["avisos"].append("La tabla de búsqueda no es monótona (dos ángulos dan el mismo voltaje): "
                                 "no la uses así.")

    if fuera:
        txt = ", ".join(f"{x:g}" for x in fuera[:8]) + (" …" if len(fuera) > 8 else "")
        out["avisos"].append(f"Fuera del rango útil (llegó a su tope o no siguió el comando): {txt}°.")

    out.update({
        "rango_util": [lo, hi], "rango_util_us": [ang_a_us(lo), min(ang_a_us(hi), US_LIMITE)],
        "recta": {"a": k1, "b": k0, "r2": r2, "mv_por_grado": 1 / abs(k1) if k1 else float("nan")},
        "error_rms": float(np.sqrt(np.mean(err ** 2))), "error_max": float(np.max(np.abs(err))),
        "dentro_1g": float(np.mean(np.abs(err) <= 1.0) * 100),
        "histeresis_max": float(hist_g.max()) if hist_g.size else float("nan"),
        "histeresis_media": float(hist_g.mean()) if hist_g.size else float("nan"),
        "ruido_mv": sig_mv, "ruido_grados": sig_mv * abs(k1),
        "mv_min": float(mv.min()), "mv_max": float(mv.max()),
        "por_pasada": por_pasada, "histeresis": hist, "tabla": tabla, "puntos": len(dentro),
    })
    return out


# ======================================================================= gráficas

def estilo(plt):
    plt.rcParams.update({
        "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.titlecolor": INK,
        "xtick.color": INK2, "ytick.color": INK2, "grid.color": GRID, "grid.linewidth": 0.7,
        "axes.grid": True, "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold", "legend.frameon": False,
    })


def puntos(ax, filas, campo_y, chs_pasadas, s=26):
    for i, p in enumerate(chs_pasadas):
        col, mk = COLORES[i % len(COLORES)], MARCAS[i % len(MARCAS)]
        for sentido in ("sube", "baja"):
            sub = [r for r in filas if r["pasada"] == p and r["sentido"] == sentido and r["util"]]
            if not sub:
                continue
            x = [r["angulo"] for r in sub]
            y = [r[campo_y] for r in sub]
            if sentido == "sube":
                ax.scatter(x, y, s=s, marker=mk, color=col, edgecolors=SURF, linewidths=0.6, zorder=3)
            else:
                ax.scatter(x, y, s=s, marker=mk, facecolors="none", edgecolors=col, linewidths=1.1, zorder=3)


def grafica_canal(res, carpeta, titulo_extra, plt):
    from matplotlib.lines import Line2D
    filas = res["filas"]
    lo, hi = res["rango_util"]
    k1, k0 = res["recta"]["a"], res["recta"]["b"]
    pasadas = sorted({r["pasada"] for r in filas})
    pasos = {r["pasada"]: r["paso"] for r in filas}

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 2], "hspace": 0.08})
    xmin = min(r["angulo"] for r in filas) - 4
    xmax = max(r["angulo"] for r in filas) + 4
    for ax in (ax1, ax2):
        ax.set_xlim(xmin, xmax)
        if lo > xmin + 4:
            ax.axvspan(xmin, lo - 0.5, color=BANDA, zorder=0, lw=0)
        if hi < xmax - 4:
            ax.axvspan(hi + 0.5, xmax, color=BANDA, zorder=0, lw=0)

    # arriba: mV contra ángulo
    puntos(ax1, filas, "media_mv", pasadas)
    malos = [r for r in filas if not r["util"]]
    if malos:
        ax1.scatter([r["angulo"] for r in malos], [r["media_mv"] for r in malos], marker="x", s=28,
                    color=MUTED, linewidths=1.2, zorder=3)
    xs = np.linspace(lo, hi, 50)
    ax1.plot(xs, (xs - k0) / k1, color=INK, lw=1.2, zorder=2)
    ax1.set_ylabel("Cable verde en el pin (mV)")
    sec = ax1.secondary_xaxis("top", functions=(lambda d: US_MIN + d * (US_ESCALA - US_MIN) / 180.0,
                                                 lambda u: (u - US_MIN) * 180.0 / (US_ESCALA - US_MIN)))
    sec.set_xlabel("Pulso (µs)", color=INK2)
    sec.tick_params(colors=INK2)

    # abajo: error con la recta
    ax2.axhspan(-1, 1, color="#e9f1fb", zorder=0, lw=0)
    ax2.axhline(0, color=AXIS, lw=1, zorder=1)
    puntos(ax2, filas, "err_recta", pasadas)
    tiene_tabla = res.get("tabla") and res["tabla"].get("monotona")
    if tiene_tabla:
        prom = {}
        for r in filas:
            if r["util"] and "err_tabla" in r:
                prom.setdefault(r["angulo"], []).append(r["err_tabla"])
        xa = sorted(prom)
        ax2.plot(xa, [np.mean(prom[x]) for x in xa], color=INK2, lw=1.0, zorder=2)
    ax2.set_ylabel("Error (°)\nmedido − pedido")
    ax2.set_xlabel("Ángulo pedido (°)")
    lim = max(1.5, min(6, 1.2 * max(abs(r["err_recta"]) for r in filas if r["util"])))
    ax2.set_ylim(-lim, lim)

    h = [Line2D([], [], marker=MARCAS[i % 5], color=COLORES[i % 5], ls="", ms=7,
                label=f"de {pasos[p]:g} en {pasos[p]:g}°") for i, p in enumerate(pasadas)]
    h += [Line2D([], [], marker="o", color=INK2, ls="", ms=7, label="subiendo"),
          Line2D([], [], marker="o", mfc="none", mec=INK2, ls="", ms=7, label="bajando"),
          Line2D([], [], color=INK, lw=1.2, label="recta")]
    if tiene_tabla:
        h.append(Line2D([], [], color=INK2, lw=1.0, label="error con la tabla (promedio)"))
    if malos:
        h.append(Line2D([], [], marker="x", color=MUTED, ls="", ms=7, label="fuera del rango útil"))
    ax1.legend(handles=h, loc="upper right", fontsize=8.5, ncol=2)

    t = res.get("tabla") or {}
    texto = (f"Rango útil {lo:g}–{hi:g}°  ·  {res['recta']['mv_por_grado']:.1f} mV/°  ·  "
             f"R² {res['recta']['r2']:.5f}\n"
             f"Error con la recta: RMS {res['error_rms']:.2f}°, máx {res['error_max']:.2f}°  ·  "
             f"con la tabla: RMS {fmt(t.get('error_rms'))}°, máx {fmt(t.get('error_max'))}°\n"
             f"Histéresis máx {fmt(res['histeresis_max'])}°  ·  ruido σ {res['ruido_mv']:.1f} mV = "
             f"{res['ruido_grados']:.2f}°")
    fig.suptitle(f"Canal {res['canal']} · {res['nombre']}{titulo_extra}", x=0.06, ha="left",
                 fontsize=13, fontweight="bold", color=INK, y=0.995)
    fig.text(0.06, 0.955, texto, fontsize=9, color=INK2, va="top")
    fig.subplots_adjust(top=0.84, left=0.1, right=0.97, bottom=0.08)
    fig.savefig(os.path.join(carpeta, "retroalimentacion.png"), dpi=150)
    plt.close(fig)


def posiciones(resultados):
    """Canales en orden y posiciones en x con un hueco entre patas."""
    chs = sorted(resultados)
    xs, x, pata_ant = [], 0.0, None
    for c in chs:
        if pata_ant is not None and c // 3 != pata_ant:
            x += 0.8
        xs.append(x)
        x += 1
        pata_ant = c // 3
    return chs, np.array(xs)


def grafica_resumen(resultados, carpeta, titulo_extra, plt):
    chs, xs = posiciones(resultados)
    R = [resultados[c] for c in chs]
    tabla = lambda r, k: (r.get("tabla") or {}).get(k, float("nan"))
    paneles = [
        ("Error RMS (°)", [r["error_rms"] for r in R], [tabla(r, "error_rms") for r in R], "menos es mejor"),
        ("Error máximo (°)", [r["error_max"] for r in R], [tabla(r, "error_max") for r in R], "menos es mejor"),
        ("Puntos con error ≤ 1° (%)", [r["dentro_1g"] for r in R], [tabla(r, "dentro_1g") for r in R],
         "más es mejor"),
        ("Histéresis máxima (°)", [r["histeresis_max"] for r in R], None, "subir contra bajar; menos es mejor"),
        ("Ruido σ (°)", [r["ruido_grados"] for r in R], None, "servo quieto; menos es mejor"),
        ("Rango útil (°)", [r["rango_util"][1] - r["rango_util"][0] for r in R], None, "más es mejor"),
    ]
    fig, axs = plt.subplots(3, 2, figsize=(12, 10.5))
    w = 0.38
    for ax, (titulo, v1, v2, nota) in zip(axs.flat, paneles):
        if v2 is None:
            ax.bar(xs, v1, width=0.62, color=COLORES[0], zorder=2)
        else:
            ax.bar(xs - w / 2 - 0.01, v1, width=w, color=COLORES[0], zorder=2, label="con la recta")
            ax.bar(xs + w / 2 + 0.01, v2, width=w, color=COLORES[1], zorder=2, label="con la tabla")
        ax.set_title(titulo, loc="left")
        ax.text(1.0, 1.02, nota, transform=ax.transAxes, ha="right", va="bottom", fontsize=8.5, color=MUTED)
        ax.set_xticks(xs)
        ax.set_xticklabels([f"{c}\n{['abd', 'cad', 'rod'][c % 3]}" for c in chs], fontsize=8.5)
        ax.grid(axis="x", visible=False)
        ax.tick_params(axis="x", length=0)
        if "%" in titulo:
            ax.set_ylim(0, 105)
        for p in sorted({c // 3 for c in chs}):
            idx = [i for i, c in enumerate(chs) if c // 3 == p]
            ax.annotate(PATAS[p], xy=(np.mean(xs[idx]), 0), xycoords=("data", "axes fraction"),
                        xytext=(0, -30), textcoords="offset points", ha="center", va="top",
                        fontsize=9.5, fontweight="bold", color=INK)
    h, l = axs.flat[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper right", ncol=2, fontsize=9.5, bbox_to_anchor=(0.98, 0.985))
    fig.suptitle("Eficiencia de los servos: qué tan bien siguen el ángulo pedido" + titulo_extra,
                 x=0.04, ha="left", fontsize=14, fontweight="bold", color=INK, y=0.985)
    fig.text(0.04, 0.945, "Error = ángulo medido con la retroalimentación − ángulo pedido, en el rango útil "
             "de cada servo. Tabla probada con las pasadas que no la formaron.", fontsize=9, color=INK2)
    fig.subplots_adjust(top=0.88, hspace=0.62, wspace=0.18, left=0.06, right=0.98, bottom=0.07)
    fig.savefig(os.path.join(carpeta, "eficiencia_12_servos.png"), dpi=150)
    plt.close(fig)


def grafica_errores(resultados, carpeta, titulo_extra, plt):
    from matplotlib.lines import Line2D
    patas = sorted({c // 3 for c in resultados})
    fig, axs = plt.subplots(len(patas), 3, figsize=(13, 2.9 * len(patas) + 1.3), sharex=True, sharey=True,
                            squeeze=False)
    todos = [r["err_recta"] for res in resultados.values() for r in res["filas"] if r.get("util")]
    lim = max(1.5, min(6, 1.15 * max(abs(x) for x in todos))) if todos else 3
    pasos = {}
    for i, p in enumerate(patas):
        for j in range(3):
            ax = axs[i][j]
            ch = p * 3 + j
            ax.axhspan(-1, 1, color="#e9f1fb", zorder=0, lw=0)
            ax.axhline(0, color=AXIS, lw=1, zorder=1)
            ax.set_ylim(-lim, lim)
            if j == 0:
                ax.set_ylabel(f"{PATAS[p]}\nerror (°)")
            if i == len(patas) - 1:
                ax.set_xlabel("Ángulo pedido (°)")
            if ch not in resultados:
                ax.text(0.5, 0.5, "sin medir", transform=ax.transAxes, ha="center", va="center", color=MUTED)
                ax.set_title(f"{ch} · {nombre_canal(ch)}", loc="left", color=MUTED)
                continue
            res = resultados[ch]
            pas = sorted({r["pasada"] for r in res["filas"]})
            pasos.update({r["pasada"]: r["paso"] for r in res["filas"]})
            puntos(ax, res["filas"], "err_recta", pas, s=14)
            ax.set_title(f"{ch} · {nombre_canal(ch)}", loc="left")
            ax.text(1.0, 1.03, f"RMS {res['error_rms']:.2f}° · útil {res['rango_util'][0]:g}–"
                    f"{res['rango_util'][1]:g}°", transform=ax.transAxes, ha="right", va="bottom",
                    fontsize=8.5, color=INK2)
    h = [Line2D([], [], marker=MARCAS[k % 5], color=COLORES[k % 5], ls="", ms=6,
                label=f"de {pasos[p]:g} en {pasos[p]:g}°") for k, p in enumerate(sorted(pasos))]
    h += [Line2D([], [], marker="o", color=INK2, ls="", ms=6, label="subiendo"),
          Line2D([], [], marker="o", mfc="none", mec=INK2, ls="", ms=6, label="bajando")]
    fig.legend(handles=h, loc="upper left", ncol=len(h), fontsize=9, bbox_to_anchor=(0.05, 0.955 if len(patas) > 2 else 0.93))
    fig.suptitle("Error de cada servo contra el ángulo pedido (con la recta)" + titulo_extra, x=0.05,
                 ha="left", fontsize=14, fontweight="bold", color=INK, y=0.99)
    fig.subplots_adjust(top=0.88 if len(patas) > 2 else 0.8, hspace=0.45, wspace=0.08, left=0.07,
                        right=0.98, bottom=0.07 if len(patas) > 2 else 0.14)
    fig.savefig(os.path.join(carpeta, "error_12_servos.png"), dpi=150)
    plt.close(fig)


# ======================================================================= salidas

def guardar_json(ruta, obj):
    def limpio(x):
        if isinstance(x, dict):
            return {k: limpio(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [limpio(v) for v in x]
        if isinstance(x, (np.floating, float)):
            return None if not math.isfinite(float(x)) else round(float(x), 6)
        if isinstance(x, np.integer):
            return int(x)
        if isinstance(x, np.bool_):
            return bool(x)
        return x
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(limpio(obj), f, ensure_ascii=False, indent=2)


def guardar_canal(res, carpeta):
    os.makedirs(carpeta, exist_ok=True)
    filas = res["filas"]
    claves = sorted({(r["pasada"], r["angulo"]) for r in filas})
    with open(os.path.join(carpeta, "tabla.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["pasada", "paso", "angulo", "pulso_us", "mv_sube", "mv_baja", "mv_media", "sigma_mv",
                    "angulo_recta", "error_recta", "angulo_tabla", "error_tabla", "histeresis_g", "util"])
        for p, x in claves:
            sub = [r for r in filas if r["pasada"] == p and r["angulo"] == x]
            s = [r["media_mv"] for r in sub if r["sentido"] == "sube"]
            b = [r["media_mv"] for r in sub if r["sentido"] == "baja"]
            m = float(np.mean([r["media_mv"] for r in sub]))
            ar = float(np.mean([r["ang_recta"] for r in sub]))
            at = [r["ang_tabla"] for r in sub if "ang_tabla" in r]
            hg = abs(np.mean(s) - np.mean(b)) * abs(res["recta"]["a"]) if s and b else None
            w.writerow([p, sub[0]["paso"], x, ang_a_us(x), fmt(np.mean(s) if s else None, 1),
                        fmt(np.mean(b) if b else None, 1), f"{m:.1f}",
                        f"{np.median([r['sigma_mv'] for r in sub]):.2f}", f"{ar:.2f}", f"{ar - x:+.2f}",
                        fmt(np.mean(at) if at else None), fmt(np.mean(at) - x if at else None),
                        fmt(hg), "sí" if sub[0]["util"] else "no"])
    with open(os.path.join(carpeta, "tabla.md"), "w", encoding="utf-8") as f:
        f.write(f"# Canal {res['canal']} · {res['nombre']}\n\n")
        f.write(texto_metricas(res) + "\n\n")
        for p in sorted({r["pasada"] for r in filas}):
            paso = next(r["paso"] for r in filas if r["pasada"] == p)
            f.write(f"## Pasada {p}: de {paso:g} en {paso:g}°\n\n")
            f.write("| Ángulo (°) | Pulso (µs) | mV ↑ | mV ↓ | Medido (°) | Error (°) | Histéresis (°) | |\n")
            f.write("|---|---|---|---|---|---|---|---|\n")
            for x in sorted({r["angulo"] for r in filas if r["pasada"] == p}):
                sub = [r for r in filas if r["pasada"] == p and r["angulo"] == x]
                s = [r["media_mv"] for r in sub if r["sentido"] == "sube"]
                b = [r["media_mv"] for r in sub if r["sentido"] == "baja"]
                ar = float(np.mean([r["ang_recta"] for r in sub]))
                hg = abs(np.mean(s) - np.mean(b)) * abs(res["recta"]["a"]) if s and b else None
                f.write(f"| {x:g} | {ang_a_us(x)} | {fmt(np.mean(s) if s else None, 1)} | "
                        f"{fmt(np.mean(b) if b else None, 1)} | {ar:.2f} | {ar - x:+.2f} | {fmt(hg)} | "
                        f"{'' if sub[0]['util'] else 'fuera'} |\n")
            f.write("\n")
        if res["avisos"]:
            f.write("## Avisos\n\n" + "".join(f"- {a}\n" for a in res["avisos"]))
    t = res.get("tabla")
    if t and t.get("monotona"):
        with open(os.path.join(carpeta, "tabla_busqueda.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["mv", "angulo"])
            for m, x in sorted(zip(t["mv"], t["angulo"])):
                w.writerow([m, x])
    guardar_json(os.path.join(carpeta, "calibracion.json"), calibracion_de(res))


def calibracion_de(res):
    t = res.get("tabla") or {}
    return {
        "canal": res["canal"], "nombre": res["nombre"],
        "recta": res["recta"], "rango_util_grados": res["rango_util"], "rango_util_us": res["rango_util_us"],
        "mv_rango": [res["mv_min"], res["mv_max"]],
        "error_rms_recta": res["error_rms"], "error_max_recta": res["error_max"],
        "error_rms_tabla": t.get("error_rms"), "error_max_tabla": t.get("error_max"),
        "histeresis_max": res["histeresis_max"], "histeresis_media": res["histeresis_media"],
        "ruido_mv": res["ruido_mv"], "ruido_grados": res["ruido_grados"],
        "tabla_busqueda": ({"mv": t["mv"], "angulo": t["angulo"], "paso": t["paso"]}
                           if t.get("monotona") else None),
        "por_pasada": res["por_pasada"], "avisos": res["avisos"],
    }


def texto_metricas(res):
    t = res.get("tabla") or {}
    lo, hi = res["rango_util"]
    lineas = [
        f"- Rango útil: {lo:g}° a {hi:g}° ({res['rango_util_us'][0]}–{res['rango_util_us'][1]} µs), "
        f"{res['mv_min']:.0f}–{res['mv_max']:.0f} mV en el pin",
        f"- Recta: ángulo = {res['recta']['a']:.5f}·mV {res['recta']['b']:+.2f}  "
        f"({res['recta']['mv_por_grado']:.2f} mV por grado), R² = {res['recta']['r2']:.5f}",
        f"- Error con la recta: RMS {res['error_rms']:.2f}°, máx {res['error_max']:.2f}°, "
        f"{res['dentro_1g']:.0f} % de los puntos dentro de ±1°",
        f"- Error con la tabla (de {fmt(t.get('paso'), 0)} en {fmt(t.get('paso'), 0)}°, probada con "
        f"{t.get('probada_con', '—')}): RMS {fmt(t.get('error_rms'))}°, máx {fmt(t.get('error_max'))}°, "
        f"{fmt(t.get('dentro_1g'), 0)} % dentro de ±1°",
        f"- Histéresis: máx {fmt(res['histeresis_max'])}°, media {fmt(res['histeresis_media'])}°",
        f"- Ruido con el servo quieto: σ {res['ruido_mv']:.2f} mV = {res['ruido_grados']:.3f}°",
    ]
    if res["por_pasada"]:
        lineas.append("- Por pasada (recta de cada una contra la de todos los datos):")
        for p in res["por_pasada"]:
            lineas.append(f"  - de {p['paso']:g} en {p['paso']:g}°: {p['puntos']} puntos, R² {p['r2']:.5f}, "
                          f"se separa hasta {p['separacion_max']:.2f}°, error RMS {p['error_rms']:.2f}°")
    return "\n".join(lineas)


def guardar_resumen(resultados, carpeta, meta):
    chs = sorted(resultados)
    cab = ["canal", "nombre", "rango_util_min", "rango_util_max", "mv_por_grado", "r2", "error_rms_recta",
           "error_max_recta", "dentro_1g_recta", "error_rms_tabla", "error_max_tabla", "dentro_1g_tabla",
           "histeresis_max", "histeresis_media", "ruido_mv", "ruido_grados"]
    with open(os.path.join(carpeta, "resumen.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cab)
        for c in chs:
            r = resultados[c]
            t = r.get("tabla") or {}
            w.writerow([c, r["nombre"], r["rango_util"][0], r["rango_util"][1], f"{r['recta']['mv_por_grado']:.3f}",
                        f"{r['recta']['r2']:.6f}", f"{r['error_rms']:.3f}", f"{r['error_max']:.3f}",
                        f"{r['dentro_1g']:.1f}", fmt(t.get("error_rms"), 3), fmt(t.get("error_max"), 3),
                        fmt(t.get("dentro_1g"), 1), fmt(r["histeresis_max"], 3), fmt(r["histeresis_media"], 3),
                        f"{r['ruido_mv']:.2f}", f"{r['ruido_grados']:.3f}"])
    with open(os.path.join(carpeta, "resumen.md"), "w", encoding="utf-8") as f:
        sim = " (SIMULADO)" if meta.get("simulado") else ""
        f.write(f"# Eficiencia de los servos con retroalimentación{sim}\n\n")
        f.write(f"- Fecha: {meta.get('fecha', '')}\n")
        f.write(f"- Servos: FEETECH FS5115M-FB · divisor {meta.get('divisor', '—')} · fuente "
                f"{meta.get('voltaje') or '—'} V\n")
        pasos = meta.get("pasos", [])
        f.write(f"- Rango pedido: {meta.get('min', 0):g}° a {meta.get('max', 0):g}°; pasadas de "
                + ", ".join(f"{p:g}" for p in pasos) + "°; "
                + ("solo subiendo" if meta.get("solo_subida") else "subiendo y bajando") + "\n")
        f.write(f"- Por punto: {meta.get('espera_s', '—')} s de espera y {meta.get('muestras', '—')} lecturas "
                f"del ADC (0.5 ms entre ellas)\n")
        if meta.get("cancelado"):
            f.write("- **La corrida se detuvo antes de terminar**: el análisis es de lo que se alcanzó a medir.\n")
        f.write("\n## Todos los servos\n\n")
        f.write("| Canal | Servo | Rango útil (°) | mV/° | Error RMS recta (°) | Error RMS tabla (°) | "
                "Error máx tabla (°) | ≤ 1° tabla (%) | Histéresis máx (°) | Ruido σ (°) |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|\n")
        for c in chs:
            r = resultados[c]
            t = r.get("tabla") or {}
            f.write(f"| {c} | {r['nombre']} | {r['rango_util'][0]:g}–{r['rango_util'][1]:g} | "
                    f"{r['recta']['mv_por_grado']:.1f} | {r['error_rms']:.2f} | {fmt(t.get('error_rms'))} | "
                    f"{fmt(t.get('error_max'))} | {fmt(t.get('dentro_1g'), 0)} | {fmt(r['histeresis_max'])} | "
                    f"{r['ruido_grados']:.2f} |\n")
        if len(chs) > 1:
            g = lambda k: [resultados[c][k] for c in chs]
            gt = lambda k: [(resultados[c].get("tabla") or {}).get(k) for c in chs]
            gt = [x for x in gt("error_rms") if x is not None]
            peor = max(chs, key=lambda c: resultados[c]["error_rms"])
            f.write(f"\n- Error RMS con la recta: de {min(g('error_rms')):.2f}° a {max(g('error_rms')):.2f}° "
                    f"(el peor: canal {peor}, {nombre_canal(peor)}).\n")
            if gt:
                f.write(f"- Error RMS con la tabla: de {min(gt):.2f}° a {max(gt):.2f}°.\n")
            f.write(f"- Histéresis máxima: hasta {np.nanmax(g('histeresis_max')):.2f}°. "
                    f"Ruido: hasta {max(g('ruido_grados')):.2f}°.\n")
            angostos = [c for c in chs if resultados[c]["rango_util"][1] - resultados[c]["rango_util"][0]
                        < (meta.get("max", 166) - meta.get("min", 0)) - 1]
            if angostos:
                f.write("- Servos que no recorren todo lo pedido: " + ", ".join(
                    f"{c} ({resultados[c]['rango_util'][0]:g}–{resultados[c]['rango_util'][1]:g}°, "
                    f"{resultados[c]['rango_util_us'][0]}–{resultados[c]['rango_util_us'][1]} µs)"
                    for c in angostos) + ". Pon ese rango en US_MAX_CH del firmware.\n")
        f.write("\n## Cómo leerla\n\n"
                "- **Error**: ángulo medido con la retroalimentación menos el pedido. Con la recta incluye "
                "que el potenciómetro no es perfectamente lineal; la tabla de búsqueda lo corrige.\n"
                "- **Histéresis**: diferencia entre llegar subiendo y llegar bajando al mismo ángulo (banda "
                "muerta del servo).\n"
                "- **Ruido**: cuánto se mueve la lectura con el servo quieto.\n"
                "- **Rango útil**: dónde el servo sigue el comando; fuera de él llegó a su tope.\n"
                "- El ángulo pedido usa la convención 500 µs = 0°, 2500 µs = 180°. Que el giro real del eje "
                "coincida se comprueba aparte, con transportador.\n")
        for c in chs:
            r = resultados[c]
            f.write(f"\n## Canal {c} · {r['nombre']}\n\n{texto_metricas(r)}\n")
            if r["avisos"]:
                f.write("".join(f"- ⚠ {a}\n" for a in r["avisos"]))
    guardar_json(os.path.join(carpeta, "calibracion_servos.json"),
                 {"fecha": meta.get("fecha"), "divisor": meta.get("divisor"), "voltaje": meta.get("voltaje"),
                  "convencion": "500 us = 0 grados, 2500 us = 180 grados; mV del pin",
                  "canales": {str(c): calibracion_de(resultados[c]) for c in chs}})


def analizar(carpeta, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    estilo(plt)
    filas = cargar_mediciones(os.path.join(carpeta, "mediciones.csv"))
    if not filas:
        print("No hay mediciones para analizar.")
        return {}
    extra = " (SIMULADO)" if meta.get("simulado") else ""
    resultados = {}
    print("\n" + "=" * 78 + "\nRESULTADOS\n" + "=" * 78)
    for ch in sorted({r["canal"] for r in filas}):
        try:
            res = analizar_canal(filas, ch)
        except Exception as e:                       # un canal raro no detiene a los demás
            print(f"\nCanal {ch}: no se pudo analizar ({e}).")
            continue
        if "recta" not in res:
            print(f"\nCanal {ch} ({nombre_canal(ch)}): " + " ".join(res["avisos"]))
            continue
        resultados[ch] = res
        sub = os.path.join(carpeta, f"canal{ch}")
        guardar_canal(res, sub)
        grafica_canal(res, sub, extra, plt)
        t = res.get("tabla") or {}
        print(f"\nCanal {ch} · {res['nombre']}")
        print(f"  rango útil {res['rango_util'][0]:g}–{res['rango_util'][1]:g}°  ·  "
              f"{res['recta']['mv_por_grado']:.2f} mV/°  ·  R² {res['recta']['r2']:.5f}")
        print(f"  error: recta RMS {res['error_rms']:.2f}° (máx {res['error_max']:.2f}°)  ·  "
              f"tabla RMS {fmt(t.get('error_rms'))}° (máx {fmt(t.get('error_max'))}°)")
        print(f"  histéresis máx {fmt(res['histeresis_max'])}°  ·  ruido {res['ruido_grados']:.2f}°")
        for av in res["avisos"]:
            print(f"  AVISO: {av}")
    if resultados:
        guardar_resumen(resultados, carpeta, meta)
        grafica_resumen(resultados, carpeta, extra, plt)
        grafica_errores(resultados, carpeta, extra, plt)
        print(f"\nResultados en {carpeta}")
        print("  eficiencia_12_servos.png, error_12_servos.png, resumen.md y una carpeta por canal.")
    return resultados


def buscar_corrida(ruta, salida):
    if ruta == "ultima":
        if not os.path.isdir(salida):
            sys.exit(f"No hay corridas en {salida}")
        cands = sorted(d for d in os.listdir(salida)
                       if os.path.isfile(os.path.join(salida, d, "mediciones.csv")))
        if not cands:
            sys.exit(f"No hay corridas en {salida}")
        return os.path.join(salida, cands[-1])
    if os.path.isfile(ruta):
        return os.path.dirname(os.path.abspath(ruta))
    if os.path.isfile(os.path.join(ruta, "mediciones.csv")):
        return ruta
    sys.exit(f"No encuentro mediciones.csv en {ruta}")


# ======================================================================= revisar cableado

def revisar(a):
    esp = ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    esp.soltar()
    print("\nServos sueltos. Gira a mano un servo a la vez: solo debe cambiar su número.")
    print("Lo normal: 0° ≈ 1860 mV y 168° ≈ 190 mV (con el divisor 100k/150k). Ctrl+C para salir.\n")
    print("  " + "".join(f"{c:>2} {nombre_corto(c):<7}" for c in a.canal))
    try:
        while True:
            vals = []
            for c in a.canal:
                try:
                    vals.append(f"{esp.leer(c, 8)['media_mv']:>10.0f}")
                except ErrorESP32:
                    vals.append(f"{'—':>10}")
            print("\r  " + "".join(vals), end="", flush=True)
            time.sleep(0.15)
    except KeyboardInterrupt:
        print()
    finally:
        esp.cerrar()


# ======================================================================= principal

def main():
    ap = argparse.ArgumentParser(description="Eficiencia de los servos con retroalimentación (perro OK1).")
    ap.add_argument("--port", help="puerto de la ESP32 principal (COM6, /dev/ttyUSB0)")
    ap.add_argument("--canal", type=int, nargs="+", default=list(range(12)),
                    help="canales a medir (por defecto los 12: 0 1 2 = FL, 3 4 5 = FR, 6 7 8 = RL, 9 10 11 = RR)")
    ap.add_argument("--pasos", type=float, nargs="+", default=PASOS, help="pasadas en grados, en orden")
    ap.add_argument("--min", type=float, default=0.0, help="ángulo mínimo (°)")
    ap.add_argument("--max", type=float, default=166.0, help="ángulo máximo (°); 166 = 2344 µs")
    ap.add_argument("--rango", action="append", default=[], metavar="CANAL:MIN:MAX",
                    help="rango propio de un canal, por ejemplo 2:40:150 (se puede repetir)")
    ap.add_argument("--grupo", choices=["pata", "uno", "todos"], default="pata",
                    help="cuántos servos se mueven a la vez: los 3 de una pata (por defecto), uno o todos")
    ap.add_argument("--solo-subida", action="store_true", help="sin bajar (más rápido, sin histéresis)")
    ap.add_argument("--espera", type=float, default=0.8, help="s entre mover y leer")
    ap.add_argument("--espera-inicial", type=float, default=2.0, help="s en el primer punto y saltos grandes")
    ap.add_argument("--muestras", type=int, default=200, help="lecturas del ADC por punto (1–1000)")
    ap.add_argument("--voltaje", type=float, help="voltaje de la fuente de los servos (para el registro)")
    ap.add_argument("--divisor", default="100k/150k", help="resistencias del divisor (para el registro)")
    ap.add_argument("--forzar", action="store_true", help="medir aunque la revisión inicial encuentre problemas")
    ap.add_argument("--revisar", action="store_true", help="solo mostrar los voltajes en vivo (cableado)")
    ap.add_argument("--simulado", action="store_true", help="sin hardware, con servos simulados")
    ap.add_argument("--reanalizar", metavar="RUTA",
                    help="rehace tablas y gráficas sin hardware: la carpeta de una corrida, su mediciones.csv "
                         "o 'ultima'")
    ap.add_argument("--salida", default=os.path.join(HERE, "resultados_eficiencia"))
    a = ap.parse_args()

    a.canal = list(dict.fromkeys(a.canal))
    if any(not 0 <= c <= 11 for c in a.canal):
        ap.error("los canales con retroalimentación van de 0 a 11")
    if a.min < 0 or a.max > 180 or a.min >= a.max:
        ap.error("--min y --max entre 0 y 180, con --min menor que --max")
    if a.max > us_a_ang(US_LIMITE) + 0.01:
        print(f"AVISO: el firmware recorta en {US_LIMITE} µs ({us_a_ang(US_LIMITE):.1f}°): arriba de eso "
              "los ángulos no se piden de verdad y saldrán fuera del rango útil.")
    if any(p <= 0 for p in a.pasos):
        ap.error("los pasos deben ser positivos")
    a.muestras = max(1, min(1000, a.muestras))
    a.rangos = {}
    for txt in a.rango:
        try:
            c, lo, hi = txt.split(":")
            c, lo, hi = int(c), float(lo), float(hi)
            assert 0 <= c <= 11 and 0 <= lo < hi <= 180
            a.rangos[c] = (lo, hi)
        except (ValueError, AssertionError):
            ap.error(f"--rango {txt}: usa CANAL:MIN:MAX, por ejemplo 2:40:150")

    if a.reanalizar:
        carpeta = buscar_corrida(a.reanalizar, a.salida)
        meta = {}
        if os.path.isfile(os.path.join(carpeta, "meta.json")):
            with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
                meta = json.load(f)
        print(f"Reanalizando {carpeta}")
        analizar(carpeta, meta)
        return
    if a.simulado:
        a.espera = a.espera_inicial = 0.0
    elif not a.port:
        ap.error("falta --port (o usa --simulado, o --reanalizar)")
    if a.revisar:
        revisar(a)
        return
    if a.grupo == "todos" and len(a.canal) > 3 and not a.simulado:
        print("AVISO: con --grupo todos se mueven todos a la vez; cada FS5115M-FB da picos de más de 3 A. "
              "Revisa que tu fuente aguante.")

    try:
        carpeta = medir(a)
    except ErrorESP32 as e:
        sys.exit(f"Error: {e}")
    with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    analizar(carpeta, meta)


if __name__ == "__main__":
    main()
