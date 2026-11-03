#!/usr/bin/env python3
"""
Prueba 04 · Rampas: velocidad contra error — Arca9
===================================================
Cada servo hace rampas a velocidad constante y su cable verde se graba cada ~2 ms. Así se ve qué
tan bien sigue el ángulo pedido según la velocidad: cuánto se atrasa, si llega a la velocidad
pedida, cuánto se pasa al llegar y cuánto tarda en asentarse.

La rampa la hace la ESP32 principal (comando G, protocolo v2), no la compu: el pulso se
actualiza cada 20 ms y los tiempos de cada lectura son los de la placa. La compu solo pide cada
rampa, espera los datos, los guarda y pide la siguiente.

Plan por defecto:
  - Velocidades: 60, 100, 120, 160, 180, 200, 220 y 240 °/s (grados reales, prueba 02).
  - Recorrido: de 30° a 150° reales (819 a 2097 µs), lejos de los dos topes.
  - En cada velocidad, los 12 servos uno por uno (0 a 11). Cada uno: 0.5 s quieto, ida,
    0.5 s quieto, vuelta, 0.5 s quieto; eso 3 veces.
  - Al terminar cada servo la ESP32 manda sus datos (4–10 s); se guardan y sigue el siguiente.
  - Antes de cada rampa el servo espera 1 s quieto en el inicio.

Hardware: esp32_principal.ino con PROTO=2 (comando G) y esp32_aux.ino sin cambios.

Uso (en la carpeta de este archivo):
  python rampas.py --simulado                     # practicar sin hardware (unos segundos)
  python rampas.py --port COM6 --revisar          # los 12 en grados reales, servos sueltos
  python rampas.py --port COM6 --voltaje 5.99     # la prueba completa (~25 min)
  python rampas.py --port COM6 --canal 0 --velocidades 60 240    # una prueba corta
  python rampas.py --reanalizar ultima            # rehace tablas y gráficas sin hardware

CUIDADO: pensado para servos al aire, sin piezas. Con las patas armadas hay que cambiar
--desde y --hasta para que no choquen. Al terminar (o con Ctrl+C) se sueltan los 12 servos.

Qué se calcula en cada rampa (ida y vuelta por separado, y el promedio de las 3 vueltas):
  - Error en la rampa: ángulo pedido − ángulo medido en la segunda mitad de la rampa (el servo
    tarda ~0.1 s en arrancar; ahí ya va a velocidad constante), en el sentido del movimiento
    (positivo = el servo va atrás).
  - Atraso: ese error entre la velocidad pedida (ms).
  - Velocidad real: pendiente del ángulo medido en ese mismo tramo.
  - Error máximo: el mayor |pedido − medido| en toda la rampa y su espera.
  - Sobrepaso: cuánto pasa del valor final después de llegar.
  - Asentamiento: desde que el pedido llega al final hasta que el medido queda a ±1° del valor
    final (con media móvil de 10 ms, para que el ruido no cuente).
  - Error final: valor final − ángulo pedido.
El pedido es el pulso que la ESP32 aplicó (cada 20 ms), pasado a grados reales; el medido sale
del cable verde con la tabla de la prueba 01 y el factor de la prueba 02 (pc/escala.py).

Resultados en resultados_rampas/AAAAMMDD_HHMMSS/:
  meta.json, ciclos.csv      parámetros y estado de cada rampa
  muestras.csv               cada lectura: velocidad, canal, t, mV (se guarda servo por servo)
  comandos.csv               cada pulso aplicado: velocidad, canal, t, µs
  metricas.csv               las métricas de cada rampa, ida y vuelta
  resumen.csv, resumen.md    promedio de los 12 servos por velocidad, y tabla por servo
  rampas.png                 error, atraso, velocidad real y error máximo contra velocidad
  trazas/canalNN.png         pedido y medido de cada servo en cada velocidad
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
from escala import Escala, Retro, US_CERO, US_LIMITE, N_SERVOS

VELOCIDADES = [60, 100, 120, 160, 180, 200, 220, 240]
DESDE, HASTA = 30.0, 150.0             # grados reales
VUELTAS = 3
ESPERA_MS = 500
PAUSA_S = 1.0                          # quieto en el inicio antes de cada rampa
VEL_ACERCAR = 60.0                     # °/s para llevar un servo al inicio
BANDA = 1.0                            # ° para el asentamiento
TRAMO = (0.5, 1.0)                     # segunda mitad de cada rampa: el servo ya va a velocidad constante

PATAS = ["FL", "FR", "RL", "RR"]
ART = ["abducción", "cadera", "rodilla"]
CORTO = ["abd", "cad", "rod"]
SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
AZUL, NARANJA = "#2a78d6", "#eb6834"

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


def nombre(ch):
    return f"{PATAS[ch // 3]} {ART[ch % 3]}"


def corto(ch):
    return f"{PATAS[ch // 3]} {CORTO[ch % 3]}"


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


def duracion_ciclo(us_a, us_b, vel_us, vueltas, espera_ms):
    dur = abs(us_b - us_a) / vel_us
    esp = espera_ms / 1000.0
    return esp + vueltas * (2 * dur + 2 * esp)


# ======================================================================= simulador

class ESP32Simulado:
    """12 servos con un control proporcional (atraso ~ v/K), velocidad máxima y banda muerta.
    Hace el comando G igual que el firmware: pulso cada 20 ms, lectura cada 2 ms."""

    def __init__(self, escala, retro, semilla=4):
        self.rng = np.random.default_rng(semilla)
        self.esc, self.retro = escala, retro
        self.K = {ch: 28 + 6 * self.rng.random() for ch in range(N_SERVOS)}     # 1/s
        self.vmax = {ch: 290 + 30 * self.rng.random() for ch in range(N_SERVOS)}  # °/s reales
        self.ruido = {ch: 3.0 if ch < 3 else 6.0 for ch in range(N_SERVOS)}      # mV, como la prueba 01
        self.pos = {ch: float(20 + 120 * self.rng.random()) for ch in range(N_SERVOS)}
        self.us = {ch: 0 for ch in range(N_SERVOS)}
        self.info = {"linea": "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=2 (SIMULADO)", "pca": "ok",
                     "fb": 12, "aux": "ok", "proto": 2}

    def _mv(self, ch, ang=None):
        c = self.retro.canales[ch]
        conv = self.esc.real_a_conv(self.pos[ch] if ang is None else ang)
        return int(round((conv - c["b"]) / c["a"] + self.rng.normal(0, self.ruido[ch] * 0.56)))

    def mover(self, ch, us):
        us = int(min(max(us, US_CERO), US_LIMITE))
        self.us[ch] = us
        self.pos[ch] = self.esc.us_a_real(us)          # fuera de G: sin dinámica
        return us

    def leer_todos(self):
        return [self._mv(ch) for ch in range(N_SERVOS)]

    def leer(self, ch, n):
        x = [self._mv(ch) for _ in range(n)]
        return {"media_mv": float(np.mean(x)), "sigma_mv": float(np.std(x)), "min_mv": min(x),
                "max_mv": max(x), "n": n}

    def rampa(self, ch, us_a, us_b, vel, vueltas, espera_ms):
        if self.us[ch] != us_a:
            raise ErrorESP32(f"la ESP32 contestó: E G: primero manda P {ch} {us_a}")
        dur = abs(us_b - us_a) * 1e6 / vel
        esp = espera_ms * 1000.0
        total = esp + vueltas * (2 * dur + 2 * esp)

        def pulso(t):
            if t < esp:
                return us_a
            t -= esp
            per = 2 * dur + 2 * esp
            k = int(t // per)
            if k >= vueltas:
                return us_a
            r = t - k * per
            if r < dur:
                return us_a + (us_b - us_a) * r / dur
            r -= dur
            if r < esp:
                return us_b
            r -= esp
            if r < dur:
                return us_b + (us_a - us_b) * r / dur
            return us_a
        M, C = [], []
        dt = 250.0                                   # µs del integrador
        obj = self.esc.us_a_real(us_a)
        t = 0.0
        sig_c, sig_m = 0.0, 0.0
        K, vmax = self.K[ch], self.vmax[ch]
        while t < total:
            if t >= sig_c:
                us = int(round(min(max(pulso(t), US_CERO), US_LIMITE)))
                obj = self.esc.us_a_real(us)
                C.append((round(t / 1e6, 4), us))
                sig_c += 20000
            e = obj - self.pos[ch]
            if abs(e) > 0.25:                         # banda muerta
                v = max(-vmax, min(vmax, K * e))
                self.pos[ch] += v * dt / 1e6
            if t >= sig_m:
                M.append((round(t / 1e6, 4), self._mv(ch)))
                sig_m += 2000
            t += dt
        self.us[ch] = us_a
        return {"total_s": total / 1e6, "M": M, "C": C}

    def soltar(self):
        self.us = {ch: 0 for ch in range(N_SERVOS)}

    def esperar(self, s):
        pass

    def cerrar(self):
        pass


# ======================================================================= medición

def imprimir_plan(a, esc):
    us_a, us_b = esc.real_a_us(a.desde), esc.real_a_us(a.hasta)
    print("\n" + "=" * 76)
    print("PRUEBA 04 · RAMPAS — velocidad contra error")
    print("=" * 76)
    print(f"Recorrido: {a.desde:g}° → {a.hasta:g}° reales ({us_a} → {us_b} µs) · {a.vueltas} idas y vueltas · "
          f"{a.espera_ms} ms quieto en cada extremo")
    print(f"Servos: {', '.join(str(c) for c in a.canal)} (uno por uno)\n")
    print("  velocidad (°/s)   µs/s   duración de cada rampa   cada servo")
    total = 0.0
    for v in a.velocidades:
        vel_us = int(round(v * esc.us_por_grado))
        dur = abs(us_b - us_a) / vel_us
        ciclo = duracion_ciclo(us_a, us_b, vel_us, a.vueltas, a.espera_ms)
        datos = (ciclo / 0.002 * 14 + ciclo / 0.02 * 13) / 11520
        total += len(a.canal) * (ciclo + datos + PAUSA_S)
        print(f"  {v:>10g}       {vel_us:>6}        {dur:5.2f} s              {ciclo:5.1f} s + ~{datos:.0f} s de datos")
    print(f"\n  Tiempo estimado: ~{total / 60:.0f} min")
    return us_a, us_b


def revisar_inicio(esp, canales):
    if esp.info.get("pca") == "NO":
        raise ErrorESP32("la ESP32 no encuentra el PCA9685 (PCA=NO). Revisa SDA 21, SCL 22, VCC 3V3 y GND.")
    if esp.info.get("proto", 1) < 2:
        raise ErrorESP32("la principal no tiene el comando G (falta PROTO=2 en '?'). Carga el "
                         "esp32_principal.ino nuevo.")
    if any(c >= 3 for c in canales) and esp.info.get("aux") == "NO":
        raise ErrorESP32("la auxiliar no está mandando datos (AUX=NO). Revisa TX/RX cruzados (16 <-> 17), "
                         "GND común y que tenga esp32_aux.ino.")
    v = esp.leer_todos()
    malos = [c for c in canales if v[c] is None or not 60 <= v[c] <= 2500]
    if malos:
        raise ErrorESP32("cables verdes fuera de rango (lo normal es 190–1870 mV): "
                         + ", ".join(f"{c} ({corto(c)}) = {v[c]}" for c in malos))
    return v


def acercar(esp, ch, desde_us, hasta_us, esc, dt=0.03):
    """Lleva un servo de desde_us a hasta_us a VEL_ACERCAR °/s, con P."""
    paso = VEL_ACERCAR * esc.us_por_grado * dt
    n = max(1, int(math.ceil(abs(hasta_us - desde_us) / paso)))
    for k in range(1, n + 1):
        esp.mover(ch, int(round(desde_us + (hasta_us - desde_us) * k / n)))
        esp.esperar(dt)
    esp.mover(ch, hasta_us)


def medir(a, esc, retro):
    us_a, us_b = imprimir_plan(a, esc)
    esp = ESP32Simulado(esc, retro, a.semilla) if a.simulado else ESP32(a.port)
    print(f"\nESP32: {esp.info['linea']}")
    try:
        v0 = revisar_inicio(esp, a.canal)
    except ErrorESP32 as e:
        esp.cerrar()
        sys.exit(f"Error: {e}")
    if not a.simulado:
        print("\nLos servos se van a mover uno por uno, entre "
              f"{a.desde:g}° y {a.hasta:g}° reales. Que estén al aire y sin piezas que choquen.")
        print("Ctrl+C detiene todo (la rampa en curso se para y se sueltan los servos).")
        input("Enter para empezar (Ctrl+C para salir) ")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = os.path.join(a.salida, stamp + ("_sim" if a.simulado else ""))
    os.makedirs(carpeta, exist_ok=True)
    meta = {"fecha": stamp, "prueba": "04_rampas", "velocidades": a.velocidades, "canales": a.canal,
            "desde_grados": a.desde, "hasta_grados": a.hasta, "us_a": us_a, "us_b": us_b,
            "vueltas": a.vueltas, "espera_ms": a.espera_ms, "pausa_s": PAUSA_S, "voltaje": a.voltaje,
            "simulado": a.simulado, "esp32": esp.info, "escala": esc.fuente, "factor_real": esc.factor,
            "us_por_grado_real": esc.us_por_grado, "calibracion": retro.fuente,
            "lectura_inicial_mv": {str(c): v0[c] for c in a.canal}}
    guardar_json(os.path.join(carpeta, "meta.json"), meta)

    f_m = open(os.path.join(carpeta, "muestras.csv"), "w", newline="", encoding="utf-8")
    f_c = open(os.path.join(carpeta, "comandos.csv"), "w", newline="", encoding="utf-8")
    f_k = open(os.path.join(carpeta, "ciclos.csv"), "w", newline="", encoding="utf-8")
    w_m, w_c, w_k = csv.writer(f_m), csv.writer(f_c), csv.writer(f_k)
    w_m.writerow(["velocidad", "canal", "t_s", "mv"])
    w_c.writerow(["velocidad", "canal", "t_s", "us"])
    w_k.writerow(["velocidad", "canal", "us_a", "us_b", "vel_us_s", "vueltas", "espera_ms", "total_s",
                  "n_muestras", "n_comandos", "estado"])
    # dónde está cada servo (µs): al principio, lo que diga su cable verde
    actual = {c: esc.real_a_us(retro.grados(c, v0[c])) for c in a.canal}
    t0 = time.time()
    hechos, total = 0, len(a.velocidades) * len(a.canal)
    fallas_seguidas = 0
    cancelado = False
    try:
        for v in a.velocidades:
            vel_us = int(round(v * esc.us_por_grado))
            print(f"\n— {v:g} °/s ({vel_us} µs/s)")
            for ch in a.canal:
                if actual[ch] != us_a:
                    acercar(esp, ch, actual[ch], us_a, esc)
                    actual[ch] = us_a
                else:
                    esp.mover(ch, us_a)
                esp.esperar(PAUSA_S)
                estado = "ok"
                try:
                    r = esp.rampa(ch, us_a, us_b, vel_us, a.vueltas, a.espera_ms)
                except ErrorESP32 as e:
                    estado = f"error: {e}"
                    r = {"total_s": float("nan"), "M": [], "C": []}
                    print(f"  {ch:>2} {corto(ch):<7} ERROR: {e}")
                    fallas_seguidas += 1
                    if fallas_seguidas >= 3:
                        raise ErrorESP32("tres rampas seguidas fallaron; se detiene la prueba")
                    actual[ch] = -1                     # no se sabe dónde quedó: se vuelve a acercar
                    try:
                        vv = esp.leer_todos()[ch]
                        if vv is not None:
                            actual[ch] = esc.real_a_us(retro.grados(ch, vv))
                    except ErrorESP32:
                        pass
                else:
                    fallas_seguidas = 0
                for t, mv in r["M"]:
                    w_m.writerow([v, ch, f"{t:.4f}", mv])
                for t, us in r["C"]:
                    w_c.writerow([v, ch, f"{t:.4f}", us])
                w_k.writerow([v, ch, us_a, us_b, vel_us, a.vueltas, a.espera_ms, r["total_s"], len(r["M"]),
                              len(r["C"]), estado])
                for f in (f_m, f_c, f_k):
                    f.flush()
                hechos += 1
                if estado == "ok":
                    m = metricas_ciclo(np.array(r["M"], float), np.array(r["C"], float), us_a, us_b, vel_us,
                                       a.vueltas, a.espera_ms, ch, esc, retro)
                    prom = promedio(m)
                    trans = time.time() - t0
                    falta = trans / hechos * (total - hechos) / 60
                    print(f"  {ch:>2} {corto(ch):<7} atraso {fmt(prom['atraso_ms'], 0):>4} ms · error {fmt(prom['error_rampa']):>5}° · "
                          f"vel real {fmt(prom['vel_real'], 0):>4} °/s · máx {fmt(prom['error_max']):>5}° · "
                          f"asienta {fmt(prom['asentamiento_ms'], 0):>4} ms   [{hechos}/{total}, faltan ~{falta:.0f} min]")
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
    meta["duracion_min"] = round((time.time() - t0) / 60, 1)
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    return carpeta


# ======================================================================= análisis

def segmentos(us_a, us_b, vel_us, vueltas, espera_ms):
    """(vuelta, sentido, t_inicio, t_fin_rampa, t_fin_espera, us_desde, us_hasta) de cada rampa."""
    dur = abs(us_b - us_a) / vel_us
    esp = espera_ms / 1000.0
    out = []
    t = esp
    for k in range(vueltas):
        out.append((k + 1, "ida", t, t + dur, t + dur + esp, us_a, us_b))
        t += dur + esp
        out.append((k + 1, "vuelta", t, t + dur, t + dur + esp, us_b, us_a))
        t += dur + esp
    return out


def media_movil(x, n=5):
    if len(x) < n:
        return x.copy()
    k = np.ones(n) / n
    y = np.convolve(x, k, mode="same")
    y[: n // 2] = x[: n // 2]
    y[-(n // 2):] = x[-(n // 2):]
    return y


def metricas_ciclo(M, C, us_a, us_b, vel_us, vueltas, espera_ms, ch, esc, retro):
    """Métricas de cada rampa (ida y vuelta) de un ciclo."""
    if len(M) < 10 or len(C) < 2:
        return []
    tm = M[:, 0]
    ang = np.array([retro.grados(ch, mv) for mv in M[:, 1]])
    suave = media_movil(ang, 5)
    tc, uc = C[:, 0], C[:, 1]
    idx = np.clip(np.searchsorted(tc, tm, side="right") - 1, 0, len(tc) - 1)
    pedido = np.array([esc.us_a_real(u) for u in uc[idx]])       # retención de orden cero, como el PCA
    v_ped = vel_us / esc.us_por_grado
    out = []
    for vuelta, sentido, ti, tf, te, u0, u1 in segmentos(us_a, us_b, vel_us, vueltas, espera_ms):
        signo = 1.0 if u1 > u0 else -1.0
        d = tf - ti
        cen = (tm >= ti + TRAMO[0] * d) & (tm <= ti + TRAMO[1] * d)
        todo = (tm >= ti) & (tm < te)
        esp_ = (tm >= tf) & (tm < te)
        final = (tm >= te - 0.1) & (tm < te)
        r = {"vuelta": vuelta, "sentido": sentido, "vel_pedida": v_ped}
        if cen.sum() >= 5:
            e = signo * (pedido[cen] - ang[cen])
            r["error_rampa"] = float(np.mean(e))
            r["atraso_ms"] = r["error_rampa"] / v_ped * 1000
            pend = np.polyfit(tm[cen], ang[cen], 1)[0]
            r["vel_real"] = float(abs(pend))
        else:
            r["error_rampa"] = r["atraso_ms"] = r["vel_real"] = float("nan")
        r["error_max"] = float(np.max(np.abs(pedido[todo] - suave[todo]))) if todo.any() else float("nan")
        obj = esc.us_a_real(u1)
        if final.sum() >= 3 and esp_.sum() >= 3:
            vf = float(np.mean(ang[final]))
            r["error_final"] = vf - obj
            r["sobrepaso"] = float(max(0.0, np.max(signo * (suave[esp_] - vf))))
            fuera = np.where(np.abs(suave[esp_] - vf) > BANDA)[0]
            r["asentamiento_ms"] = float((tm[esp_][fuera[-1]] - tf) * 1000) if len(fuera) else 0.0
            if len(fuera) and fuera[-1] >= esp_.sum() - 3:
                r["asentamiento_ms"] = float("nan")              # no se asentó en la espera
        else:
            r["error_final"] = r["sobrepaso"] = r["asentamiento_ms"] = float("nan")
        out.append(r)
    return out


CAMPOS = ["error_rampa", "atraso_ms", "vel_real", "error_max", "sobrepaso", "asentamiento_ms", "error_final"]


def promedio(filas):
    out = {}
    for k in CAMPOS:
        xs = [f[k] for f in filas if math.isfinite(f.get(k, float("nan")))]
        out[k] = float(np.mean(xs)) if xs else float("nan")
    return out


def cargar(carpeta):
    def leer(nombre):
        with open(os.path.join(carpeta, nombre), newline="", encoding="utf-8") as f:
            return list(csv.DictReader(f))
    ciclos = leer("ciclos.csv")
    datos = {}
    for nombre_csv, clave, col in (("muestras.csv", "M", "mv"), ("comandos.csv", "C", "us")):
        arr = np.genfromtxt(os.path.join(carpeta, nombre_csv), delimiter=",", skip_header=1, ndmin=2)
        if arr.size == 0:
            continue
        for (v, ch) in {(float(r[0]), int(r[1])) for r in arr}:
            sel = (arr[:, 0] == v) & (arr[:, 1] == ch)
            datos.setdefault((v, ch), {})[clave] = arr[sel][:, 2:4]
    return ciclos, datos


def analizar(carpeta, esc, retro):
    with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    ciclos, datos = cargar(carpeta)
    filas = []
    for k in ciclos:
        if k["estado"] != "ok":
            continue
        v, ch = float(k["velocidad"]), int(k["canal"])
        d = datos.get((v, ch), {})
        if "M" not in d or "C" not in d:
            continue
        for r in metricas_ciclo(d["M"], d["C"], int(k["us_a"]), int(k["us_b"]), int(k["vel_us_s"]),
                                int(k["vueltas"]), int(k["espera_ms"]), ch, esc, retro):
            filas.append({"velocidad": v, "canal": ch, "nombre": nombre(ch), **r})
    if not filas:
        print("No hay rampas completas para analizar.")
        return
    with open(os.path.join(carpeta, "metricas.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0].keys()))
        w.writeheader()
        for r in filas:
            w.writerow({kk: (round(x, 3) if isinstance(x, float) and math.isfinite(x) else
                             ("" if isinstance(x, float) else x)) for kk, x in r.items()})

    vels = sorted({r["velocidad"] for r in filas})
    canales = sorted({r["canal"] for r in filas})
    por_servo = {(v, ch): promedio([r for r in filas if r["velocidad"] == v and r["canal"] == ch])
                 for v in vels for ch in canales}
    resumen = []
    for v in vels:
        fila = {"velocidad": v}
        for c in CAMPOS:
            xs = [por_servo[(v, ch)][c] for ch in canales if math.isfinite(por_servo[(v, ch)][c])]
            fila[c] = float(np.mean(xs)) if xs else float("nan")
            fila[c + "_min"] = float(np.min(xs)) if xs else float("nan")
            fila[c + "_max"] = float(np.max(xs)) if xs else float("nan")
        fila["n_servos"] = len([ch for ch in canales if math.isfinite(por_servo[(v, ch)]["error_rampa"])])
        resumen.append(fila)
    with open(os.path.join(carpeta, "resumen.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(resumen[0].keys()))
        w.writeheader()
        for r in resumen:
            w.writerow({kk: (round(x, 3) if isinstance(x, float) and math.isfinite(x) else
                             ("" if isinstance(x, float) else x)) for kk, x in r.items()})
    escribir_md(carpeta, meta, resumen, por_servo, vels, canales)
    graficas(carpeta, meta, resumen, por_servo, vels, canales, datos, ciclos, esc, retro)
    imprimir(resumen, carpeta)


def escribir_md(carpeta, meta, resumen, por_servo, vels, canales):
    L = ["# 04 · Rampas — resumen", "", "| | |", "|---|---|",
         f"| Fecha | {meta.get('fecha')} |",
         f"| Recorrido | {meta.get('desde_grados'):g}° → {meta.get('hasta_grados'):g}° reales "
         f"({meta.get('us_a')} → {meta.get('us_b')} µs) |",
         f"| Vueltas | {meta.get('vueltas')} idas y vueltas por servo y velocidad, {meta.get('espera_ms')} ms quieto en cada extremo |",
         f"| Fuente | {fmt(meta.get('voltaje'), 2) if meta.get('voltaje') else '—'} V |",
         f"| Simulado | {'sí' if meta.get('simulado') else 'no'} |",
         f"| Completa | {'no (se detuvo)' if meta.get('cancelado') else 'sí'} |", "",
         "## Promedio de los servos, por velocidad", "",
         "Entre paréntesis, el servo con el valor más bajo y el más alto.", "",
         "| Pedida (°/s) | Velocidad real (°/s) | Error en la rampa (°) | Atraso (ms) | Error máx (°) | "
         "Sobrepaso (°) | Asentamiento (ms) | Error final (°) |",
         "|---|---|---|---|---|---|---|---|"]

    def celda(r, c, d=1):
        return f"{fmt(r[c], d)} ({fmt(r[c + '_min'], d)}–{fmt(r[c + '_max'], d)})"
    for r in resumen:
        L.append(f"| {r['velocidad']:g} | {celda(r, 'vel_real', 0)} | {celda(r, 'error_rampa')} | "
                 f"{celda(r, 'atraso_ms', 0)} | {celda(r, 'error_max')} | {celda(r, 'sobrepaso', 2)} | "
                 f"{celda(r, 'asentamiento_ms', 0)} | {celda(r, 'error_final', 2)} |")
    L += ["", "## Error en la rampa por servo (°)", "",
          "| Servo | " + " | ".join(f"{v:g} °/s" for v in vels) + " |",
          "|---|" + "---|" * len(vels)]
    for ch in canales:
        L.append(f"| {ch} · {nombre(ch)} | " + " | ".join(fmt(por_servo[(v, ch)]['error_rampa']) for v in vels) + " |")
    L += ["", "## Atraso por servo (ms)", "",
          "| Servo | " + " | ".join(f"{v:g} °/s" for v in vels) + " |",
          "|---|" + "---|" * len(vels)]
    for ch in canales:
        L.append(f"| {ch} · {nombre(ch)} | " + " | ".join(fmt(por_servo[(v, ch)]['atraso_ms'], 0) for v in vels) + " |")
    L.append("")
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


def graficas(carpeta, meta, resumen, por_servo, vels, canales, datos, ciclos, esc, retro):
    plt = _plt()
    if plt is None:
        return
    fig, ejes = plt.subplots(2, 2, figsize=(10, 7))
    paneles = [("error_rampa", "error en la rampa (°)"), ("atraso_ms", "atraso (ms)"),
               ("vel_real", "velocidad real (°/s)"), ("error_max", "error máximo (°)")]
    for ax, (c, txt) in zip(ejes.flat, paneles):
        for ch in canales:
            ys = [por_servo[(v, ch)][c] for v in vels]
            ax.plot(vels, ys, color=NARANJA if ch >= 3 else AZUL, alpha=0.25, lw=1, marker="o", ms=2.5)
        ax.plot(vels, [r[c] for r in resumen], color=INK, lw=2, marker="o", ms=5, label="promedio")
        if c == "vel_real":
            ax.plot([min(vels), max(vels)], [min(vels), max(vels)], color=MUTED, ls="--", lw=1, label="pedida")
        ax.set_title(txt, loc="left", fontsize=9.5, color=INK)
        ax.set_xlabel("velocidad pedida (°/s)")
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.set_xticks(vels)
    ejes[1][0].legend(frameon=False, fontsize=8)
    fig.suptitle("Rampas: velocidad contra error" + (" · SIMULADO" if meta.get("simulado") else ""),
                 x=0.06, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.text(0.06, 0.935, "línea negra: promedio · azul: FL (principal) · naranja: FR, RL, RR (auxiliar)",
             fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(carpeta, "rampas.png"), dpi=150)
    plt.close(fig)

    # Trazas: una figura por servo, un panel por velocidad (la primera ida y vuelta)
    os.makedirs(os.path.join(carpeta, "trazas"), exist_ok=True)
    par = {(float(k["velocidad"]), int(k["canal"])): k for k in ciclos if k["estado"] == "ok"}
    for ch in canales:
        n = len(vels)
        cols = 4 if n > 4 else n
        filas_ = int(math.ceil(n / cols))
        fig, ejes = plt.subplots(filas_, cols, figsize=(3.1 * cols, 2.4 * filas_), squeeze=False)
        for i, v in enumerate(vels):
            ax = ejes[i // cols][i % cols]
            d, k = datos.get((v, ch)), par.get((v, ch))
            if not d or not k or "M" not in d or "C" not in d:
                ax.set_visible(False)
                continue
            seg = segmentos(int(k["us_a"]), int(k["us_b"]), int(k["vel_us_s"]), int(k["vueltas"]), int(k["espera_ms"]))
            t_fin = seg[1][4]                               # primera ida y vuelta
            M, C = d["M"], d["C"]
            selm, selc = M[:, 0] <= t_fin, C[:, 0] <= t_fin
            ax.step(C[selc, 0], [esc.us_a_real(u) for u in C[selc, 1]], where="post", color=MUTED, lw=1.2,
                    label="pedido")
            ax.plot(M[selm, 0], [retro.grados(ch, mv) for mv in M[selm, 1]], color=AZUL if ch < 3 else NARANJA,
                    lw=1, label="medido")
            ax.set_title(f"{v:g} °/s · atraso {fmt(por_servo[(v, ch)]['atraso_ms'], 0)} ms", loc="left",
                         fontsize=8.5, color=INK)
            ax.grid(True, color=GRID, lw=0.5)
            ax.tick_params(labelsize=7)
        for j in range(n, filas_ * cols):
            ejes[j // cols][j % cols].set_visible(False)
        ejes[0][0].legend(frameon=False, fontsize=7)
        fig.suptitle(f"Canal {ch} · {nombre(ch)} · primera ida y vuelta (grados reales contra segundos)",
                     x=0.02, ha="left", fontsize=10, fontweight="bold", color=INK)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(os.path.join(carpeta, "trazas", f"canal{ch:02d}.png"), dpi=120)
        plt.close(fig)


def imprimir(resumen, carpeta):
    print("\n" + "=" * 92)
    print("RESULTADOS — promedio de los servos por velocidad")
    print("=" * 92)
    print("  pedida °/s   real °/s   error rampa °   atraso ms   error máx °   sobrepaso °   asienta ms   final °")
    for r in resumen:
        print(f"  {r['velocidad']:>9g}   {fmt(r['vel_real'], 0):>8}   {fmt(r['error_rampa']):>13}   "
              f"{fmt(r['atraso_ms'], 0):>9}   {fmt(r['error_max']):>11}   {fmt(r['sobrepaso'], 2):>11}   "
              f"{fmt(r['asentamiento_ms'], 0):>10}   {fmt(r['error_final'], 2):>7}")
    print(f"\nResultados en {carpeta}")
    print("  resumen.md, rampas.png, trazas/canalNN.png, metricas.csv")


# ======================================================================= revisar

def revisar(a, esc, retro):
    esp = ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    esp.soltar()
    print("\nServos sueltos. Ángulo real de cada uno; gíralos a mano. Ctrl+C para salir.\n")
    print("  " + "".join(f"{corto(c):>8}" for c in a.canal))
    try:
        while True:
            try:
                v = esp.leer_todos()
                print("\r  " + "".join(f"{fmt(retro.grados(c, v[c])) if v[c] is not None else '—':>8}"
                                       for c in a.canal), end="", flush=True)
            except ErrorESP32:
                pass
            time.sleep(0.15)
    except KeyboardInterrupt:
        print()
    finally:
        esp.cerrar()


# ======================================================================= principal

def main():
    ap = argparse.ArgumentParser(description="Prueba 04 de Arca9: rampas, velocidad contra error.")
    ap.add_argument("--port", help="puerto de la ESP32 principal (COM6, /dev/ttyUSB0)")
    ap.add_argument("--canal", type=int, nargs="+", default=list(range(N_SERVOS)), help="servos, en orden")
    ap.add_argument("--velocidades", type=float, nargs="+", default=VELOCIDADES, help="°/s reales, en orden")
    ap.add_argument("--desde", type=float, default=DESDE, help="ángulo real de inicio (°)")
    ap.add_argument("--hasta", type=float, default=HASTA, help="ángulo real del otro extremo (°)")
    ap.add_argument("--vueltas", type=int, default=VUELTAS, help="idas y vueltas por servo y velocidad")
    ap.add_argument("--espera-ms", type=int, default=ESPERA_MS, help="ms quieto en cada extremo")
    ap.add_argument("--voltaje", type=float, help="voltaje de la fuente de los servos (para el registro)")
    ap.add_argument("--revisar", action="store_true", help="ángulos reales en vivo, servos sueltos")
    ap.add_argument("--simulado", action="store_true", help="sin hardware")
    ap.add_argument("--semilla", type=int, default=4)
    ap.add_argument("--reanalizar", metavar="RUTA", help="carpeta de una corrida o 'ultima'")
    ap.add_argument("--salida", default=os.path.join(HERE, "resultados_rampas"))
    a = ap.parse_args()

    try:
        esc = Escala()
        retro = Retro(esc)
    except (FileNotFoundError, ValueError, KeyError) as e:
        sys.exit(f"Error: {e}")
    a.canal = list(dict.fromkeys(a.canal))
    if any(not 0 <= c < N_SERVOS for c in a.canal):
        ap.error("los canales van de 0 a 11")
    lim = esc.limite_grados - 2.0
    if not (0 <= a.desde <= lim and 0 <= a.hasta <= lim and abs(a.hasta - a.desde) >= 10):
        ap.error(f"--desde y --hasta entre 0 y {lim:.1f}° reales (2° antes del límite de {US_LIMITE} µs), "
                 "separados al menos 10°")
    if any(not 5 <= v <= 350 for v in a.velocidades):
        ap.error("velocidades entre 5 y 350 °/s")
    if not 1 <= a.vueltas <= 10 or not 0 <= a.espera_ms <= 5000:
        ap.error("--vueltas de 1 a 10 y --espera-ms de 0 a 5000")
    us_a, us_b = esc.real_a_us(a.desde), esc.real_a_us(a.hasta)
    for v in a.velocidades:
        vel_us = int(round(v * esc.us_por_grado))
        if not 50 <= vel_us <= 4000:
            ap.error(f"{v:g} °/s = {vel_us} µs/s: el firmware acepta de 50 a 4000 µs/s")
        if duracion_ciclo(us_a, us_b, vel_us, a.vueltas, a.espera_ms) > 17.5:
            ap.error(f"a {v:g} °/s cada servo tarda más de 17.5 s y la ESP32 solo guarda ~17.6 s: "
                     "baja --vueltas o el recorrido")

    if a.reanalizar:
        ruta = a.reanalizar
        if ruta == "ultima":
            cands = sorted(d for d in (os.listdir(a.salida) if os.path.isdir(a.salida) else [])
                           if os.path.isfile(os.path.join(a.salida, d, "ciclos.csv")))
            if not cands:
                sys.exit(f"No hay corridas en {a.salida}")
            ruta = os.path.join(a.salida, cands[-1])
        print(f"Reanalizando {ruta}")
        analizar(ruta, esc, retro)
        return
    if a.simulado and a.revisar:
        ap.error("--revisar necesita el hardware (--port)")
    if not a.simulado and not a.port:
        ap.error("falta --port (o usa --simulado o --reanalizar)")
    try:
        if a.revisar:
            revisar(a, esc, retro)
            return
        carpeta = medir(a, esc, retro)
    except ErrorESP32 as e:
        sys.exit(f"Error: {e}")
    except KeyboardInterrupt:
        sys.exit("\nCancelado antes de empezar; no se movió nada.")
    analizar(carpeta, esc, retro)


if __name__ == "__main__":
    main()
