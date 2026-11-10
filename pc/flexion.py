#!/usr/bin/env python3
"""
Prueba 08 · Flexión y extensión completas de la rodilla, cada vez más rápido — Arca9
=====================================================================================
La pregunta del profe: "dile que haga flexión total, que extienda la patita completa y que lo
haga rápido; mide la latencia entre que le das la instrucción y la ejecuta".

La rodilla de una pata (FL por defecto) va de casi recta a casi toda doblada y regresa, a
velocidades cada vez más altas, hasta la más rápida que acepta el firmware (comando G: 4000 µs/s,
~375 °/s reales, más de lo que da el servo). La abducción y la cadera se quedan quietas.

La rampa la hace la ESP32 principal (comando G, como la prueba 04): el pulso cambia cada 20 ms y
el cable verde se graba cada ~2 ms con el reloj de la placa. Así la latencia que se mide es la del
servo con la pata puesta, sin el USB ni Windows de por medio.

Qué se mide en cada flexión y cada extensión:
  - Latencia: desde que el pedido se movió 2° hasta que el medido se movió 2°.
  - Atraso en la rampa: cuánto va atrás el servo cuando ya va a velocidad constante (como la 04).
  - Velocidad real y velocidad pico del servo.
  - Sobrepaso al llegar, asentamiento (±1°) y error final.
  - Tiempo total: desde la orden hasta que la rodilla quedó quieta en su lugar.

Seguridad:
  - Recorrido: los límites de la pata (pata.py: los medidos menos 5°, la rodilla 2° antes de
    recta) y además --margen grados (3 por defecto) en cada lado, porque rápido se pasa un poco.
  - La pata debe estar adentro de sus límites para empezar; si no, no se mueve nada.
  - Primero va despacio (40 °/s) a la pose de inicio, y las velocidades van de menor a mayor.
  - Después de cada velocidad revisa que la rodilla haya llegado a los dos extremos; si no
    (¿choca?, ¿le falta fuerza?), se detiene y suelta.
  - Ctrl+C detiene todo y suelta los servos (la rampa en curso se para en la placa).

Uso (desde la carpeta Arca9, con el venv activado y el Monitor Serie cerrado):
  python pc/flexion.py --simulado                     # practicar sin hardware
  python pc/flexion.py --port COM6 --revisar          # dónde está la pata y el plan; no mueve nada
  python pc/flexion.py --port COM6 --voltaje 6.0      # la prueba (~3 min)
  python pc/flexion.py --port COM6 --velocidades 60 120     # una prueba corta
  python pc/flexion.py --reanalizar ultima            # rehace tablas y gráficas sin hardware

Resultados en pc/resultados_flexion/AAAAMMDD_HHMMSS/ (meta.json, corridas.csv, muestras.csv,
comandos.csv, metricas.csv, resumen.md, trazas.png y flexion.png). La corrida buena se copia a
datos/AAAA-MM-DD_flexion/, como en las otras pruebas.
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
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from arca9_serial import ESP32, ErrorESP32                                     # noqa: E402
from escala import Escala, Retro, MV_MIN, MV_MAX                                # noqa: E402
from pata import Pata, ART, buscar_pata                                         # noqa: E402
import rampas                                                                   # noqa: E402
from rampas import (ESP32Simulado, guardar_json, fmt, media_movil, metricas_ciclo, segmentos,  # noqa: E402
                    duracion_ciclo, _plt, INK, MUTED, GRID, AZUL, NARANJA)

VELOCIDADES = ["60", "120", "180", "240", "max"]
VEL_G_MAX = 4000             # µs/s: lo más rápido que acepta G en el firmware (~375 °/s reales)
VUELTAS = 2
ESPERA_MS = 600
MARGEN = 3.0                 # ° extra en cada extremo (además del de pata.py)
VEL_ACERCAR = 40.0           # °/s para llevar la pata a la pose de inicio
UMBRAL = 2.0                 # ° para decir "ya se movió" (el ruido es ~0.3°)
TOLERANCIA = 0.01            # fracción de líneas M o C que se puede perder en el USB (1 %): una muestra de
                             # 2 ms que falta no cambia nada; si faltan más, algo anda mal y se detiene
NOMBRES = {"abd": "abducción", "cad": "cadera", "rod": "rodilla"}
CAMPOS = ["latencia_ms", "atraso_ms", "vel_real", "vel_pico", "sobrepaso", "asentamiento_ms",
          "tiempo_total_ms", "error_final"]


# ======================================================================= el plan

def plan(a, pata, esc):
    """Pose de inicio y extremos de la rodilla, en grados del servo y en µs. ValueError si no cabe."""
    lo, hi = pata.lim["rod"]
    ext_q, flex_q = sorted(pata.q([pata.cero["abd"], pata.cero["cad"], g])[2] for g in (lo, hi))
    ext_q, flex_q = ext_q + a.margen, flex_q - a.margen          # q_rod: 0 = recta, + = doblada
    if flex_q - ext_q < 30:
        raise ValueError(f"con --margen {a.margen:g} a la rodilla le quedan {flex_q - ext_q:.0f}°: muy poco")
    th_ext = pata.servo(0.0, a.cadera, ext_q)
    th_flex = pata.servo(0.0, a.cadera, flex_q)
    for nombre, th in (("extendida", th_ext), ("doblada", th_flex)):
        fuera = pata.fuera(th)
        if fuera:
            raise ValueError(f"la pose {nombre} sale de los límites: " + ", ".join(
                f"{NOMBRES[x]} {g:.1f}° (límites {l_:.1f}–{h_:.1f}°)" for x, g, (l_, h_) in fuera))
    vels = []
    for v in a.velocidades:
        if str(v).lower() == "max":
            vels.append(("max", VEL_G_MAX / esc.us_por_grado, VEL_G_MAX))
        else:
            v = float(v)
            us = int(round(v * esc.us_por_grado))
            if not 30 <= v or us > VEL_G_MAX:
                raise ValueError(f"velocidad {v:g} °/s: debe ser de 30 a {VEL_G_MAX / esc.us_por_grado:.0f} °/s (o 'max')")
            vels.append((f"{v:g}", v, us))
    vels.sort(key=lambda x: x[2])
    return {"cadera_q": a.cadera, "ext_q": ext_q, "flex_q": flex_q, "th_ext": th_ext, "th_flex": th_flex,
            "us_ext": [esc.real_a_us(g) for g in th_ext], "us_flex": [esc.real_a_us(g) for g in th_flex],
            "vels": vels}


def imprimir_plan(p, pata, esc, a):
    us_a, us_b = p["us_ext"][2], p["us_flex"][2]
    print("\n" + "=" * 78)
    print(f"PRUEBA 08 · FLEXIÓN Y EXTENSIÓN COMPLETAS — rodilla {pata.nombre} (canal {pata.canales[2]})")
    print("=" * 78)
    pie_e, pie_f = pata.fk(p["cadera_q"], p["ext_q"]), pata.fk(p["cadera_q"], p["flex_q"])
    print(f"Abducción y cadera quietas: {p['th_ext'][0]:.1f}° y {p['th_ext'][1]:.1f}° del servo "
          f"(pata vertical de frente; muslo a {p['cadera_q']:+.0f}° de vertical)")
    print(f"Rodilla: extendida {p['ext_q']:.1f}° de flexión ({p['th_ext'][2]:.1f}° del servo, {us_a} µs; "
          f"pie en x {pie_e[0]:+.1f}, z {pie_e[1]:.1f} cm)")
    print(f"         doblada   {p['flex_q']:.1f}° de flexión ({p['th_flex'][2]:.1f}° del servo, {us_b} µs; "
          f"pie en x {pie_f[0]:+.1f}, z {pie_f[1]:.1f} cm)")
    print(f"         recorrido {p['flex_q'] - p['ext_q']:.1f}° · {a.margen:g}° más adentro de los límites de pata.py")
    lo, hi = pata.lim_medidos["rod"]
    print(f"Límites de la rodilla en {os.path.basename(pata.fuente)}: {lo:g}–{hi:g}° del servo (medidos); con el "
          f"margen de pata.py, {pata.lim['rod'][0]:.1f}–{pata.lim['rod'][1]:.1f}°. ¿Coinciden con tus notas?")
    print(f"\n{a.vueltas} flexiones y extensiones por velocidad, {a.espera_ms} ms quieta en cada extremo:\n")
    print("  velocidad (°/s)   µs/s   cada movimiento   en total")
    total = 0.0
    for nombre, v, us in p["vels"]:
        dur = abs(us_b - us_a) / us
        ciclo = duracion_ciclo(us_a, us_b, us, a.vueltas, a.espera_ms)
        datos = (ciclo / 0.002 * 14 + ciclo / 0.02 * 13) / 11520
        total += ciclo + datos + 1.0
        print(f"  {etiqueta(nombre, v):>14}   {us:>5}      {dur:5.2f} s       "
              f"{ciclo:4.1f} s + ~{datos:.0f} s de datos")
    print(f"\n  Tiempo estimado: ~{total / 60:.1f} min")


# ======================================================================= hardware

def abrir(a, esc, retro, pata, p):
    if a.simulado:
        esp = ESP32Simulado(esc, retro, a.semilla)
        rng = np.random.default_rng(a.semilla)
        for ch, g in zip(pata.canales, p["th_ext"]):         # como si la hubieras dejado así a mano
            esp.pos[ch] = float(g + rng.uniform(-3, 3))
        return esp
    return ESP32(a.port)


def revisar_inicio(esp, pata, retro):
    """Revisa la ESP32 y dónde está la pata. Regresa los 3 ángulos medidos. ErrorESP32 si no se puede."""
    if esp.info.get("pca") == "NO":
        raise ErrorESP32("la ESP32 no encuentra el PCA9685 (PCA=NO).")
    if esp.info.get("proto", 1) < 2:
        raise ErrorESP32("la principal no tiene el comando G (falta PROTO=2 en '?'). Carga esp32_principal.")
    if any(c >= 3 for c in pata.canales) and esp.info.get("aux") == "NO":
        raise ErrorESP32("la auxiliar no está mandando datos (AUX=NO).")
    v = esp.leer_todos()
    malos = [c for c in pata.canales if v[c] is None or not MV_MIN <= v[c] <= MV_MAX]
    if malos:
        raise ErrorESP32("cable verde fuera de rango (lo creíble es "
                         f"{MV_MIN}–{MV_MAX} mV; ¿cable suelto?): " + ", ".join(f"canal {c} = {v[c]} mV" for c in malos))
    g = [retro.grados(c, v[c]) for c in pata.canales]
    fuera = pata.fuera(g)
    if fuera:
        raise ErrorESP32("la pata está fuera de sus límites, así que no se mueve nada:\n  - " + "\n  - ".join(
            f"{NOMBRES[x]} en {gg:.1f}° (límites {lo:.1f}°–{hi:.1f}°)" for x, gg, (lo, hi) in fuera)
            + "\nCon los servos sueltos, dobla un poco la rodilla con la mano (que no quede recta) y vuelve a correr.")
    return g


def acercar(esp, canales, desde, hasta, esc, dt=0.03):
    """Los 3 servos juntos, en línea recta, de `desde` a `hasta` (µs) a ~VEL_ACERCAR °/s, con P."""
    salto = max(abs(h - d) for d, h in zip(desde, hasta)) / esc.us_por_grado
    n = max(1, int(math.ceil(salto / (VEL_ACERCAR * dt))))
    for k in range(1, n + 1):
        for ch, d, h in zip(canales, desde, hasta):
            esp.mover(ch, int(round(d + (h - d) * k / n)))
        esp.esperar(dt)


# ======================================================================= análisis

def metricas(M, C, us_a, us_b, vel_us, vueltas, espera_ms, ch, esc, retro):
    """Las de la prueba 04 (rampas.metricas_ciclo) más latencia, tiempo total y velocidad pico."""
    base = metricas_ciclo(M, C, us_a, us_b, vel_us, vueltas, espera_ms, ch, esc, retro)
    tm = M[:, 0]
    ang = np.array([retro.grados(ch, mv) for mv in M[:, 1]])
    suave = media_movil(ang, 5)
    lento = media_movil(ang, 11)
    tc, uc = C[:, 0], C[:, 1]
    ped = np.array([esc.us_a_real(u) for u in uc])
    out = []
    for r, (vuelta, sentido, ti, tf, te, u0, u1) in zip(base, segmentos(us_a, us_b, vel_us, vueltas, espera_ms)):
        r = dict(r)
        r["movimiento"] = "flexión" if sentido == "ida" else "extensión"
        signo = 1.0 if u1 > u0 else -1.0
        g0 = esc.us_a_real(u0)
        # orden: el primer pulso que se aparta del de reposo (después de ti - 1 ms)
        k = np.where((tc >= ti - 0.001) & (tc < te) & (np.abs(ped - g0) > 1e-6))[0]
        reposo = (tm >= ti - 0.1) & (tm < ti)
        if len(k) and reposo.sum() >= 5:
            t_orden = tc[k[0]]
            k2 = np.where((tc >= ti - 0.001) & (tc < te) & (signo * (ped - g0) >= UMBRAL))[0]
            quieto = float(np.mean(ang[reposo]))
            m2 = np.where((tm >= ti) & (tm < te) & (signo * (suave - quieto) >= UMBRAL))[0]
            r["latencia_ms"] = (tm[m2[0]] - tc[k2[0]]) * 1000 if len(k2) and len(m2) else float("nan")
            fin = (tm >= te - 0.1) & (tm < te)
            seg = (tm >= ti) & (tm < te)
            if fin.sum() >= 3:
                vf = float(np.mean(ang[fin]))
                fuera = np.where(seg & (np.abs(suave - vf) > rampas.BANDA))[0]
                if len(fuera) and fuera[-1] < np.where(seg)[0][-1] - 3:
                    r["tiempo_total_ms"] = (tm[fuera[-1] + 1] - t_orden) * 1000
                else:
                    r["tiempo_total_ms"] = float("nan")       # no se quedó quieta antes de la siguiente
            else:
                r["tiempo_total_ms"] = float("nan")
        else:
            r["latencia_ms"] = r["tiempo_total_ms"] = float("nan")
        idx = np.where((tm >= ti) & (tm < te))[0]
        idx = idx[(idx >= 5) & (idx < len(tm) - 5)]
        if len(idx):
            v = (lento[idx + 5] - lento[idx - 5]) / (tm[idx + 5] - tm[idx - 5])
            r["vel_pico"] = float(np.max(np.abs(v)))
        else:
            r["vel_pico"] = float("nan")
        # ¿llegó a los dos extremos? (para la seguridad entre velocidades)
        fin = (tm >= te - 0.1) & (tm < te)
        r["llego"] = bool(fin.sum() >= 3 and abs(float(np.mean(ang[fin])) - esc.us_a_real(u1)) < 6.0)
        out.append(r)
    return out


def promedio(filas, campo):
    x = [f[campo] for f in filas if isinstance(f.get(campo), float) and math.isfinite(f[campo])]
    return float(np.mean(x)) if x else float("nan")


def cargar(carpeta):
    with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)

    def leer(nombre, columna):
        filas = {}
        with open(os.path.join(carpeta, nombre), encoding="utf-8") as f:
            for r in csv.DictReader(f):
                filas.setdefault(r["velocidad"], []).append((float(r["t_s"]), float(r[columna])))
        return {k: np.array(v) for k, v in filas.items()}
    return meta, leer("muestras.csv", "mv"), leer("comandos.csv", "us")


def analizar(carpeta, esc, retro, pata):
    meta, M, C = cargar(carpeta)
    ch = meta["canal"]
    filas, resumen = [], []
    for nombre, v_ped, vel_us in meta["velocidades"]:
        if nombre not in M or nombre not in C:
            continue
        ms = metricas(M[nombre], C[nombre], meta["us_ext"], meta["us_flex"], vel_us, meta["vueltas"],
                      meta["espera_ms"], ch, esc, retro)
        for r in ms:
            r["velocidad"] = nombre
            filas.append(r)
        for mov in ("flexión", "extensión"):
            sub = [r for r in ms if r["movimiento"] == mov]
            resumen.append({"velocidad": nombre, "vel_pedida": v_ped, "movimiento": mov,
                            **{c: promedio(sub, c) for c in CAMPOS}})
    with open(os.path.join(carpeta, "metricas.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        cols = ["velocidad", "vuelta", "movimiento", "vel_pedida"] + CAMPOS + ["error_max", "error_rampa"]
        w.writerow(cols)
        for r in filas:
            w.writerow([r.get(c) if not isinstance(r.get(c), float) else round(r[c], 3) for c in cols])
    escribir_md(carpeta, meta, resumen)
    graficas(carpeta, meta, resumen, M, C, esc, retro, pata)
    return resumen


def etiqueta(nombre, v):
    return ("máx. " if nombre == "max" else "") + f"{v:.0f}"


def escribir_md(carpeta, meta, resumen):
    l = [f"# Prueba 08 · Flexión y extensión — {meta['fecha']}{' (SIMULADA)' if meta['simulado'] else ''}", "",
         f"Rodilla {meta['pata']} (canal {meta['canal']}), de {meta['ext_q']:.1f}° a {meta['flex_q']:.1f}° de flexión "
         f"({meta['flex_q'] - meta['ext_q']:.0f}°), cadera a {meta['cadera_q']:+.0f}°. {meta['vueltas']} vueltas por "
         f"velocidad. Fuente: {meta.get('voltaje') or '¿?'} V.", "",
         "| Velocidad pedida | Movimiento | Latencia (ms) | Atraso en la rampa (ms) | Vel. real (°/s) | "
         "Vel. pico (°/s) | Sobrepaso (°) | Asentamiento (ms) | Tiempo total (ms) | Error final (°) |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for r in resumen:
        l.append(f"| {etiqueta(r['velocidad'], r['vel_pedida'])} °/s | {r['movimiento']} | "
                 f"{fmt(r['latencia_ms'], 0)} | {fmt(r['atraso_ms'], 0)} | {fmt(r['vel_real'], 0)} | "
                 f"{fmt(r['vel_pico'], 0)} | {fmt(r['sobrepaso'])} | {fmt(r['asentamiento_ms'], 0)} | "
                 f"{fmt(r['tiempo_total_ms'], 0)} | {fmt(r['error_final'])} |")
    l += ["", "- **Latencia:** desde que el pedido se movió 2° hasta que el cable verde se movió 2°.",
          "- **Tiempo total:** desde el primer pulso nuevo hasta que la rodilla quedó a ±1° de donde se quedó.",
          "- **Atraso en la rampa:** en la segunda mitad de la rampa (como la prueba 04); a la velocidad "
          "máxima la rampa dura muy poco (0.3 s) y es menos confiable.", ""]
    with open(os.path.join(carpeta, "resumen.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(l))


def graficas(carpeta, meta, resumen, M, C, esc, retro, pata):
    plt = _plt()
    if plt is None:
        return
    ch = meta["canal"]

    def flexion(g):                       # grados del servo -> grados de flexión de la rodilla
        return (np.asarray(g) - pata.cero["rod"]) / pata.signo["rod"]
    vels = [v for v in meta["velocidades"] if v[0] in M]
    if not vels:
        return
    fig, ejes = plt.subplots(len(vels), 1, figsize=(10, 2.3 * len(vels)), squeeze=False)
    for ax, (nombre, v_ped, vel_us) in zip(ejes[:, 0], vels):
        m, c = M[nombre], C[nombre]
        ang = np.array([retro.grados(ch, mv) for mv in m[:, 1]])
        ax.step(c[:, 0], flexion([esc.us_a_real(u) for u in c[:, 1]]), where="post", color=MUTED, lw=1.2,
                label="pedido (pulso cada 20 ms)")
        ax.plot(m[:, 0], flexion(ang), color=AZUL, lw=1.0, label="medido (cable verde, cada 2 ms)")
        ax.set_ylabel(f"{etiqueta(nombre, v_ped)} °/s\nflexión [°]")
        ax.grid(color=GRID, lw=0.6)
    ejes[0, 0].legend(loc="upper right", frameon=False)
    ejes[-1, 0].set_xlabel("tiempo desde que empezó la rampa [s]")
    fig.suptitle(f"Prueba 08 · rodilla {meta['pata']}: flexión y extensión completas", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "trazas.png"), dpi=120)
    plt.close(fig)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
    for mov, col in (("flexión", AZUL), ("extensión", NARANJA)):
        rs = [r for r in resumen if r["movimiento"] == mov]
        x = [r["vel_pedida"] for r in rs]
        a1.plot(x, [r["latencia_ms"] for r in rs], "o-", color=col, label=f"latencia, {mov}")
        a1.plot(x, [r["tiempo_total_ms"] for r in rs], "s--", color=col, alpha=0.6, label=f"tiempo total, {mov}")
        a2.plot(x, [r["vel_pico"] for r in rs], "o-", color=col, label=f"pico, {mov}")
    top = max(r["vel_pedida"] for r in resumen)
    a2.plot([0, top], [0, top], color=MUTED, lw=0.8, ls=":", label="= pedida")
    a1.set_xlabel("velocidad pedida [°/s]"); a1.set_ylabel("ms"); a1.set_yscale("log")
    a2.set_xlabel("velocidad pedida [°/s]"); a2.set_ylabel("velocidad del servo [°/s]")
    for ax in (a1, a2):
        ax.grid(color=GRID, lw=0.6)
        ax.legend(frameon=False, fontsize=8)
    fig.suptitle("Latencia, tiempo total y velocidad de la rodilla", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "flexion.png"), dpi=120)
    plt.close(fig)


def imprimir(resumen, carpeta):
    if not resumen:
        print(f"\n  No terminó ninguna velocidad: no hay nada que analizar (lo que sí quedó está en "
              f"{os.path.relpath(carpeta)}).")
        return
    print("\n  velocidad   movimiento   latencia   tiempo total   vel. pico   sobrepaso   error final")
    for r in resumen:
        v = etiqueta(r["velocidad"], r["vel_pedida"])
        print(f"  {v:>9}   {r['movimiento']:<10}   {fmt(r['latencia_ms'], 0):>5} ms   {fmt(r['tiempo_total_ms'], 0):>8} ms"
              f"   {fmt(r['vel_pico'], 0):>5} °/s   {fmt(r['sobrepaso']):>6}°   {fmt(r['error_final']):>8}°")
    print(f"\n  Todo en {os.path.relpath(carpeta)} (resumen.md, trazas.png, flexion.png)")


# ======================================================================= modos

def medir(a, pata, esc, retro, p):
    imprimir_plan(p, pata, esc, a)
    esp = abrir(a, esc, retro, pata, p)
    t_abierto = time.time()
    print(f"\nESP32: {esp.info['linea']}")
    try:
        actual = revisar_inicio(esp, pata, retro)
    except ErrorESP32 as e:
        esp.soltar()
        esp.cerrar()
        sys.exit(f"No se mueve nada: {e}")
    ch = pata.canales[2]
    if not a.simulado:
        print(f"\nLa pata está en {', '.join(f'{g:.0f}°' for g in actual)}. Primero va despacio (40 °/s) a la pose de inicio")
        print("(rodilla casi recta). ANTES: con los servos sueltos, dobla y estira la rodilla con la mano por todo")
        print("el recorrido y revisa que el pie no le pegue a nada (cuerpo, soporte, cables, la otra pata).")
        print("Ctrl+C detiene todo y suelta los servos.")
        input("Enter para empezar (Ctrl+C para salir) ")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = os.path.join(a.salida, stamp + ("_sim" if a.simulado else ""))
    os.makedirs(carpeta, exist_ok=True)
    us_a, us_b = p["us_ext"][2], p["us_flex"][2]
    meta = {"fecha": stamp, "prueba": "08_flexion", "pata": pata.nombre, "pata_json": pata.fuente, "canal": ch,
            "cadera_q": p["cadera_q"], "ext_q": p["ext_q"], "flex_q": p["flex_q"], "us_ext": us_a, "us_flex": us_b,
            "pose_inicio_us": p["us_ext"], "velocidades": p["vels"], "vueltas": a.vueltas, "espera_ms": a.espera_ms,
            "margen": a.margen, "voltaje": a.voltaje, "simulado": a.simulado, "esp32": esp.info,
            "escala": esc.fuente, "us_por_grado_real": esc.us_por_grado, "calibracion": retro.fuente,
            "inicio_grados": actual}
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    f_m = open(os.path.join(carpeta, "muestras.csv"), "w", newline="", encoding="utf-8")
    f_c = open(os.path.join(carpeta, "comandos.csv"), "w", newline="", encoding="utf-8")
    f_k = open(os.path.join(carpeta, "corridas.csv"), "w", newline="", encoding="utf-8")
    w_m, w_c, w_k = csv.writer(f_m), csv.writer(f_c), csv.writer(f_k)
    w_m.writerow(["velocidad", "t_s", "mv"])
    w_c.writerow(["velocidad", "t_s", "us"])
    w_k.writerow(["velocidad", "vel_pedida", "vel_us_s", "us_ext", "us_flex", "vueltas", "espera_ms", "total_s",
                  "n_muestras", "n_comandos", "lineas_perdidas", "estado"])
    detenida = None
    try:
        desde = [esc.real_a_us(g) for g in actual]
        acercar(esp, pata.canales, desde, p["us_ext"], esc)
        for nombre, v, vel_us in p["vels"]:
            print(f"\n— {'máxima, ' if nombre == 'max' else ''}{v:.0f} °/s ({vel_us} µs/s)")
            esp.mover(ch, us_a)
            esp.esperar(1.0)
            r = esp.rampa(ch, us_a, us_b, vel_us, a.vueltas, a.espera_ms, tolerancia=TOLERANCIA)
            perd = r.get("perdidas", {})
            n_perd = perd.get("M", 0) + perd.get("C", 0)
            if n_perd or perd.get("encabezado"):
                print(f"  ⚠ se perdieron en el USB {perd['M']} de {len(r['M']) + perd['M']} muestras y {perd['C']} de "
                      f"{len(r['C']) + perd['C']} comandos ({perd['danadas']} líneas dañadas, {perd['raras']} raras"
                      f"{', y el encabezado G DATOS' if perd.get('encabezado') else ''}); es menos del 1 %: "
                      "se analiza igual")
            for t, mv in r["M"]:
                w_m.writerow([nombre, t, mv])
            for t, us in r["C"]:
                w_c.writerow([nombre, t, us])
            ms = metricas(np.array(r["M"], dtype=float), np.array(r["C"], dtype=float), us_a, us_b, vel_us,
                          a.vueltas, a.espera_ms, ch, esc, retro)
            llego = bool(ms) and all(x["llego"] for x in ms)
            w_k.writerow([nombre, round(v, 1), vel_us, us_a, us_b, a.vueltas, a.espera_ms, round(r["total_s"], 3),
                          len(r["M"]), len(r["C"]), n_perd, "ok" if llego else "no llegó"])
            for f in (f_m, f_c, f_k):
                f.flush()
            lat = [x["latencia_ms"] for x in ms if math.isfinite(x.get("latencia_ms", float("nan")))]
            pico = [x["vel_pico"] for x in ms if math.isfinite(x.get("vel_pico", float("nan")))]
            print(f"  latencia {np.mean(lat):.0f} ms · velocidad pico {max(pico):.0f} °/s" if lat and pico else
                  "  (sin métricas)")
            if not llego:
                detenida = (f"a {v:.0f} °/s la rodilla no llegó a uno de los extremos (más de 6° de diferencia). "
                            "¿Le pega a algo? ¿Le falta fuerza a la fuente? Me detengo y suelto los servos.")
                print("  ⛔ " + detenida)
                break
    except KeyboardInterrupt:
        detenida = "Ctrl+C"
        print("\nDetenido con Ctrl+C.")
    except ErrorESP32 as e:
        detenida = str(e)
        print(f"\n⛔ {e}")
    finally:
        try:
            solto = esp.soltar()
        except Exception:                          # noqa: BLE001 — soltar es lo último; que no tape el error
            solto = False
        if solto is False:
            print("⚠ La ESP32 NO confirmó que solto los servos (D). Desconecta la fuente de 6 V.")
        else:
            print("Servos sueltos (la ESP32 contestó OK D).")
        if detenida and detenida != "Ctrl+C" and not a.simulado:
            meta["esp32_al_final"] = estado_al_final(esp, t_abierto, esp.info.get("seg"))
        esp.cerrar()
        for f in (f_m, f_c, f_k):
            f.close()
    if detenida:
        meta["detenida"] = detenida
        guardar_json(os.path.join(carpeta, "meta.json"), meta)
    return carpeta


def estado_al_final(esp, t_abierto, seg0):
    """Después de una falla: un '?' para ver si la ESP32 se reinició y qué contó (firmware con DIAG).
    Los MAX son desde el '?' del principio, o sea, de toda la prueba."""
    try:
        info = esp.estado()
    except Exception as e:                         # noqa: BLE001
        print(f"  La ESP32 tampoco contesta a '?' ({e}): se colgó o se cortó el USB. Aprieta EN en la ESP32.")
        return {"contesta": False}
    print(f"  ESP32 al final: {info['linea']}")
    lleva = time.time() - t_abierto
    seg = info.get("seg")
    esperado = (seg0 if isinstance(seg0, int) else 0) + lleva    # si no se hubiera reiniciado
    if isinstance(seg, int) and seg + 3 < esperado:
        print(f"  → La ESP32 SE REINICIÓ durante la prueba, hace {seg} s (RESET={info.get('reset', '?')}; "
              "voltaje_bajo = se cayó su alimentación).")
    elif isinstance(seg, int):
        print(f"  → La ESP32 no se reinició (lleva {seg} s prendida; al empezar llevaba {seg0}).")
    l = info["linea"]
    campos = {k: int(m.group(1)) for k in ("I2CERR", "I2CCOD", "I2CMAX", "VUELTAMAX", "LINMAX")
              for m in [re.search(r"\b" + k + r"=(\d+)", l)] if m}
    if "I2CMAX" in campos:
        print(f"  → I²C: {campos['I2CERR']} fallas desde que arrancó (última: código {campos['I2CCOD']}), la "
              f"escritura más lenta {campos['I2CMAX'] / 1000:.1f} ms; la vuelta de loop() más larga "
              f"{campos['VUELTAMAX'] / 1000:.1f} ms.")
    return {"contesta": True, **{k: v for k, v in info.items() if k != "linea"}, "linea": l}


def revisar(a, pata, esc, retro, p):
    imprimir_plan(p, pata, esc, a)
    esp = abrir(a, esc, retro, pata, p)
    try:
        print(f"\nESP32: {esp.info['linea']}")
        v = esp.leer_todos()
        print(f"\n  {'':10} {'mV':>6} {'servo':>8} {'articulación':>13}   límites del servo")
        for i, (x, c) in enumerate(zip(ART, pata.canales)):
            mv = v[c]
            ok = mv is not None and MV_MIN <= mv <= MV_MAX
            g = retro.grados(c, mv) if ok else None
            q = (g - pata.cero[x]) / pata.signo[x] if ok else None
            lo, hi = pata.lim[x]
            est = "" if ok and lo <= g <= hi else ("  ⚠ fuera de límites" if ok else "  ⚠ sin lectura creíble")
            print(f"  {NOMBRES[x]:<10} {mv if mv is not None else '—':>6} {fmt(g):>7}° {fmt(q):>12}°   "
                  f"{lo:.1f}–{hi:.1f}°{est}")
        try:
            revisar_inicio(esp, pata, retro)
            print("\n  La pata está adentro de sus límites: la prueba puede empezar.")
        except ErrorESP32 as e:
            print(f"\n  ⚠ {e}")
        print("  No se movió nada.")
    finally:
        esp.cerrar()


def main():
    ap = argparse.ArgumentParser(description="Prueba 08: flexión y extensión completas de la rodilla")
    ap.add_argument("--port", help="puerto de la ESP32 principal (COM6, /dev/ttyUSB0)")
    ap.add_argument("--pata", default="fl", choices=["fl", "fr", "rl", "rr"], help="qué pata (default fl)")
    ap.add_argument("--velocidades", nargs="+", default=VELOCIDADES,
                    help="°/s reales, o 'max' (G a 4000 µs/s); van de menor a mayor")
    ap.add_argument("--cadera", type=float, default=0.0, help="ángulo del muslo (0 = vertical, + = adelante)")
    ap.add_argument("--margen", type=float, default=MARGEN, help="° extra en cada extremo de la rodilla")
    ap.add_argument("--vueltas", type=int, default=VUELTAS)
    ap.add_argument("--espera-ms", type=int, default=ESPERA_MS, help="ms quieta en cada extremo")
    ap.add_argument("--voltaje", type=float, help="voltaje de la fuente de los servos (para el registro)")
    ap.add_argument("--revisar", action="store_true", help="dónde está la pata y el plan; no mueve nada")
    ap.add_argument("--simulado", action="store_true", help="sin hardware")
    ap.add_argument("--semilla", type=int, default=8)
    ap.add_argument("--reanalizar", metavar="RUTA", help="carpeta de una corrida o 'ultima'")
    ap.add_argument("--salida", default=os.path.join(HERE, "resultados_flexion"))
    a = ap.parse_args()
    if not 1 <= a.vueltas <= 5 or not 200 <= a.espera_ms <= 3000:
        ap.error("--vueltas de 1 a 5 y --espera-ms de 200 a 3000")
    if not 0 <= a.margen <= 20:
        ap.error("--margen de 0 a 20")

    esc = Escala()
    retro = Retro(esc)
    ruta = buscar_pata(f"pata_{a.pata}.json")
    if ruta is None:
        sys.exit(f"No hay datos/*/pata_{a.pata}.json: esa pata no está medida (nota 07).")
    pata = Pata(ruta)
    print(f"Pata: {pata.fuente} · escala: {esc.fuente} · calibración: {retro.fuente}")

    if a.reanalizar:
        carpeta = a.reanalizar
        if carpeta == "ultima":
            cands = sorted(d for d in (os.listdir(a.salida) if os.path.isdir(a.salida) else [])
                           if os.path.isfile(os.path.join(a.salida, d, "muestras.csv")))
            if not cands:
                sys.exit(f"No hay corridas en {a.salida}")
            carpeta = os.path.join(a.salida, cands[-1])
        imprimir(analizar(carpeta, esc, retro, pata), carpeta)
        return
    try:
        p = plan(a, pata, esc)
    except ValueError as e:
        sys.exit(f"No se puede: {e}")
    if not a.simulado and not a.port:
        ap.error("falta --port (o usa --simulado)")
    if a.revisar:
        revisar(a, pata, esc, retro, p)
        return
    carpeta = medir(a, pata, esc, retro, p)
    imprimir(analizar(carpeta, esc, retro, pata), carpeta)


if __name__ == "__main__":
    main()
