#!/usr/bin/env python3
"""
Prueba 03 · Ruido de los cables verdes con el servo quieto — Arca9
===================================================================
En la prueba 01 los servos que lee la ESP32 auxiliar (FR, RL y RR) tuvieron el doble de ruido que
los de la principal (FL): 0.4–0.7° contra 0.3°. La causa más probable es la tierra o la
alimentación de la auxiliar. Este script mide el ruido de los 12 con los servos quietos, para
correrlo ANTES y DESPUÉS de cambiar la tierra y comparar.

Mide dos cosas en cada canal:
  - R (como la prueba 01): `R canal 200` varias veces; σ de 200 lecturas sueltas del ADC.
  - V (lo que usará la marcha): muchas veces `V`, que da una lectura de los 12 (mediana de 5 en
    cada placa); σ de esa serie. Es el ruido que verá el puente con lecturas sueltas.
Todo en mV del pin (la medida directa) y en grados reales (con pc/escala.py).

Hardware: ESP32 principal con esp32_principal.ino y auxiliar con esp32_aux.ino. Usa P, R, V, D y
? (docs/hardware.md); no cambia el protocolo.

Uso (en la carpeta de este archivo):
  python ruido.py --simulado                               # practicar sin hardware
  python ruido.py --port COM6 --revisar                    # σ de los 12 en vivo
  python ruido.py --port COM6 --etiqueta antes --voltaje 5.99
  python ruido.py --port COM6 --etiqueta despues --voltaje 5.99
  python ruido.py --comparar RUTA_ANTES RUTA_DESPUES       # tabla y gráfica antes/después
  python ruido.py --port COM6 --sueltos --etiqueta antes_sueltos   # con los servos sin fuerza

Por defecto los 12 servos se van uno por uno, despacio, a 1500 µs y se quedan con fuerza
(así es como van a estar en la marcha). Con --sueltos no se mueven: se manda D y se mide así.
Comparar las dos dice si el ruido viene de la corriente de los servos.

CUIDADO: con los servos en las patas, 1500 µs puede chocar; usa --pulso o --sueltos. Al terminar
se manda D: si el perro está parado, se cae.

Resultados en resultados_ruido/AAAAMMDD_HHMMSS_<etiqueta>/:
  meta.json, lecturas_R.csv, lecturas_V.csv   la corrida tal cual
  resumen.csv, resumen.md                     σ por canal, por placa, y contra la prueba 01
  ruido.png                                   σ de R y de V por canal
"""
import argparse
import csv
import datetime
import json
import math
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)          # para encontrar arca9_serial.py y escala.py desde cualquier carpeta
from arca9_serial import ESP32, ErrorESP32
from escala import Escala, Retro, US_CERO, US_LIMITE, US_POR_GRADO_CONV, N_SERVOS
PATAS = ["FL", "FR", "RL", "RR"]
CORTO = ["abd", "cad", "rod"]
PLACA = {ch: ("principal" if ch < 3 else "auxiliar") for ch in range(N_SERVOS)}
MEJORA_OK = 1.3        # aux/principal por debajo de esto = "ya mide como la principal"

SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
AZUL, NARANJA = "#2a78d6", "#eb6834"

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


def nombre_corto(ch):
    return f"{PATAS[ch // 3]} {CORTO[ch % 3]}"


def fmt(x, d=2):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    return f"{x:.{d}f}"


def sd(xs):
    xs = [x for x in xs if x is not None]
    return statistics.stdev(xs) if len(xs) >= 2 else float("nan")


def media(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return statistics.mean(xs) if xs else float("nan")


def guardar_json(ruta, obj):
    def limpio(x):
        if isinstance(x, dict):
            return {str(k): limpio(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [limpio(v) for v in x]
        if isinstance(x, float):
            return round(x, 5) if math.isfinite(x) else None
        return x
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(limpio(obj), f, ensure_ascii=False, indent=2)


def conv_a_us(g):
    return int(round(US_CERO + g * US_POR_GRADO_CONV))


def us_a_conv(us):
    return (us - US_CERO) / US_POR_GRADO_CONV


# ======================================================================= simulador

class ESP32Simulado:
    """12 servos quietos. Ruido en el pin: ~3 mV en FL (principal) y `ruido_aux` mV en los de la
    auxiliar, como en la prueba 01. V da la mediana de 5 lecturas, como el firmware."""

    def __init__(self, retro, ruido_aux=6.0, semilla=3):
        import random
        self.rng = random.Random(semilla)
        self.retro = retro
        self.ruido = {ch: (3.0 if ch < 3 else ruido_aux) * (0.9 + 0.2 * self.rng.random())
                      for ch in range(N_SERVOS)}
        self.pos = {ch: 40 + 100 * self.rng.random() for ch in range(N_SERVOS)}   # grados de la convención
        self.fuerza = False
        self.info = {"linea": "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 (SIMULADO)", "pca": "ok",
                     "fb": 12, "aux": "ok"}

    def _mv(self, ch):
        c = self.retro.canales[ch]
        base = (self.pos[ch] - c["b"]) / c["a"]
        s = self.ruido[ch] * (1.0 if self.fuerza else 0.8)
        return round(base + self.rng.gauss(0, s))

    def mover(self, ch, us):
        us = int(min(max(us, US_CERO), US_LIMITE))
        self.pos[ch] = us_a_conv(us)
        self.fuerza = True
        return us

    def leer(self, ch, n):
        x = [self._mv(ch) for _ in range(n)]
        m = statistics.mean(x)
        return {"media_mv": m, "sigma_mv": statistics.pstdev(x), "min_mv": min(x), "max_mv": max(x), "n": n}

    def leer_todos(self):
        return [sorted(self._mv(ch) for _ in range(5))[2] for ch in range(N_SERVOS)]

    def soltar(self):
        self.fuerza = False

    def esperar(self, s):
        pass

    def cerrar(self):
        pass


# ======================================================================= preparar los servos

def revisar_inicio(esp, canales):
    if esp.info.get("pca") == "NO":
        raise ErrorESP32("la ESP32 no encuentra el PCA9685 (PCA=NO). Revisa SDA 21, SCL 22, VCC 3V3 y GND.")
    if any(c >= 3 for c in canales) and esp.info.get("aux") == "NO":
        raise ErrorESP32("la auxiliar no está mandando datos (AUX=NO). Revisa TX/RX cruzados (16 <-> 17), "
                         "GND común y que tenga esp32_aux.ino.")
    v = esp.leer_todos()
    malos = [c for c in canales if v[c] is None or not 60 <= v[c] <= 2500]
    if malos:
        raise ErrorESP32("cables verdes fuera de rango (lo normal es 190–1870 mV): "
                         + ", ".join(f"{c} ({nombre_corto(c)}) = {v[c]}" for c in malos))
    return v


def llevar_a_pulso(esp, retro, canales, v_inicial, us_obj, vel=60.0, dt=0.03):
    """Uno por uno, despacio, desde donde dice su cable verde hasta us_obj (y se queda con fuerza)."""
    obj = us_a_conv(us_obj)
    for ch in canales:
        desde = min(max(retro.grados_conv(ch, v_inicial[ch]), 0.0), us_a_conv(US_LIMITE))
        pasos = max(1, int(math.ceil(abs(obj - desde) / (vel * dt))))
        for k in range(1, pasos + 1):
            esp.mover(ch, conv_a_us(desde + (obj - desde) * k / pasos))
            esp.esperar(dt)
        esp.mover(ch, us_obj)


# ======================================================================= medición

def medir(a, retro):
    esp = ESP32Simulado(retro, a.sim_ruido_aux, a.semilla) if a.simulado else ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    try:
        v0 = revisar_inicio(esp, a.canal)
    except ErrorESP32 as e:
        esp.cerrar()
        sys.exit(f"Error: {e}")

    modo = "sueltos (sin fuerza)" if a.sueltos else f"con fuerza en {a.pulso} µs"
    t_est = len(a.canal) * a.repeticiones * (a.muestras * 0.0006 + 0.02) + a.v_lecturas * 0.008
    print(f"\nPRUEBA 03 · RUIDO — {len(a.canal)} canales, servos {modo}")
    print(f"  R: {a.repeticiones} × `R canal {a.muestras}` por canal · V: {a.v_lecturas} lecturas de los 12")
    print(f"  Tiempo estimado: ~{t_est:.0f} s (más ~{len(a.canal) * 1.5:.0f} s para llevar los servos)")
    if not a.simulado:
        if not a.sueltos:
            print("\nLos servos se van a mover uno por uno a", a.pulso, "µs. Que nada choque.")
        input("Enter para empezar (Ctrl+C para salir) ")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    etiqueta = a.etiqueta or ("sueltos" if a.sueltos else "con_fuerza")
    carpeta = os.path.join(a.salida, f"{stamp}_{etiqueta}" + ("_sim" if a.simulado else ""))
    os.makedirs(carpeta, exist_ok=True)
    meta = {"fecha": stamp, "prueba": "03_ruido", "etiqueta": etiqueta, "canales": a.canal,
            "sueltos": a.sueltos, "pulso_us": None if a.sueltos else a.pulso, "muestras": a.muestras,
            "repeticiones": a.repeticiones, "v_lecturas": a.v_lecturas, "voltaje": a.voltaje,
            "simulado": a.simulado, "esp32": esp.info, "escala": retro.escala.fuente,
            "calibracion": retro.fuente, "lectura_inicial_mv": {str(c): v0[c] for c in a.canal},
            "nota": a.nota}
    if a.simulado:
        meta["simulador"] = {"ruido_aux_mv": a.sim_ruido_aux}
    guardar_json(os.path.join(carpeta, "meta.json"), meta)

    fr = open(os.path.join(carpeta, "lecturas_R.csv"), "w", newline="", encoding="utf-8")
    fv = open(os.path.join(carpeta, "lecturas_V.csv"), "w", newline="", encoding="utf-8")
    wr = csv.writer(fr)
    wr.writerow(["repeticion", "canal", "media_mv", "sigma_mv", "min_mv", "max_mv", "n", "t_s"])
    wv = csv.writer(fv)
    wv.writerow(["t_s"] + [f"mv{c}" for c in range(N_SERVOS)])
    t0 = time.time()
    cancelado = False
    try:
        if a.sueltos:
            esp.soltar()
            esp.esperar(0.5)
        else:
            print("Llevando los servos a", a.pulso, "µs...")
            llevar_a_pulso(esp, retro, a.canal, v0, a.pulso)
            esp.esperar(1.5)
        for rep in range(a.repeticiones):
            for ch in a.canal:
                try:
                    r = esp.leer(ch, a.muestras)
                except ErrorESP32 as e:
                    print(f"\n  AVISO: canal {ch} sin lectura: {e}")
                    continue
                wr.writerow([rep + 1, ch, round(r["media_mv"], 2), round(r["sigma_mv"], 3), r["min_mv"],
                             r["max_mv"], r["n"], round(time.time() - t0, 3)])
            fr.flush()
            print(f"\r  R: repetición {rep + 1}/{a.repeticiones}", end="", flush=True)
        print()
        for k in range(a.v_lecturas):
            try:
                v = esp.leer_todos()
            except ErrorESP32:
                continue
            wv.writerow([round(time.time() - t0, 4)] + ["" if x is None else x for x in v])
            if k % 50 == 0:
                print(f"\r  V: {k}/{a.v_lecturas}", end="", flush=True)
        print(f"\r  V: {a.v_lecturas}/{a.v_lecturas}")
    except KeyboardInterrupt:
        cancelado = True
        print("\n\nDetenido. Se analiza lo que se alcanzó a medir.")
    finally:
        esp.soltar()
        esp.cerrar()
        fr.close()
        fv.close()
    meta["cancelado"] = cancelado
    meta["duracion_s"] = round(time.time() - t0, 1)
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    return carpeta


# ======================================================================= análisis

def analizar(carpeta, retro):
    with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    r_por_canal, v_por_canal = {}, {c: [] for c in range(N_SERVOS)}
    with open(os.path.join(carpeta, "lecturas_R.csv"), newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            r_por_canal.setdefault(int(r["canal"]), []).append((float(r["media_mv"]), float(r["sigma_mv"])))
    t_v = []
    with open(os.path.join(carpeta, "lecturas_V.csv"), newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            t_v.append(float(r["t_s"]))
            for c in range(N_SERVOS):
                x = r.get(f"mv{c}", "")
                if x != "":
                    v_por_canal[c].append(float(x))
    tasa_v = (len(t_v) - 1) / (t_v[-1] - t_v[0]) if len(t_v) > 1 and t_v[-1] > t_v[0] else float("nan")

    filas = []
    for ch in meta["canales"]:
        g = retro.grados_por_mv(ch)
        rr = r_por_canal.get(ch, [])
        sig_r = statistics.median([s for _, s in rr]) if rr else float("nan")
        deriva = sd([m for m, _ in rr]) if len(rr) >= 2 else float("nan")
        vv = v_por_canal.get(ch, [])
        sig_v = sd(vv)
        p01 = retro.canales[ch].get("ruido_mv")
        filas.append({
            "canal": ch, "nombre": retro.nombre(ch), "placa": PLACA[ch],
            "sigma_R_mv": sig_r, "sigma_R_grados": sig_r * g,
            "sigma_V_mv": sig_v, "sigma_V_grados": sig_v * g,
            "pico_pico_V_mv": (max(vv) - min(vv)) if vv else float("nan"),
            "deriva_R_mv": deriva, "media_mv": media([m for m, _ in rr]),
            "prueba01_sigma_mv": p01 if p01 is not None else float("nan"),
            "grados_por_mv": g, "n_V": len(vv),
        })

    def por_placa(campo):
        out = {}
        for placa in ("principal", "auxiliar"):
            xs = [f[campo] for f in filas if f["placa"] == placa and math.isfinite(f[campo])]
            out[placa] = statistics.mean(xs) if xs else float("nan")
        p, x = out["principal"], out["auxiliar"]
        out["aux_entre_principal"] = x / p if math.isfinite(p) and math.isfinite(x) and p > 0 else float("nan")
        return out

    resumen = {"R_mv": por_placa("sigma_R_mv"), "R_grados": por_placa("sigma_R_grados"),
               "V_mv": por_placa("sigma_V_mv"), "V_grados": por_placa("sigma_V_grados"),
               "prueba01_mv": por_placa("prueba01_sigma_mv"), "tasa_V_hz": tasa_v}
    rel = resumen["R_mv"]["aux_entre_principal"]
    if not math.isfinite(rel):
        veredicto = "Hacen falta canales de las dos placas para comparar la auxiliar con la principal."
    elif rel <= MEJORA_OK:
        veredicto = (f"La auxiliar ya mide casi como la principal: {rel:.2f} veces su ruido con R "
                     f"(en la prueba 01 era {resumen['prueba01_mv']['aux_entre_principal']:.2f}).")
    else:
        veredicto = (f"La auxiliar sigue con {rel:.2f} veces el ruido de la principal con R "
                     f"(en la prueba 01: {resumen['prueba01_mv']['aux_entre_principal']:.2f}).")
    if meta.get("cancelado"):
        veredicto = "CORRIDA INCOMPLETA: repítela. Con lo que hay: " + veredicto

    with open(os.path.join(carpeta, "resumen.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0].keys()))
        w.writeheader()
        for fila in filas:
            w.writerow({k: (round(v, 4) if isinstance(v, float) and math.isfinite(v) else
                            ("" if isinstance(v, float) else v)) for k, v in fila.items()})
    guardar_json(os.path.join(carpeta, "resultados.json"),
                 {"meta": meta, "resumen": resumen, "veredicto": veredicto, "canales": filas})
    escribir_md(carpeta, meta, filas, resumen, veredicto)
    grafica(carpeta, meta, filas)
    imprimir(meta, filas, resumen, veredicto, carpeta)


def modo_txt(meta):
    return "sueltos" if meta.get("sueltos") else f"con fuerza en {meta.get('pulso_us')} µs"


def escribir_md(carpeta, meta, filas, resumen, veredicto):
    L = [f"# 03 · Ruido — {meta.get('etiqueta')}", "",
         "| | |", "|---|---|",
         f"| Fecha | {meta.get('fecha')} |",
         f"| Servos | {modo_txt(meta)} |",
         f"| Fuente | {fmt(meta.get('voltaje')) if meta.get('voltaje') else '—'} V |",
         f"| R | {meta.get('repeticiones')} × `R canal {meta.get('muestras')}` por canal (σ: mediana de las repeticiones) |",
         f"| V | {meta.get('v_lecturas')} lecturas de los 12, a ~{fmt(resumen['tasa_V_hz'], 0)} por segundo |",
         f"| Simulado | {'sí' if meta.get('simulado') else 'no'} |",
         f"| Nota | {meta.get('nota') or '—'} |", "",
         "| Servo | Placa | σ R (mV) | σ R (°) | Prueba 01 (mV) | σ V (mV) | σ V (°) | Pico a pico V (mV) |",
         "|---|---|---|---|---|---|---|---|"]
    for f in filas:
        L.append(f"| {f['canal']} · {f['nombre']} | {f['placa']} | {fmt(f['sigma_R_mv'])} | "
                 f"{fmt(f['sigma_R_grados'])} | {fmt(f['prueba01_sigma_mv'])} | {fmt(f['sigma_V_mv'])} | "
                 f"{fmt(f['sigma_V_grados'])} | {fmt(f['pico_pico_V_mv'], 0)} |")
    L += ["", "Grados reales (factor de la prueba 02). La prueba 01 reportó su ruido en grados de la "
          "convención; aquí se compara en mV.", "",
          "| Promedio | Principal (FL) | Auxiliar (FR, RL, RR) | Auxiliar / principal |", "|---|---|---|---|"]
    for clave, txt in (("R_mv", "σ R (mV)"), ("V_mv", "σ V (mV)"), ("prueba01_mv", "Prueba 01, σ R (mV)")):
        x = resumen[clave]
        L.append(f"| {txt} | {fmt(x['principal'])} | {fmt(x['auxiliar'])} | {fmt(x['aux_entre_principal'])} |")
    L += ["", f"**Veredicto:** {veredicto}", ""]
    with open(os.path.join(carpeta, "resumen.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def _plt():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("AVISO: falta matplotlib; no se hizo la gráfica (pip install -r requirements.txt).")
        return None
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": SURF, "axes.facecolor": SURF})
    return plt


def grafica(carpeta, meta, filas):
    plt = _plt()
    if plt is None:
        return
    xs = list(range(len(filas)))
    colores = [AZUL if f["placa"] == "principal" else NARANJA for f in filas]
    etiquetas = [f"{f['canal']}\n{nombre_corto(f['canal'])}" for f in filas]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    a1.bar(xs, [f["sigma_R_mv"] for f in filas], color=colores, width=0.6, zorder=3)
    a1.scatter(xs, [f["prueba01_sigma_mv"] for f in filas], marker="_", s=260, color=INK, zorder=4,
               label="prueba 01")
    a1.set_ylabel("σ con R (mV)")
    a1.legend(frameon=False, fontsize=8, loc="upper left")
    a2.bar(xs, [f["sigma_V_mv"] for f in filas], color=colores, width=0.6, zorder=3)
    a2.set_ylabel("σ con V (mV)")
    a2.set_xticks(xs)
    a2.set_xticklabels(etiquetas, fontsize=8)
    for ax in (a1, a2):
        ax.grid(True, axis="y", color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    a1.set_title("azul: principal (FL) · naranja: auxiliar (FR, RL, RR) · 1 mV ≈ 0.10° reales",
                 loc="left", fontsize=8, color=INK2)
    fig.suptitle(f"Ruido con el servo quieto · {meta.get('etiqueta')} · {modo_txt(meta)}"
                 + (" · SIMULADO" if meta.get("simulado") else ""),
                 x=0.07, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "ruido.png"), dpi=150)
    plt.close(fig)


def imprimir(meta, filas, resumen, veredicto, carpeta):
    print("\n" + "=" * 76)
    print(f"RESULTADOS — {meta.get('etiqueta')} · servos {modo_txt(meta)}")
    print("=" * 76)
    print("  canal         placa       σR mV   σR °   prueba01 mV   σV mV   σV °")
    for f in filas:
        print(f"  {f['canal']:>2} {nombre_corto(f['canal']):<8}  {f['placa']:<10} {fmt(f['sigma_R_mv']):>6} "
              f"{fmt(f['sigma_R_grados']):>6}   {fmt(f['prueba01_sigma_mv']):>8}    {fmt(f['sigma_V_mv']):>6} "
              f"{fmt(f['sigma_V_grados']):>6}")
    for clave, txt in (("R_mv", "σ R"), ("V_mv", "σ V")):
        x = resumen[clave]
        print(f"\n  {txt}: principal {fmt(x['principal'])} mV · auxiliar {fmt(x['auxiliar'])} mV · "
              f"auxiliar/principal = {fmt(x['aux_entre_principal'])}")
    print(f"\n{veredicto}")
    print(f"\nResultados en {carpeta}")


# ======================================================================= comparar antes / después

def comparar(ruta_a, ruta_b, salida):
    def cargar(ruta):
        if os.path.isfile(ruta):
            ruta = os.path.dirname(os.path.abspath(ruta))
        if not os.path.isfile(os.path.join(ruta, "resultados.json")):
            sys.exit(f"No encuentro resultados.json en {ruta}")
        with open(os.path.join(ruta, "resultados.json"), encoding="utf-8") as f:
            return ruta, json.load(f)
    ra, A = cargar(ruta_a)
    rb, B = cargar(ruta_b)
    ea, eb = A["meta"]["etiqueta"], B["meta"]["etiqueta"]
    ca = {f["canal"]: f for f in A["canales"]}
    cb = {f["canal"]: f for f in B["canales"]}
    canales = sorted(set(ca) & set(cb))
    carpeta = os.path.join(salida, f"comparacion_{os.path.basename(ra)}__vs__{os.path.basename(rb)}")
    os.makedirs(carpeta, exist_ok=True)

    def cambio(x, y):
        return (y / x - 1) * 100 if x and y and x > 0 else float("nan")
    L = [f"# 03 · Ruido: {ea} contra {eb}", "",
         f"- **{ea}:** `{os.path.basename(ra)}` · servos {modo_txt(A['meta'])}",
         f"- **{eb}:** `{os.path.basename(rb)}` · servos {modo_txt(B['meta'])}", "",
         f"| Servo | Placa | σ R {ea} (mV) | σ R {eb} (mV) | Cambio | σ V {ea} (mV) | σ V {eb} (mV) | Cambio |",
         "|---|---|---|---|---|---|---|---|"]
    for c in canales:
        a, b = ca[c], cb[c]
        L.append(f"| {c} · {a['nombre']} | {a['placa']} | {fmt(a['sigma_R_mv'])} | {fmt(b['sigma_R_mv'])} | "
                 f"{fmt(cambio(a['sigma_R_mv'], b['sigma_R_mv']), 0)} % | {fmt(a['sigma_V_mv'])} | "
                 f"{fmt(b['sigma_V_mv'])} | {fmt(cambio(a['sigma_V_mv'], b['sigma_V_mv']), 0)} % |")
    L += ["", "| Promedio | " + f"{ea} | {eb} | Cambio |", "|---|---|---|---|"]
    for clave, txt in (("R_mv", "σ R"), ("V_mv", "σ V")):
        for placa in ("principal", "auxiliar"):
            x, y = A["resumen"][clave][placa], B["resumen"][clave][placa]
            L.append(f"| {txt}, {placa} (mV) | {fmt(x)} | {fmt(y)} | {fmt(cambio(x, y), 0)} % |")
        x, y = A["resumen"][clave]["aux_entre_principal"], B["resumen"][clave]["aux_entre_principal"]
        L.append(f"| {txt}, auxiliar / principal | {fmt(x)} | {fmt(y)} | |")
    L += ["", f"**{ea}:** {A['veredicto']}", "", f"**{eb}:** {B['veredicto']}", ""]
    with open(os.path.join(carpeta, "comparacion.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))

    plt = _plt()
    if plt is not None:
        xs = list(range(len(canales)))
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        for ax, campo, txt in ((a1, "sigma_R_mv", "σ con R (mV)"), (a2, "sigma_V_mv", "σ con V (mV)")):
            ax.bar([x - 0.2 for x in xs], [ca[c][campo] for c in canales], width=0.38, color=MUTED,
                   label=ea, zorder=3)
            ax.bar([x + 0.2 for x in xs], [cb[c][campo] for c in canales], width=0.38, color=AZUL,
                   label=eb, zorder=3)
            ax.set_ylabel(txt)
            ax.grid(True, axis="y", color=GRID, lw=0.6)
            ax.set_axisbelow(True)
        a1.legend(frameon=False, fontsize=8, loc="upper left")
        a2.set_xticks(xs)
        a2.set_xticklabels([f"{c}\n{nombre_corto(c)}" for c in canales], fontsize=8)
        fig.suptitle(f"Ruido: {ea} contra {eb}", x=0.07, ha="left", fontsize=11, fontweight="bold", color=INK)
        fig.tight_layout()
        fig.savefig(os.path.join(carpeta, "comparacion.png"), dpi=150)
        plt.close(fig)
    print("\n".join(L))
    print(f"\nComparación en {carpeta}")


# ======================================================================= revisar en vivo

def revisar(a, retro):
    esp = ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    try:
        v0 = revisar_inicio(esp, a.canal)
        if a.sueltos:
            esp.soltar()
        else:
            print(f"Llevando los servos a {a.pulso} µs, uno por uno...")
            llevar_a_pulso(esp, retro, a.canal, v0, a.pulso)
        print("\nσ de las últimas 100 lecturas V, en mV del pin. Ctrl+C para salir (se sueltan los servos).\n")
        print("  " + "".join(f"{nombre_corto(c):>8}" for c in a.canal))
        hist = {c: [] for c in a.canal}
        k = 0
        while True:
            try:
                v = esp.leer_todos()
            except ErrorESP32:
                continue
            for c in a.canal:
                if v[c] is not None:
                    hist[c] = (hist[c] + [v[c]])[-100:]
            k += 1
            if k % 10 == 0:
                print("\r  " + "".join(f"{fmt(sd(hist[c]), 1):>8}" for c in a.canal), end="", flush=True)
    except KeyboardInterrupt:
        print()
    except ErrorESP32 as e:
        print(f"\nError: {e}")
    finally:
        esp.soltar()
        esp.cerrar()


# ======================================================================= principal

def main():
    ap = argparse.ArgumentParser(description="Prueba 03 de Arca9: ruido de los cables verdes con el servo quieto.")
    ap.add_argument("--port", help="puerto de la ESP32 principal (COM6, /dev/ttyUSB0)")
    ap.add_argument("--canal", type=int, nargs="+", default=list(range(N_SERVOS)), help="canales (por defecto los 12)")
    ap.add_argument("--etiqueta", help="nombre de la corrida, por ejemplo antes o despues")
    ap.add_argument("--nota", help="qué cambiaste en el cableado (se guarda en meta.json)")
    ap.add_argument("--sueltos", action="store_true", help="medir con los servos sin fuerza (no se mueven)")
    ap.add_argument("--pulso", type=int, default=1500, help=f"pulso en que se quedan los servos ({US_CERO}–{US_LIMITE} µs)")
    ap.add_argument("--muestras", type=int, default=200, help="lecturas por `R` (como la prueba 01)")
    ap.add_argument("--repeticiones", type=int, default=5, help="veces que se hace `R` en cada canal")
    ap.add_argument("--v-lecturas", type=int, default=500, help="lecturas `V` de los 12")
    ap.add_argument("--voltaje", type=float, help="voltaje de la fuente de los servos (para el registro)")
    ap.add_argument("--revisar", action="store_true", help="σ de los 12 en vivo, para probar el cableado")
    ap.add_argument("--simulado", action="store_true", help="sin hardware")
    ap.add_argument("--sim-ruido-aux", type=float, default=6.0, help="mV de ruido de la auxiliar en el simulador")
    ap.add_argument("--semilla", type=int, default=3)
    ap.add_argument("--comparar", nargs=2, metavar=("ANTES", "DESPUES"), help="compara dos corridas")
    ap.add_argument("--reanalizar", metavar="RUTA", help="rehace resumen y gráfica de una corrida")
    ap.add_argument("--salida", default=os.path.join(HERE, "resultados_ruido"))
    a = ap.parse_args()

    if a.comparar:
        comparar(a.comparar[0], a.comparar[1], a.salida)
        return
    a.canal = list(dict.fromkeys(a.canal))
    if any(not 0 <= c < N_SERVOS for c in a.canal):
        ap.error("los canales van de 0 a 11")
    if not US_CERO <= a.pulso <= US_LIMITE:
        ap.error(f"--pulso entre {US_CERO} y {US_LIMITE} µs")
    a.muestras = max(1, min(1000, a.muestras))
    a.repeticiones = max(1, a.repeticiones)
    a.v_lecturas = max(2, a.v_lecturas)
    try:
        retro = Retro(Escala())
    except (FileNotFoundError, ValueError, KeyError) as e:
        sys.exit(f"Error: {e}")

    if a.reanalizar:
        ruta = a.reanalizar
        if ruta == "ultima":
            cands = sorted(d for d in (os.listdir(a.salida) if os.path.isdir(a.salida) else [])
                           if os.path.isfile(os.path.join(a.salida, d, "lecturas_R.csv")))
            if not cands:
                sys.exit(f"No hay corridas en {a.salida}")
            ruta = os.path.join(a.salida, cands[-1])
        analizar(ruta, retro)
        return
    if a.simulado:
        if a.revisar:
            ap.error("--revisar necesita el hardware (--port)")
    elif not a.port:
        ap.error("falta --port (o usa --simulado, --comparar o --reanalizar)")
    try:
        if a.revisar:
            revisar(a, retro)
            return
        carpeta = medir(a, retro)
    except ErrorESP32 as e:
        sys.exit(f"Error: {e}")
    except KeyboardInterrupt:
        sys.exit("\nCancelado antes de empezar; no se movió nada.")
    analizar(carpeta, retro)


if __name__ == "__main__":
    main()
