#!/usr/bin/env python3
"""
Prueba 02 · Ángulo real del eje, con transportador — Arca9
===========================================================
Comprueba que los grados de la convención del proyecto (500 µs = 0°, 11.11 µs por grado) son
los que gira el eje de verdad. Mueve un servo a unos pulsos fijos, primero subiendo y luego
bajando. En cada punto espera, lee su cable verde y te pide la marca del transportador impreso
(impresiones/transportador_servo/). Al final calcula el giro real, la escala real del servo
(°/µs) y el error de la convención, y lo compara con el ángulo que da la retroalimentación
usando la tabla de búsqueda de la prueba 01.

Hardware: ESP32 principal con esp32_principal/esp32_principal.ino (límite de 2360 µs) y, si el
canal es de FR, RL o RR, la auxiliar con esp32_aux/esp32_aux.ino. Usa los comandos P, R, D y ?
de docs/hardware.md; no cambia el protocolo.

Uso (en la carpeta de este archivo):
  python angulo_real.py --simulado                       # practicar sin hardware
  python angulo_real.py --port COM6 --revisar            # mV y ángulo del canal 0 en vivo, suelto
  python angulo_real.py --port COM6 --voltaje 5.99       # la prueba, en el canal 0
  python angulo_real.py --port COM6 --canal 3            # la prueba en otro servo
  python angulo_real.py --reanalizar ultima              # rehace los cálculos sin hardware

CUIDADO: el servo recorre de 0° a 167.4°. Hazlo con el servo FUERA de la pata, sujeto por el
cuerpo y con el transportador armado. Nunca se pide más de 2360 µs.

Cómo se calcula:
  - Giro real = marca − marca en 500 µs subiendo (la referencia). El script detecta solo si los
    números del disco suben o bajan al girar, y corrige el paso por 0/360.
  - Ángulo pedido = (pulso − 500) / 11.11, el de la convención.
  - Error de la convención = giro real − ángulo pedido.
  - Escala real = pendiente de la recta giro real contra (pulso − 500), en °/µs. Si la
    convención es buena, da 0.0900 °/µs (11.11 µs/°).
  - Ángulo por retroalimentación = tabla de búsqueda de la prueba 01 aplicada a los mV
    (la toma del calibracion_servos.json más reciente en datos/, o de --calibracion).

Resultados en resultados_angulo_real/AAAAMMDD_HHMMSS_canal<c>/:
  meta.json          parámetros de la corrida y calibración usada
  mediciones.csv     cada punto (se guarda mientras mides: no se pierde si se corta)
  resultados.csv     una fila por pulso: subiendo, bajando, giro real, errores
  resultados.json    lo mismo más la escala y el veredicto
  resumen.md         la tabla de la nota 02, lista para pegar
  angulo_real.png    giro real contra ángulo pedido, y los errores
"""
import argparse
import csv
import datetime
import glob
import json
import math
import os
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(HERE)
US_MIN, US_ESCALA = 500, 2500            # 0° y 180° en la convención del proyecto
US_LIMITE = 2360                         # el firmware no deja pasar de aquí (tope medido: ~2367 µs)
ESCALA_CONV = 180.0 / (US_ESCALA - US_MIN)   # 0.0900 °/µs
PULSOS = [500, 1000, 1500, 2000, 2360]   # los de la nota 02: 0°, 45°, 90°, 135° y 167.4°
TOLERANCIA = 1.0                         # ° de error aceptable para decir que la convención es buena
PATAS = ["FL", "FR", "RL", "RR"]
ARTICULACIONES = ["abducción", "cadera", "rodilla"]
FLECHA = {"sube": "↑", "baja": "↓"}

SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
AZUL, NARANJA, AQUA = "#2a78d6", "#eb6834", "#1baf7a"

COLUMNAS = ["n", "sentido", "pulso_us", "pulso_aplicado_us", "angulo_pedido", "marca",
            "media_mv", "sigma_mv", "min_mv", "max_mv", "muestras", "t_s"]

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# ======================================================================= utilidades

def ang_a_us(ang):
    return int(round(US_MIN + ang / ESCALA_CONV))


def us_a_ang(us):
    return (us - US_MIN) * ESCALA_CONV


def nombre_canal(ch):
    if 0 <= ch < 12:
        return f"{PATAS[ch // 3]} {ARTICULACIONES[ch % 3]}"
    return f"canal {ch}"


def envolver180(x):
    """Lleva un ángulo a (−180, 180]: así 359.5° − 0° cuenta como −0.5°."""
    return ((x + 180.0) % 360.0) - 180.0


def fmt(x, d=1):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    return f"{x:.{d}f}"


def flotante(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def guardar_json(ruta, obj):
    def limpio(x):
        if isinstance(x, dict):
            return {str(k): limpio(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [limpio(v) for v in x]
        if isinstance(x, (np.floating, float)):
            x = float(x)
            return round(x, 6) if math.isfinite(x) else None
        if isinstance(x, np.integer):
            return int(x)
        return x
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(limpio(obj), f, ensure_ascii=False, indent=2)


# ======================================================================= calibración (prueba 01)

def buscar_calibracion():
    """El calibracion_servos.json más reciente de datos/ (las carpetas empiezan con la fecha)."""
    cands = sorted(glob.glob(os.path.join(RAIZ, "datos", "*", "calibracion_servos.json")))
    return cands[-1] if cands else None


class Calibracion:
    """mV del pin -> ángulo, con la tabla de búsqueda de la prueba 01 (o su recta fuera de la tabla)."""

    def __init__(self, ruta, canal):
        self.ruta = ruta
        self.tabla = None
        if ruta is None:
            # Sin archivo: recta típica de la prueba 01, solo para estimar la posición inicial.
            self.a, self.b, self.fuente = -0.1, 187.0, "aproximada (sin calibracion_servos.json)"
            return
        with open(ruta, encoding="utf-8") as f:
            d = json.load(f)
        c = d.get("canales", {}).get(str(canal))
        if c is None:
            raise ValueError(f"{ruta} no tiene el canal {canal}")
        self.a, self.b = c["recta"]["a"], c["recta"]["b"]
        t = c.get("tabla_busqueda")
        if t and len(t.get("mv", [])) >= 2:
            mv, ang = np.array(t["mv"], float), np.array(t["angulo"], float)
            orden = np.argsort(mv)                     # np.interp necesita mV crecientes
            self.tabla = (mv[orden], ang[orden])
        self.fuente = os.path.relpath(ruta, RAIZ).replace("\\", "/")

    def angulo(self, mv):
        if mv is None or not math.isfinite(mv):
            return float("nan")
        if self.tabla is not None:
            xs, ys = self.tabla
            if xs[0] <= mv <= xs[-1]:
                return float(np.interp(mv, xs, ys))
        return self.a * mv + self.b                    # fuera de la tabla: la recta

    def mv_de(self, ang):
        """Inverso (solo para el simulador)."""
        if self.tabla is not None:
            xs, ys = self.tabla
            orden = np.argsort(ys)
            return float(np.interp(ang, ys[orden], xs[orden]))
        return (ang - self.b) / self.a

    @property
    def con_tabla(self):
        return self.tabla is not None


# ======================================================================= ESP32 real

class ErrorESP32(RuntimeError):
    pass


class ESP32:
    """Habla con la ESP32 principal: P, R, D y ? (mismo protocolo que eficiencia_servos.py)."""

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

    def estado(self):
        l = None
        for _ in range(3):
            try:
                l = self.comando("?", "OK1", 1.5)
                break
            except ErrorESP32:
                pass
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


# ======================================================================= simulador

class ESP32Simulado:
    """Un servo con banda muerta y una escala real un poco distinta de la convención, y un
    transportador con su cero en cualquier lugar (como pasa al poner el horn)."""

    def __init__(self, canal, cal, semilla=2):
        self.rng = np.random.default_rng(semilla)
        self.cal = cal
        self.canal = canal
        self.factor = 1.0 + self.rng.normal(0, 0.006)          # giro real / ángulo de la convención
        self.banda = 0.3                                         # ° de banda muerta
        self.cero = float(self.rng.uniform(0, 360))              # marca del disco en 0°
        self.sentido = int(self.rng.choice([1, -1]))             # si los números suben o bajan
        self.pos = 90.0                                          # en grados de la convención
        self.info = {"linea": "OK1-RETRO listo PCA=ok FB=12 AUX=ok (SIMULADO)", "pca": "ok",
                     "fb": 12, "aux": "ok"}

    def mover(self, ch, us):
        us = int(min(max(us, US_MIN), US_LIMITE))
        obj = us_a_ang(us)
        d = obj - self.pos
        if abs(d) > self.banda:
            self.pos = obj - math.copysign(self.banda * (0.4 + 0.6 * self.rng.random()), d)
        return us

    def leer(self, ch, n):
        x = np.round(self.cal.mv_de(self.pos) + self.rng.normal(0, 3.0, n))
        return {"media_mv": float(np.mean(x)), "sigma_mv": float(np.std(x)), "min_mv": float(x.min()),
                "max_mv": float(x.max()), "n": n}

    def marca(self):
        """Lo que leerías en el disco: al grado, con ±0.4° de error de lectura."""
        real = self.pos * self.factor
        return float(round((self.cero + self.sentido * real + self.rng.normal(0, 0.4)) % 360.0))

    def soltar(self):
        pass

    def esperar(self, s):
        pass

    def cerrar(self):
        pass


# ======================================================================= medición

def mover_suave(esp, ch, desde, hasta, vel, dt=0.03):
    """Lleva el servo de `desde` a `hasta` (grados) a ~vel °/s, para no dar golpes."""
    pasos = max(1, int(math.ceil(abs(hasta - desde) / (vel * dt))))
    for k in range(1, pasos + 1):
        esp.mover(ch, ang_a_us(desde + (hasta - desde) * k / pasos))
        esp.esperar(dt)


def pedir_marca(texto):
    """Pide la marca del disco. Regresa un número, 'saltar' o None (terminar)."""
    while True:
        t = input(texto).strip().lower().replace(",", ".")
        if t in ("q", "salir"):
            return None
        if t in ("s", "saltar"):
            return "saltar"
        try:
            v = float(t)
        except ValueError:
            print("    Escribe la marca (por ejemplo 37 o 37.5), s para saltar este punto o q para terminar.")
            continue
        if not 0 <= v < 360:
            print("    El disco va de 0 a 359°.")
            continue
        return v


def imprimir_plan(a, plan):
    print("\n" + "=" * 72)
    print(f"PRUEBA 02 · ÁNGULO REAL — canal {a.canal} ({nombre_canal(a.canal)})")
    print("=" * 72)
    print("  #  sentido  pulso (µs)  ángulo pedido (°)")
    for i, (sentido, us) in enumerate(plan):
        print(f"  {i + 1:>2}    {FLECHA[sentido]}      {us:>6d}       {us_a_ang(us):>7.1f}")
    print(f"\n  {len(plan)} puntos, {a.espera:g} s de espera y {a.muestras} lecturas del ADC en cada uno.")
    print(f"  Entre puntos el servo se mueve despacio (~{a.vel:g} °/s).")


def medir(a, cal):
    plan = [("sube", us) for us in a.pulsos] + [("baja", us) for us in reversed(a.pulsos)]
    imprimir_plan(a, plan)
    print(f"\nCalibración para el ángulo por retroalimentación: {cal.fuente}"
          + ("" if cal.con_tabla else "  (sin tabla: se usa la recta)"))

    esp = ESP32Simulado(a.canal, cal, a.semilla) if a.simulado else ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    if esp.info.get("pca") == "NO":
        esp.cerrar()
        sys.exit("La ESP32 no encuentra el PCA9685 (PCA=NO). Revisa SDA 21, SCL 22, VCC 3V3 y GND.")
    if a.canal >= 3 and esp.info.get("aux") == "NO":
        esp.cerrar()
        sys.exit("El canal es de la ESP32 auxiliar y no está mandando datos (AUX=NO). Revisa TX/RX "
                 "cruzados (16 <-> 17), GND común y que tenga esp32_aux.ino.")

    try:
        inicio = esp.leer(a.canal, 20)
    except ErrorESP32 as e:
        esp.cerrar()
        sys.exit(f"No se pudo leer el cable verde del canal {a.canal}: {e}")
    mv0 = inicio["media_mv"]
    if mv0 < 60 or mv0 > 2500:
        esp.cerrar()
        sys.exit(f"El cable verde del canal {a.canal} da {mv0:.0f} mV (lo normal: 190–1870 mV). Revisa el "
                 "cable, el divisor y que el servo tenga V+. Para verlo en vivo: --revisar")
    actual = min(max(cal.angulo(mv0), 0.0), us_a_ang(US_LIMITE))
    print(f"Posición inicial: {mv0:.0f} mV ≈ {actual:.1f}°")

    if not a.simulado:
        print("\nAntes de empezar:")
        print("  - El servo está FUERA de la pata, sujeto por el cuerpo, sin nada que choque al girar.")
        print("  - El disco está atornillado al horn y no roza la cuchilla.")
        print("  - Lee la marca con el ojo justo encima de la cuchilla.")
        print("En cada punto escribe la marca y Enter. 's' salta el punto, 'q' termina, Ctrl+C detiene todo.")
        input("\nEnter para empezar (Ctrl+C para salir) ")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    carpeta = os.path.join(a.salida, f"{stamp}_canal{a.canal}" + ("_sim" if a.simulado else ""))
    os.makedirs(carpeta, exist_ok=True)
    meta = {
        "fecha": stamp, "prueba": "02_angulo_real", "canal": a.canal, "nombre": nombre_canal(a.canal),
        "pulsos_us": a.pulsos, "espera_s": a.espera, "muestras": a.muestras, "vel_grados_s": a.vel,
        "voltaje": a.voltaje, "simulado": a.simulado, "calibracion": cal.fuente,
        "calibracion_con_tabla": cal.con_tabla, "esp32": esp.info,
        "convencion": "500 us = 0 grados, 2500 us = 180 grados (11.11 us por grado)",
        "posicion_inicial_mv": mv0,
    }
    if a.simulado:
        meta["simulador"] = {"factor_real": esp.factor, "cero_disco": esp.cero, "sentido_disco": esp.sentido}
    guardar_json(os.path.join(carpeta, "meta.json"), meta)

    f = open(os.path.join(carpeta, "mediciones.csv"), "w", newline="", encoding="utf-8")
    w = csv.DictWriter(f, fieldnames=COLUMNAS)
    w.writeheader()
    t0 = time.time()
    cancelado = False
    print()
    try:
        for k, (sentido, us) in enumerate(plan):
            objetivo = us_a_ang(us)
            if abs(objetivo - actual) > 1.0:
                mover_suave(esp, a.canal, actual, objetivo, a.vel)
            aplicado = esp.mover(a.canal, us)
            actual = objetivo
            if aplicado != us:
                print(f"  AVISO: pediste {us} µs y el firmware aplicó {aplicado} µs.")
            esp.esperar(a.espera)
            r = None
            for intento in range(3):
                try:
                    r = esp.leer(a.canal, a.muestras)
                    break
                except ErrorESP32 as e:
                    if intento == 2:
                        print(f"  AVISO: sin lectura del cable verde en {us} µs: {e}")
            cabeza = f"  {k + 1:>2}/{len(plan)} {FLECHA[sentido]} {us:>4d} µs ({us_a_ang(us):5.1f}°)"
            if a.simulado:
                marca = esp.marca()
            else:
                marca = pedir_marca(cabeza + "  marca del disco: ")
                if marca is None:
                    cancelado = True
                    print("  Terminado por ti. Se analiza lo que se alcanzó a medir.")
                    break
            fila = {"n": k + 1, "sentido": sentido, "pulso_us": us, "pulso_aplicado_us": aplicado,
                    "angulo_pedido": round(us_a_ang(aplicado), 3),
                    "marca": "" if marca == "saltar" else marca,
                    "media_mv": round(r["media_mv"], 2) if r else "",
                    "sigma_mv": round(r["sigma_mv"], 3) if r else "",
                    "min_mv": r["min_mv"] if r else "", "max_mv": r["max_mv"] if r else "",
                    "muestras": r["n"] if r else "", "t_s": round(time.time() - t0, 2)}
            w.writerow(fila)
            f.flush()
            # El ángulo por retroalimentación se muestra DESPUÉS de anotar la marca, para no sesgarte.
            retro = cal.angulo(r["media_mv"]) if r else float("nan")
            txt_marca = "saltado" if marca == "saltar" else f"marca {marca:g}°"
            print(("" if not a.simulado else cabeza) +
                  f"{'' if a.simulado else '      '}  {txt_marca} · {fmt(r['media_mv'] if r else None)} mV"
                  f" · retro {fmt(retro, 2)}°")
    except KeyboardInterrupt:
        cancelado = True
        print("\n\nDetenido. Se analiza lo que se alcanzó a medir.")
    finally:
        esp.soltar()
        esp.cerrar()
        f.close()
    meta["cancelado"] = cancelado
    meta["duracion_min"] = round((time.time() - t0) / 60, 1)
    guardar_json(os.path.join(carpeta, "meta.json"), meta)
    return carpeta


# ======================================================================= análisis

def cargar_mediciones(ruta):
    filas = []
    with open(ruta, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            filas.append({
                "sentido": r["sentido"], "pulso_aplicado_us": int(float(r["pulso_aplicado_us"])),
                "angulo_pedido": flotante(r["angulo_pedido"]), "marca": flotante(r["marca"]),
                "media_mv": flotante(r["media_mv"]),
            })
    return filas


def analizar(carpeta, meta, cal, numeros="auto"):
    filas = cargar_mediciones(os.path.join(carpeta, "mediciones.csv"))
    con_marca = [f for f in filas if math.isfinite(f["marca"])]
    ref = next((f for f in con_marca if f["sentido"] == "sube" and f["pulso_aplicado_us"] == US_MIN), None)
    if ref is None:
        ref = next((f for f in con_marca if f["pulso_aplicado_us"] == US_MIN), None)
    if ref is None:
        print("\nNo hay marca en 500 µs: es la referencia (0°), sin ella no se puede calcular el giro.")
        return None
    m0 = ref["marca"]

    # ¿Los números del disco suben o bajan al girar? Se ve en el punto más lejano del 0.
    lejos = max(con_marca, key=lambda f: f["angulo_pedido"])
    if numeros == "auto":
        if lejos["angulo_pedido"] < 20:
            print("AVISO: no hay puntos lejos de 0°; supongo que los números del disco suben al girar.")
            signo = 1
        else:
            signo = 1 if envolver180(lejos["marca"] - m0) >= 0 else -1
    else:
        signo = 1 if numeros == "suben" else -1
    for f in filas:
        f["giro"] = envolver180(signo * (f["marca"] - m0)) if math.isfinite(f["marca"]) else float("nan")
        f["retro"] = cal.angulo(f["media_mv"])

    # Una fila por pulso: subiendo, bajando y su promedio
    tabla = []
    for us in sorted({f["pulso_aplicado_us"] for f in filas}):
        s = [f for f in filas if f["pulso_aplicado_us"] == us and f["sentido"] == "sube"]
        b = [f for f in filas if f["pulso_aplicado_us"] == us and f["sentido"] == "baja"]
        fs, fb = (s[0] if s else None), (b[0] if b else None)

        def campo(fila, k):
            return fila[k] if fila is not None else float("nan")
        giros = [g for g in (campo(fs, "giro"), campo(fb, "giro")) if math.isfinite(g)]
        mvs = [v for v in (campo(fs, "media_mv"), campo(fb, "media_mv")) if math.isfinite(v)]
        pedido = us_a_ang(us)
        giro = float(np.mean(giros)) if giros else float("nan")
        mv = float(np.mean(mvs)) if mvs else float("nan")
        retro = cal.angulo(mv)
        tabla.append({
            "pulso_us": us, "angulo_pedido": pedido,
            "marca_sube": campo(fs, "marca"), "marca_baja": campo(fb, "marca"),
            "giro_sube": campo(fs, "giro"), "giro_baja": campo(fb, "giro"), "giro_real": giro,
            "histeresis": abs(campo(fs, "giro") - campo(fb, "giro")),
            "media_mv": mv, "angulo_retro": retro,
            "error_convencion": giro - pedido, "error_retro": retro - giro,
        })

    # Escala real: recta de giro real contra (pulso − 500), con todos los puntos
    xs = np.array([f["pulso_aplicado_us"] - US_MIN for f in filas if math.isfinite(f["giro"])], float)
    ys = np.array([f["giro"] for f in filas if math.isfinite(f["giro"])], float)
    escala = corte = float("nan")
    if len(set(xs.tolist())) >= 2:
        escala, corte = (float(v) for v in np.polyfit(xs, ys, 1))
    extremo = max((t for t in tabla if math.isfinite(t["giro_real"]) and t["pulso_us"] > US_MIN),
                  key=lambda t: t["pulso_us"], default=None)
    escala_extremo = (extremo["giro_real"] / (extremo["pulso_us"] - US_MIN)) if extremo else float("nan")

    errs = [t["error_convencion"] for t in tabla if math.isfinite(t["error_convencion"])]
    errs_retro = [t["error_retro"] for t in tabla if math.isfinite(t["error_retro"])]
    err_max = max((abs(e) for e in errs), default=float("nan"))
    err_retro_max = max((abs(e) for e in errs_retro), default=float("nan"))
    hist = [t["histeresis"] for t in tabla if math.isfinite(t["histeresis"])]

    if not errs:
        veredicto = "Sin puntos suficientes para concluir."
    elif err_max <= TOLERANCIA:
        veredicto = (f"La convención es buena: el giro real queda dentro de ±{TOLERANCIA:g}° del ángulo pedido "
                     f"en todos los puntos (máximo {err_max:.1f}°). No hay que cambiar los 11.11 µs/°.")
    else:
        veredicto = (f"El giro real se separa hasta {err_max:.1f}° del ángulo pedido (más de ±{TOLERANCIA:g}°). "
                     f"La escala medida es {escala:.4f} °/µs ({1 / escala:.2f} µs/°) contra "
                     f"{ESCALA_CONV:.4f} °/µs (11.11 µs/°). Repite la prueba para confirmarlo antes de "
                     "cambiar el factor en el firmware y en los scripts.")
    faltan = [us for us in meta.get("pulsos_us", []) if not any(
        t["pulso_us"] == us and math.isfinite(t["giro_real"]) for t in tabla)]
    if meta.get("cancelado") or faltan:
        veredicto = ("CORRIDA INCOMPLETA" + (f" (sin marca en {', '.join(map(str, faltan))} µs)" if faltan else "")
                     + ": no sirve para concluir; repítela completa. Con lo que hay: " + veredicto)

    res = {
        "canal": meta.get("canal"), "nombre": meta.get("nombre"), "fecha": meta.get("fecha"),
        "simulado": meta.get("simulado"), "calibracion": cal.fuente,
        "referencia_marca": m0, "numeros_del_disco": "suben" if signo > 0 else "bajan",
        "escala_grados_por_us": escala, "corte_grados": corte, "us_por_grado": 1 / escala if escala else None,
        "escala_extremo_grados_por_us": escala_extremo, "escala_convencion_grados_por_us": ESCALA_CONV,
        "error_convencion_max": err_max, "error_retro_max": err_retro_max,
        "histeresis_media": float(np.mean(hist)) if hist else float("nan"),
        "histeresis_max": max(hist) if hist else float("nan"),
        "tolerancia": TOLERANCIA, "veredicto": veredicto, "tabla": tabla,
    }
    guardar_json(os.path.join(carpeta, "resultados.json"), res)
    with open(os.path.join(carpeta, "resultados.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(tabla[0].keys()))
        w.writeheader()
        for t in tabla:
            w.writerow({k: (round(v, 3) if isinstance(v, float) and math.isfinite(v) else
                            ("" if isinstance(v, float) else v)) for k, v in t.items()})
    escribir_resumen(carpeta, meta, res)
    grafica(carpeta, meta, res, filas)
    imprimir_resultados(res, carpeta)
    return res


def escribir_resumen(carpeta, meta, res):
    lin = [f"# 02 · Ángulo real — canal {res['canal']} ({res['nombre']})", ""]
    lin += ["| | |", "|---|---|",
            f"| Fecha | {meta.get('fecha', '')} |",
            f"| Fuente | {fmt(meta.get('voltaje'), 2) if meta.get('voltaje') else '—'} V |",
            f"| Simulado | {'sí' if meta.get('simulado') else 'no'} |",
            f"| Calibración de la retroalimentación | `{res['calibracion']}` |",
            f"| Referencia (0°) | marca {res['referencia_marca']:g}° en 500 µs subiendo; "
            f"los números del disco {res['numeros_del_disco']} al girar |", ""]
    lin += ["| Pulso (µs) | Ángulo pedido | Lectura subiendo (°) | Lectura bajando (°) | Giro real (°) "
            "| Error (°) | Cable verde (mV) | Ángulo por retroalimentación (°) | Retro − real (°) |",
            "|---|---|---|---|---|---|---|---|---|"]
    for t in res["tabla"]:
        lin.append(f"| {t['pulso_us']} | {fmt(t['angulo_pedido'])} | {fmt(t['marca_sube'])} | "
                   f"{fmt(t['marca_baja'])} | {fmt(t['giro_real'])} | {fmt(t['error_convencion'], 2)} | "
                   f"{fmt(t['media_mv'])} | {fmt(t['angulo_retro'], 2)} | {fmt(t['error_retro'], 2)} |")
    lin += ["", "Giro real = promedio de subiendo y bajando. Error = giro real − ángulo pedido.", ""]
    lin += [f"- **Escala real (recta con todos los puntos):** {fmt(res['escala_grados_por_us'], 4)} °/µs"
            f" = {fmt(res['us_por_grado'], 2)} µs/° (convención: {ESCALA_CONV:.4f} °/µs = 11.11 µs/°).",
            f"- **Escala con el extremo:** {fmt(res['escala_extremo_grados_por_us'], 4)} °/µs.",
            f"- **Error máximo de la convención:** {fmt(res['error_convencion_max'], 2)}°.",
            f"- **Error máximo de la retroalimentación contra el giro real:** "
            f"{fmt(res['error_retro_max'], 2)}°.",
            f"- **Diferencia subiendo contra bajando en el disco:** {fmt(res['histeresis_media'], 2)}° en "
            f"promedio, {fmt(res['histeresis_max'], 2)}° como máximo.", "",
            f"**Veredicto:** {res['veredicto']}", ""]
    with open(os.path.join(carpeta, "resumen.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lin))


def grafica(carpeta, meta, res, filas):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("AVISO: falta matplotlib; no se hizo la gráfica (pip install -r requirements.txt).")
        return
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK2,
                         "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": SURF, "axes.facecolor": SURF})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.5, 6.6), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 2]})
    tope = us_a_ang(US_LIMITE)
    ax1.plot([0, tope], [0, tope], color=MUTED, lw=1, ls="--", label="ideal (convención)")
    for sentido, relleno in (("sube", True), ("baja", False)):
        fs = [f for f in filas if f["sentido"] == sentido and math.isfinite(f["giro"])]
        ax1.scatter([f["angulo_pedido"] for f in fs], [f["giro"] for f in fs], s=46, marker="o",
                    facecolors=AZUL if relleno else "none", edgecolors=AZUL, linewidths=1.4, zorder=3,
                    label=f"giro real, {'subiendo' if relleno else 'bajando'}")
        ax2.scatter([f["angulo_pedido"] for f in fs], [f["giro"] - f["angulo_pedido"] for f in fs], s=46,
                    marker="o", facecolors=AZUL if relleno else "none", edgecolors=AZUL, linewidths=1.4,
                    zorder=3)
    t = [x for x in res["tabla"] if math.isfinite(x["angulo_retro"])]
    ax1.scatter([x["angulo_pedido"] for x in t], [x["angulo_retro"] for x in t], marker="x", s=40,
                color=NARANJA, zorder=4, label="retroalimentación (tabla de la prueba 01)")
    t = [x for x in res["tabla"] if math.isfinite(x["error_retro"])]
    ax2.scatter([x["angulo_pedido"] for x in t], [x["error_retro"] for x in t], marker="x", s=40,
                color=NARANJA, zorder=4)
    ax2.axhspan(-TOLERANCIA, TOLERANCIA, color="#e6eef9", zorder=0)
    ax2.axhline(0, color=MUTED, lw=0.8)
    for ax in (ax1, ax2):
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    ax1.set_ylabel("giro real (°)")
    ax2.set_ylabel("error (°)")
    ax2.set_xlabel("ángulo pedido (°)  ·  500 µs = 0°, 11.11 µs/°")
    lim = max(1.5, 1.2 * max([abs(f["giro"] - f["angulo_pedido"]) for f in filas if math.isfinite(f["giro"])]
                             + [abs(x["error_retro"]) for x in t] + [1.0]))
    ax2.set_ylim(-lim, lim)
    ax2.set_title("azul: giro real − pedido · naranja: retroalimentación − giro real · franja: ±1°",
                  loc="left", fontsize=8, color=INK2)
    ax1.legend(frameon=False, fontsize=8, loc="upper left")
    titulo = f"Ángulo real · canal {res['canal']} ({res['nombre']})" + (" · SIMULADO" if res["simulado"] else "")
    fig.suptitle(titulo, x=0.08, ha="left", fontsize=11, fontweight="bold", color=INK)
    ax1.set_title(f"escala real {fmt(res['escala_grados_por_us'], 4)} °/µs (convención {ESCALA_CONV:.4f}) · "
                  f"error máx {fmt(res['error_convencion_max'], 2)}°", loc="left", fontsize=8.5, color=INK2)
    fig.tight_layout()
    fig.savefig(os.path.join(carpeta, "angulo_real.png"), dpi=150)
    plt.close(fig)


def imprimir_resultados(res, carpeta):
    print("\n" + "=" * 72)
    print(f"RESULTADOS — canal {res['canal']} ({res['nombre']})")
    print("=" * 72)
    print(f"Referencia: marca {res['referencia_marca']:g}° en 500 µs; los números del disco "
          f"{res['numeros_del_disco']} al girar.")
    print("\n  pulso   pedido   subiendo  bajando   giro real   error    mV      retro   retro−real")
    for t in res["tabla"]:
        print(f"  {t['pulso_us']:>5}  {fmt(t['angulo_pedido']):>7}  {fmt(t['marca_sube']):>8}  "
              f"{fmt(t['marca_baja']):>7}  {fmt(t['giro_real']):>9}  {fmt(t['error_convencion'], 2):>6}  "
              f"{fmt(t['media_mv']):>7}  {fmt(t['angulo_retro'], 2):>7}  {fmt(t['error_retro'], 2):>8}")
    print(f"\nEscala real: {fmt(res['escala_grados_por_us'], 4)} °/µs = {fmt(res['us_por_grado'], 2)} µs/° "
          f"(convención {ESCALA_CONV:.4f} °/µs = 11.11 µs/°)")
    print(f"\n{res['veredicto']}")
    print(f"\nResultados en {carpeta}")
    print("  resumen.md (la tabla para la nota 02), angulo_real.png, resultados.csv y resultados.json")


def buscar_corrida(ruta, salida):
    if ruta == "ultima":
        cands = sorted(d for d in (os.listdir(salida) if os.path.isdir(salida) else [])
                       if os.path.isfile(os.path.join(salida, d, "mediciones.csv")))
        if not cands:
            sys.exit(f"No hay corridas en {salida}")
        return os.path.join(salida, cands[-1])
    if os.path.isfile(ruta):
        return os.path.dirname(os.path.abspath(ruta))
    if os.path.isfile(os.path.join(ruta, "mediciones.csv")):
        return ruta
    sys.exit(f"No encuentro mediciones.csv en {ruta}")


# ======================================================================= revisar

def revisar(a, cal):
    esp = ESP32(a.port)
    print(f"ESP32: {esp.info['linea']}")
    esp.soltar()
    print(f"\nServo suelto. Canal {a.canal} ({nombre_canal(a.canal)}) en vivo; gíralo a mano para ver que "
          "cambie. Ctrl+C para salir.\n")
    try:
        while True:
            try:
                r = esp.leer(a.canal, 16)
                txt = f"{r['media_mv']:7.0f} mV   ≈ {cal.angulo(r['media_mv']):6.1f}°   (σ {r['sigma_mv']:.1f} mV)"
            except ErrorESP32 as e:
                txt = f"sin lectura: {e}"
            print("\r  " + txt.ljust(60), end="", flush=True)
            time.sleep(0.15)
    except KeyboardInterrupt:
        print()
    finally:
        esp.cerrar()


# ======================================================================= principal

def main():
    ap = argparse.ArgumentParser(description="Prueba 02 de Arca9: ángulo real del eje, con transportador.")
    ap.add_argument("--port", help="puerto de la ESP32 principal (COM6, /dev/ttyUSB0)")
    ap.add_argument("--canal", type=int, default=0, help="servo a medir, 0–11 (por defecto 0 = FL abducción)")
    ap.add_argument("--pulsos", type=int, nargs="+", default=PULSOS,
                    help=f"pulsos en µs, entre {US_MIN} y {US_LIMITE}; tiene que estar 500 (la referencia)")
    ap.add_argument("--espera", type=float, default=2.0, help="s entre llegar al punto y leer")
    ap.add_argument("--muestras", type=int, default=200, help="lecturas del ADC por punto (1–1000)")
    ap.add_argument("--vel", type=float, default=60.0, help="°/s al moverse entre puntos")
    ap.add_argument("--voltaje", type=float, help="voltaje de la fuente de los servos (para el registro)")
    ap.add_argument("--calibracion", help="calibracion_servos.json de la prueba 01 (por defecto, el más "
                                          "reciente de datos/)")
    ap.add_argument("--numeros", choices=["auto", "suben", "bajan"], default="auto",
                    help="si los números del disco suben o bajan al girar (auto lo detecta)")
    ap.add_argument("--revisar", action="store_true", help="solo mostrar el cable verde en vivo, servo suelto")
    ap.add_argument("--simulado", action="store_true", help="sin hardware: servo y transportador simulados")
    ap.add_argument("--semilla", type=int, default=2, help="semilla del simulador")
    ap.add_argument("--reanalizar", metavar="RUTA",
                    help="rehace los cálculos: la carpeta de una corrida, su mediciones.csv o 'ultima'")
    ap.add_argument("--salida", default=os.path.join(HERE, "resultados_angulo_real"))
    a = ap.parse_args()

    if not 0 <= a.canal <= 11:
        ap.error("los canales con retroalimentación van de 0 a 11")
    a.pulsos = sorted(set(a.pulsos))
    fuera = [p for p in a.pulsos if not US_MIN <= p <= US_LIMITE]
    if fuera:
        ap.error(f"pulsos fuera de {US_MIN}–{US_LIMITE} µs: {fuera}. Arriba de {US_LIMITE} el servo empuja "
                 "contra su tope.")
    if US_MIN not in a.pulsos:
        ap.error("incluye 500 µs en --pulsos: es la referencia (0°)")
    if len(a.pulsos) < 2:
        ap.error("hacen falta al menos dos pulsos")
    a.muestras = max(1, min(1000, a.muestras))
    if a.vel <= 0 or a.vel > 150:
        ap.error("--vel entre 0 y 150 °/s")

    meta = {}
    carpeta = None
    if a.reanalizar:
        carpeta = buscar_corrida(a.reanalizar, a.salida)
        with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
        a.canal = meta.get("canal", a.canal)

    ruta_cal = a.calibracion
    if ruta_cal is None and meta.get("calibracion") and not meta["calibracion"].startswith("aproximada"):
        ruta_cal = os.path.join(RAIZ, meta["calibracion"])     # la misma que usó la corrida
    if ruta_cal is None:
        ruta_cal = buscar_calibracion()
    if ruta_cal is not None and not os.path.isfile(ruta_cal):
        sys.exit(f"No encuentro {ruta_cal}")
    try:
        cal = Calibracion(ruta_cal, a.canal)
    except (ValueError, KeyError) as e:
        sys.exit(f"Calibración inválida: {e}")
    if ruta_cal is None:
        print("AVISO: no encontré calibracion_servos.json en datos/; el ángulo por retroalimentación será "
              "aproximado.")

    if a.reanalizar:
        print(f"Reanalizando {carpeta}")
        analizar(carpeta, meta, cal, a.numeros)
        return
    if a.simulado:
        a.espera = 0.0
    elif not a.port:
        ap.error("falta --port (o usa --simulado, o --reanalizar)")

    try:
        if a.revisar:
            if a.simulado:
                ap.error("--revisar necesita el hardware (--port)")
            revisar(a, cal)
            return
        carpeta = medir(a, cal)
    except ErrorESP32 as e:
        sys.exit(f"Error: {e}")
    except KeyboardInterrupt:
        sys.exit("\nCancelado antes de empezar; no se movió nada.")
    with open(os.path.join(carpeta, "meta.json"), encoding="utf-8") as f:
        meta = json.load(f)
    analizar(carpeta, meta, cal, a.numeros)


if __name__ == "__main__":
    main()
