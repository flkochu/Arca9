#!/usr/bin/env python3
"""
Escala de Arca9: pulsos, grados de la convención y grados reales
=================================================================
El firmware solo maneja µs (500 a 2360). Los scripts de las pruebas 01 y 02 usan los "grados de
la convención": 500 µs = 0° y 11.11 µs por grado. La prueba 02 (2 nov 2026) midió con el
transportador que el eje gira MÁS que eso: un factor de ~1.043 para los 12 servos, o sea
~10.65 µs por grado real. Se decidió (opción B) no tocar el firmware y aplicar el factor aquí,
en la compu.

    grado real = factor × grado de la convención

Este módulo es el único lugar que hace esa cuenta. Lee el factor de datos/*/escala.json (el más
reciente) y la retroalimentación de datos/*/calibracion_servos.json (prueba 01), así que no hay
números copiados a mano. Todo lo nuevo (puente, marcha, ruido) debe convertir con él.

Uso desde otro script (en la carpeta pc/):
    from escala import Escala, Retro
    esc = Escala()
    esc.real_a_us(90)          # pulso para 90° reales (recortado a 500–2360 µs)
    esc.us_a_real(1500)        # grados reales de un pulso
    retro = Retro(esc)
    retro.grados(3, 967.0)     # grados reales del canal 3 a partir de los mV de su cable verde

Para ver la escala que se está usando:
    python escala.py
"""
import bisect
import glob
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(HERE)

US_CERO = 500                       # 0° (convención y real comparten el cero)
US_POR_GRADO_CONV = 2000.0 / 180.0  # 11.11 µs/°, la convención de las pruebas 01 y 02
US_LIMITE = 2360                    # el mismo límite duro del firmware (US_MAX_CH)
N_SERVOS = 12
MV_MIN, MV_MAX = 165, 1950       # lecturas creíbles del cable verde; fuera = cable suelto o divisor mal


def buscar_en_datos(nombre):
    """El archivo `nombre` más reciente en datos/<AAAA-MM-DD_prueba>/ (o None)."""
    cands = sorted(glob.glob(os.path.join(RAIZ, "datos", "*", nombre)))
    return cands[-1] if cands else None


def relativa(ruta):
    """La ruta desde la raíz del repo (para los mensajes). Si está en otro disco (Windows: el repo en
    E: y un archivo temporal en C:), no hay ruta relativa: se regresa completa."""
    if not ruta:
        return None
    try:
        return os.path.relpath(ruta, RAIZ).replace("\\", "/")
    except ValueError:
        return os.path.abspath(ruta).replace("\\", "/")


class Escala:
    """Conversión entre µs, grados de la convención y grados reales."""

    def __init__(self, ruta=None):
        ruta = ruta or buscar_en_datos("escala.json")
        if ruta is None:
            raise FileNotFoundError("no encontré datos/*/escala.json (resultado de la prueba 02)")
        with open(ruta, encoding="utf-8") as f:
            d = json.load(f)
        self.factor = float(d["factor_real"])
        if not 0.9 < self.factor < 1.1:
            raise ValueError(f"factor_real = {self.factor} en {ruta}: fuera de 0.9–1.1, revisa el archivo")
        self.fuente = relativa(ruta)

    @property
    def us_por_grado(self):
        return US_POR_GRADO_CONV / self.factor

    @property
    def limite_grados(self):
        """Grados reales en el límite del firmware (2360 µs)."""
        return self.us_a_real(US_LIMITE)

    def conv_a_real(self, g):
        return g * self.factor

    def real_a_conv(self, g):
        return g / self.factor

    def us_a_real(self, us):
        return (us - US_CERO) / self.us_por_grado

    def real_a_us(self, g, recortar=True):
        """Pulso para `g` grados reales. Con recortar=True nunca sale de 500–2360 µs."""
        us = int(round(US_CERO + g * self.us_por_grado))
        if recortar:
            us = min(max(us, US_CERO), US_LIMITE)
        return us


class Retro:
    """mV del cable verde -> grados reales, con la tabla de búsqueda de la prueba 01 (o su recta
    fuera de la tabla), pasada a grados reales con el factor de la prueba 02."""

    def __init__(self, escala, ruta=None):
        ruta = ruta or buscar_en_datos("calibracion_servos.json")
        if ruta is None:
            raise FileNotFoundError("no encontré datos/*/calibracion_servos.json (resultado de la prueba 01)")
        with open(ruta, encoding="utf-8") as f:
            d = json.load(f)
        self.escala = escala
        self.fuente = relativa(ruta)
        self.canales = {}
        for k, c in d["canales"].items():
            t = c.get("tabla_busqueda") or {}
            pares = sorted(zip(t.get("mv", []), t.get("angulo", [])))   # mV crecientes
            self.canales[int(k)] = {
                "a": c["recta"]["a"], "b": c["recta"]["b"],
                "mv": [p[0] for p in pares], "ang": [p[1] for p in pares],
                "ruido_mv": c.get("ruido_mv"), "nombre": c.get("nombre", f"canal {k}"),
            }
        faltan = [ch for ch in range(N_SERVOS) if ch not in self.canales]
        if faltan:
            raise ValueError(f"{ruta} no tiene los canales {faltan}")

    def grados_conv(self, ch, mv):
        c = self.canales[ch]
        xs, ys = c["mv"], c["ang"]
        if len(xs) >= 2 and xs[0] <= mv <= xs[-1]:
            i = min(max(bisect.bisect_left(xs, mv), 1), len(xs) - 1)
            x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
            return y0 + (y1 - y0) * (mv - x0) / (x1 - x0) if x1 != x0 else y0
        return c["a"] * mv + c["b"]          # fuera de la tabla: la recta

    def grados(self, ch, mv):
        """Grados reales del canal `ch` a partir de los mV de su cable verde."""
        return self.escala.conv_a_real(self.grados_conv(ch, mv))

    def grados_por_mv(self, ch):
        """Cuántos grados reales vale 1 mV del pin (para pasar ruido de mV a grados)."""
        return abs(self.canales[ch]["a"]) * self.escala.factor

    def nombre(self, ch):
        return self.canales[ch]["nombre"]


def main():
    try:
        import sys
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    esc = Escala()
    print(f"Escala: {esc.fuente}")
    print(f"  factor real / convención = {esc.factor:.4f}")
    print(f"  {esc.us_por_grado:.3f} µs por grado real (convención: {US_POR_GRADO_CONV:.3f})")
    print(f"  límite del firmware: {US_LIMITE} µs = {esc.limite_grados:.1f}° reales "
          f"({(US_LIMITE - US_CERO) / US_POR_GRADO_CONV:.1f}° de la convención)")
    print("\n  grados reales  ->  pulso (µs)")
    for g in (0, 45, 90, 135, round(esc.limite_grados, 1)):
        print(f"  {g:>12}       {esc.real_a_us(g):>6}")
    retro = Retro(esc)
    print(f"\nRetroalimentación: {retro.fuente}")
    print("  canal  nombre            1 mV = ° reales")
    for ch in range(N_SERVOS):
        print(f"  {ch:>5}  {retro.nombre(ch):<16}  {retro.grados_por_mv(ch):.4f}")


if __name__ == "__main__":
    main()
