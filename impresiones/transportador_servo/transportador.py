#!/usr/bin/env python3
"""
Transportador para comprobar el ángulo real de un servo FS5115M-FB
==================================================================
Genera dos piezas para imprimir juntas:

  disco_transportador.stl   disco de 84 mm con marcas cada 2° (números cada 10°). Va ATORNILLADO
                            ENCIMA DEL HORN y gira con el eje, así que queda centrado en el eje
                            sin importar cómo esté hecho el servo.
  indicador.stl             abrazadera que se mete por abajo del servo hasta topar con las
                            orejas de montaje, con un brazo y una cuchilla vertical que queda
                            junto al borde del disco: es la marca fija contra la que se lee.
  placa_transportador.stl   las dos piezas acomodadas en una sola cama de impresión.

Como el disco gira con el eje y el indicador se queda fijo en el cuerpo, el giro real entre
dos comandos es la diferencia de lecturas. La posición de la cuchilla solo mueve el cero, no
la escala.

Uso:  python transportador.py            (escribe los STL junto a este archivo)
      python transportador.py --radio 36 (disco más chico: marcas más juntas)
Necesita: pip install numpy trimesh shapely manifold3d mapbox_earcut matplotlib
"""
import argparse
import math
import os

import numpy as np
import trimesh
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))

# --- servo FS5115M-FB (Pololu): 40.8 × 20.1 × 39 mm ---
SERVO_L, SERVO_W = 40.8, 20.1
EJE_DESDE_CENTRO = 10.0      # el eje está ~10 mm corrido del centro del cuerpo, sobre su línea media
TAB_A_DISCO = 17.5           # de la cara de abajo de las orejas a la cara de arriba del disco (aprox.)


def extruir(geom, alto, z0=0.0):
    """Polígono (o multipolígono) de shapely -> malla extruida de z0 a z0 + alto."""
    geoms = getattr(geom, "geoms", [geom])
    mallas = [trimesh.creation.extrude_polygon(g, alto) for g in geoms if g.area > 1e-6]
    m = trimesh.util.concatenate(mallas)
    m.apply_translation([0, 0, z0])
    return m


def unir(mallas):
    m = trimesh.boolean.union(mallas, engine="manifold")
    return m


def texto(s, alto):
    """Contorno de un texto (DejaVu Sans Bold) de `alto` mm, centrado en el origen."""
    tp = TextPath((0, 0), s, size=alto * 1.36, prop=FontProperties(family="DejaVu Sans", weight="bold"))
    geom = None
    for p in tp.to_polygons():
        if len(p) < 3:
            continue
        pol = Polygon(p).buffer(0)
        geom = pol if geom is None else geom.symmetric_difference(pol)
    minx, miny, maxx, maxy = geom.bounds
    return affinity.translate(geom, -(minx + maxx) / 2, -(miny + maxy) / 2)


def disco(R, grueso=2.0, relieve=0.6):
    # base: disco con hueco central (para el cubo del horn y su tornillo) y 8 ranuras radiales
    base = Point(0, 0).buffer(R, resolution=256)
    base = base.difference(Point(0, 0).buffer(5.5, resolution=64))           # Ø 11 mm
    for k in range(8):
        a = math.radians(45 * k)
        ranura = box(7.0, -1.1, 21.0, 1.1).union(Point(7.0, 0).buffer(1.1)).union(Point(21.0, 0).buffer(1.1))
        base = base.difference(affinity.rotate(ranura, math.degrees(a), origin=(0, 0)))

    # relieve: marcas cada 2°, más largas cada 10° y en 0/90/180/270, números cada 10°
    piezas = []
    for g in range(0, 360, 2):
        if g % 90 == 0:
            largo, ancho = 8.0, 0.8
        elif g % 10 == 0:
            largo, ancho = 6.0, 0.6
        else:
            largo, ancho = 3.5, 0.45
        marca = box(R - largo, -ancho / 2, R, ancho / 2)
        piezas.append(affinity.rotate(marca, g, origin=(0, 0)))
    r_num = R - 13.6                                                          # centro de los números
    for g in range(0, 360, 10):
        t = texto(str(g), 2.6 if g % 90 else 3.0)
        # se leen a lo largo del radio, de adentro hacia afuera
        t = affinity.rotate(t, g, origin=(0, 0))
        t = affinity.translate(t, r_num * math.cos(math.radians(g)), r_num * math.sin(math.radians(g)))
        piezas.append(t)
    anillo = Point(0, 0).buffer(R - 8.6, resolution=256).difference(Point(0, 0).buffer(R - 9.1, resolution=256))
    piezas.append(anillo)
    # triángulo en el 0
    piezas.append(Polygon([(R - 8.7, 0), (R - 10.9, 1.4), (R - 10.9, -1.4)]))
    relieve_geo = unary_union(piezas).intersection(Point(0, 0).buffer(R, resolution=256))

    m = unir([extruir(base, grueso), extruir(relieve_geo, relieve + 0.01, grueso - 0.01)])
    return m


def indicador(R, holgura_borde=1.0, alto_abrazadera=7.0, pared=2.0):
    ancho_int = SERVO_W - 0.15          # un poco más angosta que el servo: aprieta
    largo_int = SERVO_L + 0.6
    ext = box(-largo_int / 2 - pared, -ancho_int / 2 - pared, largo_int / 2 + pared, ancho_int / 2 + pared)
    inte = box(-largo_int / 2, -ancho_int / 2, largo_int / 2, ancho_int / 2)
    abertura = box(largo_int / 2 - 0.1, -5.0, largo_int / 2 + pared + 0.1, 5.0)   # para el cable
    abraz = ext.difference(inte).difference(abertura)

    # la cuchilla queda a R + holgura del eje, que está a ~10 mm del centro del cuerpo en x
    y_cuchilla = math.sqrt((R + holgura_borde) ** 2 - EJE_DESDE_CENTRO ** 2)
    y0 = ancho_int / 2 + pared - 0.5
    brazo = box(-2.75, y0, 2.75, y_cuchilla + 3.6)
    pie = box(-2.75, y_cuchilla + 0.6, 2.75, y_cuchilla + 3.6)                       # refuerzo
    cuchilla = Polygon([(0, y_cuchilla), (-2.2, y_cuchilla + 3.6), (2.2, y_cuchilla + 3.6)])

    alto_cuchilla = alto_abrazadera + TAB_A_DISCO + 2.0
    m = unir([
        extruir(abraz, alto_abrazadera),
        extruir(brazo, 3.0),
        extruir(pie, 9.0),
        extruir(cuchilla, alto_cuchilla),
    ])
    return m, y_cuchilla, alto_cuchilla


def main():
    ap = argparse.ArgumentParser(description="Transportador para el servo FS5115M-FB")
    ap.add_argument("--radio", type=float, default=42.0, help="radio del disco en mm (42 = Ø 84 mm)")
    ap.add_argument("--salida", default=HERE)
    a = ap.parse_args()

    d = disco(a.radio)
    ind, y_c, alto_c = indicador(a.radio)
    for nombre, m in (("disco_transportador.stl", d), ("indicador.stl", ind)):
        m.export(os.path.join(a.salida, nombre))
        print(f"{nombre}: {m.extents[0]:.1f} × {m.extents[1]:.1f} × {m.extents[2]:.1f} mm, "
              f"cerrada: {m.is_watertight}")

    # placa: las dos piezas lado a lado, sin encimarse
    ind2 = ind.copy()
    ind2.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 0, 1]))
    dx = a.radio + 4 - ind2.bounds[0][0]
    ind2.apply_translation([dx, -ind2.centroid[1], 0])
    placa = trimesh.util.concatenate([d, ind2])
    placa.export(os.path.join(a.salida, "placa_transportador.stl"))
    print(f"placa_transportador.stl: {placa.extents[0]:.1f} × {placa.extents[1]:.1f} mm de cama")
    paso = a.radio * math.radians(2)
    print(f"Separación entre marcas de 2° en el borde: {paso:.2f} mm. Cuchilla a {y_c:.1f} mm del "
          f"centro del cuerpo, {alto_c:.1f} mm de alto.")


if __name__ == "__main__":
    main()
