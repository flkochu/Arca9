"""
Comandos de la ESP32 principal de Arca9, para los scripts de la compu
======================================================================
El protocolo está en docs/hardware.md (USB a 115200 baud, una línea por comando):

    P canal us   -> "OK P canal us"   (us = pulso que sí aplicó el firmware, 500–2360)
    R canal n    -> "R canal n media sigma min max"   (mV del pin)
    V            -> "V mv0 ... mv11"  (-1 = sin lectura)
    D            -> "OK D"            (suelta todos los servos)
    ?            -> "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=3 RESET=encendido SEG=12"
                    (+ "DIAG=1 NS=.. SMAX=.." al final desde el 9 nov: ver docs/hardware.md)
    G canal us_a us_b vel vueltas espera_ms   -> rampa hecha por la ESP32 (protocolo v2);
                 al terminar manda "G DATOS", las líneas M (t, mV) y C (t, µs) y "G FIN"
    S us0 ... us11 -> "S mv0 ... mv11"  los 12 a la vez (-1 = no cambiar, 0 = soltar) (v3)
    T INICIO mascara n / K ... / T CORRER [grabar]   trayectoria de varios servos (v3)

eficiencia_servos.py y angulo_real.py tienen su propia copia de esto (se escribieron antes);
los scripts nuevos usan este archivo.
"""
import math
import re
import time


class ErrorESP32(RuntimeError):
    pass


REINICIO = ("la ESP32 se reinició a media prueba (mandó su mensaje de arranque). Casi siempre es una caída "
            "de voltaje cuando se mueven los servos: revisa la fuente de 6 V, que la ESP32 no se alimente de "
            "la misma fuente sin protección y la tierra común. Con '?' en el Monitor Serie, RESET dice la razón.")


def es_arranque(l):
    """¿Esta línea es de la ESP32 arrancando? Su línea OK1-RETRO (aunque llegue con basura adelante,
    como pasa justo después de un reinicio) o los mensajes del arranque y del detector de voltaje."""
    return "OK1-RETRO" in l or "rst:0x" in l or "Brownout" in l or l.startswith("ets ")


def huecos(M):
    """Cuántas muestras faltan en [(t_s, mv), ...] según los huecos en el tiempo: un hueco de 2.0 veces
    el paso normal (la mediana) es una muestra perdida, uno de 3.0 son dos, etc."""
    if len(M) < 3:
        return 0
    dts = sorted(b[0] - a[0] for a, b in zip(M, M[1:]))
    paso = dts[len(dts) // 2]
    if paso <= 0:
        return 0
    return sum(max(0, int(round(dt / paso)) - 1) for dt in dts if dt > 1.5 * paso)


class ESP32:
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
        for k in ("PCA", "FB", "AUX", "DESC", "PROTO", "RESET", "SEG"):
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

    def leer_todos(self):
        """Una lectura de los 12 (comando V). None donde el firmware manda -1."""
        p = self.comando("V", "V ", 1.0).split()[1:]
        return [None if int(x) < 0 else int(x) for x in p]

    def rampa(self, ch, us_a, us_b, vel, vueltas, espera_ms, tolerancia=0.0):
        """Comando G. Espera a que termine y regresa {"total_s", "M": [(t_s, mv)], "C": [(t_s, us)], "perdidas"}.
        `tolerancia`: la fracción de líneas M y de líneas C que se puede perder en el USB sin que sea error
        (0 = ninguna, como en las pruebas 01-07). "perdidas" dice cuántas faltaron de cada una."""
        l = self.comando(f"G {ch} {us_a} {us_b} {vel} {vueltas} {espera_ms}", "OK G ", 2.0)
        total_s = float(l.split()[-1]) / 1000.0
        t_ok = time.time()
        limite = t_ok + total_s + 5.0
        vistas = []                              # lo que llegó y no se entendió (para decir qué pasó)
        primera = None                           # una línea de datos que llegó sin su encabezado
        n_m = n_c = None
        while True:
            l = self._linea(limite)
            if l is None:
                if vistas:
                    raise ErrorESP32((f"la rampa del canal {ch} no terminó a tiempo: en {time.time() - t_ok:.0f} s "
                                      f"(la rampa dura {total_s:.1f} s) llegaron {len(vistas)} líneas que no se "
                                      f"entendieron; las últimas: {vistas[-3:]!r}")[:400])
                raise ErrorESP32(f"la rampa del canal {ch} no terminó a tiempo: después de 'OK G' la ESP32 no "
                                 f"mandó NADA en {time.time() - t_ok:.0f} s (la rampa dura {total_s:.1f} s). "
                                 "Se quedó callada: ¿se colgó, o se cortó el USB?")
            if l.startswith("E "):
                raise ErrorESP32(f"la ESP32 detuvo la rampa: {l}")
            if es_arranque(l):
                raise ErrorESP32(REINICIO + f" Llegó: {l[:70]!r}")
            if l.startswith("G DATOS"):
                try:
                    _, _, c, n_m, n_c = l.split()
                    n_m, n_c = int(n_m), int(n_c)
                except ValueError:
                    n_m = n_c = None             # encabezado dañado: se cuentan los datos sin él
                break
            if l.startswith("M ") or l.startswith("C "):
                primera = l                      # el encabezado se perdió en el USB
                break
            vistas.append(l)
        sin_encabezado = n_m is None
        if sin_encabezado:
            if not tolerancia:
                raise ErrorESP32("se perdió el encabezado de los datos de la rampa (G DATOS) en el USB")
            # lo que manda el firmware: un comando cada 20 ms, una muestra cada 2 ms (FL) o una por línea
            # de la auxiliar; sin el encabezado, los que faltan se cuentan por los huecos en el tiempo.
            n_c = int(math.ceil(total_s * 1000 / 20 - 1e-6))
        # 115200 baud = ~11.5 kB/s; cada línea mide ~14 bytes. Margen de 2x + 5 s.
        limite = time.time() + 2 * ((n_m or n_c * 10) + n_c) * 14 / 11520 + 5.0
        M, C = [], []
        malas = raras = 0
        while True:
            l, primera = (primera, None) if primera else (self._linea(limite), None)
            if l is None:
                raise ErrorESP32(f"se cortaron los datos de la rampa ({len(M)}/{n_m or '?'} M, {len(C)}/{n_c} C)")
            if l.startswith("M ") or l.startswith("C "):
                try:
                    k, t, x = l.split()
                    (M if k == "M" else C).append((int(t) / 1e4, int(x)))
                except ValueError:
                    malas += 1                       # línea dañada en el USB
            elif l.startswith("G FIN"):
                break
            elif l.startswith("E "):
                raise ErrorESP32(f"la ESP32 contestó durante los datos: {l}")
            elif es_arranque(l):
                raise ErrorESP32(REINICIO + f" Llegó: {l[:70]!r}")
            elif l:
                raras += 1                           # p. ej. una M a la que se le dañó la primera letra
        if sin_encabezado:
            faltan_m = huecos(M)
            n_m = len(M) + faltan_m
            faltan_c = max(0, n_c - len(C))
            n_c = max(n_c, len(C))
        else:
            faltan_m, faltan_c = n_m - len(M), n_c - len(C)
        if (faltan_m < 0 or faltan_c < 0 or faltan_m > tolerancia * n_m or faltan_c > tolerancia * n_c
                or (tolerancia == 0 and malas)):
            raise ErrorESP32(f"datos de la rampa incompletos: {len(M)}/{n_m} M, {len(C)}/{n_c} C, "
                             f"{malas} líneas dañadas, {raras} líneas raras"
                             + (" (y se perdió el encabezado)" if sin_encabezado else ""))
        return {"total_s": total_s, "M": M, "C": C,
                "perdidas": {"M": faltan_m, "C": faltan_c, "danadas": malas, "raras": raras,
                             "encabezado": int(sin_encabezado)}}

    def mover_todos(self, us):
        """Comando S: 12 pulsos (-1 = no cambiar, 0 = soltar). Regresa las 12 lecturas (None = sin)."""
        if len(us) != 12:
            raise ValueError("S necesita 12 pulsos")
        p = self.comando("S " + " ".join(str(int(u)) for u in us), "S ", 1.0).split()[1:]
        return [None if int(x) < 0 else int(x) for x in p]

    def trayectoria(self, canales, puntos, grabar=True):
        """Comando T. `puntos` = lista de listas de µs (una por punto, un valor por canal, en el orden
        de `canales`); un punto cada 20 ms. Regresa {"canales", "M": [(t_s, [mv...])],
        "C": [(t_s, [us...])]} (vacíos si grabar=False)."""
        canales = sorted(canales)
        mascara = sum(1 << c for c in canales)
        self.comando(f"T INICIO {mascara} {len(puntos)}", "OK T INICIO", 1.0)
        for i in range(0, len(puntos), 32):                  # 32 puntos y esperar "OK K"
            bloque = puntos[i:i + 32]
            self.s.write("".join("K " + " ".join(str(int(u)) for u in p) + "\n" for p in bloque).encode())
            self.comando_sin_enviar(f"OK K {i + len(bloque)}", 2.0)
        l = self.comando("T CORRER" + ("" if grabar else " 0"), "OK T CORRER", 1.0)
        total_s = float(l.split()[-1]) / 1000.0
        limite = time.time() + total_s + 5.0
        while True:
            l = self._linea(limite)
            if l is None:
                raise ErrorESP32("la trayectoria no terminó a tiempo y la ESP32 dejó de contestar: o se reinició "
                                 "sin alcanzar a avisar, o Windows perdió la conexión USB (desconecta y conecta el "
                                 "USB, y revisa con '?' en el Monitor Serie qué dice RESET y SEG)")
            if l.startswith("E "):
                raise ErrorESP32(f"la ESP32 detuvo la trayectoria: {l}")
            if l.startswith("OK1-RETRO"):
                raise ErrorESP32(REINICIO)
            if not grabar and l.startswith("T FIN"):
                return {"canales": canales, "M": [], "C": [], "total_s": total_s}
            if l.startswith("T DATOS"):
                break
        p = l.split()
        n_c, n_f, n_p = int(p[2]), int(p[3]), int(p[4])
        canales_fw = [int(x) for x in p[5:5 + n_c]]
        if canales_fw != canales:
            raise ErrorESP32(f"la ESP32 grabó otros canales: {canales_fw}")
        limite = time.time() + 2 * (n_f * (8 + 5 * n_c) + n_p * (8 + 5 * n_c)) / 11520 + 5.0
        M, C, malas = [], [], 0
        while True:
            l = self._linea(limite)
            if l is None:
                raise ErrorESP32(f"se cortaron los datos de la trayectoria ({len(M)}/{n_f} M, {len(C)}/{n_p} C)")
            if l.startswith("M ") or l.startswith("C "):
                try:
                    v = [int(x) for x in l.split()[1:]]
                    if len(v) != n_c + 1:
                        raise ValueError
                    (M if l[0] == "M" else C).append((v[0] / 1e4, v[1:]))
                except ValueError:
                    malas += 1
            elif l.startswith("T FIN"):
                break
            elif l.startswith("E "):
                raise ErrorESP32(f"la ESP32 contestó durante los datos: {l}")
        if malas or len(M) != n_f or len(C) != n_p:
            raise ErrorESP32(f"datos de la trayectoria incompletos: {len(M)}/{n_f} M, {len(C)}/{n_p} C, "
                             f"{malas} líneas dañadas")
        return {"canales": canales, "M": M, "C": C, "total_s": total_s}

    def comando_sin_enviar(self, prefijo, timeout=1.0):
        """Espera una línea que empiece con `prefijo` (sin mandar nada)."""
        limite = time.time() + timeout
        while True:
            l = self._linea(limite)
            if l is None:
                raise ErrorESP32(f"la ESP32 no contestó '{prefijo}'")
            if l.startswith(prefijo):
                return l
            if l.startswith("E "):
                raise ErrorESP32(f"la ESP32 contestó: {l}")

    def soltar(self):
        """D. Regresa True si la ESP32 contestó OK D (los servos quedaron sin fuerza), False si no."""
        try:
            self.comando("D", "OK D", 1.0)
            return True
        except (ErrorESP32, OSError):
            return False

    def esperar(self, s):
        time.sleep(s)

    def cerrar(self):
        try:
            self.s.close()
        except Exception:
            pass
