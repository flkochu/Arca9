"""
Comandos de la ESP32 principal de Arca9, para los scripts de la compu
======================================================================
El protocolo está en docs/hardware.md (USB a 115200 baud, una línea por comando):

    P canal us   -> "OK P canal us"   (us = pulso que sí aplicó el firmware, 500–2360)
    R canal n    -> "R canal n media sigma min max"   (mV del pin)
    V            -> "V mv0 ... mv11"  (-1 = sin lectura)
    D            -> "OK D"            (suelta todos los servos)
    ?            -> "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=2"
    G canal us_a us_b vel vueltas espera_ms   -> rampa hecha por la ESP32 (protocolo v2);
                 al terminar manda "G DATOS", las líneas M (t, mV) y C (t, µs) y "G FIN"

eficiencia_servos.py y angulo_real.py tienen su propia copia de esto (se escribieron antes);
los scripts nuevos usan este archivo.
"""
import re
import time


class ErrorESP32(RuntimeError):
    pass


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
        for k in ("PCA", "FB", "AUX", "DESC", "PROTO"):
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

    def rampa(self, ch, us_a, us_b, vel, vueltas, espera_ms):
        """Comando G. Espera a que termine y regresa {"total_s", "M": [(t_s, mv)], "C": [(t_s, us)]}."""
        l = self.comando(f"G {ch} {us_a} {us_b} {vel} {vueltas} {espera_ms}", "OK G ", 2.0)
        total_s = float(l.split()[-1]) / 1000.0
        limite = time.time() + total_s + 5.0
        while True:
            l = self._linea(limite)
            if l is None:
                raise ErrorESP32(f"la rampa del canal {ch} no terminó a tiempo")
            if l.startswith("E "):
                raise ErrorESP32(f"la ESP32 detuvo la rampa: {l}")
            if l.startswith("G DATOS"):
                break
        _, _, c, n_m, n_c = l.split()
        n_m, n_c = int(n_m), int(n_c)
        # 115200 baud = ~11.5 kB/s; cada línea mide ~14 bytes. Margen de 2x + 5 s.
        limite = time.time() + 2 * (n_m + n_c) * 14 / 11520 + 5.0
        M, C = [], []
        malas = 0
        while True:
            l = self._linea(limite)
            if l is None:
                raise ErrorESP32(f"se cortaron los datos de la rampa ({len(M)}/{n_m} M, {len(C)}/{n_c} C)")
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
        if malas or len(M) != n_m or len(C) != n_c:
            raise ErrorESP32(f"datos de la rampa incompletos: {len(M)}/{n_m} M, {len(C)}/{n_c} C, "
                             f"{malas} líneas dañadas")
        return {"total_s": total_s, "M": M, "C": C}

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
