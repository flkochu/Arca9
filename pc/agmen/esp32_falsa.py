"""
ESP32 falsa: se hace pasar por el puerto serie de la ESP32 principal
=====================================================================
Para correr el puente sin el perro (puente.py --simulado) y para las pruebas
(pruebas/puente_agmen/). No reemplaza a la clase ESP32 de pc/arca9_serial.py: reemplaza al
puerto. La clase ESP32 le escribe y le lee como al USB de verdad, así que el código que se prueba
es el mismo que habla con el perro.

Contesta lo que usa el puente, como el firmware (docs/hardware.md, protocolo v3):
    ?                 "OK1-RETRO listo PCA=ok FB=12 AUX=ok DESC=0 PROTO=3 RESET=encendido SEG=12"
                      + el diagnóstico del firmware del 9 nov: "DIAG=1 NS=.. SMAX=.. LINMAX=.. VUELTAMAX=..
                      I2CERR=.. I2CCOD=.. I2CMAX=.." (sin él con diag=False, como un firmware viejo)
    S us0 ... us11    "S mv0 ... mv11" (-1 = no cambiar, 0 = soltar, recorta a 500–2360 µs)
    V                 "V mv0 ... mv11"
    P canal us        "OK P canal us"
    D                 "OK D"
Cada servo va como en la prueba 04: ~100 ms atrás del pulso (70 ms de tiempo muerto y un primer
orden de 30 ms), hasta 300 °/s. Un servo suelto no se mueve. Las lecturas salen de la misma
tabla de la prueba 01 que usa el puente (al revés), con ~1.7 mV de ruido.

Las respuestas tardan lo que tardarían a 115200 baud (~11 ms un S de ida y vuelta), salvo con
latencia=False (pruebas con un reloj que no avanza solo).

Para provocar fallas (las pruebas):
    reiniciar("voltaje_bajo")       se reinicia como con una caída de voltaje: suelta los servos,
                                    SEG vuelve a 0 y manda su línea de arranque (OK1-RETRO ...)
    reiniciar(..., aviso=False)     igual, pero la línea de arranque se pierde
    callar(n)                       no contesta los siguientes n comandos (se pierden: no cuentan en NS)
    colgar_i2c(n, s)                la escritura de I²C de los siguientes n S tarda s segundos y falla
                                    (código 5, como el timeout de Wire): la respuesta llega s tarde y
                                    SMAX, I2CMAX e I2CERR lo muestran
    danar(n)                        daña las siguientes n respuestas
    tardar(n, s)                    las siguientes n respuestas llegan s segundos tarde
    proto = 2                       un firmware viejo, sin el comando S
    reinicio_en = t                 se reinicia (voltaje_bajo) con el primer comando después de t
                                    (reloj de la falsa); lo usa puente.py --simulado --reinicio-en
"""
import bisect
import random
import time

US_MIN, US_MAX = 500, 2360
MUERTO, TAU, VMAX = 0.070, 0.030, 300.0        # s, s, °/s (prueba 04)
BAUD_BYTES_S = 11520.0


class ESP32Falsa:
    def __init__(self, esc, retro, inicio=None, semilla=7, reloj=time.monotonic, latencia=True, aux=True,
                 diag=True):
        """`inicio` = 12 grados reales donde están los servos al empezar (sueltos)."""
        self.esc, self.retro, self.reloj, self.latencia, self.aux = esc, retro, reloj, latencia, aux
        self.rng = random.Random(semilla)
        self.pos = list(inicio) if inicio else [90.0 + 10 * self.rng.random() for _ in range(12)]
        self.us = [0] * 12                       # 0 = suelto
        self.hist = [[] for _ in range(12)]      # (t, us) de cada pulso aplicado
        self.t_sim = reloj()
        self.t_arranque = reloj() - 2.0          # al abrir el puerto se reinicia; el puente espera ~2 s
        self.razon = "encendido"
        self.proto = 3
        self.timeout = 0.02
        self.entrada = b""
        self.salida = []                         # [(t en que ya llegó a la compu, bytes)]
        self.recibidas = []                      # cada línea que llegó, para las pruebas
        self.n_callar = self.n_danar = self.n_tardar = 0
        self.s_tardar = 0.0
        self.cerrado = False
        self.reinicio_en = None
        self.diag = diag
        self.n_colgar, self.s_colgar = 0, 0.0
        self._diag_cero()
        self._mandar(self._estado())             # su mensaje de arranque (el puente lo tira al conectar)

    # ---------------------------------------------------------------- como un serial.Serial
    @property
    def in_waiting(self):
        ahora = self.reloj()
        return sum(len(b) for t, b in self.salida if t <= ahora)

    def write(self, datos):
        if self.cerrado:
            raise OSError("puerto cerrado")
        if self.reinicio_en is not None and self.reloj() >= self.reinicio_en:
            self.reinicio_en = None
            self.reiniciar("voltaje_bajo")
        self.entrada += bytes(datos)
        while b"\n" in self.entrada:
            linea, self.entrada = self.entrada.split(b"\n", 1)
            self._atender(linea.decode("utf-8", "replace").strip(), len(linea) + 1)
        return len(datos)

    def read(self, n=1):
        limite = self.reloj() + self.timeout
        while True:
            ahora = self.reloj()
            listos = [i for i, (t, _) in enumerate(self.salida) if t <= ahora]
            if listos:
                out = b""
                while self.salida and self.salida[0][0] <= ahora and len(out) < n:
                    t, b = self.salida.pop(0)
                    falta = n - len(out)
                    out += b[:falta]
                    if len(b) > falta:
                        self.salida.insert(0, (t, b[falta:]))
                return out
            if not self.latencia or ahora >= limite:
                return b""
            prox = min((t for t, _ in self.salida), default=limite)
            time.sleep(max(0.0, min(prox, limite) - ahora))

    def reset_input_buffer(self):
        self.salida = []

    def close(self):
        self.cerrado = True

    # ---------------------------------------------------------------- fallas
    def reiniciar(self, razon="voltaje_bajo", aviso=True):
        self._avanzar(self.reloj())
        self.us = [0] * 12
        self.hist = [[] for _ in range(12)]
        self.t_arranque = self.reloj()
        self.razon = razon
        self._diag_cero()
        self.entrada = b""
        self.salida = []                         # lo que iba en camino se pierde
        if aviso:
            self._mandar(self._estado())

    def callar(self, n=1):
        self.n_callar += n

    def danar(self, n=1):
        self.n_danar += n

    def tardar(self, n=1, s=0.3):
        self.n_tardar, self.s_tardar = n, s

    def colgar_i2c(self, n=1, s=0.05):
        self.n_colgar, self.s_colgar = n, s

    # ---------------------------------------------------------------- por dentro
    def seg(self):
        return int(self.reloj() - self.t_arranque)

    def _diag_cero(self):
        self.ns = self.i2c_err = self.i2c_cod = 0
        self.smax = self.i2cmax = 0

    def _estado(self):
        l = (f"OK1-RETRO listo PCA=ok FB={12 if self.aux else 3} AUX={'ok' if self.aux else 'NO'} DESC=0 "
             f"PROTO={self.proto} RESET={self.razon} SEG={self.seg()}")
        if self.diag:
            l += (f" DIAG=1 NS={self.ns} SMAX={self.smax} LINMAX={5400 if self.smax else 0} VUELTAMAX=300 "
                  f"I2CERR={self.i2c_err} I2CCOD={self.i2c_cod} I2CMAX={self.i2cmax}")
            self.smax = self.i2cmax = 0
        return l + " (ESP32 FALSA)"

    def _mandar(self, texto, n_entrada=0, extra=0.0):
        datos = (texto + "\n").encode()
        espera = (n_entrada + len(datos)) / BAUD_BYTES_S + 0.0005 if self.latencia else 0.0
        espera += extra
        if self.n_tardar:
            self.n_tardar -= 1
            espera += self.s_tardar
        if self.n_danar:
            self.n_danar -= 1
            datos = datos[: len(datos) // 2] + b"#\xff" + datos[len(datos) // 2 + 3:]
        t = self.reloj() + espera
        if self.salida and self.salida[-1][0] > t:   # el USB no adelanta: lo que salió antes llega antes
            t = self.salida[-1][0]
        self.salida.append((t, datos))

    def _atender(self, l, n_entrada):
        self.recibidas.append(l)
        if not l:
            return
        if self.n_callar:
            self.n_callar -= 1
            return
        p = l.split()
        c = l[0]
        if c == "?":
            self._mandar(self._estado(), n_entrada)
        elif c == "S" and self.proto >= 3:
            try:
                v = [int(x) for x in p[1:13]]
                if len(v) != 12 or any(x < -1 for x in v):
                    raise ValueError
            except ValueError:
                self._mandar("E S: son 12 pulsos (us0 ... us11; -1 = no cambiar, 0 = soltar)", n_entrada)
                return
            self.ns += 1
            i2c_us, extra = (4500 if any(u != -1 for u in v) else 0), 0.0
            if self.n_colgar:
                self.n_colgar -= 1
                extra = self.s_colgar
                i2c_us = int(extra * 1e6)
                self.i2c_err += 1
                self.i2c_cod = 5
            self.i2cmax = max(self.i2cmax, i2c_us)
            self.smax = max(self.smax, i2c_us + 900)
            ahora = self.reloj()
            self._avanzar(ahora)
            for ch, u in enumerate(v):
                if u == -1:
                    continue
                self._aplicar(ch, 0 if u == 0 else min(max(u, US_MIN), US_MAX), ahora)
            self._mandar("S " + " ".join(str(m) for m in self._lecturas()), n_entrada, extra)
        elif c == "V":
            self._avanzar(self.reloj())
            self._mandar("V " + " ".join(str(m) for m in self._lecturas()), n_entrada)
        elif c == "P" and len(p) == 3:
            ch, u = int(p[1]), min(max(int(p[2]), US_MIN), US_MAX)
            self._avanzar(self.reloj())
            self._aplicar(ch, u, self.reloj())
            self._mandar(f"OK P {ch} {u}", n_entrada)
        elif c == "D":
            self._avanzar(self.reloj())
            for ch in range(12):
                self._aplicar(ch, 0, self.reloj())
            self._mandar("OK D", n_entrada)
        else:
            self._mandar("E comando desconocido", n_entrada)

    def _aplicar(self, ch, u, t):
        self.us[ch] = u
        self.hist[ch].append((t, u))

    def _pulso_visto(self, ch, t):
        """El pulso que el servo 've' en t: el último que se aplicó antes de t − MUERTO."""
        h = self.hist[ch]
        i = bisect.bisect_right([x[0] for x in h], t - MUERTO) - 1
        if i < 0:
            return 0
        if i > 0:
            del h[:i]                         # lo más viejo ya no hace falta
        return h[0][1]

    def _avanzar(self, t):
        dt = 0.001
        while self.t_sim + dt <= t:
            self.t_sim += dt
            for ch in range(12):
                u = self._pulso_visto(ch, self.t_sim)
                if u <= 0:
                    continue                          # suelto: no se mueve
                obj = self.esc.us_a_real(u)
                v = max(-VMAX, min(VMAX, (obj - self.pos[ch]) / TAU))
                self.pos[ch] += v * dt
        if t - self.t_sim > 1.0:                     # mucho tiempo sin preguntar
            self.t_sim = t

    def _mv(self, ch):
        c = self.retro.canales[ch]
        conv = self.esc.real_a_conv(self.pos[ch])
        pares = sorted(zip(c["ang"], c["mv"]))
        if len(pares) >= 2 and pares[0][0] <= conv <= pares[-1][0]:
            xs = [a for a, _ in pares]
            i = min(max(bisect.bisect_left(xs, conv), 1), len(xs) - 1)
            (a0, m0), (a1, m1) = pares[i - 1], pares[i]
            base = m0 + (m1 - m0) * (conv - a0) / (a1 - a0) if a1 != a0 else m0
        else:
            base = (conv - c["b"]) / c["a"]
        return int(round(base + self.rng.gauss(0, 1.7)))

    def _lecturas(self):
        return [self._mv(ch) if (ch < 3 or self.aux) else -1 for ch in range(12)]
