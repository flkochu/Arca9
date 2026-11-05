#!/usr/bin/env python3
"""
Puente entre Arca9 y Agmen
==========================
Un nodo de Agmen que habla con la ESP32 principal por USB con el comando S (protocolo v3,
docs/hardware.md): a ~50 Hz manda los 12 pulsos y recibe las 12 lecturas de los cables verdes.

    /arca9/comando        (Arca9Comando)         lo escucha: 12 ángulos objetivo, en grados reales
    /arca9/articulaciones (Arca9Articulaciones)  lo publica: las 12 lecturas, en grados reales
    /arca9/estado         (Arca9Estado)          lo publica (10 Hz): ESP32, modo, errores

Usa la clase ESP32 de pc/arca9_serial.py para hablar con la placa y pc/escala.py para pasar de
grados reales a µs y de mV a grados reales.

Seguridad (S no hace rampas: quien lo usa tiene que mandar cambios chicos):
  - Límites: cada servo se recorta a los de su pata (pc/pata.py: los medidos menos 5°, y la
    rodilla sin pasar de recta). Un servo sin pata medida no se mueve.
  - Velocidad: el pulso de cada servo avanza a lo más --vel-max °/s (por defecto 60, tope 120,
    muy por debajo de los 240 °/s de la prueba 04). Con 50 Hz son 1.2° por ciclo.
  - Arranque: cada servo empieza desde donde lo mide su cable verde (no da saltos), y se prenden
    uno por ciclo (12 en 0.24 s), para no pedirle a la fuente los 12 picos a la vez. Un servo sin
    lectura creíble no se prende.
  - Watchdog: si no llegan comandos en 0.5 s, se queda quieto donde va (no los suelta).
  - Reinicio de la ESP32: lo detecta por su línea de arranque (OK1-RETRO a media corrida) o porque
    SEG regresó (un '?' cada segundo). Lo publica, imprime el aviso REINICIO de arca9_serial.py y
    deja los servos sueltos (la ESP32 ya los soltó al arrancar). Para seguir, reinicia el nodo
    puente (↻ en el panel).
  - Al cerrar (Ctrl+C, ■ en el panel o agmen run que termina) manda D: suelta los servos.

Uso (en la carpeta de este archivo; normalmente lo lanza `agmen run robot.toml`):
    python puente.py --port COM6                  # con el perro
    python puente.py --port COM6 --vel-max 20     # primera prueba: despacio
    python puente.py --simulado                   # sin el perro: una ESP32 falsa (esp32_falsa.py)
    python puente.py --simulado --reinicio-en 20  # ...que se reinicia a los 20 s (ensayar qué hacer)
"""
import argparse
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from perro import Perro, ErrorPerro, cargar_agmen, mv_creible, NOMBRES, PATAS, CANALES   # noqa: E402
from arca9_serial import ESP32, ErrorESP32, REINICIO                                      # noqa: E402
from escala import Escala, Retro, N_SERVOS                                                # noqa: E402

HZ = 50.0
VEL_MAX = 60.0                  # °/s por defecto
VEL_TOPE = 120.0                # °/s: --vel-max no puede pasar de aquí (la prueba 04 llegó a 240 sin saturar)
WATCHDOG_S = 0.5
TIMEOUT_S = 0.10                # para la respuesta de un S (~11 ms normalmente)
CADA_ESTADO_S = 1.0             # un '?' cada segundo, para ver si SEG regresó
SIN_ESP32_TRAS = 3              # respuestas seguidas que no llegan = la ESP32 no contesta
RAZONES = {"encendido": "ENCENDIDO", "voltaje_bajo": "VOLTAJE_BAJO", "falla": "FALLA", "watchdog": "WATCHDOG",
           "software": "SOFTWARE", "externo": "EXTERNO", "otro": "OTRO"}


class Reinicio(ErrorESP32):
    """La ESP32 se reinició mientras corría el puente."""

    def __init__(self, info, como):
        super().__init__(REINICIO)
        self.info, self.como = info, como


class SinRespuesta(ErrorESP32):
    pass


def leer_info(l):
    """Los campos de una línea OK1-RETRO, como ESP32.estado()."""
    info = {"linea": l}
    for k in ("PCA", "FB", "AUX", "DESC", "PROTO", "RESET", "SEG"):
        m = re.search(k + r"=(\w+)", l)
        if m:
            info[k.lower()] = int(m.group(1)) if m.group(1).isdigit() else m.group(1)
    return info


class ESP32Puente(ESP32):
    """La ESP32 de arca9_serial.py, más el ciclo de S para el control en tiempo real: timeout
    corto, no confunde una respuesta tardía con la de ahora y se da cuenta si la placa se reinicia.
    Con `puerto_falso` (esp32_falsa.ESP32Falsa) no abre ningún puerto."""

    def __init__(self, port=None, puerto_falso=None):
        if puerto_falso is None:
            super().__init__(port)              # abre el puerto, espera 2 s y pregunta '?'
        else:
            self.s = puerto_falso
            self.s.reset_input_buffer()
            self.buf = b""
            self.info = self.estado()
        self.ultimo_seg = self.info.get("seg", 0)
        self.estados_pendientes = 0             # '?' que no contestaron a tiempo (su respuesta puede llegar después)
        self.tardias = 0                        # respuestas que llegaron tarde y se tiraron
        self.danadas = 0

    def _arranque(self, l, como):
        """Llegó una línea OK1-RETRO sin pedirla: o es la respuesta tardía de un '?' o la ESP32 arrancó."""
        info = leer_info(l)
        seg = info.get("seg", 0)
        if self.estados_pendientes and seg >= self.ultimo_seg:
            self.estados_pendientes -= 1
            self.ultimo_seg = seg
            return
        self.ultimo_seg = seg
        raise Reinicio(info, como)

    def _pendientes(self):
        """Lo que llegó sin pedirlo, antes de mandar algo nuevo."""
        n = self.s.in_waiting
        if n:
            self.buf += self.s.read(n)
        while b"\n" in self.buf:
            l, self.buf = self.buf.split(b"\n", 1)
            l = l.decode("utf-8", "replace").strip()
            if l.startswith("OK1-RETRO"):
                self._arranque(l, "mandó su línea de arranque")
            elif l:
                self.tardias += 1

    def ciclo(self, us, timeout=TIMEOUT_S):
        """S us0 ... us11 -> 12 lecturas en mV (None = sin lectura)."""
        self._pendientes()
        self.s.write(("S " + " ".join(str(int(u)) for u in us) + "\n").encode())
        limite = time.time() + timeout
        while True:
            l = self._linea(limite)
            if l is None:
                raise SinRespuesta("la ESP32 no contestó a S")
            if l.startswith("OK1-RETRO"):
                self._arranque(l, "mandó su línea de arranque")
            elif l.startswith("S "):
                p = l.split()[1:]
                try:
                    v = [int(x) for x in p]
                    if len(v) != N_SERVOS:
                        raise ValueError
                except ValueError:
                    self.danadas += 1
                    raise ErrorESP32(f"respuesta dañada a S: {l[:60]!r}")
                return [None if x < 0 else x for x in v]
            elif l.startswith("E "):
                raise ErrorESP32(f"la ESP32 contestó a S: {l}")
            elif l:
                self.danadas += 1                # una línea rara (dañada en el USB)

    def consultar(self, timeout=0.15):
        """'?' rápido. Regresa la info, o None si no contestó. Reinicio si SEG regresó."""
        self._pendientes()
        self.s.write(b"?\n")
        limite = time.time() + timeout
        while True:
            l = self._linea(limite)
            if l is None:
                self.estados_pendientes += 1
                return None
            if l.startswith("OK1-RETRO"):
                info = leer_info(l)
                seg = info.get("seg", 0)
                if seg < self.ultimo_seg:
                    self.ultimo_seg = seg
                    raise Reinicio(info, f"su SEG regresó a {seg} s")
                self.ultimo_seg = seg
                return info
            if l.startswith("S "):
                self.tardias += 1


class Puente:
    """La lógica del puente, sin Agmen: lo que se manda en cada ciclo y lo que se publica.
    `reloj` se puede cambiar en las pruebas."""

    def __init__(self, esp, esc, retro, perro, vel_max=VEL_MAX, hz=HZ, watchdog_s=WATCHDOG_S,
                 reloj=time.monotonic, log=print):
        if not 0 < vel_max <= VEL_TOPE:
            raise ValueError(f"--vel-max debe estar entre 0 y {VEL_TOPE:g} °/s")
        self.esp, self.esc, self.retro, self.perro = esp, esc, retro, perro
        self.vel_max, self.periodo, self.watchdog_s = float(vel_max), 1.0 / hz, watchdog_s
        self.reloj, self.log = reloj, log
        self.actual = [None] * N_SERVOS          # grados reales del último pulso mandado (None = no lo maneja)
        self.medido = [math.nan] * N_SERVOS      # última lectura, en grados reales
        self.cmd_q = None
        self.cmd_activo = None
        self.t_cmd = None
        self.t_ciclo = None
        self.t_estado = reloj()
        self.limitado = [False] * N_SERVOS
        self.bloqueado = [False] * N_SERVOS
        self.modo = "SUELTO"
        self.conectado = True
        self.info = dict(esp.info)
        self.reinicios = self.errores = self.ciclos = 0
        self.seguidas_sin = 0
        self.ciclo_ms = 0.0
        self.peor_ms = 0.0
        self.avisados = set()

    # ---- entradas
    def comando(self, q, activo):
        self.cmd_q, self.cmd_activo, self.t_cmd = list(q), list(activo), self.reloj()
        if not any(activo):
            self._avisar_una_vez("sin_activo", "llegó un comando sin ningún servo activo (activo = false en los 12): "
                                               "no se mueve nada")

    def _avisar_una_vez(self, clave, texto):
        if clave not in self.avisados:
            self.avisados.add(clave)
            self.log(f"⚠ {texto}")

    def _cambiar_modo(self, modo, texto=None):
        if modo != self.modo:
            self.modo = modo
            if texto:
                self.log(texto)

    # ---- un ciclo
    def pulsos(self):
        """Los 12 pulsos del ciclo (-1 = no cambiar). Actualiza modo, limitado y bloqueado."""
        ahora = self.reloj()
        dt = self.periodo if self.t_ciclo is None else min(max(ahora - self.t_ciclo, 0.0), 2 * self.periodo)
        self.t_ciclo = ahora
        us = [-1] * N_SERVOS
        self.limitado = [False] * N_SERVOS
        if self.modo in ("REINICIADA",) or not self.conectado:
            return us
        if self.t_cmd is None or self.ciclos == 0:     # sin comandos, o sin la primera lectura todavía
            self._cambiar_modo("SUELTO")
            return us
        if ahora - self.t_cmd > self.watchdog_s:
            self._cambiar_modo("QUIETO", f"⛔ watchdog: sin comandos desde hace {self.watchdog_s:g} s → quieto "
                                         "(los servos se quedan donde van)")
            return us
        self._cambiar_modo("SIGUIENDO", "▶ siguiendo /arca9/comando" if self.modo != "SUELTO" else
                           f"▶ primer comando: los servos se prenden uno por ciclo desde donde están "
                           f"(a lo más {self.vel_max:g} °/s)")
        paso_max = self.vel_max * dt
        prendidos = 0
        bloq = [False] * N_SERVOS
        for ch in range(N_SERVOS):
            if not self.cmd_activo[ch]:
                continue
            q = self.cmd_q[ch]
            if ch not in self.perro.lim:
                bloq[ch] = True
                n = next(p for p in PATAS if ch in CANALES[p])
                self._avisar_una_vez(f"sinpata{n}", f"el comando pide mover la pata {NOMBRES[n]} y no tiene límites "
                                                    f"medidos (falta datos/*/pata_{n}.json): no se mueve")
                continue
            if not math.isfinite(q):
                bloq[ch] = True
                self._avisar_una_vez(f"nan{ch}", f"el comando trae NaN en el canal {ch}: ese servo no se mueve")
                continue
            if self.actual[ch] is None:
                if not math.isfinite(self.medido[ch]):
                    bloq[ch] = True
                    self._avisar_una_vez(f"sinlectura{ch}", f"el canal {ch} no tiene lectura creíble del cable verde: "
                                                            "no se prende")
                    continue
                if prendidos:                       # uno por ciclo
                    continue
                self.actual[ch] = self.medido[ch]
                prendidos += 1
            obj = self.perro.recortar(ch, q)
            self.limitado[ch] = abs(obj - q) > 1e-6
            paso = max(-paso_max, min(paso_max, obj - self.actual[ch]))
            self.actual[ch] += paso
            us[ch] = self.esc.real_a_us(self.actual[ch])
        self.bloqueado = bloq
        return us

    def ciclo(self):
        """Un S. Regresa True si cambió algo que valga publicar el estado ya (modo, reinicio)."""
        modo_antes, conectado_antes = self.modo, self.conectado
        us = self.pulsos()
        t0 = time.perf_counter()
        try:
            mv = self.esp.ciclo(us)
            self.ciclo_ms = (time.perf_counter() - t0) * 1000
            self.peor_ms = max(self.peor_ms, self.ciclo_ms)
            self.ciclos += 1
            self.seguidas_sin = 0
            if not self.conectado:
                self.conectado = True
                self.log("✔ la ESP32 volvió a contestar")
                self._consultar()                  # ¿se reinició mientras no contestaba?
            self.medido = [self.retro.grados(ch, m) if mv_creible(m) else math.nan for ch, m in enumerate(mv)]
            if self.reloj() - self.t_estado >= CADA_ESTADO_S:
                self._consultar()
        except Reinicio as r:
            self._reinicio(r)
        except SinRespuesta:
            self.errores += 1
            self.seguidas_sin += 1
            if self.seguidas_sin >= SIN_ESP32_TRAS and self.conectado:
                self.conectado = False
                self.modo = "SIN_ESP32"
                self.log(f"⚠ la ESP32 no contesta ({self.seguidas_sin} S seguidos sin respuesta): no se manda "
                         "ningún pulso nuevo hasta que conteste")
        except ErrorESP32 as e:
            self.errores += 1
            self._avisar_cada(f"⚠ {e}")
        return self.modo != modo_antes or self.conectado != conectado_antes

    def _avisar_cada(self, texto, cada=1.0):
        t = self.reloj()
        if t - getattr(self, "_ultimo_aviso", -1e9) >= cada:
            self._ultimo_aviso = t
            self.log(texto)

    def _consultar(self):
        self.t_estado = self.reloj()
        try:
            info = self.esp.consultar()
        except Reinicio as r:
            self._reinicio(r)
            return
        if info is None:
            self.errores += 1
        else:
            self.info.update(info)
            if info.get("aux") != "ok":
                self._avisar_una_vez("aux", "la auxiliar no manda datos (AUX=NO): sin lecturas de FR, RL y RR")

    def _reinicio(self, r):
        self.reinicios += 1
        self.info.update(r.info)
        self.actual = [None] * N_SERVOS
        self.modo = "REINICIADA"
        self.log(f"⛔ la ESP32 se reinició ({r.como}; RESET={r.info.get('reset', '?')}).")
        self.log(f"   {REINICIO}")
        self.log("   Los servos quedan sueltos. Para seguir, revisa la fuente y reinicia el nodo puente (↻ en el panel).")

    # ---- salidas
    def estado(self, msgs):
        return msgs.Arca9Estado(
            conectado=self.conectado, proto=int(self.info.get("proto", 0)),
            reset=msgs.Reset[RAZONES.get(self.info.get("reset"), "OTRO")] if self.info.get("reset") else msgs.Reset.DESCONOCIDO,
            seg=int(self.info.get("seg", 0)), aux=self.info.get("aux") == "ok", modo=msgs.Modo[self.modo],
            reinicios=self.reinicios, errores=self.errores, ciclos=self.ciclos, ciclo_ms=self.ciclo_ms,
            vel_max=self.vel_max, limitado=self.limitado, bloqueado=self.bloqueado)

    def cerrar(self):
        """Manda D (suelta los servos) y cierra el puerto."""
        try:
            self.esp.soltar()
            self.log("■ servos sueltos (D)")
        except Exception as e:                    # noqa: BLE001 — al cerrar, que no truene
            self.log(f"⚠ no se pudo mandar D: {e}")
        self.esp.cerrar()


def revisar_proto(info):
    """Texto del problema si la ESP32 no tiene el comando S (PROTO < 3), o None."""
    proto = info.get("proto", 1)
    if not isinstance(proto, int) or proto < 3:
        return ("la principal no tiene el comando S (falta PROTO=3 en '?'). Carga el esp32_principal.ino "
                "nuevo (firmware/esp32_principal).")
    return None


def conectar(a, esc, retro, perro):
    if a.simulado:
        from esp32_falsa import ESP32Falsa
        inicio = [90.0] * N_SERVOS
        for n, p in perro.patas.items():                  # colgando, cerca de la pose de pie
            for ch, g in zip(p.canales, p.de_pie_servo):
                inicio[ch] = g + 4.0
        falsa = ESP32Falsa(esc, retro, inicio=inicio, semilla=a.semilla)
        if a.reinicio_en:
            falsa.reinicio_en = time.monotonic() + a.reinicio_en
        return ESP32Puente(puerto_falso=falsa)
    return ESP32Puente(a.port)


def main():
    agmen, msgs = cargar_agmen()
    ap = argparse.ArgumentParser(description="Puente entre la ESP32 principal de Arca9 y Agmen (comando S)")
    ap.add_argument("--port", help="puerto de la ESP32 principal (ej. COM6 o /dev/ttyUSB0)")
    ap.add_argument("--simulado", action="store_true", help="sin el perro: una ESP32 falsa y las 4 patas copiadas de la FL")
    ap.add_argument("--vel-max", type=float, default=VEL_MAX,
                    help=f"°/s máximos de cada servo (default {VEL_MAX:g}, tope {VEL_TOPE:g})")
    ap.add_argument("--hz", type=float, default=HZ, help=f"ciclos de S por segundo (default {HZ:g})")
    ap.add_argument("--comando", default="/arca9/comando", help="topic de los comandos (default /arca9/comando)")
    ap.add_argument("--reinicio-en", type=float, metavar="SEG",
                    help="solo con --simulado: la ESP32 falsa se reinicia (voltaje_bajo) a los SEG segundos, para "
                         "ver en el panel qué pasa")
    ap.add_argument("--semilla", type=int, default=7, help=argparse.SUPPRESS)
    a = ap.parse_args()
    if not a.simulado and not a.port:
        ap.error("falta --port (o usa --simulado)")
    if a.reinicio_en and not a.simulado:
        ap.error("--reinicio-en es solo para --simulado")
    if not 0 < a.vel_max <= VEL_TOPE:
        ap.error(f"--vel-max debe estar entre 0 y {VEL_TOPE:g} °/s (S no hace rampas: el puente tiene que mandar "
                 "cambios chicos)")
    if not 5 <= a.hz <= 80:
        ap.error("--hz debe estar entre 5 y 80 (un S de ida y vuelta tarda ~11 ms)")

    def log(t):
        print(f"[puente] {t}", flush=True)

    esc = Escala()
    retro = Retro(esc)
    try:
        perro = Perro(simulado=a.simulado)
    except ErrorPerro as e:
        sys.exit(f"[puente] {e}")
    log(f"Agmen {agmen.__version__} · {'ESP32 FALSA (--simulado)' if a.simulado else a.port} · {a.hz:g} Hz · "
        f"hasta {a.vel_max:g} °/s · watchdog {WATCHDOG_S:g} s")
    log(f"escala: {esc.fuente} · retroalimentación: {retro.fuente}")
    for l in perro.describir():
        log("  " + l)
    if not perro.patas:
        log("⚠ ninguna pata medida: el puente solo va a publicar las lecturas")
    try:
        esp = conectar(a, esc, retro, perro)
    except ErrorESP32 as e:
        sys.exit(f"[puente] {e}")
    log(f"ESP32: {esp.info['linea']}")
    problema = revisar_proto(esp.info)
    if problema:
        esp.cerrar()
        sys.exit(f"[puente] {problema}")

    puente = Puente(esp, esc, retro, perro, a.vel_max, a.hz, log=log)
    node = agmen.Node("puente")
    pub_q = node.publisher(msgs.Arca9Articulaciones, "/arca9/articulaciones")
    pub_estado = node.publisher(msgs.Arca9Estado, "/arca9/estado")
    cuenta = {"n": 0, "t": time.monotonic(), "ciclos": 0}
    salida = {"codigo": 0}

    @node.on(msgs.Arca9Comando, a.comando)
    def comando(s):
        puente.comando(s.message.q, s.message.activo)

    @node.every(1.0 / a.hz)
    def ciclo():
        try:
            urgente = puente.ciclo()
        except OSError as e:                       # el puerto USB desapareció
            log(f"⛔ se perdió el puerto: {e}")
            puente.conectado = False
            pub_estado.publish(puente.estado(msgs))
            salida["codigo"] = 1
            node.stop()
            return
        pub_q.publish(msgs.Arca9Articulaciones(q=puente.medido))
        cuenta["n"] += 1
        if urgente or cuenta["n"] % max(1, round(a.hz / 10)) == 0:
            pub_estado.publish(puente.estado(msgs))
        t = time.monotonic()
        if t - cuenta["t"] >= 10.0:
            hz = (puente.ciclos - cuenta["ciclos"]) / (t - cuenta["t"])
            log(f"{hz:.1f} Hz · S tarda {puente.ciclo_ms:.1f} ms (peor {puente.peor_ms:.1f}) · modo {puente.modo} · "
                f"errores {puente.errores} · reinicios {puente.reinicios}")
            cuenta["t"], cuenta["ciclos"] = t, puente.ciclos
            puente.peor_ms = 0.0

    log("listo: publico /arca9/articulaciones y /arca9/estado; escucho " + a.comando)
    try:
        node.spin()
    finally:
        puente.cerrar()
        node.close()
    sys.exit(salida["codigo"])


if __name__ == "__main__":
    main()
