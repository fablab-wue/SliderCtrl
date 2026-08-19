# MC_client — UART client for SliderMC (MicroPython + uasyncio).
#
# MC_API surface (duck-typed): start, send/query, motion, config setters,
# getters, set_status_callback — same names for a future RS485 client.
# Wire protocol: https://github.com/fablab-wue/SliderDoc/blob/main/contract/protocol.md

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

import math
import time

try:
    from machine import UART, Pin
except ImportError:
    UART = None
    Pin = None

import MC_config as cfg


class MC_Client:
    """Standalone UART interface to a SliderMC motion controller (MC_API)."""

    # Match SliderMC McState (motion_api.h).
    MC_STATE_DISABLED = 0
    MC_STATE_IDLE = 1
    MC_STATE_ACCELERATING = 2
    MC_STATE_MOVING = 3
    MC_STATE_DECELERATING = 4
    MC_STATE_HOMING = 5
    MC_STATE_HARD_LIMIT = 6
    MC_STATE_ERROR = 7
    MC_STATE_LOCKED = 8
    # Index = McState; '?' = LOCKED (not emitted on UART status).
    MC_STATE_CHARS = ("D", "I", "A", "M", "B", "H", "L", "E", "?")

    def __init__(self, uart_id=0, tx=None, rx=None, baud=None):
        if UART is None:
            raise RuntimeError("machine.UART not available")
        if tx is None:
            tx = getattr(cfg, "PIN_UART_TX", 16)
        if rx is None:
            rx = getattr(cfg, "PIN_UART_RX", 17)
        if baud is None:
            baud = int(getattr(cfg, "UART_BAUD", 1_000_000))
        self._uart_tx = int(tx)
        self._uart_rx = int(rx)
        self._uart = UART(
            uart_id,
            baudrate=int(baud),
            tx=Pin(int(tx)),
            rx=Pin(int(rx)),
            bits=8,
            parity=None,
            stop=1,
        )
        self._rx_task = None
        self._started = False
        self._rx_buf = b""

        self._banner_event = asyncio.Event()
        self._waiters = {}  # TAG -> list of (Event, result_box)
        self._cg_collect = None  # dict while collecting bare CG dump

        # Assignable callbacks (composition; no subclass required).
        self._status_cb = None
        self._error_cb = None
        self._answer_cb = None

        # Session / status cache
        self._speed_mm_s = None
        self._accel_mm_s2 = None
        self._max_speed_mm_s = None
        self._soft_min = None
        self._soft_max = None
        self._enabled = None

        self._state = None  # I/M/H/L/E/D/...
        self._pos_mm = None
        self._act_speed_mm_s = None
        self._target_mm = None

        self._moving = False
        self._homing = False
        self._at_soft_limit = False
        self._near_soft_limit = False
        self._at_hard_limit = False
        self._drv_error_active = False
        self._act_vel_mm_s = 0.0
        self._prev_act_speed_abs = None
        self._decelerating = False
        self._accelerating = False

        # Public MC config (filled by fetchConfig after banner).
        self.mc_config = {}
        self.max_speed = None
        self.max_accel = None
        self.slider_min = None
        self.slider_max = None
        # Public McState int; LOCKED until first verbose status.
        self.status = self.MC_STATE_LOCKED

        self._motion_task = None

    # --- callbacks ---------------------------------------------------------

    def set_status_callback(self, cb):
        """Register cb(state, pos, speed, accel, target) for verbose `#…` lines."""
        self._status_cb = cb

    def set_error_callback(self, cb):
        """Register cb(code, text) for `!E:` lines."""
        self._error_cb = cb

    def set_answer_callback(self, cb):
        """Register cb(command, answer) for `TAG:value` replies."""
        self._answer_cb = cb

    def on_error(self, code, text):
        """Hook / callback dispatch for `!E:<code> <text>`."""
        cb = self._error_cb
        if cb is not None:
            cb(code, text)

    def on_status(self, state, pos, speed, accel, target):
        """Hook / callback dispatch for compact `#…` status."""
        cb = self._status_cb
        if cb is not None:
            cb(state, pos, speed, accel, target)

    def on_answer(self, command, answer):
        """Hook / callback dispatch for `TAG:value` replies."""
        cb = self._answer_cb
        if cb is not None:
            cb(command, answer)

    # --- lifecycle ---------------------------------------------------------

    async def start(self, banner_timeout_s=3.0):
        """Open RX task, unlock MC with ``\\n``, wait for welcome ``# …``, then ``SV 1``.

        On successful banner, reads MC config via ``CG`` into ``mc_config`` /
        ``max_speed`` / ``max_accel`` / ``slider_min`` / ``slider_max``.
        Seeds session ``SS``/``SA`` from CG init_speed/init_accel when present.
        """
        if self._rx_task is None:
            self._rx_task = asyncio.create_task(self._rx_loop())
        self._banner_event.clear()
        total_ms = int(float(banner_timeout_s) * 1000)
        if total_ms < 1:
            total_ms = 1
        deadline = time.ticks_add(time.ticks_ms(), total_ms)
        got_banner = False
        while not self._banner_event.is_set():
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                break
            self._uart.write(b"\n")
            try:
                await self._wait_event(self._banner_event, 0.1)
                got_banner = True
                break
            except OSError:
                continue
        if not got_banner and not self._banner_event.is_set():
            print(
                "SliderMC banner timeout (check UART wiring / baud) — continuing without MC"
            )
        self._started = True
        await self.send("SV", 1)
        if got_banner or self._banner_event.is_set():
            await self.fetchConfig()
        # Session SS/SA from CG init_speed/init_accel when present (no CS).
        if self._speed_mm_s is not None:
            self._cmd("SS", _fmt_arg(self._speed_mm_s))
        if self._accel_mm_s2 is not None:
            self._cmd("SA", _fmt_arg(self._accel_mm_s2))

    async def fetchConfig(self, settle_ms=150):
        """Send bare ``CG`` and collect all ``CG:key=value`` lines into ``mc_config``."""
        collected = {}
        self._cg_collect = collected
        try:
            self._write_line("CG")
            idle_deadline = time.ticks_add(time.ticks_ms(), int(settle_ms))
            last_count = -1
            while time.ticks_diff(idle_deadline, time.ticks_ms()) > 0:
                await asyncio.sleep_ms(10)
                n = len(collected)
                if n != last_count:
                    last_count = n
                    idle_deadline = time.ticks_add(time.ticks_ms(), int(settle_ms))
        finally:
            self._cg_collect = None

        self.mc_config = dict(collected)
        self.max_speed = _parse_cfg_float(collected.get("max_speed"))
        self.max_accel = _parse_cfg_float(collected.get("max_accel"))
        self.slider_min = _parse_cfg_limit(
            collected.get("slider_min", collected.get("soft_min"))
        )
        self.slider_max = _parse_cfg_limit(
            collected.get("slider_max", collected.get("soft_max"))
        )
        self._soft_min = self.slider_min
        self._soft_max = self.slider_max
        if self.max_speed is not None:
            self._max_speed_mm_s = self.max_speed
        init_speed = _parse_cfg_float(
            collected.get("init_speed", collected.get("speed"))
        )
        init_accel = _parse_cfg_float(
            collected.get("init_accel", collected.get("accel"))
        )
        if init_speed is not None:
            self._speed_mm_s = init_speed
        if init_accel is not None:
            self._accel_mm_s2 = init_accel
        self._refresh_soft_limit_flag()
        return self.mc_config

    async def stop_rx(self):
        """Cancel the RX task (optional shutdown)."""
        t = self._rx_task
        self._rx_task = None
        if t is not None:
            t.cancel()
            try:
                await t
            except asyncio.CancelledError:
                pass

    def _cmd(self, command, arg=None):
        """Fire-and-forget MC line (sync; preserves UART order)."""
        if arg is None:
            self._write_line(command)
        else:
            self._write_line("%s %s" % (command, arg))

    # --- raw send ----------------------------------------------------------

    async def send(self, command, arg=None, wait_answer=False, timeout_s=1.0):
        """Send one MC command.

        Builds `COMMAND` or `COMMAND arg`. If wait_answer, awaits matching
        `TAG:payload` and returns the payload string; else returns None.
        """
        cmd = str(command).strip()
        if not cmd:
            raise ValueError("empty command")
        if arg is None:
            line = cmd
        else:
            line = "%s %s" % (cmd, arg)
        tag = cmd.split(None, 1)[0].upper()

        waiter = None
        if wait_answer:
            ev = asyncio.Event()
            box = [None]
            waiter = (ev, box)
            self._waiters.setdefault(tag, []).append(waiter)

        self._write_line(line)

        if not wait_answer:
            return None
        try:
            await self._wait_event(ev, timeout_s)
            return box[0]
        except OSError:
            self._drop_waiter(tag, waiter)
            raise OSError("timeout waiting for %s:" % tag)

    def _write_line(self, line):
        data = (str(line).rstrip("\r\n") + "\n").encode("ascii")
        self._uart.write(data)

    async def _wait_event(self, ev, timeout_s):
        ms = int(float(timeout_s) * 1000)
        if ms < 1:
            ms = 1
        deadline = time.ticks_add(time.ticks_ms(), ms)
        while not ev.is_set():
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise OSError("timeout")
            await asyncio.sleep_ms(5)

    def _drop_waiter(self, tag, waiter):
        lst = self._waiters.get(tag)
        if not lst:
            return
        try:
            lst.remove(waiter)
        except ValueError:
            pass
        if not lst:
            self._waiters.pop(tag, None)

    def _complete_waiter(self, tag, answer):
        lst = self._waiters.get(tag)
        if not lst:
            return
        ev, box = lst.pop(0)
        if not lst:
            self._waiters.pop(tag, None)
        box[0] = answer
        ev.set()

    # --- RX ----------------------------------------------------------------

    async def _rx_loop(self):
        while True:
            n = self._uart.any()
            if n:
                chunk = self._uart.read(n)
                if chunk:
                    self._rx_buf += chunk
                    self._drain_lines()
            await asyncio.sleep_ms(2)

    def _drain_lines(self):
        while True:
            i = self._rx_buf.find(b"\n")
            if i < 0:
                if len(self._rx_buf) > 512:
                    self._rx_buf = self._rx_buf[-256:]
                return
            raw = self._rx_buf[:i]
            self._rx_buf = self._rx_buf[i + 1 :]
            if raw.endswith(b"\r"):
                raw = raw[:-1]
            try:
                line = raw.decode("ascii")
            except UnicodeError:
                continue
            self._handle_line(line)

    def _handle_line(self, line):
        if not line:
            return

        if line.startswith("# "):
            self._banner_event.set()
            return

        if len(line) >= 2 and line[0] == "#" and line[1] != " ":
            self._handle_status(line)
            return

        if line.startswith("!E:"):
            rest = line[3:].strip()
            if " " in rest:
                code, text = rest.split(" ", 1)
            else:
                code, text = rest, ""
            try:
                self.on_error(code, text)
            except Exception:
                pass
            return

        colon = line.find(":")
        if colon > 0:
            tag = line[:colon].strip()
            if tag and (" " not in tag) and tag[0].isalpha():
                answer = line[colon + 1 :]
                cmd = tag.upper()
                if cmd == "CG" and self._cg_collect is not None:
                    key, sep, val = answer.partition("=")
                    if sep:
                        self._cg_collect[key.strip()] = val.strip()
                    try:
                        self.on_answer(cmd, answer)
                    except Exception:
                        pass
                    return
                try:
                    self.on_answer(cmd, answer)
                except Exception:
                    pass
                self._complete_waiter(cmd, answer)
                return

    def _handle_status(self, line):
        # #<state> <pos> [<speed> <accel> [<target>]]
        body = line[1:].strip()
        if not body:
            return
        parts = body.split()
        state = parts[0]
        if len(state) != 1:
            return
        pos = _parse_float(parts[1]) if len(parts) > 1 else None
        speed = _parse_float(parts[2]) if len(parts) > 2 else None
        accel = _parse_float(parts[3]) if len(parts) > 3 else None
        target = None
        if len(parts) > 4:
            target = _parse_float(parts[4])

        try:
            self.status = self.MC_STATE_CHARS.index(state)
        except ValueError:
            pass

        self._state = state
        self._moving = state in ("M", "A", "B", "H")
        self._homing = state == "H"
        self._at_hard_limit = state == "L"
        self._drv_error_active = state == "E"
        if state == "D":
            self._enabled = False
        elif state in ("I", "M", "H", "A", "B"):
            if self._enabled is None:
                self._enabled = True

        if pos is not None:
            self._pos_mm = pos
        if speed is not None:
            self._act_speed_mm_s = speed
            self._act_vel_mm_s = float(speed)
        elif state in ("I", "D", "L", "E"):
            self._act_speed_mm_s = 0.0
            self._act_vel_mm_s = 0.0
        if len(parts) > 4:
            self._target_mm = target
        elif state not in ("M", "H", "A", "B"):
            self._target_mm = None

        if accel is None and state in ("I", "D", "L", "E"):
            accel = 0.0
        if accel is None:
            accel = self._accel_mm_s2

        if state == "A":
            self._accelerating = True
            self._decelerating = False
        elif state == "B":
            self._accelerating = False
            self._decelerating = True
        else:
            spd_abs = abs(self._act_vel_mm_s) if self._act_vel_mm_s is not None else 0.0
            if self._moving and not self._homing and self._prev_act_speed_abs is not None:
                eps = float(getattr(cfg, "LED_ACCEL_SPEED_EPS_MM_S", 3.0))
                delta = spd_abs - self._prev_act_speed_abs
                self._decelerating = delta < -eps
                self._accelerating = delta > eps
            else:
                self._decelerating = False
                self._accelerating = False
            self._prev_act_speed_abs = spd_abs

        if state in ("A", "B", "M", "H"):
            spd_abs = abs(self._act_vel_mm_s) if self._act_vel_mm_s is not None else 0.0
            self._prev_act_speed_abs = spd_abs

        self._refresh_soft_limit_flag()

        try:
            self.on_status(
                state, self._pos_mm, self._act_speed_mm_s, accel, self._target_mm
            )
        except Exception:
            pass

    def _refresh_soft_limit_flag(self):
        pos = self._pos_mm
        if pos is None:
            self._at_soft_limit = False
            self._near_soft_limit = False
            return
        warn = float(getattr(cfg, "SOFT_LIMIT_WARN_MM", 10.0))
        at = False
        near = False
        if self._soft_min is not None:
            d = float(pos) - float(self._soft_min)
            if abs(d) < 1e-3 or d < 0:
                at = True
            elif d <= warn:
                near = True
        if self._soft_max is not None:
            d = float(self._soft_max) - float(pos)
            if abs(d) < 1e-3 or d < 0:
                at = True
            elif d <= warn:
                near = True
        self._at_soft_limit = at
        self._near_soft_limit = (not at) and near

    # --- configuration API (sync) ------------------------------------------

    def setSpeed(self, mm_per_sec):
        self._speed_mm_s = max(float(mm_per_sec), cfg.MIN_SPEED_MM_S)
        self._cmd("SS", _fmt_arg(self._speed_mm_s))

    def setMaxSpeed(self, mm_per_sec):
        """Persistent max_speed via CS (MC enforces planner ceiling)."""
        v = max(float(mm_per_sec), cfg.MIN_SPEED_MM_S)
        self._max_speed_mm_s = v
        self.max_speed = v
        if self._speed_mm_s is not None and self._speed_mm_s > v:
            self._speed_mm_s = v
        self._cmd("CS", "max_speed %s" % _fmt_arg(v))

    def setAcceleration(self, accel):
        self._accel_mm_s2 = max(float(accel), cfg.MIN_SPEED_MM_S)
        self._cmd("SA", _fmt_arg(self._accel_mm_s2))

    def setSoftLimits(self, min_limit, max_limit):
        self._soft_min = min_limit
        self._soft_max = max_limit
        self.slider_min = min_limit
        self.slider_max = max_limit
        if min_limit is None:
            self._cmd("CS", "slider_min none")
        else:
            self._cmd("CS", "slider_min %s" % _fmt_arg(float(min_limit)))
        if max_limit is None:
            self._cmd("CS", "slider_max none")
        else:
            self._cmd("CS", "slider_max %s" % _fmt_arg(float(max_limit)))
        self._refresh_soft_limit_flag()

    def enable(self, on):
        if on and self.isDRVErrorActive():
            return
        self._enabled = bool(on)
        self._cmd("SE", 1 if self._enabled else 0)

    def estimateMoveTime(self, distance_mm, speed_mm_s, accel_mm_s2):
        d = abs(float(distance_mm))
        if d < 1e-9:
            return 0.0
        v = abs(float(speed_mm_s))
        a = abs(float(accel_mm_s2))
        if v < cfg.MIN_SPEED_MM_S or a < cfg.MIN_SPEED_MM_S:
            return 0.0
        d_r = math.pi * v * v / (4.0 * a)
        t_r = math.pi * v / (2.0 * a)
        if 2.0 * d_r <= d:
            return 2.0 * t_r + (d - 2.0 * d_r) / v
        v_pk = math.sqrt(2.0 * a * d / math.pi)
        return math.pi * v_pk / a

    def estimateMoveTimeTo(self, position_mm, speed_mm_s=None, accel_mm_s2=None):
        """Stop-to-stop sine-ramp time from current position to ``position_mm``."""
        if speed_mm_s is None:
            speed_mm_s = self._speed_mm_s if self._speed_mm_s is not None else 0.0
        if accel_mm_s2 is None:
            accel_mm_s2 = self._accel_mm_s2 if self._accel_mm_s2 is not None else 0.0
        return self.estimateMoveTime(
            float(position_mm) - self.getPosition(), speed_mm_s, accel_mm_s2
        )

    # --- getters -----------------------------------------------------------

    def getPosition(self):
        return self._pos_mm if self._pos_mm is not None else 0.0

    def getSpeed(self):
        return self._act_vel_mm_s if self._act_vel_mm_s is not None else 0.0

    def getAcceleration(self):
        return self._accel_mm_s2

    def getTarget(self):
        return self._target_mm

    def isMoving(self):
        return self._moving

    def isDecelerating(self):
        if not self._moving or self._homing:
            return False
        return bool(self._decelerating)

    def isHoming(self):
        return self._homing

    def isAtSoftLimit(self):
        return self._at_soft_limit

    def isNearSoftLimit(self):
        return self._near_soft_limit

    def isAtHardLimit(self):
        return self._at_hard_limit

    def isDRVErrorActive(self):
        return self._drv_error_active or self._state == "E"

    def setPosition(self, position_mm):
        raise NotImplementedError("setPosition not supported by SliderMC protocol")

    async def query(self, command, arg=None, timeout_s=1.0):
        """Send a get/is/config command and return the answer payload."""
        return await self.send(command, arg, wait_answer=True, timeout_s=timeout_s)

    # --- motion API (sync) -------------------------------------------------

    def moveTo(self, position):
        self._cmd("MT", _fmt_arg(float(position)))

    def moveBy(self, dist):
        self._cmd("M", _fmt_arg(float(dist)))

    def move(self, speed):
        speed = float(speed)
        if abs(speed) < 1e-9:
            self._cmd("MS")
            return
        self._speed_mm_s = abs(speed)
        self._cmd("SS", _fmt_arg(abs(speed)))
        if speed > 0:
            self._cmd("MR")
        else:
            self._cmd("ML")

    def home(self):
        self._motion_task = asyncio.create_task(self._home_coro())
        return self._motion_task

    async def _home_coro(self):
        self._cmd("MH")
        for _ in range(40):
            if self._state == "H":
                break
            await asyncio.sleep_ms(50)
        while self._state == "H":
            await asyncio.sleep_ms(50)

    def stop(self):
        if self.isDRVErrorActive():
            return
        self._cmd("MS")

    def halt(self):
        self._cmd("H")
        self._enabled = False

    async def wait(self):
        t = self._motion_task
        if t is not None:
            try:
                await t
            except asyncio.CancelledError:
                pass
            finally:
                if self._motion_task is t:
                    self._motion_task = None
        while self.isMoving() or self.isHoming():
            await asyncio.sleep_ms(20)


def _parse_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _parse_cfg_float(s):
    if s is None:
        return None
    t = str(s).strip()
    if not t or t.lower() == "none":
        return None
    return _parse_float(t)


def _parse_cfg_limit(s):
    return _parse_cfg_float(s)


def _fmt_arg(v):
    if isinstance(v, int) or (isinstance(v, float) and v == int(v)):
        return str(int(v))
    s = "%.4f" % float(v)
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s
