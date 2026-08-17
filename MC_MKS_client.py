# MC_MKS_client — RS485 drop-in MC_API for MKS SERVO42D/57D (MicroPython + uasyncio).
#
# Native Makerbase FA/FB + CRC8 only (not Modbus-RTU). Motion is F5 absolute-axis
# only (no F6). Soft-limit jog = moveTo(slider_min/max). DE on PIN_RS485_DE.
#
# Same public names as MC_Client for composition with UIC_Base.

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

import MC_MKS_config as cfg


class MC_MKS_Client:
    """MC_API over MKS SERVO42D/57D RS485 (F5 position mode)."""

    MC_STATE_DISABLED = 0
    MC_STATE_IDLE = 1
    MC_STATE_ACCELERATING = 2
    MC_STATE_MOVING = 3
    MC_STATE_DECELERATING = 4
    MC_STATE_HOMING = 5
    MC_STATE_HARD_LIMIT = 6
    MC_STATE_ERROR = 7
    MC_STATE_LOCKED = 8
    MC_STATE_CHARS = ("D", "I", "A", "M", "B", "H", "L", "E", "?")

    # F1 operating status (manual §9.2.1)
    _F1_STOP = 1
    _F1_SPEED_UP = 2
    _F1_SPEED_DOWN = 3
    _F1_FULL = 4
    _F1_HOMING = 5
    _F1_CAL = 6

    def __init__(self, uart_id=None, tx=None, rx=None, baud=None, de=None, addr=None):
        if UART is None:
            raise RuntimeError("machine.UART not available")
        if uart_id is None:
            uart_id = int(getattr(cfg, "UART_ID", 0))
        if tx is None:
            tx = getattr(cfg, "PIN_UART_TX", 16)
        if rx is None:
            rx = getattr(cfg, "PIN_UART_RX", 17)
        if baud is None:
            baud = int(getattr(cfg, "UART_BAUD", 38400))
        if de is None:
            de = getattr(cfg, "PIN_RS485_DE", 18)
        if addr is None:
            addr = int(getattr(cfg, "MKS_ADDR", 1))

        self._addr = int(addr) & 0xFF
        self._uart = UART(
            int(uart_id),
            baudrate=int(baud),
            tx=Pin(int(tx)),
            rx=Pin(int(rx)),
            bits=8,
            parity=None,
            stop=1,
        )
        self._de = Pin(int(de), Pin.OUT)
        self._de.value(0)  # receive

        self._bus_lock = asyncio.Lock()
        self._started = False
        self._status_task = None
        self._motion_task = None
        self._cmd_seq = 0

        self._status_cb = None
        self._error_cb = None
        self._answer_cb = None

        self._mm_per_rot = float(getattr(cfg, "MM_PER_ROT", 5.0))
        self._axis_per_rot = int(getattr(cfg, "AXIS_PER_ROT", 0x4000))
        self._pos_bias_mm = 0.0  # UIC mm when motor axis == 0

        self._speed_mm_s = float(getattr(cfg, "INIT_SPEED_MM_S", 50.0))
        self._accel_mm_s2 = float(getattr(cfg, "INIT_ACCEL_MM_S2", 200.0))
        self._max_speed_mm_s = float(getattr(cfg, "MAX_SPEED_MM_S", 100.0))
        self._max_accel_mm_s2 = float(getattr(cfg, "MAX_ACCEL_MM_S2", 500.0))
        self._soft_min = getattr(cfg, "SLIDER_MIN", 0.0)
        self._soft_max = getattr(cfg, "SLIDER_MAX", 600.0)

        self._enabled = False
        self._state = None
        self._pos_mm = 0.0
        self._act_speed_mm_s = 0.0
        self._act_vel_mm_s = 0.0
        self._target_mm = None
        self._cmd_accel_mag = 0.0  # for status while ramping

        self._moving = False
        self._homing = False
        self._at_soft_limit = False
        self._near_soft_limit = False
        self._at_hard_limit = False
        self._drv_error_active = False
        self._decelerating = False
        self._accelerating = False
        self._f1 = self._F1_STOP

        self.mc_config = {}
        self.max_speed = self._max_speed_mm_s
        self.max_accel = self._max_accel_mm_s2
        self.slider_min = self._soft_min
        self.slider_max = self._soft_max
        self.status = self.MC_STATE_LOCKED

        self._tx_hold_ms = int(getattr(cfg, "RS485_TX_HOLD_MS", 2))
        self._rx_timeout_ms = int(getattr(cfg, "RS485_RX_TIMEOUT_MS", 80))
        self._status_hz = float(getattr(cfg, "STATUS_HZ", 5.0))

    # --- callbacks ---------------------------------------------------------

    def set_status_callback(self, cb):
        """Register cb(state, pos, speed, accel, target) (~STATUS_HZ)."""
        self._status_cb = cb

    def set_error_callback(self, cb):
        """Register cb(code, text) for stall / bus errors."""
        self._error_cb = cb

    def set_answer_callback(self, cb):
        """Optional; MKS path has no TAG:value replies (kept for MC_API)."""
        self._answer_cb = cb

    def on_error(self, code, text):
        cb = self._error_cb
        if cb is not None:
            cb(code, text)

    def on_status(self, state, pos, speed, accel, target):
        cb = self._status_cb
        if cb is not None:
            cb(state, pos, speed, accel, target)

    def on_answer(self, command, answer):
        cb = self._answer_cb
        if cb is not None:
            cb(command, answer)

    # --- lifecycle ---------------------------------------------------------

    async def start(self, banner_timeout_s=3.0):
        """Open RS485, ensure SR_vFOC, enable policy, start status task.

        No SliderMC banner / SV / CG. ``banner_timeout_s`` ignored (API compat).
        """
        _ = banner_timeout_s
        self._flush_rx()
        # Bus FOC mode 05H (idempotent if already set).
        try:
            await self._transact(self._frame(0x82, bytes([0x05])), expect_fn=0x82)
        except OSError as e:
            print("MC_MKS: mode set failed (%s) — check wiring / baud / addr" % e)

        if int(getattr(cfg, "LIMIT_REMAP", 0)):
            try:
                await self._transact(self._frame(0x9E, bytes([0x01])), expect_fn=0x9E)
            except OSError as e:
                print("MC_MKS: limit remap failed: %s" % e)

        home_use = int(getattr(cfg, "HOME_USE", 0))
        hard_use = int(getattr(cfg, "HARD_LIMIT_USE", 0))
        if home_use or hard_use:
            await self._configure_home_limits(end_limit=1 if hard_use else 0)

        # Seed public config mirror (local; no CG).
        self.mc_config = {
            "max_speed": str(self._max_speed_mm_s),
            "max_accel": str(self._max_accel_mm_s2),
            "slider_min": _fmt_limit(self._soft_min),
            "slider_max": _fmt_limit(self._soft_max),
            "mm_per_rot": str(self._mm_per_rot),
            "backend": "MC_MKS",
        }
        self.max_speed = self._max_speed_mm_s
        self.max_accel = self._max_accel_mm_s2
        self.slider_min = self._soft_min
        self.slider_max = self._soft_max

        await self._poll_once()
        self._started = True
        if self._status_task is None:
            self._status_task = asyncio.create_task(self._status_loop())

    async def fetchConfig(self, settle_ms=150):
        """No remote CG; refresh public fields from local config."""
        _ = settle_ms
        self.mc_config["max_speed"] = str(self._max_speed_mm_s)
        self.mc_config["max_accel"] = str(self._max_accel_mm_s2)
        self.mc_config["slider_min"] = _fmt_limit(self._soft_min)
        self.mc_config["slider_max"] = _fmt_limit(self._soft_max)
        return self.mc_config

    async def stop_rx(self):
        t = self._status_task
        self._status_task = None
        if t is not None:
            t.cancel()
            try:
                await t
            except asyncio.CancelledError:
                pass

    async def send(self, command, arg=None, wait_answer=False, timeout_s=1.0):
        """Not used on MKS path (ASCII TAG protocol is SliderMC-only)."""
        _ = (command, arg, wait_answer, timeout_s)
        raise NotImplementedError("send() is SliderMC ASCII; use MC_MKS motion API")

    async def query(self, command, arg=None, timeout_s=1.0):
        _ = (command, arg, timeout_s)
        raise NotImplementedError("query() is SliderMC ASCII; use getters / status")

    # --- RS485 framing -----------------------------------------------------

    def _crc8(self, data):
        s = 0
        for b in data:
            s = (s + b) & 0xFF
        return s

    def _frame(self, code, payload=b""):
        body = bytes([0xFA, self._addr, int(code) & 0xFF]) + bytes(payload)
        return body + bytes([self._crc8(body)])

    def _flush_rx(self):
        n = self._uart.any()
        if n:
            self._uart.read(n)

    async def _transact(self, frame, expect_fn=None, min_len=5, timeout_ms=None):
        """Half-duplex: DE high → TX → settle → DE low → read FB reply."""
        if timeout_ms is None:
            timeout_ms = self._rx_timeout_ms
        async with self._bus_lock:
            self._flush_rx()
            self._de.value(1)
            self._uart.write(frame)
            if self._tx_hold_ms > 0:
                await asyncio.sleep_ms(self._tx_hold_ms)
            self._de.value(0)

            buf = bytearray()
            deadline = time.ticks_add(time.ticks_ms(), int(timeout_ms))
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                n = self._uart.any()
                if n:
                    chunk = self._uart.read(n)
                    if chunk:
                        buf.extend(chunk)
                        # Find FB addr …
                        while True:
                            try:
                                i = buf.index(0xFB)
                            except ValueError:
                                buf = bytearray()
                                break
                            if i:
                                del buf[:i]
                            if len(buf) < 3:
                                break
                            if expect_fn is not None and buf[2] != (expect_fn & 0xFF):
                                del buf[0]
                                continue
                            if len(buf) < min_len:
                                break
                            # Grow until CRC lands: for known short replies use len;
                            # otherwise accept when last byte matches running CRC of prefix.
                            reply = self._try_take_reply(buf, min_len)
                            if reply is not None:
                                return reply
                            # Need more bytes
                            break
                await asyncio.sleep_ms(1)
            raise OSError("RS485 timeout fn=0x%02X" % (expect_fn if expect_fn is not None else -1))

    def _try_take_reply(self, buf, min_len):
        """Return complete FB frame from buf start, or None if incomplete/invalid."""
        if len(buf) < min_len:
            return None
        # Prefer exact CRC match for lengths min_len … min_len+8
        for L in range(min_len, min(len(buf), min_len + 10) + 1):
            if L < 4:
                continue
            body = bytes(buf[: L - 1])
            if self._crc8(body) == buf[L - 1]:
                reply = bytes(buf[:L])
                del buf[:L]
                return reply
        return None

    async def _cmd_status(self, code):
        """Short read: FB addr code [data…] CRC."""
        return await self._transact(self._frame(code), expect_fn=code, min_len=5)

    # --- unit conversion ---------------------------------------------------

    def _mm_to_axis(self, mm):
        return int(round(float(mm) * self._axis_per_rot / self._mm_per_rot))

    def _axis_to_mm(self, axis):
        return float(axis) * self._mm_per_rot / float(self._axis_per_rot)

    def _uic_to_axis(self, mm):
        return self._mm_to_axis(float(mm) - self._pos_bias_mm)

    def _axis_to_uic(self, axis):
        return self._axis_to_mm(axis) + self._pos_bias_mm

    def _mm_s_to_rpm(self, mm_s):
        # |RPM| = |mm/s| * 60 / |mm_per_rot|
        rpm = abs(float(mm_s)) * 60.0 / abs(self._mm_per_rot)
        if rpm < 1.0 and abs(mm_s) >= float(getattr(cfg, "MIN_SPEED_MM_S", 0.006)):
            rpm = 1.0
        if rpm > 3000.0:
            rpm = 3000.0
        return int(round(rpm))

    def _rpm_to_mm_s(self, rpm):
        return float(rpm) * abs(self._mm_per_rot) / 60.0

    def _accel_to_mks_acc(self, accel_mm_s2):
        a = abs(float(accel_mm_s2))
        a_ref = float(getattr(cfg, "ACCEL_MM_S2_FOR_ACC_MAX", 500.0))
        acc_max = int(getattr(cfg, "MKS_ACC_MAX", 200))
        acc_min = int(getattr(cfg, "MKS_ACC_MIN", 1))
        if a_ref < 1e-9:
            return acc_min
        acc = int(round(a * acc_max / a_ref))
        if acc < acc_min:
            acc = acc_min
        if acc > 255:
            acc = 255
        if acc > acc_max:
            acc = acc_max
        return acc

    def _pack_i16_be(self, v):
        v = int(v) & 0xFFFF
        return bytes([(v >> 8) & 0xFF, v & 0xFF])

    def _pack_i32_be(self, v):
        v = int(v) & 0xFFFFFFFF
        return bytes(
            [
                (v >> 24) & 0xFF,
                (v >> 16) & 0xFF,
                (v >> 8) & 0xFF,
                v & 0xFF,
            ]
        )

    def _unpack_i16_be(self, b):
        v = (b[0] << 8) | b[1]
        if v & 0x8000:
            v -= 0x10000
        return v

    def _unpack_i32_be(self, b):
        v = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]
        if v & 0x80000000:
            v -= 0x100000000
        return v

    def _unpack_i48_be(self, b):
        # 6-byte big-endian signed
        v = 0
        for x in b[:6]:
            v = (v << 8) | x
        if v & (1 << 47):
            v -= 1 << 48
        return v

    # --- home / limits config ----------------------------------------------

    async def _configure_home_limits(self, end_limit):
        hm_trig = int(getattr(cfg, "HOME_TRIG_LEVEL", 0)) & 0x01
        hm_dir = int(getattr(cfg, "HOME_DIR", 0)) & 0x01
        hm_rpm = self._mm_s_to_rpm(float(getattr(cfg, "HOME_SPEED_MM_S", 25.0)))
        # 94H: endstop home, offset 0, default ma
        try:
            payload94 = self._pack_i32_be(0) + bytes([0x00]) + self._pack_i16_be(800)
            await self._transact(self._frame(0x94, payload94), expect_fn=0x94)
        except OSError as e:
            print("MC_MKS: 94H failed: %s" % e)
        payload90 = bytes([hm_trig, hm_dir]) + self._pack_i16_be(hm_rpm) + bytes([int(end_limit) & 1])
        await self._transact(self._frame(0x90, payload90), expect_fn=0x90)

    # --- F5 motion ---------------------------------------------------------

    async def _f5(self, rpm, acc, abs_axis):
        rpm = int(rpm) & 0xFFFF
        if rpm > 3000:
            rpm = 3000
        acc = int(acc) & 0xFF
        payload = self._pack_i16_be(rpm) + bytes([acc]) + self._pack_i32_be(abs_axis)
        return await self._transact(self._frame(0xF5, payload), expect_fn=0xF5, min_len=5)

    async def _f5_soft_stop(self):
        acc = self._accel_to_mks_acc(self._accel_mm_s2)
        if acc < 1:
            acc = 1
        return await self._f5(0, acc, 0)

    async def _f3_enable(self, on):
        return await self._transact(
            self._frame(0xF3, bytes([0x01 if on else 0x00])), expect_fn=0xF3
        )

    async def _f7_halt(self):
        return await self._transact(self._frame(0xF7), expect_fn=0xF7, min_len=5)

    def _schedule(self, coro):
        self._cmd_seq += 1
        seq = self._cmd_seq

        async def _runner():
            try:
                await coro
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print("MC_MKS cmd error: %s" % e)
                self.on_error("MKS", str(e))

        t = asyncio.create_task(_runner())
        self._motion_task = t
        return t, seq

    def _clamp_target(self, position_mm):
        p = float(position_mm)
        if self._soft_min is not None and p < float(self._soft_min):
            p = float(self._soft_min)
        if self._soft_max is not None and p > float(self._soft_max):
            p = float(self._soft_max)
        return p

    def _reject_at_bound(self, target_mm):
        """True if already at soft bound and command pushes further out."""
        pos = self.getPosition()
        eps = 1e-3
        if self._soft_max is not None and target_mm > pos + eps:
            if abs(pos - float(self._soft_max)) < eps or pos > float(self._soft_max):
                return True
        if self._soft_min is not None and target_mm < pos - eps:
            if abs(pos - float(self._soft_min)) < eps or pos < float(self._soft_min):
                return True
        return False

    async def _goto_mm(self, position_mm, speed_mm_s=None, accel_mm_s2=None):
        if self._drv_error_active:
            return
        tgt = self._clamp_target(position_mm)
        if self._reject_at_bound(tgt):
            return
        if not self._enabled:
            try:
                await self._f3_enable(True)
                self._enabled = True
            except OSError:
                pass
        v = self._speed_mm_s if speed_mm_s is None else float(speed_mm_s)
        a = self._accel_mm_s2 if accel_mm_s2 is None else float(accel_mm_s2)
        v = max(v, float(getattr(cfg, "MIN_SPEED_MM_S", 0.006)))
        if self._max_speed_mm_s is not None:
            v = min(v, float(self._max_speed_mm_s))
        rpm = self._mm_s_to_rpm(v)
        acc = self._accel_to_mks_acc(a)
        axis = self._uic_to_axis(tgt)
        self._target_mm = tgt
        self._cmd_accel_mag = abs(a)
        self._moving = True
        reply = await self._f5(rpm, acc, axis)
        if reply and len(reply) >= 4:
            st = reply[3]
            if st == 0:
                self.on_error("F5", "run fail")
            elif st == 3:
                self._at_hard_limit = True

    async def _reissue_active_f5(self):
        if self._target_mm is None or not self._moving:
            return
        await self._goto_mm(self._target_mm)

    # --- configuration API (sync) ------------------------------------------

    def setSpeed(self, mm_per_sec):
        v = max(float(mm_per_sec), float(getattr(cfg, "MIN_SPEED_MM_S", 0.006)))
        if self._max_speed_mm_s is not None:
            v = min(v, float(self._max_speed_mm_s))
        self._speed_mm_s = v
        if self._moving and self._target_mm is not None:
            self._schedule(self._reissue_active_f5())

    def setMaxSpeed(self, mm_per_sec):
        v = max(float(mm_per_sec), float(getattr(cfg, "MIN_SPEED_MM_S", 0.006)))
        self._max_speed_mm_s = v
        self.max_speed = v
        if self._speed_mm_s is not None and self._speed_mm_s > v:
            self._speed_mm_s = v

    def setAcceleration(self, accel):
        a = max(float(accel), float(getattr(cfg, "MIN_SPEED_MM_S", 0.006)))
        if self._max_accel_mm_s2 is not None:
            a = min(a, float(self._max_accel_mm_s2))
        self._accel_mm_s2 = a
        if self._moving and self._target_mm is not None:
            self._schedule(self._reissue_active_f5())

    def setSoftLimits(self, min_limit, max_limit):
        self._soft_min = min_limit
        self._soft_max = max_limit
        self.slider_min = min_limit
        self.slider_max = max_limit
        self._refresh_soft_limit_flag()

    def enable(self, on):
        if on and self.isDRVErrorActive():
            return
        self._enabled = bool(on)

        async def _do():
            await self._f3_enable(self._enabled)

        self._schedule(_do())

    def estimateMoveTime(self, distance_mm, speed_mm_s, accel_mm_s2):
        """Simple trapezoid/triangle time (constant a). Display-grade, not MKS-discrete."""
        d = abs(float(distance_mm))
        if d < 1e-9:
            return 0.0
        v = abs(float(speed_mm_s))
        a = abs(float(accel_mm_s2))
        vmin = float(getattr(cfg, "MIN_SPEED_MM_S", 0.006))
        if v < vmin or a < vmin:
            return 0.0
        d_acc = (v * v) / (2.0 * a)
        if 2.0 * d_acc <= d:
            return 2.0 * (v / a) + (d - 2.0 * d_acc) / v
        # Triangle: peak speed below cruise
        v_pk = math.sqrt(a * d)
        return 2.0 * v_pk / a

    def estimateMoveTimeTo(self, position_mm, speed_mm_s=None, accel_mm_s2=None):
        """Stop-to-stop time from current position to ``position_mm`` (mm-space trapezoid)."""
        if speed_mm_s is None:
            speed_mm_s = self._speed_mm_s if self._speed_mm_s is not None else 0.0
        if accel_mm_s2 is None:
            accel_mm_s2 = self._accel_mm_s2 if self._accel_mm_s2 is not None else 0.0
        dist = float(position_mm) - self.getPosition()
        return self.estimateMoveTime(dist, speed_mm_s, accel_mm_s2)

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
        """Set UIC zero via motor ``92H`` (current axis → 0) + bias to ``position_mm``.

        Unlike ``MC_Client`` (raises), this is supported without SliderMC.
        """

        async def _do():
            await self._transact(self._frame(0x92), expect_fn=0x92)
            self._pos_bias_mm = float(position_mm)
            self._pos_mm = float(position_mm)
            self._refresh_soft_limit_flag()

        self._schedule(_do())

    # --- motion API (sync) -------------------------------------------------

    def moveTo(self, position):
        self._schedule(self._goto_mm(float(position)))

    def moveBy(self, dist):
        self._schedule(self._goto_mm(self.getPosition() + float(dist)))

    def move(self, speed):
        speed = float(speed)
        if abs(speed) < 1e-9:
            self.stop()
            return
        self._speed_mm_s = abs(speed)
        if self._max_speed_mm_s is not None:
            self._speed_mm_s = min(self._speed_mm_s, float(self._max_speed_mm_s))
        if speed > 0:
            if self._soft_max is None:
                print("MC_MKS: move(+) needs SLIDER_MAX")
                return
            self.moveTo(float(self._soft_max))
        else:
            if self._soft_min is None:
                print("MC_MKS: move(-) needs SLIDER_MIN")
                return
            self.moveTo(float(self._soft_min))

    def home(self):
        if not int(getattr(cfg, "HOME_USE", 0)):
            print("MC_MKS: HOME_USE=0 — home() ignored")
            return None
        self._motion_task = asyncio.create_task(self._home_coro())
        return self._motion_task

    async def _home_coro(self):
        self._homing = True
        self._moving = True
        self._state = "H"
        try:
            await self._configure_home_limits(
                end_limit=1 if int(getattr(cfg, "HARD_LIMIT_USE", 0)) else 0
            )
            if not self._enabled:
                await self._f3_enable(True)
                self._enabled = True
            # Origin homing
            await self._transact(self._frame(0x91, bytes([0x00])), expect_fn=0x91)
            saw_homing = False
            for _ in range(600):  # up to ~60 s @ 100 ms
                await self._poll_once()
                if self._f1 == self._F1_HOMING:
                    saw_homing = True
                    self._homing = True
                if self._f1 == self._F1_STOP and saw_homing:
                    break
                await asyncio.sleep_ms(100)
            # Align UIC position
            home_mm = float(getattr(cfg, "HOME_SET_POS_MM", 0.0))
            await self._transact(self._frame(0x92), expect_fn=0x92)
            self._pos_bias_mm = home_mm
            self._pos_mm = home_mm
            self._target_mm = home_mm
            self._refresh_soft_limit_flag()
        except Exception as e:
            self.on_error("HOME", str(e))
        finally:
            self._homing = False
            self._moving = False

    def stop(self):
        if self.isDRVErrorActive():
            return

        async def _do():
            await self._f5_soft_stop()
            self._cmd_accel_mag = abs(self._accel_mm_s2 or 0.0)

        self._schedule(_do())

    def halt(self):
        async def _do():
            try:
                await self._f7_halt()
            finally:
                try:
                    await self._f3_enable(False)
                except OSError:
                    pass
                self._enabled = False
                self._moving = False
                self._homing = False
                self._cmd_accel_mag = 0.0

        self._schedule(_do())

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

    # --- status ------------------------------------------------------------

    def _refresh_soft_limit_flag(self):
        pos = self.getPosition()
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

    def _letter_from_f1(self, f1):
        if self._drv_error_active:
            return "E"
        if self._at_hard_limit and f1 == self._F1_STOP:
            return "L"
        if not self._enabled:
            return "D"
        if f1 == self._F1_SPEED_UP:
            return "A"
        if f1 == self._F1_SPEED_DOWN:
            return "B"
        if f1 == self._F1_FULL:
            return "M"
        if f1 == self._F1_HOMING:
            return "H"
        if f1 == self._F1_CAL:
            return "E"
        return "I"

    def _status_int_from_letter(self, ch):
        try:
            return self.MC_STATE_CHARS.index(ch)
        except ValueError:
            return self.MC_STATE_LOCKED

    async def _poll_once(self):
        try:
            r_f1 = await self._cmd_status(0xF1)
            if r_f1 and len(r_f1) >= 4:
                self._f1 = r_f1[3]
            r31 = await self._transact(self._frame(0x31), expect_fn=0x31, min_len=10)
            if r31 and len(r31) >= 10:
                axis = self._unpack_i48_be(r31[3:9])
                self._pos_mm = self._axis_to_uic(axis)
            r32 = await self._transact(self._frame(0x32), expect_fn=0x32, min_len=6)
            if r32 and len(r32) >= 6:
                rpm = self._unpack_i16_be(r32[3:5])
                # Sign: CCW>0; UIC signed speed follows RPM × sign(mm_per_rot)
                signed_mm_s = self._rpm_to_mm_s(rpm)
                if self._mm_per_rot < 0:
                    signed_mm_s = -signed_mm_s
                self._act_vel_mm_s = signed_mm_s
                self._act_speed_mm_s = abs(signed_mm_s)

            # Stall
            try:
                r3e = await self._cmd_status(0x3E)
                if r3e and len(r3e) >= 4 and r3e[3] == 1:
                    if not self._drv_error_active:
                        self._drv_error_active = True
                        self.on_error("STALL", "motor stalled")
                else:
                    self._drv_error_active = False
            except OSError:
                pass

        except OSError as e:
            # Keep last cache; surface occasionally
            self.on_error("POLL", str(e))
            return

        self._homing = self._f1 == self._F1_HOMING
        self._accelerating = self._f1 == self._F1_SPEED_UP
        self._decelerating = self._f1 == self._F1_SPEED_DOWN
        self._moving = self._f1 in (
            self._F1_SPEED_UP,
            self._F1_SPEED_DOWN,
            self._F1_FULL,
            self._F1_HOMING,
        )
        if self._f1 == self._F1_STOP:
            self._cmd_accel_mag = 0.0
            if self._at_hard_limit:
                pass  # sticky until move away / clear
            else:
                # Clear hard-limit latch only when moving again later
                pass

        letter = self._letter_from_f1(self._f1)
        # End-limit: if stopped near target after F5 status 3, already set.
        # Also infer from hard-limit latch.
        if letter == "L":
            self._at_hard_limit = True

        self._state = letter
        self.status = self._status_int_from_letter(letter)
        self._refresh_soft_limit_flag()

        accel_out = 0.0
        if self._accelerating or self._decelerating:
            accel_out = float(self._cmd_accel_mag or self._accel_mm_s2 or 0.0)
            if self._decelerating:
                accel_out = -accel_out

        self.on_status(
            letter,
            self._pos_mm,
            self._act_vel_mm_s,
            accel_out,
            self._target_mm if self._target_mm is not None else self._pos_mm,
        )

    async def _status_loop(self):
        hz = self._status_hz if self._status_hz > 0.1 else 5.0
        period_ms = int(1000.0 / hz)
        if period_ms < 20:
            period_ms = 20
        while True:
            await self._poll_once()
            await asyncio.sleep_ms(period_ms)


def _fmt_limit(v):
    if v is None:
        return "none"
    return str(float(v))
