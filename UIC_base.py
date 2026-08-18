# UIC_base — OLED / RGB / camera / WDT on the UIC (MicroPython + uasyncio).
#
# Compose with MC_Client: mc.set_status_callback(ui.on_status).
# No inheritance from the motion client — apps instantiate both.

import sys
import time

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

from machine import I2C, PWM, Pin, WDT
from rp2 import PIO, StateMachine, asm_pio

import UIC_config as cfg

try:
    DEBUG_LEVEL = int(getattr(cfg, "DEBUG_LEVEL", 3))
except (TypeError, ValueError):
    DEBUG_LEVEL = 3


def dbg(level, *args):
    if DEBUG_LEVEL < level:
        return
    sys.stdout.write("\r\n" + " ".join(str(a) for a in args))


# WS2812 bit timing @ 8 MHz.
@asm_pio(
    sideset_init=PIO.OUT_LOW,
    out_shiftdir=PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24,
    fifo_join=PIO.JOIN_TX,
)
def _ws2812_pio():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1).side(0)[T3 - 1]
    jmp(not_x, "do_zero").side(1)[T1 - 1]
    jmp("bitloop").side(1)[T2 - 1]
    label("do_zero")
    nop().side(0)[T2 - 1]
    wrap()


def _clamp_u8(v):
    try:
        v = int(v)
    except (TypeError, ValueError):
        v = 0
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def _rgb_u8(rgb):
    if rgb is None:
        return (0, 0, 0)
    if isinstance(rgb, (tuple, list)) and len(rgb) >= 3:
        return (_clamp_u8(rgb[0]), _clamp_u8(rgb[1]), _clamp_u8(rgb[2]))
    return (0, 0, 0)


class UIC_Base:
    """Local OLED / RGB / camera / WDT; refresh from MC status callback."""

    def __init__(
        self,
        # status LED (RGB)
        pin_led_r=None,
        pin_led_g=None,
        pin_led_b=None,
        # optional WS2812 NeoPixel
        pin_neopixel=None,
        neo_sm_id=None,
        # optional UART pins to skip camera GPIO collision check
        uart_tx=None,
        uart_rx=None,
    ):
        self._uart_tx = None if uart_tx is None else int(uart_tx)
        self._uart_rx = None if uart_rx is None else int(uart_rx)
        if self._uart_tx is None:
            try:
                import MC_config as _mc

                self._uart_tx = int(getattr(_mc, "PIN_UART_TX", 16))
                self._uart_rx = int(getattr(_mc, "PIN_UART_RX", 17))
            except ImportError:
                self._uart_tx = 16
                self._uart_rx = 17

        # RGB status LED (PWM).
        self._pwm_r = PWM(Pin(pin_led_r if pin_led_r is not None else cfg.PIN_LED_R))
        self._pwm_g = PWM(Pin(pin_led_g if pin_led_g is not None else cfg.PIN_LED_G))
        self._pwm_b = PWM(Pin(pin_led_b if pin_led_b is not None else cfg.PIN_LED_B))
        for pwm in (self._pwm_r, self._pwm_g, self._pwm_b):
            pwm.freq(1000)

        self._neo_sm = None
        neo_pin = pin_neopixel if pin_neopixel is not None else getattr(cfg, "PIN_NEOPIXEL", None)
        neo_sm = neo_sm_id if neo_sm_id is not None else getattr(cfg, "PIO_NEOPIXEL_SM_ID", 1)
        if neo_pin is not None:
            try:
                self._neo_sm = StateMachine(
                    int(neo_sm),
                    _ws2812_pio,
                    freq=8_000_000,
                    sideset_base=Pin(int(neo_pin)),
                )
                self._neo_sm.active(1)
            except Exception as exc:
                dbg(1, "NeoPixel init fail", exc)
                self._neo_sm = None

        # Mirrors for OLED/LED (from on_status + app setters).
        self._speed_mm_s = None
        self._accel_mm_s2 = None
        self._soft_min = None
        self._soft_max = None
        self._enabled = None
        self._state = None
        self._pos_mm = None
        self._target_mm = None

        self._moving = False
        self._homing = False
        self._at_soft_limit = False
        self._near_soft_limit = False
        self._at_hard_limit = False
        self._drv_error_active = False
        self._act_vel_mm_s = 0.0
        self._act_acc_mm_s2 = 0.0
        self._prev_act_speed_abs = None
        self._decelerating = False
        self._accelerating = False

        self._ui_task = None

        self._oled = None
        self._oled_last_ms = 0
        self._oled_app_text = ""
        self._oled_frozen_pos_mm = None
        self._led_branch = None
        # Sticky additive mix on status base (0..255 per channel).
        self._led_add_r = 0
        self._led_add_g = 0
        self._led_add_b = 0
        # Timed effect: dict or None. Kinds: rainbow, flash, blip, pingpong.
        # Preempts status LED until finished (EMO / cam still win).
        self._led_fx = None
        self._luminosity = 1.0
        self._oled_unit = "mm"
        self._oled_badge_tl = False
        self._oled_badge_delay = False
        self._oled_badge_mark = None

        self._pin_cam = None
        self._cam_tl_div = 1
        self._cam_fps = 30
        self._cam_pulse_count = 0
        self._cam_pulse_until = None
        self._cam_next_pulse_ms = None
        self._cam_was_moving = False
        self._cam_level = 0
        self._cam_motion_override = False
        self._cam_manual = False
        self._init_camera_pin()

        self._wdt = None
        self._hb_led = None
        self._hb_on = False
        self._hb_last_ms = time.ticks_ms()
        self._init_heartbeat_led()
        self._init_oled()

        self._drive_led()
        self._update_oled(force=True)

    # --- lifecycle ---------------------------------------------------------

    async def start(self):
        """Start the UI loop (LED / camera / WDT heartbeat)."""
        if self._ui_task is None:
            try:
                self._ui_task = asyncio.create_task(self._ui_loop())
            except Exception:
                pass

    async def stop(self):
        """Cancel the UI loop task."""
        t = self._ui_task
        self._ui_task = None
        if t is not None:
            t.cancel()
            try:
                await t
            except asyncio.CancelledError:
                pass

    # --- app sync helpers (limits / commanded SS·SA for idle OLED) ---------

    def set_soft_limits(self, min_limit, max_limit):
        self._soft_min = min_limit
        self._soft_max = max_limit
        self._refresh_soft_limit_flag()
        self._drive_led()

    def set_commanded(self, speed_mm_s=None, accel_mm_s2=None):
        """Mirror commanded cruise/accel for idle OLED Spd/Acc rows."""
        if speed_mm_s is not None:
            self._speed_mm_s = float(speed_mm_s)
        if accel_mm_s2 is not None:
            self._accel_mm_s2 = float(accel_mm_s2)
        if not self._moving:
            self._update_oled(force=True)

    def set_enabled(self, on):
        self._enabled = bool(on)
        self._drive_led()
        self._update_oled(force=True)

    # --- on_status: OLED + LED ---------------------------------------------

    def on_status(self, state, pos, speed, accel, target):
        """MC_Client status callback — updates OLED and RGB LED."""
        self._state = state
        self._moving = state in ("M", "A", "B", "H")
        self._homing = state == "H"
        self._at_hard_limit = state == "L"
        self._drv_error_active = state == "E"
        if state == "D":
            self._enabled = False
        elif state in ("I", "M", "H", "A", "B"):
            # Verbose status implies not disabled unless letter is D.
            if self._enabled is None:
                self._enabled = True

        if pos is not None:
            self._pos_mm = pos
        if speed is not None:
            self._act_vel_mm_s = float(speed)
        elif state in ("I", "D", "L", "E"):
            self._act_vel_mm_s = 0.0
        self._target_mm = target

        if accel is not None:
            self._act_acc_mm_s2 = float(accel)
        elif self._accel_mm_s2 is not None:
            self._act_acc_mm_s2 = float(self._accel_mm_s2)

        if state == "A":
            self._accelerating = True
            self._decelerating = False
        elif state == "B":
            self._accelerating = False
            self._decelerating = True
        else:
            spd_abs = abs(self._act_vel_mm_s) if self._act_vel_mm_s is not None else 0.0
            if self._moving and not self._homing and self._prev_act_speed_abs is not None:
                eps = getattr(cfg, "LED_ACCEL_SPEED_EPS_MM_S", 3.0)
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
        self._update_oled(force=True)
        self._drive_led()

    def on_error(self, code, text):
        dbg(1, "MC error", code, text)

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

    # --- OLED / LED / camera public API ------------------------------------

    def setOledText(self, text):
        self._oled_app_text = "" if text is None else str(text)
        self._update_oled(force=True)

    def setOledUnit(self, unit):
        u = str(unit).strip().lower() if unit is not None else "mm"
        if u in ("in", "inch", "inches"):
            self._oled_unit = "inch"
        else:
            self._oled_unit = "mm"
        self._update_oled(force=True)

    def getOledUnit(self):
        return self._oled_unit

    def setOledBadges(self, tl=False, delay=False, mark=None):
        tl = bool(tl)
        delay = bool(delay)
        if mark is not None:
            mark = str(mark)
            if not mark:
                mark = None
        if (
            tl == self._oled_badge_tl
            and delay == self._oled_badge_delay
            and mark == self._oled_badge_mark
        ):
            return
        self._oled_badge_tl = tl
        self._oled_badge_delay = delay
        self._oled_badge_mark = mark
        self._update_oled(force=True)

    def setCameraMode(self, tl_div, fps):
        try:
            n = int(tl_div)
        except (TypeError, ValueError):
            n = 1
        if n < 1:
            n = 1
        try:
            f = int(fps)
        except (TypeError, ValueError):
            f = 30
        if f < 1:
            f = 1
        self._cam_tl_div = n
        self._cam_fps = f
        if n == 1:
            self._cam_pulse_until = None
            self._cam_next_pulse_ms = None
            if not self._moving and not self._cam_motion_override:
                self._cam_write(0)

    def setCameraMotionActive(self, active):
        self._cam_motion_override = bool(active)

    def setCameraManual(self, manual):
        self._cam_manual = bool(manual)
        if self._cam_manual:
            self._cam_motion_override = False
            self._cam_pulse_until = None
            self._cam_next_pulse_ms = None
            self._cam_was_moving = False
            self._cam_write(0)

    def pulseCamera(self):
        pulse_ms = int(getattr(cfg, "CTRL_CAMERA_PULSE_MS", 100))
        if pulse_ms < 1:
            pulse_ms = 1
        now = time.ticks_ms()
        self._cam_write(1)
        self._cam_pulse_count += 1
        self._cam_pulse_until = time.ticks_add(now, pulse_ms)
        return pulse_ms

    def getCameraPulseCount(self):
        return int(self._cam_pulse_count)

    def resetCameraPulseCount(self):
        self._cam_pulse_count = 0

    def setLuminosity(self, scale):
        try:
            scale = float(scale)
        except (TypeError, ValueError):
            scale = 1.0
        if scale < 0.0:
            scale = 0.0
        elif scale > 1.0:
            scale = 1.0
        self._luminosity = scale
        self._apply_oled_contrast()
        self._drive_led()

    def getLuminosity(self):
        return self._luminosity

    def startLedRainbowLoop(self, period_ms=None):
        if period_ms is None:
            period_ms = int(getattr(cfg, "LED_RAINBOW_MS", 1000))
        self._led_fx = {
            "kind": "rainbow",
            "period_ms": max(1, int(period_ms)),
            "until": None,
            "t0": time.ticks_ms(),
        }
        self._drive_led()

    def stopLedEffect(self):
        self.ledEffectClear()

    def driveLed(self):
        self._drive_led()

    def ledAddColor(self, r, g, b):
        """Sticky additive overlay on status base. Channels 0..255."""
        self._led_add_r = _clamp_u8(r)
        self._led_add_g = _clamp_u8(g)
        self._led_add_b = _clamp_u8(b)
        self._drive_led()

    def ledClearAdd(self):
        """Clear sticky ledAddColor overlay."""
        self._led_add_r = 0
        self._led_add_g = 0
        self._led_add_b = 0
        self._drive_led()

    def ledFlash(self, rgb, count, on_ms, off_ms=None):
        """Flash ``rgb`` (r,g,b 0..255) ``count`` times; preempts status LED."""
        if off_ms is None:
            off_ms = on_ms
        count = int(count)
        if count < 1:
            return
        on_ms = max(1, int(on_ms))
        off_ms = max(0, int(off_ms))
        self._led_fx = {
            "kind": "flash",
            "rgb": _rgb_u8(rgb),
            "count": count,
            "on_ms": on_ms,
            "off_ms": off_ms,
            "i": 0,
            "phase_on": True,
            "phase_until": time.ticks_add(time.ticks_ms(), on_ms),
        }
        self._drive_led()

    def ledBlip(self, rgb, ms):
        """Single pulse of ``rgb`` for ``ms``, then restore status LED."""
        ms = max(1, int(ms))
        self._led_fx = {
            "kind": "blip",
            "rgb": _rgb_u8(rgb),
            "until": time.ticks_add(time.ticks_ms(), ms),
        }
        self._drive_led()

    def ledPingPong(self, rgb_a, rgb_b, period_ms, duration_ms=None):
        """Alternate A↔B. duration_ms=None until ledEffectClear()."""
        period_ms = max(2, int(period_ms))
        half = max(1, period_ms // 2)
        until = None
        if duration_ms is not None:
            until = time.ticks_add(time.ticks_ms(), max(1, int(duration_ms)))
        self._led_fx = {
            "kind": "pingpong",
            "rgb_a": _rgb_u8(rgb_a),
            "rgb_b": _rgb_u8(rgb_b),
            "half_ms": half,
            "phase": 0,
            "phase_until": time.ticks_add(time.ticks_ms(), half),
            "until": until,
        }
        self._drive_led()

    def ledEffectClear(self):
        """Cancel timed LED effect early."""
        self._led_fx = None
        self._drive_led()

    async def playLedRainbow(self, duration_ms=None):
        if duration_ms is None:
            duration_ms = int(getattr(cfg, "LED_RAINBOW_MS", 1000))
        duration_ms = max(1, int(duration_ms))
        self._led_fx = {
            "kind": "rainbow",
            "period_ms": duration_ms,
            "until": time.ticks_add(time.ticks_ms(), duration_ms),
            "t0": time.ticks_ms(),
        }
        try:
            while self._led_fx is not None and self._led_fx.get("kind") == "rainbow":
                fx = self._led_fx
                if fx.get("until") is not None and time.ticks_diff(
                    fx["until"], time.ticks_ms()
                ) <= 0:
                    break
                self._drive_led()
                await asyncio.sleep_ms(20)
        finally:
            if self._led_fx is not None and self._led_fx.get("kind") == "rainbow":
                self._led_fx = None
                self._drive_led()

    def isDRVErrorActive(self):
        return self._drv_error_active or self._state == "E"

    # --- LED ---------------------------------------------------------------

    def _led_set_rgb255(self, r, g, b):
        """Drive PWM/NeoPixel from 0..255 channels."""
        full = 65535
        lum = self._luminosity

        def duty(level255):
            level = _clamp_u8(level255) / 255.0 * lum
            if level > 1.0:
                level = 1.0
            d = int(level * full + 0.5)
            if not cfg.LED_ACTIVE_HIGH:
                d = full - d
            return d

        self._pwm_r.duty_u16(duty(r))
        self._pwm_g.duty_u16(duty(g))
        self._pwm_b.duty_u16(duty(b))

        if self._neo_sm is not None:

            def byte(level255):
                level = _clamp_u8(level255) / 255.0 * lum
                if level > 1.0:
                    level = 1.0
                return int(level * 255 + 0.5)

            grb = (byte(g) << 16) | (byte(r) << 8) | byte(b)
            try:
                self._neo_sm.put(grb, 8)
            except Exception:
                pass

    def _led_hsv255(self, h, s, v):
        """HSV with h,s,v in 0..1 → RGB 0..255."""
        if s <= 0.0:
            c = int(v * 255 + 0.5)
            return c, c, c
        h = h % 1.0
        i = int(h * 6.0)
        f = h * 6.0 - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))
        i = i % 6
        if i == 0:
            r, g, b = v, t, p
        elif i == 1:
            r, g, b = q, v, p
        elif i == 2:
            r, g, b = p, v, t
        elif i == 3:
            r, g, b = p, q, v
        elif i == 4:
            r, g, b = t, p, v
        else:
            r, g, b = v, p, q
        return (
            int(r * 255 + 0.5),
            int(g * 255 + 0.5),
            int(b * 255 + 0.5),
        )

    def _led_log_branch(self, branch, r, g, b):
        if branch == self._led_branch:
            return
        self._led_branch = branch
        dbg(
            5,
            "led",
            branch,
            "rgb",
            int(r),
            int(g),
            int(b),
            "state",
            self._state,
        )

    def _led_dim255(self, key, default01):
        """Config duty 0..1 → 0..255."""
        v = float(getattr(cfg, key, default01))
        if v < 0.0:
            v = 0.0
        if v > 1.0:
            v = 1.0
        return int(v * 255 + 0.5)

    def _led_tick_fx(self, now):
        """Advance timed effect; return (r,g,b) or None if inactive."""
        fx = self._led_fx
        if fx is None:
            return None
        kind = fx.get("kind")
        if kind == "rainbow":
            until = fx.get("until")
            if until is not None and time.ticks_diff(until, now) <= 0:
                self._led_fx = None
                return None
            period = max(1, int(fx.get("period_ms", 1000)))
            t0 = fx.get("t0", now)
            done = (time.ticks_diff(now, t0) % period) / float(period)
            return self._led_hsv255(done, 1.0, 0.55)

        if kind == "blip":
            if time.ticks_diff(fx["until"], now) <= 0:
                self._led_fx = None
                return None
            return fx["rgb"]

        if kind == "flash":
            while time.ticks_diff(fx["phase_until"], now) <= 0:
                if fx["phase_on"]:
                    fx["phase_on"] = False
                    fx["phase_until"] = time.ticks_add(now, fx["off_ms"])
                    if fx["off_ms"] <= 0:
                        fx["i"] += 1
                        if fx["i"] >= fx["count"]:
                            self._led_fx = None
                            return None
                        fx["phase_on"] = True
                        fx["phase_until"] = time.ticks_add(now, fx["on_ms"])
                else:
                    fx["i"] += 1
                    if fx["i"] >= fx["count"]:
                        self._led_fx = None
                        return None
                    fx["phase_on"] = True
                    fx["phase_until"] = time.ticks_add(now, fx["on_ms"])
            if fx["phase_on"]:
                return fx["rgb"]
            return (0, 0, 0)

        if kind == "pingpong":
            until = fx.get("until")
            if until is not None and time.ticks_diff(until, now) <= 0:
                self._led_fx = None
                return None
            while time.ticks_diff(fx["phase_until"], now) <= 0:
                fx["phase"] = 1 - int(fx["phase"])
                fx["phase_until"] = time.ticks_add(now, fx["half_ms"])
            return fx["rgb_a"] if fx["phase"] == 0 else fx["rgb_b"]

        self._led_fx = None
        return None

    def _led_status_base255(self, _now=None):
        """Status-derived RGB 0..255 (no soft-limit mix, no sticky add)."""
        dim = self._led_dim255("LED_DIM_WHITE", 0.12)
        dim_orange = self._led_dim255("LED_DIM_ORANGE", 0.12)

        min_spd = 0.006
        try:
            import MC_config as _mc

            min_spd = float(getattr(_mc, "MIN_SPEED_MM_S", 0.006))
        except ImportError:
            pass
        act = abs(self._act_vel_mm_s) if self._act_vel_mm_s is not None else 0.0

        if self._moving and act >= min_spd and not self._homing:
            if self._decelerating or self._accelerating:
                return (255, 255, 0), "motion_accel"
            return (0, 255, 0), "motion_cruise"

        if self._enabled:
            return (dim, dim, dim), "enabled"

        return (dim_orange, int(dim_orange * 0.35 + 0.5), 0), "disabled"

    def _drive_led(self):
        now = time.ticks_ms()
        blink_home = ((now // cfg.LED_BLINK_MS) % 2) == 0
        hard_ms = int(getattr(cfg, "LED_BLINK_HARD_LIMIT_MS", 80))
        blink_hard = ((now // max(hard_ms, 1)) % 2) == 0

        if self.isDRVErrorActive():
            self._led_log_branch("emo", 255, 0, 0)
            self._led_set_rgb255(255, 0, 0)
            return

        if self._cam_pulse_until is not None and self._cam_level:
            self._led_log_branch("cam_pulse", 0, 0, 0)
            self._led_set_rgb255(0, 0, 0)
            return

        fx_rgb = self._led_tick_fx(now)
        if fx_rgb is not None:
            r, g, b = fx_rgb
            self._led_log_branch("fx", r, g, b)
            self._led_set_rgb255(r, g, b)
            return

        if self._at_hard_limit:
            if blink_hard:
                self._led_log_branch("hard_limit", 255, 0, 0)
                self._led_set_rgb255(255, 0, 0)
            else:
                self._led_log_branch("hard_limit", 0, 0, 0)
                self._led_set_rgb255(0, 0, 0)
            return

        if self._homing:
            if blink_home:
                self._led_log_branch("homing", 255, 0, 0)
                self._led_set_rgb255(255, 0, 0)
            else:
                self._led_log_branch("homing", 0, 0, 0)
                self._led_set_rgb255(0, 0, 0)
            return

        (r, g, b), branch = self._led_status_base255(now)

        if self._at_soft_limit:
            b = _clamp_u8(b + int(getattr(cfg, "LED_SOFT_AT_BLUE_ADD", 255)))
            if branch in ("enabled", "disabled"):
                branch = "soft_limit"
        elif self._near_soft_limit:
            b = _clamp_u8(b + int(getattr(cfg, "LED_SOFT_NEAR_BLUE_ADD", 76)))
            if branch in ("enabled", "disabled"):
                branch = "near_soft"

        r = _clamp_u8(r + self._led_add_r)
        g = _clamp_u8(g + self._led_add_g)
        b = _clamp_u8(b + self._led_add_b)

        self._led_log_branch(branch, r, g, b)
        self._led_set_rgb255(r, g, b)


    # --- camera / WDT / UI loop --------------------------------------------

    def _init_heartbeat_led(self):
        pin_id = getattr(cfg, "PIN_LED_ONBOARD", "LED")
        try:
            self._hb_led = Pin(pin_id, Pin.OUT)
        except Exception:
            try:
                self._hb_led = Pin(25, Pin.OUT)
            except Exception:
                self._hb_led = None
        if self._hb_led is not None:
            self._hb_led.value(0)

    def _init_camera_pin(self):
        pin_id = getattr(cfg, "PIN_CTRL_CAMERA", None)
        if pin_id is None:
            self._pin_cam = None
            return
        try:
            pin_i = int(pin_id)
        except (TypeError, ValueError):
            self._pin_cam = None
            return
        if pin_i == self._uart_tx or pin_i == self._uart_rx:
            dbg(1, "CTRL_CAMERA skipped: pin conflicts UART", pin_i)
            self._pin_cam = None
            return
        try:
            self._pin_cam = Pin(pin_i, Pin.OUT)
            self._cam_write(0)
        except Exception as exc:
            dbg(1, "CTRL_CAMERA init fail", exc)
            self._pin_cam = None

    def _cam_write(self, level):
        level = 1 if level else 0
        self._cam_level = level
        if self._pin_cam is None:
            return
        active_high = bool(getattr(cfg, "CTRL_CAMERA_ACTIVE_HIGH", True))
        self._pin_cam.value(level if active_high else (0 if level else 1))

    def _tick_camera_ctrl(self):
        now = time.ticks_ms()
        if self._cam_manual:
            if self._cam_pulse_until is not None:
                if time.ticks_diff(now, self._cam_pulse_until) >= 0:
                    self._cam_write(0)
                    self._cam_pulse_until = None
            return

        moving = bool(self._moving or self._cam_motion_override)
        tl = int(self._cam_tl_div)
        if tl < 1:
            tl = 1
        fps = int(self._cam_fps)
        if fps < 1:
            fps = 1

        if moving and not self._cam_was_moving:
            if tl != 1:
                self._cam_pulse_count = 0
                self._cam_next_pulse_ms = time.ticks_ms()
                self._cam_pulse_until = None
        if not moving and self._cam_was_moving:
            self._cam_pulse_until = None
            self._cam_next_pulse_ms = None
            self._cam_write(0)
        self._cam_was_moving = moving

        if not moving:
            if self._cam_level:
                self._cam_write(0)
            return

        if tl == 1:
            if not self._cam_level:
                self._cam_write(1)
            return

        pulse_ms = int(getattr(cfg, "CTRL_CAMERA_PULSE_MS", 100))
        if pulse_ms < 1:
            pulse_ms = 1
        period_ms = int(1000.0 * float(tl) / float(fps))
        if period_ms < pulse_ms + 10:
            period_ms = pulse_ms + 10

        if self._cam_pulse_until is not None:
            if time.ticks_diff(now, self._cam_pulse_until) >= 0:
                self._cam_write(0)
                self._cam_pulse_until = None
            return

        if self._cam_next_pulse_ms is None:
            self._cam_next_pulse_ms = now

        if time.ticks_diff(now, self._cam_next_pulse_ms) >= 0:
            self._cam_write(1)
            self._cam_pulse_count += 1
            self._cam_pulse_until = time.ticks_add(now, pulse_ms)
            self._cam_next_pulse_ms = time.ticks_add(now, period_ms)

    def _start_watchdog(self):
        if self._wdt is not None:
            return
        if not getattr(cfg, "WDT_ENABLED", True):
            return
        try:
            timeout = int(getattr(cfg, "WDT_TIMEOUT_MS", 3000))
            hb = int(getattr(cfg, "WDT_HEARTBEAT_MS", 1000))
            if timeout <= hb:
                timeout = hb * 3
            if timeout > 8388:
                timeout = 8388
            self._wdt = WDT(timeout=timeout)
            self._hb_last_ms = time.ticks_ms()
            if self._hb_led is not None:
                self._hb_on = True
                self._hb_led.value(1)
        except Exception as exc:
            dbg(1, "WDT init fail", exc)
            self._wdt = None

    def _tick_heartbeat(self):
        period = int(getattr(cfg, "WDT_HEARTBEAT_MS", 1000))
        if period < 50:
            period = 50
        now = time.ticks_ms()
        if time.ticks_diff(now, self._hb_last_ms) < period:
            return
        self._hb_last_ms = now
        self._hb_on = not self._hb_on
        if self._hb_led is not None:
            self._hb_led.value(1 if self._hb_on else 0)
        if self._wdt is not None:
            self._wdt.feed()

    async def _ui_loop(self):
        self._start_watchdog()
        while True:
            try:
                self._drive_led()
                self._tick_camera_ctrl()
                self._tick_heartbeat()
            except Exception as exc:
                dbg(1, "_ui_loop", repr(exc))
            await asyncio.sleep_ms(20)

    # --- OLED --------------------------------------------------------------

    def _init_oled(self):
        self._oled = None
        if not getattr(cfg, "DSP_ENABLED", False):
            return
        try:
            i2c = I2C(
                cfg.DSP_I2C_ID,
                sda=Pin(cfg.PIN_DSP_I2C_SDA),
                scl=Pin(cfg.PIN_DSP_I2C_SCL),
                freq=cfg.DSP_I2C_FREQ,
            )
            addr = cfg.DSP_I2C_ADDR
            if addr not in i2c.scan():
                return
            driver = str(getattr(cfg, "DSP_DRIVER", "ssd1306")).lower().strip()
            if driver in ("ssh1106", "ch1115", "ch1116"):
                driver = "sh1106"
            if driver in ("ssh1306", "ssd1306"):
                driver = "ssd1306"
            rotate180 = bool(getattr(cfg, "DSP_ROTATE_180", False))
            kwargs = dict(
                width=cfg.DSP_WIDTH,
                height=cfg.DSP_HEIGHT,
                i2c=i2c,
                addr=addr,
                rotate180=rotate180,
            )
            if driver == "sh1106":
                from sh1106 import SH1106_I2C

                self._oled = SH1106_I2C(**kwargs)
            elif driver == "ssd1309":
                from ssd1309 import SSD1309_I2C

                self._oled = SSD1309_I2C(**kwargs)
            else:
                from ssd1306 import SSD1306_I2C

                self._oled = SSD1306_I2C(**kwargs)
            self._apply_oled_contrast()
        except Exception:
            self._oled = None

    def _apply_oled_contrast(self):
        if self._oled is None:
            return
        full = int(getattr(cfg, "DSP_CONTRAST_FULL", 0xFF))
        if full < 1:
            full = 1
        elif full > 255:
            full = 255
        c = int(full * self._luminosity + 0.5)
        if c < 1:
            c = 1
        elif c > 255:
            c = 255
        try:
            self._oled.contrast(c)
        except Exception:
            pass

    @staticmethod
    def _fmt_oled_num(value, unit="mm"):
        v = float(value)
        if unit == "inch":
            decimals = 2
            hi, lo_fit, lo_red = 999.99, -99.99, -999.9
        else:
            decimals = 1
            hi, lo_fit, lo_red = 9999.9, -999.9, -9999.0
        if v > hi:
            v = hi
        elif v < lo_fit:
            decimals -= 1
            if v < lo_red:
                v = lo_red
        return "{:6.{}f}".format(v, decimals)

    def _oled_display_values(self, pos_mm, spd_mm_s, acc_mm_s2):
        if self._oled_unit == "inch":
            scale = 1.0 / 25.4
            return (
                pos_mm * scale,
                spd_mm_s * scale,
                acc_mm_s2 * scale,
                "in",
                "in/s",
                "in/s2",
            )
        elif self._oled_unit == "degree":
            return pos_mm, spd_mm_s, acc_mm_s2, "°", "°/s", "°/s2"
        return pos_mm, spd_mm_s, acc_mm_s2, "mm", "mm/s", "mm/s2"

    def _update_oled(self, force=False):
        if self._oled is None:
            return
        if not force:
            return
        self._oled_last_ms = time.ticks_ms()
        self._oled_draw_frame(live_pos=True)
        try:
            self._oled.show()
        except Exception:
            self._oled = None

    def _oled_draw_frame(self, live_pos=True):
        if self._oled is None:
            return

        if self._moving:
            spd_mm = self._act_vel_mm_s if self._act_vel_mm_s is not None else 0.0
            acc_mm = self._act_acc_mm_s2 if self._act_acc_mm_s2 else self._accel_mm_s2
            spd_label = "Spd*"
            acc_label = "Acc*"
        else:
            spd_mm = self._speed_mm_s if self._speed_mm_s is not None else 0.0
            acc_mm = self._accel_mm_s2 if self._accel_mm_s2 is not None else 0.0
            spd_label = "Spd"
            acc_label = "Acc"

        if live_pos or not self._moving:
            pos_mm = self._pos_mm if self._pos_mm is not None else 0.0
            self._oled_frozen_pos_mm = pos_mm
        else:
            if self._oled_frozen_pos_mm is None:
                self._oled_frozen_pos_mm = (
                    self._pos_mm if self._pos_mm is not None else 0.0
                )
            pos_mm = self._oled_frozen_pos_mm

        pos, spd, acc, u_pos, u_spd, u_acc = self._oled_display_values(
            pos_mm, spd_mm, acc_mm
        )
        if self._homing:
            status = "HOMING"
        elif self._at_hard_limit:
            status = "HARD LIMIT"
        elif self.isDRVErrorActive():
            status = "EMO"
        elif not self._enabled:
            status = "DISABLED"
        elif self._at_soft_limit:
            status = "LIMIT"
        else:
            status = ""

        badges = ""
        if self._oled_badge_delay:
            badges = "D"
        if self._oled_badge_tl:
            badges = (badges + " TL") if badges else "TL"
        if self._oled_badge_mark:
            badges = (
                (badges + " " + self._oled_badge_mark)
                if badges
                else self._oled_badge_mark
            )

        try:
            import oledfont as _ofont

            oled = self._oled
            oled.fill(0)

            if status:
                oled.text(status, 0, 4)
            if badges:
                oled.text(badges, 128 - 8 * len(badges), 4)

            num_x = 28
            unit_x = 78
            rows = (
                (17, "Pos", self._fmt_oled_num(pos, self._oled_unit), u_pos),
                (27, spd_label, self._fmt_oled_num(spd, self._oled_unit), u_spd),
                (37, acc_label, self._fmt_oled_num(acc, self._oled_unit), u_acc),
            )
            for y, label, num, unit in rows:
                _ofont.text(oled, label, 0, y + 1)
                oled.text(num, num_x, y)
                _ofont.text(oled, unit, unit_x, y + 1)

            app = self._oled_app_text
            if app:
                line_w = 21
                lines = app.replace("\r", "").split("\n")
                drawn = 0
                for line in lines:
                    while line and drawn < 2:
                        _ofont.text(oled, line[:line_w], 0, 48 + drawn * 8)
                        line = line[line_w:]
                        drawn += 1
                    if drawn >= 2:
                        break
        except Exception:
            self._oled = None
