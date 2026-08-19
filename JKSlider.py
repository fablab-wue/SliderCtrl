# JKSlider — motorized camera slider control panel.
#
# Uses SliderCtrl (MC_client.py / UIC_base.py) + uasyncio over UART to SliderMC.
#
# Wiring (Raspberry Pi Pico UIC). Panel pins: JKSliderConfig.py
# Display / camera / RGB: UIC_config.py; motion: MC_config.py + MC_Client UART GP16/17.
# Hardware overrides: SliderPins.py (one file per slider HW).
#   POT_SPEED / POT_ACCEL / POT_JOYSTICK pots -> GP26 / GP27 / GP28 (ADC)
#   JKS_INPUT_MODE = "button": one GPIO per BTN_* (active-low, pull-ups)
#     BTN_STOP=GP5, MOVE=6/7, FAST=8/9, A/B/C=10/11/12, OPTION=13, DELAY=14, TL=15
#   JKS_INPUT_MODE = "keypad": KP_ROW1..4=GP6-9, KP_COL1..3=GP10-12 (High-Z row scan);
#     discrete BTN_STOP on GP5 and BTN_OPTION on GP13 (ORed with matrix).
#     See manuals/JKSlider_Technical_Manual_Panel.md.
#
# Usage:
#   import JKSlider
#   JKSlider.run()

import time

import uasyncio as asyncio
from machine import ADC, Pin

import MC_config as config
import JKSliderConfig as jks
import UIC_config as uic_cfg
from MC_client import MC_Client
from UIC_base import UIC_Base, dbg
from button_state import allow_move_out_of_soft_limit, resolve_move_semantics

_IDLE = 0
_CRUISE = 1
_FAST = 2
_GOTO = 3
_LOOP = 4
_HOMING = 5
_JOYSTICK = 6

_OLED_FLASH_MS = 1500
_TL_DIVIDERS = (1, 5, 10, 25, 30, 50, 60, 100)

_RED = (255, 0, 0)
_OFF = (0, 0, 0)


def _duty_u8(duty01):
    try:
        d = float(duty01)
    except (TypeError, ValueError):
        d = 0.12
    if d < 0.0:
        d = 0.0
    elif d > 1.0:
        d = 1.0
    return int(d * 255 + 0.5)


class _Btn:
    """Debounced button with short / long / extra-long press.

    pin_no set: active-low GPIO with pull-up (button mode).
    pin_no None: virtual key; pass raw=True/False into update().
    """

    def __init__(self, pin_no, debounce_ms, long_ms, extra_long_ms=None):
        self._pin = (
            None if pin_no is None else Pin(pin_no, Pin.IN, Pin.PULL_UP)
        )
        self._debounce_ms = debounce_ms
        self._long_ms = long_ms
        self._extra_long_ms = extra_long_ms
        self._stable = False
        self._raw_last = False
        self._change_ms = time.ticks_ms()
        self._down_ms = None
        self._long_fired = False
        self._extra_long_fired = False
        self.edge_press = False
        self.edge_release = False
        self.short_press = False
        self.long_press = False
        self.extra_long_press = False
        self.last_hold_ms = 0

    def pressed(self):
        return self._stable

    def hold_ms(self):
        """Milliseconds held so far while pressed (0 if up)."""
        if self._stable and self._down_ms is not None:
            return time.ticks_diff(time.ticks_ms(), self._down_ms)
        return 0

    def update(self, raw=None):
        self.edge_press = False
        self.edge_release = False
        self.short_press = False
        self.long_press = False
        self.extra_long_press = False

        if raw is None:
            if self._pin is None:
                raw = False
            else:
                raw = self._pin.value() == 0  # active low
        else:
            raw = bool(raw)
        now = time.ticks_ms()

        if raw != self._raw_last:
            self._raw_last = raw
            self._change_ms = now

        if time.ticks_diff(now, self._change_ms) >= self._debounce_ms:
            if raw != self._stable:
                self._stable = raw
                if self._stable:
                    self.edge_press = True
                    self._down_ms = now
                    self._long_fired = False
                    self._extra_long_fired = False
                    self.last_hold_ms = 0
                else:
                    self.edge_release = True
                    if self._down_ms is not None:
                        self.last_hold_ms = time.ticks_diff(now, self._down_ms)
                        if not self._long_fired:
                            self.short_press = True
                    self._down_ms = None

        if self._stable and self._down_ms is not None:
            held = time.ticks_diff(now, self._down_ms)
            if not self._long_fired and held >= self._long_ms:
                self._long_fired = True
                self.long_press = True
            if (
                self._extra_long_ms is not None
                and not self._extra_long_fired
                and held >= self._extra_long_ms
            ):
                self._extra_long_fired = True
                self.extra_long_press = True


# 4x3 keypad: rows GP6-9 (KP_ROW1..KP_ROW4), cols GP10-12 (KP_COL1..KP_COL3).
# Duplicate OPTION keys OR together. KP_ROW1 = upper keys (GP6).
_KEYPAD_LAYOUT = (
    ("MOVE_L", "DELAY", "MOVE_R"),  # KP_ROW1 GP6
    ("FAST_L", "TIMELAPSE", "FAST_R"),  # KP_ROW2 GP7
    ("A", "B", "C"),  # KP_ROW3 GP8
    ("OPTION", "STOP", "OPTION"),  # KP_ROW4 GP9
)


class _KeypadScanner:
    """Scan keypad matrix: idle rows Hi-Z (IN), active row OUT/LOW; cols IN+PULL_UP.

    No row diodes required; Hi-Z idle avoids GPIO fights on multi-key presses.
    """

    def __init__(self, row_pins, col_pins, layout=_KEYPAD_LAYOUT):
        self._rows = [Pin(int(p), Pin.IN) for p in row_pins]
        self._cols = [Pin(int(p), Pin.IN, Pin.PULL_UP) for p in col_pins]
        self._layout = layout

    def read(self):
        """Return set of pressed logical key names.

        Both bottom OPTION cells become OPTION; if both are down, also
        DOUBLE_OPTION (for emergency halt with STOP).
        """
        active = set()
        option_n = 0
        for r, row in enumerate(self._rows):
            row.init(Pin.OUT, value=0)
            time.sleep_us(5)
            for c, col in enumerate(self._cols):
                if col.value() == 0:
                    name = self._layout[r][c]
                    if name == "OPTION":
                        option_n += 1
                    elif name:
                        active.add(name)
            row.init(Pin.IN)
        if option_n >= 1:
            active.add("OPTION")
        if option_n >= 2:
            active.add("DOUBLE_OPTION")
        return active


def _filter_keypad_line_ghosts(active):
    """Drop known matrix ghosts on this layout.

    Classic ghost: OPTION + two of A/B/C → false matrix STOP.
    Dual OPTION keys make OPTION+MOVE_L+MOVE_R / OPTION+FAST_L+FAST_R safe.
    """
    active = set(active)
    abc_n = (
        (1 if "A" in active else 0)
        + (1 if "B" in active else 0)
        + (1 if "C" in active else 0)
    )
    if "OPTION" in active and abc_n >= 2:
        active.discard("STOP")
    return active


def _make_panel_inputs(debounce, long_ms, stop_halt_ms, stop_disable_ms):
    """Build button objects + update_fn for button or keypad mode."""
    mode = str(getattr(jks, "JKS_INPUT_MODE", "button")).lower()
    if mode == "keypad":
        row_pins = getattr(jks, "PIN_KEYPAD_ROWS", (6, 7, 8, 9))
        col_pins = getattr(jks, "PIN_KEYPAD_COLS", (10, 11, 12))
        scanner = _KeypadScanner(row_pins, col_pins)
        stop_pin = Pin(jks.PIN_BTN_STOP, Pin.IN, Pin.PULL_UP)
        option_pin = Pin(jks.PIN_BTN_OPTION, Pin.IN, Pin.PULL_UP)

        def _vbtn():
            return _Btn(None, debounce, long_ms)

        btn_move_l = _vbtn()
        btn_move_r = _vbtn()
        btn_fast_l = _vbtn()
        btn_fast_r = _vbtn()
        btn_stop = _Btn(
            None, debounce, stop_halt_ms, extra_long_ms=stop_disable_ms
        )
        btn_a = _vbtn()
        btn_b = _vbtn()
        btn_c = _vbtn()
        btn_option = _vbtn()
        btn_delay = _vbtn()
        btn_tl = _vbtn()
        btn_double_option = _vbtn()
        by_name = {
            "MOVE_L": btn_move_l,
            "MOVE_R": btn_move_r,
            "FAST_L": btn_fast_l,
            "FAST_R": btn_fast_r,
            "STOP": btn_stop,
            "A": btn_a,
            "B": btn_b,
            "C": btn_c,
            "OPTION": btn_option,
            "DELAY": btn_delay,
            "TIMELAPSE": btn_tl,
        }

        def update_all():
            active = _filter_keypad_line_ghosts(scanner.read())
            # Discrete GP5 STOP always wins (real E-stop), after filter.
            if stop_pin.value() == 0:
                active.add("STOP")
            # Discrete GP13 OPTION ORed with matrix * (DOUBLE_OPTION stays matrix-only).
            if option_pin.value() == 0:
                active.add("OPTION")
            for name, btn in by_name.items():
                btn.update(name in active)
            btn_double_option.update("DOUBLE_OPTION" in active)

        dbg(4, "input keypad", list(row_pins), list(col_pins))
    else:
        btn_move_l = _Btn(jks.PIN_BTN_MOVE_L, debounce, long_ms)
        btn_move_r = _Btn(jks.PIN_BTN_MOVE_R, debounce, long_ms)
        btn_fast_l = _Btn(jks.PIN_BTN_FAST_L, debounce, long_ms)
        btn_fast_r = _Btn(jks.PIN_BTN_FAST_R, debounce, long_ms)
        btn_stop = _Btn(
            jks.PIN_BTN_STOP,
            debounce,
            stop_halt_ms,
            extra_long_ms=stop_disable_ms,
        )
        btn_a = _Btn(jks.PIN_BTN_A, debounce, long_ms)
        btn_b = _Btn(jks.PIN_BTN_B, debounce, long_ms)
        btn_c = _Btn(jks.PIN_BTN_C, debounce, long_ms)
        btn_option = _Btn(jks.PIN_BTN_OPTION, debounce, long_ms)
        btn_delay = _Btn(jks.PIN_BTN_DELAY, debounce, long_ms)
        btn_tl = _Btn(jks.PIN_BTN_TIMELAPSE, debounce, long_ms)
        btn_double_option = _Btn(None, debounce, long_ms)

        def update_all():
            for b in (
                btn_move_l,
                btn_move_r,
                btn_fast_l,
                btn_fast_r,
                btn_stop,
                btn_a,
                btn_b,
                btn_c,
                btn_option,
                btn_delay,
                btn_tl,
            ):
                b.update()
            btn_double_option.update(False)

        dbg(4, "input button")

    named_buttons = (
        ("MOVE_L", btn_move_l),
        ("MOVE_R", btn_move_r),
        ("FAST_L", btn_fast_l),
        ("FAST_R", btn_fast_r),
        ("STOP", btn_stop),
        ("A", btn_a),
        ("B", btn_b),
        ("C", btn_c),
        ("OPTION", btn_option),
        ("DELAY", btn_delay),
        ("TIMELAPSE", btn_tl),
    )
    return (
        btn_move_l,
        btn_move_r,
        btn_fast_l,
        btn_fast_r,
        btn_stop,
        btn_a,
        btn_b,
        btn_c,
        btn_option,
        btn_delay,
        btn_tl,
        btn_double_option,
        named_buttons,
        update_all,
    )


class _PotFilter:
    """Oversample + EMA + hysteresis on ADC pots (output 0..1)."""

    def __init__(self, samples, alpha, hyst):
        self._n = max(1, int(samples))
        self._alpha = float(alpha)
        if self._alpha < 0.0:
            self._alpha = 0.0
        elif self._alpha > 1.0:
            self._alpha = 1.0
        self._hyst = max(0.0, float(hyst))
        self._y = None
        self._out = 0.0

    def read_norm(self, adc):
        n = self._n
        total = 0
        for _ in range(n):
            total += adc.read_u16()
        x = (total / n) / 65535.0
        if self._y is None:
            self._y = x
            self._out = x
            return self._out
        self._y += self._alpha * (x - self._y)
        if abs(self._y - self._out) >= self._hyst:
            self._out = self._y
        return self._out


def _pot_norm(norm, deadzone):
    """Map filtered 0..1 reading through low-end deadzone → 0..1."""
    if norm < deadzone:
        return 0.0
    norm = (norm - deadzone) / (1.0 - deadzone)
    if norm > 1.0:
        return 1.0
    return norm


def _read_speed_mm_s(filt, adc, min_speed_mm_s, max_speed_mm_s):
    """SPEED pot with fine low-end curve (gamma). Maps to min..max."""
    lo = float(min_speed_mm_s)
    hi = float(max_speed_mm_s)
    if hi < lo:
        hi = lo
    norm = _pot_norm(filt.read_norm(adc), jks.JKS_SPEED_DEADZONE)
    gamma = jks.JKS_SPEED_CURVE_GAMMA
    if gamma != 1.0 and norm > 0.0:
        norm = norm ** gamma
    if norm <= 0.0:
        return lo
    return lo + norm * (hi - lo)


def _read_accel_mm_s2(filt, adc, min_accel_mm_s2, max_accel_mm_s2):
    """ACCEL pot: left/low = min, right/high = max."""
    norm = _pot_norm(filt.read_norm(adc), jks.JKS_SPEED_DEADZONE)
    lo = float(min_accel_mm_s2)
    hi = float(max_accel_mm_s2)
    if hi < lo:
        hi = lo
    return lo + norm * (hi - lo)


def _clamp_joy_center(center):
    """Keep calibrated centre away from ends so both sides map cleanly."""
    try:
        c = float(center)
    except (TypeError, ValueError):
        c = 0.5
    if c < 0.05:
        return 0.05
    if c > 0.95:
        return 0.95
    return c


def _read_joystick_mm_s(filt, adc, max_speed_mm_s, swap_lr, joy_center=0.5):
    """Centre-return joystick → signed speed (mm/s).

    joy_center is the calibrated 0-speed pot reading (0..1). Default 0.5.
    swap_lr inverts stick polarity so it stays consistent with MOVE/FAST
    after a handedness toggle.
    """
    deadzone = jks.JOYSTICK_DEADZONE
    center = _clamp_joy_center(joy_center)
    raw = filt.read_norm(adc)
    # Map 0..1 around centre → -1 .. +1 (asymmetric spans OK).
    if raw >= center:
        span = 1.0 - center
        norm = ((raw - center) / span) if span > 1e-6 else 0.0
    else:
        span = center
        norm = -((center - raw) / span) if span > 1e-6 else 0.0
    if norm > 1.0:
        norm = 1.0
    elif norm < -1.0:
        norm = -1.0

    if abs(norm) <= deadzone:
        return 0.0

    sign = 1.0 if norm > 0.0 else -1.0
    magnitude = (abs(norm) - deadzone) / (1.0 - deadzone)
    if magnitude > 1.0:
        magnitude = 1.0
    gamma = getattr(jks, "JKS_JOYSTICK_CURVE_GAMMA", 1.0)
    if gamma != 1.0 and magnitude > 0.0:
        magnitude = magnitude ** gamma
    speed = sign * magnitude * max_speed_mm_s
    if not jks.JKS_LEFT_IS_NEGATIVE:
        speed = -speed
    if swap_lr:
        speed = -speed
    return speed


def _signed(direction_positive, speed):
    if jks.JKS_LEFT_IS_NEGATIVE:
        return speed if direction_positive else -speed
    return -speed if direction_positive else speed


def _default_positions(soft_min, soft_max):
    pos_a = soft_min if soft_min is not None else 0.0
    pos_c = soft_max if soft_max is not None else 0.0
    if soft_min is not None and soft_max is not None:
        pos_b = 0.5 * (soft_min + soft_max)
    else:
        pos_b = pos_a
    return pos_a, pos_b, pos_c


def _clamp_store_position(pos_mm, soft_min, soft_max):
    margin = jks.JKS_STORE_MARGIN_MM
    lo = (soft_min + margin) if soft_min is not None else pos_mm
    hi = (soft_max - margin) if soft_max is not None else pos_mm
    if soft_min is not None and soft_max is not None and lo > hi:
        mid = 0.5 * (float(soft_min) + float(soft_max))
        return mid
    if soft_min is not None and pos_mm < lo:
        return lo
    if soft_max is not None and pos_mm > hi:
        return hi
    return pos_mm


def _tl_index_for(div):
    for i, d in enumerate(_TL_DIVIDERS):
        if d == div:
            return i
    return 0


def _clamp_camera_fps(fps):
    steps = getattr(jks, "JKS_CAMERA_FPS_STEPS", (24, 25, 30, 48, 50, 60))
    try:
        fps = int(fps)
    except (TypeError, ValueError):
        fps = int(getattr(jks, "JKS_CAMERA_FPS", 30))
    if fps in steps:
        return fps
    # Nearest step.
    best = steps[0]
    best_d = abs(best - fps)
    for s in steps[1:]:
        d = abs(s - fps)
        if d < best_d:
            best = s
            best_d = d
    return int(best)


def _next_camera_fps(fps):
    steps = getattr(jks, "JKS_CAMERA_FPS_STEPS", (24, 25, 30, 48, 50, 60))
    fps = _clamp_camera_fps(fps)
    try:
        i = list(steps).index(fps)
    except ValueError:
        i = 0
    return int(steps[(i + 1) % len(steps)])


def _clamp_tl_mode(mode):
    m = str(mode or "").strip().lower()
    if m in ("msm", "continuous"):
        return m
    return "msm"


def _load_panel_state(soft_min, soft_max):
    """Load PosA/B/C and optional tl_div…camera_fps, tl_mode."""
    pos_a, pos_b, pos_c = _default_positions(soft_min, soft_max)
    tl_div = 1
    swap_lr = bool(jks.JKS_SWAP_LR)
    delay_s = 0.0
    joy_center = 0.5
    camera_fps = _clamp_camera_fps(getattr(jks, "JKS_CAMERA_FPS", 30))
    tl_mode = _clamp_tl_mode(getattr(jks, "JKS_TL_MODE", "msm"))
    delay_max = float(getattr(jks, "JKS_DELAY_MAX_S", 30.0))
    try:
        with open(jks.JKS_POSITIONS_FILE, "r") as f:
            parts = f.readline().strip().split(",")
            if len(parts) >= 2:
                pos_a = _clamp_store_position(float(parts[0]), soft_min, soft_max)
                pos_b = _clamp_store_position(float(parts[1]), soft_min, soft_max)
            if len(parts) >= 3:
                pos_c = _clamp_store_position(float(parts[2]), soft_min, soft_max)
            if len(parts) >= 4:
                tl_div = int(float(parts[3]))
                if tl_div < 1:
                    tl_div = 1
            if len(parts) >= 5:
                swap_lr = parts[4].strip() in ("1", "true", "True")
            if len(parts) >= 6:
                delay_s = float(parts[5])
                if delay_s < 0:
                    delay_s = 0.0
                elif delay_s > delay_max:
                    delay_s = delay_max
            if len(parts) >= 7:
                joy_center = _clamp_joy_center(float(parts[6]))
            if len(parts) >= 8:
                camera_fps = _clamp_camera_fps(parts[7])
            if len(parts) >= 9:
                tl_mode = _clamp_tl_mode(parts[8])
    except Exception:
        pass
    return (
        pos_a,
        pos_b,
        pos_c,
        tl_div,
        swap_lr,
        delay_s,
        joy_center,
        camera_fps,
        tl_mode,
    )


def _save_panel_state(
    pos_a,
    pos_b,
    pos_c,
    tl_div,
    swap_lr,
    delay_s,
    joy_center=0.5,
    camera_fps=30,
    tl_mode="msm",
):
    try:
        with open(jks.JKS_POSITIONS_FILE, "w") as f:
            f.write(
                "%s,%s,%s,%d,%d,%.3f,%.4f,%d,%s\n"
                % (
                    pos_a,
                    pos_b,
                    pos_c,
                    int(tl_div),
                    1 if swap_lr else 0,
                    delay_s,
                    _clamp_joy_center(joy_center),
                    _clamp_camera_fps(camera_fps),
                    _clamp_tl_mode(tl_mode),
                )
            )
    except Exception as exc:
        dbg(1, "save fail", exc)


def _dbg_btn_edges(named_buttons, btn_double_option=None):
    """L3: log press (+) / release (-) edges for all panel keys."""
    for name, btn in named_buttons:
        if btn.edge_press:
            dbg(3, name + "+")
        if btn.edge_release:
            dbg(3, name + "-")
    if btn_double_option is not None:
        if btn_double_option.edge_press:
            dbg(3, "DOUBLE_OPTION+")
        if btn_double_option.edge_release:
            dbg(3, "DOUBLE_OPTION-")


def _bind_lr(btn_move_l, btn_move_r, btn_fast_l, btn_fast_r, swap_lr):
    if swap_lr:
        return btn_move_r, btn_move_l, btn_fast_r, btn_fast_l
    return btn_move_l, btn_move_r, btn_fast_l, btn_fast_r


async def _wait_controls_clear(mc, ui, named_buttons, update_all):
    warned = False
    last_oled = None
    while True:
        update_all()
        stuck = []
        for name, btn in named_buttons:
            if btn.pressed():
                stuck.append(name)
        if mc.isDRVErrorActive():
            stuck.append("HW_EMO")
        if not stuck:
            return
        # OLED: "Release NAME" / second line lists others (21 chars).
        if len(stuck) == 1:
            oled = "Release " + stuck[0]
        else:
            oled = "Release " + stuck[0] + "\n" + ",".join(stuck[1:])[:21]
        if oled != last_oled:
            ui.setOledText(oled)
            last_oled = oled
        if not warned:
            dbg(2, "stuck", ",".join(stuck))
            warned = True
        await asyncio.sleep_ms(50)


async def _wait_boot_unlock(ui, btn_option, btn_stop, update_all):
    """Block until OPTION or STOP is pressed; rainbow LED + OLED hint while locked."""
    oled = "Unlock: OPTION\nor STOP"
    ui.setOledText(oled)
    dbg(3, "locked")
    ui.startLedRainbowLoop()
    try:
        while True:
            update_all()
            if btn_option.edge_press or btn_stop.edge_press:
                dbg(3, "unlocked")
                return
            ui.driveLed()
            await asyncio.sleep_ms(20)
    finally:
        ui.stopLedEffect()


def _loop_pair_label(loop_pair):
    if loop_pair == "AB":
        return "A-B"
    if loop_pair == "AC":
        return "A-C"
    if loop_pair == "BC":
        return "B-C"
    return loop_pair or ""


def _status_oled_text(
    mode,
    cruise_dir,
    loop_pair,
    goto_target,
    loop_target,
    cruise_boost=False,
    fast_dir=0,
    driver_off=False,
):
    if driver_off:
        return "Disabled"
    if mode == _HOMING:
        return "Homing..."
    if mode == _JOYSTICK:
        return "Joystick"
    if mode == _CRUISE:
        side = "L" if cruise_dir < 0 else "R"
        if cruise_boost:
            return "Cruise " + side + " fast"
        return "Cruising " + side
    if mode == _FAST:
        if fast_dir < 0:
            return "Fast L"
        if fast_dir > 0:
            return "Fast R"
        return "Fast jog"
    if mode == _GOTO:
        if goto_target:
            return "Move to " + goto_target
        return "Moving..."
    if mode == _LOOP:
        if loop_target:
            return "Loop to " + loop_target
        label = _loop_pair_label(loop_pair)
        return "Loop " + label if label else "Looping"
    return "Ready"


async def main():
    mc = MC_Client()
    ui = UIC_Base()
    mc.set_status_callback(ui.on_status)
    await mc.start()
    await ui.start()
    # Ceilings / soft limits from MC CG (MC_Client.fetchConfig); no boot CS.
    soft_min = mc.slider_min
    soft_max = mc.slider_max if mc.slider_max is not None else 0.0
    ui.set_soft_limits(mc.slider_min, mc.slider_max)
    ui.setOledUnit(getattr(jks, "JKS_DSP_UNIT", "mm"))

    def _enable(on):
        mc.enable(on)
        ui.set_enabled(bool(on) and not mc.isDRVErrorActive())

    adc_speed = ADC(Pin(jks.PIN_POT_SPEED))
    adc_accel = ADC(Pin(jks.PIN_POT_ACCEL))
    adc_joy = None
    if jks.PIN_POT_JOYSTICK is not None:
        adc_joy = ADC(Pin(jks.PIN_POT_JOYSTICK))
    filt_speed = _PotFilter(
        getattr(jks, "JKS_POT_OVERSAMPLE", 8),
        getattr(jks, "JKS_POT_EMA_ALPHA", 0.2),
        getattr(jks, "JKS_POT_HYST", 0.008),
    )
    filt_accel = _PotFilter(
        getattr(jks, "JKS_POT_OVERSAMPLE", 8),
        getattr(jks, "JKS_ACCEL_EMA_ALPHA", 0.15),
        getattr(jks, "JKS_ACCEL_HYST", 0.01),
    )
    filt_joy = None
    if adc_joy is not None:
        filt_joy = _PotFilter(
            getattr(jks, "JKS_JOY_OVERSAMPLE", 8),
            getattr(jks, "JKS_JOY_EMA_ALPHA", 0.3),
            getattr(jks, "JKS_JOY_HYST", 0.006),
        )
    debounce = jks.JKS_BTN_DEBOUNCE_MS
    long_ms = jks.JKS_LONG_PRESS_MS
    stop_halt_ms = getattr(jks, "JKS_STOP_HALT_MS", 1000)
    stop_disable_ms = getattr(jks, "JKS_STOP_DISABLE_MS", 2000)
    delay_clear_ms = getattr(jks, "JKS_DELAY_CLEAR_MS", 300)
    loop_dwell_ms = getattr(jks, "JKS_LOOP_DWELL_MS", 0)

    (
        btn_move_l,
        btn_move_r,
        btn_fast_l,
        btn_fast_r,
        btn_stop,
        btn_a,
        btn_b,
        btn_c,
        btn_option,
        btn_delay,
        btn_tl,
        btn_double_option,
        named_buttons,
        update_all,
    ) = _make_panel_inputs(debounce, long_ms, stop_halt_ms, stop_disable_ms)
    delay_scale = float(getattr(jks, "JKS_OPTION_DELAY_SCALE", 5.0))
    delay_preset_s = float(getattr(jks, "JKS_OPTION_DELAY_PRESET_S", 5.0))
    tl_option_max = int(getattr(jks, "JKS_OPTION_TL_MAX", 100))
    tl_favorite = int(getattr(jks, "JKS_OPTION_TL_FAVORITE", 25))
    delay_option_latched = False
    move_swap_since = None
    move_swap_fired = False
    move_pair_since = None
    move_pair_fired = False
    swap_option_latched = False
    display_dim = float(getattr(jks, "JKS_DISPLAY_DIM", 0.25))
    if display_dim < 0.01:
        display_dim = 0.01
    elif display_dim > 1.0:
        display_dim = 1.0

    (
        pos_a,
        pos_b,
        pos_c,
        tl_div,
        swap_lr,
        action_delay_s,
        joy_center,
        camera_fps,
        tl_mode,
    ) = _load_panel_state(soft_min, mc.slider_max)
    tl_index = _tl_index_for(tl_div)
    delay_max_s = float(getattr(jks, "JKS_DELAY_MAX_S", 30.0))
    oled_rotate_ms = int(getattr(jks, "JKS_DSP_EXTRA_ROTATE_MS", 1000))
    move_l, move_r, fast_l, fast_r = _bind_lr(
        btn_move_l, btn_move_r, btn_fast_l, btn_fast_r, swap_lr
    )

    mode = _IDLE
    cruise_dir = 0
    cruise_locked = False
    loop_pair = None
    loop_toward_p2 = True
    loop_dwell_until = None
    combo_prev = None
    combo_lock = False
    abc_since = None
    abc_fired = False
    abc_option_latched = False
    swap_since = None
    swap_fired = False
    app_oled_prev = None
    oled_flash = None
    oled_flash_until = 0
    goto_target = None
    loop_target = None
    goto_boost = False
    pending = None
    pending_due_ms = 0
    driver_on = False
    goto_t0_ms = None
    at_mark_mm = float(getattr(jks, "JKS_AT_MARK_MM", 0.5))
    if at_mark_mm < 0.0:
        at_mark_mm = 0.0
    move_paused = False
    pause_resume_guard = False
    msm_exposure_ms = int(getattr(jks, "JKS_MSM_EXPOSURE_MS", 200))
    if msm_exposure_ms < 0:
        msm_exposure_ms = 0
    msm_settle_ms = int(getattr(jks, "JKS_MSM_SETTLE_MS", 50))
    if msm_settle_ms < 0:
        msm_settle_ms = 0
    msm_min_step_mm = float(getattr(jks, "JKS_MSM_MIN_STEP_MM", 0.1))
    if msm_min_step_mm < 0.01:
        msm_min_step_mm = 0.01
    msm_active = False
    msm_phase = None
    msm_phase_until = 0
    msm_delta = 0.0
    msm_end_pos = None
    msm_dir = 0
    msm_frame_due = 0
    msm_kind = None  # "goto" | "cruise" | "loop"
    cont_move_ms = 0
    cont_tick_last = None
    tl_mode_chord_fired = False


    def _persist():
        _save_panel_state(
            pos_a,
            pos_b,
            pos_c,
            tl_div,
            swap_lr,
            action_delay_s,
            joy_center,
            camera_fps,
            tl_mode,
        )

    def _sync_camera_mode():
        # Cont + TL≠1: hold-high like video; speed still uses tl_div.
        cam_div = 1 if (tl_mode == "continuous" and tl_div != 1) else tl_div
        ui.setCameraMode(cam_div, camera_fps)

    def _tl_status_line():
        if tl_div == 1:
            return "TL x1 @%dfps" % camera_fps
        if tl_mode == "msm":
            return "MSM x%d @%dfps" % (tl_div, camera_fps)
        return "Cont x%d @%dfps" % (tl_div, camera_fps)

    def _tl_frame_line():
        if tl_mode == "continuous" and tl_div != 1:
            div = tl_div if tl_div > 0 else 1
            secs = int((cont_move_ms / 1000.0) / float(div))
            if secs < 0:
                secs = 0
            return "%02d:%02d" % (secs // 60, secs % 60)
        frames = ui.getCameraPulseCount()
        fps = camera_fps if camera_fps > 0 else 1
        secs = frames // fps
        return "%02d:%02d  %d" % (secs // 60, secs % 60, frames)

    def _reset_cont_timer():
        nonlocal cont_move_ms, cont_tick_last
        cont_move_ms = 0
        cont_tick_last = None

    def _tick_cont_timer():
        """Accumulate Cont recording time while moving or soft-paused."""
        nonlocal cont_move_ms, cont_tick_last
        if not (tl_mode == "continuous" and tl_div != 1):
            cont_tick_last = None
            return
        recording = (
            (mc.isMoving() or move_paused)
            and mode in (_CRUISE, _GOTO, _LOOP, _FAST, _JOYSTICK)
            and not msm_active
        )
        now = time.ticks_ms()
        if recording:
            if cont_tick_last is not None:
                dt = time.ticks_diff(now, cont_tick_last)
                if dt > 0:
                    cont_move_ms += dt
            cont_tick_last = now
        else:
            cont_tick_last = None

    def _mark_positions():
        return (("A", pos_a), ("B", pos_b), ("C", pos_c))

    def _at_mark_letter():
        """Return A/B/C if idle carriage is within tolerance of that Pos."""
        if mc.isMoving():
            return None
        pos = mc.getPosition()
        best = None
        best_d = None
        for letter, mm in _mark_positions():
            d = abs(pos - mm)
            if d <= at_mark_mm:
                if best_d is None or d < best_d:
                    best = letter
                    best_d = d
                elif d == best_d and letter < best:
                    best = letter
        return best

    def _goto_mark_letter():
        if mode != _GOTO or not goto_target:
            return None
        if goto_target == "PosA":
            return "A"
        if goto_target == "PosB":
            return "B"
        if goto_target == "PosC":
            return "C"
        return None

    def _badge_mark_token():
        dest = _goto_mark_letter()
        if dest:
            return "->" + dest
        at = _at_mark_letter()
        return at

    def _eff_speed_accel(speed_mm_s, accel_mm_s2):
        div = tl_div if tl_div > 0 else 1
        return (
            max(speed_mm_s / div, config.MIN_SPEED_MM_S),
            max(accel_mm_s2 / div, config.MIN_SPEED_MM_S),
        )

    def _estimate_eta(to_mm, speed_mm_s, accel_mm_s2):
        v, a = _eff_speed_accel(speed_mm_s, accel_mm_s2)
        return mc.estimateMoveTimeTo(to_mm, v, a)

    def _fmt_eta_s(secs):
        if secs < 0.0:
            secs = 0.0
        return "%.1f s" % secs

    def _idle_mark_eta_text(speed_mm_s, accel_mm_s2):
        """Two lines: ETA to the other two marks from current Pos."""
        at = _at_mark_letter()
        if not at:
            return None
        lines = []
        for letter, mm in _mark_positions():
            if letter == at:
                continue
            t = _estimate_eta(mm, speed_mm_s, accel_mm_s2)
            lines.append("->%s %s" % (letter, _fmt_eta_s(t)))
        if len(lines) != 2:
            return None
        return lines[0] + "\n" + lines[1]

    def _goto_time_text(speed_mm_s, accel_mm_s2):
        """Elapsed + remaining ETA while goto to PosA/B/C."""
        dest = _goto_mark_letter()
        if not dest or goto_t0_ms is None:
            return None
        if dest == "A":
            tgt = pos_a
        elif dest == "B":
            tgt = pos_b
        else:
            tgt = pos_c
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, goto_t0_ms) * 0.001
        if elapsed < 0.0:
            elapsed = 0.0
        remain = _estimate_eta(tgt, speed_mm_s, accel_mm_s2)
        return "%s\n->%s %s" % (
            _fmt_eta_s(elapsed),
            dest,
            _fmt_eta_s(remain),
        )

    def _flash_oled(msg):
        nonlocal oled_flash, oled_flash_until
        oled_flash = msg
        oled_flash_until = time.ticks_add(time.ticks_ms(), _OLED_FLASH_MS)

    def _peek_marks():
        _flash_oled(
            "A:%.0f B:%.0f C:%.0f" % (pos_a, pos_b, pos_c)
        )
        dbg(3, "marks", round(pos_a, 2), round(pos_b, 2), round(pos_c, 2))

    def _speed_ok(speed_mm_s):
        return speed_mm_s >= config.MIN_SPEED_MM_S

    def _apply_motion_params(speed_mm_s, accel_mm_s2):
        div = tl_div if tl_div > 0 else 1
        acc = max(accel_mm_s2 / div, config.MIN_SPEED_MM_S)
        mc.setAcceleration(acc)
        spd = None
        if speed_mm_s >= config.MIN_SPEED_MM_S:
            spd = max(speed_mm_s / div, config.MIN_SPEED_MM_S)
            mc.setSpeed(spd)
        ui.set_commanded(speed_mm_s=spd, accel_mm_s2=acc)
        return speed_mm_s / div, accel_mm_s2 / div

    def _cancel_pending():
        nonlocal pending, pending_due_ms
        pending = None
        pending_due_ms = 0

    def _want_msm():
        return tl_div != 1 and tl_mode == "msm"

    def _msm_interval_ms():
        fps = camera_fps if camera_fps > 0 else 1
        div = tl_div if tl_div > 0 else 1
        period = int(1000.0 * float(div) / float(fps))
        pulse_ms = int(getattr(config, "CTRL_CAMERA_PULSE_MS", 100))
        if pulse_ms < 1:
            pulse_ms = 1
        if period < pulse_ms + 10:
            period = pulse_ms + 10
        return period

    def _msm_move_budget_s():
        """Seconds available for the hop inside one frame interval."""
        T = _msm_interval_ms() * 0.001
        pulse_s = int(getattr(config, "CTRL_CAMERA_PULSE_MS", 100)) * 0.001
        return T - (msm_exposure_ms * 0.001) - (msm_settle_ms * 0.001) - pulse_s

    def _msm_max_delta(v_mm_s, a_mm_s2):
        """Largest hop (mm) that fits in the interval move budget, or None."""
        budget = _msm_move_budget_s()
        if budget <= 0.0:
            return None
        v = max(float(v_mm_s), config.MIN_SPEED_MM_S)
        a = max(float(a_mm_s2), config.MIN_SPEED_MM_S)
        lo = msm_min_step_mm
        if mc.estimateMoveTime(lo, v, a) > budget:
            return None
        hi = soft_max if soft_max and soft_max > 0 else 1000.0
        if hi < lo * 2:
            hi = 1000.0
        if mc.estimateMoveTime(hi, v, a) <= budget:
            return hi
        for _ in range(24):
            mid = (lo + hi) * 0.5
            if mc.estimateMoveTime(mid, v, a) <= budget:
                lo = mid
            else:
                hi = mid
        return lo

    def _msm_clear():
        nonlocal msm_active, msm_phase, msm_phase_until, msm_delta
        nonlocal msm_end_pos, msm_dir, msm_frame_due, msm_kind
        msm_active = False
        msm_phase = None
        msm_phase_until = 0
        msm_delta = 0.0
        msm_end_pos = None
        msm_dir = 0
        msm_frame_due = 0
        msm_kind = None
        ui.setCameraManual(False)

    def _apply_full_motion_params(speed_mm_s, accel_mm_s2):
        """Undivided SPEED/ACCEL for MSM hops and live jog."""
        mc.setAcceleration(max(accel_mm_s2, config.MIN_SPEED_MM_S))
        if speed_mm_s >= config.MIN_SPEED_MM_S:
            mc.setSpeed(max(speed_mm_s, config.MIN_SPEED_MM_S))
        return speed_mm_s, accel_mm_s2

    def _msm_begin(action, speed_mm_s, accel_mm_s2):
        """Start MSM take. Returns False if refused (OLED already flashed)."""
        nonlocal mode, cruise_dir, cruise_locked, goto_target, loop_pair, loop_target
        nonlocal loop_toward_p2, loop_dwell_until, driver_on, goto_t0_ms
        nonlocal msm_active, msm_phase, msm_phase_until, msm_delta
        nonlocal msm_end_pos, msm_dir, msm_frame_due, msm_kind
        if not _speed_ok(speed_mm_s):
            _flash_oled("Set SPEED")
            return False
        delta = _msm_max_delta(speed_mm_s, accel_mm_s2)
        if delta is None:
            _flash_oled("TL too fast")
            dbg(2, "MSM refuse budget", round(_msm_move_budget_s(), 3))
            return False
        kind = action[0]
        _enable(True)
        driver_on = True
        _apply_full_motion_params(speed_mm_s, accel_mm_s2)
        ui.setCameraManual(True)
        ui.resetCameraPulseCount()
        msm_delta = delta
        msm_active = True
        msm_phase = "shoot"
        msm_phase_until = 0
        msm_frame_due = time.ticks_ms()
        msm_kind = kind
        loop_dwell_until = None
        if kind == "cruise":
            direction = action[1]
            cruise_dir = direction
            cruise_locked = not (move_l.pressed() or move_r.pressed())
            mode = _CRUISE
            goto_target = None
            goto_t0_ms = None
            loop_pair = None
            loop_target = None
            msm_dir = 1 if direction > 0 else -1
            msm_end_pos = None
            dbg(5, "MSM cruise", "R" if direction > 0 else "L", round(delta, 3))
        elif kind == "goto":
            name, pos = action[1], action[2]
            mode = _GOTO
            goto_target = name
            cruise_dir = 0
            cruise_locked = False
            loop_pair = None
            loop_target = None
            msm_end_pos = float(pos)
            msm_dir = 0
            if name in ("PosA", "PosB", "PosC"):
                goto_t0_ms = time.ticks_ms()
            else:
                goto_t0_ms = None
            dbg(5, "MSM goto", name, round(pos, 2), round(delta, 3))
        elif kind == "loop":
            pair, p2, n2 = action[1], action[2], action[3]
            mode = _LOOP
            loop_pair = pair
            loop_toward_p2 = True
            loop_target = n2
            goto_target = None
            goto_t0_ms = None
            cruise_dir = 0
            cruise_locked = False
            msm_end_pos = float(p2)
            msm_dir = 0
            dbg(5, "MSM loop", pair, round(delta, 3))
        else:
            _msm_clear()
            return False
        return True

    def _msm_finish(msg=None):
        nonlocal mode, cruise_dir, cruise_locked, goto_target, loop_target, loop_pair
        nonlocal loop_dwell_until, goto_t0_ms
        arrived = goto_target
        _msm_clear()
        mode = _IDLE
        cruise_dir = 0
        cruise_locked = False
        goto_target = None
        goto_t0_ms = None
        loop_target = None
        loop_pair = None
        loop_dwell_until = None
        if msg:
            _flash_oled(msg)
            if msg == "Hard limit":
                dbg(1, "Limit hard")
            elif msg == "Soft limit":
                dbg(2, "Limit soft")
            elif msg == "TL too fast":
                dbg(2, "TL too fast")
            else:
                dbg(3, msg)
        elif arrived:
            _flash_oled("At " + arrived)
        dbg(3, "MSM done", round(mc.getPosition(), 2))

    def _msm_next_end_for_loop():
        """Flip loop end target; returns new absolute mm."""
        nonlocal loop_toward_p2, loop_target, msm_end_pos
        if loop_pair == "AB":
            p1, p2 = pos_a, pos_b
            n1, n2 = "PosA", "PosB"
        elif loop_pair == "AC":
            p1, p2 = pos_a, pos_c
            n1, n2 = "PosA", "PosC"
        else:
            p1, p2 = pos_b, pos_c
            n1, n2 = "PosB", "PosC"
        if loop_toward_p2:
            loop_toward_p2 = False
            loop_target = n1
            msm_end_pos = p1
            dbg(5, "MSM loop ->", n1)
        else:
            loop_toward_p2 = True
            loop_target = n2
            msm_end_pos = p2
            dbg(5, "MSM loop ->", n2)
        return msm_end_pos

    def _msm_step_target():
        """Absolute position for the next hop."""
        pos = mc.getPosition()
        if msm_kind == "cruise":
            return pos + msm_dir * msm_delta
        # goto / loop toward msm_end_pos
        remain = msm_end_pos - pos
        if abs(remain) <= msm_min_step_mm * 0.5:
            return msm_end_pos
        if abs(remain) <= msm_delta:
            return msm_end_pos
        if remain > 0:
            return pos + msm_delta
        return pos - msm_delta

    def _msm_tick(speed_mm_s, accel_mm_s2):
        """Advance MSM state machine one main-loop iteration."""
        nonlocal msm_phase, msm_phase_until, msm_frame_due, msm_delta
        if not msm_active or move_paused:
            return
        if not _want_msm():
            _msm_finish("TL off")
            return
        now = time.ticks_ms()
        if msm_phase == "shoot":
            if mc.isMoving():
                return
            # Re-plan delta if pots changed (keep hops fitting budget).
            d = _msm_max_delta(speed_mm_s, accel_mm_s2)
            if d is None:
                _msm_finish("TL too fast")
                return
            msm_delta = d
            _apply_full_motion_params(speed_mm_s, accel_mm_s2)
            ui.pulseCamera()
            msm_phase = "expose"
            msm_phase_until = time.ticks_add(now, msm_exposure_ms)
            # Cadence deadline for this frame's cycle (shoot → next shoot).
            msm_frame_due = time.ticks_add(now, _msm_interval_ms())
            return
        if msm_phase == "expose":
            if time.ticks_diff(now, msm_phase_until) < 0:
                return
            # Arrived before stepping?
            if msm_kind != "cruise" and msm_end_pos is not None:
                if abs(mc.getPosition() - msm_end_pos) <= msm_min_step_mm * 0.5:
                    if msm_kind == "loop":
                        _msm_next_end_for_loop()
                        msm_phase = "pace"
                        msm_phase_until = msm_frame_due
                    else:
                        _msm_finish()
                    return
            tgt = _msm_step_target()
            _apply_full_motion_params(speed_mm_s, accel_mm_s2)
            mc.moveTo(tgt)
            msm_phase = "wait_move"
            return
        if msm_phase == "wait_move":
            if mc.isMoving():
                return
            if mc.isAtHardLimit():
                _msm_finish("Hard limit")
                return
            if msm_kind == "cruise" and mc.isAtSoftLimit():
                _msm_finish("Soft limit")
                return
            msm_phase = "settle"
            msm_phase_until = time.ticks_add(now, msm_settle_ms)
            return
        if msm_phase == "settle":
            if time.ticks_diff(now, msm_phase_until) < 0:
                return
            if msm_kind != "cruise" and msm_end_pos is not None:
                if abs(mc.getPosition() - msm_end_pos) <= msm_min_step_mm * 0.5:
                    if msm_kind == "loop":
                        _msm_next_end_for_loop()
                    else:
                        # One last shutter at the mark, then done.
                        msm_phase = "final_shoot"
                        return
            msm_phase = "pace"
            msm_phase_until = msm_frame_due
            return
        if msm_phase == "final_shoot":
            if mc.isMoving():
                return
            ui.pulseCamera()
            msm_phase = "final_expose"
            msm_phase_until = time.ticks_add(now, msm_exposure_ms)
            return
        if msm_phase == "final_expose":
            if time.ticks_diff(now, msm_phase_until) < 0:
                return
            _msm_finish()
            return
        if msm_phase == "pace":
            if time.ticks_diff(now, msm_phase_until) < 0:
                return
            # Overran interval: stretched already by waiting on move.
            if time.ticks_diff(now, msm_frame_due) > 20:
                _flash_oled("Step slow")
                dbg(2, "MSM overdue", time.ticks_diff(now, msm_frame_due))
            msm_phase = "shoot"
            return

    def _clear_move_pause():
        nonlocal move_paused, pause_resume_guard
        move_paused = False
        pause_resume_guard = False
        ui.setCameraMotionActive(False)

    def _resume_after_pause(speed_mm_s, accel_mm_s2):
        """Re-issue the paused move; do not reset goto_t0_ms."""
        nonlocal move_paused, pause_resume_guard, driver_on
        move_paused = False
        pause_resume_guard = False
        if msm_active:
            # Freeze/resume MSM phase only — no continuous retarget.
            ui.setCameraMotionActive(False)
            dbg(3, "Resume")
            return
        eff_speed, _ = _apply_motion_params(speed_mm_s, accel_mm_s2)
        _enable(True)
        driver_on = True
        if mode == _GOTO and goto_target:
            if goto_target == "PosA":
                tgt = pos_a
            elif goto_target == "PosB":
                tgt = pos_b
            elif goto_target == "PosC":
                tgt = pos_c
            elif goto_target == "min":
                tgt = soft_min
            elif goto_target == "mid":
                tgt = (
                    0.5 * (float(soft_min) + float(soft_max))
                    if soft_min is not None
                    else None
                )
            elif goto_target == "max":
                tgt = soft_max
            else:
                tgt = None
            if tgt is not None:
                mc.setSpeed(max(eff_speed, config.MIN_SPEED_MM_S))
                mc.moveTo(tgt)
        elif mode == _CRUISE and cruise_dir != 0:
            mc.move(_signed(cruise_dir > 0, eff_speed))
            if not cruise_locked:
                pause_resume_guard = True
        elif mode == _LOOP and loop_target:
            if loop_target == "PosA":
                tgt = pos_a
            elif loop_target == "PosB":
                tgt = pos_b
            elif loop_target == "PosC":
                tgt = pos_c
            else:
                tgt = None
            if tgt is not None:
                mc.setSpeed(max(eff_speed, config.MIN_SPEED_MM_S))
                mc.moveTo(tgt)
        elif mode == _FAST:
            max_spd = (
                mc.max_speed
                if mc.max_speed is not None
                else jks.JKS_SPEED_MAX_MM_S
            )
            max_acc = (
                mc.max_accel
                if mc.max_accel is not None
                else jks.JKS_ACCEL_MAX_MM_S2
            )
            _apply_full_motion_params(max_spd, max_acc)
            jog = max_spd
            if fast_l.pressed() and not fast_r.pressed():
                mc.move(_signed(False, jog))
            elif fast_r.pressed() and not fast_l.pressed():
                mc.move(_signed(True, jog))
        # _JOYSTICK: next loop restores from stick
        ui.setCameraMotionActive(False)
        dbg(3, "Resume")

    def _run_action(action, speed_mm_s, accel_mm_s2):
        nonlocal mode, cruise_dir, cruise_locked, goto_target, loop_pair, loop_target
        nonlocal loop_toward_p2, loop_dwell_until, driver_on, goto_t0_ms, goto_boost
        _clear_move_pause()
        _msm_clear()
        goto_boost = False
        kind = action[0]
        if _want_msm() and kind in ("cruise", "goto", "loop"):
            _msm_begin(action, speed_mm_s, accel_mm_s2)
            return
        _reset_cont_timer()
        eff_speed, _ = _apply_motion_params(speed_mm_s, accel_mm_s2)
        if kind == "cruise":
            direction = action[1]
            _enable(True)
            driver_on = True
            cruise_dir = direction
            # Already released → locked; still held → hold until release decides.
            cruise_locked = not (move_l.pressed() or move_r.pressed())
            mode = _CRUISE
            goto_target = None
            goto_t0_ms = None
            mc.move(_signed(direction > 0, eff_speed))
            dbg(3, "Cruise", "R" if direction > 0 else "L")
        elif kind == "goto":
            name, pos = action[1], action[2]
            _enable(True)
            driver_on = True
            mc.setSpeed(max(eff_speed, config.MIN_SPEED_MM_S))
            mc.moveTo(pos)
            mode = _GOTO
            goto_target = name
            cruise_dir = 0
            cruise_locked = False
            if name in ("PosA", "PosB", "PosC"):
                goto_t0_ms = time.ticks_ms()
            else:
                goto_t0_ms = None
            dbg(3, "Goto", name, round(pos, 2))
        elif kind == "loop":
            pair, p2, n2 = action[1], action[2], action[3]
            _enable(True)
            driver_on = True
            mc.setSpeed(max(eff_speed, config.MIN_SPEED_MM_S))
            loop_pair = pair
            loop_toward_p2 = True
            loop_target = n2
            loop_dwell_until = None
            goto_t0_ms = None
            mc.moveTo(p2)
            mode = _LOOP
            cruise_dir = 0
            cruise_locked = False
            dbg(3, "Loop", pair)

    def _request_action(action, speed_mm_s, accel_mm_s2):
        nonlocal pending, pending_due_ms
        if action[0] in ("cruise", "goto", "loop") and not _speed_ok(speed_mm_s):
            _flash_oled("Set SPEED")
            return
        if action_delay_s <= 0:
            _run_action(action, speed_mm_s, accel_mm_s2)
            return
        pending = (action, speed_mm_s, accel_mm_s2)
        pending_due_ms = time.ticks_add(
            time.ticks_ms(), int(action_delay_s * 1000.0)
        )
        dbg(3, "Wait delay", action_delay_s, action[0])

    def _oled_dist_text(mm):
        """Format a distance for OLED app lines; API/storage remain mm."""
        if ui.getOledUnit() == "inch":
            return "%.2fin" % (float(mm) / 25.4)
        return "%.0fmm" % float(mm)

    def _goto_remain_text():
        if mode == _GOTO and goto_target:
            if goto_target == "PosA":
                tgt, tag = pos_a, "A"
            elif goto_target == "PosB":
                tgt, tag = pos_b, "B"
            elif goto_target == "PosC":
                tgt, tag = pos_c, "C"
            elif goto_target == "min":
                tgt = soft_min
                if tgt is None:
                    return None
                tag = "min"
            elif goto_target == "mid":
                if soft_min is None:
                    return None
                tgt = 0.5 * (float(soft_min) + float(soft_max))
                tag = "mid"
            elif goto_target == "max":
                tgt = soft_max
                if tgt is None or tgt == 0.0:
                    return None
                tag = "max"
            else:
                return None
            remain = abs(tgt - mc.getPosition())
            return "->%s %s" % (tag, _oled_dist_text(remain))
        if mode == _LOOP and loop_target:
            if loop_target == "PosA":
                tgt, tag = pos_a, "A"
            elif loop_target == "PosB":
                tgt, tag = pos_b, "B"
            elif loop_target == "PosC":
                tgt, tag = pos_c, "C"
            else:
                return None
            remain = abs(tgt - mc.getPosition())
            return "->%s %s" % (tag, _oled_dist_text(remain))
        return None

    def _push_oled(
        cruise_boost=False,
        fast_dir=0,
        wait_s=None,
        dwell_s=None,
        delay_preview_s=None,
        combo_held=False,
        speed_mm_s=None,
        accel_mm_s2=None,
    ):
        nonlocal app_oled_prev, oled_flash
        now = time.ticks_ms()
        if speed_mm_s is None:
            speed_mm_s = config.MIN_SPEED_MM_S
        if accel_mm_s2 is None:
            accel_mm_s2 = (
                mc.max_accel
                if mc.max_accel is not None
                else jks.JKS_ACCEL_MIN_MM_S2
            )

        if oled_flash is not None and time.ticks_diff(oled_flash_until, now) > 0:
            line1 = oled_flash
        elif combo_held:
            line1 = "Release A/B"
        elif move_paused:
            oled_flash = None
            line1 = "Paused"
        elif msm_active:
            oled_flash = None
            if mode == _GOTO and goto_target:
                line1 = "MSM " + goto_target
            elif mode == _LOOP:
                line1 = "MSM loop"
            elif mode == _CRUISE:
                side = "L" if cruise_dir < 0 else "R"
                line1 = "MSM cruise " + side
            else:
                line1 = "MSM"
        else:
            oled_flash = None
            line1 = _status_oled_text(
                mode,
                cruise_dir,
                loop_pair,
                goto_target,
                loop_target,
                cruise_boost=cruise_boost,
                fast_dir=fast_dir,
                driver_off=not driver_on,
            )

        # Priority: Wait > Near > Dwell > Delay > remain > TL
        ranked = []
        if wait_s is not None and wait_s > 0:
            ranked.append((0, "Wait %.1fs" % wait_s))
        if mc.isNearSoftLimit() and not mc.isAtSoftLimit():
            ranked.append((1, "Near limit"))
        if dwell_s is not None and dwell_s > 0:
            ranked.append((2, "Dwell %.1fs" % dwell_s))
        if delay_preview_s is not None and delay_preview_s > 0:
            ranked.append((3, "Delay %.1fs" % delay_preview_s))
        elif action_delay_s > 0 and pending is None:
            ranked.append((3, "Delay %.1fs" % action_delay_s))
        # Skip mm-remain for PosA/B/C goto — time lines replace it.
        if _goto_mark_letter() is None:
            remain_txt = _goto_remain_text()
            if remain_txt:
                ranked.append((4, remain_txt))
        if tl_div != 1:
            ranked.append((5, _tl_status_line()))
        ranked.sort(key=lambda item: item[0])
        extras = [t for _, t in ranked]
        high_extras = [t for p, t in ranked if p < 5]
        flashing = (
            oled_flash is not None and time.ticks_diff(oled_flash_until, now) > 0
        )

        goto_time = None
        idle_mark = None
        if not flashing and not combo_held and not high_extras:
            goto_time = _goto_time_text(speed_mm_s, accel_mm_s2)
            if goto_time is None and not mc.isMoving() and not move_paused:
                idle_mark = _idle_mark_eta_text(speed_mm_s, accel_mm_s2)

        if move_paused and not flashing and not combo_held:
            if goto_time is not None:
                # Keep wall-clock elapsed on line 2 while paused.
                text = "Paused\n" + goto_time.split("\n", 1)[0]
            else:
                text = line1 if not extras else (
                    line1 + "\n" + extras[0]
                )
        elif msm_active and not high_extras and not flashing and not combo_held:
            text = line1 + "\n" + _tl_frame_line()
        elif goto_time is not None:
            text = goto_time
        elif idle_mark is not None:
            text = idle_mark
        elif tl_div != 1 and not high_extras and not flashing and not combo_held:
            text = _tl_status_line() + "\n" + _tl_frame_line()
        elif extras:
            packed = extras[0]
            fits = True
            for part in extras[1:]:
                if len(packed) + 1 + len(part) <= 21:
                    packed = packed + " " + part
                else:
                    fits = False
                    break
            if fits:
                line2 = packed
            else:
                idx = (now // max(oled_rotate_ms, 1)) % len(extras)
                line2 = extras[idx]
            text = line1 + "\n" + line2
        else:
            text = line1

        if text != app_oled_prev:
            ui.setOledText(text)
            app_oled_prev = text

    dim_w = _duty_u8(getattr(uic_cfg, "LED_DIM_WHITE", 0.12))
    dim_c = _duty_u8(getattr(uic_cfg, "LED_DIM_CYAN", 0.12))
    dim_m = _duty_u8(getattr(uic_cfg, "LED_DIM_MAGENTA", 0.12))
    _dim_white = (dim_w, dim_w, dim_w)
    _dim_cyan = (0, dim_c, dim_c)
    _dim_magenta = (dim_m, 0, dim_m)
    _dim_blue = (0, 0, max(dim_w * 2, 80))
    loop_blue = int(getattr(jks, "JKS_LOOP_BLUE_ADD", 26))
    ping_ms = int(getattr(jks, "JKS_LED_PINGPONG_MS", 600))
    flash_on = int(getattr(jks, "JKS_LED_FLASH_ON_MS", 80))
    flash_off = int(getattr(jks, "JKS_LED_FLASH_OFF_MS", 80))
    delay_wait_half = int(getattr(uic_cfg, "LED_BLINK_DELAY_WAIT_MS", 500))
    delay_wait_period = max(2 * delay_wait_half, 2)
    panel_led_kind = None  # delay_wait|delay|tl|loop_idle|loop_move|halt_flash|None

    def _halt_led_flash():
        nonlocal panel_led_kind
        ui.ledClearAdd()
        ui.ledFlash(_RED, 2, flash_on, flash_off)
        panel_led_kind = "halt_flash"

    def _sync_panel_led(move_btn_down=False):
        """Delay / TL / loop panel colours via UIC_Base effects + OLED badges."""
        nonlocal panel_led_kind
        delay_on = action_delay_s > 0 or pending is not None
        moving = mc.isMoving()
        # Priority: delay_wait > delay > tl > loop_idle; loop_move while moving.
        if not moving:
            if pending is not None or (action_delay_s > 0 and move_btn_down):
                desired = "delay_wait"
            elif action_delay_s > 0:
                desired = "delay"
            elif tl_div != 1:
                desired = "tl"
            elif mode == _LOOP:
                desired = "loop_idle"
            else:
                desired = None
        elif mode == _LOOP:
            desired = "loop_move"
        else:
            desired = None

        if desired != panel_led_kind:
            if desired == "delay_wait":
                ui.ledClearAdd()
                ui.ledPingPong(_dim_cyan, _OFF, delay_wait_period)
            elif desired == "delay":
                ui.ledClearAdd()
                ui.ledPingPong(_dim_cyan, _dim_cyan, ping_ms)
            elif desired == "tl":
                ui.ledClearAdd()
                ui.ledPingPong(_dim_magenta, _dim_magenta, ping_ms)
            elif desired == "loop_idle":
                ui.ledClearAdd()
                ui.ledPingPong(_dim_white, _dim_blue, ping_ms)
            elif desired == "loop_move":
                ui.ledEffectClear()
                ui.ledAddColor(0, 0, loop_blue)
            else:
                # None — leave halt_flash to finish; otherwise clear panel fx/add.
                if panel_led_kind == "halt_flash":
                    ui.ledClearAdd()
                else:
                    ui.ledClearAdd()
                    ui.ledEffectClear()
            panel_led_kind = desired

        ui.setOledBadges(
            tl=(tl_div != 1),
            delay=delay_on,
            mark=_badge_mark_token(),
        )

    _sync_camera_mode()
    _sync_panel_led()

    # Boot splash, then refuse to start while a control is stuck.
    boot_text = getattr(jks, "JKS_BOOT_TEXT", "JKSlider V1 by JK")
    boot_ms = getattr(jks, "JKS_BOOT_SPLASH_MS", 2000)
    rainbow_ms = int(getattr(uic_cfg, "LED_RAINBOW_MS", 1000))
    ui.setOledText(boot_text)
    app_oled_prev = boot_text
    dbg(3, boot_text)
    # Rainbow for 1 s at power-on; finish remaining splash time after.
    await ui.playLedRainbow(rainbow_ms)
    remain_ms = boot_ms - rainbow_ms
    if remain_ms > 0:
        await asyncio.sleep_ms(remain_ms)

    if getattr(jks, "JKS_BOOT_UNLOCK", True):
        await _wait_boot_unlock(ui, btn_option, btn_stop, update_all)

    await _wait_controls_clear(mc, ui, named_buttons, update_all)

    _enable(True)
    driver_on = True
    if getattr(jks, "JKS_HOMING_ENABLED", True):
        dbg(3, "Homing")
        mode = _HOMING
        _push_oled()
        mc.home()
        while mc.isMoving():
            await asyncio.sleep_ms(20)
        if mc.isDRVErrorActive():
            mode = _IDLE
            _flash_oled("Homing abort")
            dbg(2, "Homing abort")
        else:
            mode = _IDLE
            _flash_oled("Homed")
            dbg(3, "Ready", round(mc.getPosition(), 2))
    else:
        mode = _IDLE
        dbg(3, "Ready (no homing)", round(mc.getPosition(), 2))
    _push_oled()
    dbg(4, "marks", round(pos_a, 2), round(pos_b, 2), round(pos_c, 2))
    dbg(
        4,
        "cfg",
        "tap_ms",
        getattr(jks, "JKS_MOVE_TAP_MS", 333),
        "swap",
        swap_lr,
        "TL",
        tl_div,
        camera_fps,
        tl_mode,
        "delay",
        action_delay_s,
        "joy",
        "GP{}".format(jks.PIN_POT_JOYSTICK) if adc_joy is not None else "off",
    )

    # Clamp MC ceilings with panel config; all panel max uses these attributes.
    jks_speed_max = float(jks.JKS_SPEED_MAX_MM_S)
    jks_accel_max = float(jks.JKS_ACCEL_MAX_MM_S2)
    if mc.max_speed is not None:
        if mc.max_speed > jks_speed_max:
            mc.max_speed = jks_speed_max
    else:
        mc.max_speed = jks_speed_max
    if mc.max_accel is not None:
        if mc.max_accel > jks_accel_max:
            mc.max_accel = jks_accel_max
    else:
        mc.max_accel = jks_accel_max
    mc._max_speed_mm_s = mc.max_speed

    pot_max = float(mc.max_speed)
    pot_min = float(getattr(jks, "JKS_SPEED_MIN_MM_S", 1.0))
    if pot_min > pot_max:
        pot_min = pot_max
    accel_lo = float(jks.JKS_ACCEL_MIN_MM_S2)
    accel_hi = float(mc.max_accel)
    if accel_hi < accel_lo:
        accel_hi = accel_lo
    move_tap_ms = int(getattr(jks, "JKS_MOVE_TAP_MS", 333))
    # soft_min / soft_max already set from CG after start()

    try:
        while True:
            update_all()
            _dbg_btn_edges(named_buttons, btn_double_option)

            speed = _read_speed_mm_s(filt_speed, adc_speed, pot_min, pot_max)
            accel = _read_accel_mm_s2(filt_accel, adc_accel, accel_lo, accel_hi)
            if msm_active:
                _apply_full_motion_params(speed, accel)
                eff_speed = max(speed, config.MIN_SPEED_MM_S)
            else:
                eff_speed, _ = _apply_motion_params(speed, accel)
            wait_s = None
            dwell_s = None
            move_btn_down = (
                move_l.pressed()
                or move_r.pressed()
                or fast_l.pressed()
                or fast_r.pressed()
                or btn_a.pressed()
                or btn_b.pressed()
                or btn_c.pressed()
            )
            _sync_panel_led(move_btn_down)

            option = btn_option.pressed()
            move_sem = resolve_move_semantics(
                move_l, move_r, option, move_tap_ms
            )

            # --- T + D + OPTION: toggle MSM ↔ Cont ----------------------
            mode_chord = (
                btn_tl.pressed() and btn_delay.pressed() and option
            )
            if mode_chord:
                if not tl_mode_chord_fired and (
                    btn_tl.edge_press
                    or btn_delay.edge_press
                    or btn_option.edge_press
                ):
                    tl_mode_chord_fired = True
                    tl_mode = (
                        "continuous" if tl_mode == "msm" else "msm"
                    )
                    _msm_clear()
                    _sync_camera_mode()
                    _persist()
                    if tl_div != 1:
                        _flash_oled(_tl_status_line())
                    else:
                        _flash_oled(
                            "MSM" if tl_mode == "msm" else "Cont"
                        )
                    dbg(3, "TL mode", tl_mode)
            else:
                tl_mode_chord_fired = False

            # --- DELAY: mid-move pause/resume, or idle walk-in arm ---------
            # OPTION + DELAY: held time is scaled (default ×5).
            delay_preview_s = None
            _pausable = mode in (_CRUISE, _GOTO, _LOOP, _FAST, _JOYSTICK) and (
                mc.isMoving() or move_paused or msm_active
            )
            if _pausable:
                if btn_delay.pressed() and not move_paused:
                    move_paused = True
                    if msm_active:
                        if mc.isMoving():
                            mc.stop()
                    else:
                        ui.setCameraMotionActive(True)
                        mc.stop()
                    dbg(3, "Paused")
                elif move_paused and btn_delay.edge_release:
                    _resume_after_pause(speed, accel)
            elif not mode_chord:
                if btn_delay.edge_press:
                    delay_option_latched = option
                if btn_delay.pressed() and option:
                    delay_option_latched = True
                dscale = delay_scale if (
                    btn_delay.pressed() and delay_option_latched
                ) else 1.0
                if btn_delay.pressed() and btn_delay.hold_ms() >= delay_clear_ms:
                    held_s = (btn_delay.hold_ms() / 1000.0) * dscale
                    if held_s > delay_max_s:
                        held_s = delay_max_s
                    delay_preview_s = held_s
                if btn_delay.edge_release:
                    held = btn_delay.last_hold_ms
                    scale = delay_scale if delay_option_latched else 1.0
                    opt_delay = delay_option_latched
                    delay_option_latched = False
                    if held < delay_clear_ms:
                        if opt_delay:
                            # OPTION + DELAY short: arm fixed preset delay.
                            action_delay_s = delay_preset_s
                            if action_delay_s > delay_max_s:
                                action_delay_s = delay_max_s
                            _persist()
                            _flash_oled("Delay %.1fs" % action_delay_s)
                            dbg(3, "Delay", action_delay_s)
                        else:
                            action_delay_s = 0.0
                            _cancel_pending()
                            _persist()
                            _flash_oled("Delay off")
                            dbg(3, "Delay off")
                    else:
                        action_delay_s = (held / 1000.0) * scale
                        if action_delay_s > delay_max_s:
                            action_delay_s = delay_max_s
                        _persist()
                        if scale != 1.0:
                            _flash_oled(
                                "Delay %.1fs x%d" % (action_delay_s, int(scale))
                            )
                        else:
                            _flash_oled("Delay %.1fs" % action_delay_s)
                        dbg(3, "Delay", action_delay_s, "x", scale)

            # --- TIMELAPSE divider ----------------------------------------
            if not mode_chord:
                if btn_tl.long_press:
                    if btn_option.pressed():
                        # OPTION + TL hold: jump to favourite divider.
                        tl_div = tl_favorite
                        if tl_div < 1:
                            tl_div = 1
                        tl_index = _tl_index_for(tl_div)
                        _sync_camera_mode()
                        _persist()
                        _flash_oled(_tl_status_line())
                        dbg(3, "TL fav", tl_div)
                    else:
                        tl_div = 1
                        tl_index = 0
                        _sync_camera_mode()
                        _persist()
                        _flash_oled("TL x1")
                        dbg(3, "TL", 1)
                elif btn_tl.short_press:
                    if btn_option.pressed():
                        # OPTION + TL: increment divider by 1.
                        if tl_div < tl_option_max:
                            tl_div += 1
                        tl_index = _tl_index_for(tl_div)
                        _sync_camera_mode()
                        _persist()
                        _flash_oled(_tl_status_line())
                        dbg(3, "TL", tl_div, "+1")
                    else:
                        tl_index = (tl_index + 1) % len(_TL_DIVIDERS)
                        tl_div = _TL_DIVIDERS[tl_index]
                        _sync_camera_mode()
                        _persist()
                        if tl_div == 1:
                            _flash_oled("TL x1")
                        else:
                            _flash_oled(_tl_status_line())
                        dbg(3, "TL", tl_div)

            # --- OPTION + MOVE_L + MOVE_R ≥ 1 s → swap handedness ----------
            if option and move_l.pressed() and move_r.pressed():
                now = time.ticks_ms()
                if move_swap_since is None:
                    move_swap_since = now
                    move_swap_fired = False
                elif (
                    not move_swap_fired
                    and time.ticks_diff(now, move_swap_since) >= long_ms
                ):
                    swap_lr = not swap_lr
                    move_l, move_r, fast_l, fast_r = _bind_lr(
                        btn_move_l,
                        btn_move_r,
                        btn_fast_l,
                        btn_fast_r,
                        swap_lr,
                    )
                    move_swap_fired = True
                    _persist()
                    _flash_oled("L/R+Joy swap" if swap_lr else "L/R+Joy ok")
                    dbg(3, "SWAP_LR", swap_lr)
            else:
                move_swap_since = None
                move_swap_fired = False

            # --- MOVE_L + MOVE_R ≥ 3 s → swap direction, like FAST pair ----
            swap_hold_ms = 3000
            if move_l.pressed() and move_r.pressed():
                now = time.ticks_ms()
                if move_pair_since is None:
                    move_pair_since = now
                    move_pair_fired = False
                if (
                    not move_pair_fired
                    and time.ticks_diff(now, move_pair_since) >= swap_hold_ms
                ):
                    move_pair_fired = True
                    swap_lr = not swap_lr
                    move_l, move_r, fast_l, fast_r = _bind_lr(
                        btn_move_l,
                        btn_move_r,
                        btn_fast_l,
                        btn_fast_r,
                        swap_lr,
                    )
                    _persist()
                    _flash_oled(
                        "Move swap" if swap_lr else "Move swap ok"
                    )
                    dbg(3, "MOVE_SWAP", swap_lr)
            else:
                move_pair_since = None
                move_pair_fired = False

            # --- FAST_L + FAST_R ≥ 3 s → swap; with OPTION → dim toggle ----
            if btn_fast_l.pressed() and btn_fast_r.pressed():
                now = time.ticks_ms()
                if swap_since is None:
                    swap_since = now
                    swap_fired = False
                    swap_option_latched = option
                if option:
                    swap_option_latched = True
                if (
                    not swap_fired
                    and time.ticks_diff(now, swap_since) >= swap_hold_ms
                ):
                    swap_fired = True
                    if swap_option_latched:
                        # OPTION + FAST_L + FAST_R: toggle LED/OLED luminosity.
                        if ui.getLuminosity() < 0.99:
                            ui.setLuminosity(1.0)
                            _flash_oled("Bright")
                            dbg(3, "Bright")
                        else:
                            ui.setLuminosity(display_dim)
                            pct = int(display_dim * 100 + 0.5)
                            _flash_oled("Dim %d%%" % pct)
                            dbg(3, "Dim", pct)
                    else:
                        # Toggle MOVE/FAST binding and joystick polarity.
                        swap_lr = not swap_lr
                        move_l, move_r, fast_l, fast_r = _bind_lr(
                            btn_move_l,
                            btn_move_r,
                            btn_fast_l,
                            btn_fast_r,
                            swap_lr,
                        )
                        _persist()
                        _flash_oled(
                            "L/R+Joy swap" if swap_lr else "L/R+Joy ok"
                        )
                        dbg(3, "SWAP_LR", swap_lr)
            else:
                swap_since = None
                swap_fired = False
                swap_option_latched = False

            # --- STOP: press=stop(), ≥1s=halt(), ≥2s=disable; tap enables --
            # Ignore STOP while OPTION + two or more of A/B/C (real or ghosted
            # matrix STOP during joy-cal / OPTION+pair chords).
            _abc_n = (
                (1 if btn_a.pressed() else 0)
                + (1 if btn_b.pressed() else 0)
                + (1 if btn_c.pressed() else 0)
            )
            _stop_masked = option and _abc_n >= 2
            _double_option = btn_double_option.pressed()
            if (btn_stop.pressed() or btn_stop.edge_press) and not _stop_masked:
                if (
                    _double_option
                    and btn_stop.pressed()
                    and (btn_stop.edge_press or btn_double_option.edge_press)
                ):
                    # Both keypad OPTION keys + STOP → immediate emergency halt.
                    mc.halt()
                    _clear_move_pause()
                    _msm_clear()
                    mode = _IDLE
                    cruise_dir = 0
                    cruise_locked = False
                    goto_target = None
                    goto_t0_ms = None
                    loop_target = None
                    loop_dwell_until = None
                    _cancel_pending()
                    _flash_oled("Halt")
                    _halt_led_flash()
                    dbg(3, "Halt")
                elif btn_stop.edge_press and option and btn_a.pressed():
                    # OPTION + STOP + A (A already down when STOP pressed)
                    _cancel_pending()
                    _clear_move_pause()
                    _msm_clear()
                    if getattr(jks, "JKS_HOMING_ENABLED", True):
                        _enable(True)
                        driver_on = True
                        mode = _HOMING
                        cruise_dir = 0
                        cruise_locked = False
                        goto_target = None
                        loop_target = None
                        loop_dwell_until = None
                        mc.home()
                        dbg(3, "Homing")
                    else:
                        _flash_oled("No homing")
                        dbg(3, "No homing")
                elif btn_stop.edge_press and option and _abc_n == 0:
                    if tl_div != 1 and tl_mode == "msm":
                        # OPTION + STOP in MSM: cycle camera FPS.
                        camera_fps = _next_camera_fps(camera_fps)
                        _sync_camera_mode()
                        _persist()
                        _flash_oled(_tl_status_line())
                        dbg(3, "FPS", camera_fps)
                    else:
                        # OPTION + STOP: peek marks (TL×1, Cont, or idle).
                        _peek_marks()
                elif btn_stop.edge_press and not option:
                    if not driver_on:
                        _enable(True)
                        driver_on = True
                        _flash_oled("Enabled")
                        dbg(3, "Enable")
                    else:
                        # Already decelerating (e.g. 2nd press) → halt(); else stop().
                        # Keeps Delay Ns setting either way.
                        _clear_move_pause()
                        _msm_clear()
                        mode = _IDLE
                        cruise_dir = 0
                        cruise_locked = False
                        goto_target = None
                        goto_t0_ms = None
                        loop_target = None
                        loop_dwell_until = None
                        _cancel_pending()
                        if mc.isDecelerating():
                            mc.halt()
                            _flash_oled("Halt")
                            _halt_led_flash()
                            dbg(3, "Halt")
                        else:
                            mc.stop()
                            _flash_oled("Stopped")
                            dbg(3, "STOP")
                elif btn_stop.extra_long_press:
                    mc.halt()
                    _clear_move_pause()
                    _msm_clear()
                    mode = _IDLE
                    cruise_dir = 0
                    cruise_locked = False
                    goto_target = None
                    goto_t0_ms = None
                    loop_target = None
                    loop_dwell_until = None
                    _cancel_pending()
                    _enable(False)
                    driver_on = False
                    _flash_oled("Disabled")
                    dbg(3, "Disable")
                elif btn_stop.long_press:
                    mc.halt()
                    _clear_move_pause()
                    _msm_clear()
                    mode = _IDLE
                    cruise_dir = 0
                    cruise_locked = False
                    goto_target = None
                    goto_t0_ms = None
                    loop_target = None
                    loop_dwell_until = None
                    _cancel_pending()
                    _flash_oled("Halt")
                    _halt_led_flash()
                    dbg(3, "Halt")
                elif option and btn_a.edge_press:
                    # OPTION + STOP + A → homing
                    _cancel_pending()
                    _clear_move_pause()
                    _msm_clear()
                    if getattr(jks, "JKS_HOMING_ENABLED", True):
                        _enable(True)
                        driver_on = True
                        mode = _HOMING
                        cruise_dir = 0
                        cruise_locked = False
                        goto_target = None
                        loop_target = None
                        loop_dwell_until = None
                        mc.home()
                        dbg(3, "Homing")
                    else:
                        _flash_oled("No homing")
                        dbg(3, "No homing")
                elif btn_a.edge_press:
                    if not driver_on:
                        _enable(True)
                        driver_on = True
                    if soft_min is None:
                        _flash_oled("No soft min")
                    elif not _speed_ok(speed):
                        _flash_oled("Set SPEED")
                    else:
                        _request_action(
                            ("goto", "min", soft_min), speed, accel
                        )
                    dbg(3, "Goto min", soft_min)
                elif btn_b.edge_press:
                    if soft_min is None:
                        _flash_oled("No soft mid")
                    else:
                        mid = 0.5 * (float(soft_min) + float(soft_max))
                        if not driver_on:
                            _enable(True)
                            driver_on = True
                        if not _speed_ok(speed):
                            _flash_oled("Set SPEED")
                        else:
                            _request_action(
                                ("goto", "mid", mid), speed, accel
                            )
                        dbg(3, "Goto mid", round(mid, 2))
                elif btn_c.edge_press:
                    if not driver_on:
                        _enable(True)
                        driver_on = True
                    if not _speed_ok(speed):
                        _flash_oled("Set SPEED")
                    else:
                        _request_action(
                            ("goto", "max", soft_max), speed, accel
                        )
                    dbg(3, "Goto max", round(soft_max, 2))
                _push_oled(delay_preview_s=delay_preview_s)
                await asyncio.sleep_ms(20)
                continue

            # --- Pending delayed action countdown -------------------------
            if pending is not None:
                remain_ms = time.ticks_diff(pending_due_ms, time.ticks_ms())
                if remain_ms <= 0:
                    action, sp, ac = pending
                    _cancel_pending()
                    _run_action(action, sp, ac)
                else:
                    wait_s = remain_ms / 1000.0
                    _push_oled(
                        wait_s=wait_s, delay_preview_s=delay_preview_s
                    )
                    await asyncio.sleep_ms(20)
                    continue

            if mode == _HOMING:
                if not mc.isMoving():
                    mode = _IDLE
                    if mc.isDRVErrorActive():
                        _flash_oled("Homing abort")
                        dbg(2, "Homing abort")
                    else:
                        _flash_oled("Homed")
                        dbg(3, "Homing done", round(mc.getPosition(), 2))
                _push_oled()
                await asyncio.sleep_ms(20)
                continue

            # Mid-move DELAY pause: hold mode, skip motion retarget / arrival.
            if move_paused:
                _tick_cont_timer()
                _push_oled(
                    wait_s=wait_s,
                    delay_preview_s=delay_preview_s,
                    speed_mm_s=speed,
                    accel_mm_s2=accel,
                )
                await asyncio.sleep_ms(20)
                continue

            # MSM take: panel owns shutter + hops; skip continuous retarget.
            if msm_active:
                _msm_tick(speed, accel)
                _push_oled(
                    wait_s=wait_s,
                    delay_preview_s=delay_preview_s,
                    speed_mm_s=speed,
                    accel_mm_s2=accel,
                )
                await asyncio.sleep_ms(20)
                continue

            _tick_cont_timer()
            # --- A/B/C: hold all ≥ 1 s resets; +OPTION = joy centre cal ---
            pa = btn_a.pressed()
            pb = btn_b.pressed()
            pc = btn_c.pressed()
            now = time.ticks_ms()
            if pa and pb and pc:
                combo_now = "ABC"
                if abc_since is None:
                    abc_since = now
                    abc_fired = False
                    abc_option_latched = option
                if option:
                    abc_option_latched = True
                if (
                    not abc_fired
                    and time.ticks_diff(now, abc_since) >= long_ms
                ):
                    abc_fired = True
                    combo_lock = True
                    if abc_option_latched:
                        # OPTION + A + B + C: recalibrate joystick 0-speed.
                        if mc.isMoving() or mc.isHoming():
                            _flash_oled("Stop first")
                            dbg(2, "Joy centre busy")
                        elif adc_joy is None or filt_joy is None:
                            _flash_oled("No joystick")
                            dbg(2, "Joy centre none")
                        else:
                            joy_center = _clamp_joy_center(
                                filt_joy.read_norm(adc_joy)
                            )
                            _persist()
                            _flash_oled("Joy 0 set")
                            dbg(3, "Joy centre", round(joy_center, 4))
                    else:
                        pos_a, pos_b, pos_c = _default_positions(
                            soft_min, mc.slider_max
                        )
                        _persist()
                        if mode == _LOOP:
                            mc.stop()
                            _msm_clear()
                            mode = _IDLE
                            loop_pair = None
                            loop_target = None
                            loop_dwell_until = None
                        cruise_dir = 0
                        cruise_locked = False
                        _flash_oled("Pos reset")
                        dbg(
                            3,
                            "Pos reset",
                            round(pos_a, 2),
                            round(pos_b, 2),
                            round(pos_c, 2),
                        )
            else:
                abc_since = None
                abc_fired = False
                abc_option_latched = False
                if pa and pb:
                    combo_now = "AB"
                elif pa and pc:
                    combo_now = "AC"
                elif pb and pc:
                    combo_now = "BC"
                else:
                    combo_now = None

            if (
                combo_now is not None
                and combo_now != "ABC"
                and combo_now != combo_prev
            ):
                combo_lock = True
                if mode == _LOOP and loop_pair == combo_now:
                    mc.stop()
                    mode = _IDLE
                    loop_pair = None
                    loop_target = None
                    loop_dwell_until = None
                    _flash_oled("Loop stop")
                    dbg(3, "Loop stop")
                else:
                    if combo_now == "AB":
                        p2, n2 = pos_b, "PosB"
                    elif combo_now == "AC":
                        p2, n2 = pos_c, "PosC"
                    else:
                        p2, n2 = pos_c, "PosC"
                    _request_action(("loop", combo_now, p2, n2), speed, accel)

            combo_prev = combo_now
            if combo_lock:
                if not pa and not pb and not pc:
                    combo_lock = False
                _push_oled(
                    wait_s=wait_s,
                    delay_preview_s=delay_preview_s,
                    combo_held=True,
                )
                await asyncio.sleep_ms(20)
                continue

            # --- Store PosA / PosB / PosC (long press ≥ 1 s) ---------------
            if btn_a.long_press and not (pb or pc):
                pos_a = _clamp_store_position(
                    mc.getPosition(), soft_min, mc.slider_max
                )
                _persist()
                _flash_oled("PosA saved")
                dbg(3, "Store A", round(pos_a, 2))
            if btn_b.long_press and not (pa or pc):
                pos_b = _clamp_store_position(
                    mc.getPosition(), soft_min, mc.slider_max
                )
                _persist()
                _flash_oled("PosB saved")
                dbg(3, "Store B", round(pos_b, 2))
            if btn_c.long_press and not (pa or pb):
                pos_c = _clamp_store_position(
                    mc.getPosition(), soft_min, mc.slider_max
                )
                _persist()
                _flash_oled("PosC saved")
                dbg(3, "Store C", round(pos_c, 2))

            # --- Short press A / B / C → goto (OPTION before press = max speed + max accel) ---
            goto_spd = float(mc.max_speed) if option else speed
            goto_acc = float(mc.max_accel) if option else accel
            if btn_a.short_press and not (pb or pc):
                _request_action(("goto", "PosA", pos_a), goto_spd, goto_acc)
            if btn_b.short_press and not (pa or pc):
                _request_action(("goto", "PosB", pos_b), goto_spd, goto_acc)
            if btn_c.short_press and not (pa or pb):
                _request_action(("goto", "PosC", pos_c), goto_spd, goto_acc)

            if mode == _GOTO and goto_target is not None:
                if option and not goto_boost:
                    goto_boost = True
                    mc.setSpeed(float(mc.max_speed))
                    dbg(3, "Goto boost", goto_target)
                elif (not option) and goto_boost:
                    goto_boost = False
                    mc.setSpeed(max(speed, config.MIN_SPEED_MM_S))
                    dbg(3, "Goto boost end", goto_target)

            # --- Optional joystick (OPTION = max speed/accel) --------------
            joy_active = False
            cruise_boost = False
            fast_dir = 0
            if adc_joy is not None:
                if option:
                    joy_ref = float(mc.max_speed)
                    _apply_motion_params(
                        float(mc.max_speed), float(mc.max_accel)
                    )
                else:
                    joy_ref = max(speed, config.MIN_SPEED_MM_S)
                joy_cmd = _read_joystick_mm_s(
                    filt_joy, adc_joy, joy_ref, swap_lr, joy_center
                )
                if abs(joy_cmd) >= config.MIN_SPEED_MM_S:
                    joy_active = True
                    was_loop = mode == _LOOP
                    had_pending = pending is not None
                    _cancel_pending()
                    _enable(True)
                    driver_on = True
                    if mode != _JOYSTICK:
                        if was_loop:
                            _flash_oled("Joy/Loop stop")
                        elif had_pending:
                            _flash_oled("Joy take")
                        else:
                            _flash_oled("Joy")
                        dbg(3, "Joystick")
                    mc.move(joy_cmd)
                    mode = _JOYSTICK
                    cruise_dir = 0
                    cruise_locked = False
                    loop_pair = None
                    loop_target = None
                    loop_dwell_until = None
                    goto_target = None
                elif mode == _JOYSTICK:
                    mc.move(0.0)
                    mode = _IDLE
                    dbg(3, "Joystick center")

            if not joy_active:
                # Skip FAST swap chord from normal FAST jog.
                both_fast = btn_fast_l.pressed() and btn_fast_r.pressed()

                for _mv_btn, _mv_dir in ((move_l, -1), (move_r, 1)):
                    if not _mv_btn.edge_press:
                        continue
                    if (
                        mode == _CRUISE
                        and cruise_dir == _mv_dir
                        and cruise_locked
                    ):
                        # Same MOVE tip while locked → stop.
                        mc.stop()
                        mode = _IDLE
                        cruise_dir = 0
                        cruise_locked = False
                        _flash_oled("Stopped")
                        dbg(3, "Cruise tip stop")
                    elif not _speed_ok(speed):
                        _flash_oled("Set SPEED")
                    elif mode == _CRUISE:
                        cruise_dir = _mv_dir
                        cruise_locked = False
                        goto_target = None
                        mc.move(_signed(_mv_dir > 0, eff_speed))
                        dbg(3, "Cruise", "R" if _mv_dir > 0 else "L")
                    else:
                        cruise_locked = False
                        move_start_accel = float(mc.max_accel) if option else accel
                        _request_action(("cruise", _mv_dir), speed, move_start_accel)

                # Release decides locked vs hold-to-run.
                for _mv_btn, _mv_sign in ((move_l, -1), (move_r, 1)):
                    if not (
                        _mv_btn.edge_release
                        and mode == _CRUISE
                        and cruise_dir * _mv_sign > 0
                    ):
                        continue
                    if _mv_btn.last_hold_ms <= move_tap_ms:
                        cruise_locked = True
                        dbg(4, "Cruise locked", "R" if _mv_sign > 0 else "L")
                    elif pause_resume_guard:
                        pause_resume_guard = False
                    else:
                        mc.stop()
                        mode = _IDLE
                        cruise_dir = 0
                        cruise_locked = False
                        dbg(3, "Cruise hold stop", "R" if _mv_sign > 0 else "L")

                if move_sem.short_release_latched and mode == _CRUISE and cruise_dir != 0:
                    cruise_locked = True
                    dbg(4, "Cruise latched by shared move semantics", move_sem.direction)
                if move_sem.long_release_stop and mode == _CRUISE and cruise_dir != 0:
                    mc.stop()
                    mode = _IDLE
                    cruise_dir = 0
                    cruise_locked = False
                    dbg(3, "Cruise hold stop via shared semantics", move_sem.direction)

                if mode == _CRUISE and cruise_dir != 0:
                    # FAST matching side, or OPTION held → max cruise boost.
                    cruise_boost = option or (
                        not both_fast
                        and (
                            (
                                cruise_dir < 0
                                and fast_l.pressed()
                                and not fast_r.pressed()
                            )
                            or (
                                cruise_dir > 0
                                and fast_r.pressed()
                                and not fast_l.pressed()
                            )
                        )
                    )
                    if cruise_boost:
                        cruise_speed = float(mc.max_speed) / (
                            tl_div if tl_div > 0 else 1
                        )
                        mc.setSpeed(cruise_speed)
                    else:
                        cruise_speed = eff_speed
                        mc.setSpeed(cruise_speed)
                    mc.move(_signed(cruise_dir > 0, cruise_speed))
                    if mc.isAtHardLimit():
                        mode = _IDLE
                        cruise_dir = 0
                        cruise_locked = False
                        _flash_oled("Hard limit")
                        dbg(1, "Limit hard")
                    elif mc.isAtSoftLimit() and not allow_move_out_of_soft_limit(
                        mc.getPosition(), cruise_dir, soft_min, soft_max
                    ):
                        mc.stop()
                        mode = _IDLE
                        cruise_dir = 0
                        cruise_locked = False
                        _flash_oled("Soft limit")
                        dbg(2, "Limit soft")
                elif not both_fast and (fast_l.pressed() or fast_r.pressed()):
                    fast_speed = float(mc.max_speed)
                    fast_accel = float(mc.max_accel)
                    _enable(True)
                    driver_on = True
                    _apply_full_motion_params(fast_speed, fast_accel)
                    if fast_l.pressed() and not fast_r.pressed():
                        mode = _FAST
                        cruise_dir = 0
                        cruise_locked = False
                        fast_dir = -1
                        goto_target = None
                        mc.move(_signed(False, fast_speed))
                    elif fast_r.pressed() and not fast_l.pressed():
                        mode = _FAST
                        cruise_dir = 0
                        cruise_locked = False
                        fast_dir = 1
                        goto_target = None
                        mc.move(_signed(True, fast_speed))
                elif mode == _FAST:
                    mc.stop()
                    mode = _IDLE

                # --- Pair loop with optional dwell ------------------------
                if mode == _LOOP and loop_pair is not None and mc.isAtHardLimit():
                    mode = _IDLE
                    loop_pair = None
                    loop_target = None
                    loop_dwell_until = None
                    _flash_oled("Hard limit")
                    dbg(1, "Limit hard")
                elif mode == _LOOP and loop_pair is not None:
                    if speed >= config.MIN_SPEED_MM_S:
                        mc.setSpeed(max(eff_speed, config.MIN_SPEED_MM_S))
                    if loop_pair == "AB":
                        p1, p2 = pos_a, pos_b
                        n1, n2 = "PosA", "PosB"
                    elif loop_pair == "AC":
                        p1, p2 = pos_a, pos_c
                        n1, n2 = "PosA", "PosC"
                    else:
                        p1, p2 = pos_b, pos_c
                        n1, n2 = "PosB", "PosC"
                    if not mc.isMoving() and not move_paused:
                        if loop_dwell_ms > 0:
                            if loop_dwell_until is None:
                                loop_dwell_until = time.ticks_add(
                                    time.ticks_ms(), loop_dwell_ms
                                )
                            remain = time.ticks_diff(
                                loop_dwell_until, time.ticks_ms()
                            )
                            if remain > 0:
                                dwell_s = remain / 1000.0
                            else:
                                loop_dwell_until = None
                                if loop_toward_p2:
                                    mc.moveTo(p1)
                                    loop_toward_p2 = False
                                    loop_target = n1
                                    dbg(3, "Loop ->", n1)
                                else:
                                    mc.moveTo(p2)
                                    loop_toward_p2 = True
                                    loop_target = n2
                                    dbg(3, "Loop ->", n2)
                        else:
                            if loop_toward_p2:
                                mc.moveTo(p1)
                                loop_toward_p2 = False
                                loop_target = n1
                                dbg(3, "Loop ->", n1)
                            else:
                                mc.moveTo(p2)
                                loop_toward_p2 = True
                                loop_target = n2
                                dbg(3, "Loop ->", n2)

            if mode == _GOTO and not mc.isMoving() and not move_paused and not msm_active:
                if mc.isAtHardLimit():
                    mode = _IDLE
                    goto_target = None
                    goto_t0_ms = None
                    ui.setCameraMotionActive(False)
                    _flash_oled("Hard limit")
                    dbg(1, "Limit hard")
                else:
                    arrived = goto_target
                    mode = _IDLE
                    goto_target = None
                    goto_t0_ms = None
                    ui.setCameraMotionActive(False)
                    if arrived:
                        _flash_oled("At " + arrived)
                    dbg(3, "Goto done", round(mc.getPosition(), 2))

            _push_oled(
                cruise_boost=cruise_boost,
                fast_dir=fast_dir,
                wait_s=wait_s,
                dwell_s=dwell_s,
                delay_preview_s=delay_preview_s,
                speed_mm_s=speed,
                accel_mm_s2=accel,
            )

            await asyncio.sleep_ms(20)
    finally:
        mc.halt()
        await mc.wait()
        _enable(False)
        ui.setOledText("")
        dbg(3, "stopped")


def run():
    asyncio.run(main())


if __name__ == "__main__":
    run()
