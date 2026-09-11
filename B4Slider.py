# B4Slider — MOVE_L/R + AXIS_1..5 camera slider panel (OPTION, SET,
# SPEED/ACCEL pot or QD encoder). Recommended silk is AXIS 1/2/3.
#
# Soft limits are the A/B working window per axis. Reuses MC_Client + UIC_Base.
# Config: B4SliderConfig.py (B4S_*); overlay via SliderPins.B4Slider.
#
# Usage:
#   import B4Slider
#   B4Slider.run()

import time

import uasyncio as asyncio
from machine import ADC, Pin

import B4SliderConfig as b4s
from MC_client import MC_Client
from UIC_base import UIC_Base, dbg
from button_state import (
    ButtonAdapter,
    allow_move_out_of_soft_limit,
    resolve_move_semantics,
)
from b4_logic import (
    apply_encoder_steps,
    clamp_enter,
    format_axis_oled,
    qd_detents,
    resolve_axis_mask,
    rotary_boot_value,
)

_IDLE = 0
_CRUISE = 1
_HOLD = 2
_HOMING = 3

_WHITE = (255, 255, 255)
_VIOLET = (180, 0, 255)
_RED = (255, 0, 0)
_GREEN = (0, 255, 0)
_BLUE = (0, 0, 255)
_DIM_WHITE = (31, 31, 31)
_DIM_BLUE = (0, 0, 80)


class _Btn:
    """Debounced button with short / long / extra-long / learn-long press."""

    def __init__(self, pin_no, debounce_ms, long_ms, extra_long_ms=None, learn_ms=None):
        self._pin = Pin(pin_no, Pin.IN, Pin.PULL_UP)
        self._debounce_ms = debounce_ms
        self._long_ms = long_ms
        self._extra_long_ms = extra_long_ms
        self._learn_ms = learn_ms
        self._stable = False
        self._raw_last = False
        self._change_ms = time.ticks_ms()
        self._down_ms = None
        self._long_fired = False
        self._extra_long_fired = False
        self._learn_fired = False
        self.edge_press = False
        self.edge_release = False
        self.short_press = False
        self.long_press = False
        self.extra_long_press = False
        self.learn_press = False
        self.last_hold_ms = 0

    def pressed(self):
        return self._stable

    def hold_ms(self):
        if self._stable and self._down_ms is not None:
            return time.ticks_diff(time.ticks_ms(), self._down_ms)
        return 0

    def update(self):
        self.edge_press = False
        self.edge_release = False
        self.short_press = False
        self.long_press = False
        self.extra_long_press = False
        self.learn_press = False

        raw = self._pin.value() == 0
        now = time.ticks_ms()
        if raw != self._raw_last:
            self._raw_last = raw
            self._change_ms = now

        if time.ticks_diff(now, self._change_ms) >= self._debounce_ms:
            if raw != self._stable:
                self._stable = raw
                if self._stable:
                    self._down_ms = now
                    self._long_fired = False
                    self._extra_long_fired = False
                    self._learn_fired = False
                    self.edge_press = True
                else:
                    held = 0
                    if self._down_ms is not None:
                        held = time.ticks_diff(now, self._down_ms)
                    self.last_hold_ms = held
                    self.edge_release = True
                    if (
                        not self._long_fired
                        and not self._extra_long_fired
                        and not self._learn_fired
                    ):
                        self.short_press = True
                    self._down_ms = None

        if self._stable and self._down_ms is not None:
            held = time.ticks_diff(now, self._down_ms)
            self.last_hold_ms = held
            if (
                self._learn_ms is not None
                and not self._learn_fired
                and held >= self._learn_ms
            ):
                self._learn_fired = True
                self._extra_long_fired = True
                self._long_fired = True
                self.learn_press = True
            elif (
                self._extra_long_ms is not None
                and not self._extra_long_fired
                and held >= self._extra_long_ms
            ):
                self._extra_long_fired = True
                self._long_fired = True
                self.extra_long_press = True
            elif not self._long_fired and held >= self._long_ms:
                self._long_fired = True
                self.long_press = True


class _PotFilter:
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
        total = 0
        for _ in range(self._n):
            total += adc.read_u16()
        x = (total / self._n) / 65535.0
        if self._y is None:
            self._y = x
            self._out = x
            return self._out
        self._y += self._alpha * (x - self._y)
        if abs(self._y - self._out) >= self._hyst:
            self._out = self._y
        return self._out


def _pot_norm(norm, deadzone):
    if norm < deadzone:
        return 0.0
    norm = (norm - deadzone) / (1.0 - deadzone)
    if norm > 1.0:
        return 1.0
    return norm


def _read_speed(filt, adc, lo, hi):
    norm = _pot_norm(filt.read_norm(adc), b4s.B4S_SPEED_DEADZONE)
    gamma = float(getattr(b4s, "B4S_SPEED_CURVE_GAMMA", 1.0))
    if gamma != 1.0 and norm > 0.0:
        norm = norm ** gamma
    lo = float(lo)
    hi = float(hi)
    if hi < lo:
        hi = lo
    if norm <= 0.0:
        return lo
    return lo + norm * (hi - lo)


def _read_accel(filt, adc, lo, hi):
    norm = _pot_norm(filt.read_norm(adc), b4s.B4S_SPEED_DEADZONE)
    lo = float(lo)
    hi = float(hi)
    if hi < lo:
        hi = lo
    return lo + norm * (hi - lo)


def _envelope_side(raw, fallback):
    if raw is None:
        return float(fallback)
    return float(raw)


def _accel_input():
    acc = str(getattr(b4s, "B4S_ACCEL_INPUT", "pot")).strip().lower()
    if acc in ("pot", "rotary", "set"):
        return acc
    if int(getattr(b4s, "B4S_USE_ACCEL_POT", 0)) != 0:
        return "pot"
    return "set"


async def _wait_boot_unlock(ui, btn_option, update_all):
    dbg(3, "B4S locked")
    ui.startLedRainbowLoop()
    try:
        while True:
            update_all()
            if btn_option.edge_press:
                dbg(3, "B4S unlocked")
                return
            ui.driveLed()
            await asyncio.sleep_ms(20)
    finally:
        ui.stopLedEffect()


async def _run_homing(mc, ui, axes):
    dbg(3, "B4S homing", axes)
    for axis in axes:
        mc.home(axis)
        while mc.isMoving() or mc.isHoming():
            await asyncio.sleep_ms(20)
        if mc.isDRVErrorActive():
            ui.ledFlash(_RED, 2, 80, 80)
            dbg(2, "B4S homing abort", axis)
            return False
    dbg(3, "B4S homed")
    return True


def run():
    try:
        asyncio.run(main())
    finally:
        try:
            asyncio.new_event_loop()
        except Exception:
            pass


async def main():
    mc = MC_Client()
    ui = UIC_Base()
    mc.set_axis_status_callback(ui.on_axis_status)
    await mc.start()
    await ui.start()

    near_mm = float(getattr(b4s, "B4S_NEAR_SOFT_MM", 3.0))
    try:
        import UIC_config as uic_cfg

        uic_cfg.SOFT_LIMIT_WARN_MM = near_mm
    except Exception:
        pass

    debounce = int(b4s.B4S_BTN_DEBOUNCE_MS)
    long_ms = int(b4s.B4S_LONG_PRESS_MS)
    extra_ms = int(b4s.B4S_EXTRA_LONG_MS)
    learn_ms = int(b4s.B4S_LEARN_HOLD_MS)
    move_tap_ms = int(b4s.B4S_MOVE_TAP_MS)

    move_l_pin = Pin(b4s.PIN_BTN_MOVE_L, Pin.IN, Pin.PULL_UP)
    move_r_pin = Pin(b4s.PIN_BTN_MOVE_R, Pin.IN, Pin.PULL_UP)
    option_pin = Pin(b4s.PIN_BTN_OPTION, Pin.IN, Pin.PULL_UP)

    move_l = ButtonAdapter(lambda: move_l_pin.value() == 0, debounce, long_ms, extra_ms)
    move_r = ButtonAdapter(lambda: move_r_pin.value() == 0, debounce, long_ms, extra_ms)
    btn_option = ButtonAdapter(lambda: option_pin.value() == 0, debounce, long_ms, extra_ms)
    btn_set = _Btn(b4s.PIN_BTN_SET, debounce, long_ms, extra_ms, learn_ms)

    ax1_pin = Pin(b4s.PIN_BTN_AXIS_1, Pin.IN, Pin.PULL_UP)
    ax2_pin = Pin(b4s.PIN_BTN_AXIS_2, Pin.IN, Pin.PULL_UP)
    ax3_pin = Pin(b4s.PIN_BTN_AXIS_3, Pin.IN, Pin.PULL_UP)
    ax4_pin = Pin(b4s.PIN_BTN_AXIS_4, Pin.IN, Pin.PULL_UP)
    ax5_pin = Pin(b4s.PIN_BTN_AXIS_5, Pin.IN, Pin.PULL_UP)
    btn_ax1 = ButtonAdapter(lambda: ax1_pin.value() == 0, debounce, long_ms, extra_ms)
    btn_ax2 = ButtonAdapter(lambda: ax2_pin.value() == 0, debounce, long_ms, extra_ms)
    btn_ax3 = ButtonAdapter(lambda: ax3_pin.value() == 0, debounce, long_ms, extra_ms)
    btn_ax4 = ButtonAdapter(lambda: ax4_pin.value() == 0, debounce, long_ms, extra_ms)
    btn_ax5 = ButtonAdapter(lambda: ax5_pin.value() == 0, debounce, long_ms, extra_ms)

    axis_count = int(mc.getAxisCount())
    if axis_count < 1:
        axis_count = 1
    motor_count = int(mc.getMotorCount())
    if motor_count < 1:
        motor_count = 1
    panel_axes = 5 if axis_count > 5 else axis_count

    def update_all():
        move_l.update()
        move_r.update()
        btn_option.update()
        btn_set.update()
        btn_ax1.update()
        btn_ax2.update()
        btn_ax3.update()
        btn_ax4.update()
        btn_ax5.update()

    if getattr(b4s, "B4S_BOOT_UNLOCK", True):
        await _wait_boot_unlock(ui, btn_option, update_all)

    speed_max = float(b4s.B4S_SPEED_MAX_MM_S)
    accel_max = float(b4s.B4S_ACCEL_MAX_MM_S2)
    if mc.max_speed is not None and mc.max_speed < speed_max:
        speed_max = float(mc.max_speed)
    if mc.max_accel is not None and mc.max_accel < accel_max:
        accel_max = float(mc.max_accel)
    mc.max_speed = speed_max
    mc.max_accel = accel_max
    mc._max_speed_mm_s = speed_max

    fb_min = float(getattr(b4s, "B4S_SOFT_FALLBACK_MIN", -2000.0))
    fb_max = float(getattr(b4s, "B4S_SOFT_FALLBACK_MAX", 2000.0))

    def _mc_env_min(n):
        if n == 1:
            return mc.slider_min
        return getattr(mc, "slider_min_%d" % n, None)

    def _mc_env_max(n):
        if n == 1:
            return mc.slider_max
        return getattr(mc, "slider_max_%d" % n, None)

    def refresh_envelopes():
        lo = []
        hi = []
        i = 1
        while i <= 5:
            lo.append(_envelope_side(_mc_env_min(i), fb_min))
            hi.append(_envelope_side(_mc_env_max(i), fb_max))
            i += 1
        return lo, hi

    env_lo, env_hi = refresh_envelopes()
    soft_lo = list(env_lo)
    soft_hi = list(env_hi)

    left_neg = [
        bool(b4s.B4S_LEFT_IS_NEGATIVE),
        bool(getattr(b4s, "B4S_LEFT2_IS_NEGATIVE", b4s.B4S_LEFT_IS_NEGATIVE)),
        bool(getattr(b4s, "B4S_LEFT3_IS_NEGATIVE", b4s.B4S_LEFT_IS_NEGATIVE)),
        bool(getattr(b4s, "B4S_LEFT4_IS_NEGATIVE", b4s.B4S_LEFT_IS_NEGATIVE)),
        bool(getattr(b4s, "B4S_LEFT5_IS_NEGATIVE", b4s.B4S_LEFT_IS_NEGATIVE)),
    ]

    selected = frozenset((1,))

    def apply_soft_limits():
        n = panel_axes
        i = 0
        while i < n:
            if soft_lo[i] > soft_hi[i]:
                soft_lo[i], soft_hi[i] = soft_hi[i], soft_lo[i]
            i += 1
        left = [soft_lo[j] for j in range(n)]
        right = [soft_hi[j] for j in range(n)]
        mc.setLeft(*left)
        mc.setRight(*right)
        ui.set_soft_limits(soft_lo[0], soft_hi[0])

    apply_soft_limits()

    if getattr(b4s, "B4S_HOMING_ENABLED", True):
        homing_axes = list(range(1, motor_count + 1))
        mode = _HOMING
        ok = await _run_homing(mc, ui, homing_axes)
        if ok:
            env_lo, env_hi = refresh_envelopes()
            soft_lo = list(env_lo)
            soft_hi = list(env_hi)
            apply_soft_limits()
        mode = _IDLE
    else:
        mode = _IDLE

    speed_input = str(getattr(b4s, "B4S_SPEED_INPUT", "pot")).strip().lower()
    if speed_input not in ("pot", "rotary"):
        speed_input = "pot"
    accel_input = _accel_input()
    use_accel_pot = accel_input == "pot"
    use_accel_set = accel_input == "set"
    use_accel_rot = accel_input == "rotary"
    use_speed_rot = speed_input == "rotary"

    accel_preset = "L"
    accel_l = float(b4s.B4S_ACCEL_PRESET_L)
    accel_h = float(b4s.B4S_ACCEL_PRESET_H)
    accel_cmd = accel_l

    loop_armed = False
    cruise_dir = 0
    cruise_locked = False
    option_boost = False
    set_tick_sec = 0
    learn_active = False
    swap_lr = False
    move_swap_since = None
    move_swap_fired = False
    driver_enabled = True
    speed_at_limit = False
    accel_at_limit = False

    adc_speed = None
    filt_speed = None
    adc_accel = None
    filt_accel = None
    if not use_speed_rot:
        adc_speed = ADC(Pin(b4s.PIN_POT_SPEED))
        filt_speed = _PotFilter(
            b4s.B4S_POT_OVERSAMPLE, b4s.B4S_POT_EMA_ALPHA, b4s.B4S_POT_HYST
        )
    if use_accel_pot:
        adc_accel = ADC(Pin(b4s.PIN_POT_ACCEL))
        filt_accel = _PotFilter(
            b4s.B4S_POT_OVERSAMPLE,
            b4s.B4S_ACCEL_EMA_ALPHA,
            b4s.B4S_ACCEL_HYST,
        )

    enc_speed = None
    enc_accel = None
    speed_detents = 0
    accel_detents = 0
    qd_div = int(getattr(b4s, "B4S_QD_DIV", 4))
    qd_round = int(getattr(b4s, "B4S_QD_ROUND", 2))
    speed_qd_mode = int(getattr(b4s, "B4S_SPEED_QD_MODE", 11))
    accel_qd_mode = int(getattr(b4s, "B4S_ACCEL_QD_MODE", 11))
    if use_speed_rot or use_accel_rot:
        from QD import QD

        if use_speed_rot:
            enc_speed = QD(
                int(getattr(b4s, "B4S_QD_SPEED_SM", 2)),
                int(b4s.PIN_ENC_SPEED_A),
                int(b4s.PIN_ENC_SPEED_B),
                use_irq=True,
            )
            speed_detents = qd_detents(enc_speed.position, qd_div, qd_round)
        if use_accel_rot:
            enc_accel = QD(
                int(getattr(b4s, "B4S_QD_ACCEL_SM", 3)),
                int(b4s.PIN_ENC_ACCEL_A),
                int(b4s.PIN_ENC_ACCEL_B),
                use_irq=True,
            )
            accel_detents = qd_detents(enc_accel.position, qd_div, qd_round)

    pot_min = float(b4s.B4S_SPEED_MIN_MM_S)
    accel_lo = float(b4s.B4S_ACCEL_MIN_MM_S2)
    loop_blue = int(getattr(b4s, "B4S_LOOP_BLUE_ADD", 26))
    flash_on = int(b4s.B4S_LED_FLASH_ON_MS)
    flash_off = int(b4s.B4S_LED_FLASH_OFF_MS)
    blip_ms = int(b4s.B4S_LED_BLIP_MS)
    ping_ms = int(b4s.B4S_LED_PINGPONG_MS)
    flash_half_count = 3

    if use_speed_rot:
        speed_mm_s = rotary_boot_value(speed_max)
    else:
        speed_mm_s = pot_min
    if use_accel_rot:
        accel_cmd = rotary_boot_value(accel_max)

    def axis_pressed_list():
        held = []
        if btn_ax1.pressed():
            held.append(1)
        if btn_ax2.pressed():
            held.append(2)
        if btn_ax3.pressed():
            held.append(3)
        if btn_ax4.pressed():
            held.append(4)
        if btn_ax5.pressed():
            held.append(5)
        return held

    def axis_edge_press():
        return (
            btn_ax1.edge_press
            or btn_ax2.edge_press
            or btn_ax3.edge_press
            or btn_ax4.edge_press
            or btn_ax5.edge_press
        )

    def show_selection(mask, flash=False):
        ui.setOledText(format_axis_oled(mask))
        if flash:
            n = len(mask)
            if n < 1:
                n = 1
            if n > 5:
                n = 5
            ui.ledFlash(_WHITE, n, flash_on, flash_off)

    def get_pos(axis_1based):
        if axis_1based == 1:
            return mc.getPosition()
        if axis_1based == 2:
            return mc.getPosition2()
        if axis_1based == 3:
            return mc.getPosition3()
        if axis_1based == 4:
            return mc.getPosition4()
        return mc.getPosition5()

    def target_for_dir(axis_index, direction):
        lo = soft_lo[axis_index]
        hi = soft_hi[axis_index]
        if left_neg[axis_index]:
            return lo if direction < 0 else hi
        return hi if direction < 0 else lo

    def pos_dir(axis_index, button_dir):
        if left_neg[axis_index]:
            return button_dir
        return -button_dir

    def any_move_pressed():
        return move_l.pressed() or move_r.pressed()

    def sync_loop_led():
        if loop_armed and mode == _IDLE and not mc.isMoving():
            ui.ledClearAdd()
            ui.ledPingPong(_DIM_WHITE, _DIM_BLUE, ping_ms)
        elif mc.isMoving() and loop_armed:
            ui.ledEffectClear()
            ui.ledAddColor(0, 0, loop_blue)
        else:
            if not mc.isMoving():
                ui.ledClearAdd()
                ui.ledEffectClear()

    def soft_stop():
        nonlocal mode, cruise_dir, cruise_locked, option_boost
        mc.stop()
        mode = _IDLE
        cruise_dir = 0
        cruise_locked = False
        option_boost = False
        sync_loop_led()

    def start_selected_cruise(direction, locked, speed_boost=False, accel_boost=False):
        nonlocal mode, cruise_dir, cruise_locked, option_boost
        args = [None, None, None, None, None]
        for ax in selected:
            if ax <= panel_axes:
                args[ax - 1] = target_for_dir(ax - 1, direction)
        spd = speed_max if speed_boost else speed_mm_s
        acc = accel_max if accel_boost else accel_cmd
        mc.setSpeed(spd)
        mc.setAcceleration(acc)
        mc.enable(True)
        mc.moveTo(args[0], args[1], args[2], args[3], args[4])
        cruise_dir = direction
        cruise_locked = locked
        option_boost = speed_boost
        mode = _CRUISE if locked else _HOLD
        sync_loop_led()

    def allow_selected_move(direction):
        for ax in selected:
            if ax > panel_axes:
                continue
            i = ax - 1
            pos = get_pos(ax)
            d = pos_dir(i, direction)
            if not allow_move_out_of_soft_limit(pos, d, soft_lo[i], soft_hi[i]):
                return False
        return True

    def poll_rotary(enc, last_det, value, vmax, qd_mode, at_limit):
        det = qd_detents(enc.position, qd_div, qd_round)
        delta = det - last_det
        if delta == 0:
            return value, det, at_limit, False
        new_v, limited = apply_encoder_steps(value, delta, qd_mode, vmax)
        enter = clamp_enter(at_limit, limited)
        return new_v, det, limited, enter

    def power_up_reset():
        nonlocal soft_lo, soft_hi, loop_armed, mode
        nonlocal cruise_dir, cruise_locked, accel_preset, accel_cmd, option_boost
        nonlocal driver_enabled, selected, speed_mm_s
        nonlocal speed_at_limit, accel_at_limit
        mc.halt()
        soft_lo = list(env_lo)
        soft_hi = list(env_hi)
        apply_soft_limits()
        selected = frozenset((1,))
        show_selection(selected, flash=False)
        loop_armed = False
        mode = _IDLE
        cruise_dir = 0
        cruise_locked = False
        option_boost = False
        driver_enabled = False
        accel_preset = "L"
        if use_accel_rot:
            accel_cmd = rotary_boot_value(accel_max)
            accel_at_limit = False
        else:
            accel_cmd = accel_l
        if use_speed_rot:
            speed_mm_s = rotary_boot_value(speed_max)
            speed_at_limit = False
        if use_accel_set:
            mc.setAcceleration(accel_cmd)
        ui.set_enabled(False)
        ui.ledClearAdd()
        ui.ledEffectClear()
        ui.ledFlash(_RED, 2, flash_on, flash_off)
        dbg(3, "B4S all-four reset")

    def apply_accel_preset(name):
        nonlocal accel_preset, accel_cmd
        if name == "H":
            accel_preset = "H"
            accel_cmd = accel_h
        else:
            accel_preset = "L"
            accel_cmd = accel_l
        if use_accel_set:
            mc.setAcceleration(accel_cmd)
        return accel_cmd

    def blink_green_clamp():
        ui.ledFlash(_GREEN, flash_half_count, flash_on, flash_off)

    mc.enable(True)
    if use_accel_set:
        mc.setAcceleration(accel_cmd)
    ui.set_commanded(speed_mm_s=speed_mm_s, accel_mm_s2=accel_cmd)
    show_selection(selected, flash=False)
    dbg(3, "B4Slider ready", "axes", axis_count, "sel", tuple(sorted(selected)))

    try:
        while True:
            update_all()

            if enc_speed is not None:
                speed_mm_s, speed_detents, speed_at_limit, enter = poll_rotary(
                    enc_speed,
                    speed_detents,
                    speed_mm_s,
                    speed_max,
                    speed_qd_mode,
                    speed_at_limit,
                )
                if enter:
                    blink_green_clamp()
            elif speed_input == "pot" and adc_speed is not None:
                speed_mm_s = _read_speed(filt_speed, adc_speed, pot_min, speed_max)

            if enc_accel is not None:
                accel_cmd, accel_detents, accel_at_limit, enter_a = poll_rotary(
                    enc_accel,
                    accel_detents,
                    accel_cmd,
                    accel_max,
                    accel_qd_mode,
                    accel_at_limit,
                )
                if enter_a:
                    blink_green_clamp()
            elif use_accel_pot:
                accel_cmd = _read_accel(filt_accel, adc_accel, accel_lo, accel_max)

            ui.set_commanded(speed_mm_s=speed_mm_s, accel_mm_s2=accel_cmd)

            held_axes = axis_pressed_list()
            new_sel, sel_invalid = resolve_axis_mask(held_axes, panel_axes, selected)
            if sel_invalid:
                if axis_edge_press():
                    ui.ledFlash(_BLUE, flash_half_count, flash_on, flash_off)
                    dbg(3, "B4S axis invalid", held_axes)
            elif new_sel != selected:
                selected = new_sel
                show_selection(selected, flash=True)
                dbg(3, "B4S axis", tuple(sorted(selected)))

            opt = btn_option.pressed()
            st = btn_set.pressed()
            move_sem = resolve_move_semantics(
                move_l.state, move_r.state, opt, move_tap_ms
            )

            all_four = move_l.pressed() and move_r.pressed() and opt and st
            lr_halt = move_l.pressed() and move_r.pressed() and not all_four

            if mode == _HOMING:
                await asyncio.sleep_ms(20)
                continue

            if all_four:
                if (
                    move_l.edge_press
                    or move_r.edge_press
                    or btn_option.edge_press
                    or btn_set.edge_press
                ):
                    power_up_reset()
                    await asyncio.sleep_ms(flash_on * 2 + flash_off * 2)
                    ui.ledFlash(_WHITE, 3, flash_on, flash_off)
                await asyncio.sleep_ms(20)
                continue

            if lr_halt:
                edge = move_l.edge_press or move_r.edge_press
                if edge:
                    mc.halt()
                    soft_stop()
                    driver_enabled = False
                    ui.set_enabled(False)
                    ui.ledFlash(_RED, 2, flash_on, flash_off)
                    dbg(3, "B4S halt")
                await asyncio.sleep_ms(20)
                continue

            if not driver_enabled:
                edge = (
                    move_l.edge_press
                    or move_r.edge_press
                    or btn_option.edge_press
                    or btn_set.edge_press
                    or axis_edge_press()
                )
                if edge:
                    mc.enable(True)
                    driver_enabled = True
                    ui.set_enabled(True)
                    dbg(3, "B4S enable")
                await asyncio.sleep_ms(20)
                continue

            if opt and st and not any_move_pressed():
                if btn_set.long_press or btn_option.long_press:
                    loop_armed = not loop_armed
                    sync_loop_led()
                    dbg(3, "B4S loop", loop_armed)
                elif (
                    (btn_set.short_press or btn_option.short_press)
                    and btn_set.last_hold_ms < long_ms
                    and btn_option.last_hold_ms < long_ms
                ):
                    mc.enable(False)
                    driver_enabled = False
                    ui.set_enabled(False)
                    soft_stop()
                    dbg(3, "B4S disable")
                await asyncio.sleep_ms(20)
                continue

            swap_hold_ms = 3000
            if move_l.pressed() and move_r.pressed() and not opt:
                now = time.ticks_ms()
                if move_swap_since is None:
                    move_swap_since = now
                    move_swap_fired = False
                if (
                    not move_swap_fired
                    and time.ticks_diff(now, move_swap_since) >= swap_hold_ms
                ):
                    move_swap_fired = True
                    swap_lr = not swap_lr
                    move_l, move_r = move_r, move_l
                    ui.ledBlip(_WHITE, blip_ms)
                    dbg(3, "B4S MOVE_SWAP", swap_lr)
            else:
                move_swap_since = None
                move_swap_fired = False

            # SET + MOVE soft limits on selected axes
            if st and not opt:
                if move_l.pressed() or move_r.pressed():
                    if move_l.pressed() and move_r.pressed():
                        if move_l.long_press or move_r.long_press or btn_set.long_press:
                            for ax in selected:
                                if ax <= panel_axes:
                                    soft_lo[ax - 1] = env_lo[ax - 1]
                                    soft_hi[ax - 1] = env_hi[ax - 1]
                            apply_soft_limits()
                            ui.ledBlip(_WHITE, blip_ms)
                            dbg(3, "B4S reset both soft", tuple(sorted(selected)))
                    elif move_l.pressed():
                        if move_l.long_press or btn_set.long_press:
                            for ax in selected:
                                if ax <= panel_axes:
                                    soft_lo[ax - 1] = env_lo[ax - 1]
                            apply_soft_limits()
                            ui.ledBlip(_WHITE, blip_ms)
                            dbg(3, "B4S reset soft_l", tuple(sorted(selected)))
                    elif move_r.pressed():
                        if move_r.long_press or btn_set.long_press:
                            for ax in selected:
                                if ax <= panel_axes:
                                    soft_hi[ax - 1] = env_hi[ax - 1]
                            apply_soft_limits()
                            ui.ledBlip(_WHITE, blip_ms)
                            dbg(3, "B4S reset soft_r", tuple(sorted(selected)))
                    await asyncio.sleep_ms(20)
                    continue

                if move_l.short_press and btn_set.last_hold_ms < long_ms:
                    for ax in selected:
                        if ax <= panel_axes:
                            soft_lo[ax - 1] = float(get_pos(ax))
                    apply_soft_limits()
                    ui.ledBlip(_WHITE, blip_ms)
                    dbg(3, "B4S set soft_l", tuple(sorted(selected)))
                    await asyncio.sleep_ms(20)
                    continue
                if move_r.short_press and btn_set.last_hold_ms < long_ms:
                    for ax in selected:
                        if ax <= panel_axes:
                            soft_hi[ax - 1] = float(get_pos(ax))
                    apply_soft_limits()
                    ui.ledBlip(_WHITE, blip_ms)
                    dbg(3, "B4S set soft_r", tuple(sorted(selected)))
                    await asyncio.sleep_ms(20)
                    continue

            if st and not opt and not any_move_pressed():
                if mc.isMoving() or mode != _IDLE:
                    if btn_set.edge_press:
                        soft_stop()
                        dbg(3, "B4S SET stop")
                elif use_accel_set:
                    sec = btn_set.hold_ms() // 1000
                    if btn_set.pressed() and sec > set_tick_sec:
                        set_tick_sec = sec
                        ui.ledFlash(_WHITE, 1, flash_on, flash_off)
                    if not btn_set.pressed():
                        set_tick_sec = 0

                    if btn_set.learn_press:
                        learn_active = True
                        dbg(3, "B4S accel learn…")
                    if learn_active and btn_set.pressed() and adc_speed is not None:
                        accel_cmd = _read_accel(
                            filt_speed, adc_speed, accel_lo, accel_max
                        )
                        mc.setAcceleration(accel_cmd)
                    if learn_active and btn_set.edge_release:
                        if accel_preset == "H":
                            accel_h = accel_cmd
                        else:
                            accel_l = accel_cmd
                        learn_active = False
                        ui.ledFlash(_VIOLET, 3, flash_on, flash_off)
                        dbg(3, "B4S accel learn", accel_cmd)
                    elif btn_set.extra_long_press:
                        apply_accel_preset("H")
                        ui.ledFlash(_VIOLET, 2, flash_on, flash_off)
                        dbg(3, "B4S accel H", accel_cmd)
                    elif btn_set.long_press:
                        apply_accel_preset("L")
                        ui.ledFlash(_VIOLET, 1, flash_on, flash_off)
                        dbg(3, "B4S accel L", accel_cmd)
                await asyncio.sleep_ms(20)
                continue

            moving = mc.isMoving() or mode in (_CRUISE, _HOLD)

            if moving and mode in (_CRUISE, _HOLD):
                if opt:
                    if not option_boost:
                        option_boost = True
                        mc.setSpeed(speed_max)
                else:
                    if option_boost:
                        option_boost = False
                    mc.setSpeed(speed_mm_s)
                if use_accel_pot or use_accel_rot:
                    mc.setAcceleration(accel_cmd)

            if mode == _CRUISE and cruise_locked and not mc.isMoving():
                if loop_armed:
                    cruise_dir = -cruise_dir
                    start_selected_cruise(
                        cruise_dir,
                        True,
                        speed_boost=(option_boost or opt),
                    )
                else:
                    soft_stop()

            if mode == _HOLD:
                d = 0
                if move_l.pressed() and not move_r.pressed():
                    d = -1
                elif move_r.pressed() and not move_l.pressed():
                    d = 1
                if d == 0 or d != cruise_dir:
                    soft_stop()

            if moving and mode == _CRUISE and cruise_locked:
                if move_l.short_press:
                    if cruise_dir < 0:
                        soft_stop()
                    else:
                        start_selected_cruise(-1, True, speed_boost=bool(opt))
                elif move_r.short_press:
                    if cruise_dir > 0:
                        soft_stop()
                    else:
                        start_selected_cruise(1, True, speed_boost=bool(opt))

            if mode == _IDLE and not moving:
                if move_sem.short_release_latched and not st:
                    if allow_selected_move(move_sem.direction):
                        start_selected_cruise(
                            move_sem.direction,
                            True,
                            speed_boost=False,
                            accel_boost=bool(move_sem.boost),
                        )
                        dbg(3, "B4S cruise lock", move_sem.direction)

                if move_sem.hold_to_run and not st:
                    if allow_selected_move(move_sem.direction):
                        start_selected_cruise(
                            move_sem.direction,
                            False,
                            speed_boost=bool(opt),
                        )
                        dbg(3, "B4S cruise hold", move_sem.direction)

            sync_loop_led()
            await asyncio.sleep_ms(20)
    finally:
        try:
            mc.stop()
        except Exception:
            pass
        if enc_speed is not None:
            try:
                enc_speed.deinit()
            except Exception:
                pass
        if enc_accel is not None:
            try:
                enc_accel.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    run()
