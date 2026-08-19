# B4Slider — 4-button camera slider panel (MOVE_L/R, OPTION, SET + SPEED pot).
#
# Soft limits are the A/B working window. Reuses MC_Client + UIC_Base.
# Config: B4SliderConfig.py (B4S_*); overlay via SliderPins.B4Slider.
#
# Usage:
#   import B4Slider
#   B4Slider.run()

import time

import uasyncio as asyncio
from machine import ADC, Pin

import MC_config as config
import B4SliderConfig as b4s
from MC_client import MC_Client
from UIC_base import UIC_Base, dbg
from button_state import ButtonAdapter, allow_move_out_of_soft_limit, resolve_move_semantics

_IDLE = 0
_CRUISE = 1
_HOLD = 2

_WHITE = (255, 255, 255)
_VIOLET = (180, 0, 255)
_RED = (255, 0, 0)
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
    mc.set_status_callback(ui.on_status)
    await mc.start()
    await ui.start()

    # Near soft distance for shared UIC mix
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

    def update_all():
        move_l.update()
        move_r.update()
        btn_option.update()
        btn_set.update()

    if getattr(b4s, "B4S_BOOT_UNLOCK", True):
        await _wait_boot_unlock(ui, btn_option, update_all)

    # Panel ceilings
    speed_max = float(b4s.B4S_SPEED_MAX_MM_S)
    accel_max = float(b4s.B4S_ACCEL_MAX_MM_S2)
    if mc.max_speed is not None and mc.max_speed < speed_max:
        speed_max = float(mc.max_speed)
    if mc.max_accel is not None and mc.max_accel < accel_max:
        accel_max = float(mc.max_accel)
    mc.max_speed = speed_max
    mc.max_accel = accel_max
    mc._max_speed_mm_s = speed_max

    slider_min = mc.slider_min if mc.slider_min is not None else 0.0
    slider_max = mc.slider_max if mc.slider_max is not None else 600.0
    soft_l = float(slider_min)
    soft_r = float(slider_max)

    def apply_soft_limits():
        nonlocal soft_l, soft_r
        if soft_l > soft_r:
            soft_l, soft_r = soft_r, soft_l
        mc.setSoftLimits(soft_l, soft_r)
        ui.set_soft_limits(soft_l, soft_r)

    apply_soft_limits()

    use_accel_pot = int(getattr(b4s, "B4S_USE_ACCEL_POT", 0)) != 0
    accel_preset = "L"
    accel_l = float(b4s.B4S_ACCEL_PRESET_L)
    accel_h = float(b4s.B4S_ACCEL_PRESET_H)
    accel_cmd = accel_l

    loop_armed = False
    mode = _IDLE
    cruise_dir = 0  # -1 L, +1 R
    cruise_locked = False
    option_boost = False
    set_tick_sec = 0
    learn_active = False
    swap_lr = False
    move_swap_since = None
    move_swap_fired = False

    adc_speed = ADC(Pin(b4s.PIN_POT_SPEED))
    filt_speed = _PotFilter(
        b4s.B4S_POT_OVERSAMPLE, b4s.B4S_POT_EMA_ALPHA, b4s.B4S_POT_HYST
    )
    adc_accel = None
    filt_accel = None
    if use_accel_pot:
        adc_accel = ADC(Pin(b4s.PIN_POT_ACCEL))
        filt_accel = _PotFilter(
            b4s.B4S_POT_OVERSAMPLE,
            b4s.B4S_ACCEL_EMA_ALPHA,
            b4s.B4S_ACCEL_HYST,
        )

    pot_min = float(b4s.B4S_SPEED_MIN_MM_S)
    accel_lo = float(b4s.B4S_ACCEL_MIN_MM_S2)

    left_neg = bool(b4s.B4S_LEFT_IS_NEGATIVE)
    loop_blue = int(getattr(b4s, "B4S_LOOP_BLUE_ADD", 26))
    flash_on = int(b4s.B4S_LED_FLASH_ON_MS)
    flash_off = int(b4s.B4S_LED_FLASH_OFF_MS)
    blip_ms = int(b4s.B4S_LED_BLIP_MS)
    ping_ms = int(b4s.B4S_LED_PINGPONG_MS)
    driver_enabled = True

    def target_for_dir(direction):
        # direction: -1 toward soft_l (left), +1 toward soft_r (right)
        if left_neg:
            return soft_l if direction < 0 else soft_r
        return soft_r if direction < 0 else soft_l

    def dir_from_buttons():
        if move_l.pressed() and not move_r.pressed():
            return -1
        if move_r.pressed() and not move_l.pressed():
            return 1
        return 0

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

    def power_up_reset():
        nonlocal soft_l, soft_r, loop_armed, mode, cruise_dir
        nonlocal cruise_locked, accel_preset, accel_cmd, option_boost
        nonlocal driver_enabled
        mc.halt()
        soft_l = float(slider_min)
        soft_r = float(slider_max)
        apply_soft_limits()
        loop_armed = False
        mode = _IDLE
        cruise_dir = 0
        cruise_locked = False
        option_boost = False
        driver_enabled = False
        accel_preset = "L"
        accel_cmd = accel_l
        if not use_accel_pot:
            mc.setAcceleration(accel_cmd)
        ui.set_enabled(False)
        ui.ledClearAdd()
        ui.ledEffectClear()
        ui.ledFlash(_RED, 2, flash_on, flash_off)
        dbg(3, "B4S all-four reset")

    def start_cruise(direction, locked, speed_boost=False, accel_boost=False):
        nonlocal mode, cruise_dir, cruise_locked, option_boost
        cruise_dir = direction
        cruise_locked = locked
        option_boost = speed_boost
        mode = _CRUISE if locked else _HOLD
        tgt = target_for_dir(direction)
        spd = speed_max if speed_boost else speed_mm_s
        acc = accel_max if accel_boost else accel_cmd
        mc.setSpeed(spd)
        mc.setAcceleration(acc)
        mc.enable(True)
        mc.moveTo(tgt)
        sync_loop_led()

    def soft_stop():
        nonlocal mode, cruise_dir, cruise_locked, option_boost
        mc.stop()
        mode = _IDLE
        cruise_dir = 0
        cruise_locked = False
        option_boost = False
        sync_loop_led()

    mc.enable(True)
    if not use_accel_pot:
        mc.setAcceleration(accel_cmd)
    ui.set_commanded(speed_mm_s=pot_min, accel_mm_s2=accel_cmd)
    dbg(3, "B4Slider ready", "soft", soft_l, soft_r)

    try:
        while True:
            update_all()

            speed_mm_s = _read_speed(filt_speed, adc_speed, pot_min, speed_max)
            if use_accel_pot:
                accel_cmd = _read_accel(
                    filt_accel, adc_accel, accel_lo, accel_max
                )
            ui.set_commanded(speed_mm_s=speed_mm_s, accel_mm_s2=accel_cmd)

            opt = btn_option.pressed()
            st = btn_set.pressed()
            move_sem = resolve_move_semantics(
                move_l.state, move_r.state, opt, move_tap_ms
            )
            all_four = (
                move_l.pressed()
                and move_r.pressed()
                and opt
                and st
            )
            lr_halt = move_l.pressed() and move_r.pressed() and not all_four

            # --- 1) All four: panic + power-up reset -----------------------
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

            # --- 2) L+R halt ----------------------------------------------
            if lr_halt:
                if move_l.edge_press or move_r.edge_press:
                    mc.halt()
                    soft_stop()
                    driver_enabled = False
                    ui.set_enabled(False)
                    ui.ledFlash(_RED, 2, flash_on, flash_off)
                    dbg(3, "B4S halt")
                await asyncio.sleep_ms(20)
                continue

            # Re-enable on any button while disabled
            if not driver_enabled:
                if (
                    move_l.edge_press
                    or move_r.edge_press
                    or btn_option.edge_press
                    or btn_set.edge_press
                ):
                    mc.enable(True)
                    driver_enabled = True
                    ui.set_enabled(True)
                    dbg(3, "B4S enable")
                await asyncio.sleep_ms(20)
                continue

            # --- 3) OPTION+SET (no MOVE): hold>1s loop; short release disable
            if opt and st and not move_l.pressed() and not move_r.pressed():
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

            # --- MOVE_L + MOVE_R ≥ 3 s → swap direction, like FAST pair ----
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

            # --- 4/5) SET + MOVE soft limits ------------------------------
            if st and (move_l.pressed() or move_r.pressed()) and not opt:
                if move_l.pressed() and move_r.pressed():
                    if move_l.long_press or move_r.long_press or btn_set.long_press:
                        soft_l = float(slider_min)
                        soft_r = float(slider_max)
                        apply_soft_limits()
                        ui.ledBlip(_WHITE, blip_ms)
                        dbg(3, "B4S reset both soft")
                elif move_l.pressed():
                    if move_l.long_press or btn_set.long_press:
                        soft_l = float(slider_min)
                        apply_soft_limits()
                        ui.ledBlip(_WHITE, blip_ms)
                        dbg(3, "B4S reset soft_l")
                elif move_r.pressed():
                    if move_r.long_press or btn_set.long_press:
                        soft_r = float(slider_max)
                        apply_soft_limits()
                        ui.ledBlip(_WHITE, blip_ms)
                        dbg(3, "B4S reset soft_r")
                await asyncio.sleep_ms(20)
                continue

            # SET+MOVE short: set limit on release of chord
            if st and not opt:
                if move_l.short_press and btn_set.last_hold_ms < long_ms:
                    soft_l = mc.getPosition()
                    apply_soft_limits()
                    ui.ledBlip(_WHITE, blip_ms)
                    dbg(3, "B4S set soft_l", soft_l)
                    await asyncio.sleep_ms(20)
                    continue
                if move_r.short_press and btn_set.last_hold_ms < long_ms:
                    soft_r = mc.getPosition()
                    apply_soft_limits()
                    ui.ledBlip(_WHITE, blip_ms)
                    dbg(3, "B4S set soft_r", soft_r)
                    await asyncio.sleep_ms(20)
                    continue

            # --- SET alone ------------------------------------------------
            if st and not opt and not move_l.pressed() and not move_r.pressed():
                if mc.isMoving() or mode != _IDLE:
                    if btn_set.edge_press:
                        soft_stop()
                        dbg(3, "B4S SET stop")
                elif not use_accel_pot:
                    sec = btn_set.hold_ms() // 1000
                    if btn_set.pressed() and sec > set_tick_sec:
                        set_tick_sec = sec
                        ui.ledFlash(_WHITE, 1, flash_on, flash_off)
                    if not btn_set.pressed():
                        set_tick_sec = 0

                    if btn_set.learn_press:
                        learn_active = True
                        dbg(3, "B4S accel learn…")
                    if learn_active and btn_set.pressed():
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
                        accel_preset = "H"
                        accel_cmd = accel_h
                        mc.setAcceleration(accel_cmd)
                        ui.ledFlash(_VIOLET, 2, flash_on, flash_off)
                        dbg(3, "B4S accel H", accel_cmd)
                    elif btn_set.long_press:
                        accel_preset = "L"
                        accel_cmd = accel_l
                        mc.setAcceleration(accel_cmd)
                        ui.ledFlash(_VIOLET, 1, flash_on, flash_off)
                        dbg(3, "B4S accel L", accel_cmd)
                await asyncio.sleep_ms(20)
                continue

            # --- Motion ---------------------------------------------------
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
                if use_accel_pot:
                    mc.setAcceleration(accel_cmd)

            if mode == _CRUISE and cruise_locked and not mc.isMoving():
                if loop_armed:
                    cruise_dir = -cruise_dir
                    start_cruise(
                        cruise_dir,
                        True,
                        speed_boost=(option_boost or opt),
                        accel_boost=False,
                    )
                else:
                    soft_stop()

            if mode == _HOLD:
                d = dir_from_buttons()
                if d == 0 or d != cruise_dir:
                    soft_stop()

            if moving and mode == _CRUISE and cruise_locked:
                if move_l.short_press:
                    if cruise_dir < 0:
                        soft_stop()
                    else:
                        start_cruise(-1, True, opt)
                elif move_r.short_press:
                    if cruise_dir > 0:
                        soft_stop()
                    else:
                        start_cruise(1, True, opt)

            if mode == _IDLE and not moving:
                if move_sem.short_release_latched and not st:
                    if allow_move_out_of_soft_limit(
                        mc.getPosition(), move_sem.direction, soft_l, soft_r
                    ) or not mc.isAtSoftLimit():
                        start_cruise(
                            move_sem.direction,
                            True,
                            speed_boost=False,
                            accel_boost=bool(move_sem.boost),
                        )
                        dbg(
                            3,
                            "B4S cruise lock",
                            move_sem.direction,
                            "boost",
                            move_sem.boost,
                        )
                if move_sem.hold_to_run and not st:
                    if allow_move_out_of_soft_limit(
                        mc.getPosition(), move_sem.direction, soft_l, soft_r
                    ) or not mc.isAtSoftLimit():
                        start_cruise(
                            move_sem.direction,
                            False,
                            speed_boost=bool(opt),
                            accel_boost=False,
                        )
                        dbg(3, "B4S cruise hold", move_sem.direction)

            sync_loop_led()
            await asyncio.sleep_ms(20)
    finally:
        try:
            mc.stop()
        except Exception:
            pass


if __name__ == "__main__":
    run()
