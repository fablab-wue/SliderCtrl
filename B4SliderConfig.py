# B4Slider — panel configuration (shipped defaults).
#
# Users: copy SliderPins.example.py → SliderPins.py and edit that file only.
# SliderPins.B4Slider may override any key below (pins + behaviour).
#
#   MC_config.py / UIC_config.py — link / LED / OLED (shared)
#   this file — B4Slider panel pins + B4S_* behaviour

# ---------------------------------------------------------------------------
# GPIO — potentiometers (when B4S_SPEED_INPUT / B4S_ACCEL_INPUT is "pot")
# ---------------------------------------------------------------------------
PIN_POT_SPEED = 26        # ADC0 — SPEED
PIN_POT_ACCEL = 27        # ADC1 — ACCEL

# ---------------------------------------------------------------------------
# Discrete buttons (active-low, pull-ups). AXIS_4/5 on former MOVE_L2/R2 pads.
# ---------------------------------------------------------------------------
PIN_BTN_MOVE_L = 6
PIN_BTN_MOVE_R = 7
PIN_BTN_AXIS_1 = 12
PIN_BTN_AXIS_2 = 11
PIN_BTN_AXIS_3 = 10
PIN_BTN_AXIS_4 = 9
PIN_BTN_AXIS_5 = 8
PIN_BTN_AXIS_6 = 22
PIN_BTN_OPTION = 13
PIN_BTN_SET = 5           # was STOP on JKSlider discrete map

# ---------------------------------------------------------------------------
# Optional quadrature encoders (pin_b must be pin_a + 1)
# ---------------------------------------------------------------------------
PIN_ENC_SPEED_A = 14
PIN_ENC_SPEED_B = 15
PIN_ENC_ACCEL_A = 18
PIN_ENC_ACCEL_B = 19
B4S_QD_SPEED_SM = 2       # UIC NeoPixel is SM 1
B4S_QD_ACCEL_SM = 3
B4S_QD_DIV = 4
B4S_QD_ROUND = 2          # detents = (raw + 2) // 4

# ---------------------------------------------------------------------------
# Speed / accel input
# ---------------------------------------------------------------------------
# "pot" | "rotary"
B4S_SPEED_INPUT = "pot"
# "pot" | "rotary" | "set"  (SET-button L/H/learn)
B4S_ACCEL_INPUT = "pot"
# 1..4 linear, 11..14 log. Rotary boot value is vmax/8.
B4S_SPEED_QD_MODE = 11
B4S_ACCEL_QD_MODE = 11

# Legacy overlay: SliderPins.B4S_USE_ACCEL_POT without B4S_ACCEL_INPUT → set/pot.
B4S_USE_ACCEL_POT = 0

# ---------------------------------------------------------------------------
# Pots / feel
# ---------------------------------------------------------------------------
B4S_SPEED_DEADZONE = 0.02
B4S_SPEED_CURVE_GAMMA = 2.0
B4S_POT_OVERSAMPLE = 8
B4S_POT_EMA_ALPHA = 0.2
B4S_POT_HYST = 0.008
B4S_ACCEL_EMA_ALPHA = 0.15
B4S_ACCEL_HYST = 0.01

B4S_SPEED_MIN_MM_S = 1.0
B4S_SPEED_MAX_MM_S = 100.0
B4S_ACCEL_MIN_MM_S2 = 50.0
B4S_ACCEL_MAX_MM_S2 = 500.0

# Accel presets when B4S_ACCEL_INPUT is "set" (L=low, H=high — not A/B marks).
B4S_ACCEL_PRESET_L = 100.0
B4S_ACCEL_PRESET_H = 400.0

# ---------------------------------------------------------------------------
# Button timing
# ---------------------------------------------------------------------------
B4S_BTN_DEBOUNCE_MS = 10
B4S_LONG_PRESS_MS = 1000       # >1 s
B4S_EXTRA_LONG_MS = 3000       # >3 s
B4S_LEARN_HOLD_MS = 5000       # >5 s accel learn
# MOVE release ≤ this → locked cruise; longer → stop on release.
B4S_MOVE_TAP_MS = 333

# ---------------------------------------------------------------------------
# Motion / soft limits
# ---------------------------------------------------------------------------
B4S_LEFT_IS_NEGATIVE = True
B4S_LEFT2_IS_NEGATIVE = True
B4S_LEFT3_IS_NEGATIVE = True
B4S_LEFT4_IS_NEGATIVE = True
B4S_LEFT5_IS_NEGATIVE = True
B4S_LEFT6_IS_NEGATIVE = True
B4S_SOFT_FALLBACK_MIN = -2000.0
B4S_SOFT_FALLBACK_MAX = 2000.0
# Boot homing: motors 1..getMotorCount() (do not home servos).
B4S_HOMING_ENABLED = True
# Near soft-limit distance for UIC blue mix (mm). Overlay UIC SOFT_LIMIT_WARN_MM too.
B4S_NEAR_SOFT_MM = 3.0
# Loop +10% blue sticky add (0..255).
B4S_LOOP_BLUE_ADD = 26

# ---------------------------------------------------------------------------
# Boot / LED feedback
# ---------------------------------------------------------------------------
B4S_BOOT_UNLOCK = True
B4S_BOOT_TEXT = "B4Slider"
B4S_LED_FLASH_ON_MS = 80
B4S_LED_FLASH_OFF_MS = 80
B4S_LED_BLIP_MS = 120
B4S_LED_PINGPONG_MS = 600

# Optional board pin overlay (data-only SliderPins.py).
try:
    import SliderPins as _board_pins

    _ov = getattr(_board_pins, "B4Slider", None)
    if isinstance(_ov, dict):
        for _k, _v in _ov.items():
            globals()[_k] = _v
        if "B4S_ACCEL_INPUT" not in _ov and "B4S_USE_ACCEL_POT" in _ov:
            B4S_ACCEL_INPUT = (
                "pot" if int(_ov["B4S_USE_ACCEL_POT"]) else "set"
            )
except ImportError:
    pass
