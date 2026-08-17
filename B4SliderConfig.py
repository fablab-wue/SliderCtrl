# B4Slider — 4-button panel configuration (shipped defaults).
#
# Users: copy SliderPins.example.py → SliderPins.py and edit that file only.
# SliderPins.B4Slider may override any key below (pins + behaviour).
#
#   MC_config.py / UIC_config.py — link / LED / OLED (shared)
#   this file — B4Slider panel pins + B4S_* behaviour

# ---------------------------------------------------------------------------
# GPIO — potentiometers
# ---------------------------------------------------------------------------
PIN_POT_SPEED = 26        # ADC0 — SPEED
PIN_POT_ACCEL = 27        # ADC1 — ACCEL (only if B4S_USE_ACCEL_POT)

# ---------------------------------------------------------------------------
# Discrete buttons (active-low, pull-ups)
# ---------------------------------------------------------------------------
PIN_BTN_MOVE_L = 6
PIN_BTN_MOVE_R = 7
PIN_BTN_OPTION = 13
PIN_BTN_SET = 5           # was STOP on JKSlider discrete map

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

# Accel presets when no ACCEL pot (L=low, H=high — not A/B marks).
B4S_ACCEL_PRESET_L = 100.0
B4S_ACCEL_PRESET_H = 400.0

# 1 = use ACCEL pot (JKSlider-style); disables SET-alone Hold>N accel gestures.
B4S_USE_ACCEL_POT = 0

# ---------------------------------------------------------------------------
# Button timing
# ---------------------------------------------------------------------------
B4S_BTN_DEBOUNCE_MS = 30
B4S_LONG_PRESS_MS = 1000       # >1 s
B4S_EXTRA_LONG_MS = 3000       # >3 s
B4S_LEARN_HOLD_MS = 5000       # >5 s accel learn
# MOVE release ≤ this → locked cruise; longer → stop on release.
B4S_MOVE_TAP_MS = 333

# ---------------------------------------------------------------------------
# Motion / soft limits
# ---------------------------------------------------------------------------
B4S_LEFT_IS_NEGATIVE = True
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
except ImportError:
    pass
