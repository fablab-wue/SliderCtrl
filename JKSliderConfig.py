# JKSlider — application / panel configuration (shipped defaults).
#
# Users: copy SliderPins.example.py → SliderPins.py and edit that file only.
# SliderPins.JKSlider may override any key below (pins + behaviour).
#
#   MC_config.py   — UART / MIN_SPEED (MC_Client)
#   UIC_config.py  — OLED / LED / camera / WDT (UIC_Base)
#   this file      — panel pins + JKSlider behaviour
#
# SPEED pot full scale and soft travel limits come from MC via MC_Client
# (max_speed, slider_min, slider_max). After CG, JKSlider clamps
# mc.max_speed / mc.max_accel with JKS_SPEED_MAX_MM_S /
# JKS_ACCEL_MAX_MM_S2 (panel ceilings ≤ MC).

# ---------------------------------------------------------------------------
# GPIO — potentiometers (always)
# ---------------------------------------------------------------------------
PIN_POT_SPEED = 26        # ADC0 — SPEED (left=min floor, right=MC max_speed)
PIN_POT_ACCEL = 27        # ADC1 — ACCEL (left=min, right=local/MC max)
PIN_POT_JOYSTICK = None   # Optional centre-return stick (e.g. 28 = ADC2). None = off.

# ---------------------------------------------------------------------------
# Input mode
# ---------------------------------------------------------------------------
# "button" = one GPIO per switch (active-low, pull-ups).
# "keypad" = 4x3 matrix on PIN_KEYPAD_ROWS / PIN_KEYPAD_COLS;
#            discrete BTN_STOP (GP5) and BTN_OPTION (GP13) still ORed in.
JKS_INPUT_MODE = "keypad"

# ---------------------------------------------------------------------------
# Discrete buttons (always available)
# ---------------------------------------------------------------------------
# In keypad mode: ORed with matrix STOP / OPTION after ghost filter.
PIN_BTN_STOP = 5
PIN_BTN_OPTION = 13     # modifier (hold + other control); alone does nothing

# Discrete BTN_* — used when JKS_INPUT_MODE == "button"
PIN_BTN_MOVE_L = 6
PIN_BTN_MOVE_R = 7
PIN_BTN_FAST_L = 8
PIN_BTN_FAST_R = 9
PIN_BTN_A = 10
PIN_BTN_B = 11
PIN_BTN_C = 12
PIN_BTN_DELAY = 14      # Optional: hold N s → delay; short → delay off
PIN_BTN_TIMELAPSE = 15  # Optional: tap → TL divider; long → divider 1

# ---------------------------------------------------------------------------
# Keypad matrix — used when JKS_INPUT_MODE == "keypad"
# ---------------------------------------------------------------------------
# Rows GP6..GP9 = KP_ROW1..KP_ROW4 (KP_ROW1 = upper keys on GP6).
# Cols GP10..GP12 = KP_COL1..KP_COL3.
# High-Z row scan (no row diodes). See manuals/JKSlider_Technical_Manual_Panel.md.
PIN_KEYPAD_ROWS = (6, 7, 8, 9)
PIN_KEYPAD_COLS = (10, 11, 12)

# ---------------------------------------------------------------------------
# Joystick / pot feel
# ---------------------------------------------------------------------------
# Centre deadzone for POT_JOYSTICK (fraction of full-scale deflection).
JOYSTICK_DEADZONE = 0.08
# Low end of SPEED / ACCEL pots treated as zero (fraction of full scale).
JKS_SPEED_DEADZONE = 0.02
# SPEED pot response curve: 1.0 = linear, >1.0 = finer control at low speed.
JKS_SPEED_CURVE_GAMMA = 2.0
# Joystick deflection curve after deadzone (1.0 = linear, >1.0 = finer near centre).
JKS_JOYSTICK_CURVE_GAMMA = 2.0
# Pot ADC denoise: oversample count, EMA alpha (0..1), publish hysteresis (FS fraction).
JKS_POT_OVERSAMPLE = 8
JKS_POT_EMA_ALPHA = 0.2
JKS_POT_HYST = 0.008
JKS_ACCEL_EMA_ALPHA = 0.15
JKS_ACCEL_HYST = 0.01
JKS_JOY_OVERSAMPLE = 8
JKS_JOY_EMA_ALPHA = 0.3
JKS_JOY_HYST = 0.006

# ---------------------------------------------------------------------------
# Speed / accel ranges (panel)
# ---------------------------------------------------------------------------
# SPEED pot maps JKS_SPEED_MIN_MM_S .. slider.max_speed (after clamp below).
JKS_SPEED_MIN_MM_S = 1.0
# Panel ceiling for slider.max_speed (mm/s): min(MC max_speed, this).
JKS_SPEED_MAX_MM_S = 100.0
# ACCEL pot floor (mm/s²).
JKS_ACCEL_MIN_MM_S2 = 50.0
# Panel ceiling for slider.max_accel (mm/s²): min(MC max_accel, this).
JKS_ACCEL_MAX_MM_S2 = 500.0

# ---------------------------------------------------------------------------
# Button timing
# ---------------------------------------------------------------------------
JKS_LONG_PRESS_MS = 1000       # store PosA / PosB / PosC
JKS_STOP_HALT_MS = 1000        # STOP hold → halt() with EMO deceleration
JKS_STOP_DISABLE_MS = 2000     # STOP hold → disable driver
JKS_BTN_DEBOUNCE_MS = 30
# MOVE release ≤ this → locked cruise; longer hold → stop on release.
JKS_MOVE_TAP_MS = 333

# ---------------------------------------------------------------------------
# Motion behaviour (panel)
# ---------------------------------------------------------------------------
# Left buttons move toward decreasing position when True.
JKS_LEFT_IS_NEGATIVE = True
# Swap L/R meaning for MOVE/FAST and joystick (handedness).
# Runtime toggle: hold FAST_L + FAST_R ≥ 1 s.
JKS_SWAP_LR = False
# Pause at each loop endpoint before reversing (ms). 0 = no dwell.
JKS_LOOP_DWELL_MS = 1000

# ---------------------------------------------------------------------------
# Delay / OPTION modifiers
# ---------------------------------------------------------------------------
JKS_DELAY_CLEAR_MS = 300
JKS_DELAY_MAX_S = 30.0
# OPTION + DELAY: multiply held time by this factor when arming Delay.
JKS_OPTION_DELAY_SCALE = 5.0
# OPTION + DELAY short tap: arm this fixed delay (seconds).
JKS_OPTION_DELAY_PRESET_S = 5.0
# OPTION + TIMELAPSE tap: increment divider by 1 up to this value.
JKS_OPTION_TL_MAX = 100
# OPTION + TIMELAPSE hold: jump to this favourite divider.
JKS_OPTION_TL_FAVORITE = 25
# OPTION + FAST_L + FAST_R hold: toggle LED/OLED to this scale (vs 1.0 full).
JKS_DISPLAY_DIM = 0.25

# ---------------------------------------------------------------------------
# Positions / marks
# ---------------------------------------------------------------------------
# Keep stored PosA/PosB/PosC this far inside soft limits (mm).
JKS_STORE_MARGIN_MM = 3.0
# Treat carriage as "at" PosA/B/C within this distance (mm).
JKS_AT_MARK_MM = 0.5
# File on the Pico:
# PosA,PosB,PosC[,tl_div,swap_lr,delay_s,joy_center,camera_fps[,tl_mode]]
# tl_mode: "msm" | "continuous" (UI: Cont). Cont = ÷N crawl + CTRL_CAMERA hold-high.
JKS_POSITIONS_FILE = "jks_positions.txt"

# ---------------------------------------------------------------------------
# Timelapse / camera
# ---------------------------------------------------------------------------
# Camera shutter FPS for CTRL_CAMERA intervalometer (MSM: period = tl_div / fps).
JKS_CAMERA_FPS = 30
JKS_CAMERA_FPS_STEPS = (24, 25, 30, 48, 50, 60)
# Default TL≠1 style when file has no tl_mode: "msm" or "continuous" (Cont).
# Runtime toggled with T+D+OPTION and saved to JKS_POSITIONS_FILE.
JKS_TL_MODE = "msm"
# MSM: wait after shutter pulse before moving (ms).
JKS_MSM_EXPOSURE_MS = 200
# MSM: wait after hop settles before next shoot (ms).
JKS_MSM_SETTLE_MS = 50
# MSM: smallest planned hop (mm); refuse start if even this overruns the interval.
JKS_MSM_MIN_STEP_MM = 0.1

# ---------------------------------------------------------------------------
# Display / boot
# ---------------------------------------------------------------------------
# Display extra-line rotate period when messages do not all fit (ms).
JKS_DSP_EXTRA_ROTATE_MS = 1000
# Display Pos/Spd/Acc* unit: "mm" or "inch". Internals stay millimetres.
JKS_DSP_UNIT = "mm"
# Boot splash text (OLED app line).
JKS_BOOT_TEXT = "JKSlider V1 by JK"
JKS_BOOT_SPLASH_MS = 2000
# Require OPTION or STOP before first enable/homing (security). Rainbow while locked.
JKS_BOOT_UNLOCK = True
# True: home at boot and via STOP+A. False: treat power-up position as 0; SW_HOME still hard-limits.
JKS_HOMING_ENABLED = True

# Status LED panel effects (UIC_Base ledPingPong / ledAddColor / ledFlash)
JKS_LOOP_BLUE_ADD = 26          # ~10% blue while AB/AC/BC loop is moving (0…255)
JKS_LED_PINGPONG_MS = 600       # loop-idle white↔blue period
JKS_LED_FLASH_ON_MS = 80
JKS_LED_FLASH_OFF_MS = 80

# Optional board pin overlay (data-only SliderPins.py). Missing = keep defaults.
try:
    import SliderPins as _board_pins
    _ov = getattr(_board_pins, "JKSlider", None)
    if isinstance(_ov, dict):
        for _k, _v in _ov.items():
            globals()[_k] = _v
except ImportError:
    pass
