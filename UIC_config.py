# UIC_config — shipped defaults for UIC_Base (OLED, LED, camera, WDT, debug).
#
# Users: copy SliderPins.example.py → SliderPins.py and edit that file only.
# This module holds versioned defaults; SliderPins.UIC_config may override any key.
#
# App panel pins / JKS_* live in JKSliderConfig.py (also overridable via SliderPins).

# ---------------------------------------------------------------------------
# GPIO — RGB status LED
# ---------------------------------------------------------------------------
# Common housing; polarity via LED_ACTIVE_HIGH.
PIN_LED_R = 2
PIN_LED_G = 3
PIN_LED_B = 4

# Optional single WS2812 NeoPixel (same colours as RGB). None = disabled.
# Uses PIO state machine PIO_NEOPIXEL_SM_ID only (motor STEP PIO is on SliderMC).
PIN_NEOPIXEL = None
PIO_NEOPIXEL_SM_ID = 1

# ---------------------------------------------------------------------------
# GPIO — camera shutter / intervalometer
# ---------------------------------------------------------------------------
# Active-high into optocoupler LED by default.
PIN_CTRL_CAMERA = 22
CTRL_CAMERA_PULSE_MS = 100
CTRL_CAMERA_ACTIVE_HIGH = True

# ---------------------------------------------------------------------------
# OLED 128x64 over I2C (set DSP_ENABLED = False to skip)
# ---------------------------------------------------------------------------
# Driver: "ssd1306" (0.96"), "sh1106" (1.3"; also CH1115/CH1116, SSH1106),
# or "ssd1309" (1.54" / 2.42"). Same UI layout for all.
DSP_ENABLED = True
DSP_DRIVER = "ssd1306"
# True when the module is mounted 180° vs default panel artwork.
DSP_ROTATE_180 = False
PIN_DSP_I2C_SDA = 0
PIN_DSP_I2C_SCL = 1
DSP_I2C_ID = 0
DSP_I2C_ADDR = 0x3C
DSP_I2C_FREQ = 400_000
DSP_WIDTH = 128
DSP_HEIGHT = 64
DSP_UPDATE_MS = 250
# During motion, refresh Spd/Acc every DSP_UPDATE_MS.
# True: also refresh Pos (more I2C). False: freeze Pos until standstill.
DSP_LIVE_POS = True
# OLED contrast (0..255). Dim mode multiplies by panel luminosity scale.
DSP_CONTRAST_FULL = 0xFF

# ---------------------------------------------------------------------------
# RGB LED polarity and blink timing
# ---------------------------------------------------------------------------
LED_ACTIVE_HIGH = True
LED_BLINK_MS = 250             # red blink while homing
LED_BLINK_HARD_LIMIT_MS = 80   # red on/off half-period on hard limit
LED_BLINK_DELAY_WAIT_MS = 500  # cyan blink half-period during Delay wait (1 Hz); apps use 2× as ping-pong period
LED_DIM_WHITE = 0.12           # duty (0..1) for driver enabled idle white
LED_DIM_ORANGE = 0.12          # duty for driver disabled
LED_DIM_CYAN = 0.12            # duty for Delay armed (JKSlider panel ledPingPong)
LED_DIM_MAGENTA = 0.12         # duty for timelapse idle (JKSlider panel ledPingPong)
LED_RAINBOW_MS = 1000          # boot rainbow duration (ms)
# Soft-limit blue mix adds (0..255). Docs: near ~30%, at 100%.
LED_SOFT_NEAR_BLUE_ADD = 76
LED_SOFT_AT_BLUE_ADD = 255
# Distance from soft limit that triggers near-soft blue mix (mm).
SOFT_LIMIT_WARN_MM = 10.0
# |cmd − actual| above this (mm/s) counts as accelerating / decelerating
# when MC status letter is not A/B (fallback for LED yellow).
LED_ACCEL_SPEED_EPS_MM_S = 3.0

# ---------------------------------------------------------------------------
# Watchdog + Pico onboard LED heartbeat
# ---------------------------------------------------------------------------
# Hardware WDT resets the Pico if the asyncio I/O monitor stops feeding it.
# Onboard LED toggles at WDT_HEARTBEAT_MS (1 Hz = alive).
WDT_ENABLED = True
WDT_TIMEOUT_MS = 3000  # RP2040 max ≈ 8388; must exceed heartbeat period
WDT_HEARTBEAT_MS = 1000  # feed + LED toggle period (1 Hz)
# MicroPython Pico / Pico W alias ("LED"); classic Pico is also GP25.
PIN_LED_ONBOARD = "LED"

# ---------------------------------------------------------------------------
# USB debug output (CDC serial) — short expert tokens, CRLF-prefixed
# ---------------------------------------------------------------------------
# 0 = off (also reserved for future USB CLI)
# 1 = errors (hard limits, init failures)
# 2 = warnings (soft limits, aborted homing, refused actions)
# 3 = info (buttons/keys, main status) — default
# 4 = debug (L3 + USB "." heartbeat + ~2 Hz motion telemetry)
# 5 = detailed (L4 + MSM hop/timing chatter)
DEBUG_LEVEL = 3

# Optional full overlay (data-only SliderPins.py). Missing = keep defaults.
try:
    import SliderPins as _board_pins
    _ov = getattr(_board_pins, "UIC_config", None)
    if isinstance(_ov, dict):
        for _k, _v in _ov.items():
            globals()[_k] = _v
except ImportError:
    pass
