# MC_config — shipped defaults for MC_Client (UART link to SliderMC).
#
# Users: copy SliderPins.example.py → SliderPins.py and edit that file only.
# This module holds versioned defaults; SliderPins.MC_config may override any key.
#
# Axis mechanics / max_speed / slider_min/max live on SliderMC (read via CG).

# ---------------------------------------------------------------------------
# GPIO — UART to SliderMC
# ---------------------------------------------------------------------------
# UART0 @ 1 Mbaud. Remap via SliderPins if needed.
PIN_UART_TX = 16
PIN_UART_RX = 17
UART_BAUD = 1_000_000

# ---------------------------------------------------------------------------
# Motion floor (UIC command clamp; planner ceilings live on MC)
# ---------------------------------------------------------------------------
# Minimum cruise / command speed (mm/s). Avoids zero-division / dead SS.
MIN_SPEED_MM_S = 0.006
# Distance from soft limit for isNearSoftLimit() (mm). UIC LED uses UIC_config copy.
SOFT_LIMIT_WARN_MM = 10.0
# |Δspeed| above this (mm/s) counts as accel/decel when MC letter is not A/B.
LED_ACCEL_SPEED_EPS_MM_S = 3.0

# Optional full overlay (data-only SliderPins.py). Missing = keep defaults.
try:
    import SliderPins as _board_pins
    _ov = getattr(_board_pins, "MC_config", None)
    if isinstance(_ov, dict):
        for _k, _v in _ov.items():
            globals()[_k] = _v
except ImportError:
    pass
