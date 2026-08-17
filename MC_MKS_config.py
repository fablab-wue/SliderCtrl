# MC_MKS_config — shipped defaults for MC_MKS_Client (RS485 to MKS SERVO42D/57D).
#
# Alternate axis backend: no SliderMC. Users overlay via SliderPins.MC_MKS_config.
# Native FA/FB protocol only (not Modbus-RTU).

# ---------------------------------------------------------------------------
# GPIO — UART + MAX485 DE/RE (tied together)
# ---------------------------------------------------------------------------
# Same UART0 pins as MC_Client; baud is RS485-typical (motor menu UartBaud).
PIN_UART_TX = 16
PIN_UART_RX = 17
PIN_RS485_DE = 18  # MAX485 DE + RE (active high = drive bus)
UART_ID = 0
UART_BAUD = 38400
MKS_ADDR = 1

# Post-TX settle before releasing DE (ms). Tune if replies are truncated.
RS485_TX_HOLD_MS = 2
# Per-command reply wait (ms).
RS485_RX_TIMEOUT_MS = 80

# ---------------------------------------------------------------------------
# Mechanics (UIC owns mm)
# ---------------------------------------------------------------------------
# Millimetres of travel per motor revolution (lead / belt pitch × teeth).
MM_PER_ROT = 5.0
# Encoder axis counts per revolution (MKS 14-bit: 0x4000).
AXIS_PER_ROT = 0x4000

# Soft travel limits (mm). None = open that side.
SLIDER_MIN = 0.0
SLIDER_MAX = 600.0

MAX_SPEED_MM_S = 100.0
MAX_ACCEL_MM_S2 = 500.0
MIN_SPEED_MM_S = 0.006
SOFT_LIMIT_WARN_MM = 10.0
LED_ACCEL_SPEED_EPS_MM_S = 3.0

# Session init (used until setSpeed / setAcceleration)
INIT_SPEED_MM_S = 50.0
INIT_ACCEL_MM_S2 = 200.0

# Status push to UIC_Base callback
STATUS_HZ = 5.0

# ---------------------------------------------------------------------------
# Homing (optional) — motor endstop / GoHome 91H
# ---------------------------------------------------------------------------
HOME_USE = 0  # 1 = enable MH-equivalent home()
HOME_DIR = 0  # 0=CW, 1=CCW (MKS HmDir)
HOME_SPEED_MM_S = 25.0
HOME_TRIG_LEVEL = 0  # 0=Low, 1=High
# After origin home, UIC position = this mm (typically SLIDER_MIN).
HOME_SET_POS_MM = 0.0

# ---------------------------------------------------------------------------
# Hard limits on motor (optional) — EndLimit via 90H
# ---------------------------------------------------------------------------
HARD_LIMIT_USE = 0  # 1 = enable EndLimit
LIMIT_REMAP = 0  # 1 = 9EH remap En/Dir as L/R (needed for 42D dual limits)

# ---------------------------------------------------------------------------
# Accel byte map (simple linear UI mm/s² → MKS acc 1..255)
# ---------------------------------------------------------------------------
# When commanded accel == ACCEL_MM_S2_FOR_ACC_MAX, MKS acc = MKS_ACC_MAX.
MKS_ACC_MIN = 1
MKS_ACC_MAX = 200
ACCEL_MM_S2_FOR_ACC_MAX = 500.0

# Optional full overlay (data-only SliderPins.py). Missing = keep defaults.
try:
    import SliderPins as _board_pins

    _ov = getattr(_board_pins, "MC_MKS_config", None)
    if isinstance(_ov, dict):
        for _k, _v in _ov.items():
            globals()[_k] = _v
except ImportError:
    pass
