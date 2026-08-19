# SliderPins.example.py — copy to SliderPins.py on the UIC for your hardware.
#
# ONE file per slider HW. Edit this file only (pins + behaviour overrides).
# Data only (no functions). Shipped MC_config.py / UIC_config.py /
# JKSliderConfig.py hold defaults; missing keys keep those defaults.
# If SliderPins.py is absent, built-in defaults are used.
#
# Later apps can add another dict here (e.g. OtherApp = { ... }) and cherry-pick
# it from their own *Config.py the same way JKSlider does.

# ---------------------------------------------------------------------------
# MC_Client defaults — consumed by MC_config.py
# ---------------------------------------------------------------------------
MC_config = {
    # GPIO — UART to SliderMC @ 1 Mbaud (UART0)
    "PIN_UART_TX": 16,
    "PIN_UART_RX": 17,
    "UART_BAUD": 1_000_000,
    # UIC command floor (mm/s); planner ceilings live on MC
    "MIN_SPEED_MM_S": 0.006,
    "SOFT_LIMIT_WARN_MM": 10.0,
    "LED_ACCEL_SPEED_EPS_MM_S": 3.0,
}

# ---------------------------------------------------------------------------
# MC_MKS_Client — alternate RS485 axis (no SliderMC). Consumed by MC_MKS_config.py
# Use instead of MC_Client when driving MKS SERVO42D/57D via MAX485.
# Shares GP16/17 UART0; adds DE on GP18. Do not run both clients at once.
# ---------------------------------------------------------------------------
MC_MKS_config = {
    "PIN_UART_TX": 16,
    "PIN_UART_RX": 17,
    "PIN_RS485_DE": 18,
    "UART_ID": 0,
    "UART_BAUD": 38400,
    "MKS_ADDR": 1,
    "MM_PER_ROT": 5.0,
    "SLIDER_MIN": 0.0,
    "SLIDER_MAX": 600.0,
    "MAX_SPEED_MM_S": 100.0,
    "MAX_ACCEL_MM_S2": 500.0,
    "INIT_SPEED_MM_S": 50.0,
    "INIT_ACCEL_MM_S2": 200.0,
    "STATUS_HZ": 5.0,
    "HOME_USE": 0,
    "HARD_LIMIT_USE": 0,
    "LIMIT_REMAP": 0,
}

# ---------------------------------------------------------------------------
# UIC_Base defaults — consumed by UIC_config.py
# ---------------------------------------------------------------------------
UIC_config = {
    # GPIO — RGB status LED (polarity: LED_ACTIVE_HIGH)
    "PIN_LED_R": 2,
    "PIN_LED_G": 3,
    "PIN_LED_B": 4,
    # Optional single WS2812 NeoPixel. None = disabled.
    "PIN_NEOPIXEL": None,
    "PIO_NEOPIXEL_SM_ID": 1,
    # GPIO — camera shutter / intervalometer
    "PIN_CTRL_CAMERA": 22,
    "CTRL_CAMERA_PULSE_MS": 100,
    "CTRL_CAMERA_ACTIVE_HIGH": True,
    # OLED 128x64 over I2C
    "DSP_ENABLED": True,
    "DSP_DRIVER": "ssd1306",
    "DSP_ROTATE_180": False,
    "PIN_DSP_I2C_SDA": 0,
    "PIN_DSP_I2C_SCL": 1,
    "DSP_I2C_ID": 0,
    "DSP_I2C_ADDR": 0x3C,
    "DSP_I2C_FREQ": 400_000,
    "DSP_WIDTH": 128,
    "DSP_HEIGHT": 64,
    "DSP_UPDATE_MS": 250,
    "DSP_LIVE_POS": True,
    "DSP_CONTRAST_FULL": 0xFF,
    # RGB LED polarity and blink timing
    "LED_ACTIVE_HIGH": True,
    "LED_BLINK_MS": 250,
    "LED_BLINK_HARD_LIMIT_MS": 80,
    "LED_BLINK_DELAY_WAIT_MS": 500,
    "LED_DIM_WHITE": 0.12,
    "LED_DIM_ORANGE": 0.12,
    "LED_DIM_CYAN": 0.12,
    "LED_DIM_MAGENTA": 0.12,
    "LED_RAINBOW_MS": 1000,
    "LED_SOFT_NEAR_BLUE_ADD": 76,
    "LED_SOFT_AT_BLUE_ADD": 255,
    "SOFT_LIMIT_WARN_MM": 10.0,
    "LED_ACCEL_SPEED_EPS_MM_S": 3.0,
    # Watchdog + Pico onboard LED heartbeat
    "WDT_ENABLED": True,
    "WDT_TIMEOUT_MS": 3000,
    "WDT_HEARTBEAT_MS": 1000,
    "PIN_LED_ONBOARD": "LED",
    # USB debug (0=off … 5=detailed)
    "DEBUG_LEVEL": 3,
}

# ---------------------------------------------------------------------------
# JKSlider panel app — consumed by JKSliderConfig.py
# ---------------------------------------------------------------------------
JKSlider = {
    # GPIO — potentiometers (always)
    "PIN_POT_SPEED": 26,  # ADC0 — SPEED (left=min floor, right=MC max_speed)
    "PIN_POT_ACCEL": 27,  # ADC1 — ACCEL (left=min, right=local/MC max)
    "PIN_POT_JOYSTICK": None,  # Optional centre-return stick (e.g. 28 = ADC2). None = off.
    # Input mode
    # "button" = one GPIO per switch (active-low, pull-ups).
    # "keypad" = 4x3 matrix on PIN_KEYPAD_ROWS / PIN_KEYPAD_COLS;
    #            discrete BTN_STOP (GP5) and BTN_OPTION (GP13) still ORed in.
    "JKS_INPUT_MODE": "keypad",
    # Discrete buttons (always available)
    # In keypad mode: ORed with matrix STOP / OPTION after ghost filter.
    "PIN_BTN_STOP": 5,
    "PIN_BTN_OPTION": 13,  # modifier (hold + other control); alone does nothing
    # Discrete BTN_* — used when JKS_INPUT_MODE == "button"
    "PIN_BTN_MOVE_L": 6,
    "PIN_BTN_MOVE_R": 7,
    "PIN_BTN_FAST_L": 8,
    "PIN_BTN_FAST_R": 9,
    "PIN_BTN_A": 10,
    "PIN_BTN_B": 11,
    "PIN_BTN_C": 12,
    "PIN_BTN_DELAY": 14,  # Optional: hold N s → delay; short → delay off
    "PIN_BTN_TIMELAPSE": 15,  # Optional: tap → TL divider; long → divider 1
    # Keypad matrix — used when JKS_INPUT_MODE == "keypad"
    # Rows GP6..GP9 = KP_ROW1..KP_ROW4 (KP_ROW1 = upper keys on GP6).
    # Cols GP10..GP12 = KP_COL1..KP_COL3.
    # High-Z row scan (no row diodes). See SliderDoc uic/projects/jkslider/technical/panel.md
    "PIN_KEYPAD_ROWS": (6, 7, 8, 9),
    "PIN_KEYPAD_COLS": (10, 11, 12),
    # Joystick / pot feel
    "JOYSTICK_DEADZONE": 0.08,
    "JKS_SPEED_DEADZONE": 0.02,
    "JKS_SPEED_CURVE_GAMMA": 2.0,
    "JKS_JOYSTICK_CURVE_GAMMA": 2.0,
    "JKS_POT_OVERSAMPLE": 8,
    "JKS_POT_EMA_ALPHA": 0.2,
    "JKS_POT_HYST": 0.008,
    "JKS_ACCEL_EMA_ALPHA": 0.15,
    "JKS_ACCEL_HYST": 0.01,
    "JKS_JOY_OVERSAMPLE": 8,
    "JKS_JOY_EMA_ALPHA": 0.3,
    "JKS_JOY_HYST": 0.006,
    # Speed / accel panel ceilings
    "JKS_SPEED_MIN_MM_S": 1.0,
    "JKS_SPEED_MAX_MM_S": 100.0,
    "JKS_ACCEL_MIN_MM_S2": 50.0,
    "JKS_ACCEL_MAX_MM_S2": 500.0,
    # Button timing
    "JKS_LONG_PRESS_MS": 1000,
    "JKS_STOP_HALT_MS": 1000,
    "JKS_STOP_DISABLE_MS": 2000,
    "JKS_BTN_DEBOUNCE_MS": 30,
    "JKS_MOVE_TAP_MS": 333,
    # Motion behaviour
    "JKS_LEFT_IS_NEGATIVE": True,
    "JKS_SWAP_LR": False,
    "JKS_LOOP_DWELL_MS": 1000,
    # Delay / OPTION
    "JKS_DELAY_CLEAR_MS": 300,
    "JKS_DELAY_MAX_S": 30.0,
    "JKS_OPTION_DELAY_SCALE": 5.0,
    "JKS_OPTION_DELAY_PRESET_S": 5.0,
    "JKS_OPTION_TL_MAX": 100,
    "JKS_OPTION_TL_FAVORITE": 25,
    "JKS_DISPLAY_DIM": 0.25,
    # Positions / marks
    "JKS_STORE_MARGIN_MM": 3.0,
    "JKS_AT_MARK_MM": 0.5,
    "JKS_POSITIONS_FILE": "jks_positions.txt",
    # Timelapse / camera
    "JKS_CAMERA_FPS": 30,
    "JKS_CAMERA_FPS_STEPS": (24, 25, 30, 48, 50, 60),
    "JKS_TL_MODE": "msm",
    "JKS_MSM_EXPOSURE_MS": 200,
    "JKS_MSM_SETTLE_MS": 50,
    "JKS_MSM_MIN_STEP_MM": 0.1,
    # Display / boot
    "JKS_DSP_EXTRA_ROTATE_MS": 1000,
    "JKS_DSP_UNIT": "mm",
    "JKS_BOOT_TEXT": "JKSlider V1 by JK",
    "JKS_BOOT_SPLASH_MS": 2000,
    "JKS_BOOT_UNLOCK": True,
    "JKS_HOMING_ENABLED": True,
}

# ---------------------------------------------------------------------------
# B4Slider — 4-button app (MOVE_L/R, OPTION, SET + SPEED pot). Consumed by B4SliderConfig.py
# ---------------------------------------------------------------------------
B4Slider = {
    "PIN_POT_SPEED": 26,
    "PIN_POT_ACCEL": 27,
    "PIN_BTN_MOVE_L": 6,
    "PIN_BTN_MOVE_R": 7,
    "PIN_BTN_OPTION": 13,
    "PIN_BTN_SET": 5,
    "B4S_USE_ACCEL_POT": 0,
    "B4S_ACCEL_PRESET_L": 100.0,
    "B4S_ACCEL_PRESET_H": 400.0,
    "B4S_NEAR_SOFT_MM": 3.0,
    "B4S_BOOT_UNLOCK": True,
    "B4S_LEFT_IS_NEGATIVE": True,
    "B4S_MOVE_TAP_MS": 333,
    "B4S_LONG_PRESS_MS": 1000,
    "B4S_EXTRA_LONG_MS": 3000,
    "B4S_LEARN_HOLD_MS": 5000,
}
