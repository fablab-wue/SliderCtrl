# SliderCtrl — potentiometer / joystick velocity control example.
#
# Wiring (Raspberry Pi Pico):
#   Potentiometer wiper -> PIN_POT_JOYSTICK_ADC (default GP28 / ADC2)
#   Potentiometer ends  -> 3V3 and GND
#
# Behaviour:
#   Centre   → stop (move(0))
#   Right    → forward  (positive mm/s)
#   Left     → backward (negative mm/s)
#   Deflection from centre sets speed (up to setMaxSpeed)
#
# Copy MC_config.py, MC_client.py, and this file to the Pico
# (SliderMC on UART GP16/17 @ 1 Mbaud), then:
#   import JoystickExample
#   JoystickExample.run()

import uasyncio as asyncio
from machine import ADC, Pin

from MC_client import MC_Client

# Potentiometer / joystick wiper for JoystickExample.py (ADC2 = GP28).
PIN_POT_JOYSTICK_ADC = 28

# Fraction of full-scale travel treated as centre deadzone.
JOYSTICK_DEADZONE = 0.08


def _adc_to_speed(raw_u16, max_speed_mm_s, deadzone):
    """Map 16-bit ADC reading to signed speed (mm/s). Mid-scale = 0."""
    # MicroPython ADC: 0 .. 65535
    norm = (raw_u16 - 32768) / 32768.0  # ≈ -1 .. +1
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
    return sign * magnitude * max_speed_mm_s


async def demo():
    mc = MC_Client()
    await mc.start()
    mc.setMaxSpeed(80.0)          # mm/s at full deflection
    mc.setAcceleration(250.0)     # used by move() ramps / reversals
    soft_max = mc.slider_max if mc.slider_max is not None else 600.0
    mc.setSoftLimits(0.0, soft_max)
    mc.enable(True)

    adc = ADC(Pin(PIN_POT_JOYSTICK_ADC))
    deadzone = JOYSTICK_DEADZONE
    max_speed = 80.0

    print("Joystick velocity control running.")
    print("Centre=stop, right=forward, left=backward. Ctrl+C to exit.")

    last_speed = None
    try:
        while True:
            speed = _adc_to_speed(adc.read_u16(), max_speed, deadzone)
            mc.move(speed)

            # Light console feedback when the command changes noticeably.
            if last_speed is None or abs(speed - last_speed) > 1.0:
                print(
                    "speed =",
                    round(speed, 1),
                    "mm/s  pos =",
                    round(mc.getPosition(), 2),
                    "mm",
                )
                last_speed = speed

            await asyncio.sleep_ms(20)
    finally:
        mc.move(0.0)
        await mc.wait()
        mc.enable(False)
        print("Stopped.")


def run():
    asyncio.run(demo())


if __name__ == "__main__":
    run()
