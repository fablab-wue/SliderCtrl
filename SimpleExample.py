# SimpleExample — minimal asyncio demo (MC_Client → SliderMC over UART).
#
# Copy MC_config.py, MC_client.py, and this file to the Pico,
# then: import SimpleExample; SimpleExample.run()
#
# Optional: UIC_base.py + UIC_config.py if you want OLED/LED.

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

from MC_client import MC_Client


async def main():
    mc = MC_Client()
    await mc.start()

    mc.setMaxSpeed(80.0)          # mm/s (CS to MC)
    mc.setSpeed(40.0)             # mm/s cruise
    mc.setAcceleration(150.0)     # mm/s² (sine ramp peak)
    soft_max = mc.slider_max if mc.slider_max is not None else 600.0
    mc.setSoftLimits(0.0, soft_max)

    mc.enable(True)

    # Home, then absolute / relative moves.
    mc.home()
    await mc.wait()
    print("Position after home:", mc.getPosition(), "mm")

    mc.moveTo(100.0)
    while mc.isMoving():
        await asyncio.sleep_ms(200)
        print("  pos =", round(mc.getPosition(), 2), "mm")

    print("Arrived:", mc.getPosition(), "mm")

    mc.moveBy(25.0)
    await mc.wait()
    print("Arrived:", mc.getPosition(), "mm")

    mc.moveTo(0.0)
    await mc.wait()

    mc.enable(False)


def run():
    asyncio.run(main())


if __name__ == "__main__":
    run()
