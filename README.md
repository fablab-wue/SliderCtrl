# SliderCtrl

**Open UI controller firmware for DIY motorized camera sliders** — MicroPython on a **Raspberry Pi Pico** (or compact **RP2040-Zero**), built to shoot, not to demo.

## About

**SliderCtrl** is the **UI controller (UIC)** side of an open **motorized camera slider** system. It gives operators a **laptop-free panel on set** — analogue knobs, muscle-memory buttons, OLED status, and camera trigger — instead of juggling phone apps mid-take.

You bring the rail, motor, and housing. The **firmware and on-set workflow** aim at behaviour comparable to expensive commercial motorized sliders: live retarget, smooth ramps, marks and loops, timelapse, STOP / EMO, and hard-limit homing. **Mechanics quality depends on your build** — the motion stack and panel UX are designed to keep up.

Motion runs on a separate board: **[SliderMC](https://github.com/fablab-wue/SliderMC)** (STEP/DIR, planner, limits). Docs and manuals live in **[SliderDoc](https://github.com/fablab-wue/SliderDoc)**. Optionally a **second** STEP/DIR axis (typical **slider travel + pan**) is time-synced with the first — enable `axis2_use` on SliderMC; `MC_Client` already speaks both axes.

> Documentation: [SliderDoc](https://github.com/fablab-wue/SliderDoc)

---

## Features

**DIY project, pro-set manners.** Open MicroPython panel firmware talks to a dedicated motion controller over UART. Control feel, safety interlocks, and motion firmware are meant to stand alongside commercial units — your rail, driver, and enclosure are yours to spec.

**Easy on set. Ready for every take.**

- **Set-first panel** — no phone required; analogue SPEED / ACCEL; STOP / EMO / soft and hard limits  
- **Feel the move** — live retarget · sine-smooth ramps · tap/hold MOVE cruise · FAST jog · optional joystick  
- **Production moves** — Pos A / B / C with power-off recall · pair loops · DELAY walk-ins · TIMELAPSE dividers · pause / resume  
- **Eyes-off status** — I2C OLED (SSD1306 / SH1106 / SSD1309) · RGB LED · optional NeoPixel (same colours)  
- **Open stack** — edit `SliderPins.py`, Thonny / REPL workflow · fork the panel or build on `MC_Client` / `UIC_Base` · or use the stack as a **construction kit** for custom 1- or 2-axis rigs  
- **Optional 2-axis** — linear travel + time-synced pan (or tilt/turn); SliderMC `axis2_use`; `MC_Client` dual `moveTo` / `home`. Shipping JKSlider / B4Slider stay 1-axis faces  
- **Split architecture** — OLED, keypad, and pots never steal STEP timing ([SliderMC](https://github.com/fablab-wue/SliderMC) owns motion)  
- **Maker-friendly** — upcycle rails and linear units · A4988, DRV8825, TMC, and other STEP/DIR drivers  

How this compares to commercial motorized sliders: [architecture/compare.md](https://github.com/fablab-wue/SliderDoc/blob/main/architecture/compare.md).

---

## UIC panel projects

The stack is a **software and electronics construction kit** — turnkey panel **faces** on the same motion firmware, or your own mix of libs and wiring. Pick the panel that fits your shoot and enclosure, or aim the same parts at a mini-dolly, rotating head, turntable, or a **slider + pan** (2-axis) custom UI.

| Project | Purpose | When to use | Entry |
|---------|---------|-------------|--------|
| **JKSlider** | Full motorized camera slider panel — keypad or discrete buttons, SPEED/ACCEL pots, OLED, marks A/B/C, timelapse, DELAY | Default for interviews, product, B-roll, and any shoot that needs the full feature set | [`JKSlider.py`](JKSlider.py) · [user manual](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/jkslider/user-manual.md) |
| **B4Slider** | Minimal 4-button remote — MOVE L/R, SET, OPTION, one SPEED pot, RGB status | Slim handheld, budget builds, or when you do not need OLED, keypad, marks, or timelapse | [`B4Slider.py`](B4Slider.py) · [user manual](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/b4slider/user-manual.md) |
| *More coming* | Additional UIC apps on the same `MC_Client` / UART protocol | Custom rigs and new panel ideas | [project template](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/_template/README.md) |

Under the hood, all projects share **`MC_Client`** + **`UIC_Base`** — kit libraries for your own feature-rich motorized camera slider UI, mini-dolly, rotating head, turntable, **2-axis slider + pan**, or other STEP/DIR rig.

---

## Architecture

UIC panel I/O on one Pico, motion (STEP/DIR, home, limits) on **SliderMC**, linked by UART.

![JKSlider architecture overview](https://github.com/fablab-wue/SliderDoc/raw/main/assets/img/architecture_overview.svg)

**Why two boards**

- Dedicated motion MCU — display redraws, keypad scans, and optional WLAN never steal STEP timing  
- **MicroPython + AsyncIO** on the UIC — easy panel iteration; Thonny / REPL DIY workflow  
- Replaceable panel face — same motion board, different UIC project or fork  
- **Handheld wired remote** — UIC in hand, MC by the driver and PSU; **4-wire** cable (**5 V**, **GND**, **TX**, **RX**)  
- Build and debug panel and axis separately  

Trade-off: a second Pico (~€5), a little more wiring. Philosophy and pinouts: [architecture/overview.md](https://github.com/fablab-wue/SliderDoc/blob/main/architecture/overview.md).

| UIC may connect | MC may connect |
|-----------------|----------------|
| Buttons, keypads, OLED, RGB/NeoPixel, pots, joysticks, camera | Motor / STEP·DIR driver, home switch, hard limits, Ext, `DRV_ERROR` |
| Optional WLAN, USB debug, UART → MC | USB debug, UART → UIC |

![JKSlider UIC Pico pinout — keypad mode](https://github.com/fablab-wue/SliderDoc/raw/main/uic/projects/jkslider/panel-layouts/pico_pinout_keypad.png)

SliderMC motion pinout: [mc/pins.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/pins.md) · [pico_pinout_mc.png](https://github.com/fablab-wue/SliderDoc/raw/main/assets/img/pico_pinout_mc.png)

---

## Quick start

1. Flash [MicroPython](https://micropython.org/download/RPI_PICO/) onto the **UIC** Pico (or matching UF2 for an **RP2040-Zero**). Flash [SliderMC](https://github.com/fablab-wue/SliderMC) onto the **motion** Pico.  
2. Wire **crossed UART** (GP16/17 both sides, shared GND) — [link and handshake](https://github.com/fablab-wue/SliderDoc/blob/main/contract/link-and-handshake.md#communication-mc--uic).  
3. Copy `SliderPins.example.py` → `SliderPins.py` and edit **that file only** — [config](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/jkslider/technical/config.md) · [panel](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/jkslider/technical/panel.md).  
4. Copy project files to the UIC ([Thonny](https://thonny.org/) or `mpremote`) — [bring-up](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/jkslider/technical/bring-up.md).  
5. Run the panel:

```python
import JKSlider
JKSlider.run()
```

`await start()` unlocks the MC with `\n` and expects the welcome banner (retry 100 ms, 3 s). If the MC is missing, the UIC prints a timeout on USB/REPL and continues without motion.

Or try the motion library alone:

```python
import SimpleExample
SimpleExample.run()
```

**More:** [Technical manual](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/jkslider/technical/README.md) · [Protocol](https://github.com/fablab-wue/SliderDoc/blob/main/contract/protocol.md) · [Hardware / mechanics](https://github.com/fablab-wue/SliderDoc/blob/main/build/hardware-manual.md)

---

## Documentation (SliderDoc)

| Topic | Document |
|-------|----------|
| Architecture | [architecture/overview.md](https://github.com/fablab-wue/SliderDoc/blob/main/architecture/overview.md) |
| JKSlider install | [uic/projects/jkslider/technical/](https://github.com/fablab-wue/SliderDoc/blob/main/uic/projects/jkslider/technical/README.md) |
| UIC API | [uic/api/overview.md](https://github.com/fablab-wue/SliderDoc/blob/main/uic/api/overview.md) |
| Protocol | [contract/protocol.md](https://github.com/fablab-wue/SliderDoc/blob/main/contract/protocol.md) |
| MC build | [mc/build.md](https://github.com/fablab-wue/SliderDoc/blob/main/mc/build.md) |

---

## MC_Client + UIC_Base — build your own panel

Beyond turnkey panels, treat **JKSlider**, **B4Slider**, **`MC_Client`**, and **`UIC_Base`** as kit parts — fork a face, strip features, or wire a new enclosure for a one-off slider, mini-dolly, rotating head, or turntable.

Compose a UART motion client (`MC_Client`) with local UI (`UIC_Base`) on the UIC Pico: millimetre API to **SliderMC**, soft and hard limits, EMO, RGB / NeoPixel / OLED. STEP/DIR generation runs on the motion Pico. Optional **2-axis** (typical **linear + pan**, time-synced) is on the same client.

JKSlider and B4Slider are applications on top — use the libraries when you want a custom UI, scripted moves, or the next panel face.

```python
import uasyncio as asyncio
from MC_client import MC_Client
from UIC_base import UIC_Base

async def main():
    mc = MC_Client()
    ui = UIC_Base()

    mc.set_status_callback(ui.on_status)
    await mc.start()
    await ui.start()

    mc.enable(True)
    mc.setSpeed(40)
    mc.setAcceleration(150)
    mc.home()
    await mc.wait()
    mc.moveTo(100)
    await mc.wait()
    mc.move(-25)
    await asyncio.sleep(2)
    mc.stop()
    await mc.wait()
    mc.enable(False)

asyncio.run(main())
```

**Optional 2-axis** (typical **axis 1 = linear**, **axis 2 = pan**): dual `MT`/`M` is [time-synced](https://github.com/fablab-wue/SliderDoc/blob/main/mc/dual-movement.md), not CNC. `mc.axis_count` / `getAxisCount()` come from CG `axis2_use` (not live `IA`). Shipping panels keep `set_status_callback` (5-arg, axis 1) even on a 2-axis MC. Custom 2-axis UIs register `set_status2_callback` (9-arg) and use `moveTo(pos, pos2)`, `moveTo(None, pos2)` → `MT _ pos2`, `home(2)`.

| Idea | Entry point |
|------|-------------|
| Point-to-point demo | `SimpleExample.py` |
| Pot → velocity | `JoystickExample.py` |
| Full camera panel | `JKSlider.py` |
| Minimal 4-button panel | `B4Slider.py` |
| API reference | [uic/api/overview.md](https://github.com/fablab-wue/SliderDoc/blob/main/uic/api/overview.md) |

Copy `SliderPins.example.py` → `SliderPins.py` and edit **that file only** for your hardware.

---

## Hardware requirements

**Base (all sliders)**

- Raspberry Pi Pico (RP2040), Pico W, or RP2040-Zero  
- MicroPython with `rp2.PIO` and `uasyncio`  
- External STEP/DIR motor driver + [SliderMC](https://github.com/fablab-wue/SliderMC) motion board  
- Motorized camera slider mechanics — [hardware manual](https://github.com/fablab-wue/SliderDoc/blob/main/build/hardware-manual.md)

**JKSlider**

- Buttons or 3×4 keypad · pots for SPEED and ACCEL · RGB LED · OLED · (optional) joystick

**B4Slider**

- 4 buttons · RGB LED · SPEED pot · (optional) ACCEL pot · no OLED required

---

## License

Copyright (c) 2026 Jochen Krapf \<jk@nerd2nerd.org\>

Licensed under the [MIT License](LICENSE).

Company names and product names mentioned in this project are trademarks or registered trademarks of their respective owners. Use here is for identification only.

`ssd1306.py` / `sh1106.py` / `ssd1309.py` are based on common MicroPython OLED patterns (SSD1306 lineage typically MIT). `oledfont.py` uses Adafruit GFX 5×7 font data (BSD-style upstream). Keep their notices if you redistribute those files alone.
