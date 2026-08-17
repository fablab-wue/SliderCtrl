# I2C bus scan + SSD1309 "hello" smoke test for Raspberry Pi Pico.
#
# Copy this file and ssd1309.py to the Pico, then:
#   import I2C_OLED_Test
#
# Wiring (project defaults): SDA=GP0, SCL=GP1, I2C0 @ 400 kHz.
#
# If i2c.scan() is empty / SSD1309 not detected — known causes:
#
# 1) Module still in SPI mode (common on 1.5"/2.42" boards)
#    Many SSD1309 modules ship as SPI. For I2C you often must:
#    - move resistor R4 -> R3
#    - bridge R5 & R7
#    - join D1-D2 (I2C needs both data pins)
#    - set address via DC/SA0: GND -> 0x3C, VCC -> 0x3D
#    Without that: nothing on the scan.
#
# 2) RES/RST floating or not pulsed
#    Especially DIYMORE 2.42" and similar:
#    - after power-on, pulse RES low then high (or ~10k pull-up to VCC)
#    - without reset: no scan, or scan OK but blank screen
#    - no software reset over I2C (per datasheet)
#
# 3) Wiring / power
#    - loose breadboard jumpers (most common "no devices")
#    - common GND
#    - Pico: 3.3 V (not 5 V on VCC/logic without level shifter)
#    - SDA/SCL swapped or wrong GPIOs vs this script (GP0/GP1)
#
# 4) Pull-ups
#    - I2C needs pull-ups on SDA/SCL (~4.7k to 3.3 V)
#    - many modules have them onboard; bare panels often do not
#    - idle SDA/SCL should read HIGH; else scan empty/flaky
#
# 5) I2C ACK / protection diodes on some boards
#    Some 2.42" modules block ACK (diode on SDA). Fix per board
#    docs (short/replace diode/resistor) so SDA can be pulled low.
#
# 6) Address
#    Usually 0x3C, sometimes 0x3D. Use whatever scan() reports.
#
# 7) Wrong chip / wrong driver
#    Marketplace "SSD1309" may be SSD1306 or SH1106. Scan can still
#    show 0x3C while the image is blank/shifted — try the matching
#    driver. That case is "detected but no text", not empty scan.
#
# Check order: power+GND -> SDA/SCL GP0/GP1 -> I2C jumpers ->
# RES pull-up/pulse -> external 4.7k pull-ups if needed -> re-scan.

from machine import I2C, Pin
from ssd1309 import SSD1309_I2C

I2C_ID = 0
SDA_PIN = 0
SCL_PIN = 1
I2C_FREQ = 400_000
WIDTH = 128
HEIGHT = 64


def main():
    i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
    devices = i2c.scan()

    if not devices:
        print("No I2C devices found")
        return

    print("I2C devices present:")
    for addr in devices:
        print("  Found: 0x{:02X}".format(addr))

    addr = devices[0]
    print("Init SSD1309 at 0x{:02X}".format(addr))
    oled = SSD1309_I2C(WIDTH, HEIGHT, i2c, addr=addr)
    oled.fill(0)
    oled.text("hello", 0, 0)
    oled.show()
    print("Displayed: hello")


main()
