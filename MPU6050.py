'''
MPU6050.py — I2C accel/gyro reader for hand-held tilt sensing.

    imu = MPU6050(pin_SDA=0, pin_SCL=1, addr=0x68, auto=True)
    # auto=True: await imu.start() for ~25 Hz asyncio poll; else call imu.poll()
    # optional: callback=fn or subclass and override on_data()

Public: accel_x/y/z (g, EMA), gyro_x/y/z (deg/s)
Angles: getTilt() / getRoll() in degrees (0 = upright, +Z up)

written by Jochen Krapf (jk@nerd2nerd.org)
'''

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

import math
import time

try:
    from machine import I2C, Pin
except ImportError:
    I2C = None
    Pin = None


# Registers
_REG_SMPLRT_DIV = 0x19
_REG_CONFIG = 0x1A
_REG_GYRO_CONFIG = 0x1B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B
_REG_PWR_MGMT_1 = 0x6B
_REG_WHO_AM_I = 0x75

_WHO_AM_I_EXPECTED = 0x68

# ±2g / ±250 °/s
_ACCEL_LSB_PER_G = 16384.0
_GYRO_LSB_PER_DPS = 131.0

_DLPF_CFG = 4          # ~21 Hz accel / ~20 Hz gyro
_SMPLRT_DIV = 39       # 1000 / (1+39) = 25 Hz
_EMA_ALPHA = 0.35


def _s16(hi, lo):
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v


class MPU6050:
    """MPU6050 6-axis IMU → public accel/gyro + tilt/roll."""

    def __init__(
        self,
        i2c_id=0,
        pin_SDA=0,
        pin_SCL=1,
        addr=0x68,
        auto=True,
        interval_ms=40,
        callback=None,
    ):
        if I2C is None:
            raise RuntimeError("machine.I2C not available")
        if interval_ms < 1:
            raise ValueError("interval_ms must be >= 1")

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.callback = callback

        self._auto = bool(auto)
        self._interval_ms = int(interval_ms)
        self._addr = addr
        self._ema_ready = False
        self._rx_task = None

        self._i2c = I2C(
            i2c_id,
            sda=Pin(pin_SDA),
            scl=Pin(pin_SCL),
            freq=400_000,
        )
        self._init_device()

    def _w(self, reg, val):
        self._i2c.writeto_mem(self._addr, reg, bytes((val,)))

    def _r(self, reg, n=1):
        return self._i2c.readfrom_mem(self._addr, reg, n)

    def _init_device(self):
        who = self._r(_REG_WHO_AM_I)[0]
        if who != _WHO_AM_I_EXPECTED:
            raise RuntimeError(
                "MPU6050 WHO_AM_I=0x%02X (expected 0x%02X)" % (who, _WHO_AM_I_EXPECTED)
            )
        # Wake: clear sleep, use PLL with X-gyro as clock (more stable than internal osc).
        self._w(_REG_PWR_MGMT_1, 0x01)
        time.sleep_ms(50)
        self._w(_REG_CONFIG, _DLPF_CFG)
        self._w(_REG_SMPLRT_DIV, _SMPLRT_DIV)
        self._w(_REG_GYRO_CONFIG, 0x00)   # ±250 °/s
        self._w(_REG_ACCEL_CONFIG, 0x00)  # ±2g
        time.sleep_ms(20)

    # --- lifecycle ---------------------------------------------------------

    def on_data(self):
        """Called after public fields update. Override in a subclass."""
        pass

    def _notify(self):
        self.on_data()
        cb = self.callback
        if cb:
            cb(self)

    async def start(self):
        """Start asyncio sample task (used when auto=True)."""
        if self._rx_task is None:
            self._rx_task = asyncio.create_task(self._rx_loop())

    async def stop(self):
        """Cancel asyncio sample task."""
        t = self._rx_task
        self._rx_task = None
        if t is not None:
            t.cancel()
            try:
                await t
            except asyncio.CancelledError:
                pass

    def deinit(self):
        self._rx_task = None
        self._i2c = None

    def poll(self):
        """Read one IMU sample; update public axes (EMA on accel)."""
        i2c = self._i2c
        if i2c is None:
            return
        raw = i2c.readfrom_mem(self._addr, _REG_ACCEL_XOUT_H, 14)
        ax = _s16(raw[0], raw[1]) / _ACCEL_LSB_PER_G
        ay = _s16(raw[2], raw[3]) / _ACCEL_LSB_PER_G
        az = _s16(raw[4], raw[5]) / _ACCEL_LSB_PER_G
        # raw[6:8] = temperature (skip)
        gx = _s16(raw[8], raw[9]) / _GYRO_LSB_PER_DPS
        gy = _s16(raw[10], raw[11]) / _GYRO_LSB_PER_DPS
        gz = _s16(raw[12], raw[13]) / _GYRO_LSB_PER_DPS

        self.gyro_x = gx
        self.gyro_y = gy
        self.gyro_z = gz

        if not self._ema_ready:
            self.accel_x = ax
            self.accel_y = ay
            self.accel_z = az
            self._ema_ready = True
        else:
            a = _EMA_ALPHA
            b = 1.0 - a
            self.accel_x = a * ax + b * self.accel_x
            self.accel_y = a * ay + b * self.accel_y
            self.accel_z = a * az + b * self.accel_z

        self._notify()

    async def _rx_loop(self):
        try:
            while True:
                self.poll()
                await asyncio.sleep_ms(self._interval_ms)
        except asyncio.CancelledError:
            raise

    # --- angles ------------------------------------------------------------

    def getTilt(self):
        """Pitch about Y in degrees; 0 = upright (+Z up)."""
        return math.atan2(-self.accel_x, self.accel_z) * 180.0 / math.pi

    def getRoll(self):
        """Roll about X in degrees; 0 = upright (+Z up)."""
        return math.atan2(self.accel_y, self.accel_z) * 180.0 / math.pi


if __name__ == "__main__":
    imu = MPU6050(pin_SDA=0, pin_SCL=1, addr=0x68, auto=False, interval_ms=40)
    next_print = time.ticks_add(time.ticks_ms(), 250)
    try:
        while True:
            imu.poll()
            now = time.ticks_ms()
            if time.ticks_diff(now, next_print) >= 0:
                print(
                    "A",
                    round(imu.accel_x, 3),
                    round(imu.accel_y, 3),
                    round(imu.accel_z, 3),
                    "G",
                    round(imu.gyro_x, 1),
                    round(imu.gyro_y, 1),
                    round(imu.gyro_z, 1),
                    "tilt",
                    round(imu.getTilt(), 1),
                    "roll",
                    round(imu.getRoll(), 1),
                )
                next_print = time.ticks_add(now, 250)
            time.sleep_ms(40)
    finally:
        imu.deinit()
