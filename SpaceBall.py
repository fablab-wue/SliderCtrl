'''
SpaceBall.py — UART reader for classic serial 6DOF devices.

Supports Magellan/SpaceMouse, Spaceball 2003/3003/4000, and SpaceOrb 360
(not USB; not modern SpaceMouse Module framing).

    sb = SpaceBall(pin_TX=8, pin_RX=9, protocol=0, auto=True)
    # protocol: 0=auto, 1=Magellan, 2=Spaceball, 3=SpaceOrb
    # auto=True: await sb.start() for asyncio RX; else call sb.poll()
    # optional: callback=fn or subclass and override on_data()

Public: trans_x/y/z, rot_x/y/z, buttons, protocol

written by Jochen Krapf (jk@nerd2nerd.org)
'''

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

import time

try:
    from machine import UART, Pin
except ImportError:
    UART = None
    Pin = None


# Magellan / SpaceMouse printable nibble alphabet (Linux magellan.c).
_MAG_NIBBLES = b"0AB3D56GH9:K<MN?"

# SpaceOrb XOR mask for 'D' payload (Linux spaceorb.c).
_ORB_XOR = b"SpaceWare"


class SpaceBall:
    """Serial SpaceMouse / Spaceball / SpaceOrb → public 6DOF axes."""

    PROTO_AUTO = 0
    PROTO_MAGELLAN = 1
    PROTO_SPACEBALL = 2
    PROTO_SPACEORB = 3

    def __init__(self, pin_TX=8, pin_RX=9, protocol=0, auto=True, callback=None):
        if UART is None:
            raise RuntimeError("machine.UART not available")
        if protocol not in (
            self.PROTO_AUTO,
            self.PROTO_MAGELLAN,
            self.PROTO_SPACEBALL,
            self.PROTO_SPACEORB,
        ):
            raise ValueError("protocol must be 0..3")

        self.trans_x = 0
        self.trans_y = 0
        self.trans_z = 0
        self.rot_x = 0
        self.rot_y = 0
        self.rot_z = 0
        self.buttons = 0
        self.protocol = protocol
        self.callback = callback

        self._auto = bool(auto)
        self._wanted = protocol
        self._uart = UART(
            1,
            baudrate=9600,
            tx=Pin(pin_TX),
            rx=Pin(pin_RX),
            bits=8,
            parity=None,
            stop=1,
        )
        self._rx_task = None

        # Magellan / Spaceball CR path (Spaceball uses ^ escape).
        self._cr = bytearray()
        self._escape = False

        # SpaceOrb MSB-framed path.
        self._orb = bytearray()

        # Drain boot noise.
        while self._uart.any():
            self._uart.read()

        if protocol == self.PROTO_AUTO:
            self._autodetect()
        elif protocol == self.PROTO_MAGELLAN:
            self._init_magellan()
        elif protocol == self.PROTO_SPACEBALL:
            self._init_spaceball()
        # SpaceOrb: no host init required.

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
        """Start asyncio RX task (used when auto=True)."""
        if self._rx_task is None:
            self._rx_task = asyncio.create_task(self._rx_loop())

    async def stop(self):
        """Cancel asyncio RX task."""
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
        u = self._uart
        self._uart = None
        if u is not None:
            try:
                u.deinit()
            except Exception:
                pass

    def poll(self):
        """Drain UART into parsers; update public axes."""
        u = self._uart
        if u is None:
            return
        n = u.any()
        if not n:
            return
        data = u.read(n)
        if not data:
            return
        for b in data:
            self._feed_byte(b)

    async def _rx_loop(self):
        try:
            while True:
                self.poll()
                await asyncio.sleep_ms(1)
        except asyncio.CancelledError:
            raise

    # --- byte feed ---------------------------------------------------------

    def _feed_byte(self, b):
        proto = self.protocol
        if proto in (self.PROTO_AUTO, self.PROTO_SPACEORB):
            self._feed_orb(b)
        if proto in (self.PROTO_AUTO, self.PROTO_MAGELLAN, self.PROTO_SPACEBALL):
            self._feed_cr(b)

    def _feed_cr(self, b):
        # Spaceball escape: ^ then M/Q/S → control; ^^ → ^
        if self._escape:
            self._escape = False
            if b in (ord("M"), ord("Q"), ord("S")):
                b &= 0x1F
            elif b != ord("^"):
                # stray after ^ — keep as-is (Linux clears escape)
                pass
            if len(self._cr) < 64:
                self._cr.append(b)
            return

        if b == 0x0D:  # CR → packet
            self._process_cr(bytes(self._cr))
            self._cr = bytearray()
            self._escape = False
            return

        if b == ord("^") and self.protocol in (
            self.PROTO_AUTO,
            self.PROTO_SPACEBALL,
        ):
            self._escape = True
            return

        if b == 0x0A:  # LF ignored
            return

        if len(self._cr) < 64:
            self._cr.append(b)

    def _feed_orb(self, b):
        # New packet when bit7 clear (Linux spaceorb_interrupt).
        if (b & 0x80) == 0:
            if self._orb:
                self._process_orb(bytes(self._orb))
            self._orb = bytearray()
        if len(self._orb) < 64:
            self._orb.append(b & 0x7F)

    # --- CR packets (Magellan + Spaceball) ---------------------------------

    def _process_cr(self, pkt):
        if not pkt:
            return
        hdr = pkt[0]
        proto = self.protocol

        if proto in (self.PROTO_AUTO, self.PROTO_MAGELLAN):
            if hdr == ord("d") and self._try_magellan_d(pkt):
                self._lock(self.PROTO_MAGELLAN)
                self._notify()
                return
            if hdr == ord("k") and self._try_magellan_k(pkt):
                self._lock(self.PROTO_MAGELLAN)
                self._notify()
                return
            if hdr == ord("v") and proto == self.PROTO_AUTO:
                # Version reply during probe — likely Magellan.
                if b"Magellan" in pkt or b"SPACE" in pkt or b"Space" in pkt:
                    self._lock(self.PROTO_MAGELLAN)
                return

        if proto in (self.PROTO_AUTO, self.PROTO_SPACEBALL):
            if hdr == ord("D") and self._try_spaceball_d(pkt):
                self._lock(self.PROTO_SPACEBALL)
                self._notify()
                return
            if hdr == ord("K") and self._try_spaceball_k(pkt):
                self._lock(self.PROTO_SPACEBALL)
                self._notify()
                return
            if hdr == ord(".") and self._try_spaceball_adv_k(pkt):
                self._lock(self.PROTO_SPACEBALL)
                self._notify()
                return

    def _mag_crunch(self, data, count):
        """Verify upper nibbles and strip to low nibble in-place copy."""
        out = bytearray(data)
        for i in range(count, 0, -1):
            c = out[i]
            if c != _MAG_NIBBLES[c & 0x0F]:
                return None
            out[i] = c & 0x0F
        return out

    def _try_magellan_d(self, pkt):
        # idx == 25 including header (Linux magellan_process_packet).
        if len(pkt) != 25:
            return False
        data = self._mag_crunch(pkt, 24)
        if data is None:
            return False
        axes = []
        for i in range(6):
            o = (i << 2) + 1
            v = (data[o] << 12) | (data[o + 1] << 8) | (data[o + 2] << 4) | data[o + 3]
            axes.append(v - 32768)
        self.trans_x, self.trans_y, self.trans_z = axes[0], axes[1], axes[2]
        self.rot_x, self.rot_y, self.rot_z = axes[3], axes[4], axes[5]
        return True

    def _try_magellan_k(self, pkt):
        if len(pkt) != 4:
            return False
        data = self._mag_crunch(pkt, 3)
        if data is None:
            return False
        self.buttons = (data[1] << 1) | (data[2] << 5) | data[3]
        return True

    def _try_spaceball_d(self, pkt):
        # 'D' + 14 data bytes = 15 (Linux spaceball.c).
        if len(pkt) != 15:
            return False
        # Skip first three bytes; six BE int16.
        # Linux axis order: X, Z, Y, RX, RZ, RY
        raw = pkt[3:]
        vals = []
        for i in range(6):
            hi = raw[i * 2]
            lo = raw[i * 2 + 1]
            v = (hi << 8) | lo
            if v & 0x8000:
                v -= 0x10000
            vals.append(v)
        self.trans_x = vals[0]
        self.trans_z = vals[1]
        self.trans_y = vals[2]
        self.rot_x = vals[3]
        self.rot_z = vals[4]
        self.rot_y = vals[5]
        return True

    def _try_spaceball_k(self, pkt):
        if len(pkt) != 3:
            return False
        d1, d2 = pkt[1], pkt[2]
        btns = 0
        if (d2 & 0x01) or (d2 & 0x20):
            btns |= 1 << 0
        if d2 & 0x02:
            btns |= 1 << 1
        if d2 & 0x04:
            btns |= 1 << 2
        if d2 & 0x08:
            btns |= 1 << 3
        if d1 & 0x01:
            btns |= 1 << 4
        if d1 & 0x02:
            btns |= 1 << 5
        if d1 & 0x04:
            btns |= 1 << 6
        if d1 & 0x10:
            btns |= 1 << 7
        self.buttons = btns
        return True

    def _try_spaceball_adv_k(self, pkt):
        # '.' advanced buttons (4000 FLX)
        if len(pkt) != 3:
            return False
        d1, d2 = pkt[1], pkt[2]
        btns = 0
        for i in range(6):
            if d2 & (1 << i):
                btns |= 1 << i
        if d2 & 0x80:
            btns |= 1 << 6
        for i in range(6):
            if d1 & (1 << i):
                btns |= 1 << (7 + i)
        self.buttons = btns
        return True

    # --- SpaceOrb ----------------------------------------------------------

    def _process_orb(self, pkt):
        if len(pkt) < 2:
            return
        c = 0
        for b in pkt:
            c ^= b
        if c:
            return

        hdr = pkt[0]
        if hdr == ord("R"):
            self._lock(self.PROTO_SPACEORB)
            return
        if hdr == ord("D"):
            if self._try_orb_d(pkt):
                self._lock(self.PROTO_SPACEORB)
                self._notify()
            return
        if hdr == ord("K"):
            if len(pkt) == 5:
                self.buttons = pkt[2] & 0x3F
                self._lock(self.PROTO_SPACEORB)
                self._notify()

    def _try_orb_d(self, pkt):
        if len(pkt) != 12:
            return False
        data = bytearray(pkt)
        for i in range(9):
            data[i + 2] ^= _ORB_XOR[i]
        axes = [0] * 6
        axes[0] = (data[2] << 3) | (data[3] >> 4)
        axes[1] = ((data[3] & 0x0F) << 6) | (data[4] >> 1)
        axes[2] = ((data[4] & 0x01) << 9) | (data[5] << 2) | (data[4] >> 5)
        axes[3] = ((data[6] & 0x1F) << 5) | (data[7] >> 2)
        axes[4] = ((data[7] & 0x03) << 8) | (data[8] << 1) | (data[7] >> 6)
        axes[5] = ((data[9] & 0x3F) << 4) | (data[10] >> 3)
        for i in range(6):
            if axes[i] & 0x200:
                axes[i] -= 1024
        self.trans_x, self.trans_y, self.trans_z = axes[0], axes[1], axes[2]
        self.rot_x, self.rot_y, self.rot_z = axes[3], axes[4], axes[5]
        self.buttons = data[1] & 0x3F
        return True

    # --- lock / init / autodetect ------------------------------------------

    def _lock(self, proto):
        if self.protocol == self.PROTO_AUTO:
            self.protocol = proto

    def _write(self, data):
        u = self._uart
        if u is not None:
            u.write(data)

    def _init_magellan(self):
        self._write(b"\rvz\r")
        time.sleep_ms(50)
        self._write(b"vQ\r")
        time.sleep_ms(50)
        self._write(b"kQ\r")
        time.sleep_ms(20)
        self._write(b"m3\r")
        time.sleep_ms(20)

    def _init_spaceball(self):
        self._write(b"MSS\r")
        time.sleep_ms(50)

    def _autodetect(self):
        """Listen, then probe Magellan, then Spaceball; late lock still allowed."""
        # 1) Listen for SpaceOrb spontaneous traffic.
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 300:
            self.poll()
            if self.protocol != self.PROTO_AUTO:
                return
            time.sleep_ms(10)

        # 2) Magellan probe.
        self._write(b"\rvz\r")
        time.sleep_ms(40)
        self._write(b"vQ\r")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 400:
            self.poll()
            if self.protocol == self.PROTO_MAGELLAN:
                self._write(b"kQ\r")
                time.sleep_ms(20)
                self._write(b"m3\r")
                return
            time.sleep_ms(10)

        # 3) Spaceball probe.
        self._write(b"MSS\r")
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 500:
            self.poll()
            if self.protocol == self.PROTO_SPACEBALL:
                return
            time.sleep_ms(10)
        # Stay PROTO_AUTO — late lock on first unambiguous packet.


if __name__ == "__main__":
    sb = SpaceBall(pin_TX=8, pin_RX=9, protocol=0, auto=False)
    print("protocol:", sb.protocol)
    next_print = time.ticks_add(time.ticks_ms(), 250)
    try:
        while True:
            sb.poll()
            now = time.ticks_ms()
            if time.ticks_diff(now, next_print) >= 0:
                print(
                    "T",
                    sb.trans_x,
                    sb.trans_y,
                    sb.trans_z,
                    "R",
                    sb.rot_x,
                    sb.rot_y,
                    sb.rot_z,
                    "B",
                    sb.buttons,
                    "P",
                    sb.protocol,
                )
                next_print = time.ticks_add(now, 250)
            time.sleep_ms(5)
    finally:
        sb.deinit()
