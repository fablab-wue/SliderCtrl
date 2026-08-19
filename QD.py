'''
QD.py — quadrature decoder (QD) and single-pin Denoiser for RP2040 PIO.

QD: bounce-immune A/B edge decoder (other channel ignored while waiting).
Denoiser: independent per-pin cleaner (N agreeing samples, N pulled once);
the app HW-couples Denoiser outs to QD inputs — classes are not linked in code.

# See https://github.com/fablab-wue/SliderDoc/blob/main/uic/libraries/qd.md

written by Jochen Krapf (jk@nerd2nerd.org)

pin_b must be pin_a + 1 (B waits use in_base + 1).
States: s0=A↓, s1=B↑, s2=A↑, s3=B↓. A leads B → FIFO −1; B leads A → +1.
Other-channel noise ignored; waited-line spike can cost ±1.

Graph 1 — A leads B (3 cycles):
    A: ______/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\_________
    B: ____________/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\___
 edge:       A↑    B↑    A↓    B↓    A↑    B↑    A↓    B↓    A↑    B↑    A↓    B↓
state: s2    s1    s0    s3    s2    s1    s0    s3    s2    s1    s0    s3    s2
 FIFO:       -1    -1    -1    -1    -1    -1    -1    -1    -1    -1    -1    -1

Graph 1 — B leads A (ignored first B↑) → all FIFO +1
    A: ____________/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\___
    B: ______/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\___________/¯¯¯¯¯¯¯¯¯¯¯\_________
 edge:       (B↑)  A↑    B↓    A↓    B↑    A↑    B↓    A↓    B↑    A↑    B↓    A↓
state: s2    s2    s3    s0    s1    s2    s3    s0    s1    s2    s3    s0    s1
 FIFO:             +1    +1    +1    +1    +1    +1    +1    +1    +1    +1    +1

Graph 3 — Noisy environment (| = glitch):
    A: ____________/¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯\___|__|_________________/¯\/¯¯¯¯¯¯¯¯¯¯¯\______
    B: ______/¯¯¯¯¯¯¯¯¯¯¯\/¯\/\______________/¯¯¯|¯¯|¯\/¯¯¯¯¯¯¯¯¯¯¯¯¯\_____________
 edge:       (B↑)  A↑    B↓         A↓       B↑              A↑      B↓     A↓
state: s2..........s3....s0.........s1.......s2..............s3......s0.....s1
 FIFO:             +1    +1         +1       +1              +1      +1     +1
'''

from machine import Pin
import rp2
import time

import micropython
micropython.alloc_emergency_exception_buf(100)

# PIO: evaluate A and B edges, push direction (1=forward,0=back) into FIFO
@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT,
             out_shiftdir=rp2.PIO.SHIFT_LEFT,
             autopush=False, push_thresh=1,
             fifo_join=rp2.PIO.JOIN_RX)
def quad_a_edges():
    set(y, 1)              # need for a '1' to move in ISR

    wrap_target()

    # state 3 - wait for low in B, ignore bouncing A
    label("state_3")
    push()                 # push previous direction into FIFO
    wait(0, pin, 1)        # wait for B to go low
    in_(pins, 1)           # sample A into ISR
    mov(x, isr)            # move A bit into X
    jmp(x, "state_3_rev")  # jump if A is NOT high

    label("state_3_for")
    mov(isr, y)
    jmp("state_0")

    label("state_3_rev")
    mov(isr, null)
    jmp("state_2")


    # state 1 - wait for high in B, ignore bouncing A
    label("state_1")
    push()                 # push previous direction into FIFO
    wait(1, pin, 1)        # wait for B to go high
    in_(pins, 1)           # sample A into ISR
    mov(x, isr)            # move A bit into X
    jmp(x, "state_1_for")  # jump if A is NOT high

    label("state_1_rev")
    mov(isr, null)
    jmp("state_0")

    label("state_1_for")
    mov(isr, y)
    #jmp("state_2") # fall through to state_2


    # state 2 - wait for high in A, ignore bouncing B
    label("state_2")
    push()                   # push previous direction into FIFO
    wait(1, pin, 0)          # wait for A to go high
    jmp(pin, "state_2_for")  # jump if B is high

    label("state_2_rev")
    mov(isr, null)
    jmp("state_1")

    label("state_2_for")
    mov(isr, y)
    jmp("state_3")


    # state 0 - wait for low in A, ignore bouncing B
    label("state_0")
    push()                   # push previous direction into FIFO
    irq(0)
    wait(0, pin, 0)          # wait for A to go low
    jmp(pin, "state_0_rev")  # jump if B is high

    label("state_0_for")
    mov(isr, y)
    jmp("state_1")

    label("state_0_rev")
    mov(isr, null)
    #jmp("state_3") # wrap to state_3

    wrap()


def _check_sm_id(sm_id):
    if not isinstance(sm_id, int) or sm_id < 0 or sm_id > 7:
        raise ValueError("sm_id must be 0..7")


class QD:
    """Quadrature encoder via PIO (prefer SM 2; leave PIO1 for Denoiser)."""

    def __init__(self, sm_id, pin_a, pin_b, use_irq=True):
        _check_sm_id(sm_id)
        if not isinstance(pin_a, int) or pin_a < 0 or pin_a > 31:
            raise ValueError("pin_a must be 0..31")
        if not isinstance(pin_b, int) or pin_b < 0 or pin_b > 31:
            raise ValueError("pin_b must be 0..31")
        if pin_b != pin_a + 1:
            raise ValueError("pin_b must be pin_a + 1 (B waits use in_base + 1)")
        
        self._position = 0
        self._pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self._pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self._sm = rp2.StateMachine(
            sm_id,
            quad_a_edges,
            freq=125_000_000,
            in_base=self._pin_a,
            jmp_pin=self._pin_b,
        )
        self._sm.active(1)
        while self._sm.rx_fifo():
            self._sm.get()
        if use_irq:
            print("IRQ enabled")
            self._sm.irq(self._irq_handler)

    def _drain(self):
        while self._sm.rx_fifo():
            if self._sm.get():
                self._position += 1
            else:
                self._position -= 1

    def _irq_handler(self, sm):
        self._drain()

    @property
    def position(self):
        self._drain()
        return self._position

    @property
    def pin_a(self):
        return self._pin_a.value()

    @property
    def pin_b(self):
        return self._pin_b.value()

    def poll(self):
        """Drain RX FIFO into position (IRQ fallback / debug)."""
        self._drain()

    def reset(self, value=0):
        self._position = value

    def stop(self):
        self._sm.active(0)

    def deinit(self):
        self.stop()


# ---------------------------------------------------------------------------
# Denoiser PIO: output follows input after N consecutive agreeing samples.
# N is pulled once at start and kept in ISR for reload. jmp_pin = pin_in.
# This is a unanimous sliding window of length N (strict majority).
# ---------------------------------------------------------------------------

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def denoise_majority():
    pull(block)
    mov(isr, osr)                   # ISR = N (reload source)

    wrap_target()

    label("start")
    mov(x, isr)                     # X = N
    jmp(pin, "try_high")

    # ---- debounce low ----
    label("try_low")
    jmp(pin, "start")               # bounced high → restart
    jmp(x_dec, "try_low")
    set(pins, 0)
    label("hold_low")
    jmp(pin, "start")               # input went high → debounce high
    jmp("hold_low")

    # ---- debounce high ----
    label("try_high")
    jmp(pin, "high_ok")
    jmp("start")                    # bounced low → restart
    label("high_ok")
    jmp(x_dec, "try_high")
    set(pins, 1)
    label("hold_high")
    jmp(pin, "hold_high")           # stay until input goes low
    jmp("start")

    wrap()


class Denoiser:
    """Single-pin PIO de-noiser. Not coupled to QD — wire outs in the app."""

    def __init__(self, sm_id, pin_in, pin_out, n=15):
        _check_sm_id(sm_id)
        if not isinstance(n, int) or n < 1 or n > 31:
            raise ValueError("n must be 1..31 (odd recommended)")
        self._n = n
        self._pin_in = Pin(pin_in, Pin.IN, Pin.PULL_UP)
        self._pin_out = Pin(pin_out, Pin.OUT)
        self._sm = rp2.StateMachine(
            sm_id,
            denoise_majority,
            freq=125_000_000,
            jmp_pin=self._pin_in,
            set_base=self._pin_out,
        )
        self._sm.active(1)
        self._sm.put(n)                 # pulled once at program start

    def stop(self):
        self._sm.active(0)

    def deinit(self):
        self.stop()


def run():
    enc = QD(0, 14, 15)
    try:
        while True:
            print("Pos:", enc.position, enc.pin_a, enc.pin_b)
            time.sleep_ms(100)
    finally:
        enc.deinit()


if __name__ == "__main__":
    run()