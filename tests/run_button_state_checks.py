import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import time

import button_state


class FakeClock:
    def __init__(self):
        self.now = 0

    def ticks_ms(self):
        return self.now

    def ticks_diff(self, a, b):
        return int(a - b)


clock = FakeClock()
orig_ticks_ms = getattr(time, "ticks_ms", None)
orig_ticks_diff = getattr(time, "ticks_diff", None)
time.ticks_ms = clock.ticks_ms
time.ticks_diff = clock.ticks_diff

try:
    left = button_state.ButtonState(debounce_ms=20, long_ms=1000)
    right = button_state.ButtonState(debounce_ms=20, long_ms=1000)

    clock.now = 50
    left.update(True)
    clock.now = 70
    left.update(True)
    clock.now = 190
    left.update(False)
    clock.now = 210
    left.update(False)

    res = button_state.resolve_move_semantics(left, right, False, 333)
    assert res.direction == -1, res
    assert res.short_release_latched, res

    left = button_state.ButtonState(debounce_ms=20, long_ms=1000)
    right = button_state.ButtonState(debounce_ms=20, long_ms=1000)

    clock.now = 50
    left.update(True)
    clock.now = 70
    left.update(True)
    clock.now = 500
    left.update(False)
    clock.now = 520
    left.update(False)

    res = button_state.resolve_move_semantics(left, right, False, 333)
    assert res.direction == -1, res
    assert res.long_release_stop, res

    assert button_state.allow_move_out_of_soft_limit(10.0, -1, 0.0, 100.0)
    assert not button_state.allow_move_out_of_soft_limit(0.0, 1, 0.0, 100.0)
    assert button_state.allow_move_out_of_soft_limit(50.0, 1, 0.0, 100.0)

    print("button_state checks passed")
finally:
    if orig_ticks_ms is not None:
        time.ticks_ms = orig_ticks_ms
    else:
        delattr(time, "ticks_ms")
    if orig_ticks_diff is not None:
        time.ticks_diff = orig_ticks_diff
    else:
        delattr(time, "ticks_diff")
