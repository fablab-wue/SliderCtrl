import time
import unittest

import button_state


class TestButtonState(unittest.TestCase):
    def setUp(self):
        self._real_ticks_ms = time.ticks_ms
        self._now = 0
        time.ticks_ms = lambda: self._now

    def tearDown(self):
        time.ticks_ms = self._real_ticks_ms

    def _advance(self, ms):
        self._now += ms

    def test_short_release_latches_move(self):
        left = button_state.ButtonState(debounce_ms=20, long_ms=1000)
        right = button_state.ButtonState(debounce_ms=20, long_ms=1000)

        self._advance(50)
        left.update(True)
        self._advance(20)
        left.update(True)
        self._advance(120)
        left.update(False)
        self._advance(20)
        left.update(False)

        result = button_state.resolve_move_semantics(left, right, False, 333)
        self.assertEqual(result.direction, -1)
        self.assertTrue(result.short_release_latched)

    def test_long_release_stops_move(self):
        left = button_state.ButtonState(debounce_ms=20, long_ms=1000)
        right = button_state.ButtonState(debounce_ms=20, long_ms=1000)

        self._advance(50)
        left.update(True)
        self._advance(20)
        left.update(True)
        self._advance(420)
        left.update(False)
        self._advance(20)
        left.update(False)

        result = button_state.resolve_move_semantics(left, right, False, 333)
        self.assertEqual(result.direction, -1)
        self.assertTrue(result.long_release_stop)

    def test_allow_move_out_of_soft_limit(self):
        self.assertTrue(
            button_state.allow_move_out_of_soft_limit(10.0, -1, 0.0, 100.0)
        )
        self.assertFalse(
            button_state.allow_move_out_of_soft_limit(0.0, 1, 0.0, 100.0)
        )
        self.assertTrue(
            button_state.allow_move_out_of_soft_limit(50.0, 1, 0.0, 100.0)
        )


if __name__ == "__main__":
    unittest.main()
