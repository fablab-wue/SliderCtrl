import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import b4_logic as bl


class TestAxisMask(unittest.TestCase):
    def test_startup_keeps_axis_1_when_nothing_pressed(self):
        mask, invalid = bl.resolve_axis_mask((), 3, frozenset((1,)))
        self.assertEqual(mask, frozenset((1,)))
        self.assertFalse(invalid)

    def test_chords(self):
        cases = (
            ((1, 2), frozenset((1, 2))),
            ((1, 3), frozenset((1, 3))),
            ((2, 3), frozenset((2, 3))),
            ((1, 2, 3), frozenset((1, 2, 3))),
            ((2,), frozenset((2,))),
        )
        for pressed, want in cases:
            mask, invalid = bl.resolve_axis_mask(pressed, 3, frozenset((1,)))
            self.assertEqual(mask, want, pressed)
            self.assertFalse(invalid, pressed)

    def test_chords_4_and_5(self):
        cases = (
            ((4,), frozenset((4,))),
            ((5,), frozenset((5,))),
            ((1, 4), frozenset((1, 4))),
            ((1, 4, 5), frozenset((1, 4, 5))),
            ((4, 5), frozenset((4, 5))),
            ((1, 2, 3, 4, 5), frozenset((1, 2, 3, 4, 5))),
        )
        for pressed, want in cases:
            mask, invalid = bl.resolve_axis_mask(pressed, 5, frozenset((1,)))
            self.assertEqual(mask, want, pressed)
            self.assertFalse(invalid, pressed)

    def test_invalid_keeps_current(self):
        mask, invalid = bl.resolve_axis_mask((2,), 1, frozenset((1,)))
        self.assertEqual(mask, frozenset((1,)))
        self.assertTrue(invalid)
        mask, invalid = bl.resolve_axis_mask((1, 2, 3), 2, frozenset((1, 2)))
        self.assertEqual(mask, frozenset((1, 2)))
        self.assertTrue(invalid)
        mask, invalid = bl.resolve_axis_mask((5,), 2, frozenset((1,)))
        self.assertEqual(mask, frozenset((1,)))
        self.assertTrue(invalid)


class TestOled(unittest.TestCase):
    def test_format(self):
        self.assertEqual(bl.format_axis_oled(frozenset((1,))), "Ax 1")
        self.assertEqual(bl.format_axis_oled(frozenset((1, 3))), "Ax 1+3")
        self.assertEqual(bl.format_axis_oled(frozenset((3, 2, 1))), "Ax 1+2+3")
        self.assertEqual(bl.format_axis_oled(frozenset((1, 4, 5))), "Ax 1+4+5")
        self.assertEqual(bl.format_axis_oled(frozenset()), "Ax 1")


class TestQdDetents(unittest.TestCase):
    def test_round_half_up(self):
        self.assertEqual(bl.qd_detents(0), 0)
        self.assertEqual(bl.qd_detents(1), 0)
        self.assertEqual(bl.qd_detents(2), 1)
        self.assertEqual(bl.qd_detents(4), 1)
        self.assertEqual(bl.qd_detents(6), 2)

    def test_reverse_plus_minus_one_stays(self):
        base = 8
        d = bl.qd_detents(base)
        self.assertEqual(d, 2)
        self.assertEqual(bl.qd_detents(base - 1), d)
        self.assertEqual(bl.qd_detents(base + 1), d)


class TestEncoder(unittest.TestCase):
    def test_linear_and_log_modes(self):
        vmax = 128.0
        v, limited = bl.apply_encoder_steps(vmax / 8.0, 1, 1, vmax)
        self.assertFalse(limited)
        self.assertAlmostEqual(v, vmax / 8.0 + vmax / 128.0)

        v, limited = bl.apply_encoder_steps(vmax / 8.0, 1, 11, vmax)
        self.assertFalse(limited)
        self.assertAlmostEqual(v, (vmax / 8.0) * math.sqrt(2.0))

        v, limited = bl.apply_encoder_steps(vmax / 8.0, -1, 11, vmax)
        self.assertAlmostEqual(v, (vmax / 8.0) / math.sqrt(2.0))

        for mode in (1, 2, 3, 4, 11, 12, 13, 14):
            v, limited = bl.apply_encoder_steps(vmax / 8.0, 1, mode, vmax)
            self.assertFalse(limited)
            self.assertGreater(v, vmax / 8.0)

    def test_clamp_and_reverse_unclamp(self):
        vmax = 128.0
        v, limited = bl.apply_encoder_steps(vmax, 4, 1, vmax)
        self.assertTrue(limited)
        self.assertEqual(v, vmax)
        v, limited = bl.apply_encoder_steps(v, -1, 1, vmax)
        self.assertFalse(limited)
        self.assertLess(v, vmax)

        vmin = vmax / 128.0
        v, limited = bl.apply_encoder_steps(vmin, -4, 1, vmax)
        self.assertTrue(limited)
        self.assertEqual(v, vmin)
        v, limited = bl.apply_encoder_steps(v, 1, 1, vmax)
        self.assertFalse(limited)
        self.assertGreater(v, vmin)

    def test_boot_and_clamp_enter(self):
        vmax = 100.0
        self.assertAlmostEqual(bl.rotary_boot_value(vmax), vmax / 8.0)
        self.assertTrue(bl.clamp_enter(False, True))
        self.assertFalse(bl.clamp_enter(True, True))
        self.assertFalse(bl.clamp_enter(False, False))
        self.assertFalse(bl.clamp_enter(True, False))


class TestAxisGestures(unittest.TestCase):
    def test_exclusive_short(self):
        mask, invalid = bl.exclusive_axis(3, 6, frozenset((1,)))
        self.assertEqual(mask, frozenset((3,)))
        self.assertFalse(invalid)
        mask, invalid = bl.exclusive_axis(7, 6, frozenset((1,)))
        self.assertEqual(mask, frozenset((1,)))
        self.assertTrue(invalid)

    def test_toggle_refuses_empty(self):
        mask, invalid = bl.toggle_axis(frozenset((2,)), 2, 6)
        self.assertEqual(mask, frozenset((2,)))
        self.assertTrue(invalid)
        mask, invalid = bl.toggle_axis(frozenset((2, 5)), 5, 6)
        self.assertEqual(mask, frozenset((2,)))
        self.assertFalse(invalid)
        mask, invalid = bl.toggle_axis(frozenset((2,)), 5, 6)
        self.assertEqual(mask, frozenset((2, 5)))
        self.assertFalse(invalid)

    def test_suffix_and_prefix(self):
        mask, invalid = bl.axis_range_suffix(3, 6, frozenset((1,)))
        self.assertEqual(mask, frozenset((3, 4, 5, 6)))
        self.assertFalse(invalid)
        mask, invalid = bl.axis_range_prefix(3, 6, frozenset((5,)))
        self.assertEqual(mask, frozenset((1, 2, 3)))
        self.assertFalse(invalid)
        mask, invalid = bl.axis_range_suffix(1, 6, frozenset((2,)))
        self.assertEqual(mask, frozenset((1, 2, 3, 4, 5, 6)))
        self.assertFalse(invalid)
        mask, invalid = bl.axis_range_suffix(4, 3, frozenset((1,)))
        self.assertEqual(mask, frozenset((1,)))
        self.assertTrue(invalid)

    def test_apply_short_long_option(self):
        mask, invalid = bl.apply_axis_short(2, False, frozenset((1,)), 6)
        self.assertEqual(mask, frozenset((2,)))
        mask, invalid = bl.apply_axis_short(5, True, frozenset((2,)), 6)
        self.assertEqual(mask, frozenset((2, 5)))
        mask, invalid = bl.apply_axis_long(3, False, frozenset((1,)), 6)
        self.assertEqual(mask, frozenset((3, 4, 5, 6)))
        mask, invalid = bl.apply_axis_long(3, True, frozenset((5,)), 6)
        self.assertEqual(mask, frozenset((1, 2, 3)))

    def test_update_chord_then_release_ignores_shorts(self):
        mask, invalid = bl.update_axis_selection(
            (1, 3), (), (), False, frozenset((1,)), 6, 0
        )
        self.assertEqual(mask, frozenset((1, 3)))
        self.assertFalse(invalid)
        mask, invalid = bl.update_axis_selection(
            (), (3,), (), False, frozenset((1, 3)), 6, 2
        )
        self.assertEqual(mask, frozenset((1, 3)))
        self.assertFalse(invalid)

    def test_update_single_short_and_long(self):
        mask, invalid = bl.update_axis_selection(
            (), (2,), (), False, frozenset((1,)), 6, 1
        )
        self.assertEqual(mask, frozenset((2,)))
        mask, invalid = bl.update_axis_selection(
            (3,), (), (3,), False, frozenset((1,)), 6, 0
        )
        self.assertEqual(mask, frozenset((3, 4, 5, 6)))
        mask, invalid = bl.update_axis_selection(
            (3,), (), (3,), True, frozenset((1,)), 6, 0
        )
        self.assertEqual(mask, frozenset((1, 2, 3)))


class TestMjPct(unittest.TestCase):
    def test_slots_same_pct_on_mask(self):
        self.assertEqual(
            bl.mj_pct_slots(frozenset((1, 3)), 100, 6),
            (100, 0, 100, 0, 0, 0),
        )
        self.assertEqual(
            bl.mj_pct_slots(frozenset((2, 5)), -100, 5),
            (0, -100, 0, 0, -100),
        )

    def test_from_axis_map_and_order(self):
        self.assertEqual(
            bl.mj_pct_from_axis_map({3: 40, 1: -80}, 6),
            (-80, 0, 40, 0, 0, 0),
        )
        self.assertEqual(bl.selected_axis_order({3, 1}), (1, 3))
        self.assertEqual(bl.selected_axis_order(frozenset()), (1,))


class TestOledSix(unittest.TestCase):
    def test_six_axis(self):
        self.assertEqual(
            bl.format_axis_oled(frozenset((1, 2, 3, 4, 5, 6))),
            "Ax 1+2+3+4+5+6",
        )
        self.assertEqual(bl.format_axis_oled(frozenset((6,))), "Ax 6")


if __name__ == "__main__":
    unittest.main()
