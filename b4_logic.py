# B4Slider — machine-free helpers (axis mask, encoder curves, OLED text).
#
# Safe to import from CPython tests. No machine / QD / uasyncio.


def cap_panel_axes(axis_count):
    """Packed live channels on the panel: 1..6."""
    n = int(axis_count)
    if n < 1:
        n = 1
    if n > 6:
        n = 6
    return n


def _cur_mask(current):
    cur = frozenset(int(a) for a in current)
    if not cur:
        cur = frozenset((1,))
    return cur


def selected_axis_order(mask):
    """Sorted 1-based axes for joystick 1st/2nd mapping."""
    return tuple(sorted(int(a) for a in _cur_mask(mask)))


def resolve_axis_mask(pressed, axis_count, current):
    """Return (mask, invalid) from currently held AXIS buttons.

    ``pressed`` is 1-based axis numbers held now. Empty hold keeps ``current``.
    Any held axis above ``axis_count`` is invalid and keeps ``current``.
    """
    cur = _cur_mask(current)
    held = frozenset(int(a) for a in pressed)
    if not held:
        return cur, False
    n = cap_panel_axes(axis_count)
    for a in held:
        if a < 1 or a > n:
            return cur, True
    return held, False


def exclusive_axis(n, axis_count, current):
    """Short AXIS_N: select only N."""
    cur = _cur_mask(current)
    last = cap_panel_axes(axis_count)
    n = int(n)
    if n < 1 or n > last:
        return cur, True
    return frozenset((n,)), False


def toggle_axis(current, n, axis_count):
    """OPTION+short AXIS_N. Refuse empty (invalid=True, keep current)."""
    cur = _cur_mask(current)
    last = cap_panel_axes(axis_count)
    n = int(n)
    if n < 1 or n > last:
        return cur, True
    s = set(cur)
    if n in s:
        if len(s) <= 1:
            return cur, True
        s.remove(n)
    else:
        s.add(n)
    return frozenset(s), False


def axis_range_suffix(n, axis_count, current):
    """Long AXIS_N: N..last (1..N-1 off)."""
    cur = _cur_mask(current)
    last = cap_panel_axes(axis_count)
    n = int(n)
    if n < 1 or n > last:
        return cur, True
    return frozenset(range(n, last + 1)), False


def axis_range_prefix(n, axis_count, current):
    """OPTION+long AXIS_N: 1..N."""
    cur = _cur_mask(current)
    last = cap_panel_axes(axis_count)
    n = int(n)
    if n < 1 or n > last:
        return cur, True
    return frozenset(range(1, n + 1)), False


def apply_axis_short(n, option, current, axis_count):
    if option:
        return toggle_axis(current, n, axis_count)
    return exclusive_axis(n, axis_count, current)


def apply_axis_long(n, option, current, axis_count):
    if option:
        return axis_range_prefix(n, axis_count, current)
    return axis_range_suffix(n, axis_count, current)


def update_axis_selection(
    held, shorts, longs, option, current, axis_count, prev_held_n=0
):
    """One main-loop tick of AXIS gestures.

    Two-or-more held: chord preview (held set). After releasing a chord,
    ignore shorts that fire on that release. Single-key long then short
    as specified. Returns ``(mask, invalid)``.
    """
    cur = _cur_mask(current)
    last = cap_panel_axes(axis_count)
    held_set = frozenset(int(a) for a in held)
    held_n = len(held_set)
    if held_n >= 2:
        return resolve_axis_mask(held_set, last, cur)
    if int(prev_held_n) >= 2:
        return cur, False
    for n in longs:
        n = int(n)
        if n in held_set and held_n == 1:
            return apply_axis_long(n, option, cur, last)
    for n in shorts:
        return apply_axis_short(int(n), option, cur, last)
    return cur, False


def mj_pct_slots(mask, signed_pct, axis_count):
    """Same ±pct on every selected axis; 0 elsewhere. Length ``axis_count`` (cap 6)."""
    n = cap_panel_axes(axis_count)
    slots = [0] * n
    pct = int(signed_pct)
    for a in mask:
        ia = int(a)
        if 1 <= ia <= n:
            slots[ia - 1] = pct
    return tuple(slots)


def mj_pct_from_axis_map(axis_to_pct, axis_count):
    """Independent percents; missing axes 0."""
    n = cap_panel_axes(axis_count)
    slots = [0] * n
    if not axis_to_pct:
        return tuple(slots)
    for a, pct in axis_to_pct.items():
        ia = int(a)
        if 1 <= ia <= n:
            slots[ia - 1] = int(pct)
    return tuple(slots)


def format_axis_oled(mask):
    """``{1, 3}`` → ``Ax 1+3``."""
    axes = sorted(int(a) for a in mask)
    if not axes:
        axes = [1]
    out = "Ax "
    i = 0
    while i < len(axes):
        if i:
            out += "+"
        out += str(axes[i])
        i += 1
    return out


def qd_detents(raw, div=4, round_add=2):
    """Raw QD counts → detents: ``(raw + 2) // 4`` (absorbs ±1 reverse error)."""
    d = int(div)
    if d < 1:
        d = 1
    return (int(raw) + int(round_add)) // d


def encoder_range(vmax):
    vmax = float(vmax)
    return vmax / 128.0, vmax


def clamp_encoder(value, vmax):
    vmin, vmax = encoder_range(vmax)
    v = float(value)
    limited = False
    if v < vmin:
        v = vmin
        limited = True
    elif v > vmax:
        v = vmax
        limited = True
    return v, limited


def rotary_boot_value(vmax):
    v, _ = clamp_encoder(float(vmax) / 8.0, vmax)
    return v


def clamp_enter(was_at_limit, now_at_limit):
    """True only on the transition into min/max, not while held at the rail."""
    return bool(now_at_limit) and not bool(was_at_limit)


_LINEAR_DEN = {
    1: 128.0,
    2: 256.0,
    3: 512.0,
    4: 1024.0,
}


def apply_encoder_steps(value, delta, mode, vmax):
    """Apply ``delta`` detent steps. Returns ``(value, limited)``."""
    mode = int(mode)
    v = float(value)
    n = int(delta)
    if n == 0:
        return clamp_encoder(v, vmax)
    den = _LINEAR_DEN.get(mode)
    if den is not None:
        v += n * (float(vmax) / den)
        return clamp_encoder(v, vmax)
    if 11 <= mode <= 14:
        factor = 2.0 ** (0.5 ** (mode - 10))
        if n > 0:
            i = 0
            while i < n:
                v *= factor
                i += 1
        else:
            inv = 1.0 / factor
            i = 0
            while i < -n:
                v *= inv
                i += 1
        return clamp_encoder(v, vmax)
    return clamp_encoder(v, vmax)
