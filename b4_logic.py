# B4Slider — machine-free helpers (axis mask, encoder curves, OLED text).
#
# Safe to import from CPython tests. No machine / QD / uasyncio.


def resolve_axis_mask(pressed, axis_count, current):
    """Return (mask, invalid) from currently held AXIS buttons.

    ``pressed`` is 1-based axis numbers held now. Empty hold keeps ``current``.
    Any held axis above ``axis_count`` is invalid and keeps ``current``.
    """
    cur = frozenset(int(a) for a in current)
    if not cur:
        cur = frozenset((1,))
    held = frozenset(int(a) for a in pressed)
    if not held:
        return cur, False
    n = int(axis_count)
    if n < 1:
        n = 1
    for a in held:
        if a < 1 or a > n:
            return cur, True
    return held, False


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
