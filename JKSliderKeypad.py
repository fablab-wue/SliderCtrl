# JKSlider keypad layout — edit this file to remap keys.
#
# Copy to the Pico with JKSlider.py. You do not need to be a programmer:
# change the names in LAYOUT to match the keys on your pad.
#
# Matrix: up to 4 rows x 4 columns.
#   KP_ROW1..4 = GP6..GP9  (ROW1 = upper keys)
#   KP_COL1..3 = GP10..GP12
#   KP_COL_4   = GP13      (used only if a row has 4 names)
#
# Discrete (not in this grid): STOP extra switch on GP5; OPTION extra on GP14.
# Pico also ORs PIN_BTN_AXIS_1..4 (GP21..18) when those pins are set.
#
# Names you may use (must match exactly):
#   MOVE_L  MOVE_R  FAST_L  FAST_R  STOP  A  B  C  OPTION  DELAY  TIMELAPSE
#   AXIS_1  AXIS_2  AXIS_3  AXIS_4  AXIS_5  AXIS_6
# Empty cell: None
#
# Two OPTION cells OR together. Both down at once → DOUBLE_OPTION
# (emergency halt with STOP).
#
# Ghosting: there is NO firmware filter. Without per-key diodes, three
# corners of a rectangle can fake a fourth key. Check your chords yourself.
# AXIS_1+AXIS_2+MOVE_L can ghost FAST_L on a diode-less 4x4.
#
# Silk on the stock 3x4 pad (ROW1 at the top):
#
#               COL1 GP10     COL2 GP11     COL3 GP12
#   ROW1 GP6    <  MOVE_L     D  DELAY      >  MOVE_R
#   ROW2 GP7    << FAST_L     T  TIMELAPSE  >> FAST_R
#   ROW3 GP8    A             B             C
#   ROW4 GP9    *  OPTION     0  STOP       *  OPTION
#
# Optional override: in SliderPins.py JKSlider dict set KEYPAD_LAYOUT to
# a tuple like LAYOUT below (that wins over this file).

LAYOUT = (
    ("MOVE_L", "DELAY", "MOVE_R"),       # KP_ROW1  GP6
    ("FAST_L", "TIMELAPSE", "FAST_R"),   # KP_ROW2  GP7
    ("A", "B", "C"),                     # KP_ROW3  GP8
    ("OPTION", "STOP", "OPTION"),        # KP_ROW4  GP9
)

# Optional 4x4 — left 3 cols same as LAYOUT; col4 AXIS_1 (top) .. AXIS_4.
# Uncomment the last line to scan KP_COL_4 (GP13).
LAYOUT_4X4 = (
    ("MOVE_L", "DELAY", "MOVE_R", "AXIS_1"),
    ("FAST_L", "TIMELAPSE", "FAST_R", "AXIS_2"),
    ("A", "B", "C", "AXIS_3"),
    ("OPTION", "STOP", "OPTION", "AXIS_4"),
)
# LAYOUT = LAYOUT_4X4
