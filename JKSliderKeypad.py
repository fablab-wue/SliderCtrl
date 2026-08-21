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
#
# Names you may use (must match exactly):
#   MOVE_L  MOVE_R  FAST_L  FAST_R  STOP  A  B  C  OPTION  DELAY  TIMELAPSE
# Empty cell: None
# Unknown names (e.g. MOVE_L2) are ignored until a later 2-axis UI.
#
# Two OPTION cells OR together. Both down at once → DOUBLE_OPTION
# (emergency halt with STOP).
#
# Ghosting: there is NO firmware filter. Without per-key diodes, three
# corners of a rectangle can fake a fourth key. Check your chords yourself.
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

# 4x4 example — uncomment and replace LAYOUT to use KP_COL_4 (GP13).
# Empty cell = None (not the string "None").
# MOVE_L2 / MOVE_R2 are ignored until a later 2-axis UI.
#
# LAYOUT = (
#     ("MOVE_L", "DELAY", "MOVE_R", "A"),
#     ("FAST_L", "TIMELAPSE", "FAST_R", "B"),
#     ("MOVE_L2", None, "MOVE_R2", "C"),
#     ("OPTION", "STOP", "OPTION", None),
# )
