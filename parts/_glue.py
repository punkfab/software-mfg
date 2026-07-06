"""_glue.py — shared parameters for the self-actuated hot-glue applicator tool.

A switchable end-effector that lays a hot-melt adhesive bead under robot control.
Designed to the same conventions as the shear (CONCEPT.md §5.2/5.3):
  * SWITCHABLE — mounts the tool-side kinematic coupling (parts/coupling_tool_side.py);
    the SAME interface as the shear, so the changer already knows how to grab it.
  * BUILD/BUY — the printed body + motorized stick feed are the "build"; the melt
    cartridge + nozzle (harvested from a cheap glue gun, or a cartridge-heater block)
    are the "buy", held off the body by a THERMAL BREAK so the print doesn't soften.
  * SELF-CONTAINED — heater power + feed-motor drive cross the joint on pogo pins
    (central bore). No process force reaches the arm (unlike the press/shear, gluing
    is nearly force-free — the load is thermal + a few N of feed back-pressure).

Axes: +X forward (nozzle points this way), Y = feed-motor shaft / width, +Z up
(coupling on top). Feeds a 7 mm "mini" glue stick. v1 geometry — iterate on the bench.
"""

import math

# --- coupling interface (mates parts/coupling_tool_side.py — identical to the shear) ---
COUPLING_OD = 48.0
COUPLING_T = 6.0
BORE_D = 12.0            # central bore: EPM / pogo pass-through (heater + motor power)
BOLT_R = 21.0
BOLT_CLR = 3.4          # M3 clearance
MOUNT_ANGLES = (30, 150, 270)

# --- body ---
BODY_X = 46.0           # forward depth (back stick entry -> front melt)
BODY_Y = 34.0           # width / feed-motor axis
BODY_H = 40.0           # height

# --- glue stick + feed channel (7 mm mini stick, fed along +X at mid-height) ---
STICK_D = 7.6           # 7 mm stick + clearance
STICK_Z = 22.0          # channel axis height

# --- feed drive: a small gearmotor on the -Y face turns a knurled wheel that grips
#     the stick against a spring idler (like a 3D-printer extruder) ---
DRIVE_X = -9.0          # feed location (near the back)
FEED_PILOT_D = 13.0     # gearmotor face-boss recess
FEED_SHAFT_CLR = 7.0    # shaft + wheel-hub pass-through
FEED_BOLT_SP = 18.0     # 2 motor bolts, spaced vertically
FEED_HOLE = 3.2         # M3
WHEEL_CAV_D = 16.0      # cavity housing the knurled drive wheel (opens into the channel)

# --- idler (presses the stick onto the drive wheel from above) ---
IDLER_PIN_X = -3.0
IDLER_PIN_D = 4.0       # pivot pin bore (along Y)
IDLER_Z = 32.0          # pivot height (above the channel)

# --- front: thermal-break nozzle bracket + bought melt cartridge beyond a gap ---
FRONT_MOUNT_SP = 20.0   # 2 self-tap bosses on the front face (either side of the channel)
FRONT_TAP = 2.5         # M3 self-tap pilot into printed plastic
THERMAL_GAP = 7.0       # air gap body -> hot cartridge (the thermal break)
HOTEND_BARREL_D = 12.0  # bought melt-cartridge barrel Ø the bracket clamps
NOZZLE_D = 2.0          # bead orifice (sets bead width)


def pos(r, angle_deg):
    return r * math.cos(math.radians(angle_deg)), r * math.sin(math.radians(angle_deg))
