"""The written-down transform chain for the private CAD parallel tool.

One place for the numbers. `docs/TOOL_FRAME.md` explains them; the
orientation regression (`tests/test_tool_orientation.py`) measures the built
model against them. Nothing here is used to place geometry - the pack's own
MJCF does that. These are the values the pack must keep producing.

Frames, in order:

    so101:gripper          the calibrated chain frame (wrist-roll output body)
      |  mount             profile['mount_position'] / mount_rotation
      v
    cad:parallel_base      the tool base *body*; jaw bodies hang off it
      |  base_geom         the merged base part's visual geom offset
      v
    base part frame        the frame gripper_base_1__solid1.stl is authored in
      |  jaw_body + jaw_geom
      v
    jaw part frame         the frame gripper_jaw_1__solid1.stl is authored in

Millimetres and unit quaternions in MuJoCo order (w, x, y, z).
"""
from __future__ import annotations

# so101:gripper -> cad:parallel_base (the profile mount; == cad_tool.install)
MOUNT_POS_MM = (9.462, 23.429, -62.577)
MOUNT_QUAT = (0.707107, 0.0, 0.0, -0.707107)

# cad:parallel_base -> base part file frame (the pack's base visual geom)
BASE_GEOM_POS_MM = (23.4292, -9.46249, 75.8269)
BASE_GEOM_QUAT = (0.0, 0.707107, 0.0, -0.707107)

# The same composition, collapsed: gripper <- base part file frame.
#   g_x = -p_y, g_y = p_z, g_z = 13.25 - p_x
BASE_PART_IN_GRIPPER_R = ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0))
BASE_PART_IN_GRIPPER_T_MM = (0.0, 0.0, 13.25)

# cad:parallel_base -> jaw bodies (both are children of the tool base)
JAW_BODY_POS_MM = {
    "left": (4.879, -41.462, 8.127),
    "right": (41.979, 22.537, 8.127),
}
JAW_BODY_QUAT = {
    "left": (0.5, -0.5, 0.5, 0.5),
    "right": (0.5, 0.5, 0.5, -0.5),
}

# jaw body -> jaw part file frame (the pack's jaw visual geom)
JAW_GEOM_POS_MM = (22.0, -18.7, 19.0)
JAW_GEOM_QUAT = (0.0, -0.707107, 0.0, 0.707107)

# --- what the chain has to produce ---------------------------------------
# Base part file frame, from the merged Complete_Fixed_Jaw STL.
BASE_PART_BBOX_MM = ((13.25, -32.0, -35.25), (69.2, 32.0, 23.45))
# Deepest jaw vertex below the gripper body origin. Block 2 measured 110.450 on
# both the 5x and 5y packs by max(-z) over every jaw geom vertex; 110.27/110.60
# were a vertex-band average and an extremum that do not reproduce under one rule.
TIP_DEPTH_MM = 110.450
TIP_DEPTH_TOL_MM = 0.05
# Pinion/gear mesh origin in the gripper frame: -45.25 + 58.0.
GEAR_IN_GRIPPER_MM = (0.0, 0.0, 12.75)
# Base top face sits on the mount plane (mount y is 23.429).
BASE_TOP_Y_MM = 23.45
# Jaw body centres in the gripper frame, three definitions, measured from the
# built model. The 2026-11-19 plan quoted (-0.300, -/+27.828, -76.600) as the
# pre-5y body-centre placement; none of these three reproduces it (the quoted
# triple has the pair spanning y, the built pair spans x - see TOOL_FRAME.md).
JAW_BODY_ORIGIN_MM = {"left": (-32.000, 18.550, -54.450),
                      "right": (32.000, -18.550, -54.450)}
JAW_VERTEX_CENTROID_MM = {"left": (-5.786, 5.268, -56.579),
                          "right": (5.786, -5.269, -56.579)}
JAW_BBOX_MID_MM = {"left": (9.477, -0.150, -78.200),
                   "right": (-9.478, 0.150, -78.200)}
CAD_BODY_CENTRE_QUOTED_MM = (-0.300, -27.828, -76.600)

# Left and right jaw meshes are mirror images: max point-to-point error.
SYMMETRY_TOL_MM = 0.01
