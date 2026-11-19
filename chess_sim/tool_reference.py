"""The accepted B5 tool + camera arrangement (commit 37e7232), in sim frames.

Source of truth: ``reports/b5_reference_config.json`` at 37e7232. Nothing here is
re-derived; the constants below are that file's values. Only the frame map is
computed, and it is computed from geometry (see ``frame_map`` and the check in
``tools/a2_reference_check.py``).

Frames
------
G  ``so101:gripper`` body frame - the sim's calibrated chain frame.
W  the b5 ``tool_mount`` frame - the reference scene's world.

They differ by a pure roll about the approach axis::

    W = Rz(-90 deg) * G            zero translation
    residual: 4e-5 mm, 1e-8 deg    (recomputable, not assumed)

Consequence, measured on the compiled model: the reference base and the pack's
base are the *same* placement once mapped into one frame (0.000000 deg, 4e-5 mm).
The reference's ``rotation_tool`` matrix is NOT a tilted base pose: it is the
base body rotation Ry(+90) composed with the STL's own mesh frame, which MuJoCo
applies to any mesh geom with no explicit quat. Treating it as a body rotation
would tilt the tool by the mesh frame's ~10 deg.

Camera
------
The accepted camera is the 4c calibrated pose moved 15 mm along its optical
axis, with the axis swung into x-z, roll zeroed then rolled 180, expressed in
the reference's tool frame. Its recorded values are in W; the sim needs them in
G, which is a -90 deg roll of position and orientation (``camera_in_gripper``).
The derived mount, not the raw W numbers, is what must be compared against the
compiled model.

Aspect (recorded, not implied): 640x480 landscape, ``fovy`` from ``fy``,
roll 180.0 deg. ``fov_x`` back-computed from that aspect must equal
42.0797756766466 deg; a different render aspect would surface here.
"""
from __future__ import annotations

import math

import numpy as np

# --- verbatim from reports/b5_reference_config.json ----------------------
MOUNT_FACE_OFFSET_MM = 13.25
TIP_PLANE_Z_MM = -110.6

PLACEMENTS = {
    "base": {
        "body": "parallel_base",
        "rotation_tool": [[-0.006386171, -0.180990623, 0.983464087],
                          [0.999159314, -0.040982361, -0.00105405],
                          [0.040495454, 0.982630572, 0.181100187]],
        "body_pos_tool_mm": [0.0, 0.0, 13.25],
        "body_quat_wxyz": [0.7071067813090637, -1.6487944761902842e-10,
                           0.7071067810640314, -8.530708659421611e-11],
        "cad_rotation": [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]],
        "cad_offset_mm": [0.0, 0.0, 13.25],
    },
    "jaw_left": {
        "body": "jaw_left",
        "rotation_tool": [[-0.316298136, 0.890241775, -0.327757641],
                          [-0.948640949, -0.294634441, 0.115199377],
                          [0.005986608, 0.347361668, 0.937712126]],
        "body_pos_tool_mm": [-0.3, -27.828, -76.6],
        "body_quat_wxyz": [0.7071067805789976, 8.057445012036852e-10,
                           -3.46864917848608e-10, -0.7071067817940981],
        "cad_rotation": [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        "cad_offset_mm": [-0.3, -27.828, -76.6],
    },
    "jaw_right": {
        "body": "jaw_right",
        "rotation_tool": [[0.316298136, -0.890241776, 0.327757638],
                          [0.948640949, 0.294634441, -0.115199378],
                          [0.00598661, 0.347361665, 0.937712127]],
        "body_pos_tool_mm": [-0.3, 27.828, -76.6],
        "body_quat_wxyz": [0.7071067817940975, -3.46864959738121e-10,
                           -8.057442801406447e-10, 0.7071067805789982],
        "cad_rotation": [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        "cad_offset_mm": [-0.3, 27.828, -76.6],
    },
    "gear": {
        "body": "gear",
        "rotation_tool": [[1.67e-07, -0.725156023, -0.688584594],
                          [2.463e-06, -0.688584594, 0.725156023],
                          [-1.0, -1.817e-06, 1.671e-06]],
        "body_pos_tool_mm": [0.0, 0.0, 12.75],
        "body_quat_wxyz": [0.1393112425333196, -0.6932477028727906,
                           0.13930941312780656, 0.6932480707361599],
        "cad_rotation": [[0.0, -0.386309, -0.92237],
                         [0.0, -0.92237, 0.386309],
                         [-1.0, 0.0, 0.0]],
        "cad_offset_mm": [0.0, 0.0, 12.75],
    },
}

CAMERA = {
    "position_mm": [60.3426, -0.4146, -23.7402],
    "optical_axis": [-0.4944839849556766, 0.0, -0.8691867397874603],
    "right": [0.0, -1.0, 0.0],
    "up": [0.8691867397874603, 0.0, -0.4944839849556766],
    "rotation": [[0.0, 0.8691867397874603, 0.4944839849556766],
                 [-1.0, 0.0, 0.0],
                 [0.0, -0.4944839849556766, 0.8691867397874603]],
    "roll_deg": 180.0,
    "intrinsics": {"fx": 831.897, "fy": 833.014, "cx": 320.0, "cy": 240.0,
                   "width": 640, "height": 480},
    "calibrated_pose": {
        "position_mm": [67.656, -2.916, -10.885],
        "rotation": [[-0.845366, -0.218268, 0.48756],
                     [0.155227, -0.973702, -0.166759],
                     [0.511136, -0.06529, 0.857016]],
        "shift_along_axis_mm": 15.0,
    },
}

ASPECT = {"width": 640, "height": 480, "orientation": "landscape",
          "fovy_from": "fy", "roll_deg": 180.0,
          "fov_y_deg": 32.14448310040661, "fov_x_deg": 42.0797756766466}

MEASURED = {
    "base_bbox_mm": [[-35.25, -32.0, -55.95], [87.3898, 32.0, 0.0]],
    "jaw_left_bbox_mm": [[-30.896, -30.873, -110.6], [30.2961, 20.172, -46.1]],
    "jaw_right_bbox_mm": [[-30.8961, -20.172, -110.6], [30.2961, 30.873, -46.1]],
    "gear_bbox_mm": [[-13.7712, -13.8152, -51.25], [13.7712, 13.8154, -45.25]],
    "tip_depth_mm": 110.6,
    "jaw_in_frustum_pct": 2.2393,
}

# --- frame map (computed) ------------------------------------------------
def rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame_map() -> dict:
    """G -> W as (rotation, translation_mm). Pure -90 deg roll about +z."""
    R = rz(-90.0)
    return {"R": R, "t_mm": np.zeros(3),
            "note": "W = Rz(-90 deg) * G about +z, zero translation"}


def w_to_g(R_w, t_mm):
    """Map a reference-frame rotation/translation into the gripper frame."""
    Z = rz(90.0)
    return Z @ np.asarray(R_w, float), Z @ np.asarray(t_mm, float)


def part_in_gripper(key: str):
    """Reference mesh placement part -> so101:gripper, metres."""
    p = PLACEMENTS[key]
    R, t = w_to_g(p["cad_rotation"], p["cad_offset_mm"])
    return R, t / 1000.0


def camera_in_gripper():
    """Reference camera (pos_m, R) expressed in so101:gripper."""
    R, t = w_to_g(CAMERA["rotation"], CAMERA["position_mm"])
    return t / 1000.0, R


def camera_xyaxes():
    """MuJoCo camera xyaxes (x then y), in the gripper body frame."""
    _, R = camera_in_gripper()
    return (*R[:, 0].tolist(), *R[:, 1].tolist())


def fovy_deg() -> float:
    """Vertical FOV from fy and the image height (the sim's fovy)."""
    i = CAMERA["intrinsics"]
    return math.degrees(2.0 * math.atan(i["cy"] / i["fy"]))


def fovx_from_aspect(width: int, height: int, fovy_degrees: float) -> float:
    """Horizontal FOV MuJoCo would produce at (width, height)."""
    return math.degrees(2.0 * math.atan(math.tan(math.radians(fovy_degrees) / 2.0)
                                        * width / height))


# --- variants (select exactly one; nothing is applied by importing) ------
VARIANTS = {
    "camera_only": "keep the A1 mount and tool; add the reference camera",
    "pose": "also pose the tool at the reference opening (gripper -1.247 rad)",
    "rotate_mount": "also rotate the tool -90 deg about the approach axis",
}
VARIANT = "camera_only"

# jaw reference opening, measured: the jaw cloud matches this gripper angle to
# 0.000 mm along the slide (bisection on the compiled left-jaw x minimum;
# -20.172 mm is the reference's own x minimum mapped into G).
REFERENCE_GRIPPER_RAD = -0.347131
# the reference's gear pose needs gripper +1.111 rad; the joint range is
# [-1.598, 0], so the gear pose is NOT reachable and cannot be ported. At the
# jaw-matched angle the gear stands 83.6 deg from the reference.
REFERENCE_GEAR_GRIPPER_RAD = +1.111
GEAR_OUT_OF_RANGE = True


def summary() -> dict:
    """Everything a startup line / /api/status field needs."""
    return {
        "variant": VARIANT,
        "variant_note": VARIANTS[VARIANT],
        "reference_commit": "37e7232",
        "tip_depth_mm": MEASURED["tip_depth_mm"],
        "tip_depth_cad_mm": 110.45,
        "jaw_in_frustum_pct": MEASURED["jaw_in_frustum_pct"],
        "aspect": dict(ASPECT),
        "fovy_deg": round(fovy_deg(), 6),
        "fovx_at_640x480_deg": round(fovx_from_aspect(640, 480, fovy_deg()), 6),
        "reference_gripper_rad": REFERENCE_GRIPPER_RAD,
        "reference_gear_gripper_rad": REFERENCE_GEAR_GRIPPER_RAD,
        "gear_reachable": not GEAR_OUT_OF_RANGE,
    }
