"""The aim ray — one module, one definition, for the whole live stack.

`tip-center.md` fixes the vocabulary and this module is its only implementation:

===============  ====================================================  =========
name              definition                                           pose dep.
===============  ====================================================  =========
`TIP_GRIPPER`     the jaw tip, ``(0, 0, -0.1106)`` m in               no
                  ``so101:gripper``
`TIP_PIXEL`       `TIP_GRIPPER` projected through the calibrated        **no**
                  camera mount — a constant.  Diagnostic only.
`AIM_WORLD(q)`    the ray jaw origin -> tip centre -> straight out,     yes
                  extended to the *real* board plane
`AIM_PIXEL(q)`    `AIM_WORLD` projected through the same mount          yes
===============  ====================================================  =========

Two things this module refuses to blur:

* `TIP_PIXEL` constancy is an *identity*, not a measurement.  The camera and
  the tip are rigid on the same body, so the pixel cannot move under any joint
  configuration.  It is exposed as `tip_pixel()` so the regression test can
  assert it, and it is labelled ``diagnostic`` everywhere.  Never quote it as a
  finding.
* `AIM_PIXEL` is the one that moves, and in the confirmed landscape +
  camera-roll -90 arrangement it moves along a single image direction: the tool
  axis projects to a fixed image *line*, and the aim point is where that line
  meets the board plane, so it can only slide along the line.  Rotating the
  wrist about the tool axis moves neither the line nor the aim.

The plane is the board the arm is actually over, recovered from the ArUco /
ChArUco markers by PnP and carried into the arm-base frame through the
calibrated camera mount — **not** the simulator's board plane.  Because a
2-marker PnP is noisy (10-20 mm along the plane normal today), recent solves
are averaged by `PlaneTracker` in plane coordinates (normal + offset), which is
well defined and cannot average a rotation wrongly.

Frame conventions, stated once:

* ``camera_in_tool`` is stored in **OpenGL** camera axes (+x right, +y up,
  -z view); ``cv2.solvePnP`` returns **OpenCV** axes (+z view).  The two differ
  by ``diag(1, -1, -1)`` and that flip is the difference between a plane that
  reconstructs level to ~4 deg and one that reconstructs 62 deg off (measured
  2026-11-24 on the 21 phase-3 captures, see `tools/aim_validate.py`).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

#: The physical jaw tip in ``so101:gripper``, metres (tip-center.md §1).
TIP_GRIPPER_M = np.array([0.0, 0.0, -0.1106])

#: GL (stored) camera axes -> OpenCV camera axes.  Proper rotation, det +1.
GL_TO_CV = np.diag([1.0, -1.0, -1.0])

#: Board half-size used to decide whether an aim lands on the board.
DEFAULT_BOARD_HALF_MM = 122.0
#: Below this |axis . normal| the ray grazes the plane and the intersection is
#: not worth reporting (0.12 ~= 7 deg).
MIN_RAY_COS = 0.12
#: A plane older than this is reported stale (the arm may have been restarted,
#: the board may have moved).  30 s is generous because the plane is a static
#: property: age only matters as a "was this ever measured" test, and the
#: per-solve quality is reported separately.
DEFAULT_PLANE_MAX_AGE_S = 30.0


# --------------------------------------------------------------------- camera
@dataclass(frozen=True)
class CameraMount:
    """Camera pose in ``so101:gripper`` plus the lens, in metres/pixels."""

    position: np.ndarray            # camera origin in the tool frame, m
    rotation_gl: np.ndarray         # tool -> GL camera axes
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 640
    height: int = 480

    @classmethod
    def from_calibration(cls, calibration) -> "CameraMount":
        """Build from a ``WristCalibration`` or the raw calibration dict."""
        if isinstance(calibration, dict):
            matrix = np.asarray(calibration["camera_in_tool"], float)
            fx, fy, cx, cy = (float(v) for v in calibration["intrinsics"])
            width = int(calibration.get("width", 640))
            height = int(calibration.get("height", 480))
        else:                                   # WristCalibration
            matrix = np.asarray(calibration.camera_in_tool, float)
            fx, fy, cx, cy = (float(v) for v in calibration.intrinsics)
            width, height = int(calibration.width), int(calibration.height)
        return cls(position=matrix[:3, 3].copy(),
                   rotation_gl=matrix[:3, :3].copy(),
                   fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)

    # ---------------------------------------------------------------- helper
    def camera_to_tool(self, rotation_cv: np.ndarray) -> np.ndarray:
        """A rotation given in OpenCV camera axes, expressed in the tool frame."""
        return self.rotation_gl @ GL_TO_CV @ np.asarray(rotation_cv, float)

    def project(self, pose_base_tool: np.ndarray, point_base) -> tuple:
        """Pixel and depth of an arm-base point, seen through a tool pose.

        Returns ``((u, v), depth_m)``; depth is the OpenCV +z distance, so a
        point behind the camera comes back with a negative depth and a pixel
        that must be ignored.
        """
        pose = np.asarray(pose_base_tool, float)
        local = np.linalg.inv(pose) @ np.array([*np.asarray(point_base, float), 1.0])
        camera = self.rotation_gl @ GL_TO_CV
        cam = camera.T @ (local[:3] - self.position)
        depth = float(cam[2])
        if abs(depth) < 1e-9:
            return (float("nan"), float("nan")), 0.0
        u = self.cx + self.fx * cam[0] / depth
        v = self.cy + self.fy * cam[1] / depth
        return (float(u), float(v)), depth

    def in_frame(self, uv) -> bool:
        return (0.0 <= uv[0] < self.width) and (0.0 <= uv[1] < self.height)

    def tip_pixel(self) -> tuple:
        """`TIP_PIXEL` — the constant projection of `TIP_GRIPPER`.

        Diagnostic only: it is rigid to the mount, so this cannot vary with any
        pose.  Do not report its invariance as a result.
        """
        camera = self.rotation_gl @ GL_TO_CV
        cam = camera.T @ (TIP_GRIPPER_M - self.position)
        depth = float(cam[2])
        if abs(depth) < 1e-9:
            return (float("nan"), float("nan"))
        return (self.cx + self.fx * cam[0] / depth,
                self.cy + self.fy * cam[1] / depth)


# ---------------------------------------------------------------------- plane
@dataclass(frozen=True)
class Plane:
    """A plane in the arm-base frame: ``normal . x = offset`` (metres)."""

    normal: np.ndarray
    offset: float
    samples: int = 1
    spread_mm: float = 0.0
    age_s: float = 0.0
    source: str = "markers"
    rms_px: float | None = None
    ids: int | None = None

    @property
    def point(self) -> np.ndarray:
        return self.normal * self.offset

    def intersect(self, origin, direction) -> tuple:
        """Ray/plane intersection.  ``(point, reason)``; point is None on miss."""
        axis = np.asarray(direction, float)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12:
            return None, "tool axis is degenerate"
        axis = axis / norm
        cosine = float(axis @ self.normal)
        if abs(cosine) < MIN_RAY_COS:
            return None, (f"ray grazes the board: tool axis is "
                          f"{math.degrees(math.acos(min(1.0, abs(cosine)))):.1f} deg "
                          f"from the board plane")
        gap = float(np.asarray(origin, float) @ self.normal) - self.offset
        s = -gap / cosine
        if s <= 0.0:
            return None, "aim ray points away from the board"
        return np.asarray(origin, float) + s * axis, ""


class PlaneTracker:
    """Running estimate of the board plane in the arm-base frame.

    Each PnP solve gives a plane; a 2-marker solve is worth 10-20 mm, so the
    last ``window`` observations are averaged in (normal, offset) space.  A
    normal is a direction, so it is sign-aligned to +z before averaging and
    renormalised after; the offset is the mean of ``n . p``.  A jump larger
    than ``jump_mm`` means the board moved (or the calibration changed): the
    window is dropped rather than blended.
    """

    def __init__(self, window: int = 9, jump_mm: float = 80.0,
                 max_rms_px: float = 6.0) -> None:
        self.window = max(1, int(window))
        self.jump_mm = float(jump_mm)
        # A 2-marker solve can go badly wrong (one phase-3 view came back at
        # 65 px RMS and 74 deg of tilt); averaging that in would poison the
        # plane for nine solves, so a poor solve is refused, not blended.
        self.max_rms_px = float(max_rms_px)
        self.rejected = 0
        self._normals: list[np.ndarray] = []
        self._offsets: list[float] = []
        self._meta: list[dict] = []
        self._keys: list[object] = []

    def __len__(self) -> int:
        return len(self._normals)

    def reset(self) -> None:
        self._normals.clear()
        self._offsets.clear()
        self._meta.clear()
        self._keys.clear()
    def add(self, normal, point, *, key: object = None, ts: float | None = None,
            rms_px: float | None = None, ids: int | None = None,
            source: str = "markers") -> bool:
        """Add one observation.  ``key`` deduplicates a repeated snapshot."""
        if key is not None and key in self._keys:
            return False
        if rms_px is not None and self.max_rms_px and float(rms_px) > self.max_rms_px:
            self.rejected += 1
            return False
        normal = np.asarray(normal, float)
        normal = normal / max(1e-12, float(np.linalg.norm(normal)))
        if normal[2] < 0.0:                 # the board faces up out of the table
            normal = -normal
        offset = float(np.asarray(point, float) @ normal)
        # Metres in, millimetres out: this guard was inert when it compared a
        # metre offset against a millimetre threshold (caught by
        # tests/test_aim_geometry.py, 2026-11-24).
        if self._offsets and abs(offset - float(np.median(self._offsets))) * 1000.0 > self.jump_mm:
            self.reset()
        elif self._normals and float(normal @ np.mean(self._normals, axis=0)) < 0.0:
            normal = -normal
        self._normals.append(normal)
        self._offsets.append(offset)
        self._keys.append(key)
        self._meta.append({"ts": time.time() if ts is None else float(ts),
                           "rms_px": rms_px, "ids": ids, "source": source})
        while len(self._normals) > self.window:
            self._normals.pop(0)
            self._offsets.pop(0)
            self._keys.pop(0)
            self._meta.pop(0)
        return True

    def plane(self, now: float | None = None) -> Plane | None:
        """Median combination, so one outlier cannot drag the plane.

        The normal is the component-wise median renormalised; the offset is the
        median of ``n . p``; the spread is the MAD scaled to a standard
        deviation.  A mean would let the occasional 20 mm solve move the plane
        the aim is computed against, which is the whole reason for averaging.
        """
        if not self._normals:
            return None
        mean = np.median(np.asarray(self._normals, float), axis=0)
        normal = mean / max(1e-12, float(np.linalg.norm(mean)))
        offsets = np.asarray(self._offsets, float)
        offset = float(np.median(offsets))
        mad = float(np.median(np.abs(offsets - offset))) * 1.4826
        now = time.time() if now is None else now
        newest_meta = max(self._meta, key=lambda m: m["ts"])
        return Plane(normal=normal,
                     offset=offset,
                     samples=len(offsets),
                     spread_mm=mad * 1000.0,
                     age_s=max(0.0, now - max(m["ts"] for m in self._meta)),
                     source=newest_meta["source"],
                     rms_px=newest_meta["rms_px"],
                     ids=newest_meta["ids"])


def cam_board_from_result(board: dict) -> np.ndarray | None:
    """``T_cam_board`` (OpenCV camera axes) from a detector ``result["board"]``."""
    if not board or "rvec" not in board or "tvec" not in board:
        return None
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_rotvec(
        np.asarray(board["rvec"], float).reshape(3)).as_matrix()
    pose[:3, 3] = np.asarray(board["tvec"], float).reshape(3)
    return pose


def plane_from_solution(board: dict, pose_board_tool: np.ndarray,
                        mount: CameraMount, board_frame: np.ndarray) -> tuple:
    """The board plane in the arm-base frame from one solve.

    ``pose_board_tool`` is ``T_board_tool`` at the instant of the solve — the
    detector's own record, not the current pose, because the arm may have moved
    since.  ``board_frame`` is ``T_base_board`` as configured (used only to lift
    the solve into the base frame; the plane itself comes from the markers).

    Returns ``(normal_base, point_base, info)`` or ``(None, None, info)``.
    """
    cam_board = cam_board_from_result(board)
    if cam_board is None:
        return None, None, {"reason": "solve has no board pose"}
    pose = np.asarray(pose_board_tool, float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        return None, None, {"reason": "no tool pose paired with the solve"}
    # base <- tool <- camera <- board, with the camera axes flipped GL -> CV
    # once, at the only place the two conventions meet.
    base_camera = (np.asarray(board_frame, float) @ pose) @ np.block([
        [mount.rotation_gl @ GL_TO_CV, mount.position.reshape(3, 1)],
        [np.zeros((1, 3)), 1.0]])
    base_board = base_camera @ cam_board
    normal = base_board[:3, 2] / max(1e-12, float(np.linalg.norm(base_board[:3, 2])))
    return normal, base_board[:3, 3], {"rms_px": board.get("rms_px"),
                                       "ids": len(board.get("ids") or [])}


# ----------------------------------------------------------------------- aim
def aim_from_pose(pose_base_tool, plane: Plane) -> tuple:
    """`AIM_WORLD`: jaw origin -> tip centre -> out, to the plane."""
    pose = np.asarray(pose_base_tool, float)
    origin = pose[:3, 3]
    axis = -pose[:3, 2]                     # the tool's +z points back up the jaws
    return plane.intersect(origin, axis)


def tip_offset_px(aim_uv, tip_uv) -> tuple:
    du = float(aim_uv[0] - tip_uv[0])
    dv = float(aim_uv[1] - tip_uv[1])
    return du, dv, float(math.hypot(du, dv))


def square_at(x_mm: float, y_mm: float, square_mm: float = 28.0,
              files: str = "abcdefgh") -> str | None:
    """Board millimetres -> ``a1..h8`` (same mapping as ``board_target``)."""
    half = 4.0 * square_mm
    col = int(math.floor((float(x_mm) + half) / square_mm))
    rank = int(math.floor((float(y_mm) + half) / square_mm))
    if 0 <= col < 8 and 0 <= rank < 8:
        return f"{files[col]}{rank + 1}"
    return None


def evaluate(pose_base_tool, mount: CameraMount, plane: Plane | None, *,
             board_frame: np.ndarray | None = None,
             board_half_mm: float = DEFAULT_BOARD_HALF_MM,
             square_mm: float = 28.0,
             plane_max_age_s: float = DEFAULT_PLANE_MAX_AGE_S) -> dict:
    """Everything a client needs, and a validity flag with its reason.

    Never raises: a missing plane, a ray that misses the board or a stale
    estimate come back as ``ok: False`` with ``reason`` set, and whatever could
    still be computed is included.
    """
    out: dict = {
        "schema": "aim/1",
        "ok": False,
        "reason": "",
        "tip_pixel": [round(v, 2) for v in mount.tip_pixel()],
        "tip_pixel_note": "constant of the mount, diagnostic only",
        "aim_pixel": None,
        "offset_px": None,
        "offset_uv": None,
        "aim_world_mm": None,
        "aim_board_mm": None,
        "ray_length_mm": None,
        "square": None,
        "visible": False,
    }
    if plane is None:
        out["reason"] = "no board plane: no marker solve yet"
        return out
    out["plane"] = {"source": plane.source,
                    "samples": plane.samples,
                    "spread_mm": round(plane.spread_mm, 2),
                    "age_s": round(plane.age_s, 2),
                    "rms_px": None if plane.rms_px is None else round(float(plane.rms_px), 3),
                    "ids": plane.ids}
    if plane.age_s > plane_max_age_s:
        out["reason"] = (f"board plane is stale: last marker solve "
                         f"{plane.age_s:.0f} s ago (limit {plane_max_age_s:.0f} s)")
    world, reason = aim_from_pose(pose_base_tool, plane)
    if world is None:
        out["reason"] = out["reason"] or reason
        return out

    pose = np.asarray(pose_base_tool, float)
    out["aim_world_mm"] = [round(float(v) * 1000.0, 1) for v in world]
    out["ray_length_mm"] = round(float(np.linalg.norm(world - pose[:3, 3])) * 1000.0, 1)
    if board_frame is not None:
        local = (np.linalg.inv(np.asarray(board_frame, float))
                 @ np.array([*world, 1.0]))[:3] * 1000.0
        out["aim_board_mm"] = [round(float(v), 1) for v in local]
        out["square"] = square_at(local[0], local[1], square_mm)
    uv, depth = mount.project(pose, world)
    if np.isfinite(uv[0]) and depth > 0.0:
        out["aim_pixel"] = [round(float(uv[0]), 2), round(float(uv[1]), 2)]
        out["visible"] = mount.in_frame(uv)
        tip = mount.tip_pixel()
        du, dv, total = tip_offset_px(uv, tip)
        out["offset_uv"] = [round(du, 2), round(dv, 2)]
        out["offset_px"] = round(total, 2)

    on_board = (out["aim_board_mm"] is not None
                and abs(out["aim_board_mm"][0]) <= board_half_mm
                and abs(out["aim_board_mm"][1]) <= board_half_mm)
    if not on_board:
        detail = ("aim is off the board"
                  if out["aim_board_mm"] is not None
                  else "aim could not be expressed in board millimetres")
        out["reason"] = out["reason"] or detail
        return out
    out["ok"] = not out["reason"]
    return out


def aim_translation(pose_base_tool, plane: Plane, target_board_xy,
                    board_frame: np.ndarray) -> np.ndarray:
    """Base-frame translation that moves the aim onto ``target_board_xy``.

    A rigid translation of the tool moves the plane intersection by exactly the
    in-plane part of that translation, so this is one step of an exact Newton
    iteration, not an approximation — the residual only comes from IK error.
    """
    world, reason = aim_from_pose(pose_base_tool, plane)
    if world is None:
        raise ValueError(reason)
    local = (np.linalg.inv(np.asarray(board_frame, float))
             @ np.array([*world, 1.0]))[:3] * 1000.0
    delta_board = np.array([float(target_board_xy[0]) - local[0],
                            float(target_board_xy[1]) - local[1], 0.0])
    rotation = np.asarray(board_frame, float)[:3, :3]
    return rotation @ (delta_board / 1000.0)
