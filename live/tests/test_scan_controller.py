"""ScanController against a simulator-backed fake bridge.

The controller never touches MuJoCo; these fakes give it the two things it
expects from the real system — a bridge that moves to joint targets and reports
where it is, and a camera that returns a frame with the joints that produced it.
That is enough to test the orchestration contract without a robot.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from chess_sim import fiducials  # noqa: E402
from chess_sim.config import Camera  # noqa: E402
from chess_sim.env import ChessSimEnv  # noqa: E402
from chess_sim.marker_scan import (MarkerLayout, MarkerScanPlan,  # noqa: E402
                                   PoseCoverage, board_offset, load_plan,
                                   marker_corners_world)
from chess_sim.product import current_layout, layout_env_config  # noqa: E402
from chess_sim.wrist_calibration import WristCalibration  # noqa: E402
from scan_controller import Frame, ScanController, plan_in_degrees  # noqa: E402

CALIBRATION = Path(__file__).resolve().parents[2] / ".private/phase11d/calibration.json"
PLAN = Path(__file__).resolve().parents[2] / "chess_sim/assets/marker_scan_plan.json"
NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeBridge:
    def __init__(self, env, clock, *, rate_deg_s: float = 240.0,
                 mode: str = "REMOTE", armed: bool = True) -> None:
        self.env = env
        self.clock = clock
        self.rate = rate_deg_s
        self.mode = mode
        self.armed = armed
        self.calls = 0
        self.targets: list[tuple[float, ...]] = []

    def joints(self) -> tuple[float, ...]:
        # The bridge speaks degrees; the scene stores radians.
        return tuple(float(np.degrees(v))
                     for v in self.env.data.qpos[self.env._joint_qpos])

    def state(self) -> dict:
        return {"mode": self.mode, "armed": self.armed,
                "joints": dict(zip(NAMES, self.joints()))}

    def command(self, joints, *, seq: int, ttl_ms: int, speed_scale: float) -> dict:
        self.calls += 1
        self.targets.append(tuple(float(v) for v in joints))
        target = np.radians(np.asarray(joints, dtype=float))
        current = self.env.data.qpos[self.env._joint_qpos].copy()
        step = math.radians(self.rate) * 0.1
        self.env.data.qpos[self.env._joint_qpos] = current + np.clip(
            target - current, -step, step)
        mujoco.mj_forward(self.env.model, self.env.data)
        return {"accepted": True, "seq": seq}


class FakeFrames:
    """Renders the wrist view and pairs it with the joints that produced it."""

    def __init__(self, env, clock, calibration: WristCalibration, *,
                 stale_first: bool = False, hide: dict | None = None) -> None:
        self.env = env
        self.clock = clock
        self.calibration = calibration
        self.stale_first = stale_first
        self.hide = hide or {}
        self.captures = 0

    def joints(self) -> tuple[float, ...]:
        return tuple(float(np.degrees(v))
                     for v in self.env.data.qpos[self.env._joint_qpos])

    def capture(self) -> Frame:
        self.captures += 1
        joints = self.joints()
        image = self.env.render(Camera.WRIST).copy()
        hide_markers = self.hide.get(self.captures, ())
        if hide_markers:
            calibration = self.calibration.camera(
                tuple(math.radians(v) for v in joints))
            draw = ImageDraw.Draw(Image.fromarray(image))
            frame_image = Image.fromarray(image)
            draw = ImageDraw.Draw(frame_image)
            for marker in hide_markers:
                world = marker_corners_world(
                    MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0),
                    marker, origin_xy=self.env.board.origin,
                    yaw=self.env.board.yaw,
                    marker_z=self.env.board.table_top
                    + fiducials.marker_height_above_table(self.env.board.mat))
                pixels = [calibration.project(point) for point in world]
                if any(pixel is None for pixel in pixels):
                    continue
                draw.polygon([tuple(p) for p in pixels], fill=(120, 140, 160))
            image = np.asarray(frame_image)
        if self.stale_first and self.captures == 1:
            joints = tuple(v + 3.0 for v in joints)
        return Frame(image=image, joints_deg=joints, timestamp=self.clock())


def build_env(origin=(0.0, 0.0), yaw_deg=0.0):
    config = layout_env_config("printed", cameras=())
    board = config.board.model_copy(update={"origin": (float(origin[0]), float(origin[1])),
                                            "yaw": math.radians(yaw_deg)})
    env = ChessSimEnv(config.model_copy(update={"board": board}))
    env.reset("8/8/8/8/8/8/8/8")
    return env


@pytest.fixture
def calibration():
    return WristCalibration.load(CALIBRATION)


def controller(env, clock, calibration, plan=None, bridge=None, frames=None, **kwargs):
    plan = plan_in_degrees(plan or load_plan(PLAN))
    bridge = bridge or FakeBridge(env, clock)
    frames = frames or FakeFrames(env, clock, calibration)
    layout = MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0)
    marker_z = env.board.table_top + fiducials.marker_height_above_table(env.board.mat)
    mount = current_layout()["mount"]
    base_xy = (mount["white_arm_base_frame_mm"][0] / 1000.0,
               mount["white_arm_base_frame_mm"][1] / 1000.0)
    return ScanController(
        bridge, frames,
        lambda joints_deg: calibration.camera(
            tuple(math.radians(v) for v in joints_deg)),
        plan, layout=layout,
        square_m=env.board.square, board_z_m=env.board.top, marker_z_m=marker_z,
        base_xy_m=base_xy, board_half_mm=env.board.width * 500.0,
        clock=clock, sleep=clock.sleep, **kwargs), bridge, frames


def truth_offset(env):
    mount = current_layout()["mount"]
    base_xy = (mount["white_arm_base_frame_mm"][0] / 1000.0,
               mount["white_arm_base_frame_mm"][1] / 1000.0)
    return board_offset(env.board.origin, env.board.yaw, base_xy=base_xy,
                        board_half_mm=env.board.width * 500.0)


def test_scan_returns_board_offset_at_nominal():
    env = build_env()
    try:
        clock = FakeClock()
        ctrl, bridge, frames = controller(env, clock, WristCalibration.load(CALIBRATION))
        report = ctrl.run()
        assert report.ok, report.note
        assert report.views_used >= 6
        truth = truth_offset(env)
        assert abs(report.offset.x_mm - truth.x_mm) <= 2.0
        assert abs(report.offset.y_mm - truth.y_mm) <= 2.0
        assert abs(report.offset.yaw_deg - truth.yaw_deg) <= 1.0
        assert bridge.calls > 0 and report.duration_s > 0
    finally:
        env.close()


def test_scan_handles_a_displaced_board():
    env = build_env(origin=(0.03, -0.02), yaw_deg=3.0)
    try:
        clock = FakeClock()
        ctrl, _, _ = controller(env, clock, WristCalibration.load(CALIBRATION))
        report = ctrl.run()
        assert report.ok, report.note
        truth = truth_offset(env)
        assert abs(report.offset.x_mm - truth.x_mm) <= 2.0
        assert abs(report.offset.y_mm - truth.y_mm) <= 2.0
    finally:
        env.close()


def test_stale_frame_is_rejected_and_recaptured():
    env = build_env()
    try:
        clock = FakeClock()
        calibration = WristCalibration.load(CALIBRATION)
        frames = FakeFrames(env, clock, calibration, stale_first=True)
        ctrl, _, _ = controller(env, clock, calibration, frames=frames)
        report = ctrl.run()
        assert report.ok, report.note
        assert any("stale frame" in visit.note for visit in report.visits)
        assert frames.captures > len(report.visits)
    finally:
        env.close()


def test_recovery_runs_when_too_few_markers_are_seen():
    env = build_env()
    try:
        clock = FakeClock()
        calibration = WristCalibration.load(CALIBRATION)
        full = load_plan(PLAN)
        short = MarkerScanPlan(poses=full.poses[:2], min_views=2,
                               layout_offset_mm=full.layout_offset_mm,
                               note="test subset", recovery=full.recovery)
        frames = FakeFrames(env, clock, calibration,
                            hide={1: (2, 3), 2: (2, 3)})
        ctrl, _, _ = controller(env, clock, calibration, plan=short, frames=frames)
        report = ctrl.run()
        assert report.recovery_used, report.note
        assert report.ok, report.note
        assert any(v.name.startswith("recovery-") for v in report.visits)
    finally:
        env.close()


def test_degrees_plan_is_rejected_before_it_can_move():
    from chess_sim.marker_scan import PoseCoverage
    plan = MarkerScanPlan(poses=(PoseCoverage("bad", (1000.0,) * 6, (), ()),))
    env = build_env()
    try:
        clock = FakeClock()
        calibration = WristCalibration.load(CALIBRATION)
        with pytest.raises(ValueError, match="implausible joint values"):
            controller(env, clock, calibration, plan=plan)
    finally:
        env.close()


def test_refuses_when_bridge_is_not_in_remote_mode():
    env = build_env()
    try:
        clock = FakeClock()
        calibration = WristCalibration.load(CALIBRATION)
        bridge = FakeBridge(env, clock, mode="TELEOP")
        ctrl, bridge, _ = controller(env, clock, calibration, bridge=bridge)
        report = ctrl.run()
        assert not report.ok
        assert "TELEOP" in report.note
        assert bridge.calls == 0
    finally:
        env.close()


def test_refuses_when_bridge_is_not_armed():
    env = build_env()
    try:
        clock = FakeClock()
        calibration = WristCalibration.load(CALIBRATION)
        bridge = FakeBridge(env, clock, armed=False)
        ctrl, bridge, _ = controller(env, clock, calibration, bridge=bridge)
        report = ctrl.run()
        assert not report.ok
        assert "armed" in report.note
        assert bridge.calls == 0
    finally:
        env.close()


def test_pose_timeout_is_reported_not_hidden():
    env = build_env()
    try:
        clock = FakeClock()
        calibration = WristCalibration.load(CALIBRATION)
        bridge = FakeBridge(env, clock, rate_deg_s=0.5)   # never arrives
        ctrl, _, _ = controller(env, clock, calibration, bridge=bridge,
                                pose_timeout_s=2.0, settle_s=0.2)
        report = ctrl.run()
        assert not report.ok
        assert report.visits and not report.visits[0].reached
        assert report.visits[0].note == "pose timeout"
    finally:
        env.close()
