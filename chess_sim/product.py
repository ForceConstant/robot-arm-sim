"""Frozen product digital twin for the single-follower chess arm.

`hardware/product_twin.json` is the committed source of truth for the measured
product scene.  This module validates it, turns it into the existing
`EnvConfig`, and computes the measured/assumed piece mass properties used by the
Phase 8 qualification.

The distinction between *measured* and *assumed* is deliberate:

* the board pitch, slab thickness, border and label presence are Brian's
  2026-11-02 physical measurements;
* the rook's assembled 4.8 g (including one M6 nut) is a physical measurement,
  while its exact nut height, centre of mass and inertia are a bounded model;
* every other piece's assembled mass is unmeasured and stays a range, never a
  fabricated single number.

Nothing here starts hardware, imports a servo driver or changes the published
simulator defaults unless a caller explicitly asks for the product config.
"""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

import chess
import numpy as np
from pydantic import Field

from . import fiducials, gripper
from .config import (JOINTS, AppearanceConfig, BoardConfig, Camera, Config,
                     ControlConfig, EnvConfig, ToleranceConfig)


PRODUCT_TWIN_PATH = Path(__file__).resolve().parent.parent / "hardware" / "product_twin.json"
SCHEMA = "chess-robot-product-twin/1"


class MassProperties(Config):
    """A rigid-body mass model in the units the source pack uses."""

    mass_g: float
    com_mm: tuple[float, float, float]
    inertia_kg_m2: tuple[tuple[float, float, float], ...]


class PieceTwin(Config):
    """One piece type: an unweighted printed shell plus a measured or bounded
    bottom-weight model."""

    count_per_side: int
    measured: bool
    unweighted_pla: MassProperties
    assembled: MassProperties | None = None      # only when measured
    assembled_includes: str | None = None
    nut_mass_g: float | None = None
    nut_model_z_mm: float | None = None
    nut_model_z_range_mm: tuple[float, float] | None = None
    mass_range_g: tuple[float, float] | None = None   # unmeasured bound
    added_mass_com_z_mm: float | None = None           # assumed point-mass height
    provenance: str


class RobotTwin(Config):
    model: str
    arms: int
    arm_side: str
    joint_order: tuple[str, ...]
    dof: int
    control_hz: int
    image_size: tuple[int, int]


class BoardTwin(Config):
    square_mm: float
    border_mm: float
    thickness_mm: float
    labels: bool
    playing_field_mm: float
    overall_mm: float
    measurement: str
    border_assumption: str


class MountTwin(Config):
    table_top_mm: float
    arm_riser_mm: float
    board_edge_to_arm_base_mm: float
    arm_base_from_board_centre_mm: tuple[float, float]
    footprint_mm: tuple[float, float]


class ToolTwin(Config):
    kind: str
    name: str
    source_microversion: str
    aperture_mm: tuple[float, float]
    tip_reach_mm: float
    finger_halfwidth_mm: float
    squeeze_mm: float
    servo_mass_g: float
    camera_mass_g: float


class RejectedTool(Config):
    kind: str
    name: str
    result: str
    evidence: str


class ToolSelection(Config):
    selected: str
    rejected: tuple[RejectedTool, ...]


class CameraTwin(Config):
    operational: tuple[str, ...]
    debug: tuple[str, ...]
    wrist_pos_mm: tuple[float, float, float]
    wrist_target_mm: tuple[float, float, float]
    wrist_up: tuple[float, float, float]
    wrist_fovy_deg: float
    image_size: tuple[int, int]
    provenance: str


class TrayTwin(Config):
    slots: int
    first_slot_from_board_centre_mm: tuple[float, float]
    slot_pitch_mm: float
    surface: str


class PerturbationTwin(Config):
    board_shift_mm: float
    arm_joint_jitter_rad: float
    supported: tuple[str, ...]
    excluded_until_phase_9: tuple[str, ...]


class EvaluatorTwin(Config):
    placement_mm: float
    tray_mm: float
    disturbance_mm: float
    upright_min: float
    rule: str


class ProductTwin(Config):
    """The whole frozen product scene and its acceptance contract."""

    schema_version: str = Field(alias="schema")
    frozen_at: str
    sources: dict[str, str]
    robot: RobotTwin
    board: BoardTwin
    mount: MountTwin
    tool: ToolTwin
    tool_selection: ToolSelection
    camera: CameraTwin
    tray: TrayTwin
    pieces: dict[str, PieceTwin]
    perturbations: PerturbationTwin
    evaluator: EvaluatorTwin
    interfaces: dict[str, str]

    # -- conversion to the simulator's existing configuration -----------------

    def board_config(self) -> BoardConfig:
        b, m = self.board, self.mount
        return BoardConfig(
            square=b.square_mm / 1000.0,
            border=b.border_mm / 1000.0,
            thickness=b.thickness_mm / 1000.0,
            table_top=m.table_top_mm / 1000.0,
            mat=fiducials.MAT_THICKNESS_M if self.tool is not None else 0.0,
            arm_gap=m.board_edge_to_arm_base_mm / 1000.0,
            arm_riser=m.arm_riser_mm / 1000.0,
            arm_side=self.robot.arm_side,
            tray_x=self.tray.first_slot_from_board_centre_mm[0] / 1000.0,
            tray_y0=self.tray.first_slot_from_board_centre_mm[1] / 1000.0,
            tray_pitch=self.tray.slot_pitch_mm / 1000.0,
            tray_slots=self.tray.slots,
        )

    def appearance(self, piece_set: str = "printed") -> AppearanceConfig:
        return AppearanceConfig(piece_set=piece_set, board_labels=self.board.labels,
                                board_finish="plastic", camera_mast=False,
                                board_markers=True)

    def env_config(self, piece_set: str = "printed",
                   cameras: tuple[Camera, ...] = (Camera.WRIST,),
                   dual_arms: bool = False) -> EnvConfig:
        return EnvConfig(
            board=self.board_config(),
            appearance=self.appearance(piece_set),
            control=ControlConfig(hz=self.robot.control_hz, cameras=cameras,
                                  image_size=self.robot.image_size),
            tolerances=ToleranceConfig(
                placement=self.evaluator.placement_mm / 1000.0,
                tray=self.evaluator.tray_mm / 1000.0,
                disturbance=self.evaluator.disturbance_mm / 1000.0,
                upright_min=self.evaluator.upright_min),
            dual_arms=dual_arms,
        )

    def validate_against_code(self) -> None:
        """Fail loudly when the frozen document and the live simulator drift."""
        if self.schema_version != SCHEMA:
            raise ValueError(f"unexpected product-twin schema {self.schema_version!r}")
        if tuple(self.robot.joint_order) != tuple(str(j) for j in JOINTS):
            raise ValueError("product-twin joint order no longer matches chess_sim.JOINTS")
        if self.robot.dof != len(JOINTS):
            raise ValueError("product-twin DOF does not match the simulator")
        if self.robot.arms != 1:
            raise ValueError("the product twin must contain exactly one follower")
        b = self.board
        if abs(b.playing_field_mm - 8 * b.square_mm) > 1e-6:
            raise ValueError("board playing field does not match 8 * square pitch")
        if abs(b.overall_mm - (b.playing_field_mm + 2 * b.border_mm)) > 1e-6:
            raise ValueError("board overall size does not match field + two borders")
        expected_base_y = -(b.overall_mm / 2 + self.mount.board_edge_to_arm_base_mm)
        if abs(self.mount.arm_base_from_board_centre_mm[1] - expected_base_y) > 1e-6:
            raise ValueError("arm base is not at board edge + gap")
        if set(self.pieces) != {"pawn", "rook", "knight", "bishop", "queen", "king"}:
            raise ValueError("product twin must define all six piece types")
        if sum(p.count_per_side for p in self.pieces.values()) != 16:
            raise ValueError("piece counts must total 16 per side")
        rook = self.pieces["rook"]
        if not rook.measured or rook.assembled is None:
            raise ValueError("the measured rook mass is missing")
        if abs(rook.assembled.mass_g - 4.8) > 1e-9:
            raise ValueError("the measured rook mass must be 4.8 g")
        if tuple(self.camera.operational) != ("wrist",):
            raise ValueError("operational vision must be wrist-only")
        if self.tool_selection.selected != self.tool.kind:
            raise ValueError("selected tool does not match the frozen tool block")
        ev = self.evaluator
        tol = ToleranceConfig()
        for name, got, want in (
                ("placement", ev.placement_mm, tol.placement * 1000),
                ("tray", ev.tray_mm, tol.tray * 1000),
                ("disturbance", ev.disturbance_mm, tol.disturbance * 1000),
                ("upright_min", ev.upright_min, tol.upright_min)):
            if abs(got - want) > 1e-9:
                raise ValueError(f"evaluator {name} drifted from ToleranceConfig")


@lru_cache(maxsize=1)
def load_product_twin(path: str | Path = PRODUCT_TWIN_PATH) -> ProductTwin:
    raw = json.loads(Path(path).read_text())
    twin = ProductTwin.model_validate(raw)
    twin.validate_against_code()
    return twin


LAYOUT_PATH = PRODUCT_TWIN_PATH.parent.parent / "hardware" / "current_layout.json"


@lru_cache(maxsize=1)
def current_layout() -> dict:
    """The measured mount the live service runs (not the historical twin)."""
    payload = json.loads(LAYOUT_PATH.read_text())
    if payload.get("schema") != "robot-arm-current-layout/1":
        raise ValueError("unexpected current-layout schema")
    return payload


def layout_env_config(piece_set: str = "printed",
                      cameras: tuple[Camera, ...] = (Camera.WRIST,)) -> EnvConfig:
    """Product scene at the current mount, including the fiducial mat."""
    twin = load_product_twin()
    layout = current_layout()
    mount, measured = layout["mount"], layout["board"]
    config = twin.env_config(piece_set=piece_set, cameras=cameras)
    board = config.board.model_copy(update={
        "square": measured["square_mm"] / 1000.0,
        "border": measured["border_mm"] / 1000.0,
        "thickness": measured["thickness_mm"] / 1000.0,
        # Fields absent from a frozen layout keep the qualified defaults:
        # a 0.5 mm mat under the slab with the markers printed on it.
        "mat": measured.get("mat_mm", 0.5) / 1000.0,
        "marker_plane": measured.get("marker_plane", "mat"),
        "arm_gap": mount["board_edge_to_base_frame_mm"] / 1000.0,
        "arm_riser": mount["base_mount_datum_above_table_mm"] / 1000.0,
        "tray_x": layout.get("tray_x_mm", -160.0) / 1000.0,
    })
    if not np.allclose(np.array(board.arm_base) * 1000,
                       mount["white_arm_base_frame_mm"], atol=1e-6):
        raise ValueError("current layout does not match its own arm-base record")
    return config.model_copy(update={"board": board})


def mass_properties(base: MassProperties, added_mass_g: float,
                    added_com_z_mm: float) -> MassProperties:
    """Combine an unweighted mesh with a bottom point mass.

    The base inertia is the source pack's tensor about the mesh's centre of
    mass.  The added mass is treated as a point on the piece's vertical axis;
    that is an explicit nuts-and-bolts approximation, not a claimed scan.
    """
    m0 = base.mass_g / 1000.0
    m1 = added_mass_g / 1000.0
    if m0 <= 0 or m1 < 0:
        raise ValueError("masses must be positive")
    total = m0 + m1
    c0 = np.asarray(base.com_mm, dtype=float) / 1000.0
    c1 = np.array([0.0, 0.0, added_com_z_mm / 1000.0])
    c = (m0 * c0 + m1 * c1) / total
    i0 = np.asarray(base.inertia_kg_m2, dtype=float)
    eye = np.eye(3)

    def shift(m: float, point: np.ndarray) -> np.ndarray:
        d = point - c
        return m * (float(d @ d) * eye - np.outer(d, d))

    inertia = i0 + shift(m0, c0) + shift(m1, c1)
    return MassProperties(
        mass_g=total * 1000.0,
        com_mm=tuple(float(v) * 1000.0 for v in c),
        inertia_kg_m2=tuple(tuple(float(v) for v in row) for row in inertia),
    )


def piece_mass_properties(twin: ProductTwin, piece_type: str,
                          regime: str = "nominal") -> MassProperties:
    """Mass model for one piece at nominal or heavy-bound weight.

    `nominal` uses the measured rook and the unweighted nominal for pieces whose
    assembled weight is unknown.  `heavy_bound` adds the top of each unmeasured
    range at its assumed bottom point; it is a sensitivity case, not a fact.
    """
    if regime not in ("nominal", "heavy_bound"):
        raise ValueError("regime must be nominal or heavy_bound")
    p = twin.pieces[piece_type]
    if p.measured and p.assembled is not None:
        return p.assembled
    if regime == "nominal" or p.mass_range_g is None:
        return p.unweighted_pla
    added = p.mass_range_g[1] - p.unweighted_pla.mass_g
    z = p.added_mass_com_z_mm if p.added_mass_com_z_mm is not None else 3.0
    return mass_properties(p.unweighted_pla, added, z)


def rook_mass_range(twin: ProductTwin) -> list[MassProperties]:
    """The measured 4.8 g total with the nut-height model at both bounds."""
    p = twin.pieces["rook"]
    if p.nut_model_z_range_mm is None or p.nut_mass_g is None:
        raise ValueError("rook nut model is incomplete")
    return [mass_properties(p.unweighted_pla, p.nut_mass_g, z)
            for z in p.nut_model_z_range_mm]


def apply_mass_model(env, twin: ProductTwin, regime: str = "nominal", *, all_slots: bool = False) -> list[dict]:
    """Apply the product mass model to every piece already in an environment.

    Call after `env.reset()` (so slots exist), then call `env.reset()` again if
    the changed masses should settle the scene.  Returns the applied records for
    the qualification report; the caller owns any further physics checks.
    `all_slots=True` also covers parked pieces before the first reset, for a
    service that must keep the same mass model in every subsequent case.
    """
    import mujoco

    applied = []
    slots = (env.slots if all_slots else
             list(env._square_slot.values()) + list(getattr(env, "_captured", [])))
    for slot in slots:
        ptype = chess.piece_name(slot.piece.piece_type)
        props = piece_mass_properties(twin, ptype, regime)
        body = env.model.body(slot.body)
        body.mass[0] = props.mass_g / 1000.0
        body.ipos[:] = np.asarray(props.com_mm) / 1000.0
        inertia = np.asarray(props.inertia_kg_m2)
        body.inertia[:] = np.diag(inertia)
        applied.append({"body": slot.body, "piece": ptype, "regime": regime,
                        "mass_g": props.mass_g,
                        "com_mm": list(props.com_mm),
                        "inertia_diag_kg_m2": [float(v) for v in np.diag(inertia)]})
    mujoco.mj_setConst(env.model, env.data)
    return applied


def current_tool_kind() -> str:
    return "cad_parallel" if gripper.CUSTOM else "stock"


def gripper_ready(twin: ProductTwin | None = None) -> bool:
    """Whether the process was started with the frozen product tool."""
    twin = load_product_twin() if twin is None else twin
    return current_tool_kind() == twin.tool.kind


def require_tool(twin: ProductTwin | None = None) -> None:
    twin = load_product_twin() if twin is None else twin
    actual = current_tool_kind()
    if actual != twin.tool.kind:
        raise RuntimeError(
            f"the product twin selects {twin.tool.kind!r}; this process is running "
            f"{actual!r}. Set ROBOT_GRIPPER accordingly (stock is the default).")


def contract_digest(twin: ProductTwin) -> dict:
    """Small, stable summary used in reports and tests."""
    return {
        "schema": twin.schema_version,
        "frozen_at": twin.frozen_at,
        "board": {"square_mm": twin.board.square_mm, "border_mm": twin.board.border_mm,
                  "thickness_mm": twin.board.thickness_mm, "labels": twin.board.labels,
                  "overall_mm": twin.board.overall_mm},
        "robot": {"arms": twin.robot.arms, "side": twin.robot.arm_side,
                  "joints": list(twin.robot.joint_order), "hz": twin.robot.control_hz},
        "tool": {"kind": twin.tool.kind, "name": twin.tool.name,
                 "aperture_mm": list(twin.tool.aperture_mm)},
        "operational_cameras": list(twin.camera.operational),
        "piece_masses_g": {k: (p.assembled.mass_g if p.assembled else p.mass_range_g)
                           for k, p in twin.pieces.items()},
        "evaluator": twin.evaluator.model_dump(),
    }
