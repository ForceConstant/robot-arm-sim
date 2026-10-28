"""Pack contract and real free-body contacts, without publishing private meshes."""
import json
import os

import chess
import mujoco
import numpy as np
import pytest
import trimesh
from PIL import Image

from chess_sim import AppearanceConfig, BoardConfig, ControlConfig, EnvConfig
from chess_sim import assets
from chess_sim.dual import DualArmEnv
from chess_sim.scene import build_scene


@pytest.fixture(scope="module")
def printed_pack(tmp_path_factory):
    previous = os.environ.get("ROBOT_PIECE_ASSETS")
    if previous:
        root = previous
    else:
        root = str(tmp_path_factory.mktemp("synthetic-printed-pack"))
        from pathlib import Path
        manifest = {"name": "Synthetic contract fixture", "scale": "absolute", "pieces": {}}
        for name in assets._TYPE_NAME.values():
            folder = Path(root) / name
            folder.mkdir()
            mesh = trimesh.creation.cylinder(radius=.008, height=.03, sections=24)
            mesh.apply_translation([0, 0, .015])
            mesh.export(folder / f"{name}.obj")
            Image.new("RGB", (8, 8), "white").save(folder / "material_0.png")
            mesh.density = 600
            manifest["pieces"][name] = dict(collision_meshes=[f"{name}/{name}.obj"],
                profile=[[i*.0075, (i+1)*.0075, .008] for i in range(4)],
                mass_kg=mesh.mass, center_mass_m=mesh.center_mass.tolist(),
                inertia_kg_m2=mesh.moment_inertia.tolist())
        (Path(root) / "set.json").write_text(json.dumps(manifest))
    os.environ["ROBOT_PIECE_ASSETS"] = root
    assets.set_properties.cache_clear()
    yield root
    if previous is None:
        os.environ.pop("ROBOT_PIECE_ASSETS", None)
    else:
        os.environ["ROBOT_PIECE_ASSETS"] = previous
    assets.set_properties.cache_clear()


def test_absolute_dimensions_and_identical_colour_geometry(printed_pack):
    for kind in chess.PIECE_TYPES:
        a, b = [assets.piece_geometry(chess.Piece(kind, side), .028, piece_set="printed")
                for side in (chess.WHITE, chess.BLACK)]
        source = trimesh.load(assets.piece_obj_path(chess.Piece(kind, True), "printed"), force="mesh")
        assert a.height == pytest.approx(source.extents[2], abs=1e-8)
        assert a.width == b.width and a.height == b.height and a.profile == b.profile
        assert a.waist >= .25 * a.height
        assert a.collider_radius >= a.profile[0][2]
        # Absolute sets do not grow with the board.
        larger = assets.piece_geometry(chess.Piece(kind, True), .035, piece_set="printed")
        assert larger.height == a.height
    with pytest.raises(ValueError, match="uniform scale"):
        assets.piece_geometry(chess.Piece(chess.KING, True), .010, piece_set="printed")


def test_compound_contacts_mass_and_stable_settling(printed_pack):
    spec = build_scene(BoardConfig(), AppearanceConfig(piece_set="printed", camera_mast=False), dual_arms=True)
    model = spec.compile()
    data = mujoco.MjData(model)
    # Avoid creating an IK solver just to check standalone free-body contacts.
    records = assets.set_properties("printed")["pieces"]
    for color in (chess.WHITE, chess.BLACK):
        for kind in chess.PIECE_TYPES:
            piece = chess.Piece(kind, color)
            prefix = assets.piece_asset_name(piece)
            from chess_sim.scene import piece_slots
            slot = next(s for s in piece_slots(BoardConfig(), piece_set="printed") if s.piece == piece)
            body = model.body(slot.body)
            contacts = [g for g in range(body.geomadr[0], body.geomadr[0]+body.geomnum[0])
                        if model.geom_contype[g]]
            assert contacts
            assert all(model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH for g in contacts)
            assert body.mass[0] == pytest.approx(records[assets._TYPE_NAME[kind]]["mass_kg"])
    assert model.neq == (4 if __import__('chess_sim.gripper', fromlist=['CUSTOM']).CUSTOM else 0)
    # Test actual reset and board settling of all 32 pieces in the two-arm world.
    env = DualArmEnv(EnvConfig(appearance=AppearanceConfig(piece_set="printed", camera_mast=False),
                              control=ControlConfig(cameras=())))
    try:
        env.reset()
        assert len(env._square_slot) == 32
        assert env.full_board_error(chess.Board()) is None
        for slot in env._square_slot.values():
            pos = env.piece_position(slot)
            assert abs(pos[2] - env.board.top) < .0015
        assert np.isfinite(env.data.qpos).all()
        assert env.model.nu == 12
    finally:
        env.close()


def test_missing_requested_pack_is_not_silent_fallback(monkeypatch):
    monkeypatch.delenv("ROBOT_PIECE_ASSETS", raising=False)
    with pytest.raises(ValueError, match="ROBOT_PIECE_ASSETS"):
        assets.piece_obj_path(chess.Piece(chess.KING, True), "printed")
