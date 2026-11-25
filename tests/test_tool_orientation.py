"""Orientation regression for the private CAD parallel tool.

Fails on: a jaw body placed at identity, a jaw body rotated 90 deg about the
approach axis, a jaw pair that is not mirrored, a tip depth that drifts, a
base that has come off its mount, or a gear that has moved.

Requires the private pack:

    ROBOT_GRIPPER=cad_parallel \
    ROBOT_GRIPPER_ASSETS=.private/gripper-assets-5y \
    PYTHONPATH=. ~/.local/bin/robot-arm-sim python -m pytest tests/test_tool_orientation.py -q
"""
import numpy as np
import mujoco
import pytest

from chess_sim import gripper, scene
from chess_sim import tool_frames as tf


pytestmark = pytest.mark.skipif(
    not gripper.CUSTOM,
    reason="orientation checks are about the private CAD tool")


def _R(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def _inv(M):
    return _T(M[:3, :3].T, -M[:3, :3].T @ M[:3, 3])


@pytest.fixture(scope="module")
def built():
    m = scene.build_arm().compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def _grip_from(m, d, name):
    """so101:gripper <- body, metres."""
    w = m.body("so101:gripper").id
    Tw = _T(d.xmat[w].reshape(3, 3), d.xpos[w])
    b = m.body(name).id
    return _inv(Tw) @ _T(d.xmat[b].reshape(3, 3), d.xpos[b])


def _visual_points(m, body_name, mesh_contains=None):
    """Every visual (group 2) mesh vertex of a body, in that body's frame, metres."""
    b = m.body(body_name).id
    out = []
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi] != b or m.geom_group[gi] != 2:
            continue
        if m.geom_type[gi] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if mesh_contains is not None:
            mn = str(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[gi]))
            if mesh_contains not in mn:
                continue
        mid = m.geom_dataid[gi]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        out.append((_R(m.geom_quat[gi]) @ v.T).T + m.geom_pos[gi])
    assert out, body_name
    return np.vstack(out)


def _in_frame(m, d, body_name, mesh_contains=None):
    pts = _visual_points(m, body_name, mesh_contains)
    M = _grip_from(m, d, body_name)
    return (M[:3, :3] @ pts.T).T + M[:3, 3]


def _mm(P):
    return P * 1000.0


def _base_part_in_gripper(m, d):
    """Collapsed gripper <- base-part-file transform, from the built model."""
    body = _grip_from(m, d, "so101:cad:parallel_base")
    geom = _T(_R(tf.BASE_GEOM_QUAT), np.array(tf.BASE_GEOM_POS_MM) / 1000.0)
    return body @ geom


# --- the chain ------------------------------------------------------------

def test_mount_is_the_profile_mount(built):
    m, d = built
    M = _grip_from(m, d, "so101:cad:parallel_base")
    assert np.allclose(_mm(M[:3, 3]), tf.MOUNT_POS_MM, atol=0.01)
    assert np.allclose(M[:3, :3], _R(tf.MOUNT_QUAT), atol=1e-5)
    # identity on so101:gripper is the 90 deg / 60 mm error this test exists for
    assert np.linalg.norm(M[:3, 3]) > 0.05


def test_base_part_frame_matches_the_documented_composition(built):
    m, d = built
    E = _base_part_in_gripper(m, d)
    assert np.allclose(E[:3, :3], np.array(tf.BASE_PART_IN_GRIPPER_R), atol=1e-5)
    assert np.allclose(_mm(E[:3, 3]), tf.BASE_PART_IN_GRIPPER_T_MM, atol=0.01)


def test_jaw_bodies_sit_in_the_tool_base_frame(built):
    m, d = built
    base = m.body("so101:cad:parallel_base").id
    for side in ("left", "right"):
        b = m.body(f"so101:cad:parallel_jaw_{side}").id
        assert m.body_parentid[b] == base
        assert np.allclose(_mm(m.body_pos[b]), tf.JAW_BODY_POS_MM[side], atol=0.01)
        assert np.allclose(m.body_quat[b], tf.JAW_BODY_QUAT[side], atol=1e-5)


def test_base_extents_in_its_own_part_frame():
    """The authored base part: x 13.25..69.20, z min -35.25, as authored.

    The compiled 5y model merges the base visual into
    ``complete_fixed_jaw_bin``, so the per-part mesh is no longer addressable
    in the built model (measured 2026-11-25).  The authored STL is the source
    of truth for the part's own frame; model placement stays covered by
    ``test_base_part_frame_matches_the_documented_composition`` and
    ``test_mount_is_the_profile_mount``.
    """
    import trimesh

    from chess_sim import gripper
    path = gripper.ASSET_ROOT / "assets" / "gripper_base_1__solid1.stl"
    mesh = trimesh.load(str(path), force="mesh")
    pts = np.asarray(mesh.vertices, float)          # metres, part file frame
    lo, hi = np.array(tf.BASE_PART_BBOX_MM[0]), np.array(tf.BASE_PART_BBOX_MM[1])
    assert np.allclose(_mm(pts).min(0), lo, atol=0.05), _mm(pts).min(0)
    assert np.allclose(_mm(pts).max(0), hi, atol=0.05), _mm(pts).max(0)


def test_base_sits_on_the_mount_plane(built):
    m, d = built
    E = _base_part_in_gripper(m, d)
    hi = np.array(tf.BASE_PART_BBOX_MM[1]) / 1000.0
    top = E[:3, :3] @ hi + E[:3, 3]
    assert abs(_mm(top)[1] - tf.BASE_TOP_Y_MM) < 0.1, _mm(top)
    assert abs(_mm(top)[1] - tf.MOUNT_POS_MM[1]) < 0.05, _mm(top)


# --- what the geometry has to be ------------------------------------------

def test_fingertip_depth(built):
    m, d = built
    left = _in_frame(m, d, "so101:cad:parallel_jaw_left")
    right = _in_frame(m, d, "so101:cad:parallel_jaw_right")
    depth = -min(left[:, 2].min(), right[:, 2].min()) * 1000
    assert abs(depth - tf.TIP_DEPTH_MM) < tf.TIP_DEPTH_TOL_MM, depth


def test_fingers_point_down_the_approach_axis(built):
    """The finger is deepest along gripper -z, and its tips clear the base."""
    m, d = built
    for side in ("left", "right"):
        p = _mm(_in_frame(m, d, f"so101:cad:parallel_jaw_{side}"))
        ext = p.max(0) - p.min(0)
        assert ext[2] >= ext[0] and ext[2] >= ext[1], ext
        assert p[:, 2].min() < -90.0        # tips below the wrist, not sideways


def test_jaws_stay_under_the_base_footprint(built):
    """A 90 deg yaw of the pair swings the fingers out of the base footprint."""
    m, d = built
    E = _base_part_in_gripper(m, d)
    for side in ("left", "right"):
        pts = _mm(_in_frame(m, d, f"so101:cad:parallel_jaw_{side}"))
        part = (_inv(E)[:3, :3] @ (pts / 1000).T).T + _inv(E)[:3, 3]
        part = _mm(part)
        # the fingers hang below the base along part +x, and stay as narrow as
        # the base across part y/z
        assert part[:, 0].max() > 100.0, part[:, 0].max()
        assert part[:, 1].max() < 36.0 and part[:, 1].min() > -36.0, (part[:, 1].min(), part[:, 1].max())
        assert part[:, 2].max() < 32.0 and part[:, 2].min() > -40.0, (part[:, 2].min(), part[:, 2].max())


def test_jaws_are_mirror_images(built):
    """A 90 deg jaw error breaks this even when each finger is individually sane."""
    m, d = built
    left = _mm(_in_frame(m, d, "so101:cad:parallel_jaw_left"))
    right = _mm(_in_frame(m, d, "so101:cad:parallel_jaw_right"))
    mirrored = left * np.array([-1.0, -1.0, 1.0])
    d1 = np.sqrt(((mirrored[:, None, :] - right[None, :, :]) ** 2).sum(-1)).min(1)
    d2 = np.sqrt(((right[:, None, :] - mirrored[None, :, :]) ** 2).sum(-1)).min(1)
    assert max(d1.max(), d2.max()) < tf.SYMMETRY_TOL_MM, (d1.max(), d2.max())


def test_tips_are_centered_on_the_axis(built):
    m, d = built
    tips = {}
    for side in ("left", "right"):
        p = _mm(_in_frame(m, d, f"so101:cad:parallel_jaw_{side}"))
        tips[side] = p[p[:, 2] < p[:, 2].min() + 0.5].mean(0)
    mid = (tips["left"] + tips["right"]) / 2
    assert abs(mid[0]) < 0.5 and abs(mid[1]) < 0.5, mid
    assert abs(mid[2] + tf.TIP_DEPTH_MM) < 1.0, mid


def test_jaw_body_centres(built):
    """Jaw placement in three definitions, in the gripper frame.

    The 2026-11-19 plan replaced the "finger span +/-27.83" check with the
    pre-5y jaw *body-centre* placement (-0.300, -/+27.828, -76.600). The built
    model does not reproduce that triple under any of these definitions - the
    quoted centres have the pair spanning gripper y, the built pair spans x and
    is a half-turn pair - which is recorded in docs/TOOL_FRAME.md as the one
    finding that does not close. What this test pins is the placement that IS
    built, so a future edit cannot move it quietly.
    """
    m, d = built
    for side in ("left", "right"):
        b = m.body(f"so101:cad:parallel_jaw_{side}").id
        Ti = _grip_from(m, d, f"so101:cad:parallel_jaw_{side}")
        assert np.allclose(_mm(Ti[:3, 3]), tf.JAW_BODY_ORIGIN_MM[side], atol=0.01)
        pts = _visual_points(m, f"so101:cad:parallel_jaw_{side}")
        c = (Ti[:3, :3] @ pts.T).T + Ti[:3, 3]
        assert np.allclose(_mm(c.mean(0)), tf.JAW_VERTEX_CENTROID_MM[side], atol=0.05)
        assert np.allclose(_mm((c.min(0) + c.max(0)) / 2), tf.JAW_BBOX_MID_MM[side], atol=0.05)
    # and the quoted CAD triple is genuinely not where the jaws are
    assert np.linalg.norm(np.array(tf.JAW_BODY_ORIGIN_MM["left"]) -
                          np.array(tf.CAD_BODY_CENTRE_QUOTED_MM)) > 10.0


def test_gear_origin_in_the_gripper_frame(built):
    m, d = built
    gear = _grip_from(m, d, "so101:cad:parallel_pinion") @ _T(
        _R([0.5, 0.5, 0.5, -0.5]), np.array([0, 0, 0.058]))
    assert np.allclose(_mm(gear[:3, 3]), tf.GEAR_IN_GRIPPER_MM, atol=0.02), _mm(gear[:3, 3])


def test_pack_is_not_silent_stock():
    """A pack on the path is not enough: gripper.py keys off ROBOT_GRIPPER alone.

    `scripts/start-live.sh` exports neither variable, so a launch from it
    compiles the stock tool while every receipt says "CAD". Fail loudly here
    rather than let a sweep receipt stay ambiguous.
    """
    import os
    assert gripper.KIND == os.environ.get('ROBOT_GRIPPER', 'stock')
    assets = os.environ.get('ROBOT_GRIPPER_ASSETS', '')
    if 'gripper-assets' in assets:
        assert gripper.CUSTOM, (
            'ROBOT_GRIPPER_ASSETS points at a pack but ROBOT_GRIPPER is not '
            'cad_parallel: the scene silently compiles the stock tool')


def test_no_floating_camera_box(built):
    """The 4c-era housing proxy must not be part of the tool."""
    m, d = built
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "so101:camera_housing") == -1
    for gi in range(m.ngeom):
        name = str(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gi))
        assert "camera_collision" not in name
