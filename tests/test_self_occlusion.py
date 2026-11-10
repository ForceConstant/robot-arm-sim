"""Inference-only self-model tests: no renderer or simulator body/camera truth."""
import numpy as np
import pytest

from chess_sim.board_registration import BoardRegistration
from chess_sim.perception import CLASSES, SquarePrediction
from chess_sim.self_occlusion import SelfOcclusionModel
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import ScanFrame, WristScanner


def _model(triangle, padding=0):
    calibration = WristCalibration(
        64, 48, (40., 40., 32., 24.), (0.,) * 5,
        ({'name': 'gripper', 'position': [0., 0., 0.], 'rotation': np.eye(3).tolist(),
          'joint_index': 0, 'anchor': [0., 0., 0.], 'axis': [0., 0., 1.],
          'reference': 0., 'limits': [-1., 1.]},), np.eye(4).tolist(), 'unit')
    model = SelfOcclusionModel.__new__(SelfOcclusionModel)
    vertices = np.asarray(triangle, dtype=float)
    model.calibration = calibration
    model.padding_pixels = padding
    model._custom = True
    model._jaw_limits = (-1., 1.)
    model._parts = [('gripper', vertices, np.eye(4))]
    model._triangles = {id(vertices): vertices[None]}
    model._tool_parts = lambda joints: []
    model._mask_cache = {}
    return model


def test_projection_tracks_measured_joint_rotation_and_cache_is_not_mutable():
    model = _model([[.1, .1, -1.], [.4, .1, -1.], [.1, .4, -1.]])
    original = model.exclusion_mask(np.zeros(6)).copy()
    assert original[16, 40] and not original[32, 40]
    rotated = model.exclusion_mask([1., 0., 0., 0., 0., 0.])
    assert not np.array_equal(original, rotated)
    original[:] = False
    assert model.exclusion_mask(np.zeros(6)).any()


def test_near_plane_clipping_and_behind_camera_rejection():
    assert not _model([[-.1, -.1, 1.], [.1, -.1, 1.], [0., .1, 1.]]).exclusion_mask(np.zeros(6)).any()
    mask = _model([[.1, .1, -.5], [.2, .1, -.5], [.15, .2, .1]]).exclusion_mask(np.zeros(6))
    assert mask.any() and not mask.all()


def test_pixel_margin_is_conservative_and_invalid_joints_raise():
    triangle = [[-.1, -.1, -1.], [.1, -.1, -1.], [0., .1, -1.]]
    base = _model(triangle).exclusion_mask(np.zeros(6))
    model = _model(triangle, padding=2)
    padded = model.exclusion_mask(np.zeros(6))
    assert ((padded | base) == padded).all() and padded.sum() > base.sum()
    for joints in ([0.] * 5, [float('nan')] * 6, [2., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 2.]):
        with pytest.raises(ValueError):
            model.exclusion_mask(joints)


def _scanner(monkeypatch, covered):
    import chess_sim.wrist_scan as module
    scanner = WristScanner.__new__(WristScanner)
    scanner.calibration = type('Calibration', (), {'rectify': staticmethod(lambda frame: frame)})()
    scanner.cameras = lambda joints: None
    scanner._reference_patches = lambda *args: None
    scanner.self_model = type('Mask', (), {'exclusion_mask': staticmethod(lambda joints: np.zeros((48, 64), bool))})()
    scanner.model = type('Model', (), {'confidence_threshold': .75})()
    probs = tuple(.97 if c == 'white' else .01 for c in CLASSES)
    monkeypatch.setattr(module, 'predict_board', lambda *args, **kwargs: [SquarePrediction(i, 'white', .97, probs, 1.) for i in range(64)])
    monkeypatch.setattr(module, 'square_mask_fraction', lambda *args: covered)
    return scanner


def test_intersecting_arm_patch_never_votes_but_clear_patch_keeps_threshold(monkeypatch):
    covered = np.zeros(64)
    covered[12] = 1 / 256  # even one sampled pixel vetoes the square/view
    scanner = _scanner(monkeypatch, covered)
    frame = np.tile(np.arange(64, dtype=np.uint8)[None, :, None] * 3, (48, 1, 3))
    result = scanner.perceive([ScanFrame(frame, (0.,) * 6)], BoardRegistration((0., 0.), 0., .028, 0., 1., 1.))
    assert result.predictions[12].label == 'unknown'
    assert result.predictions[12].valid_fraction == 0.
    assert result.predictions[11].label == 'white'


def test_blank_frame_probabilities_cannot_be_revived_by_fusion(monkeypatch):
    scanner = _scanner(monkeypatch, np.zeros(64))
    result = scanner.perceive([ScanFrame(np.zeros((48, 64, 3), np.uint8), (0.,) * 6)], BoardRegistration((0., 0.), 0., .028, 0., 1., 1.))
    assert all(p.label == 'unknown' for p in result.predictions)
