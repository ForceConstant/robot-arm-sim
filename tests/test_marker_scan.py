"""Marker-coverage planning: projection, occlusion and offset arithmetic."""
from __future__ import annotations

import math

import numpy as np
import pytest

from chess_sim.fiducials import MarkerLayout
from chess_sim.marker_scan import (MarkerScanPlan, PoseCoverage, board_offset,
                                   load_plan, marker_corners_world,
                                   observe_markers, plan_from_observations,
                                   refine_target, save_plan, select_poses)
from chess_sim.vision_geometry import CameraCalibration

LAYOUT = MarkerLayout.for_board(224.0, 15.0, plane="mat", margin_mm=40.0)


def looking_down(height: float = 0.45, size=(640, 480), focal: float = 500.0,
                 centre=(0.0, 0.0), marker_z: float = 0.43) -> CameraCalibration:
    width, height_px = size
    return CameraCalibration(width, height_px, focal, focal,
                             width / 2.0, height_px / 2.0,
                             (centre[0], centre[1], marker_z + height),
                             (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))


def test_layout_offset_is_half_field_plus_margin():
    assert LAYOUT.offset_mm == pytest.approx(112.0 + 40.0)


def test_marker_corners_rotate_with_board():
    plain = marker_corners_world(LAYOUT, 2, marker_z=0.43)
    turned = marker_corners_world(LAYOUT, 2, yaw=math.pi / 2, marker_z=0.43)
    assert turned[0][0] == pytest.approx(-plain[0][1], abs=1e-9)
    assert turned[0][1] == pytest.approx(plain[0][0], abs=1e-9)


def test_all_four_markers_visible_from_one_overhead_pose():
    found = observe_markers(looking_down(), (0,) * 5 + (0.6,), LAYOUT,
                            marker_z=0.43)
    assert sorted(found) == [0, 1, 2, 3]
    for observation in found.values():
        assert observation.edge_px > 40.0
        assert 0 < observation.centre_px[0] < 640
        assert 0 < observation.centre_px[1] < 480


def test_distant_pose_rejects_undecodable_markers():
    found = observe_markers(looking_down(height=1.2), (0,) * 6, LAYOUT,
                            marker_z=0.43)
    assert found == {}


def test_edge_margin_rejects_half_visible_marker():
    # 0.30 m puts the far marker centre past the frame edge.
    found = observe_markers(looking_down(height=0.30), (0,) * 6, LAYOUT,
                            marker_z=0.43)
    assert 2 not in found and 0 not in found


def test_occlusion_mask_removes_markers():
    frame = looking_down()
    empty = observe_markers(frame, (0,) * 6, LAYOUT, marker_z=0.43)
    mask = np.ones((frame.height, frame.width), dtype=bool)
    blocked = observe_markers(frame, (0,) * 6, LAYOUT, marker_z=0.43, occlusion=mask)
    assert empty and blocked == {}


def test_occlusion_mask_only_where_drawn():
    frame = looking_down()
    mask = np.zeros((frame.height, frame.width), dtype=bool)
    mask[0:10, 0:10] = True
    found = observe_markers(frame, (0,) * 6, LAYOUT, marker_z=0.43, occlusion=mask)
    assert sorted(found) == [0, 1, 2, 3]


def pose(name: str, visible, edges=None) -> PoseCoverage:
    edges = edges or [50.0] * len(visible)
    return PoseCoverage(name, (0.0,) * 6, tuple(visible), tuple(edges))


def test_greedy_prefers_the_widest_single_view():
    candidates = [pose("pair", (0, 1)), pose("other", (2, 3)),
                  pose("all", (0, 1, 2, 3), [45.0] * 4)]
    chosen = select_poses(candidates, min_views=1)
    assert [p.name for p in chosen] == ["all"]


def test_greedy_reaches_min_views():
    candidates = [pose("pair", (0, 1)), pose("other", (2, 3)),
                  pose("all", (0, 1, 2, 3))]
    chosen = select_poses(candidates, min_views=2)
    assert len(chosen) == 3
    counts = {m: sum(1 for p in chosen if m in p.visible) for m in range(4)}
    assert all(count >= 2 for count in counts.values())


def test_greedy_uses_edge_quality_as_tiebreak():
    candidates = [pose("narrow", (0,), [37.0]), pose("wide", (0,), [60.0])]
    chosen = select_poses(candidates, min_views=1)
    assert [p.name for p in chosen] == ["wide"]


def test_greedy_rejects_a_duplicate_viewpoint():
    def pose(name, visible, edge, position):
        from chess_sim.marker_scan import PoseCoverage
        return PoseCoverage(name, (0.0,) * 6, (visible,), (edge,), position)

    candidates = [pose("a", 0, 45.0, (0.0, 0.0, 0.70)),
                  pose("b", 0, 45.0, (0.002, 0.0, 0.70)),
                  pose("c", 0, 40.0, (0.0, 0.0, 0.90))]
    chosen = select_poses(candidates, min_views=2, marker_ids=(0,),
                          min_separation_m=0.025)
    assert [p.name for p in chosen] == ["a", "c"]


def test_greedy_falls_back_when_no_diverse_view_exists():
    def pose(name, visible, edge, position):
        from chess_sim.marker_scan import PoseCoverage
        return PoseCoverage(name, (0.0,) * 6, (visible,), (edge,), position)

    candidates = [pose("a", 0, 45.0, (0.0, 0.0, 0.70)),
                  pose("b", 0, 44.0, (0.001, 0.0, 0.70))]
    chosen = select_poses(candidates, min_views=2, marker_ids=(0,))
    assert len(chosen) == 2


def test_plan_reports_missing_markers():
    plan = plan_from_observations([
        ("a", (0.0,) * 6, {0: _obs(0), 1: _obs(1)}),
    ], min_views=2)
    assert not plan.complete()
    assert plan.missing() == (0, 1, 2, 3)


def _obs(marker_id: int):
    from chess_sim.marker_scan import MarkerObservation
    corners = ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0))
    return MarkerObservation(marker_id, corners, 10.0)


def test_plan_round_trip(tmp_path):
    plan = plan_from_observations([
        ("a", (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
         {m: _obs(m) for m in range(4)}),
        ("b", (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
         {m: _obs(m) for m in range(4)}),
    ], min_views=2)
    path = save_plan(plan, tmp_path / "plan.json")
    loaded = load_plan(path)
    assert loaded.min_views == 2
    assert [p.name for p in loaded.poses] == [p.name for p in plan.poses]
    assert loaded.complete()


def test_board_offset_nominal_layout():
    offset = board_offset((0.0, 0.0), 0.0, base_xy=(0.0, -0.217),
                          board_half_mm=127.0)
    assert offset.x_mm == pytest.approx(0.0, abs=1e-9)
    assert offset.y_mm == pytest.approx(217.0)
    assert offset.yaw_deg == pytest.approx(0.0)
    # 90 mm gap is the qualified layout invariant.
    assert offset.edge_gap_mm == pytest.approx(90.0)


def test_board_offset_with_rotated_board():
    offset = board_offset((0.0, 0.0), math.pi / 2, base_xy=(0.0, -0.217),
                          board_half_mm=127.0)
    assert offset.yaw_deg == pytest.approx(90.0)
    assert offset.edge_gap_mm == pytest.approx(90.0)


def test_board_offset_inside_board_is_negative():
    offset = board_offset((0.0, 0.0), 0.0, base_xy=(0.0, -0.05),
                          board_half_mm=127.0)
    assert offset.edge_gap_mm == pytest.approx(-77.0)


def test_committed_plan_covers_every_marker_twice():
    from pathlib import Path
    plan = load_plan(Path(__file__).resolve().parents[1]
                     / "chess_sim" / "assets" / "marker_scan_plan.json")
    assert plan.complete(), plan.missing()
    assert all(count >= 2 for count in plan.views_per_marker().values())
    # Every pose that resolves a marker at nominal placement must clear the
    # decode threshold; tolerant plans also carry poses that only pay off off
    # nominal, and those legitimately see nothing here.
    for pose in plan.poses:
        assert len(pose.joints) == 6
        if pose.edge_px:
            assert min(pose.edge_px) >= 36.0


def test_committed_plan_has_independent_views():
    from pathlib import Path
    plan = load_plan(Path(__file__).resolve().parents[1]
                     / "chess_sim" / "assets" / "marker_scan_plan.json")
    for marker in range(4):
        seen = [p for p in plan.poses
                if marker in p.visible and p.camera_position is not None]
        assert len(seen) >= 2
        positions = [np.asarray(p.camera_position) for p in seen]
        assert max(np.linalg.norm(a - b)
                   for a in positions for b in positions) > 0.05


def test_tolerant_plan_keeps_nominal_quality():
    import json
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    raw = json.loads((root / "chess_sim" / "assets"
                      / "marker_scan_plan_50mm.json").read_text())
    plan = load_plan(root / "chess_sim" / "assets" / "marker_scan_plan_50mm.json")
    assert raw["tolerance_mm"] == 50.0
    assert raw["tolerance_yaw_deg"] == 5.0
    # The tolerant set must still cover the nominal placements twice over.
    assert all(count >= 2 for count in plan.views_per_marker().values())
    assert set(raw["recovery"]) == {"0", "1", "2", "3"}
    assert "placements reach" in plan.note


def test_refine_target_sits_above_the_missing_marker():
    target = refine_target(LAYOUT, 2, marker_z=0.43, height_m=0.24)
    assert target[0] == pytest.approx(0.152)
    assert target[1] == pytest.approx(0.152)
    assert target[2] == pytest.approx(0.67)
