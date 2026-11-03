"""Verify span-only corrections across all four current tall-piece failures.

This is a candidate-parameter experiment, not a deployed planner change.
"""
import hashlib
import json
from pathlib import Path
import sys
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from chess_sim import gripper
from chess_sim.controller import PlanOverride
from chess_sim.env import ChessSimEnv
from chess_sim.product import apply_mass_model, load_product_twin
from chess_sim.tasks import MoveTask
from robot_web.torture import CASES


def main():
    assert gripper.CUSTOM
    twin = load_product_twin()
    layout = json.loads((ROOT / 'hardware/current_layout.json').read_text())
    cfg = twin.env_config(cameras=())
    cfg = cfg.model_copy(update={'board': cfg.board.model_copy(update={
        'arm_gap': layout['mount']['board_edge_to_base_frame_mm']/1000,
        'arm_riser': layout['mount']['base_mount_datum_above_table_mm']/1000})})
    report = dict(schema='robot-arm-phase8-span-verification/1', layout=layout,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  scope='All 25 crowded-v3 cases, same per-case seeds as requalification. Only white queen/king cases receive a forced +20 degree span. Candidate parameters, not live defaults.', rows=[])
    path = ROOT / 'reports/phase8_updated_span_verification.json'
    env = ChessSimEnv(cfg)
    try:
        for case in CASES:
            seed = zlib.crc32(case.id.encode()) & 0xffffffff
            env.reset(case.fen)
            apply_mass_model(env, twin, all_slots=True)
            env.reset(case.fen)
            env._noise_rng = np.random.default_rng(seed)
            params = {'span_deg': 20} if case.id in ('white-queen', 'white-queen-far', 'white-king', 'white-king-far') else {}
            task = MoveTask(from_square=case.move[:2], to_square=case.move[2:])
            try:
                result = env.execute(task, override=PlanOverride(**params))
                outcome = dict(success=bool(result.success), error_mm=result.placement_error*1000,
                               disturbed=[str(s) for s in result.disturbed], reason=str(result.reason),
                               precise=bool(result.waypoints_precise))
            except Exception as exc:
                outcome = dict(success=False, reason=f'{type(exc).__name__}: {exc}')
            report['rows'].append(dict(case=case.id, move=case.move, seed=seed, params=params, **outcome))
            report.update(total=len(report['rows']), passed=sum(r['success'] for r in report['rows']))
            path.write_text(json.dumps(report, indent=2)+'\n')
            print(case.id, params, outcome, flush=True)
    finally:
        env.close()


if __name__ == '__main__':
    main()
