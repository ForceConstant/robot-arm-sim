"""Bounded, same-state parameter sweeps for the current crowded queen/king.

Uses the requalification's mount and mass model. Reports neighbour displacement
by controller phase; never modifies geometry or evaluator tolerances.
"""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import chess
import numpy as np
from chess_sim.controller import PlanOverride, grasp_plan
from chess_sim.env import ChessSimEnv
from chess_sim.product import apply_mass_model, load_product_twin
from chess_sim.tasks import MoveTask
from chess_sim import gripper


def main():
    assert gripper.CUSTOM
    twin = load_product_twin()
    layout = json.loads((ROOT / 'hardware/current_layout.json').read_text())
    cfg = twin.env_config(cameras=())
    cfg = cfg.model_copy(update={'board': cfg.board.model_copy(update={
        'arm_gap': layout['mount']['board_edge_to_base_frame_mm']/1000,
        'arm_riser': layout['mount']['base_mount_datum_above_table_mm']/1000})})
    report = dict(schema='robot-arm-phase8-parameter-sweep/1', layout=layout,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  scope='Fresh identical nominal starts, seed 0, two representative crowded failures; not historical incident replay or exhaustive geometry proof.', rows=[])
    path = ROOT / 'reports/phase8_updated_parameter_sweep.json'
    env = ChessSimEnv(cfg)
    try:
        for uci in ('d1d3', 'e1e3'):
            for params in ({}, {'grasp_dz': -.004}, {'grasp_dz': .004},
                           {'span_deg': -20}, {'span_deg': 20},
                           {'place_dy': -.002}, {'place_dy': .002}):
                env.reset(chess.STARTING_FEN)
                apply_mass_model(env, twin, all_slots=True)
                env.reset(chess.STARTING_FEN)
                env._noise_rng = np.random.default_rng(0)
                task = MoveTask(from_square=uci[:2], to_square=uci[2:])
                slot = env.slot_at(uci[:2])
                plan = grasp_plan(slot.geometry, env.board)
                point = np.array([*env.piece_position(slot)[:2], plan.z_grasp])
                spans = [{'direction': list(d), 'margin_mm': margin*1000}
                         for d, margin in env.span_options(point[:2], exclude=slot)]
                before = {s: env.piece_position(sl)[:2].copy() for s, sl in env._square_slot.items() if sl is not slot}
                crossings = {}
                z0 = float(env.piece_position(slot)[2])
                max_z = z0
                def sample(action):
                    nonlocal max_z
                    max_z = max(max_z, float(env.piece_position(slot)[2]))
                    for s, xy in before.items():
                        distance = float(np.linalg.norm(env.piece_position(env._square_slot[s])[:2]-xy))
                        if distance > cfg.tolerances.disturbance and s not in crossings:
                            crossings[s] = dict(square=chess.square_name(s), phase=env.diagnostic_phase,
                                                distance_mm=distance*1000)
                try:
                    result = env.execute(task, override=PlanOverride(**params), on_step=sample)
                    outcome = dict(success=bool(result.success), error_mm=result.placement_error*1000,
                                   disturbed=[str(s) for s in result.disturbed], reason=str(result.reason),
                                   precise=bool(result.waypoints_precise))
                except Exception as exc:
                    outcome = dict(success=False, reason=f'{type(exc).__name__}: {exc}')
                report['rows'].append(dict(move=uci, params=params, **outcome,
                    lift_mm=(max_z-z0)*1000, first_disturbance=list(crossings.values()),
                    nominal_span_options=spans))
                path.write_text(json.dumps(report, indent=2)+'\n')
                print(uci, params, outcome, flush=True)
    finally:
        env.close()


if __name__ == '__main__':
    main()
