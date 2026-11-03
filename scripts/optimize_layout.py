"""Bounded, execution-based layout search for one follower and the printed set.

Private CAD/piece assets must be supplied with ROBOT_*_ASSETS. Every case is
independent and uses the real contact controller/evaluator; moves need not be
chess-legal. Rank success first, then worst and mean placement error. This is a
best-tested layout, never a proof of a global optimum or physical-arm reach.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from chess_sim.layout_cases import CASES,case_fen

def full_board_cases():
    import chess
    rows=[]
    for i,source in enumerate(chess.SQUARE_NAMES):
        square=chess.parse_square(source)
        target=square+8 if chess.square_rank(square)<7 else square-8
        rows.append((f'square-{source}','rook' if i%2==0 else 'king',source,chess.square_name(target)))
    return tuple(rows)


def run_layout(spec,case_definitions=CASES):
    import chess
    import mink
    import numpy as np
    from chess_sim.env import ChessSimEnv
    from chess_sim.product import load_product_twin,apply_mass_model
    gap,height,dx,dy=spec
    twin=load_product_twin()
    cfg=twin.env_config(piece_set='printed',cameras=())
    cfg=cfg.model_copy(update={'board':cfg.board.model_copy(update={
        'arm_gap':gap/1000,'arm_riser':height/1000,'origin':(dx/1000,dy/1000)})})
    env=ChessSimEnv(cfg)
    rows=[]
    started=time.monotonic()
    try:
        for ident,kind,source,destination in case_definitions:
            fen=case_fen(kind,source)
            board=chess.Board(fen);move=chess.Move.from_uci(source+destination)
            assert board.turn==chess.WHITE and not board.is_capture(move) and not board.is_castling(move)
            seed=zlib.crc32(ident.encode())&0xffffffff
            env.reset(fen);apply_mass_model(env,twin,'nominal');env.reset(fen)
            env._noise_rng=np.random.default_rng(seed)
            slot=env.slot_at(source);z0=float(env.piece_position(slot)[2]);max_z=z0
            close_error=None;close_tilt=None;phases=[]
            def record(action):
                nonlocal max_z,close_error,close_tilt
                max_z=max(max_z,float(env.piece_position(slot)[2]))
                phase=getattr(env,'diagnostic_phase','idle')
                if not phases or phases[-1]!=phase:phases.append(phase)
                if phase=='close':
                    pos,rot=env.ik.tool_pose(env.data,env.diagnostic_offset)
                    close_error=float(np.linalg.norm(pos-np.asarray(env.diagnostic_target)))*1000
                    close_tilt=float(np.degrees(np.arccos(np.clip(-rot[2,0],-1,1))))
            try:
                result=env.move(source,destination,on_step=record)
            except mink.exceptions.NoSolutionFound as exc:
                # An infeasible candidate must remain a named failed case, not
                # abort the entire sweep or silently disappear from its score.
                error=float(np.linalg.norm(env.piece_position(slot)[:2]-np.asarray(env.board.square_center(destination))))*1000
                rows.append(dict(case=ident,piece=kind,source=source,destination=destination,seed=seed,
                    success=False,error_mm=round(error,4),upright=bool(env.piece_upright(slot)>=cfg.tolerances.upright_min),
                    picked=None,precise=False,reason=f'IK solver: {exc}',lift_mm=round((max_z-z0)*1000,3),
                    close_error_mm=close_error,close_tilt_deg=close_tilt,stall='ik_no_solution',phases=phases,steps=None))
                continue
            rows.append(dict(case=ident,piece=kind,source=source,destination=destination,seed=seed,
                success=bool(result.success),error_mm=round(float(result.placement_error)*1000,4),
                upright=bool(result.upright),picked=result.picked,
                precise=bool(result.waypoints_precise),reason=str(result.reason) if result.reason else None,
                lift_mm=round((max_z-z0)*1000,3),close_error_mm=close_error,
                close_tilt_deg=close_tilt,stall=getattr(env,'diagnostic_stall',None),phases=phases,
                steps=result.steps))
        errors=[r['error_mm'] for r in rows]
        return dict(gap_mm=gap,riser_mm=height,board_offset_mm=[dx,dy],
            arm_base_m=list(cfg.board.arm_base),passed=sum(r['success'] for r in rows),total=len(rows),
            worst_error_mm=max(errors),mean_error_mm=sum(errors)/len(errors),
            elapsed_s=round(time.monotonic()-started,2),cases=rows)
    finally:env.close()

def rank(row):
    return (-row['passed'],row['worst_error_mm'],row['mean_error_mm'],row['riser_mm'],row['gap_mm'])

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--gaps',default='60,80,100')
    ap.add_argument('--heights',default='0,20,40,60')
    ap.add_argument('--offsets',default='0:0',help='board x:y perturbations in mm, comma-separated')
    ap.add_argument('--workers',type=int,default=2)
    ap.add_argument('--full-board',action='store_true',help='one adjacent transfer from each square, alternating rook and king')
    ap.add_argument('--resume',action='store_true',help='retain compatible finished layout rows and run missing candidates')
    ap.add_argument('--output',type=Path,default=ROOT/'reports/layout_search.json')
    args=ap.parse_args()
    from chess_sim import gripper,ik
    assert gripper.CUSTOM,'Search must use the deployed custom jaw'
    gaps=[float(v) for v in args.gaps.split(',')];heights=[float(v) for v in args.heights.split(',')]
    offsets=[tuple(map(float,p.split(':'))) for p in args.offsets.split(',')]
    specs=[(g,h,*o) for g in gaps for h in heights for o in offsets]
    case_definitions=full_board_cases() if args.full_board else CASES
    report=dict(schema='robot-arm-layout-search/1',tool=gripper.description(),
        point_cost=ik.POSITION_COST,tilt_cost=ik.TILT_COST,mujoco=__import__('mujoco').__version__,
        board_thickness_mm=3.2,rook_mass_g=4.8,king_mass='unmeasured nominal PLA model',
        ranking='passed descending, worst placement error ascending, mean error ascending; lower riser then gap breaks ties',
        qualification='single white follower; isolated contact moves, not physical or crowded-board proof',
        piece_manifest_sha256=hashlib.sha256((Path(os.environ['ROBOT_PIECE_ASSETS'])/'set.json').read_bytes()).hexdigest(),
        controller_sha256=hashlib.sha256((ROOT/'chess_sim/controller.py').read_bytes()).hexdigest(),
        ik_sha256=hashlib.sha256((ROOT/'chess_sim/ik.py').read_bytes()).hexdigest(),
        expected_layouts=len(specs),case_definitions=case_definitions,layouts=[])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.resume and args.output.exists():
        old=json.loads(args.output.read_text())
        for key in ('controller_sha256','ik_sha256','piece_manifest_sha256'):
            if old[key]!=report[key]:raise ValueError(f'Cannot resume across changed {key}')
        if json.dumps(old['case_definitions'])!=json.dumps(case_definitions):
            raise ValueError('Cannot resume across changed cases')
        report['layouts']=old['layouts']
        if report['layouts']:report['best']=min(report['layouts'],key=rank)
        completed={(r['gap_mm'],r['riser_mm'],*r['board_offset_mm']) for r in old['layouts']}
        specs=[s for s in specs if s not in completed]
    def save():args.output.write_text(json.dumps(report,indent=2)+'\n')
    save()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(run_layout,s,case_definitions):s for s in specs}
        for fut in as_completed(futures):
            row=fut.result();report['layouts'].append(row);report['layouts'].sort(key=rank)
            report['best']=report['layouts'][0];save()
            print(f"{len(report['layouts'])}/{report['expected_layouts']} gap={row['gap_mm']:g} riser={row['riser_mm']:g} offset={row['board_offset_mm']} {row['passed']}/{row['total']} worst={row['worst_error_mm']:.2f} mean={row['mean_error_mm']:.2f}",flush=True)
    print('BEST',json.dumps({k:v for k,v in report['best'].items() if k!='cases'}),flush=True)

if __name__=='__main__':main()
