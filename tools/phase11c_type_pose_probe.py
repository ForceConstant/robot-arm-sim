#!/usr/bin/env python3
"""Phase 11c(b) scoped local type-ID+pose training probe (fast baseline, numpy/cv2 only).

- Generates labeled per-square wrist features from registered scans (sim truth for labels only).
- Trains a small numpy MLP for 6-way piece type classification.
- Saves/reloads model and evaluates held-out type accuracy + end-to-end type+pose
  (B1 mesh-axis fit using predicted type; knight yaw/axis metrics reported).
- Emits a receipt with dataset/manifest summary, train/reload metrics, held-out metrics,
  and throughput/runtime evidence. Intended to be run under phase11c_resource_harness.py
  for memory/thermal evidence.
"""
from __future__ import annotations
import argparse, hashlib, json, math, os, random, time, traceback
from pathlib import Path
from collections import Counter

import numpy as np
import cv2  # noqa: F401
import chess
import mujoco

from chess_sim.config import Camera, RandomizationConfig
from chess_sim import gripper
from chess_sim.env import ChessSimEnv
from chess_sim.fiducials import MarkerLayout
from chess_sim.product import layout_env_config, load_product_twin
from chess_sim.positions import PositionConfig
from chess_sim.wrist_calibration import WristCalibration
from chess_sim.wrist_scan import ScanFrame, WristScanner, load_plan
from chess_sim.self_occlusion import SelfOcclusionModel
from chess_sim.perception import board_patches, square_features, normalize_per_frame, FEATURE_COUNT
from chess_sim.piece_orientation import OrientationView, fit_mesh_axis, axis_distance_degrees
from chess_sim.assets import piece_obj_path, piece_texture_path, piece_tint

TYPE_LIST = [chess.PAWN, chess.ROOK, chess.KNIGHT, chess.BISHOP, chess.QUEEN, chess.KING]
TYPE_NAME = {chess.PAWN:"pawn", chess.ROOK:"rook", chess.KNIGHT:"knight", chess.BISHOP:"bishop", chess.QUEEN:"queen", chess.KING:"king"}
N_TYPES = 6


def make_env(seed: int):
    twin = load_product_twin()
    draw = np.random.default_rng(seed)
    appearance = twin.appearance("printed").model_copy(update={
        "light_intensity": float(draw.uniform(0.82, 1.15)),
        "ambient": float(draw.uniform(0.0, 0.05)),
        "light_dir": (float(draw.uniform(-0.35, -0.2)), float(draw.uniform(0.2, 0.4)), -1.0),
        "fill_intensity": float(draw.uniform(0.7, 1.3)),
        "shadow_softness": float(draw.uniform(0.015, 0.035)),
    })
    config = layout_env_config("printed", cameras=(Camera.WRIST,)).model_copy(update={
        "appearance": appearance,
        "randomization": RandomizationConfig(board_shift=0.008, board_yaw=math.radians(2.5), board_side=0.008, arm_joint_jitter=0.0, camera=False),
    })
    return ChessSimEnv(config)

def park(env, joints):
    cmd = np.asarray(joints, dtype=float).copy()
    cmd[5] = np.clip(cmd[5], gripper.JOINT_MIN, gripper.JOINT_MAX)
    env.data.qpos[env._joint_qpos] = cmd
    gripper.seed_coupled(env.model, env.data, "so101:")
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    return tuple(np.asarray(env.data.qpos[env._joint_qpos], dtype=float).tolist())

def capture_scan_frames(env, poses):
    frames = []
    for pose in poses:
        measured = park(env, pose.joints)
        frames.append(ScanFrame(env.render(Camera.WRIST).copy(), measured, 0.0, pose.name))
    return frames

def capture_views(env, scanner, poses, self_model):
    views = []
    for pose in poses:
        measured = park(env, pose.joints)
        rgb = scanner.calibration.rectify(env.render(Camera.WRIST).copy())
        cam = scanner.cameras(measured)
        mask = self_model.exclusion_mask(measured) if self_model is not None else None
        views.append(OrientationView(pose.name, rgb, cam, mask))
    return views

class TypeMLP:
    def __init__(self, mean, std, W1, b1, W2, b2, meta=None):
        self.mean=np.asarray(mean,np.float32); self.std=np.asarray(std,np.float32)
        self.W1=np.asarray(W1,np.float32); self.b1=np.asarray(b1,np.float32)
        self.W2=np.asarray(W2,np.float32); self.b2=np.asarray(b2,np.float32)
        self.meta=dict(meta or {})
    def probs(self, x):
        x=np.atleast_2d(np.asarray(x,np.float32))
        z=(x-self.mean)/np.where(self.std<1e-6,1.0,self.std)
        h=np.tanh(z@self.W1+self.b1)
        logits=h@self.W2+self.b2; logits-=logits.max(axis=1,keepdims=True)
        e=np.exp(logits); return e/e.sum(axis=1,keepdims=True)
    def predict(self,x):
        p=self.probs(x); return p.argmax(axis=1), p.max(axis=1), p
    def save(self,path):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        payload={"schema":"phase11c-type-mlp/v1","classes":[TYPE_NAME[t] for t in TYPE_LIST],
                 "mean":self.mean.tolist(),"std":self.std.tolist(),
                 "W1":self.W1.tolist(),"b1":self.b1.tolist(),"W2":self.W2.tolist(),"b2":self.b2.tolist(),
                 "meta":self.meta}
        path.write_text(json.dumps(payload,indent=1)+"\n")
        return hashlib.sha256(path.read_bytes()).hexdigest()
    @classmethod
    def load(cls,path):
        d=json.loads(Path(path).read_text())
        return cls(d["mean"],d["std"],d["W1"],d["b1"],d["W2"],d["b2"],d.get("meta",{}))

def train_type_mlp(X,y,hidden=48,epochs=350,lr=0.01,l2=1e-3,batch=128,seed=0,val=None):
    X=np.asarray(X,np.float32); y=np.asarray(y,np.int64)
    mean=X.mean(0); std=X.std(0); std=np.where(std<1e-6,1.0,std)
    x=(X-mean)/std
    rng=np.random.default_rng(seed)
    W1=rng.normal(0,1/np.sqrt(X.shape[1]),(X.shape[1],hidden)).astype(np.float32); b1=np.zeros(hidden,np.float32)
    W2=rng.normal(0,1/np.sqrt(hidden),(hidden,N_TYPES)).astype(np.float32); b2=np.zeros(N_TYPES,np.float32)
    params=[W1,b1,W2,b2]; m=[np.zeros_like(p) for p in params]; v=[np.zeros_like(p) for p in params]
    counts=np.bincount(y,minlength=N_TYPES).astype(np.float32)
    w=np.ones(N_TYPES,np.float32); present=counts>0
    if present.any(): w[present]=counts.sum()/(present.sum()*counts[present]); w/=w[present].mean()
    oh=np.eye(N_TYPES, dtype=np.float32)[y]
    best=[p.copy() for p in params]; best_loss=1e9; step=0; hist=[]
    for ep in range(epochs):
        order=rng.permutation(len(x)); tot=0.0
        for s in range(0,len(x),batch):
            idx=order[s:s+batch]; xb,yb,wb=x[idx],oh[idx],w[y[idx]]
            hp=xb@W1+b1; h=np.tanh(hp)
            lg=h@W2+b2; lg-=lg.max(1,keepdims=True); ex=np.exp(lg); pr=ex/ex.sum(1,keepdims=True)
            loss=-np.mean(wb*np.sum(yb*np.log(pr+1e-9),axis=1)); tot+=loss*len(idx)
            dlg=(pr-yb)*wb[:,None]/len(idx)
            dW2=h.T@dlg+l2*W2; db2=dlg.sum(0); dh=(dlg@W2.T)*(1-h*h); dW1=xb.T@dh+l2*W1; db1=dh.sum(0)
            step+=1
            for i,(p,g) in enumerate(zip(params,[dW1,db1,dW2,db2])):
                m[i]=0.9*m[i]+0.1*g; v[i]=0.999*v[i]+0.001*g*g
                mh=m[i]/(1-0.9**step); vh=v[i]/(1-0.999**step)
                p-=lr*mh/(np.sqrt(vh)+1e-8)
        rec={"epoch":ep,"train_loss":float(tot/len(x))}
        if val is not None:
            vx,vy=val; vx=(np.asarray(vx,np.float32)-mean)/std
            vh=np.tanh(vx@W1+b1); vl=vh@W2+b2; vl-=vl.max(1,keepdims=True); ve=np.exp(vl); vp=ve/ve.sum(1,keepdims=True)
            rec["val_loss"]=float(-np.mean(np.log(vp[np.arange(len(vy)),vy]+1e-9)))
            rec["val_acc"]=float((vp.argmax(1)==vy).mean())
            if rec["val_loss"]<best_loss: best_loss=rec["val_loss"]; best=[p.copy() for p in params]
        hist.append(rec)
    if val is not None and best_loss<1e8: W1,b1,W2,b2=best
    model=TypeMLP(mean,std,W1,b1,W2,b2,meta={"epochs":epochs,"hidden":hidden,"train_rows":int(len(x)),"class_counts":counts.astype(int).tolist()})
    return model,{"best_val_loss":None if best_loss>=1e8 else float(best_loss),"final_train_loss":hist[-1]["train_loss"] if hist else None,"epochs":epochs}

def random_case_board(rng, crowded):
    if crowded:
        b=chess.Board(chess.STARTING_FEN)
        for _ in range(rng.randrange(0,6)):
            ms=list(b.legal_moves)
            if not ms: break
            b.push(rng.choice(ms))
        return b
    return PositionConfig(min_extra_pieces=2,max_extra_pieces=8).sample(rng)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seed",type=int,default=12101)
    ap.add_argument("--train-sessions",type=int,default=4)
    ap.add_argument("--test-sessions",type=int,default=2)
    ap.add_argument("--positions-per-session",type=int,default=2)
    ap.add_argument("--calibration",default=".private/phase9/calibration.json")
    ap.add_argument("--plan",default="chess_sim/assets/wrist_scan_plan.json")
    ap.add_argument("--model-out",default=".private/phase11c/followup/type_probe_model_seed12101.json")
    ap.add_argument("--out",default=".private/phase11c/followup/type_pose_probe_seed12101.json")
    ap.add_argument("--self-occlusion",action=argparse.BooleanOptionalAction,default=True)
    ap.add_argument("--max-views-per-sample",type=int,default=3)
    ap.add_argument("--pose-cases-per-type",type=int,default=2)
    args=ap.parse_args()

    watch=WristCalibration.load(args.calibration)
    poses=load_plan(args.plan)
    self_model=SelfOcclusionModel(watch) if args.self_occlusion else None
    t0=time.time()

    def gen_split(n_sessions, seed_base, split_name):
        X=[]; y=[]; manifest=[]
        for si in range(n_sessions):
            sseed=seed_base*1000+si
            env=make_env(sseed); srng=random.Random(sseed)
            crowded = (si%2==0)
            try:
                env.reset(chess.Board(None), rng=srng)
                board_geom=env.board
                marker_layout=MarkerLayout.for_board(board_geom.field*1000.0, board_geom.border*1000.0, board_geom.marker_plane)
                scanner=WristScanner(watch, poses, None, square=board_geom.square, board_z=board_geom.top,
                                     marker_z=board_geom.table_top+board_geom.marker_above_table,
                                     marker_layout=marker_layout, method="markers", self_occlusion=args.self_occlusion)
                empty_frames=capture_scan_frames(env, poses)
                refs={f.pose:f.frame for f in empty_frames}
                reg=scanner.register(empty_frames)
                if reg is None:
                    manifest.append({"split":split_name,"session":si,"seed":sseed,"registered":False,"samples":0}); continue
                ref_patches={}
                for pose in poses:
                    cam=scanner.cameras(pose.joints)
                    ref_frame=scanner.calibration.rectify(refs[pose.name])
                    rp,_=board_patches(ref_frame,cam,reg,patch=16); ref_patches[pose.name]=rp
                sess_samples=0
                for _ in range(args.positions_per_session):
                    b=random_case_board(srng,crowded)
                    env.reset(b)
                    sq_types={int(sq):p.piece_type for sq,p in b.piece_map().items()}
                    if not sq_types: continue
                    use_poses=poses[:max(1,args.max_views_per_sample)]
                    for pose in use_poses:
                        park(env,pose.joints)
                        rgb=scanner.calibration.rectify(env.render(Camera.WRIST).copy())
                        cam=scanner.cameras(pose.joints)
                        patches,vf=board_patches(rgb,cam,reg,patch=16)
                        feats=square_features(patches,vf,view="wrist",reference=ref_patches.get(pose.name))
                        feats=normalize_per_frame(feats,np.zeros(64,dtype=int))
                        for sq,ptype in sq_types.items():
                            if vf[sq] < 0.45: continue
                            X.append(feats[sq].astype(np.float32))
                            y.append(TYPE_LIST.index(ptype))
                            sess_samples+=1
                manifest.append({"split":split_name,"session":si,"seed":sseed,"registered":True,"crowded":crowded,"samples":sess_samples})
            except Exception as e:
                manifest.append({"split":split_name,"session":si,"seed":sseed,"error":repr(e),"trace":traceback.format_exc()})
            finally:
                env.close()
        if not X:
            return np.zeros((0,FEATURE_COUNT),np.float32), np.zeros((0,),np.int64), manifest
        return np.stack(X), np.array(y,dtype=np.int64), manifest

    t_gen=time.time()
    Xtr,ytr,man_tr = gen_split(args.train_sessions, args.seed, "train")
    Xte,yte,man_te = gen_split(args.test_sessions, args.seed+50000, "test")
    gen_time=time.time()-t_gen

    dataset_summary={
        "train":{"sessions":args.train_sessions,"samples":int(len(ytr)),"class_counts":{TYPE_NAME[TYPE_LIST[int(k)]]:int(v) for k,v in sorted(Counter(int(v) for v in ytr.tolist()).items())},"manifest":man_tr},
        "test":{"sessions":args.test_sessions,"samples":int(len(yte)),"class_counts":{TYPE_NAME[TYPE_LIST[int(k)]]:int(v) for k,v in sorted(Counter(int(v) for v in yte.tolist()).items())},"manifest":man_te},
        "feature_count":FEATURE_COUNT,
    }

    blocker=None; train_info=None; model_sha=None; reload_sha=None; reload_ok=False
    type_metrics={}; e2e_metrics={}
    if len(ytr)<12 or len(set(ytr.tolist()))<2:
        blocker={"stage":"dataset","reason":"insufficient labeled training samples/classes","train_samples":int(len(ytr))}
    else:
        t_train=time.time()
        val=(Xte,yte) if len(yte)>=6 and len(set(yte.tolist()))>=2 else None
        model,train_info=train_type_mlp(Xtr,ytr,hidden=48,epochs=350,lr=0.01,seed=args.seed,val=val)
        train_time=time.time()-t_train
        model_sha=model.save(args.model_out)
        m2=TypeMLP.load(args.model_out)
        reload_sha=hashlib.sha256(Path(args.model_out).read_bytes()).hexdigest()
        probe_x = Xte[:8] if len(Xte)>=8 else (Xte if len(Xte) else Xtr[:8])
        p1=model.probs(probe_x); p2=m2.probs(probe_x)
        reload_ok=bool(np.allclose(p1,p2,atol=1e-6,rtol=1e-6))
        train_info["train_time_s"]=round(train_time,4)
        train_info["reload_prob_match"]=reload_ok
        if len(yte):
            pred,conf,prob=model.predict(Xte)
            acc=float((pred==yte).mean())
            per_type={}
            for i,t in enumerate(TYPE_LIST):
                mask=(yte==i)
                if mask.any(): per_type[TYPE_NAME[t]]={"n":int(mask.sum()),"acc":float((pred[mask]==yte[mask]).mean())}
            cm=np.zeros((N_TYPES,N_TYPES),dtype=int)
            for a,b in zip(yte,pred): cm[int(a),int(b)]+=1
            type_metrics={"n":int(len(yte)),"accuracy":acc,"per_type":per_type,"confusion_matrix":cm.tolist()}
        else:
            type_metrics={"n":0,"note":"no held-out samples"}

        t_e2e=time.time(); e2e_records=[]
        try:
            cases_per_type=max(1,int(args.pose_cases_per_type))
            for ti,ptype in enumerate(TYPE_LIST):
                tname=TYPE_NAME[ptype]
                for j in range(cases_per_type):
                    cseed=args.seed+90000+ti*1000+j
                    env=make_env(cseed); crng=random.Random(cseed)
                    try:
                        env.reset(chess.Board(None), rng=crng)
                        board_geom=env.board
                        marker_layout=MarkerLayout.for_board(board_geom.field*1000.0, board_geom.border*1000.0, board_geom.marker_plane)
                        scanner=WristScanner(watch, poses, None, square=board_geom.square, board_z=board_geom.top,
                                             marker_z=board_geom.table_top+board_geom.marker_above_table,
                                             marker_layout=marker_layout, method="markers", self_occlusion=args.self_occlusion)
                        empty_frames=capture_scan_frames(env, poses)
                        refs={f.pose:f.frame for f in empty_frames}
                        reg=scanner.register(empty_frames)
                        if reg is None:
                            e2e_records.append({"type":tname,"case":j,"seed":cseed,"error":"no_registration"}); continue
                        ref_patches={}
                        for pose in poses:
                            cam=scanner.cameras(pose.joints)
                            rf=scanner.calibration.rectify(refs[pose.name])
                            rp,_=board_patches(rf,cam,reg,patch=16); ref_patches[pose.name]=rp
                        sq=crng.choice(list(chess.SQUARES))
                        cx,cy=board_geom.square_center(sq); off=board_geom.square*0.12
                        px=cx+crng.uniform(-off,off); py=cy+crng.uniform(-off,off)
                        yaw_true=crng.uniform(0,2*math.pi)
                        piece=chess.Piece(ptype,chess.WHITE)
                        b=chess.Board(None); b.set_piece_at(sq,piece); env.reset(b)
                        slot=env.slot_at(sq)
                        if slot is None:
                            e2e_records.append({"type":tname,"case":j,"seed":cseed,"error":"no_slot"}); continue
                        env.set_piece_pose(slot,[px-slot.geometry.center[0],py-slot.geometry.center[1],board_geom.top-slot.geometry.bottom_offset+0.0015],[math.cos(yaw_true/2),0,0,math.sin(yaw_true/2)],zero_velocity=True)
                        mujoco.mj_forward(env.model, env.data)
                        truth_xy=np.array(env.piece_position(slot)[:2],dtype=float)
                        xmat=env.data.xmat[env._slot_body[slot.body]].reshape(3,3)
                        body_yaw=math.atan2(float(xmat[1,0]),float(xmat[0,0]))
                        truth_axis=body_yaw % math.pi
                        # multi-view type prediction (aggregate probs across scan poses)
                        sq_probs=[]
                        for pose in poses:
                            park(env,pose.joints)
                            rgb_t=scanner.calibration.rectify(env.render(Camera.WRIST).copy())
                            cam_t=scanner.cameras(pose.joints)
                            patches_t,vf_t=board_patches(rgb_t,cam_t,reg,patch=16)
                            if vf_t[sq] < 0.20:
                                continue
                            feats_t=square_features(patches_t,vf_t,view="wrist",reference=ref_patches.get(pose.name))
                            feats_t=normalize_per_frame(feats_t,np.zeros(64,dtype=int))
                            sq_probs.append(model.probs(feats_t[sq:sq+1])[0])
                        if sq_probs:
                            mean_prob=np.mean(np.stack(sq_probs,axis=0),axis=0)
                            pred_idx=int(np.argmax(mean_prob)); pred_conf=float(np.max(mean_prob))
                        else:
                            pred_idx, pred_conf, _ = model.predict(np.zeros((1,FEATURE_COUNT),dtype=np.float32))
                            pred_idx=int(pred_idx[0]); pred_conf=float(pred_conf[0])
                        pred_t=TYPE_LIST[pred_idx]; pred_name=TYPE_NAME[pred_t]
                        type_correct=(pred_t==ptype)
                        piece_set=env.config.appearance.piece_set
                        pred_piece=chess.Piece(pred_t,chess.WHITE)
                        mesh=Path(piece_obj_path(pred_piece,piece_set))
                        tex_path=piece_texture_path(pred_piece,piece_set)
                        texture=Path(tex_path) if tex_path and os.path.exists(tex_path) else None
                        tint=piece_tint(pred_piece,env.config.appearance.white_rgba,env.config.appearance.black_rgba,piece_set)
                        views=capture_views(env,scanner,poses,self_model)
                        t_fit=time.time()
                        est=fit_mesh_axis(mesh,views,(px,py),board_geom.top,texture_path=texture,tint=tint)
                        fit_dt=time.time()-t_fit
                        rec={"type":tname,"pred_type":pred_name,"type_correct":type_correct,"type_conf":pred_conf,"case":j,"seed":cseed,
                             "truth_axis_mod_pi_rad":float(truth_axis),"fit_runtime_s":float(fit_dt)}
                        if est is None:
                            rec.update({"pose_ok":False,"accepted":False,"reason":"no_estimate"})
                        else:
                            axis_err=float(axis_distance_degrees(math.degrees(est.axis_rad),math.degrees(truth_axis)))
                            xy_err_mm=(float(np.linalg.norm(np.array(est.world_xy)-truth_xy)*1000.0) if est.world_xy is not None else None)
                            rec.update({"pose_ok":True,"accepted":bool(est.acceptable()),"axis_err_deg_mod180":axis_err,
                                        "sigma_deg":float(math.degrees(est.sigma_rad)),"confidence":float(est.confidence),
                                        "supporting_views":int(est.supporting_views),"xy_err_mm":xy_err_mm})
                        e2e_records.append(rec)
                    except Exception as e:
                        e2e_records.append({"type":tname,"case":j,"seed":cseed,"error":repr(e),"trace":traceback.format_exc()})
                    finally:
                        env.close()
        except Exception as e:
            e2e_metrics={"error":repr(e),"trace":traceback.format_exc()}
        e2e_time=time.time()-t_e2e
        ok=[r for r in e2e_records if r.get("pose_ok")]
        acc=[r for r in ok if r.get("accepted")]
        knight=[r for r in e2e_records if r.get("type")=="knight" and "error" not in r]
        knight_acc=[r for r in knight if r.get("accepted")]
        def svals(rs,k):
            vals=[float(r[k]) for r in rs if r.get(k) is not None]
            if not vals: return None
            a=np.array(vals,dtype=float); a=a[np.isfinite(a)]
            if len(a)==0: return None
            return {"n":len(a),"mean":float(np.mean(a)),"median":float(np.median(a)),"max":float(np.max(a))}
        e2e_metrics={
            "time_s":round(e2e_time,4),"cases":len(e2e_records),
            "errors":len([r for r in e2e_records if "error" in r]),
            "type_correct":int(sum(1 for r in e2e_records if r.get("type_correct"))),
            "type_accuracy": (sum(1 for r in e2e_records if r.get("type_correct"))/len(e2e_records)) if e2e_records else None,
            "pose_ok":len(ok),"accepted":len(acc),
            "accepted_given_type_correct":int(sum(1 for r in acc if r.get("type_correct"))),
            "axis_err_deg_mod180_accepted":svals(acc,"axis_err_deg_mod180"),
            "fit_runtime_s":svals([r for r in e2e_records if "fit_runtime_s" in r],"fit_runtime_s"),
            "knight":{"cases":len(knight),"type_correct":int(sum(1 for r in knight if r.get("type_correct"))),"accepted":len(knight_acc),
                      "axis_err_deg_mod180_accepted":svals(knight_acc,"axis_err_deg_mod180"),
                      "xy_err_mm_accepted":svals(knight_acc,"xy_err_mm")},
            "records":e2e_records,
        }

    receipt={
        "schema":"robot-arm-phase11c-type-pose-probe/v1",
        "seed":args.seed,"self_occlusion":args.self_occlusion,
        "env":{"cwd":os.getcwd(),"piece_set":"printed","piece_assets":os.environ.get("ROBOT_PIECE_ASSETS"),
               "gripper_assets":os.environ.get("ROBOT_GRIPPER_ASSETS")},
        "params":{"train_sessions":args.train_sessions,"test_sessions":args.test_sessions,"positions_per_session":args.positions_per_session,
                  "max_views_per_sample":args.max_views_per_sample,"pose_cases_per_type":args.pose_cases_per_type},
        "dataset":dataset_summary,
        "generation_time_s":round(gen_time,4),
        "train":train_info,
        "model":{"path":args.model_out,"sha256_saved":model_sha,"sha256_reload":reload_sha,"reload_prob_match":reload_ok},
        "heldout_type_metrics":type_metrics,
        "end_to_end_type_pose":e2e_metrics,
        "blocker":blocker,
        "fallback":"If type-ID accuracy/acceptance is insufficient, fall back to hybrid type-known + B1 refinement with abstention (documented B1 baseline) or collect larger local labeled dataset and stronger local model under measured budgets.",
        "elapsed_s":round(time.time()-t0,3),
    }
    Path(args.out).parent.mkdir(parents=True,exist_ok=True)
    Path(args.out).write_text(json.dumps(receipt,indent=2,default=str)+"\n")
    print(json.dumps({"elapsed_s":receipt["elapsed_s"],"dataset_train":dataset_summary["train"]["samples"],"dataset_test":dataset_summary["test"]["samples"],
                      "blocker":blocker,"type_acc":type_metrics.get("accuracy") if isinstance(type_metrics,dict) else None,
                      "e2e_type_acc":e2e_metrics.get("type_accuracy") if isinstance(e2e_metrics,dict) else None,
                      "e2e_accepted":e2e_metrics.get("accepted") if isinstance(e2e_metrics,dict) else None,
                      "knight": (e2e_metrics.get("knight") if isinstance(e2e_metrics,dict) else None)}, indent=2, default=str))
    print("receipt:", args.out)

if __name__=="__main__":
    main()
