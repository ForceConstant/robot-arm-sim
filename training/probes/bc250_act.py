"""Full pinned ACT training/reload/30-minute shadow-runtime qualification.

Offline ONLY. Reads simulator images/feedback over LAN; sends no commands.
User-space CPU fallback: does not install/change AMD or gaming drivers.
"""
import argparse
import gc
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import resource
import time
import traceback
import urllib.request

from act_runtime import load_act, config, loss, REVISION, SOURCE_SHA256
from action_chunk_gate import verdict, faults, motion_verdict


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('train','reload','soak'), required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--artifact', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=20)
    parser.add_argument('--seconds', type=int, default=1800)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--simulator', default='http://192.168.1.138:8098')
    args = parser.parse_args()
    if not 1 <= args.steps <= 100 or not 1 <= args.seconds <= 3600 or not 1 <= args.threads <= 8:
        parser.error('invalid bounded workload')
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = dict(scope='full ACT platform/runtime qualification; NOT trained-controller acceptance',
                  mode=args.mode, lerobot_revision=REVISION, source_sha256=SOURCE_SHA256,
                  python=platform.python_version(), platform=platform.platform(), device='cpu', precision='float32',
                  threads=args.threads, steps=[], no_robot_commands=True)
    try:
        import numpy as np
        import torch
        import torchvision
        from PIL import Image
        torch.set_num_threads(args.threads)
        torch.set_num_interop_threads(1)
        torch.use_deterministic_algorithms(True)
        torch.manual_seed(20261103)
        ACT = load_act(args.source)
        cfg = config()
        model = ACT(cfg)
        report.update(torch=torch.__version__, torchvision=torchvision.__version__,
                      parameters=sum(p.numel() for p in model.parameters()),
                      architecture={k:v for k,v in vars(cfg).items() if isinstance(v,(str,int,float,bool,type(None)))},
                      image_shape=[3,224,224], goal='source/destination xyz metres in calibrated board frame; z0 surface',
                      cuda_available=torch.cuda.is_available(), hip=torch.version.hip)
        data = np.load(args.dataset, allow_pickle=False)
        report['dataset_sha256'] = sha(args.dataset)
        audit_manifest = json.loads((args.dataset.parent/'manifest.json').read_text())
        if audit_manifest.get('dataset_sha256') != report['dataset_sha256'] or not audit_manifest.get('audit',{}).get('passed'):
            raise ValueError('missing/failed audited dataset provenance')
        report['audit'] = audit_manifest['audit']
        train_idx = np.flatnonzero(data['split']=='train')
        test_idx = np.flatnonzero(data['split']=='test')
        if not len(train_idx) or not len(test_idx) or set(data['episode'][train_idx]) & set(data['episode'][test_idx]):
            raise ValueError('missing/leaking episode split')
        if data['image'].dtype != np.uint8 or data['image'].shape[1:] != (224,224,3):
            raise ValueError('image contract differs')
        for key in ('state','action','goal'):
            if not np.isfinite(data[key]).all():
                raise ValueError('nonfinite dataset')
        report.update(train_samples=len(train_idx), test_samples=len(test_idx))
        # Train-only normalization. Goals remain metres, never FEN/object truth.
        proc = dict(state_mean=data['state'][train_idx].mean(0).tolist(),
                    state_std=np.maximum(data['state'][train_idx].std(0),.01).tolist(),
                    action_mean=data['action'][train_idx][~data['pad'][train_idx]].mean(0).tolist(),
                    action_std=np.maximum(data['action'][train_idx][~data['pad'][train_idx]].std(0),.01).tolist(),
                    image_mean=[.485,.456,.406], image_std=[.229,.224,.225], action_hz=10,
                    chunk_size=30, replan_period_s=1.5, deadline_s=1.0, chunk_duration_s=3.0,
                    activation='offline_shadow_only_no_commands')
        if args.artifact:
            artifact = torch.load(args.artifact, map_location='cpu', weights_only=True)
            if artifact['dataset_sha256'] != report['dataset_sha256'] or artifact['source_sha256'] != SOURCE_SHA256:
                raise ValueError('artifact provenance differs')
            model.load_state_dict(artifact['model'])
            proc = artifact['processor']
            del artifact
            report['artifact_sha256'] = sha(args.artifact)
        def image_tensor(images):
            array = np.asarray(images, dtype=np.float32)/255
            tensor = torch.from_numpy(array).permute(0,3,1,2)
            return (tensor-torch.tensor(proc['image_mean'])[None,:,None,None])/torch.tensor(proc['image_std'])[None,:,None,None]
        def batch(indices):
            return {'observation.images':[image_tensor(data['image'][indices])],
                    'observation.state':(torch.from_numpy(data['state'][indices])-torch.tensor(proc['state_mean']))/torch.tensor(proc['state_std']),
                    'observation.environment_state':torch.from_numpy(data['goal'][indices]),
                    'action':(torch.from_numpy(data['action'][indices])-torch.tensor(proc['action_mean']))/torch.tensor(proc['action_std']),
                    'action_is_pad':torch.from_numpy(data['pad'][indices])}
        def infer(b):
            model.eval()
            with torch.inference_mode():
                raw = model({k:v for k,v in b.items() if k not in ('action','action_is_pad')})[0]
                result = raw*torch.tensor(proc['action_std']) + torch.tensor(proc['action_mean'])
                if not torch.isfinite(result).all():
                    raise RuntimeError('nonfinite inference')
                return result
        def step(opt, b):
            begin = time.monotonic()
            model.train()
            opt.zero_grad(set_to_none=True)
            value = loss(model,b)
            value.backward()
            if not torch.isfinite(value) or any(p.grad is None or not torch.isfinite(p.grad).all() for p in model.parameters() if p.requires_grad):
                raise RuntimeError('missing/nonfinite gradients or loss')
            opt.step()
            return dict(loss=float(value.detach()),seconds=time.monotonic()-begin,batch=len(b['observation.state']))
        def evaluate():
            predictions = torch.cat([infer(batch([i])) for i in test_idx]).numpy()
            valid = ~data['pad'][test_idx]
            error = predictions-data['action'][test_idx]
            hold = data['state'][test_idx,None,:]-data['action'][test_idx]
            return predictions, dict(mse_rad2=float(np.mean(error[valid]**2)),
                                     hold_position_mse_rad2=float(np.mean(hold[valid]**2)),
                                     scope='untouched historical episode; no manipulation rollouts/quality acceptance')
        if args.mode == 'train':
            opt = torch.optim.AdamW(model.parameters(),lr=1e-5,weight_decay=1e-4,foreach=False)
            # Full initialized graph, b1 and b2; target values synthetic ONLY for this subgate.
            report['initialized_graph'] = []
            for size in (1,2):
                b = batch(train_idx[:size])
                b['action'] = torch.randn_like(b['action'])
                report['initialized_graph'].append(step(opt,b))
            del opt,model
            gc.collect()
            torch.manual_seed(20261103)
            model = ACT(cfg)
            opt = torch.optim.AdamW(model.parameters(),lr=1e-5,weight_decay=1e-4,foreach=False)
            for i in range(args.steps):
                indices = train_idx[(np.arange(2)+2*i)%len(train_idx)]
                row = step(opt,batch(indices))
                report['steps'].append(row)
                print('STEP',i+1,json.dumps(row),flush=True)
            reference, report['heldout'] = evaluate()
            np.save(args.out/'predictions.npy',reference)
            payload = dict(model=model.state_dict(), processor=proc, source_sha256=SOURCE_SHA256,
                           dataset_sha256=report['dataset_sha256'])
            torch.save(payload,args.out/'inference.pt')
            del payload
            # Checkpoint includes optimizer + RNG: stochastic VAE and dropout resume exactly.
            torch.save(dict(model=model.state_dict(),optimizer=opt.state_dict(),rng=torch.get_rng_state(),step=args.steps),args.out/'checkpoint.pt')
            resume_batch = batch(train_idx[:2])
            checkpoint_rng = torch.get_rng_state().clone()
            step(opt,resume_batch)
            torch.save(dict(model=model.state_dict(),optimizer=opt.state_dict()),args.out/'uninterrupted.pt')
            del opt,model
            gc.collect()
            model = ACT(cfg)
            opt = torch.optim.AdamW(model.parameters(),lr=1e-5,weight_decay=1e-4,foreach=False)
            checkpoint = torch.load(args.out/'checkpoint.pt',map_location='cpu',weights_only=True)
            model.load_state_dict(checkpoint['model'])
            opt.load_state_dict(checkpoint['optimizer'])
            torch.set_rng_state(checkpoint['rng'])
            if not torch.equal(checkpoint_rng,checkpoint['rng']):
                raise RuntimeError('RNG differs')
            # eval forward consumes no RNG: compare before taking resumed update.
            reloaded,_ = evaluate()
            if not np.array_equal(reference,reloaded):
                raise RuntimeError('checkpoint prediction reload differs')
            report['checkpoint_reload_exact'] = True
            del checkpoint
            step(opt,resume_batch)
            expected = torch.load(args.out/'uninterrupted.pt',map_location='cpu',weights_only=True)
            def exact(a,b):
                if isinstance(a,torch.Tensor): return torch.equal(a,b)
                if isinstance(a,dict): return a.keys()==b.keys() and all(exact(a[k],b[k]) for k in a)
                if isinstance(a,(list,tuple)): return len(a)==len(b) and all(exact(x,y) for x,y in zip(a,b))
                return a==b
            if not exact(expected,dict(model=model.state_dict(),optimizer=opt.state_dict())):
                raise RuntimeError('resumed model/optimizer differs')
            report.update(resumed_update_exact=True,artifact_sha256=sha(args.out/'inference.pt'),
                          artifact_bytes=(args.out/'inference.pt').stat().st_size,
                          checkpoint_sha256=sha(args.out/'checkpoint.pt'))
            (args.out/'processor.json').write_text(json.dumps(proc,indent=2)+'\n')
        elif args.mode == 'reload':
            predictions, report['heldout'] = evaluate()
            expected = np.load(args.artifact.parent/'predictions.npy')
            if not np.array_equal(predictions,expected):
                raise RuntimeError('independent inference export reload differs')
            report['independent_reload_exact'] = True
            np.save(args.out/'predictions.npy',predictions)
        else:
            if args.artifact is None:
                raise ValueError('soak requires inference-only artifact')
            report.update(schedule=proc,fault_injection=faults(), cycles=[],
                          transport_scope='live simulator LAN cached JPEG/status retrieval; NOT Pi or physical capture synchronization',
                          command_boundary='hold/replan: source has no verified capture timestamp; GET-to-decode is a runtime timing gate ONLY')
            warm = time.monotonic()
            for i in range(5): infer(batch([test_idx[i]]))
            report['warmup_s'] = time.monotonic()-warm
            soak_started = time.monotonic()
            next_start = soak_started
            cycle = 0
            with (args.out/'cycles.jsonl').open('w') as stream:
                while time.monotonic()-soak_started < args.seconds:
                    time.sleep(max(0,next_start-time.monotonic()))
                    begin = time.monotonic()
                    with urllib.request.urlopen(args.simulator+'/api/status',timeout=1) as response:
                        status = json.load(response)
                    with urllib.request.urlopen(args.simulator+'/api/frame/white_wrist.jpg',timeout=1) as response:
                        jpeg = response.read()
                    fetched = time.monotonic()
                    joints = status['joints']['white']
                    if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
                        raise RuntimeError('invalid joint feedback')
                    frame = np.asarray(Image.open(io.BytesIO(jpeg)).convert('RGB').resize((224,224),Image.Resampling.BOX))
                    b = {'observation.images':[image_tensor(frame[None])],
                         'observation.state':(torch.tensor([joints])-torch.tensor(proc['state_mean']))/torch.tensor(proc['state_std']),
                         'observation.environment_state':torch.from_numpy(data['goal'][test_idx[:1]])}
                    actions = infer(b)[0].tolist()
                    done = time.monotonic()
                    outcome = verdict(actions,request_id=cycle,expected_id=cycle,captured_at=begin,now=done,deadline=proc['deadline_s'])
                    row = dict(index=cycle,elapsed_s=done-soak_started,transport_s=fetched-begin,
                               preprocess_infer_decode_s=done-fetched,total_s=done-begin,
                               result=outcome,sim_version=status['version'],frame_id=status['frame_id'],
                               motion_verdict=motion_verdict(actions,request_id=cycle,expected_id=cycle,captured_at=None,now=done,capture_verified=False),
                               frame_bytes=len(jpeg), max_raw_joint_delta_rad=max(abs(v-j) for a in actions for v,j in zip(a,joints)))
                    stream.write(json.dumps(row)+'\n'); stream.flush()
                    report['cycles'].append(row)
                    if outcome != 'timely_finite_chunk':
                        raise RuntimeError('action replenishment deadline missed; offline hold/replan')
                    cycle += 1
                    next_start = begin+proc['replan_period_s']
                    if cycle % 40 == 0:
                        print('SOAK',cycle,round(done-soak_started,1),flush=True)
            totals = sorted(r['total_s'] for r in report['cycles'])
            report.update(soak_duration_s=time.monotonic()-soak_started,cycles_count=len(totals),
                          deadline_misses=0,latency_p50_s=totals[math.ceil(len(totals)*.5)-1],
                          latency_p95_s=totals[math.ceil(len(totals)*.95)-1],latency_max_s=max(totals),
                          unique_frame_ids=len({r['frame_id'] for r in report['cycles']}),
                          unverified_capture_holds=sum(r['motion_verdict']=='hold_replan:unverified_capture_age' for r in report['cycles']))
        report['status'] = 'completed'
    except Exception as error:
        report.update(status='failed',error=repr(error),traceback=traceback.format_exc())
        print(report['traceback'],flush=True)
    finally:
        report.update(elapsed_s=time.monotonic()-started,peak_process_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
        (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print('RESULT',report['status'],args.out/'report.json',flush=True)
    return 0 if report['status']=='completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
