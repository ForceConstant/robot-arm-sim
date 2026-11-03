"""Pinned pretrained SmolVLA action-graph CPU compatibility/latency probe.

No training or commands. This tests an actual VLA artifact, not a VLM caption.
The historical failed chess training recipe is not retried by this runtime test.
"""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import sys
import time
import traceback
import types


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    report=dict(artifact='lerobot/smolvla_base',revision='5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6',
                device='cpu',precision='float32',no_robot_commands=True,steps=[])
    try:
        import numpy as np
        import torch
        from PIL import Image
        from safetensors.torch import load_file
        from transformers import AutoTokenizer
        torch.set_num_threads(2)
        torch.set_num_interop_threads(1)
        # Avoid importing the aggregate policy factory (which loads unrelated
        # robot/env integrations). All actual model submodules are unmodified.
        import lerobot
        namespace=types.ModuleType('lerobot.policies')
        namespace.__path__=[str(args.root/'lerobot-2774d9bddcbbda50e697e162e89e7eaada8d7105/src/lerobot/policies')]
        sys.modules['lerobot.policies']=namespace
        from lerobot.policies.smolvla.modeling_smolvla import VLAFlowMatching
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.common.vla_utils import pad_vector
        report['steps'].append('model_import')
        fields=json.loads((args.root/'smolvla-artifact/config.json').read_text())
        # Use current typed configuration with every architecture field from
        # the artifact; omit only serialization fields handled by the wrapper.
        import dataclasses
        valid={f.name for f in dataclasses.fields(SmolVLAConfig)}
        fields={k:v for k,v in fields.items() if k in valid and k not in ('input_features','output_features','normalization_mapping')}
        fields.update(device='cpu',load_vlm_weights=False,push_to_hub=False,
                      vlm_model_name=str(args.root/'smolvlm-base-config'))
        cfg=SmolVLAConfig(**fields)
        model=VLAFlowMatching(cfg)
        weights_path=args.root/'smolvla-artifact/model.safetensors'
        with weights_path.open('rb') as stream:
            report['artifact_sha256']=hashlib.file_digest(stream,'sha256').hexdigest()
        weights=load_file(str(weights_path),device='cpu')
        model.load_state_dict({k.removeprefix('model.'):v for k,v in weights.items()},strict=True)
        del weights
        model.float().eval()
        report.update(parameters=sum(p.numel() for p in model.parameters()),artifact_bytes=weights_path.stat().st_size,
                      torch=torch.__version__,image_shape=[1,3,512,512],chunk_size=cfg.chunk_size,
                      diffusion_steps=cfg.num_steps,base_revision=(args.root/'smolvlm-base-revision.txt').read_text().strip())
        report['steps'].append('strict_full_artifact_reload')
        data=np.load(args.root/'dataset.npz',allow_pickle=False)
        index=int(np.flatnonzero(data['split']=='test')[0])
        image=torch.from_numpy(np.asarray(Image.fromarray(data['image'][index]).resize((512,512)),dtype=np.float32)/127.5-1).permute(2,0,1)[None]
        state=pad_vector(torch.from_numpy(data['state'][index:index+1]),cfg.max_state_dim)
        tokenizer=AutoTokenizer.from_pretrained(str(args.root/'smolvlm-base-config'))
        text=tokenizer('pick up the piece on e2 and place it on e4',return_tensors='pt',padding='max_length',max_length=48,truncation=True)
        # Fixed noise allows exact repeat comparison; no action rollout occurs.
        torch.manual_seed(20261103)
        noise=torch.randn(1,cfg.chunk_size,cfg.max_action_dim)
        predictions=[]
        for i in range(4):
            begin=time.monotonic()
            with torch.inference_mode():
                actions=model.sample_actions([image],[torch.ones(1,dtype=torch.bool)],text['input_ids'],text['attention_mask'].bool(),state,noise=noise.clone())
            elapsed=time.monotonic()-begin
            if not torch.isfinite(actions).all(): raise RuntimeError('nonfinite action graph')
            predictions.append(actions[:,:,:6].clone())
            report.setdefault('latencies_s',[]).append(elapsed)
            print('ACTION_FORWARD',i,elapsed,flush=True)
        report.update(exact_repeat=all(torch.equal(predictions[0],p) for p in predictions[1:]),
                      warm_p50_s=sorted(report['latencies_s'][1:])[1],
                      deadline_s=1.0,status='measured',
                      deadline_pass=all(v<=1.0 for v in report['latencies_s'][1:]),
                      quality='not evaluated; upstream generic artifact not qualified chess controller')
        np.save(args.out/'actions.npy',torch.stack(predictions).numpy())
    except Exception as error:
        report.update(status='rejected',error=repr(error),traceback=traceback.format_exc())
        print(report['traceback'],flush=True)
    report.update(elapsed_s=time.monotonic()-started,peak_process_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)
    (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return 1 if report['status']=='rejected' else 0


if __name__=='__main__':
    raise SystemExit(main())
