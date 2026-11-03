"""Try the complete pinned ACT through Tinygrad's experimental torch bridge.

No CPU fallback is allowed. A failed bridge/operator/parity gate is a rejection,
not permission to claim the earlier MLP qualified this network.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

from act_runtime import load_act, config, loss


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    started = time.monotonic()
    report = {'route':'Tinygrad/WebGPU full ACT torch bridge','stages':[], 'fallback_allowed':False}
    try:
        stage = Path.home()/'.local/share/robot-arm-bc250-investigation/training-webgpu'
        if hashlib.sha256((stage/'libwebgpu_dawn.so').read_bytes()).hexdigest() != 'cf36091d266a32c9d5080f14662de44cece241987939713282ea0ff558db81c6':
            raise ValueError('Dawn identity differs')
        sys.path.insert(0,str(stage/'tinygrad-0dd490c19492e606aadde8dc25e10b9223e06dbf'))
        os.environ.update(DEV='WEBGPU',WEBGPU_BACKEND='Vulkan',WEBGPU_PATH=str(stage/'libwebgpu_dawn.so'),
                          VK_DRIVER_FILES='/usr/share/vulkan/icd.d/radeon_icd.x86_64.json',MAX_JOBS='1')
        import torch
        torch.set_num_threads(2)
        report['torch'] = torch.__version__
        import extra.torch_backend.backend
        report['stages'].append('bridge_import')
        torch.manual_seed(20261103)
        ACT = load_act(args.source)
        model = ACT(config())
        b = {'observation.images':[torch.randn(1,3,224,224)],'observation.state':torch.randn(1,6),
             'observation.environment_state':torch.randn(1,6),'action':torch.randn(1,30,6),
             'action_is_pad':torch.zeros(1,30,dtype=torch.bool)}
        model.eval()
        with torch.no_grad(): reference = model(b)[0]
        model = model.to('tiny')
        b = {k:[v[0].to('tiny')] if isinstance(v,list) else v.to('tiny') for k,v in b.items()}
        report['stages'].append('full_model_transfer')
        with torch.no_grad(): result = model(b)[0].cpu()
        report['max_inference_error'] = float((reference-result).abs().max())
        if not torch.allclose(reference,result,atol=1e-4,rtol=1e-4):
            raise RuntimeError('CPU/WebGPU inference parity failed')
        report['stages'].append('forward_parity')
        model.train()
        opt = torch.optim.AdamW(model.parameters(),lr=1e-5,foreach=False)
        opt.zero_grad(); value=loss(model,b); value.backward()
        if any(p.grad is None or not torch.isfinite(p.grad.cpu()).all() for p in model.parameters() if p.requires_grad):
            raise RuntimeError('missing/nonfinite backward')
        opt.step()
        report['stages'].append('backward_optimizer')
        report['status'] = 'partial_requires_checkpoint_and_task_pilot'
    except Exception as error:
        report.update(status='rejected',error=repr(error),traceback=traceback.format_exc())
        print(report['traceback'],flush=True)
    report['elapsed_s'] = time.monotonic()-started
    (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return 0 if report['status']!='rejected' else 1


if __name__=='__main__':
    raise SystemExit(main())
