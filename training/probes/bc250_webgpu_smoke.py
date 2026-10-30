"""Synthetic autograd/AdamW checkpoint probe. No robot/task or large-model qualification."""
import os, sys, json, time, math, random, platform, traceback, argparse, hashlib
from pathlib import Path
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--width', type=int, default=1024)
parser.add_argument('--steps', type=int, default=20)
parser.add_argument('--out', type=Path)
args = parser.parse_args()
if not 8 <= args.width <= 2048 or not 1 <= args.steps <= 100:
    parser.error('width must be 8..2048 and steps 1..100')
root = Path.home() / '.local/share/robot-arm-bc250-investigation'
stage = root / 'training-webgpu'
sys.path.insert(0, str(stage / 'tinygrad-0dd490c19492e606aadde8dc25e10b9223e06dbf'))
os.environ.update(DEV='WEBGPU', WEBGPU_BACKEND='Vulkan', WEBGPU_PATH=str(stage / 'libwebgpu_dawn.so'), VK_DRIVER_FILES='/usr/share/vulkan/icd.d/radeon_icd.x86_64.json')
out = args.out or root / 'receipts' / f'webgpu-train-width{args.width}'
out.mkdir(parents=True, exist_ok=True)
r = {'scope': 'synthetic small-network training only; NOT MolmoAct2 or task training', 'python': sys.version, 'platform': platform.platform(), 'tinygrad_revision': '0dd490c19492e606aadde8dc25e10b9223e06dbf', 'device_requested': 'WEBGPU/Vulkan', 'steps': []}
started = time.monotonic()
try:
    if not (stage / 'tinygrad-0dd490c19492e606aadde8dc25e10b9223e06dbf').is_dir():
        raise RuntimeError('Pinned tinygrad source is missing; do not fall back to another installation')
    if hashlib.file_digest((stage / 'libwebgpu_dawn.so').open('rb'), 'sha256').hexdigest() != 'cf36091d266a32c9d5080f14662de44cece241987939713282ea0ff558db81c6':
        raise RuntimeError('Dawn library differs from the tested artifact')
    from tinygrad import Tensor, Device, Context, GlobalCounters
    from tinygrad.nn.optim import AdamW
    from tinygrad.nn.state import get_parameters, get_state_dict, load_state_dict, safe_save, safe_load
    from tinygrad.runtime.ops_webgpu import InstanceRequestAdapter, instance, webgpu, from_wgpu_str
    print('backend enums', webgpu.enum_WGPUBackendType, flush=True)
    adapter = InstanceRequestAdapter(instance, webgpu.WGPURequestAdapterOptions(backendType=webgpu.WGPUBackendType_Vulkan))
    info = webgpu.WGPUAdapterInfo()
    status = webgpu.wgpuAdapterGetInfo(adapter, info)
    r['adapter'] = {'name': from_wgpu_str(info.device), 'vendor': from_wgpu_str(info.vendor), 'architecture': from_wgpu_str(info.architecture), 'description': from_wgpu_str(info.description), 'backend': int(info.backendType), 'adapter_type': int(info.adapterType), 'vendor_id': int(info.vendorID), 'device_id': int(info.deviceID)}
    print('ADAPTER', r['adapter'], flush=True)
    if info.vendorID != 4098:
        raise RuntimeError('Not an AMD hardware adapter')
    webgpu.wgpuAdapterInfoFreeMembers(info)
    webgpu.wgpuAdapterRelease(adapter)

    class Net:

        def __init__(self):
            rng = random.Random(20261030)
            self.layers = []
            for fan_in, fan_out in [(6, args.width), (args.width, args.width), (args.width, 6)]:
                bound = 1 / math.sqrt(fan_in)
                self.layers.append({'weight': Tensor([[rng.uniform(-bound, bound) for _ in range(fan_out)] for _ in range(fan_in)], device='WEBGPU'), 'bias': Tensor([0.0] * fan_out, device='WEBGPU')})

        def __call__(self, x):
            for i, layer in enumerate(self.layers):
                x = x @ layer['weight'] + layer['bias']
                if i < len(self.layers) - 1:
                    x = x.tanh()
            return x
    net = Net()
    opt = AdamW(get_parameters(net), lr=0.001)
    r['parameters'] = sum((p.numel() for p in get_parameters(net)))
    xrows = [[math.sin(i * 6 + j) for j in range(6)] for i in range(32)]
    x = Tensor(xrows, device='WEBGPU').is_param_(False)
    y = Tensor([[0.5 * v for v in row] for row in xrows], device='WEBGPU').is_param_(False)

    def values(t):
        return t.flatten().tolist()

    def step(net, opt):
        with Context(TRAINING=1):
            opt.zero_grad()
            loss = (net(x) - y).square().mean()
            loss.backward()
            lv = loss.item()
            if not math.isfinite(lv):
                raise RuntimeError('Nonfinite loss')
            for p in get_parameters(net):
                if p.grad is None or not p.grad.isfinite().all().item():
                    raise RuntimeError('Missing/nonfinite gradients')
            opt.step()
            Device['WEBGPU'].synchronize()
            return lv
    for i in range(args.steps):
        begin = time.monotonic()
        loss = step(net, opt)
        r['steps'].append({'step': i + 1, 'loss': loss, 'seconds': time.monotonic() - begin})
        print('STEP', i + 1, loss, flush=True)
    if r['steps'][-1]['loss'] >= r['steps'][0]['loss']:
        raise RuntimeError('Loss did not decrease')
    expected = values(net(x))
    bundle = {'model': net, 'optimizer': opt}
    path = out / 'checkpoint.safetensors'
    safe_save(get_state_dict(bundle), str(path), metadata={'step': str(args.steps)})
    net2 = Net()
    opt2 = AdamW(get_parameters(net2), lr=0.001)
    load_state_dict({'model': net2, 'optimizer': opt2}, safe_load(str(path)), verbose=False)
    if values(net2(x)) != expected:
        raise RuntimeError('Reload changed predictions')
    r['checkpoint_reload_exact'] = True
    step(net, opt)
    step(net2, opt2)
    if values(net2(x)) != values(net(x)):
        raise RuntimeError('Resumed AdamW update differs from uninterrupted update')
    r.update(resumed_update_exact=True, initial_loss=r['steps'][0]['loss'], final_loss=r['steps'][-1]['loss'], checkpoint_bytes=path.stat().st_size, checkpoint_sha256=hashlib.file_digest(path.open('rb'), 'sha256').hexdigest(), framework_current_bytes=GlobalCounters.mem_used, status='completed')
except Exception as e:
    r.update(status='failed', error=repr(e), traceback=traceback.format_exc())
    print(r['traceback'], flush=True)
finally:
    r['elapsed_seconds'] = time.monotonic() - started
    (out / 'report.json').write_text(json.dumps(r, indent=2))
    print('FINAL', json.dumps(r), flush=True)
if r['status'] != 'completed':
    raise SystemExit(1)
