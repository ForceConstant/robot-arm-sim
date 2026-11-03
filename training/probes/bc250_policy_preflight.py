"""Nonallocating large-action-artifact resource and ROCm-route preflight."""
import argparse
import json
from pathlib import Path
import platform
import urllib.request

REPO='allenai/MolmoAct2-SO100_101'
REV='152569fe57914d97be91055800035f54e250d009'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    with urllib.request.urlopen(f'https://huggingface.co/api/models/{REPO}/revision/{REV}',timeout=30) as response:
        model=json.load(response)
    counts=model['safetensors']['parameters']
    width={'F32':4,'F16':2,'BF16':2,'F64':8,'I64':8,'I32':4,'U8':1,'I8':1}
    if set(counts)-width.keys(): raise ValueError('unknown artifact dtype')
    size=sum(counts[k]*width[k] for k in counts)
    mem={l.split(':')[0]:int(l.split()[1])*1024 for l in Path('/proc/meminfo').read_text().splitlines() if l.endswith('kB')}
    reserve=768*2**20
    report=dict(model=REPO,revision=REV,metadata_sha=model['sha'],parameters=model['safetensors']['total'],
                dtypes=counts,uncompressed_weights_bytes=size,
                fp16_weights_lower_bound_bytes=model['safetensors']['total']*2,
                cpu_total_bytes=mem['MemTotal'],cpu_available_bytes=mem['MemAvailable'],reserve_bytes=reserve,
                cpu_raw_artifact_fits=size+reserve<=mem['MemAvailable'],
                cpu_fp16_weights_fit_even_total=model['safetensors']['total']*2+reserve<=mem['MemTotal'],
                kernel=platform.release(),hip_libraries=[str(p) for p in Path('/usr/lib64').glob('libamdhip*')],
                bc250_module_parameters={p.name:p.read_text().strip() for p in Path('/sys/module/amdgpu/parameters').glob('bc250*')},
                status='rejected_on_untouched_cpu_runtime',
                scope='nonallocating exact-artifact preflight; NOT a model forward or a hardware-wide impossibility claim',
                reason='raw F32 and hypothetical F16 weights exceed CPU memory; no installed patched gfx1013 HIP stack; no VLA GGUF runtime validated',
                host_change_policy='ROCm investigation only; kernel/firmware/driver changes require a separate concrete user-approved change set')
    (args.out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__': main()
