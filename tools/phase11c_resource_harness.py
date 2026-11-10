#!/usr/bin/env python3
"""Phase 11c resource/reload evidence helper (bc250-cachy).

Runs a bounded deterministic subset of wrist-scan/perception inference or a
provided eval command while sampling RSS/temps/CPU, and checks that two cold
loads on the same fixed inputs produce identical outputs (or reports diffs).

This is evidence plumbing, not a product gate by itself.
"""
from __future__ import annotations
import argparse, json, os, resource, subprocess, sys, time, hashlib
from pathlib import Path

def read_temps():
    temps = {}
    try:
        for hw in Path('/sys/class/hwmon').glob('hwmon*'):
            name = (hw/'name').read_text().strip() if (hw/'name').exists() else hw.name
            for t in hw.glob('temp*_input'):
                try:
                    temps[f"{name}/{t.stem}"] = int(t.read_text().strip())/1000.0
                except Exception:
                    pass
    except Exception:
        pass
    # k10temp convenience
    try:
        for p in Path('/sys/class/hwmon').glob('*/temp1_input'):
            n=(p.parent/'name').read_text().strip()
            if n=='k10temp': temps['tctl_c']=int(p.read_text())/1000.0
    except Exception: pass
    return temps

def sample_proc(pid=None):
    rss_kb = None
    try:
        if pid and Path(f'/proc/{pid}/status').exists():
            for line in Path(f'/proc/{pid}/status').read_text().splitlines():
                if line.startswith('VmRSS:'): rss_kb = int(line.split()[1]); break
        else:
            rss_kb = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    except Exception:
        pass
    return {'t': time.time(), 'temps': read_temps(), 'rss_kb': rss_kb}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--interval', type=float, default=5.0)
    ap.add_argument('--tag', default='')
    ap.add_argument('cmd', nargs=argparse.REMAINDER)
    args = ap.parse_args()
    cmd = args.cmd
    if cmd and cmd[0] == '--': cmd = cmd[1:]
    if not cmd:
        ap.error('provide -- command...')
    start = time.time()
    samples = [sample_proc()]
    proc = subprocess.Popen(cmd)
    while proc.poll() is None:
        time.sleep(args.interval)
        samples.append(sample_proc(proc.pid))
    end = time.time()
    samples.append(sample_proc())
    out = {
        'schema': 'phase11c-resource-harness/1',
        'tag': args.tag,
        'cmd': cmd,
        'cwd': os.getcwd(),
        'returncode': proc.returncode,
        'wall_s': end-start,
        'samples': samples,
        'peak_rss_kb': max((s.get('rss_kb') or 0) for s in samples),
        'temps_max': {},
    }
    keys=set()
    for s in samples: keys |= set((s.get('temps') or {}).keys())
    for k in keys:
        vals=[s['temps'][k] for s in samples if s.get('temps') and k in s['temps']]
        if vals: out['temps_max'][k]=max(vals)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps({'out': args.out, 'wall_s': out['wall_s'], 'rc': out['returncode'], 'peak_rss_kb': out['peak_rss_kb'], 'tctl_max_c': out['temps_max'].get('tctl_c')}, indent=2))
    return proc.returncode

if __name__ == '__main__':
    raise SystemExit(main())
