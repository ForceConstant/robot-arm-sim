"""Run a bounded local probe with sampled memory, swap and GPU thermal guards."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def snapshot():
    mem = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if value.strip().endswith('kB'):
            mem[key] = int(value.split()[0]) * 1024
    temperatures = []
    for folder in Path('/sys/class/hwmon').glob('hwmon*'):
        if (folder / 'name').read_text().strip() == 'amdgpu':
            temperatures.append(int((folder / 'temp1_input').read_text()) / 1000)
    if not temperatures:
        raise RuntimeError('AMD GPU temperature is unavailable; refusing unmonitored run')
    return {
        'cpu_available_bytes': mem['MemAvailable'],
        'swap_used_bytes': mem['SwapTotal'] - mem['SwapFree'],
        'gpu_edge_c': max(temperatures),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=300)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not 1 <= args.seconds <= 1200:
        parser.error('provide a command after -- and a time limit of 1..1200 seconds')
    args.out.mkdir(parents=True, exist_ok=False)
    baseline = snapshot()
    report = {'baseline': baseline, 'command': command, 'abort_reason': [], 'samples': []}
    proc = None
    started = time.monotonic()
    low_samples = 0
    try:
        if baseline['gpu_edge_c'] >= 85 or baseline['cpu_available_bytes'] < 768 * 2**20:
            raise RuntimeError('Insufficient baseline thermal or CPU-memory headroom')
        with (args.out / 'worker.log').open('w') as log:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            while True:
                sample = snapshot()
                sample['elapsed_s'] = time.monotonic() - started
                report['samples'].append(sample)
                low_samples = low_samples + 1 if sample['cpu_available_bytes'] < 768 * 2**20 else 0
                if low_samples >= 3:
                    report['abort_reason'].append('CPU available below 768 MiB for three samples')
                if sample['swap_used_bytes'] - baseline['swap_used_bytes'] > 512 * 2**20:
                    report['abort_reason'].append('Swap grew over 512 MiB')
                if sample['gpu_edge_c'] >= 85:
                    report['abort_reason'].append('GPU edge reached 85 C')
                if sample['elapsed_s'] > args.seconds:
                    report['abort_reason'].append('Time limit exceeded')
                if report['abort_reason'] or proc.poll() is not None:
                    break
                time.sleep(1)
    except Exception as error:
        report['abort_reason'].append(repr(error))
    finally:
        if proc is not None:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            report['exit_code'] = proc.wait()
        report['elapsed_s'] = time.monotonic() - started
        report['passed'] = report.get('exit_code') == 0 and not report['abort_reason']
        (args.out / 'guard.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: report[key] for key in ('passed', 'abort_reason', 'elapsed_s')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
