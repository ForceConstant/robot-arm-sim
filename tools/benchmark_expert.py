"""Score the scripted expert over randomized placements and piece sets.

    ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS="$PWD/.private/piece-assets-v5" \
    MUJOCO_GL=egl python tools/benchmark_expert.py \
        --moves 32 --captures 16 --seeds 271828 314159 161803 \
        --out .private/expert-baseline.json

This is the Priority 2 baseline: the expert is the control ceiling for every
learned policy, so its reliability is measured with the same success rule and
reported per task family, per piece type, per piece set and per seed. Failures
are data, not a crash: the process exits 0 when the run completes.

The expert plans against simulator state. Task metadata in the report names the
piece and layout for slicing only; it is never available to a policy under test.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from chess_sim.benchmark import EpisodeRecord, ExpertBenchmarkConfig, resolve_piece_sets, run_benchmark


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--moves", type=int, default=32, help="move episodes per seed per piece set")
    ap.add_argument("--captures", type=int, default=16, help="capture episodes per seed per piece set")
    ap.add_argument("--seeds", nargs="+", type=int, default=[271828, 314159, 161803],
                    help="independent seeds; each is reported separately")
    ap.add_argument("--piece-sets", nargs="+", default=None,
                    help="sets to measure; default is every verified set plus the printed pack")
    ap.add_argument("--extra-min", type=int, default=2, help="pieces besides the kings, minimum")
    ap.add_argument("--extra-max", type=int, default=8, help="pieces besides the kings, maximum")
    ap.add_argument("--board-shift", type=float, default=0.005, help="m per axis, the supported envelope")
    ap.add_argument("--arm-jitter", type=float, default=0.10, help="rad per joint, the supported envelope")
    ap.add_argument("--board-yaw-deg", type=float, default=0.0, help="optional wider envelope: board turned")
    ap.add_argument("--board-side", type=float, default=0.0, help="optional wider envelope: board off centre")
    ap.add_argument("--out", default=None, help="write the full JSON report here")
    ap.add_argument("--markdown", default=None, help="write the summary markdown here as well as stdout")
    args = ap.parse_args()

    config = ExpertBenchmarkConfig(
        moves=args.moves, captures=args.captures, seeds=tuple(args.seeds),
        piece_sets=tuple(args.piece_sets or ()),
        extra_pieces=(args.extra_min, args.extra_max),
        board_shift=args.board_shift, arm_joint_jitter=args.arm_jitter,
        board_yaw_deg=args.board_yaw_deg, board_side=args.board_side)
    sets = resolve_piece_sets(config)
    total = len(sets) * len(config.seeds) * (config.moves + config.captures)
    print(f"measuring {total} episodes: sets {sets}, seeds {list(config.seeds)}, "
          f"{config.moves} moves + {config.captures} captures each", flush=True)

    done = 0
    started = time.perf_counter()

    def progress(record: EpisodeRecord) -> None:
        nonlocal done
        done += 1
        mark = "OK" if record.success else f"FAIL {record.reason}"
        print(f"[{done}/{total}] {record.piece_set} seed={record.seed} {record.family} "
              f"{record.task} {record.piece_type} {mark} "
              f"{record.placement_error_mm:.1f} mm {record.seconds:.1f}s", flush=True)

    report = run_benchmark(config, on_episode=progress)
    text = report.markdown()
    print()
    print(text)
    print(f"run took {time.perf_counter() - started:.0f}s")

    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            f.write(report.to_json())
        print(f"wrote {args.out}")
    if args.markdown:
        os.makedirs(os.path.dirname(args.markdown) or ".", exist_ok=True)
        with open(args.markdown, "w") as f:
            f.write(text + "\n")
        print(f"wrote {args.markdown}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
