#!/usr/bin/env python3
"""Phase 2, step 1 — diagnose the 2026-11-23 silent non-execution.

The brief names the prime suspect: `if len(joints) == 6: measured = ...` in
`live/app.py::_stream_to` freezes `measured` at `start` whenever `/state` hands
back an empty or partial joint dict, and the loop then commands `start + 0.7deg`
forever. Three other candidates must be killed or confirmed:

  C1  empty / partial `state["joints"]`  -> `measured` frozen at `start`
  C2  stale readback (6 good joints, but the same values) -> same shape as C1
  C3  `/state` echoing the commanded goal instead of the measured position
  C4  the 10.5 deg/s step budget starving a big move inside the 30 s deadline
  C5  seq/TTL rejection (a rejected command is silently dropped)

This tool runs the REAL `Live._stream_to` — imported, not copied — against a
fake bridge with those behaviours injectable, and logs every tick:
command, measured, and the bridge's ack. That is the evidence the brief asks
for: a named cause, not a guess.

    robot-arm-sim python tools/phase2_diagnose_stream.py --out reports/phase2_diagnosis.json
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "live"))

NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex",
         "wrist_flex", "wrist_roll", "gripper")

# The relocation from the 2026-11-23 report: (48,27,40) -> (0,0,70) in board mm.
# In joint space that is a large pan+lift change; the exact numbers matter less
# than the size, so use the postures Phase 0 measured on the real arm.
START_DEG = [20.57, -19.03, 19.12, 93.58, 87.65, 43.88]
TARGET_DEG = [-47.12, -28.00, 24.13, 94.90, 87.65, 43.88]


class Mechanism:
    """A servo that integrates toward its goal at a real-ish rate."""

    def __init__(self, start, *, rate_deg_s=35.0, mode="normal",
                 freeze_after=None) -> None:
        self.joints = [float(v) for v in start]
        self.goal = list(self.joints)
        self.rate = float(rate_deg_s)
        self.mode = mode
        self.freeze_after = freeze_after
        self.armed = True
        self.last_seq = -1
        self.accepted = 0
        self.rejected = 0
        self.clamped = []
        self._frozen = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._tick, daemon=True)
        self._thread.start()

    def _tick(self) -> None:
        step = self.rate * 0.033      # bridge loop ~30 Hz
        while not self._stop.is_set():
            with self._lock:
                here = list(self.joints)
                goal = list(self.goal)
            new = []
            for h, g in zip(here, goal):
                d = g - h
                new.append(h + max(-step, min(step, d)))
            with self._lock:
                self.joints = new
            time.sleep(0.033)

    def submit(self, body) -> dict:
        with self._lock:
            if body["seq"] <= self.last_seq:
                self.rejected += 1
                return {"http_error": 409, "detail": f"seq {body['seq']} not newer "
                                                     f"than {self.last_seq}"}
            self.last_seq = body["seq"]
            wanted = [float(v) for v in body["joints"]]
            self.accepted += 1
            if self.mode == "reject_every_other":
                # Every other command is refused for a non-seq reason.
                if self.accepted % 2 == 0:
                    self.rejected += 1
                    return {"http_error": 409, "detail": "rejected (control)"}
            self.goal = wanted
            return {"accepted": True, "seq": body["seq"]}

    def state(self) -> dict:
        with self._lock:
            if self.mode == "empty_joints":
                joints_out = {}
            elif self.mode == "zeros_joints":
                # What the REAL Pi bridge emits when it has never read the
                # follower: `state()` does `joints.get(name, 0.0)` for all six,
                # so `len(joints) == 6` PASSES and the pose is all zeros.
                joints_out = {n: 0.0 for n in NAMES}
            elif self.mode == "partial_joints":
                joints_out = dict(list(self._pairs(self.joints).items())[:3])
            elif self.mode == "stale":
                if self._frozen is None:
                    self._frozen = list(self.joints)
                joints_out = dict(self._pairs(self._frozen))
            elif self.mode == "echo":
                joints_out = dict(self._pairs(self.goal))
            else:
                joints_out = dict(self._pairs(self.joints))
            real = [round(v, 3) for v in self.joints]
        return {"protocol": 1, "mode": "REMOTE", "desired": "REMOTE",
                "armed": self.armed, "fault": None, "error": None,
                "joints": joints_out,
                "joints_age_s": 0.05, "joints_source": "fake",
                "last_seq": self.last_seq, "commands_applied": self.accepted,
                "commands_rejected": self.rejected,
                "last_clamped": [], "lelab_active": False, "follower_open": True,
                "limits": {"shoulder_pan": [-120, 130], "shoulder_lift": [-115, 115],
                           "elbow_flex": [-100, 100], "wrist_flex": [-110, 105],
                           "wrist_roll": [-90, 65], "gripper": [0, 100]},
                "uptime_s": 1.0, "_real_joints": real}

    @staticmethod
    def _pairs(values):
        return {n: round(float(v), 3) for n, v in zip(NAMES, values)}

    # the truth, independent of whatever /state pretends
    def true_joints(self):
        with self._lock:
            return [round(v, 3) for v in self.joints]


def serve(mech: Mechanism) -> tuple[ThreadingHTTPServer, str]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):        # silence
            pass

        def _send(self, payload: dict, code: int = 200) -> None:
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/state":
                payload = mech.state()
                payload.pop("_real_joints", None)
                return self._send(payload)
            if self.path == "/health":
                return self._send({"ok": True})
            return self._send({"error": "not found"}, 404)

        def do_POST(self):
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/command":
                result = mech.submit(body)
                if result.get("http_error"):
                    return self._send({"detail": result["detail"]}, result["http_error"])
                return self._send(result)
            if self.path == "/mode":
                return self._send({"mode": body.get("mode", "REMOTE")})
            if self.path == "/arm":
                return self._send({"armed": True})
            return self._send({"error": "not found"}, 404)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_port}"


class LoggingBridge:
    """Wraps the real HttpBridge and records every command ack + state read."""

    def __init__(self, inner, tick_log: list) -> None:
        self.inner = inner
        self.log = tick_log

    def state(self) -> dict:
        return self.inner.state()

    def command(self, joints, **kwargs) -> dict:
        started = time.time()
        ack = self.inner.command(joints, **kwargs)
        entry = {"t": round(started, 3), "sent": [round(float(v), 3) for v in joints],
                 "seq": kwargs.get("seq"), "ack": ack}
        self.log.append(entry)
        return ack

    def __getattr__(self, name):
        return getattr(self.inner, name)


def make_live(bridge_url: str, tick_log: list):
    """A bare `Live` with only what `_stream_to` touches."""
    import app as app_module
    from bridge_client import HttpBridge

    live = app_module.Live.__new__(app_module.Live)
    live.command_seq = int(time.time() * 1000)
    live.bridge = LoggingBridge(HttpBridge(bridge_url, timeout=6.0,
                                           token="diag-token"), tick_log)
    live.config = {"speed_scale": 0.3}
    live.frame_offset = app_module._JointFrame()
    return live, app_module.BridgeError


def run_case(mode: str, *, label: str) -> dict:
    mech = Mechanism(START_DEG, mode=mode)
    server, url = serve(mech)
    tick_log: list = []
    try:
        live, BridgeError = make_live(url, tick_log)
        started = time.time()
        error = None
        reply = None
        try:
            reply = live._stream_to(list(TARGET_DEG), list(START_DEG), tolerance=0.7)
        except Exception as exc:                      # noqa: BLE001
            error = f"{type(exc).__name__}: {exc}"
        elapsed = time.time() - started
        truth = mech.true_joints()
        travel = round(max(abs(a - b) for a, b in zip(truth, START_DEG)), 3)
        needed = round(max(abs(a - b) for a, b in zip(TARGET_DEG, START_DEG)), 3)
        # per-tick table: what the console commanded vs what the arm really did
        samples = []
        for entry in tick_log[:400]:
            samples.append({"sent": entry["sent"], "ack": entry["ack"]})
        return {
            "case": label, "mode": mode,
            "required_joint_delta_deg": needed,
            "real_max_travel_deg": travel,
            "achieved_fraction": round(travel / needed, 3) if needed else None,
            "elapsed_s": round(elapsed, 2),
            "commands_sent": len(tick_log),
            "commands_accepted": mech.accepted,
            "commands_rejected": mech.rejected,
            "reported_error_deg": None if reply is None else reply.get("error_deg"),
            "exception": error,
            "true_final_joints": truth,
            "target_joints": TARGET_DEG,
            "converged": bool(reply and reply.get("error_deg") is not None
                              and reply["error_deg"] <= 0.8),
            # The failure signature the brief describes: a move reported as
            # fine, that did not move the arm.
            "silent_non_execution": bool(
                reply is not None and travel < 1.0
                and (reply.get("error_deg") or 0.0) <= 1.0),
            "stalled_incomplete": bool(reply is not None and travel < needed * 0.9),
            "first_ticks": samples[:4],
            "last_ticks": samples[-3:],
        }
    finally:
        server.shutdown()
        mech._stop.set()


def run_normal_budget() -> dict:
    """C4: does the step budget starve a big move inside the 30 s deadline?"""
    mech = Mechanism(START_DEG, mode="normal", rate_deg_s=35.0)
    server, url = serve(mech)
    tick_log: list = []
    try:
        live, _ = make_live(url, tick_log)
        started = time.time()
        reply = live._stream_to(list(TARGET_DEG), list(START_DEG), tolerance=0.7)
        elapsed = time.time() - started
        truth = mech.true_joints()
        needed = max(abs(a - b) for a, b in zip(TARGET_DEG, START_DEG))
        travel = max(abs(a - b) for a, b in zip(truth, START_DEG))
        return {"case": "budget_ok", "mode": "normal", "elapsed_s": round(elapsed, 2),
                "required_joint_delta_deg": round(needed, 2),
                "real_max_travel_deg": round(travel, 3),
                "achieved_fraction": round(travel / needed, 3),
                "commands_sent": len(tick_log),
                "reported_error_deg": reply.get("error_deg"),
                "converged": (reply.get("error_deg") or 9) <= 0.8,
                "starved": travel < needed * 0.9}
    finally:
        server.shutdown()
        mech._stop.set()


def run_stale_start() -> dict:
    """C7: the console's `start` is an OLD pose; the arm is really elsewhere.

    This is what a stale `HOLD` readback actually does: the console solves and
    streams from a pose the arm is no longer in. The loop then commands
    `reported_start + step`, which is a *small* joint change from the console's
    point of view but a large one from the arm's. Included because it is the
    most faithful reconstruction of the 2026-11-23 report.
    """
    stale_start = [-13.0, -11.0, 25.0, 78.0, 67.0, 44.0]   # what /state says
    true_start = START_DEG                                   # where the arm is
    mech = Mechanism(true_start, mode="normal")
    server, url = serve(mech)
    tick_log: list = []
    try:
        live, _ = make_live(url, tick_log)
        started = time.time()
        reply = live._stream_to(list(TARGET_DEG), list(stale_start), tolerance=0.7)
        elapsed = time.time() - started
        truth = mech.true_joints()
        travel = max(abs(a - b) for a, b in zip(truth, true_start))
        reported_travel = max(abs(a - b) for a, b in zip(stale_start, TARGET_DEG))
        return {"case": "C7_console_start_is_stale", "mode": "normal",
                "elapsed_s": round(elapsed, 2),
                "required_joint_delta_deg": round(reported_travel, 2),
                "real_max_travel_deg": round(travel, 3),
                "commands_sent": len(tick_log),
                "reported_error_deg": reply.get("error_deg"),
                "true_final_joints": truth,
                "converged": bool((reply.get("error_deg") or 9) <= 0.8),
                "silent_non_execution": bool(
                    reply is not None and travel < 1.0
                    and (reply.get("error_deg") or 0.0) <= 1.0)}
    finally:
        server.shutdown()
        mech._stop.set()


def run_zero_distance() -> dict:
    """C6: a target already within `tolerance` of `start` returns success, no motion.

    This is the ONLY code path in `_stream_to` that produces the 2026-11-23
    signature — accepted, tiny `error_deg`, arm never moved — and it needs no
    bridge misbehaviour at all. It fires whenever the console's notion of
    `start` is already at the goal, e.g. a solve seeded from a stale pose that
    returns its own seed, or a caller asking for where it already is.
    """
    mech = Mechanism(START_DEG, mode="normal")
    server, url = serve(mech)
    tick_log: list = []
    try:
        live, _ = make_live(url, tick_log)
        started = time.time()
        # Command the pose the arm already holds, exactly as a bad solve would.
        reply = live._stream_to(list(START_DEG), list(START_DEG), tolerance=0.7)
        elapsed = time.time() - started
        truth = mech.true_joints()
        travel = max(abs(a - b) for a, b in zip(truth, START_DEG))
        return {"case": "C6_target_equals_start", "mode": "normal",
                "elapsed_s": round(elapsed, 3),
                "required_joint_delta_deg": 0.0,
                "real_max_travel_deg": round(travel, 3),
                "achieved_fraction": 1.0,
                "commands_sent": len(tick_log),
                "reported_error_deg": reply.get("error_deg"),
                # The dangerous shape: success reported, nothing moved, and no
                # way for the caller to tell this from a real arrival.
                "silent_non_execution": bool(
                    len(tick_log) == 0 and travel < 1e-6
                    and (reply.get("error_deg") or 0.0) <= 1.0),
                "converged": True}
    finally:
        server.shutdown()
        mech._stop.set()


def run_seq_ttl() -> dict:
    """C5: a rejected command is dropped with no retry — does it strand a move?"""
    mech = Mechanism(START_DEG, mode="reject_every_other")
    server, url = serve(mech)
    tick_log: list = []
    try:
        live, _ = make_live(url, tick_log)
        started = time.time()
        error = None
        reply = None
        try:
            reply = live._stream_to(list(TARGET_DEG), list(START_DEG), tolerance=0.7)
        except Exception as exc:                      # noqa: BLE001
            error = f"{type(exc).__name__}: {exc}"
        elapsed = time.time() - started
        truth = mech.true_joints()
        needed = max(abs(a - b) for a, b in zip(TARGET_DEG, START_DEG))
        travel = max(abs(a - b) for a, b in zip(truth, START_DEG))
        return {"case": "seq_reject_half", "mode": "reject_every_other",
                "elapsed_s": round(elapsed, 2),
                "required_joint_delta_deg": round(needed, 2),
                "real_max_travel_deg": round(travel, 3),
                "achieved_fraction": round(travel / needed, 3),
                "commands_sent": len(tick_log),
                "commands_accepted": mech.accepted,
                "commands_rejected": mech.rejected,
                "reported_error_deg": None if reply is None else reply.get("error_deg"),
                "exception": error,
                "converged": bool(reply and (reply.get("error_deg") or 9) <= 0.8)}
    finally:
        server.shutdown()
        mech._stop.set()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="reports/phase2_diagnosis.json")
    args = ap.parse_args()

    cases = [run_normal_budget()]
    for mode, label in (("empty_joints", "C1_empty_joint_dict"),
                        ("partial_joints", "C1b_partial_joint_dict"),
                        ("zeros_joints", "C1c_all_zero_joint_dict"),
                        ("stale", "C2_stale_readback"),
                        ("echo", "C3_state_echoes_command")):
        cases.append(run_case(mode, label=label))
    cases.append(run_seq_ttl())
    cases.append(run_zero_distance())
    cases.append(run_stale_start())

    print(f"{'case':32} {'travel':>8} {'need':>7} {'frac':>6} {'err':>7} "
          f"{'secs':>6} {'sent':>5} {'rej':>4}  verdict")
    verdicts = {}
    for case in cases:
        frac = case.get("achieved_fraction")
        verdict = "converged"
        if case.get("silent_non_execution"):
            verdict = "SILENT NON-EXECUTION"
        elif frac is not None and frac < 0.9:
            verdict = "stalled"
        elif case.get("starved"):
            verdict = "starved"
        verdicts[case["case"]] = verdict
        print(f"{case['case']:32} {case.get('real_max_travel_deg', 0):8.2f} "
              f"{case.get('required_joint_delta_deg', 0):7.1f} "
              f"{(frac if frac is not None else -1):6.2f} "
              f"{str(case.get('reported_error_deg')):>7} "
              f"{case.get('elapsed_s', 0):6.2f} {case.get('commands_sent', 0):5} "
              f"{case.get('commands_rejected', 0):4}  {verdict}")

    payload = {"tool": "tools/phase2_diagnose_stream.py",
               "when": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "start_deg": START_DEG, "target_deg": TARGET_DEG,
               "cases": cases, "verdicts": verdicts}
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=1) + "\n")
    print(f"\n-> {out}")


if __name__ == "__main__":
    main()
