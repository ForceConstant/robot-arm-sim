# Deadband Step 6 — decision gate: what the floor is, and what it costs to move

**Date:** 2026-11-25
**Configuration under test:** P=32, I=0, D=32 (settled), take-up ON at 3.5°.
**Answer in one line:** the near-1° goal is **met** (worst measured settle 0.73°,
was 1.12°), the ≤0.3°-both-directions target is **not** met on the elbow's
gravity-assisted side (0.73°), and the end-to-end board-mm comparison **could
not be measured** because the console cannot keep a marker board plane. Neither
number was fudged to close the gate.

---

## 1. The measured floor, per joint and per direction

Same anchor for every rep (pan −60°, elbow 40°, tip at board (98, 70, 152) mm),
3 reps, ±10° steps, exact-goal hold 1.5 s, tail spread 0.000° in all runs.

| joint | direction | direct approach | with take-up | what it is |
|---|---|---|---|---|
| `shoulder_pan` | +10° | +0.24° | **−0.11°** | at one encoder count (0.088°) |
| `shoulder_pan` | −10° | −0.24° | **−0.07°** | at one encoder count |
| `elbow_flex` | +10° (gravity-assisted) | −0.02° | **−0.73°** | static band, released from past the goal |
| `elbow_flex` | −10° (loaded) | **−1.12°** | **−0.24°** | was hysteresis, now ~the pan floor |

The four residuals are now **all on the same side of the goal** (past it), which
is the signature of a known approach direction. The spread between directions
fell from 1.10° (elbow) and 0.48° (pan) to 0.48° and 0.04°.

### What each remaining term is

1. **Static band on the loaded elbow — 0.6–0.8°, mechanical.** It does not scale
   with the load (P=32 vs P=16 changed the direct approach by 0.8°, but the
   take-up's residual is the same 0.6–0.8 at both margins 3.5 and 4.5, and it got
   *worse* with a slower final leg). It is the point where stiction releases, so
   it is set by gearbox friction and preload, not by gain. Pan has the same
   mechanism but only ~0.07–0.11° of it, i.e. one count.
2. **Load droop — payload-proportional, still present where the loop is loaded.**
   With I=0 the loop cannot null a constant gravity torque; P=32 halved it, and
   the take-up removes the history-dependent part of what is left *at this
   pose*. It will grow at the extended corner poses: the 06:43 sweep's own
   per-leg `error_deg` was p50 **2.27°**, p90 2.92°, max 3.47° across 61 tip legs
   and 91 aim steps — none under 0.3° and only 6/61 under 1°. That number is the
   *settle shortfall of a long move*, which is where droop and the old 3.5°
   settle allowance compound. It is the biggest joint-level error in the last
   measured baseline, and it is not a hysteresis term.
3. **Frame/model bias — ~19 mm, unchanged and dominant in board-mm.** The 06:43
   baseline had `aim_err_mm` p50 7.44 mm and `aim_px_cfg` p50 52.9 px with a
   **0.0 mm** readout spread: a rigid offset, not servo noise. No gain, take-up
   or integrator moves it.
4. **Encoder quantisation — 0.088°/count.** The pan floor is exactly this.

## 2. Cheapest mitigations, ranked by cost per degree

1. **Keep the take-up ON (free, done).** 4.7–7.4× on the loaded elbow, pan at the
   quantisation floor, ~2× the commands on a 10° move (p50 15 → 30). It is the
   only change in this whole line of work that improved one direction without
   degrading the other joint.
2. **Bridge anti-windup, then I=8 (free, ~1 hour).** The step-3 data says the
   integrator is the only term that makes the loaded-direction error
   payload-independent; its failure was windup against an unreachable goal. The
   bridge already detects a stall (`_remote_tick`/stall guard) — on a detected
   stall, re-target to the measured pose instead of holding an error the loop
   cannot clear, then re-measure I=8 with the step-3 probe. Expected win: the
   2.3° corner-pose settle term, i.e. the largest remaining contribution to
   board-mm error.
3. **Take-up-aware settle allowance everywhere (free, done).** The old 3.5°
   allowance let long moves stop with p50 2.27° of error; the take-up final leg
   stops at 0.3–1.2°. This is why the console's reported `error_deg` on the A/B
   moves fell from p50 0.59 to 0.20.
4. **Mechanical: damping/grease the elbow+ wrist geartrain, check the horn
   screws and printed adapter (cheap hardware, ~30 min).** This is the only
   lever on the 0.6–0.8° static band; no software term can remove stiction.
5. **Replace one servo (cheap hardware, one part).** A higher-torque/higher-
   resolution servo at the elbow would cut both the droop (more Kp per degree of
   error) and the band. It is the most expensive option and the least justified
   until 1–3 are exhausted: the arm is already within 1° everywhere measured.
6. **Reduce payload (free-ish).** Lighter jaws reduce the droop term linearly;
   the current CAD jaw is the heavy option.

## 3. Is the goal met?

- **Near-1° per joint: yes.** Worst settle at the tested pose is 0.73°, and the
  loaded direction — the one the original 2–4° complaint was about — is 0.24°
  (−0.15 on the previous run).
- **≤0.3° both directions: no.** The elbow's gravity-assisted direction is
  0.73° (−0.64…−0.81 across runs), and it does not improve with a bigger margin
  (4.5°) or a slower final leg; it is stiction, not control.
- **End-to-end board-mm: not measured.** The 64-square sweep was started once
  the plane came back; it completed one square (a1) and then stopped itself
  ("board plane unavailable for 3 squares; stopping rather than commanding
  blind"). The single end-to-end datum is a1's tip leg: **4.19 mm / 1.04°**
  against the baseline's **7.17 mm / 2.47°** at the same square — consistent with
  the joint-level improvement, but one square is not a comparison and it is not
  presented as one. The last measured baseline remains
  `reports/aim_squares_out/aim_squares_boardfix_20261125.json` (aim p50 7.44 mm,
  p95 11.6, max 12.01; 3 tip refusals; 60/64 aim legs). Since the dominant term
  there is the ~19 mm frame bias rather than the servo settle, even a perfect
  servo would not move that p50 much — but that prediction is not a measurement,
  and this report does not treat it as one.

## 4. Blocking condition (step 5)

The 64-square sweep ran for one square and then stopped itself, because the
console cannot keep a marker board plane alive: `/aim-point` reports
`no board plane` / `board plane is stale (49-81 s, limit 30 s)` and the recover
refresh cannot renew it; a direct bridge frame decodes 0 ArUco ids with the
strict, lenient, inverted and 3×-upscaled detectors while the same detector
returns 32 ids on the rendered `board_charuco.png`; the live frame is very soft
(Laplacian variance 25 at home, 184 at the pose that points at the arm's own
body) and an independently described frame showed a blurred, partly blown-out
marker sheet in a corner with a large yellow surface across the centre. Frames
are live and stable, so this is aiming/focus, not a frozen capture.
`verify_baseline.py` is **7/7** in the run where the plane happened to come up,
and **6/7** (aim check only) in the runs where it did not — the failure is
intermittent and pose-dependent, and actuation, rel mode, rig-state and
return-home all pass.

Fixing it means refocusing the wrist lens or moving the camera, which is a
physical change to the mount and outside this task's scope. **Owner decision
needed:** is the wrist camera's focus the intended state (e.g. someone swapped
or knocked it today), or should it be re-focused and the sweep re-run?

## 5. Recommendation

Ship the take-up at 3.5° as the default (it is in `DEFAULTS`, so it survives a
fresh `live_config.json`), keep P=32/I=0, and do the bridge anti-windup + I=8
re-measurement next — it is free, it targets the largest remaining joint-level
term, and step 3 already has the probe and the failure protocol to test it.
Do not chase the 0.6–0.8° elbow band in software; that one is mechanical.
