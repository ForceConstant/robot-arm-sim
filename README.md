# Robot Arm · Chess Lab

**Brian's fork:** [ForceConstant/robot-arm-sim](https://github.com/ForceConstant/robot-arm-sim)
of [qalby-tech/embryo_lab_chess_so101](https://github.com/qalby-tech/embryo_lab_chess_so101).
Upstream project documentation follows below; it describes upstream work, not
newly reproduced results for this fork. See [UPSTREAM.md](UPSTREAM.md).

## Colab GPU preflight (free tier)

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/ForceConstant/robot-arm-sim/blob/main/training/notebooks/colab_preflight.ipynb)

Run the hardware check on a free Colab GPU runtime, then hand the JSON report
back to the project session:

- **Open in Colab:** <https://colab.research.google.com/github/ForceConstant/robot-arm-sim/blob/main/training/notebooks/colab_preflight.ipynb>
- **View the notebook on GitHub:** <https://github.com/ForceConstant/robot-arm-sim/blob/main/training/notebooks/colab_preflight.ipynb>

The notebook installs nothing, mounts no Drive, uploads no data and fetches no
weights. It reports the allocated GPU, VRAM, versions and free disk, runs 20
synthetic optimizer steps with an exact checkpoint reload, and adds a pure
arithmetic fit estimate for a 5.44B policy (10.1 GiB of fp16 weights; ~40 GiB of
AdamW state for a full fine-tune, so a 16 GB T4 cannot full-finetune it).
It is a compute smoke test, not task training.

## Perception milestone — board registration and occupancy/colour (Priority 6)

A calibrated overhead camera, a known board size/height and one empty-board
reference per session are enough to register the board and classify all 64
squares as empty, white, black or unknown. On held-out sessions the unoccluded
top view answered 96% of squares at 99.4% accuracy; the wrist view answered
99.9% at 97.8%. Natural arm occlusion abstains 90.6%. Board registration is
1.42 mm median error; 5 of 8 held-out appearance draws passed the registration
gate (the rest abstained). No FEN, piece pose or simulator truth is read at
inference.

[Acceptance, limits, receipts and reproduction](training/PERCEPTION_ACCEPTANCE.md)

## Environment-first milestone (v1.0.0)

### Incident recording, replay and the torture suite (v1.2.0)

Every physical action can now be recorded and interrogated: full MuJoCo integration
state, the controller's intent (phase, target TCP, commanded jaw gap), the executed tool
pose and actual gap, neighbour displacement, contact candidates with peak force, and
before/after camera frames. Recordings are gated on a model-plus-source signature and the
MuJoCo version, so they only replay on the build that produced them.

- **Replay** re-renders saved states. It is a visualisation, not a physics re-simulation.
- **Re-run** restores the recording's frame 0 (the state before the command) and executes
  the same task again.
- **`crowded-v1` torture suite**: 12 independent manipulation cases (both bishops from the
  observed failures included), each on a fresh board, no chess engine involved. A failed
  case does not contaminate the next.
- **Fixed camera views**: overview, four isometrics, left, right, white side, black
  side, overhead and the two wrist cameras. Deliberately not a free orbit camera —
  dragging gimbal-locked near the poles and drifted while the arm moved.

`GET /api/incidents`, `GET /api/incidents/{id}`, `GET /api/incidents/{id}/trace`,
`GET /api/incidents/{id}.zip`, `POST /api/incidents/{id}/label`, and `GET /api/suite`
expose the same data over HTTP. Review categories are stored with the recording; a
candidate category is not a root cause until triaged.

### Parallel CAD tool integration (v1.1.0)

- Both arms can mount an external parallel-jaw MJCF tool pack while retaining
  all five arm joints. Each tool has one bounded pinion actuator and two
  independent coupled slider jaws; the existing six-command arm API remains.
- Symmetric aperture/TCP mapping, full compound contact geometry, an estimated
  servo/camera envelope and conservative IK envelopes replace stock finger pads.
- Stock executed-reach calibration is disabled for the new tool. Grip planning
  uses the full finger band and actual tool aperture rather than stock jaw geometry.
- The runtime reports the active tool and measured jaw aperture for each arm.
- Private CAD meshes, profiles, capture JSON and screenshots are **not included**
  in Git or public releases. Public checkout defaults to the stock tool. To use
  an external pack, set `ROBOT_GRIPPER=cad_parallel` and
  `ROBOT_GRIPPER_ASSETS=/absolute/path/to/pack` (containing `profile.json`,
  `model.xml`, referenced assets and license attribution).
- The HTPC compose selects the custom tool; deploy requires the separately
  supplied private pack. `scripts/deploy.py` adds it only to the authorized LAN
  build context, never to the public source/runtime release archive.
- Mount alignment, servo/camera masses, printed mass, friction and motor response
  are simulation assumptions until hardware exists. This is not full-game,
  crowded-board, payload or physical-fit qualification.

See [the tool-pack contract](docs/PRIVATE_TOOL_PACK.md) for mounting and testing.

- Two distinct SO101 follower models on opposite sides of one 32-piece board.
- Browser UI with Start, Pause/Resume, Stop, Reset, one-turn stepping and target speed.
- Two-hand keyboard teleoperation of the selected arm: the left hand drives the
  tool point (W/S forward-back, A/D left-right, Q/E lower-raise) and the right
  hand turns the wrist (left/right arrows) and opens/closes the jaws (up/down
  arrows). Every step is solved so the wrist stays vertical and the elbow stays
  above the tool; a pose that would fold the elbow down is refused with a
  BLOCKED readout rather than snapped to.
- Overview, overhead-debug and independent white/black wrist-camera views.
- Stockfish self-play driver and deterministic opening/capture test modes.
- Wrist-only versus overhead-plus-wrists observation contract for future
  perception experiments; overhead remains available separately for debugging.
- Confirmed-position display, physical failure logs, placement diagnostics and PGN export.
- CPU physics plus NVIDIA EGL rendering in a single Dockhand-managed container.
- No real robot, serial device or Docker socket is connected to the application.

**Scope:** this milestone commissions the environment, not robust manipulation.
The upstream scripted controller uses exact simulator state and tuned contact
pads. Failed grasps, disturbed neighbours, inaccessible moves and collisions
stop a run visibly; complete-game reliability is not claimed. Promotion exchange
is not automated. Castling/en-passant action expansion is present but not yet
physically validated. Full pick/place optimization is explicitly deferred.

### Run

The simulator and development environment target Python 3.11, which is already
provided by the OpenClaw container. From the repository root, create/sync the
project venv and launch with it; this does not install another Python runtime.
Install Stockfish separately (`STOCKFISH_PATH=/usr/games/stockfish` in the
Docker image). Without it, only the opening/capture replay is available.

```sh
scripts/ensure_venv.sh
MUJOCO_GL=egl ROBOT_DATA_DIR=/tmp/robot-arm-data \
  .venv/bin/python -m uvicorn robot_web.app:app --host 0.0.0.0 --port 8098
```

Docker builds the dependencies and Stockfish into the image; startup does not
download models or install packages. The shipped [compose](deploy/compose.yaml)
is specific to Brian's HTPC LAN/GPU and embeds the [Dockerfile](Dockerfile).
`python scripts/deploy.py` registers/uploads/builds only the `robot_arm` stack
through Dockhand. It requires explicit infrastructure authorization; it is not
an installation step for other users. Every deploy then waits for the job, waits
for the new build to report ready, and runs the 25-case `crowded-v3` suite
against it, including mirrored transfers to the far half and opposite corners,
writing `.private/post-deploy-<version>.json`. Case failures are
reported as data, not treated as a failed deploy. `--no-verify` skips the run
for debugging only; `scripts/verify_deploy.py` can rerun it alone.

The web service remains running when the simulation is stopped. Use the browser
controls for run control, or Dockhand to stop/start the entire container. Pause
resumes in place; Stop cancels a run and the next Start resets. Press M to switch
the teleoperated arm; the HUD reports achieved motion, elbow clearance, wrist
tilt, wrist roll and jaw gap for each step. Configure the
driver, initial scene and observations only while stopped; Apply resets the scene.

State/health: `/api/status`, `/health`. Selected sensor contract:
`/api/observations` (joint feedback and selected camera links; no piece poses/FEN).
View frames: `/api/frame/<view>.jpg` for `overview`, `top`, `left`, `right`,
`white_side`, `black_side`, `iso_{ne,nw,se,sw}`, `white_wrist`, `black_wrist`.
Completed moves persist as `/data/last-game.pgn` and `/data/last-run.json`.

### Tests

```sh
MUJOCO_GL=egl python -m pytest -q tests
TEST_URL=http://127.0.0.1:8098 node scripts/browser-check.mjs
```

Local browser checks require Playwright and Chromium; they use the actual UI and
will briefly start/pause/stop/reset the simulation. No hardware is commanded.

---

# Upstream: chess_so101

An SO-101 arm playing chess in MuJoCo: sim-ready assets, one assembled scene, a
scripted expert that actually picks pieces up, and a typed API for writing your
own data collection, imitation learning, reinforcement learning and inference
scripts.

The trained policy, the demonstrations it learned from and two showcase reels
are published:

| | |
| --- | --- |
| policy | [XvKuoMing/so101_chess_molmoact2](https://huggingface.co/XvKuoMing/so101_chess_molmoact2) — MolmoAct2 5B + LoRA, 70,000 steps |
| corrected policy | [XvKuoMing/so101_chess_molmoact2_dagger](https://huggingface.co/XvKuoMing/so101_chess_molmoact2_dagger) — the same, continued on 1,000 expert corrections from its own missed grasps; captures 78/100 against 68 at the shipped horizon |
| demonstrations | [XvKuoMing/so101_chess](https://huggingface.co/datasets/XvKuoMing/so101_chess) — 6,095 episodes, 497,565 frames, 10 fps |
| showcase | [moves](https://huggingface.co/XvKuoMing/so101_chess_molmoact2/resolve/main/media/reel_moves.mp4) · [captures](https://huggingface.co/XvKuoMing/so101_chess_molmoact2/resolve/main/media/reel_captures.mp4) |
| experiment record | [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) |
| roadmap | [docs/ROADMAP.md](docs/ROADMAP.md) — what comes next, in order, with costs and exit criteria |
| worked examples | [docs/COOKBOOK.md](docs/COOKBOOK.md) |
| building the rig | [docs/hardware.md](docs/hardware.md) |

Both are reachable in code as `chess_sim.hub.MODEL_REPO` / `DATASET_REPO`.

## What it scores

Closed-loop in the simulator, scored by the same rule the expert is held to —
piece within 11 mm of the target square, upright, nothing else displaced:

| task | policy | expert |
| --- | --- | --- |
| move a piece to a square | 54/64 (84%) | 30/30 |
| take a piece off the board | 25/32 (78%) | 24/24 |

96 episodes on unseen positions, board and arm start pose randomized, median
placement error 5.3 mm. Rollout sampling alone flips about a quarter of
identical positions, so smaller comparisons say nothing — see
[docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) §6.

The expert's own bar, measured over randomized placements and both piece sets
([§11](docs/EXPERIMENTS.md)), is **332/360 (92.2%)**: moves 229/240 (95%),
captures 103/120 (86%). It is not uniform — the vendored set scored 175/180
(97%) against the deployed printed pack's 157/180 (87%), with printed captures
at 73% and printed knights at 50%. Compare a policy against the matching piece
set and task family, not the aggregate.

### Scripted-expert baseline

`tools/benchmark_expert.py` scores the expert itself over randomized placements
and piece sets, reporting per task family, per piece type, per piece set and per
seed — not one aggregate rate. Every verdict still comes from `env.evaluate`;
the task metadata used to slice the report is reporting-only and must never be
handed to a policy under test.

```bash
ROBOT_PIECE_SET=printed ROBOT_PIECE_ASSETS="$PWD/.private/piece-assets-v5" \
  MUJOCO_GL=egl python tools/benchmark_expert.py \
    --moves 40 --captures 20 --seeds 271828 314159 161803 \
    --piece-sets default printed --out .private/expert-baseline.json
```

## The package

```
chess_sim/
  assets/           sim-ready files: so101/ (MJCF + STL), pieces/<name>/ (OBJ + texture), scene/
  calib/            executed_reach.json - measured per-square fingertip accuracy
  config.py         every tunable number, as frozen pydantic models
  env.py            ChessSimEnv - reset, step, the scripted expert, the success rule
  tasks.py          MoveTask / CaptureTask and the samplers that draw episodes
  positions.py      random practice positions the piece set can actually hold
  policies.py       the Policy protocol and the LeRobot checkpoint adapter
  rollout.py        run_episode, evaluate (with confidence intervals), demonstrate, EpisodeRunner
  rewards.py        reward terms for reinforcement learning
  recorder.py       EpisodeRecorder - LeRobot-style episode files
  conventions.py    simulator radians <-> SO-101 dataset degrees
  training.py       the MolmoAct2 fine-tuning command, flag by flag
  hub.py            where the published policy and dataset live
  scene.py          MjSpec assembly (table, board, 32 pieces, arm, cameras), XML export
  ik.py             IK for the arm: mink QP tasks on an arm-only kinematic model
  gripper.py        measured pinch-pocket / jaw-gap calibration
  controller.py     the scripted pick-and-place expert
  reach.py          which squares and moves the expert can execute
examples/           play_expert.py, play_game.py, collect_demonstrations.py, export_dataset.py,
                    train_policy.py, train_rl.py, evaluate_policy.py, push_dataset.py,
                    calibrate_reach.py
hardware/           the physical build: dimensions, printable parts, 1:1 board artwork
docs/               EXPERIMENTS.md (what was measured), COOKBOOK.md, hardware.md
```

## Quick start

```bash
export MUJOCO_GL=egl PYTHONPATH=~/chess_so101
export GALLIUM_DRIVER=d3d12   # WSL2: render on the GPU (see below); harmless elsewhere
python examples/play_game.py                      # 20-ply game, side-view video
python examples/play_expert.py --episodes 5       # random moves, verdict per episode
python examples/play_expert.py --task capture --viewer   # live, real time
```

```python
from chess_sim import Camera, ChessSimEnv, EnvConfig, MoveTask, START_FEN

env = ChessSimEnv(EnvConfig())          # builds and compiles the scene once (~4 s)
env.reset(START_FEN)                    # any FEN board field; absent pieces park off-board
result = env.execute(MoveTask(from_square="g1", to_square="f3"))   # scripted expert
print(result.success, result.placement_error, env.position.fen())

observation = env.step(action)          # or drive the 6 joints yourself (radians, 30 Hz)
frame = env.render(Camera.TOP)          # robot cameras: TOP, WRIST; demo view: EXTERNAL
env.export_xml("scene.xml")             # standalone MuJoCo XML of the scene
```

Everything tunable is a frozen pydantic model, so a run's setup is one value you
can print, diff and store:

```python
from chess_sim import AppearanceConfig, BoardConfig, ControlConfig, EnvConfig, RandomizationConfig

config = EnvConfig(board=BoardConfig(square=0.030),
                   control=ControlConfig(hz=30, image_size=(320, 240)),
                   randomization=RandomizationConfig(board_shift=0.02))
env = ChessSimEnv(config)
env.reset(START_FEN, rng=random.Random(0))    # board shifted, arm start jittered
env.recolor(AppearanceConfig.sample(np.random.default_rng(3)))   # colors only, no recompile
```

The defaults are the measured ones — `EnvConfig()` is the setup every published
result was produced with.

## Writing your own scripts

Fuller worked examples live in [docs/COOKBOOK.md](docs/COOKBOOK.md).

**Data collection.** A sampler draws a task the arm can attempt in a fresh
random position; the recorder writes the episode with its verdict.

```python
from chess_sim import EpisodeRecorder, MoveSampler, RecorderConfig, demonstrate

recorder = EpisodeRecorder(env, RecorderConfig(root="datasets/chess_moves"))
results = demonstrate(env, MoveSampler(), episodes=500, rng=random.Random(0), recorder=recorder)
```

**Inference.** The adapter takes any checkpoint in LeRobot's registry - the type
and the input features are read from the checkpoint itself - and owns the two
knobs that decide how a chunked policy behaves: how many actions of a chunk to
execute before looking again, and how to spread each emitted target over the
control period. It executes the whole 30-action chunk by default, which is what
keeps up with a moving arm; `n_action_steps=5` is worth 17 points on captures
wherever the loop can wait for six times the model calls.

```python
from chess_sim import LeRobotPolicy, LeRobotPolicyConfig

policy = LeRobotPolicy.load(LeRobotPolicyConfig.for_dataset(
    checkpoint, dataset_root="datasets/lerobot/chess_mc", n_action_steps=5))
action = policy.select_action(observation, task)
```

**Evaluation.** Scored by `env.evaluate`, the same rule as the expert, with
Wilson intervals so two checkpoints are not mistaken for different.

```python
from chess_sim import CaptureSampler, MoveSampler, RolloutConfig, evaluate

report = evaluate(env, policy, [(MoveSampler(), 64), (CaptureSampler(), 32)], RolloutConfig())
print(report.summary())     # 79/96 successes (82%, 95% CI 73-89%); median 5.3 mm
```

**Reinforcement learning.** `EpisodeRunner` puts a gym-shaped loop around a task:
shaping reward per step, outcome terms at the end, the same verdict at the close.

```python
from chess_sim import EpisodeRunner, RewardConfig, RolloutConfig

runner = EpisodeRunner(env, RolloutConfig(max_steps=450), RewardConfig())
task = MoveSampler().sample(env, rng)
observation = runner.begin(task)
while True:
    step = runner.step(my_policy(observation, task))
    observation = step.observation
    if step.done:
        print(step.reward, step.result.success)
        break
```

The reward weights in `RewardConfig` are a starting point — no run has been
trained against them yet.

**Reinforcement learning.** `examples/train_rl.py` fits a residual of per-joint
offsets over a frozen base policy with the cross-entropy method - no gradients
through the base, which matters when a model call costs seconds.

**DAgger.** `recover(env, policy, task, recorder)` lets the policy drive until it
is demonstrably going wrong — wrong piece engaged, a neighbour knocked, the named
piece still where it started — then hands the episode to the expert and records
only the correction, tagged with what went wrong.
`examples/collect_recoveries.py` runs that over a dataset's worth of episodes.

## The pipeline end to end

```bash
# 1. record demonstrations in parallel, each with a different look   (~4 s/episode, 4 workers)
python examples/collect_demonstrations.py --episodes 5000 --workers 4 --randomize \
    --task move --out datasets/chess_moves

# 2. convert to a LeRobot v3.0 dataset                               (in the VLA env)
python examples/export_dataset.py --in datasets/chess_moves datasets/chess_captures \
    --repo-id XvKuoMing/so101_chess --root datasets/lerobot/chess_mc --stride 3

# 3. fine-tune MolmoAct2 in blocks, keeping the best checkpoint      (one GPU)
python examples/train_policy.py --root datasets/lerobot/chess_mc \
    --out outputs/molmoact2_mc --total-steps 70000 --block 10000

# 4. score it closed-loop
python examples/evaluate_policy.py --checkpoint outputs/molmoact2_mc/checkpoints/070000/pretrained_model \
    --moves 64 --captures 32 --dataset-root datasets/lerobot/chess_mc

# 5. publish the dataset
python examples/push_dataset.py --root datasets/lerobot/chess_mc --collection so101_datasets
```

`training/` wraps steps 1–3 as unattended shell scripts with every setting in
`training/settings.env`. The isolated CUDA/LeRobot training image remains on its
own pinned Python 3.12 stack; steps 2–5 need that VLA environment. It is separate
from local simulator development and does not require a host Python 3.12 install.
Point `PYTHONPATH` at this repo so training scripts can import `chess_sim`.

## Hardware

![The rig](docs/media/rig_overview.png)

[docs/hardware.md](docs/hardware.md) is the build guide: scale drawings of the
layout and the heights, what to buy, what to print, the board artwork at 1:1,
piece dimensions, where the cameras go and what they must see, and the assembly
order. Everything in it is generated from `chess_sim` by `hardware/generate.py`,
so the bench matches the scene the policy trained in - change a dimension in
`chess_sim/config.py` and re-run it.

Nothing there has been built and measured on a physical bench yet; the
sim-to-real gap is unmeasured.

## Design notes

- **Fixed topology.** All 32 pieces always exist as free bodies; `reset(fen)`
  moves them onto squares or into off-board graveyard slots. One compiled model
  serves every episode, and the scene exports as a plain XML.
- **Mini board.** 2.8 cm squares, arm on a 6 cm riser 8 cm from the edge: the
  SO-101 reaches all 64 squares with the tool vertical, and the servos track
  every square center within 3 mm (`examples/calibrate_reach.py` measures that
  map; the sampler excludes squares over 5 mm or inside the near field). Closer
  mounts lose the arm's own back ranks: 4 cm from the edge the servos cannot
  track ranks 1–3 at all, so no game from the initial position.
- **Cameras.** The real SO-101 rig has two cameras and datasets record exactly
  those (`chess_sim.ROBOT_CAMERAS`): `top`, mounted as on the rig, where a plate
  under the arm base carries a 60 cm square-tube mast beside the arm with the
  camera on top looking down at the board (`scene.MAST_*`; the image is upright
  along the files, white at the bottom), and `wrist`, a wide-angle camera on the
  gripper beside the wrist-roll motor, looking at the fingertips
  (`scene.WRIST_CAMERA`). `external` is a fixed side view for demos and
  debugging; `ChessSimEnv` renders the robot cameras by default and
  `EpisodeRecorder` refuses an env without them.
- **The board moves.** It is a mocap body, so `reset(rng=...)` can shift it up to
  5 mm on each axis between episodes and the collision geometry moves with it —
  as a static world geom its contact bounds stay where the model was compiled,
  and pieces fall through the strip it was moved onto.
- **Contact with the table and the board** carries near-unit impedance
  (`scene.SURFACE_SOLIMP`) and contact priority. MuJoCo's default soft contact
  leaves a fraction of the pushing acceleration unopposed, which a 1 g piece
  pressed by a servo-driven arm converts into sinking through the slab. An
  explicit stiffness (negative `solref`) fixes the sinking and flings gram-scale
  pieces instead.
- **Piece colliders** are profiled stacks of cylinders sampled from the mesh
  (flat base for stable settling, true radii above it). Generated convex hulls
  rest on a rounded nub and creep across the board.
- **Where a piece is held.** Straight prongs cannot pinch a narrow neck below a
  wider crown, so each piece is grasped at its widest segment above the base (a
  king by its crown, a pawn by its head) and the jaws close to that radius. The
  2 cm pads must not reach down into a wider segment below the one they clamp, so
  a rook is held high enough that the pads stay above its base flange (it drops
  7 mm at release), and open prongs are positioned to clear the widest radius
  anywhere in the band the pads span, on the way in and out.
- **The grasp is physical.** Pieces are held purely by friction between fingertip
  contact pads: the jaws close 2.5 mm past the piece surface and the piece rides
  on the resulting clamp (~1–2 N) through lift, transit and placement, with the
  live piece-to-pocket offset measured before setting down. What made it work
  (see `gripper.py`): the stock finger meshes collide as convex hulls that fill
  the pinch pocket, so thin pads replace them at the fingertips; MuJoCo's default
  soft contact is mass-normalized and clamps a 4 g piece with ~0.1 N, so the pads
  carry an explicit stiffness (`solref` in N/m), full 6-D friction and contact
  priority; the moving pad is mounted parallel to the fixed one at the nominal
  grasp angle; and the scene uses elliptic friction cones with `impratio` 10,
  because with MuJoCo's default pyramidal cones a held piece creeps through the
  pads by about a centimeter over a two-second carry.
- **IK is only a target generator.** `So101Ik` turns "pinch pocket at this
  square" into joint targets; the position actuators then track them under full
  dynamics, with a small integral bias per waypoint to cancel the servos'
  pose-dependent steady-state error. The solver is
  [mink](https://github.com/kevinzakka/mink): each solve is a handful of
  quadratic programs over an arm-only copy of the SO-101 mounted as in the scene
  (`scene.build_arm`: 6 degrees of freedom instead of the scene's 198, about 25x
  faster per QP), with a tool-point task, an axis-align task for the approach
  direction, a roll-only jaw-span task, a weak posture tie-breaker, and joint
  limits plus a per-iteration step bound as hard constraints. The step bound
  matters: the arm's zero pose is a singular vertical stack, and an unbounded
  Gauss-Newton step from there lands in a wrong basin on the near squares.
- **Jaw-span selection.** The wrist roll is chosen per move so the moving jaw
  opens toward the freest neighboring square that the arm can actually reach with
  the tool near vertical and without folding into itself (`ChessSimEnv.grasp_span`,
  `So101Ik.solve(span=...)`), and the piece rides next to the fixed prong while
  the jaws are open so all the opening slack lands on the free side. The solver
  keeps the hand clear of the arm's own links (mink's collision-avoidance limit):
  the tight fold over the near squares otherwise drives the gripper into the
  shoulder.
- **Crowding.** On a fully populated board the ~5 mm prongs have only ~2 mm of
  corridor between 28 mm squares, so the jaw-span selection matters;
  `env.executable_moves()` is deliberately conservative there (it admits only two
  moves from the initial position), while `env.execute()` performs any quiet move
  you ask for. Measured with the physical grasp: 30/30 random sparse-board moves
  over three seeds at 0.0–4.6 mm placement, a 5-move ladder mate 5/5, and the
  20-ply Giuoco Pianissimo of `examples/play_game.py` from the initial position.
  A larger square pitch trades reach for clearance.
- **Rendering on WSL2.** MuJoCo's EGL backend picks Mesa's `llvmpipe` there, so
  every frame is rasterized on the CPU at ~300 ms. `GALLIUM_DRIVER=d3d12` routes
  it through WSL's D3D12 layer onto the real GPU: 22 ms per 640x480 frame with
  shadows, 12 ms without. Recording a 216-step episode from both robot cameras
  drops from ~2 minutes to ~16 seconds. NVIDIA's own EGL vendor library cannot be
  used (no `PLATFORM_DEVICE` support under WSL).
- **Not implemented.** Castling, promotion and en passant. A capture is executed
  as two pick-and-places (take the piece to the tray, then move in), which is
  what `examples/play_game.py` does when a move lands on an occupied square.
  Pieces lying flat on the board cannot be grasped at all — the jaws only close
  on a standing piece, so a toppled one ends the episode.
