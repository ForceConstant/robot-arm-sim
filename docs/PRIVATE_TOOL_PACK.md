# External parallel-tool packs

The optional `cad_parallel` profile imports a standalone MJCF plus an external
JSON contract. Private geometry is not part of this repository or its public
release archives. Supply a pack locally and set:

```sh
ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=/absolute/path/to/pack \
  MUJOCO_GL=egl python -m uvicorn robot_web.app:app --port 8098
```

Missing requested assets are an explicit startup error, not a silent stock-tool
fallback. Without that environment setting, the public checkout uses stock SO101.

## Contract

`profile.json` contains:

- `name`, `source_microversion`, `model_file`: display/provenance and MJCF path.
- `base_body`, `pinion_body`, `pinion_joint`, `left_body`, `right_body`,
  `jaw_joints`: names in the standalone model. One pinion drives two sliders.
- `ratio_m_per_rad`, `gap_at_zero_m`, `angle_range_rad`, `aperture_range_m`:
  symmetric command conversion `gap = reference_gap - 2*ratio*angle`.
- `mount_rotation` (3×3), `mount_position` (m): CAD coordinates to the retained
  wrist-roll body's frame. `tcp_position` is in that same arm-body frame;
  tool-site +x is approach, +z is jaw span, and the pinch center is fixed.
- `tip_reach_m`, `finger_halfwidth_m`, `finger_proximal_reach_m`: physical finger
  envelope relative to the TCP, used instead of stock pad/jaw assumptions.
- `servo_position`, `servo_rotation`, `servo_mass_kg`, `camera_mass_kg`:
  installed-envelope/mass estimates, not hardware calibration.
- `ik_boxes`: conservative full-aperture envelope boxes (`pos`, `size` in metres,
  where size is half-extent). Only the IK-only model enables these collision
  proxies; world physics retains the full imported compound contacts.
- `assumptions`: source ratio differences, material, mounting and calibration limits.

The model references its assets under `assets/`; include attribution/licenses.
Keep captures, credentials and CAD-edit receipts out of runtime packs. Position
and torque limits belong in the supplied MJCF and remain enabled after mounting.

## Integration invariants

- Retain five original arm joints, including `wrist_roll` on body `gripper`.
  Remove only the stock finger, old gripper actuator and moving jaw. Add one
  external pinion actuator; the six-command action vector remains radians.
- Initialize dependent jaw positions consistently on explicit resets. White and
  black arms use separate prefixes/DOF addresses/equality constraints.
- Symmetric tools do not use the stock fixed-prong offset or empirical executed
  reach map. Existing heuristic clearance is not a full path planner.
- Match grip height to an actual piece section that fits the tool aperture.
  Use the whole straight finger band, respect tip/table clearance and keep force
  bounds. Small heads can require a lower wider section; do not command beyond
  the model's limits or mask failures with welds/teleports.
- The custom profile uses a 0.5 mm commanded squeeze estimate. This is an assumed
  contact preload, not measured motor calibration or rubber pads. Source model
  friction and torque limits stay unchanged.
- The wrist camera remains on each arm; include a colliding estimated housing
  with its own mass. Mount/servo/camera assumptions require hardware verification.

## Checks and publication

```sh
MUJOCO_GL=egl python -m pytest -q tests
ROBOT_GRIPPER=cad_parallel ROBOT_GRIPPER_ASSETS=/path/to/private/pack \
  MUJOCO_GL=egl python -m pytest -q tests
```

The public suite includes a synthetic (non-CAD) adapter fixture. Two additional
integration tests require an explicitly supplied private pack; they check both
arm graphs, independent bounded actuator-driven cycles and absence of hidden
attachment welds, measured mounted mesh aperture and the supplied zero-opening
endpoint within 0.1 mm after settling. The same two-arm pawn lift/place regression
runs for both stock and custom profiles, preserving physical-lift and undisturbed-
board thresholds. Passing these two moves is not full manipulation qualification.

Inspect all four real camera images and browser start/pause/resume/stop/reset.
Save private screenshots/receipts under `.private/`, not in the public release.
Only the authorized deployment script adds the private pack to the LAN Docker
build context. Do not publish that resulting private image to a registry.

Empty jaw cycling, two baseline pawn moves and environment controls do not
establish full-game reliability, crowded-board clearance or physical validation.
