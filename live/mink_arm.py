"""An arm-only MuJoCo model built FROM the live calibration, for the mink solver.

Phase 1 of the solver unification.  ``chess_sim/ik.py`` solves with mink over an
arm-only MuJoCo model; the live console solves with a hand-rolled
``scipy.optimize.least_squares`` over the serial chain in
``live/kinematics.py``.  Two models of one arm is the thing that has to stop,
and the only way for the sim solver to be *the* solver is for its model to be
built from the same source of truth the live console uses:

    the WristCalibration chain JSON  (``live_config.calibration``)
  + ``live/tip_frame.TIP_IN_CHAIN``  (the one chain -> jaw-tip transform)

Nothing here reads ``chess_sim.config.BoardConfig`` or ``chess_sim.scene``: the
sim's mount is a scene constant, while the live arm's mount is a measurement
recorded in the chain's first link.  Mounting the model at that link is what
makes the mink solver's answers mean the same thing as the console's.

Reproducing the chain's FK
--------------------------
``ArmKinematics.fk`` composes, for each link::

    world = world @ T(link.position, link.rotation)
    if joint:  R = rotvec(axis * (angle - reference))
               world = world @ T(anchor - R @ anchor, R)

So a joint's rotation happens *inside* the link frame, about an axis through
``anchor``.  A MuJoCo hinge with ``pos=anchor``, ``axis=axis`` applies exactly
``T(anchor - R @ anchor, R)`` in that body's frame — the same transform, one for
one.  FK(chain) and FK(model) must therefore agree to floating-point noise, and
``tools/phase1_parity.py`` asserts that before anything else in Phase 1 is
believed.

``reference`` is folded into the qpos mapping rather than the model: a MuJoCo
hinge has no zero-offset attribute that keeps limits in the model's own
coordinates, so the adapter converts ``model = chain - reference`` on the way in
and back on the way out.

The site frame
--------------
``chess_sim.ik`` documents its site contract: **+x is the approach axis**,
**+z spans the jaws**.  The live tip frame has +z pointing back up the fingers
(approach = -z) and +y across the jaws.  The site this module adds is therefore
the tip frame carried through the fixed rotation

    site_x = -tip_z,   site_y = -tip_x,   site_z = +tip_y

placed at the tip origin, so the mink tasks and the console mean the same thing
by "approach" and "span".

What the model does NOT carry
-----------------------------
No link masses or collision geometry come from the sim's SO-101 MJCF.  The QPs
here are kinematic; the only geometry is the installed tool's conservative
``ik_boxes`` from its own live-side profile (the pack ``live/tip_frame.py``
already requires), because ``So101Ik`` wants a hand envelope for its
self-collision limit.  FK parity is unaffected by either choice.
"""
from __future__ import annotations

import numpy as np

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll")

#: tip-frame components of the three site axes (columns are the site axes):
#:   site_x = -tip_z = (0, 0, -1);  site_y = -tip_x = (-1, 0, 0);
#:   site_z = +tip_y = (0, 1, 0)
SITE_IN_TIP = np.array([[0.0, -1.0, 0.0],
                        [0.0, 0.0, 1.0],
                        [-1.0, 0.0, 0.0]])


def _quat_from_matrix(rotation: np.ndarray) -> list[float]:
    """(w, x, y, z) quaternion for a rotation matrix, MuJoCo's order."""
    m = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s,
             (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
             0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
             (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.asarray(q, dtype=float)
    return [float(v) for v in q / np.linalg.norm(q)]


def build_arm_model(calibration, tip_in_chain, prefix: str = "so101:",
                    tip_site: str = "gripperframe", limits: dict | None = None,
                    tool_boxes=None):
    """Compile an arm-only ``mujoco.MjModel`` from the calibration chain.

    ``limits`` overrides the chain's own limits, in degrees, exactly like
    ``ArmKinematics(limits=...)``; the live console refreshes its limits from the
    bridge, so a model carrying the chain's stale numbers would plan past the
    servo's range.

    ``tool_boxes`` is a list of ``(pos, size)`` pairs, in the tip/site frame,
    turned into ``clearance_proxy_*`` geoms for ``So101Ik``'s self-collision
    limit.  ``None`` means no collision geometry at all.

    Returns ``(model, info)``; ``info`` records the source values a caller may
    need to trust the model (references, limits, tip pose).
    """
    import mujoco

    chain = calibration.chain
    spec = mujoco.MjSpec()
    spec.modelname = "live_so101"
    spec.option.timestep = 0.002
    parent = spec.worldbody

    references = []
    joint_names = []
    for link in chain:
        position = [float(v) for v in link["position"]]
        quat = _quat_from_matrix(link["rotation"])
        # Body names come straight from the chain's link names: they are the
        # same names the sim's arm model uses (shoulder, upper_arm, lower_arm,
        # wrist, gripper), which is what So101Ik's body lookups require.  A
        # joint on a link rotates that link, exactly as the chain's FK does.
        body = parent.add_body(name=str(link.get("name") or (prefix + ARM_JOINTS[len(joint_names)])),
                               pos=position, quat=quat)
        if "joint_index" in link:
            name = ARM_JOINTS[len(joint_names)]
            anchor = [float(v) for v in link.get("anchor", [0.0, 0.0, 0.0])]
            axis = [float(v) for v in link["axis"]]
            if limits is not None and name in limits:
                lo, hi = limits[name]
                range_units = [float(lo), float(hi)]
            else:
                range_units = [float(np.degrees(v)) for v in link["limits"]]
            # MjSpec compiles with MJCF's default angle unit: degrees.
            # armature keeps the effective-metric solve away from a singular
            # inertia: the QPs are kinematic, this is numerical conditioning.
            body.add_joint(name=prefix + name, type=mujoco.mjtJoint.mjJNT_HINGE,
                           axis=axis, pos=anchor, range=range_units, limited=True,
                           armature=1e-3)
            joint_names.append(prefix + name)
            references.append(float(link.get("reference", 0.0)))
        # Kinematics only: the QPs never integrate dynamics, but MuJoCo rejects
        # a zero-mass body, so every link carries a nominal unit mass/inertia.
        body.explicitinertial = True
        body.mass = 0.1
        body.inertia = [1e-4, 1e-4, 1e-4]
        parent = body

    # Site = tip frame carried through SITE_IN_TIP, on the last body.
    tip = np.asarray(tip_in_chain, dtype=float)
    site_rotation = tip[:3, :3] @ SITE_IN_TIP
    wrist_body = parent
    wrist_body.add_site(name=prefix + tip_site,
                        pos=[float(v) for v in tip[:3, 3]],
                        quat=_quat_from_matrix(site_rotation),
                        size=[0.002, 0.002, 0.002], group=3)

    # Jaw bookkeeping joints: So101Ik's servo profile refers to the gripper and
    # its two slaved jaws, and a velocity limit of zero pins them.  They carry
    # no arm motion, so they cannot affect FK or the QP's arm solution.
    from chess_sim import gripper
    jaw_names = (prefix + "gripper",) + tuple(
        prefix + n for n in (gripper.PROFILE.get("jaw_joints", []) if gripper.CUSTOM else []))
    for name in jaw_names:
        wrist_body.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_HINGE,
                             axis=[0.0, 0.0, 1.0], range=[-0.6, 0.6], limited=True,
                             armature=1e-3)

    # Hand envelope: the installed tool's own conservative boxes, already in the
    # site frame (the pack's `ik_boxes` are site-frame, matching `gripperframe`).
    if tool_boxes:
        for index, (pos, size) in enumerate(tool_boxes):
            wrist_body.add_geom(
                name=f"clearance_proxy_{index}", type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[float(v) for v in pos], size=[float(v) for v in size],
                contype=0, conaffinity=0, rgba=[0.5, 0.5, 0.5, 0.2], group=4)

    model = spec.compile()
    info = {
        "joints": joint_names,
        "jaw_joints": list(jaw_names),
        "references_rad": references,
        "limits_deg": {n: [float(np.degrees(v)) for v in
                           model.jnt_range[model.joint(n).id]]
                       for n in joint_names},
        "tip_site": prefix + tip_site,
        "tip_in_chain_m": [float(v) for v in tip[:3, 3]],
        "mount_position": [float(v) for v in chain[0]["position"]],
        "mount_rotation": [list(map(float, row)) for row in chain[0]["rotation"]],
        "tool_boxes": len(tool_boxes or []),
    }
    return model, info


class MinkArm:
    """``chess_sim.ik.So101Ik`` over a chain-derived model, plus frame glue.

    The live console already does the board-frame bookkeeping — mode expansion,
    span/roll-lock, keep-in, guards — in ``ArmKinematics``.  This class replaces
    only the *solve*: it takes the same chain-frame goal pose the existing
    solver takes and returns the same shape of result dict, so a caller cannot
    tell which solver ran except by the numbers.
    """

    def __init__(self, calibration, tip_in_chain, limits: dict | None = None,
                 prefix: str = "so101:", rest: tuple[float, ...] | None = None,
                 tool_boxes=None):
        import mujoco
        from chess_sim.ik import So101Ik

        if tool_boxes is None:
            try:
                from chess_sim import gripper
                tool_boxes = [(b["pos"], b["size"]) for b in
                              gripper.PROFILE.get("ik_boxes", [])] if gripper.CUSTOM else []
            except Exception:
                tool_boxes = []
        self.model, self.info = build_arm_model(
            calibration, tip_in_chain, prefix=prefix, limits=limits,
            tool_boxes=tool_boxes)
        self.prefix = prefix
        self.references = np.asarray(self.info["references_rad"], dtype=float)
        self._arm_joints = [prefix + n for n in ARM_JOINTS]
        self._qpos_adr = np.array([self.model.jnt_qposadr[self.model.joint(n).id]
                                   for n in self._arm_joints])
        # One model for the solve and for its own FK: the chain-derived arm has
        # no separate scene, so the solver's FK cannot drift from its report.
        self.ik = So101Ik(self.model, self.model, prefix=prefix,
                          site="gripperframe",
                          rest=rest or (0.0, -0.6, 0.9, 0.7, 0.0))
        self.site = self.ik.site
        self._data = mujoco.MjData(self.model)

    # -- frames ------------------------------------------------------------
    def _set(self, q: np.ndarray, jaw_rad: float | None = None):
        """Put the model at arm joint angles ``q`` (radians, model frame)."""
        import mujoco
        data = self._data
        data.qpos[:] = self.model.qpos0
        data.qpos[self._qpos_adr] = q
        if jaw_rad is not None:
            data.qpos[self.model.joint(self.prefix + "gripper").qposadr[0]] = jaw_rad
        mujoco.mj_forward(self.model, data)
        return data

    def set_joints(self, joints_deg, jaw_deg: float | None = None):
        """MuJoCo data at the given *chain* joint angles (degrees)."""
        q = np.radians(np.asarray(joints_deg, dtype=float)[:5]) - self.references
        jaw = None if jaw_deg is None else float(np.radians(jaw_deg))
        return self._set(q, jaw)

    def tip_pose(self, joints_deg, jaw_deg: float | None = None):
        """(position_m, rotation) of the *tip* frame, in the chain's world frame."""
        data = self.set_joints(joints_deg, jaw_deg)
        rotation = data.site_xmat[self.site].reshape(3, 3).copy()
        # Undo the site rotation to recover the tip frame: R_tip = R_site @ S^T.
        return data.site_xpos[self.site].copy(), rotation @ SITE_IN_TIP.T

    # -- solve -------------------------------------------------------------
    def solve(self, goal_chain: np.ndarray, seed_deg, span_deg: float | None = None,
              freeze_roll: bool = False) -> dict:
        """Place the tip at ``goal_chain``'s origin with the tip's +z along
        ``goal_chain``'s +z, exactly as ``ArmKinematics.solve`` contracts.

        ``span_deg`` is the jaw-span azimuth in the goal's own x/y plane (the
        same convention ``ArmKinematics.solve`` uses).  ``freeze_roll`` pins
        wrist_roll to the seed, which the relative-move path asks for.
        """
        import math

        goal = np.asarray(goal_chain, dtype=float)
        seed = np.asarray(seed_deg, dtype=float)
        if seed.size < 6:
            seed = np.concatenate([seed, np.zeros(6 - seed.size)])
        target = goal[:3, 3]
        target_z = goal[:3, 2] / max(1e-9, np.linalg.norm(goal[:3, 2]))
        seed_q = np.radians(seed[:5]) - self.references

        self.ik._tilt.set_target(-target_z)
        span = None
        if span_deg is not None:
            direction = goal[:3, :3] @ np.array(
                [math.cos(math.radians(span_deg)),
                 math.sin(math.radians(span_deg)), 0.0])
            flat = direction[:2]
            norm = np.linalg.norm(flat)
            if norm > 1e-9:
                span = flat / norm

        data = self._set(seed_q, float(np.radians(seed[5])))
        if freeze_roll:
            result = self._solve_frozen_roll(target, span, seed_q)
        else:
            result = self.ik.solve(data, target, offset=np.zeros(3), span=span,
                                   seed_q=seed_q)

        joints = np.degrees(result.q) + self.references
        position, rotation = self.tip_pose(joints, seed[5])
        approach_error_deg = float(np.degrees(np.arccos(
            np.clip(float(rotation[:, 2] @ target_z), -1.0, 1.0))))
        position_error_mm = float(np.linalg.norm(position - target) * 1000.0)
        return {"joints_deg": dict(zip(
                    ("shoulder_pan", "shoulder_lift", "elbow_flex",
                     "wrist_flex", "wrist_roll", "gripper"),
                    [round(float(v), 3) for v in
                     np.concatenate([joints, seed[5:6]])])),
                "position_error_mm": round(position_error_mm, 2),
                "approach_error_deg": round(approach_error_deg, 3),
                "cost": float(result.position_error),
                "starts": 1,
                "solver": "mink"}

    def _solve_frozen_roll(self, target, span, seed_q):
        """Solve with wrist_roll held at the seed by narrowing its hinge range.

        A frozen roll is a hard constraint for this caller, not a soft pull, so
        the span task is dropped (it would fight the freeze) and the flipped-roll
        retry is skipped.
        """
        joint = self.model.joint(self._arm_joints[4])
        lo = float(self.model.jnt_range[joint.id][0])
        hi = float(self.model.jnt_range[joint.id][1])
        keep = float(seed_q[4])
        self.model.jnt_range[joint.id, 0] = keep - 1e-5
        self.model.jnt_range[joint.id, 1] = keep + 1e-5
        self.ik.lower[4] = keep - 1e-5
        self.ik.upper[4] = keep + 1e-5
        try:
            q, _residual = self.ik._iterate(seed_q, target, np.zeros(3), None)
        finally:
            self.model.jnt_range[joint.id, 0] = lo
            self.model.jnt_range[joint.id, 1] = hi
            self.ik.lower[4] = lo
            self.ik.upper[4] = hi
        position, _rotation = self.ik.forward(q, np.zeros(3))
        return _IkResult(q, float(np.linalg.norm(position - target)))


class _IkResult:
    def __init__(self, q, residual):
        self.q = np.asarray(q, dtype=float)
        self.residual = residual
        self.position_error = residual
        self.tilt = 0.0
