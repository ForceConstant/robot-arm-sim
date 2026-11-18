"""The one chain(tool) -> jaw-tip frame used by the live solver.

The chain ends at the ``so101:gripper`` body (the arm's wrist-roll output).
Cartesian targets mean the *jaw tip*, so every target, every guard check and
every reported position goes through this constant.  It was measured from the
MJCF with :mod:`tools.audit_tip_frame`, not tuned on the bench:

* the ``gripperframe`` site origin sits at ``(-7.900, -0.218, -98.127) mm`` in
  the chain frame, on the fixed finger face; its +x is the approach axis
  (chain -z) and its +z spans the jaws;
* ``GAP_AT_ZERO`` is 15.8 mm, so the pinch centre at the reference aperture is
  7.9 mm inboard of that face -- i.e. exactly on the chain's z axis.  That is
  the point a Cartesian target should mean;
* the furthest solid point of the stock finger mesh is at chain
  ``z = -104.425 mm`` on that axis (the contact pad face is 0.325 mm inside),
  constant over eight poses spanning pan, lift, elbow, wrist flex/roll and jaw
  opening (spread 0.000 mm / 0.000 deg).  The live calibration chain reproduces
  the MJCF body pose to 0.000 mm / 0.000 deg.

Keeping the origin on the chain axis is not cosmetic.  The five-joint arm
cannot hold azimuth independently, so ``ArmKinematics.solve`` pins position
and approach and leaves in-plane rotation weak.  A tip origin offset sideways
from that axis turns a few degrees of yaw error into millimetres of tip error,
which is how the solver ended up reporting 1 mm while the guard saw 9 mm.

Orientation: the tip frame's +z points back up the fingers, so the approach is
its -z; +y spans the jaws.  That is the frame ``ArmKinematics.xyz_target``
produces and the frame ``_keep_in_check`` tests against.

Do not add a second number.  ``tool_tip_mm = 83`` was a vertical measurement
taken with the wrist tilted (~39 deg); used as a pure z offset it understates
the axial offset by ~21 mm.  ``-173.027`` came from the custom CAD mount plus a
stale 110.45 mm hard-code; the CAD model's own site is only -100.45 mm, so that
value was wrong in both models.  Overrides via ``live_config.json`` are gone on
purpose: the split between config and defaults is what produced three offsets.
"""
from __future__ import annotations

import numpy as np

#: ``T_chain_tip``: tip-frame origin in the ``so101:gripper`` body frame.
#: On the wrist axis at the fingertip plane, per the evidence above.
TIP_IN_CHAIN = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, -0.104425],
    [0.0, 0.0, 0.0, 1.0],
])
