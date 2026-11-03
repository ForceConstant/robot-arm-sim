"""Wrist geometry from saved lens/hand–eye calibration and measured joints.

No scene, simulator state, or board transform is accepted by inference. The
serial chain is a calibrated installation input (metres, radians, column
vectors); an evaluator may export it from a digital twin, never at inference.
Camera axes follow the project's OpenGL convention: +X right, +Y up, -Z view.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import map_coordinates

from .vision_geometry import CameraCalibration


def _transform(position, rotation):
    value = np.eye(4)
    value[:3, :3] = np.asarray(rotation).reshape(3, 3)
    value[:3, 3] = position
    if not np.isfinite(value).all() or not np.allclose(value[:3,:3].T @ value[:3,:3], np.eye(3), atol=1e-6) or np.linalg.det(value[:3,:3]) < .999:
        raise ValueError('invalid rigid transform')
    return value


@dataclass(frozen=True)
class WristCalibration:
    """Versioned installation geometry; lens distortion is Brown–Conrady."""
    width: int
    height: int
    intrinsics: tuple[float, float, float, float]  # fx, fy, cx, cy
    distortion: tuple[float, float, float, float, float]  # k1,k2,p1,p2,k3
    chain: tuple[dict, ...]
    camera_in_tool: tuple[tuple[float, ...], ...]
    calibration_id: str

    @classmethod
    def load(cls, path: str | Path):
        data = json.loads(Path(path).read_text())
        if data.pop('schema') != 'robot-arm-wrist-calibration/1':
            raise ValueError('unsupported wrist calibration')
        data['chain'] = tuple(data['chain'])
        result = cls(**data)
        result.validate()
        return result

    def validate(self):
        if self.width <= 0 or self.height <= 0 or len(self.intrinsics) != 4 or len(self.distortion) != 5:
            raise ValueError('invalid lens calibration')
        if not np.isfinite((*self.intrinsics,*self.distortion)).all() or min(self.intrinsics[:2]) <= 0:
            raise ValueError('invalid lens calibration')
        camera = np.asarray(self.camera_in_tool)
        if camera.shape != (4,4) or not np.allclose(camera[3], [0,0,0,1]):
            raise ValueError('invalid hand–eye matrix')
        _transform(camera[:3,3], camera[:3,:3])
        for link in self.chain:
            _transform(link['position'], link['rotation'])
            if 'joint_index' in link:
                if np.asarray(link['axis']).shape != (3,) or not np.isclose(np.linalg.norm(link['axis']), 1):
                    raise ValueError('invalid hinge axis')
                if not 0 <= link['joint_index'] < 6:
                    raise ValueError('invalid joint index')

    def camera(self, measured_joints) -> CameraCalibration:
        """Forward kinematics only; reject invalid/out-of-limit measurements."""
        joints = np.asarray(measured_joints, dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError('six finite measured joint angles required')
        world = np.eye(4)
        for link in self.chain:
            world = world @ _transform(link['position'], link['rotation'])
            if 'joint_index' not in link:
                continue
            angle = joints[link['joint_index']]
            if not link['limits'][0] - 1e-6 <= angle <= link['limits'][1] + 1e-6:
                raise ValueError('measured joint outside calibrated hard limits')
            anchor = np.asarray(link['anchor'])
            rot = Rotation.from_rotvec(np.asarray(link['axis']) * (angle - link['reference'])).as_matrix()
            world = world @ _transform(anchor - rot @ anchor, rot)
        world = world @ np.asarray(self.camera_in_tool)
        fx,fy,cx,cy = self.intrinsics
        return CameraCalibration(self.width,self.height,fx,fy,cx,cy,
                                 tuple(world[:3,3]), tuple(world[:3,:3].ravel()))

    def rectify(self, frame: np.ndarray) -> np.ndarray:
        """Undistort RGB into the calibrated pinhole grid; no hidden resize."""
        if frame.shape != (self.height,self.width,3) or frame.dtype != np.uint8:
            raise ValueError('frame must match calibrated RGB resolution')
        if not any(self.distortion):
            return frame.copy()
        fx,fy,cx,cy = self.intrinsics
        yy,xx = np.mgrid[:self.height,:self.width]
        x,y = (xx-cx)/fx, (yy-cy)/fy
        k1,k2,p1,p2,k3 = self.distortion
        r2=x*x+y*y
        radial=1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        u=fx*(x*radial+2*p1*x*y+p2*(r2+2*x*x))+cx
        v=fy*(y*radial+p1*(r2+2*y*y)+2*p2*x*y)+cy
        return np.stack([map_coordinates(frame[:,:,c], [v,u], order=1, mode='constant',cval=0)
                         for c in range(3)],axis=-1).astype(np.uint8)


@dataclass
class ReferenceEpoch:
    """Empty references belong to one registration/calibration epoch only.

    A board re-registration, camera recalibration or timestamp reversal clears
    references. Callers must acquire a NEW empty reference or continue without
    reference features; warping the old empty reference is not permitted.
    """
    epoch: str | None = None
    timestamp: float = -1.0
    references: dict | None = None

    def update(self, registration, calibration_id: str, timestamp: float) -> bool:
        if not np.isfinite(timestamp) or timestamp < 0:
            raise ValueError('finite nonnegative capture timestamp required')
        # Quantization is intentionally not used: every fresh accepted estimate
        # denotes a new registration epoch, even when movement is submillimetre.
        payload = [calibration_id, *registration.origin, registration.yaw,
                   registration.square, registration.board_z]
        epoch = hashlib.sha256(json.dumps(payload).encode()).hexdigest()
        changed = epoch != self.epoch or timestamp < self.timestamp
        if changed:
            self.references = {}
        self.epoch, self.timestamp = epoch, timestamp
        return changed
