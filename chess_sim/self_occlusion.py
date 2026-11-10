"""Authored arm geometry projected from calibrated, measured joints only.

No scene, renderer, body pose or simulator camera is consumed.  Authored mesh triangles
preserve apertures/concavities; a two-pixel image margin protects
bilinear sampling/raster edges.  This is a silhouette exclusion, NOT a cast
shadow detector, foreign-object detector or hardware calibration certificate.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_dilation
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh

from .wrist_calibration import WristCalibration, _transform

ARM_XML = Path(__file__).parent / 'assets' / 'so101' / 'so101.xml'


def _pose(element):
    position = np.fromstring(element.get('pos', '0 0 0'), sep=' ')
    quat = np.fromstring(element.get('quat', '1 0 0 0'), sep=' ')
    return _transform(position, Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix())


class SelfOcclusionModel:
    """Reusable installation model; ``exclusion_mask(joints)`` is rectified HxW bool.

    Geometry is loaded once from the authored SO101 XML/STLs and, when selected,
    the installed CAD tool pack/profile (same env selection as gripper.py).
    ``sources`` records hashes of every authored dependency for receipts.
    """

    def __init__(self, calibration: WristCalibration, *, padding_pixels: int = 2):
        from . import gripper
        calibration.validate()
        if not isinstance(padding_pixels, int) or padding_pixels < 0:
            raise ValueError('nonnegative integer image margin required')
        self.calibration = calibration
        self.padding_pixels = padding_pixels
        self.sources = {}
        self._meshes = {}
        self._triangles = {}
        self._parts = []
        self._mask_cache = {}
        root = self._xml(ARM_XML)
        assets = self._assets(root, ARM_XML.parent)
        bodies = {b.get('name'): b for b in root.iter('body')}
        self._custom = gripper.CUSTOM
        self._jaw_limits = (gripper.JOINT_MIN, gripper.JOINT_MAX)
        for link in calibration.chain:
            name = link['name'].split(':')[-1]
            if name not in bodies:
                raise ValueError(f'calibrated link missing authored geometry: {name}')
            if name == 'gripper' and gripper.CUSTOM:
                continue
            self._add_geoms(name, bodies[name], assets, np.eye(4))
        self._tool_tree = None
        if gripper.CUSTOM:
            profile = gripper.PROFILE
            self._record(gripper.ASSET_ROOT / 'profile.json')
            tool_path = gripper.ASSET_ROOT / profile['model_file']
            tool = self._xml(tool_path)
            self._tool_tree = tool.find('worldbody/body')
            self._tool_assets = self._assets(tool, tool_path.parent / tool.find('compiler').get('meshdir', '.'))
            self._mount = _transform(profile['mount_position'], profile['mount_rotation'])
            self._profile = profile
            self._tool_parts(np.zeros(6))  # eagerly hash all installed visual meshes
            servo = _transform(profile['servo_position'], profile['servo_rotation'])
            self._parts.append(('gripper', self._vertices(assets['sts3215_03a_v1']), servo))
        else:
            self._stock_jaw = bodies['moving_jaw_so101_v1']
            self._stock_assets = assets

    def _record(self, path):
        path = Path(path).resolve()
        self.sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

    def _xml(self, path):
        self._record(path)
        return ET.parse(path).getroot()

    @staticmethod
    def _assets(root, directory):
        result = {}
        for mesh in root.findall('asset/mesh'):
            if mesh.get('scale', '1 1 1') != '1 1 1':
                raise ValueError('scaled geometry requires an explicit calibrated model')
            path = directory / mesh.get('file')
            result[mesh.get('name', path.stem)] = path
        return result

    def _vertices(self, path):
        key = str(path)
        if key not in self._meshes:
            self._record(path)
            mesh = trimesh.load_mesh(path, process=False)
            vertices = np.unique(np.asarray(mesh.vertices), axis=0)
            hull = ConvexHull(vertices)
            self._meshes[key] = vertices[hull.vertices]
            self._triangles[id(self._meshes[key])] = np.asarray(mesh.triangles)
        return self._meshes[key]

    def _add_geoms(self, name, body, assets, transform):
        for geom in body.findall('geom'):
            if geom.get('class') != 'visual':
                continue
            if geom.get('type', 'mesh') != 'mesh':
                raise ValueError('unsupported authored visual primitive')
            self._parts.append((name, self._vertices(assets[geom.get('mesh')]), transform @ _pose(geom)))

    def _link_transforms(self, joints):
        # camera() validates joint shape, finiteness and calibrated hard limits.
        self.calibration.camera(joints)
        world = np.eye(4)
        transforms = {}
        for link in self.calibration.chain:
            world = world @ _transform(link['position'], link['rotation'])
            if 'joint_index' in link:
                angle = joints[link['joint_index']] - link['reference']
                rot = Rotation.from_rotvec(np.asarray(link['axis']) * angle).as_matrix()
                anchor = np.asarray(link['anchor'])
                world = world @ _transform(anchor - rot @ anchor, rot)
            transforms[link['name'].split(':')[-1]] = world.copy()
        return transforms

    def _tool_parts(self, joints):
        result = []
        def visit(body, parent):
            world = parent @ _pose(body)
            for joint in body.findall('joint'):
                name = joint.get('name')
                axis = np.fromstring(joint.get('axis', '0 0 1'), sep=' ')
                anchor = np.fromstring(joint.get('pos', '0 0 0'), sep=' ')
                if name == self._profile['pinion_joint']:
                    angle = joints[5]
                    rotation = Rotation.from_rotvec(axis * angle).as_matrix()
                    world = world @ _transform(anchor - rotation @ anchor, rotation)
                elif name in self._profile['jaw_joints']:
                    world = world @ _transform(axis * joints[5] * self._profile['ratio_m_per_rad'], np.eye(3))
                else:
                    raise ValueError(f'unmeasured tool joint: {name}')
            for geom in body.findall('geom'):
                if geom.get('class') == 'visual':
                    result.append((self._vertices(self._tool_assets[geom.get('mesh')]), world @ _pose(geom)))
            for child in body.findall('body'):
                visit(child, world)
        visit(self._tool_tree, self._mount)
        return result

    def exclusion_mask(self, measured_joints) -> np.ndarray:
        """Conservative silhouette on the undistorted calibrated pinhole grid.

        Invalid measurements raise, never silently produce an empty mask.
        Convex components are clipped at the near lens plane before projection;
        wholly behind-camera components are ignored.
        """
        joints = np.asarray(measured_joints, dtype=float)
        transforms = self._link_transforms(joints)
        if not self._jaw_limits[0] - 1e-6 <= joints[5] <= self._jaw_limits[1] + 1e-6:
            raise ValueError('measured jaw outside installed hard limits')
        key = tuple(joints)
        if key in self._mask_cache:
            return self._mask_cache[key].copy()
        camera = self.calibration.camera(joints)
        parts = [(vertices, transforms[name] @ local) for name, vertices, local in self._parts]
        if self._custom:
            parts += [(vertices, transforms['gripper'] @ local) for vertices, local in self._tool_parts(joints)]
        else:
            body = self._stock_jaw
            joint = body.find('joint')
            axis = np.fromstring(joint.get('axis'), sep=' ')
            rotation = Rotation.from_rotvec(axis * joints[5]).as_matrix()
            jaw = transforms['gripper'] @ _pose(body) @ _transform([0, 0, 0], rotation)
            for geom in body.findall('geom'):
                if geom.get('class') == 'visual':
                    parts.append((self._vertices(self._stock_assets[geom.get('mesh')]), jaw @ _pose(geom)))
        image = Image.new('1', (camera.width, camera.height))
        draw = ImageDraw.Draw(image)
        rotation = np.asarray(camera.camera_rotation).reshape(3, 3)
        for vertices, world in parts:
            xyz = (vertices @ world[:3, :3].T + world[:3, 3] - camera.camera_position) @ rotation
            depth = -xyz[:, 2]
            if np.max(depth) <= 0:
                continue
            # Rasterize actual triangles: a hull would fill jaw gaps and
            # the camera aperture, incorrectly removing useful board views.
            triangles = self._triangles[id(vertices)]
            triangles = (triangles @ world[:3, :3].T + world[:3, 3]
                         - camera.camera_position) @ rotation
            d = -triangles[:, :, 2]
            keep = d.max(axis=1) > 1e-4
            for values in (camera.fx * triangles[:, :, 0] + camera.cx * d,
                           (camera.width - camera.cx) * d - camera.fx * triangles[:, :, 0],
                           camera.cy * d - camera.fy * triangles[:, :, 1],
                           (camera.height - camera.cy) * d + camera.fy * triangles[:, :, 1]):
                keep &= values.max(axis=1) >= 0
            for triangle in triangles[keep]:
                polygon = []
                for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
                    da, db = -a[2], -b[2]
                    if da >= 1e-4:
                        polygon.append(a)
                    if (da < 1e-4) != (db < 1e-4):
                        polygon.append(a + (1e-4 - da) / (db - da) * (b - a))
                if len(polygon) >= 3:
                    polygon = np.asarray(polygon)
                    dd = -polygon[:, 2]
                    pp = np.column_stack((camera.cx + camera.fx * polygon[:, 0] / dd,
                                          camera.cy - camera.fy * polygon[:, 1] / dd))
                    draw.polygon([tuple(v) for v in pp], fill=1)
        mask = np.asarray(image, dtype=bool)
        if self.padding_pixels:
            mask = binary_dilation(mask, iterations=self.padding_pixels)
        if len(self._mask_cache) >= 64:
            self._mask_cache.pop(next(iter(self._mask_cache)))
        self._mask_cache[key] = mask.copy()
        return mask
