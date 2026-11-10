"""Known-geometry yaw fitting from calibrated RGB observations, never body poses.

This controlled-scene estimator renders an isolated authored mesh as an appearance
reference. Its camera poses come from calibration; its search centre comes from
the registered task square. The physical simulator environment is not an input.
Yaw is returned modulo pi because opposite thin-face normals identify one grasp
axis. Fit disagreement is retained as uncertainty/abstention, not hidden by a
best-scoring angle.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math
import cv2
import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from .vision_geometry import CameraCalibration


@dataclass(frozen=True)
class OrientationView:
    name: str
    rgb: np.ndarray
    calibration: CameraCalibration
    exclusion_mask: np.ndarray | None = None


@dataclass(frozen=True)
class AxisEstimate:
    axis_rad: float
    sigma_rad: float
    confidence: float
    supporting_views: int
    fit_loss: float
    diagnostics: dict
    world_xy: tuple[float, float] | None = None
    xy_sigma_m: float = float("inf")

    def acceptable(self, max_sigma_rad: float = math.radians(15),
                   min_confidence: float = .7) -> bool:
        return (np.isfinite(self.axis_rad) and np.isfinite(self.sigma_rad)
                and self.sigma_rad <= max_sigma_rad
                and self.confidence >= min_confidence
                and self.supporting_views >= 3)


def axis_distance_degrees(a, b):
    return abs((float(a) - float(b) + 90.) % 180. - 90.)


def fit_mesh_axis(mesh_path: str | Path, views: list[OrientationView],
                  expected_xy, board_z: float, *,
                  texture_path: str | Path | None = None,
                  tint=(1., 1., 1., 1.), angular_step_deg: int = 5,
                  search_pixels: int = 6) -> AxisEstimate | None:
    """Fit an upright known mesh using image appearance and calibrated views.

    Requires three independently named views. Inputs contain no simulation
    state, square occupancy oracle, body rotation, or target identity labels.
    Lighting is a fixed reference, not a measurement from the physical scene;
    poor appearance matches and inconsistent views must fail the confidence gate.
    """
    if len({v.name for v in views}) < 3:
        return None
    if angular_step_deg <= 0 or 180 % angular_step_deg:
        raise ValueError('angular step must divide 180')
    mesh_path = Path(mesh_path)
    mesh = trimesh.load(mesh_path, force='mesh')
    vertices = np.asarray(mesh.vertices)
    lo, hi = mesh.bounds[:, 2]
    head = vertices[vertices[:, 2] > lo + .47 * (hi-lo)]
    if len(head) < 4:
        return None
    spec = mujoco.MjSpec()
    width, height = views[0].calibration.width, views[0].calibration.height
    spec.visual.global_.offwidth = width
    spec.visual.global_.offheight = height
    spec.add_mesh(name='known_shape', file=str(mesh_path.resolve()))
    material = spec.add_material(name='reference', reflectance=.2, rgba=list(tint))
    if texture_path is not None:
        spec.add_texture(name='known_texture', type=mujoco.mjtTexture.mjTEXTURE_2D,
                         file=str(Path(texture_path).resolve()))
        material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'known_texture'
    body = spec.worldbody.add_body(name='template')
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='known_shape',
                  material='reference', density=0, contype=0, conaffinity=0)
    light = np.array([-.3, .3, -1.]); light /= np.linalg.norm(light)
    spec.worldbody.add_light(name='key', pos=(-2.4*light+[0,0,.4]).tolist(),
        dir=light.tolist(), diffuse=[.85,.82,.75], ambient=[0.,0.,0.],
        specular=[.3,.3,.3], castshadow=True)
    spec.worldbody.add_light(name='fill', pos=[-1.2,1.,1.8], dir=[.5,-.4,-1.],
        diffuse=[.35,.36,.4], castshadow=False)
    spec.worldbody.add_camera(name='observed', fovy=75)
    model = spec.compile(); data = mujoco.MjData(model)
    body_id = model.body('template').id
    renderer = mujoco.Renderer(model, height=height, width=width)
    xy = np.asarray(expected_xy, dtype=float)
    angles = np.arange(0, 360, angular_step_deg)
    losses, shifts, names = [], [], []
    try:
        for view in views:
            cam = view.calibration
            if (cam.width,cam.height) != (width,height):
                raise ValueError('all orientation frames must have the same resolution')
            frame = np.asarray(view.rgb)
            if frame.shape != (height,width,3) or frame.dtype != np.uint8:
                raise ValueError('orientation views must be calibrated uint8 RGB')
            model.cam_pos[0] = cam.camera_position
            q = Rotation.from_matrix(np.asarray(cam.camera_rotation).reshape(3,3)).as_quat()
            model.cam_quat[0] = q[[3,0,1,2]]
            model.cam_fovy[0] = np.degrees(2*np.arctan(height/(2*cam.fy)))
            model.body_pos[body_id] = [*xy,board_z-lo]
            projected = cam.project([*xy,board_z+.5*(hi-lo)])
            if projected is None: continue
            ux,uy = np.rint(projected).astype(int)
            resolution_scale = width / 640.
            half = int(round(20 * resolution_scale)); pad = int(round(search_pixels * resolution_scale)); x0,y0 = ux-half,uy-half
            if min(x0-pad,y0-pad)<0 or x0+2*half+pad>=width or y0+2*half+pad>=height:
                continue
            observed = frame[y0-pad:y0+2*half+pad,x0-pad:x0+2*half+pad].astype(np.float32)
            observed_exclusion = (None if view.exclusion_mask is None else
                np.asarray(view.exclusion_mask,dtype=bool)[y0-pad:y0+2*half+pad,x0-pad:x0+2*half+pad])
            values, offsets = [], []
            for yaw in angles:
                a = np.radians(yaw)
                model.body_quat[body_id] = [np.cos(a/2),0,0,np.sin(a/2)]
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera='observed')
                patch = renderer.render()[y0:y0+2*half,x0:x0+2*half].astype(np.float32).copy()
                rotation = np.array([[np.cos(a),-np.sin(a),0],
                                     [np.sin(a),np.cos(a),0],[0,0,1]])
                points = head@rotation.T + np.r_[xy,board_z-lo]
                local = (points-cam.camera_position)@np.asarray(cam.camera_rotation).reshape(3,3)
                depth = -local[:,2]
                pixels = np.c_[cam.cx+cam.fx*local[:,0]/depth-x0,
                                cam.cy-cam.fy*local[:,1]/depth-y0].astype(np.float32)
                mask = np.zeros((2*half,2*half),np.uint8)
                cv2.fillConvexPoly(mask,cv2.convexHull(pixels).astype(np.int32),1)
                mask[patch.sum(axis=2)<10] = 0
                if mask.sum()<12:
                    values.append(100.); offsets.append([0,0]); continue
                response = cv2.matchTemplate(observed,patch,cv2.TM_SQDIFF_NORMED,mask=mask)
                response[~np.isfinite(response)] = 100.
                if observed_exclusion is not None:
                    # Do not let an arm-covered patch vote as the named piece.
                    occlusion = cv2.matchTemplate(observed_exclusion.astype(np.float32),
                        mask.astype(np.float32),cv2.TM_CCORR)/max(1,int(mask.sum()))
                    response[occlusion>.25] = 100.
                iy,ix = np.unravel_index(np.argmin(response),response.shape)
                values.append(float(response[iy,ix])); offsets.append([int(ix-pad),int(iy-pad)])
            losses.append(values); shifts.append(offsets); names.append(view.name)
    finally:
        renderer.close()
    if len(losses)<3: return None
    loss = np.asarray(losses)
    # Fit a common yaw across views, allowing only bounded image translation.
    aggregate = np.median(loss,axis=0)
    best = int(np.argmin(aggregate)); axis = float(angles[best]%180)
    support = (loss[:,best]<=.25)&(loss[:,best]<=loss.min(axis=1)+.04)
    count = int(support.sum())
    alternative = [float(aggregate[i]) for i,yaw in enumerate(angles)
                   if axis_distance_degrees(yaw,axis)>=30]
    separation = max(0.,min(alternative,default=100.)-float(aggregate[best]))
    confidence = float(min(.99,count/3*min(1.,separation/.04)))
    close = [axis_distance_degrees(yaw,axis) for i,yaw in enumerate(angles)
             if aggregate[i]<=aggregate[best]+.02]
    leave_out = []
    for omitted in range(len(loss)):
        candidate = int(np.argmin(np.median(np.delete(loss,omitted,axis=0),axis=0)))
        leave_out.append(float(angles[candidate]%180))
    sigma = max(5.,max(close,default=90.),
                max((axis_distance_degrees(yaw,axis) for yaw in leave_out),default=90.))
    diagnostics = {'views':names,'best_yaw_deg':float(angles[best]),
        'axis_deg':axis,'supporting_views':[names[i] for i in np.flatnonzero(support)],
        'per_view_best_deg':[float(angles[i]%180) for i in loss.argmin(axis=1)],
        'leave_one_out_axis_deg':leave_out,'alternative_loss_gap':separation,
        'angles_deg':angles.tolist(),'aggregate_losses':aggregate.tolist(),
        'fit_offsets_px':[shifts[i][best] for i in range(len(names))]}
    points = []
    for i in np.flatnonzero(support):
        cam = next(view.calibration for view in views if view.name == names[i])
        pixel = cam.project([*xy,board_z+.5*(hi-lo)])
        shift = shifts[i][best]
        point = cam.unproject_to_plane((pixel[0]+shift[0],pixel[1]+shift[1]),
                                       board_z+.5*(hi-lo))
        if point is not None: points.append(point)
    refined = np.median(points,axis=0) if points else None
    xy_sigma = (max(.0005,float(np.median(np.linalg.norm(np.asarray(points)-refined,axis=1))))
                if len(points)>=3 else float('inf'))
    diagnostics['image_fitted_xy'] = None if refined is None else refined.tolist()
    diagnostics['image_fitted_xy_sigma_m'] = xy_sigma
    return AxisEstimate(math.radians(axis),math.radians(sigma),confidence,count,
                        float(aggregate[best]),diagnostics,
                        None if refined is None else tuple(map(float,refined)),xy_sigma)
