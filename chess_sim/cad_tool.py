"""Mount an external private CAD tool pack; never bundle private CAD in public source."""
from pathlib import Path
import numpy as np
import mujoco


def quat(matrix):
    result = np.empty(4)
    mujoco.mju_mat2Quat(result, np.asarray(matrix, dtype=float).reshape(-1))
    return result


def install(arm, profile, asset_root):
    root = Path(asset_root)
    # Keep all five arm joints, including the wrist_roll carried by this body.
    wrist = arm.body('gripper')
    arm.delete(arm.actuator('gripper'))
    arm.delete(arm.body('moving_jaw_so101_v1'))
    for geom in list(wrist.geoms):
        arm.delete(geom)
    wrist.mass = .001  # estimated output adaptor; CAD printed masses are in the attached model
    wrist.ipos = [0, 0, 0]
    wrist.fullinertia = [1e-7, 1e-7, 1e-7, 0, 0, 0]
    wrist.explicitinertial = True

    tool = mujoco.MjSpec.from_file(str(root / profile['model_file']))
    for name in ('timestep', 'integrator', 'cone', 'impratio'):
        setattr(tool.option, name, getattr(arm.option, name))
    tool.meshdir = str(root / 'assets')
    for mesh in tool.meshes:
        mesh.file = str((root / 'assets' / mesh.file).resolve())
    aliases = {profile['base_body']: 'parallel_base', profile['pinion_body']: 'parallel_pinion',
               profile['left_body']: 'parallel_jaw_left', profile['right_body']: 'parallel_jaw_right'}
    for old, new in aliases.items():
        tool.body(old).name = new
    for exclude in tool.excludes:
        exclude.bodyname1 = aliases.get(exclude.bodyname1, exclude.bodyname1)
        exclude.bodyname2 = aliases.get(exclude.bodyname2, exclude.bodyname2)
    original_joint = profile['pinion_joint']
    tool.joint(original_joint).name = 'gripper'
    for actuator in tool.actuators:
        if actuator.target == original_joint:
            actuator.target = 'gripper'
            actuator.name = 'gripper'
    for equality in tool.equalities:
        if equality.name1 == original_joint:
            equality.name1 = 'gripper'
        if equality.name2 == original_joint:
            equality.name2 = 'gripper'
        equality.solref = [.002, 1]
    frame = wrist.add_frame(pos=profile['mount_position'], quat=quat(profile['mount_rotation']))
    frame.attach_body(tool.body('parallel_base'), 'cad:', '')
    joint_aliases = {'cad:gripper': 'gripper', **{'cad:' + n: n for n in profile['jaw_joints']}}
    for old, new in joint_aliases.items():
        arm.joint(old).name = new
    actuator = arm.actuator('cad:gripper')
    actuator.name, actuator.target = 'gripper', 'gripper'
    for equality in arm.equalities:
        equality.name1 = joint_aliases.get(equality.name1, equality.name1)
        equality.name2 = joint_aliases.get(equality.name2, equality.name2)

    site = arm.site('gripperframe')
    site.pos = profile['tcp_position']
    # Retain the controller's +x approach / +z span convention.
    site.quat = [.7071067811865476, 0, .7071067811865476, 0]

    # Reuse one servo, not the old servo-plus-finger mass or a second actuator.
    servo = wrist.add_body(name='tool_servo', pos=profile['servo_position'], quat=quat(profile['servo_rotation']))
    servo.add_geom(name='tool_servo_visual', type=mujoco.mjtGeom.mjGEOM_MESH,
                   meshname='sts3215_03a_v1', material='sts3215_03a_v1_material',
                   contype=0, conaffinity=0, group=2)
    servo.add_geom(name='tool_servo_collision', type=mujoco.mjtGeom.mjGEOM_BOX,
                   size=[.0227, .0124, .0198], pos=[0, 0, .0004],
                   mass=profile['servo_mass_kg'], rgba=[.12, .12, .12, 1], group=3)
    for i, box in enumerate(profile['ik_boxes']):
        wrist.add_geom(name=f'clearance_proxy_{i}', type=mujoco.mjtGeom.mjGEOM_BOX,
                       pos=box['pos'], size=box['size'], contype=0, conaffinity=0, density=0, group=5)
