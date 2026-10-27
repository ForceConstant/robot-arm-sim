"""Public synthetic adapter test plus integration checks using an explicitly supplied private pack."""
import numpy as np
import mujoco
import pytest

from chess_sim import gripper, EnvConfig, ControlConfig, AppearanceConfig
from chess_sim import assets
from chess_sim.cad_tool import install


def test_external_tool_adapter_retains_arm_and_replaces_only_tool(tmp_path):
    # Deliberately synthetic dimensions; no private CAD is a test fixture.
    (tmp_path / 'toy.xml').write_text('''<mujoco>
      <worldbody><body name="tool_base"><geom type="box" size=".01 .01 .01"/>
        <body name="drive"><joint name="drive_joint" type="hinge" range="-1 1"/><geom size=".005"/>
        </body><body name="left" pos="0 -.015 0"><joint name="left_slide" type="slide" axis="0 1 0" range="-.01 .01"/><geom size=".003"/>
        </body><body name="right" pos="0 .015 0"><joint name="right_slide" type="slide" axis="0 -1 0" range="-.01 .01"/><geom size=".003"/></body>
      </body></worldbody><actuator><position name="drive_joint" joint="drive_joint" kp="1"/></actuator>
      <equality><joint joint1="left_slide" joint2="drive_joint" polycoef="0 .01 0 0 0"/>
      <joint joint1="right_slide" joint2="drive_joint" polycoef="0 .01 0 0 0"/></equality></mujoco>''')
    p = dict(model_file='toy.xml', base_body='tool_base', pinion_joint='drive_joint',
             pinion_body='drive', left_body='left', right_body='right', jaw_joints=['left_slide', 'right_slide'],
             mount_position=[0, 0, 0], mount_rotation=np.eye(3).tolist(),
             tcp_position=[0, 0, -.1], servo_position=[0, 0, -.02], servo_rotation=np.eye(3).tolist(),
             servo_mass_kg=.05, ik_boxes=[dict(pos=[0, 0, -.05], size=[.02, .03, .05])])
    arm = mujoco.MjSpec.from_file(assets.SO101_XML)
    install(arm, p, tmp_path)
    model = arm.compile()
    assert model.njnt == 8 and model.nu == 6 and model.neq == 2
    assert model.joint('wrist_roll').id >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'moving_jaw_so101_v1') == -1
    assert model.actuator('gripper').trnid[0] == model.joint('gripper').id
    assert set(model.eq_obj2id) == {model.joint('gripper').id}


@pytest.fixture(scope='module')
def custom_env():
    if not gripper.CUSTOM:
        pytest.skip('Private CAD integration requires ROBOT_GRIPPER=cad_parallel and an explicit asset pack')
    from chess_sim.dual import DualArmEnv
    e = DualArmEnv(EnvConfig(control=ControlConfig(cameras=()),
                            appearance=AppearanceConfig(camera_mast=False)))
    e.reset()
    yield e
    e.close()


def test_custom_both_arms_keep_five_joints_and_separate_couplings(custom_env):
    e = custom_env
    assert e.model.nu == 12 and e.model.neq == 4
    for prefix in ('so101:', 'black:'):
        for joint in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'):
            assert e.model.joint(prefix + joint).id >= 0
        pinion = e.model.joint(prefix + 'gripper').id
        dependent = {e.model.joint(prefix + n).id for n in gripper.PROFILE['jaw_joints']}
        eqs = [i for i in range(e.model.neq) if e.model.eq_obj2id[i] == pinion]
        assert {int(e.model.eq_obj1id[i]) for i in eqs} == dependent
        assert np.allclose(e.model.eq_data[eqs, 1], gripper.PROFILE['ratio_m_per_rad'])
        assert mujoco.mj_name2id(e.model, mujoco.mjtObj.mjOBJ_BODY, prefix + 'moving_jaw_so101_v1') == -1
    assert np.allclose(gripper.pocket_offset(.02), gripper.pocket_offset(.04))


def test_custom_actuator_cycles_are_independent_bounded_and_contact_based(custom_env):
    e = custom_env
    e.reset()
    ratio = gripper.PROFILE['ratio_m_per_rad']
    for side in ('white', 'black'):
        e.select_arm(side)
        prefix = 'so101:' if side == 'white' else 'black:'
        other = 'black:' if side == 'white' else 'so101:'
        before = [float(e.data.qpos[e.model.joint(other + n).qposadr[0]]) for n in gripper.PROFILE['jaw_joints']]
        observed = []
        for target in (gripper.JOINT_MIN, gripper.JOINT_MAX, gripper.gap_to_angle(.04)):
            action = e.home_pose()
            start = e.arm_joint_positions()[5]
            for fraction in np.r_[np.linspace(0, 1, 30), np.ones(15)]:
                action[5] = start + (target - start) * fraction
                e.apply_action(action)
                angle = float(e.arm_joint_positions()[5])
                jaws = np.array([e.data.qpos[e.model.joint(prefix + n).qposadr[0]] for n in gripper.PROFILE['jaw_joints']])
                assert np.max(np.abs(jaws - ratio * angle)) < 1e-4
                observed.append(gripper.GAP_AT_ZERO - jaws.sum())
                assert np.isfinite(e.data.qpos).all() and not np.any(e.data.warning.number)
            # Measure the distal solid envelopes in the mounted TCP frame;
            # this catches incorrect mount axes even when equalities are valid.
            site = e.model.site(prefix + 'gripperframe').id
            rotation = e.data.site_xmat[site].reshape(3, 3)
            point_sets = []
            for name in ('cad:parallel_jaw_left', 'cad:parallel_jaw_right'):
                bid = e.model.body(prefix + name).id
                points = []
                for gid in np.flatnonzero((e.model.geom_bodyid == bid) & (e.model.geom_group == 3)):
                    mid = e.model.geom_dataid[gid]
                    start, count = e.model.mesh_vertadr[mid], e.model.mesh_vertnum[mid]
                    vertices = e.model.mesh_vert[start:start + count]
                    world = vertices @ e.data.geom_xmat[gid].reshape(3, 3).T + e.data.geom_xpos[gid]
                    local = (world - e.data.site_xpos[site]) @ rotation
                    points.extend(local[local[:, 0] > -.001])
                point_sets.append(np.asarray(points))
            measured_gap = point_sets[1][:, 2].min() - point_sets[0][:, 2].max()
            assert abs(measured_gap - gripper.angle_to_gap(float(e.arm_joint_positions()[5]))) < .0001
        after = [float(e.data.qpos[e.model.joint(other + n).qposadr[0]]) for n in gripper.PROFILE['jaw_joints']]
        assert np.max(np.abs(np.array(after) - before)) < .0002
        assert max(observed) - min(observed) > .9 * np.ptp(gripper.PROFILE['aperture_range_m'])
        assert np.all(e.model.actuator_forcelimited[e._actuators])
    assert not any(e.model.eq_type == mujoco.mjtEq.mjEQ_WELD), 'No invisible attachment constraints'
    assert len(e._square_slot) == 32
