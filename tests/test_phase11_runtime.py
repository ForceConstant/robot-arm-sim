"""Critical offline boundaries for Phase 11; no torch or hardware required."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from tools.export_act_pilot import goal


def load_gate():
    path = Path(__file__).resolve().parents[1]/'training/probes/action_chunk_gate.py'
    spec = importlib.util.spec_from_file_location('action_chunk_gate',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_goal_is_instruction_only_in_board_metres():
    # Source/destination centres, not piece poses or arm-root world coordinates.
    assert np.allclose(goal('a1h8'),[-.098,-.098,0,.098,.098,0])
    assert np.allclose(goal('e2e4'),[.014,-.070,0,.014,-.014,0])
    with pytest.raises(ValueError):
        goal('e7e8q')  # promotion is not silently truncated


@pytest.mark.parametrize('kind',['late','request','nonfinite','short','clock'])
def test_chunk_faults_never_admit_actions(kind):
    module = load_gate()
    chunk = [[0.0]*6 for _ in range(30)]
    kwargs = dict(request_id=4,expected_id=4,captured_at=10,now=10.2)
    if kind=='late': kwargs['now']=11.001
    if kind=='request': kwargs['request_id']=3
    if kind=='nonfinite': chunk[29][5]=float('nan')
    if kind=='short': chunk.pop()
    if kind=='clock': kwargs['now']=9
    assert module.verdict(chunk,**kwargs).startswith('hold_replan:')


def test_finite_chunk_does_not_claim_motion_safety():
    module = load_gate()
    # Timing/shape gate deliberately makes no kinematic/physical-safety claim.
    assert module.verdict([[100.0]*6]*30,request_id=1,expected_id=1,captured_at=0,now=.1)=='timely_finite_chunk'
    assert len(module.faults())==5


def test_fresh_http_request_cannot_certify_camera_capture_age():
    module = load_gate()
    args=dict(request_id=1,expected_id=1,captured_at=100,now=100.01)
    assert module.motion_verdict([[0.0]*6]*30,**args,capture_verified=False)=='hold_replan:unverified_capture_age'
    assert module.motion_verdict([[0.0]*6]*30,**args,capture_verified=True)=='timely_finite_chunk'
