"""Offline action-boundary checks. No transport or robot access."""
import math


def verdict(actions, *, request_id, expected_id, captured_at, now, deadline=1.0):
    if request_id != expected_id:
        return 'hold_replan:request_mismatch'
    if not all(math.isfinite(v) for v in (captured_at, now, deadline)) or deadline <= 0 or now < captured_at or now-captured_at > deadline:
        return 'hold_replan:stale'
    if len(actions) != 30 or any(len(row) != 6 or not all(math.isfinite(v) for v in row) for row in actions):
        return 'hold_replan:malformed'
    return 'timely_finite_chunk'  # NOT physical limits, trajectory safety or policy quality acceptance.


def faults():
    chunk = [[0.0]*6 for _ in range(30)]
    cases = {
        'on_time': verdict(chunk, request_id=1, expected_id=1, captured_at=0, now=.1),
        'late': verdict(chunk, request_id=1, expected_id=1, captured_at=0, now=1.01),
        'reordered': verdict(chunk, request_id=0, expected_id=1, captured_at=0, now=.1),
        'malformed': verdict([[float('nan')]*6]*30, request_id=1, expected_id=1, captured_at=0, now=.1),
        'clock_reversed': verdict(chunk, request_id=1, expected_id=1, captured_at=1, now=0),
    }
    expected = {'on_time':'timely_finite_chunk','late':'hold_replan:stale','reordered':'hold_replan:request_mismatch',
                'malformed':'hold_replan:malformed','clock_reversed':'hold_replan:stale'}
    if cases != expected:
        raise RuntimeError('fault gate failed')
    return cases


def motion_verdict(actions, *, request_id, expected_id, captured_at, now, capture_verified):
    """Fail closed when a real capture timestamp/synchronization is unavailable.

    The runtime benchmark separately measures GET-to-decode latency. A fresh
    HTTP retrieval is NOT proof that the cached camera image is newly captured.
    """
    if not capture_verified or captured_at is None:
        return 'hold_replan:unverified_capture_age'
    return verdict(actions,request_id=request_id,expected_id=expected_id,captured_at=captured_at,now=now)
