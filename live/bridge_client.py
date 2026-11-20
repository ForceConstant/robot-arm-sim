"""HTTP adapters for the Pi bridge: the transport the controller expects.

Kept separate from the controller so the same orchestration code runs against a
simulator-backed fake in tests and against the real arm in the demo.  Nothing
here decides motion policy; it only moves bytes.
"""
from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Sequence
import urllib.error
import urllib.request

import cv2
import numpy as np

from scan_controller import Frame


class BridgeError(RuntimeError):
    pass


class HttpBridge:
    """`/state` and `/command` against the Pi bridge."""

    def __init__(self, base_url: str = "http://127.0.0.1:8100", *,
                 token: str | None = None, token_file: str | Path | None = None,
                 timeout: float = 4.0) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout = float(timeout)
        self._token = token
        self._token_file = Path(token_file) if token_file else None

    # ------------------------------------------------------------------ token
    @property
    def token(self) -> str:
        if self._token:
            return self._token
        if self._token_file and self._token_file.exists():
            self._token = self._token_file.read_text().strip()
            return self._token
        raise BridgeError("no bridge token configured")

    # --------------------------------------------------------------- requests
    def _request(self, path: str, payload: dict | None = None,
                 timeout: float | None = None, auth: bool | None = None) -> dict:
        data = None if payload is None else json.dumps(payload).encode()
        headers = {"Content-Type": "application/json"} if data else {}
        # A GET can need the token too; default keeps the old behaviour of
        # authenticating exactly the requests that carry a body.
        use_token = (data is not None) if auth is None else auth
        if use_token:
            headers["x-bridge-token"] = self.token
        request = urllib.request.Request(self.base_url + path, data=data,
                                         headers=headers)
        try:
            with urllib.request.urlopen(
                    request, timeout=self.timeout if timeout is None else timeout) as response:
                return json.loads(response.read().decode() or "{}")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode(errors="replace")
            raise BridgeError(f"{path} -> {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise BridgeError(f"{path} unreachable: {exc}") from exc

    def state(self) -> dict:
        return self._request("/state")

    def camera_controls(self) -> dict:
        return self._request("/camera/controls", auth=True)

    def set_camera_controls(self, controls: dict, save: bool = False) -> dict:
        return self._request("/camera/controls",
                             {"controls": controls, "save": bool(save)},
                             timeout=20.0)

    def arm(self) -> dict:
        return self._request("/arm", {"confirm": True})

    def set_mode(self, mode: str) -> dict:
        return self._request("/mode", {"mode": mode})

    def estop(self) -> dict:
        return self._request("/estop", {})

    def wrist_zero(self, span_deg: float = 90.0, dry_run: bool = False) -> dict:
        """Persist the current wrist pose as servo zero (EEPROM write)."""
        return self._request("/calibration/wrist-zero",
                             {"span_deg": float(span_deg),
                              "dry_run": bool(dry_run)}, timeout=25.0)

    def command(self, joints: Sequence[float], *, seq: int, ttl_ms: int,
                speed_scale: float) -> dict:
        return self._request("/command", {
            "v": 1, "seq": int(seq), "ttl_ms": int(ttl_ms),
            "joints": [float(v) for v in joints],
            "speed_scale": float(speed_scale)})


class HttpFrames:
    """`/capture`: one JPEG plus the joint snapshot that came with it."""

    def __init__(self, base_url: str = "http://127.0.0.1:8100", *,
                 token: str | None = None, token_file: str | Path | None = None,
                 timeout: float = 10.0) -> None:
        self.bridge = HttpBridge(base_url, token=token, token_file=token_file,
                                 timeout=timeout)

    def capture(self) -> Frame:
        payload = self.bridge._request("/capture")
        try:
            raw = base64.b64decode(payload["jpeg_b64"])
        except (KeyError, ValueError) as exc:
            raise BridgeError("capture response has no usable jpeg") from exc
        image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise BridgeError("capture jpeg did not decode")
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex", "wrist_roll", "gripper")
        joints = tuple(float(payload["joints"][name]) for name in names)
        return Frame(image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                     joints_deg=joints,
                     timestamp=float(payload.get("frame_ts", 0.0)))
