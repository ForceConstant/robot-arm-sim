"""Read the leader arm's present joint positions. READ ONLY.

Opens the leader serial bus, sync-reads Present_Position (normalised with the
leader's own calibration) and disconnects.  It never writes a register, never
sets torque, never touches the follower — the leader is a passive teaching arm
and the bridge is in REMOTE, so the port is free.

Prints one JSON object: {"ok": true, "joints": {...}, "calibration": "..."}.
"""
from __future__ import annotations

import json
import sys

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.motors_bus import MotorCalibration

PORT = ("/dev/serial/by-id/"
        "usb-1a86_USB_Single_Serial_5C93131956-if00")
CAL = "so-arm-101"
DIR = "/home/pi/.cache/huggingface/lerobot/calibration/teleoperators/so_leader"

MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
    "gripper": Motor(6, "sts3215", MotorNormMode.DEGREES),
}


def main() -> int:
    from pathlib import Path

    calibration = json.loads((Path(DIR) / f"{CAL}.json").read_text())
    calibration = {name: MotorCalibration(**entry)
                   for name, entry in calibration.items()}
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    try:
        bus.connect()
        values = bus.sync_read("Present_Position", normalize=True)
    finally:
        try:
            bus.disconnect()
        except Exception:                            # noqa: BLE001
            pass
    print(json.dumps({"ok": True, "calibration": CAL,
                      "joints": {name: round(float(value), 2)
                                 for name, value in values.items()}}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
