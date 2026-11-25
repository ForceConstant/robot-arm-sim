#!/usr/bin/env python3
"""READ-ONLY dump of the STS3215 control registers that shape the deadband.

Reads CW/CCW_Dead_Zone, Minimum_Startup_Force, the PID coefficients,
Acceleration, Torque_Limit, Operating_Mode and Angular_Resolution for ids 1-6.
Writes nothing. Run with the bridge stopped (it owns the port).
"""
import json
import sys
import time

from scservo_sdk import PacketHandler, PortHandler, COMM_SUCCESS

BAUD = 1000000
REGS = [
    ("P_Coefficient", 21, 1),
    ("D_Coefficient", 22, 1),
    ("I_Coefficient", 23, 1),
    ("Minimum_Startup_Force", 24, 2),
    ("CW_Dead_Zone", 26, 1),
    ("CCW_Dead_Zone", 27, 1),
    ("Protection_Current", 28, 2),
    ("Angular_Resolution", 30, 1),
    ("Operating_Mode", 33, 1),
    ("Protective_Torque", 34, 1),
    ("Overload_Torque", 36, 1),
    ("Velocity_P", 37, 1),
    ("Velocity_I", 39, 1),
    ("Torque_Enable", 40, 1),
    ("Acceleration", 41, 1),
    ("Torque_Limit", 48, 2),
]

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
ids = [int(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2 else [1, 2, 3, 4, 5, 6]

handler = PortHandler(port)
if not handler.openPort():
    raise SystemExit(f"cannot open {port}")
if not handler.setBaudRate(BAUD):
    raise SystemExit(f"cannot set baud on {port}")
packet = PacketHandler(0)

out = {"port": port, "servos": {}}
for servo in ids:
    model, comm, _err = packet.ping(handler, servo)
    if comm != COMM_SUCCESS:
        out["servos"][servo] = {"present": False}
        continue
    entry = {"present": True, "model": model, "registers": {}}
    for name, address, size in REGS:
        if size == 1:
            value, comm, _err = packet.read1ByteTxRx(handler, servo, address)
        else:
            value, comm, _err = packet.read2ByteTxRx(handler, servo, address)
        entry["registers"][name] = value if comm == COMM_SUCCESS else None
        time.sleep(0.01)
    entry["dead_zone_deg"] = {
        "cw": round(entry["registers"]["CW_Dead_Zone"] * 360.0 / 4096.0, 3)
        if entry["registers"].get("CW_Dead_Zone") is not None else None,
        "ccw": round(entry["registers"]["CCW_Dead_Zone"] * 360.0 / 4096.0, 3)
        if entry["registers"].get("CCW_Dead_Zone") is not None else None,
    }
    out["servos"][servo] = entry

handler.closePort()
print(json.dumps(out, indent=1))
