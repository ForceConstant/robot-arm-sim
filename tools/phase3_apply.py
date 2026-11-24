import sys, json, shutil, datetime
sys.path.insert(0, "/home/brian/robot-arm-sim")
sys.path.insert(0, "/home/brian/robot-arm-sim/live")
sys.path.insert(0, "/home/brian/robot-arm-sim/tools")
import numpy as np
import phase3_pair as P
from chess_sim.wrist_calibration import WristCalibration
from live.kinematics import ArmKinematics, frame_at

ROOT = "/home/brian/robot-arm-sim/"
cfg = json.load(open(ROOT + "live/live_config.json"))
cal = WristCalibration.load(ROOT + cfg["calibration"])
b = json.load(open(ROOT + "live/board.json"))
board = frame_at(b["x_mm"], b["y_mm"], b["z_mm"], b["yaw_deg"])
kin = ArmKinematics(cal, {}, board)
X0 = np.asarray(cal.camera_in_tool, float)
views = P.load([ROOT + "live/captures_phase3", ROOT + "live/captures_handeye_all"], cal)
for v in views:
    v["fk"] = kin.fk(v["joints"])
X, B, _ = P.alternate(views, kin, board, X0, iters=60)
e = P.cam_error(views, X, B)
e0 = P.cam_error(views, X0, board)
print("mount t", np.round(X[:3, 3] * 1000, 3).tolist())
print("mount R", np.round(X[:3, :3], 6).tolist())
print("board t", np.round(B[:3, 3] * 1000, 3).tolist())
print(f"camera-position error: before {e0.mean():.2f} after {e.mean():.2f} mm")

path = ROOT + cfg["calibration"]
stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
shutil.copy(path, path + f".bak-phase3-{stamp}")
data = json.load(open(path))
data["camera_in_tool"] = np.round(X, 9).tolist()
data["phase3_note"] = ("camera_in_tool updated by Phase 3 (2026-11-24) from a "
                       "5-roll absolute marker solve; rotation corrected 22.1 deg, "
                       "translation delta (mm) " +
                       str(np.round((X[:3, 3] - X0[:3, 3]) * 1000, 2).tolist()))
json.dump(data, open(path, "w"), indent=2)
open(path, "a").write("\n")
print("wrote", path, "backup", path + f".bak-phase3-{stamp}")

# re-load and confirm
cal2 = WristCalibration.load(path)
print("reloaded ok; t", np.round(np.asarray(cal2.camera_in_tool)[:3, 3] * 1000, 3).tolist())
