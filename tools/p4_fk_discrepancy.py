#!/usr/bin/env python3
"""Check whether app FK (from live-reported joints) matches keep-in tip and
whether the bridge's reported joints match the last commanded joints."""
import json, urllib.request, sys
import numpy as np
sys.path.insert(0,'/home/brian/robot-arm-sim/live')
from kinematics import ArmKinematics, JOINTS
from tip_frame import TIP_IN_CHAIN
k=ArmKinematics(".private/phase11d/calibration.json")
st=json.load(urllib.request.urlopen('http://192.168.1.167:8100/state',timeout=8))
j=np.array([st['joints'][n] for n in JOINTS])
tip_app=(k.to_board((k.fk(j)@TIP_IN_CHAIN)[:3,3])*1000)
keep=json.load(urllib.request.urlopen('http://192.168.1.171:8080/keep-in',timeout=8))
pose=json.load(urllib.request.urlopen('http://192.168.1.171:8080/pose',timeout=8))
print('bridge joints (deg)',{n:round(st['joints'][n],2) for n in JOINTS})
print('app FK tip from bridge joints (mm)',tip_app.round(1).tolist())
print('keep-in tip_mm',keep['tip_mm'])
print('pose tip_mm',pose.get('tip_mm'))
print('pose tool_mm',pose.get('tool_mm'))
