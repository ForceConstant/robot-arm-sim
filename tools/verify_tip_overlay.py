#!/usr/bin/env python3
"""Verify the detection overlay draws the cyan tip marker at the saved pixel."""
import sys, json
import cv2
sys.path.insert(0,'/home/brian/robot-arm-sim');sys.path.insert(0,'/home/brian/robot-arm-sim/tools')
from p4_irl_servo_detect import grab_frame
from p4_detect_overlay import detect, saved_tip_pixel

print('saved_tip_pixel() ->', saved_tip_pixel())
f=grab_frame(timeout=6)
ann,info=detect(f)
print('info ->', {k:info[k] for k in info if k in ('markers','rms','tip_pixel','tip_source','centroid_mm','offset_mm')})
cv2.imwrite('/tmp/verify_tip_overlay.jpg',ann)
print('saved frame')
