#!/usr/bin/env python3
"""Dry-run probe: which board-axis rel directions are feasible from current pose
without an IK branch flip (max_joint_delta_deg below threshold)? No motion."""
import json, urllib.request, urllib.error
CONSOLE='http://192.168.1.171:8080'
def post(p,body,t=20):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code}
steps=[(5,0),(-5,0),(0,5),(0,-5),(5,-5),(-5,-5),(5,5),(-5,5),(4,-10),(-4,10),(10,0),(-10,0),(0,10),(0,-10)]
print('probe rel dx,dy -> max_joint_delta_deg / pos_err')
for dx,dy in steps:
    r=post('/move',{'mode':'rel','dx_mm':dx,'dy_mm':dy,'dz_mm':0,'dry_run':True})
    md=r.get('max_joint_delta_deg'); pe=r.get('requested',{}).get('position_error_mm')
    flag='OK' if (md is not None and md<=15) else 'FLIP'
    print(f'  dx={dx:+3d} dy={dy:+3d} -> delta={md} poserr={pe}  {flag}')
