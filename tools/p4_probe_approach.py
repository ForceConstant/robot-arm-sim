#!/usr/bin/env python3
"""Dry-run absolute xyz moves toward the piece at several approach heights;
report feasibility, pos_err and joint swing. NO motion."""
import json, urllib.request, urllib.error
CONSOLE='http://192.168.1.171:8080'
def post(p,body,t=20):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:120]}
# piece board ~ (4,11); approach points at increasing standoff and slight offsets
pts=[(4,11,16),(4,11,40),(4,11,60),(4,11,80),(4,30,60),(4,40,60),(-2,36,30),(-2,36,60),
     (14,14,60),(14,14,80),(0,20,60),(0,20,80),(4,11,100)]
print('abs xyz -> detail | pos_err | max_delta')
for x,y,z in pts:
    r=post('/move',{'mode':'xyz','x_mm':x,'y_mm':y,'z_mm':z,'jaw':99,'dry_run':True})
    print(f'  ({x:3d},{y:3d},{z:3d}) -> {r.get("detail") or "OK"} '
          f'poserr={r.get("requested",{}).get("position_error_mm")} '
          f'delta={r.get("max_joint_delta_deg")}')
