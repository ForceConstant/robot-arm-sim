#!/usr/bin/env python3
"""Dry-run probe of reachable look_at viewpoints around a board point.
No motion. Finds candidate camera angles for the multi-view capture ring."""
import json, urllib.request, urllib.error
CONSOLE='http://192.168.1.171:8080'
def post(body,t=20):
    req=urllib.request.Request(CONSOLE+'/move',data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code}
print('look_at around (14,14,0): az/pit/dist -> detail|poserr|roll')
for az in (135,160,180,200,225):
    for pit in (30,45,60):
        for dist in (160,200,240):
            b={'mode':'look_at','x_mm':14,'y_mm':14,'z_mm':0,'distance_mm':dist,
               'azimuth_deg':az,'pitch_deg':pit,'roll_deg':0,'jaw':100,'dry_run':True}
            r=post(b)
            det=r.get('detail')
            if not det:
                print(f'  az={az} pit={pit} d={dist}: OK poserr={r.get("requested",{}).get("position_error_mm")} delta={r.get("max_joint_delta_deg")}')
