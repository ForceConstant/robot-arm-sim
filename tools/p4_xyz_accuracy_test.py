#!/usr/bin/env python3
"""Verify absolute xyz lands the FK tip where commanded. One test move, read-only after.
Command tip to a safe central point at z=60, wait for settle, report commanded vs actual."""
import json, time, urllib.request, urllib.error
CONSOLE='http://192.168.1.171:8080'; BRIDGE='http://192.168.1.167:8100'
def post(p,body,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(body).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:150]}
def get(u,t=8):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)

TARGET=(0.0,40.0,60.0)
st=get(BRIDGE+'/state')
print('start mode',st['mode'],'fault',st['fault'],'tip',get(CONSOLE+'/keep-in')['tip_mm'],flush=True)
if st['mode']!='REMOTE': print('->REMOTE',post('/mode',{'mode':'REMOTE'}));time.sleep(0.6)
body={'mode':'xyz','x_mm':TARGET[0],'y_mm':TARGET[1],'z_mm':TARGET[2],'jaw':100}
dr=post('/move',{**body,'dry_run':True})
print('DRY detail',dr.get('detail'),'poserr',dr.get('requested',{}).get('position_error_mm'),
      'delta',dr.get('max_joint_delta_deg'),flush=True)
if dr.get('detail'): print('ABORT dry',flush=True);post('/mode',{'mode':'HOLD'});raise SystemExit
print('EXEC',body,flush=True)
r=post('/move',body,t=90)
print('resp joints',r.get('joints'),'error_deg',r.get('error_deg'),'detail',r.get('detail'),flush=True)
for k in range(6):
    time.sleep(1.0)
    tip=get(CONSOLE+'/keep-in')['tip_mm']
    j=get(BRIDGE+'/state')
    err=[round(tip[i]-TARGET[i],1) for i in range(3)]
    print(f'  t+{k+1}s tip {[round(v,1) for v in tip]} err {err} fault {j["fault"]} mode {j["mode"]}',flush=True)
print('COMMANDED',TARGET,flush=True)
post('/mode',{'mode':'HOLD'})
print('HOLD',flush=True)
