#!/usr/bin/env python3
"""Lower the CLOSED jaw tip toward the board in small rel steps ( Brian 2026-11-22).

Purpose: at z~2mm the closed-tip contact point is unambiguous (no z-parallax),
so we can verify where the true finger tip is and fix the cyan marker.

Safety:
  - jaws already closed (verified before start)
  - each rel dz step dry-run first; abort if max joint delta > 12 deg
  - stop on fault, mode change, or 409
  - absolute z floor 2mm; never positive-penetrate
  - prints every leg before executing (standing motion rule)
  - rolls branch preserved (wrist_roll ~68)
"""
import json,sys,time,urllib.request,urllib.error
CONSOLE='http://192.168.1.171:8080';BRIDGE='http://192.168.1.167:8100'
def get(u,t=10):
    with urllib.request.urlopen(u,timeout=t) as r:return json.load(r)
def post(p,b,t=60):
    req=urllib.request.Request(CONSOLE+p,data=json.dumps(b).encode(),method='POST',headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,timeout=t) as r:return json.load(r)
    except urllib.error.HTTPError as e:return {'http_error':e.code,'detail':e.read().decode()[:200]}

def main():
    st=get(BRIDGE+'/state')
    assert st['fault'] is None, st['fault']
    assert st['mode']=='REMOTE', st['mode']
    jaw=st['joints']['gripper']; assert jaw<25, f'jaws not closed: {jaw}'
    tip=get(CONSOLE+'/keep-in')['tip_mm']
    roll0=st['joints']['wrist_roll']
    print(f'START mode={st["mode"]} jaw={jaw:.1f} tip={tip} roll={roll0:.2f}',flush=True)
    z=tip[2]
    step=4.0
    while z>2.5:
        dz=-min(step,z-2.0)
        body={'mode':'rel','dz_mm':round(dz,2)}
        dr=post('/move',{**body,'dry_run':True},t=25)
        md=dr.get('max_joint_delta_deg')
        if dr.get('http_error') or md is None or md>12:
            print(f'ABORT step {body}: max_delta={md} dry={dr}',flush=True);break
        print(f'LEG rel dz={dz:+.2f}mm (max_delta={md:.1f}deg) -> EXEC',flush=True)
        r=post('/move',body,t=60)
        if r.get('http_error'):
            print('ABORT exec',r,flush=True);break
        time.sleep(0.9)
        st=get(BRIDGE+'/state')
        tip=get(CONSOLE+'/keep-in')['tip_mm']
        z=tip[2]
        print(f'  after: jaw={st["joints"]["gripper"]:.1f} roll={st["joints"]["wrist_roll"]:.2f} '
              f'tip={tip} fault={st["fault"]}',flush=True)
        if st['fault'] or st['mode']!='REMOTE' or abs(st['joints']['wrist_roll']-roll0)>3:
            print('ABORT guard',flush=True);break
    print('DONE z=',z,flush=True)
    open('/tmp/lower_tip_result.json','w').write(json.dumps({'tip_final':tip,'z':z,'jaw':jaw}))

if __name__=='__main__':main()
