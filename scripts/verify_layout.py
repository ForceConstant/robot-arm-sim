"""Run the deployed layout-v1 cases and verify the selected geometry is live."""
import argparse,json,time
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from verify_deploy import call,wait_ready,download

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base',default='http://192.168.1.138:8098')
    ap.add_argument('--version',required=True)
    ap.add_argument('--gap',type=float,required=True)
    ap.add_argument('--riser',type=float,required=True)
    ap.add_argument('--output',type=Path,default=ROOT/'.private/live-layout.json')
    args=ap.parse_args()
    status=wait_ready(args.base,args.version)
    layout=status.get('layout') or {}
    assert abs(layout['board_edge_to_base_frame_mm']-args.gap)<1e-6
    assert abs(layout['base_riser_mm']-args.riser)<1e-6
    assert abs(layout['board_thickness_mm']-3.2)<1e-6
    assert status['gripper']['source_revision']=='4dca0986f24304c301118aa4'
    definitions=call(args.base,'/api/suite?name=layout')
    assert definitions['version']=='layout-v1' and len(definitions['cases'])==16
    call(args.base,'/api/control',{'action':'speed','speed':8})
    call(args.base,'/api/control',{'action':'suite','suite':'layout'})
    started=time.monotonic();last=-1
    while time.monotonic()-started<900:
        status=call(args.base,'/api/status');suite=status.get('suite') or {}
        assert status['version']==args.version
        assert suite.get('version')=='layout-v1'
        if suite.get('index')!=last:
            last=suite.get('index');print('Layout cases finished:',last,'/ 16',flush=True)
        if suite.get('phase')=='complete':break
        if suite.get('phase')=='stopped':raise RuntimeError('Layout suite stopped')
        time.sleep(3)
    else:raise RuntimeError('Layout suite timed out')
    results=suite['results'];assert len(results)==16
    receipt=dict(version=status['version'],renderer=status['renderer'],layout=layout,
                 suite=suite,passed=sum(r['success'] for r in results),total=len(results),
                 elapsed_s=round(time.monotonic()-started,1))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(receipt,indent=2)+'\n')
    for row in results:
        print(row['case'],row['success'],row['error_mm'],flush=True)
        if not row['success'] and row.get('recording'):
            (args.output.parent/f"layout-failure-{row['recording']}.zip").write_bytes(download(args.base,f"/api/incidents/{row['recording']}.zip"))
    print(f"{receipt['passed']}/{receipt['total']} layout cases passed",flush=True)

if __name__=='__main__':main()
