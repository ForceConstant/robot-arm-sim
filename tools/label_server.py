#!/usr/bin/env python3
"""Local labeling UI for the real piece dataset (2026-11-22).

Serves each clean captured view; the operator clicks the piece's BASE CENTRE.
Clicks are saved to labels_human.json. Progress tracked; can resume.
Standalone HTTP server (stdlib only) on port 8123, localhost-friendly.

    PYTHONPATH=. .venv/bin/python tools/label_server.py --dataset adapt1 --port 8123
"""
import argparse, json, os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

ROOT=Path('/home/brian/robot-arm-sim')

class State:
    def __init__(self, dataset):
        self.dir=ROOT/'.private/real_piece_views'/dataset
        self.mf=json.load(open(self.dir/'manifest.json'))
        self.lpath=self.dir/'labels_human.json'
        self.labels=json.load(open(self.lpath)) if self.lpath.exists() else {}
        # flat list of (key, image, piece, square, view, cam)
        self.items=[]
        for pl in self.mf['placements']:
            for v in pl['views']:
                self.items.append({'key':v['image'],
                    'image':v['image'],'piece':pl['piece'],'square':pl['square'],
                    'view':v['view'],'cam':v.get('cam')})
    def save(self):
        self.lpath.write_text(json.dumps(self.labels,indent=2)+'\n')

S=None

PAGE="""<!doctype html><html><head><meta charset=utf-8><title>Label piece base centres</title>
<style>
body{font:14px ui-sans-serif,system-ui;background:#111;color:#eee;margin:0}
header{padding:10px 16px;background:#1a2330;border-bottom:1px solid #2a3a4f}
.wrap{display:flex;gap:16px;padding:16px;align-items:flex-start}
img{max-width:640px;max-height:70vh;cursor:crosshair;border:1px solid #333}
.side{min-width:280px}
button{background:#26405c;color:#eef;border:1px solid #3a6288;padding:6px 10px;border-radius:4px;cursor:pointer;margin:2px}
button:hover{background:#31547a}
.done{color:#7fd4a0}.todo{color:#ffb4b4}
.kv{font:12px ui-monospace,monospace;color:#9fb6cc;margin:4px 0}
.list{max-height:70vh;overflow:auto;font:12px ui-monospace,monospace}
.list div{padding:2px 4px;cursor:pointer}
.list div.sel{background:#26405c}
</style></head><body>
<header><b id=hdr>Label base centres</b> <span id=prog></span></header>
<div class=wrap>
 <div><img id=img></div>
 <div class=side>
  <div class=kv id=meta></div>
  <div class=kv>Click the CENTRE OF THE PIECE BASE (where it touches the board).</div>
  <div class=kv id=cur></div>
  <button onclick=skip()>Skip</button>
  <div class=list id=list></div>
 </div>
</div>
<script>
let items=[],labels={},idx=0;
const el=id=>document.getElementById(id);
async function init(){
  const r=await fetch('/api/state');const d=await r.json();
  items=d.items;labels=d.labels;buildList();show(0);
}
function buildList(){
  el('list').innerHTML=items.map((it,i)=>`<div id=li${i} onclick="show(${i})">${labels[it.key]?'✓':'·'} ${it.piece} ${it.square} ${it.view}</div>`).join('');
}
function show(i){
  idx=i;const it=items[i];
  el('img').src='/img/'+encodeURIComponent(it.image);
  el('meta').textContent=`${it.piece} @ ${it.square} · view ${it.view}`;
  el('cur').textContent=labels[it.key]?('saved px '+labels[it.key].join(', ')):'not labeled';
  el('prog').textContent=`${Object.keys(labels).length} / ${items.length} labeled`;
  for(let k=0;k<items.length;k++){const e=el('li'+k);if(e)e.className=(k===i?'sel':'')+(labels[items[k].key]?' done':'');}
}
el('img').onclick=async function(e){
  const r=el('img').getBoundingClientRect();
  const im=el('img');
  const u=(e.clientX-r.left)/r.width*im.naturalWidth;
  const v=(e.clientY-r.top)/r.height*im.naturalHeight;
  await fetch('/api/label',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({key:items[idx].key,pixel:[Math.round(u),Math.round(v)]})});
  labels[items[idx].key]=[Math.round(u),Math.round(v)];
  // auto-advance to the next UNLABELED view so many cases flow quickly
  let next=-1;
  for(let s=1;s<=items.length;s++){
    const k=(idx+s)%items.length;
    if(!(items[k].key in labels)){next=k;break;}
  }
  buildList();
  show(next>=0?next:(idx+1)%items.length);
};
async function skip(){show((idx+1)%items.length);}
init();
</script></body></html>"""

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*a):pass
    def send(self,body,ct='text/html'):
        b=body.encode() if isinstance(body,str) else body
        self.send_response(200);self.send_header('Content-Type',ct)
        self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
    def do_GET(self):
        p=urlparse(self.path)
        if p.path=='/':self.send(PAGE)
        elif p.path=='/api/state':
            self.send(json.dumps({'items':S.items,'labels':S.labels}),'application/json')
        elif p.path.startswith('/img/'):
            name=p.path[len('/img/'):]
            f=S.dir/name
            if f.exists():self.send(f.read_bytes(),'image/jpeg')
            else:self.send_response(404);self.end_headers()
        else:self.send_response(404);self.end_headers()
    def do_POST(self):
        p=urlparse(self.path)
        if p.path=='/api/label':
            n=int(self.headers.get('Content-Length',0))
            d=json.loads(self.rfile.read(n))
            S.labels[d['key']]=d['pixel'];S.save()
            self.send('ok','text/plain')
        else:self.send_response(404);self.end_headers()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--dataset',default='adapt1');ap.add_argument('--port',type=int,default=8123)
    a=ap.parse_args()
    global S
    S=State(a.dataset)
    srv=ThreadingHTTPServer(('0.0.0.0',a.port),Handler)
    print(f'label UI for {a.dataset}: http://localhost:{a.port}/ ({len(S.items)} views)',flush=True)
    srv.serve_forever()

if __name__=='__main__':main()
