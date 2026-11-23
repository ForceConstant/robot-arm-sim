"""Labeler routes for the real-piece dataset, served by the console on :8080.

Port 8123 is filtered off-box; port 8080 (the console) is reachable, so we mount
the click-to-label UI here. State is loaded from/saved to labels_human.json in
the dataset dir on the BC-250. Installed by importing install(app) in app.py.
"""
from __future__ import annotations
import json
from pathlib import Path
from fastapi import Request
from fastapi.responses import HTMLResponse, JSONResponse, Response

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DATASET = 'adapt1'

_state = None


def _dir(dataset: str) -> Path:
    return ROOT / '.private' / 'real_piece_views' / dataset


def _load(dataset: str):
    global _state
    ddir = _dir(dataset)
    mf = json.loads((ddir / 'manifest.json').read_text())
    lp = ddir / 'labels_human.json'
    labels = json.loads(lp.read_text()) if lp.exists() else {}
    items = []
    for pl in mf['placements']:
        for v in pl['views']:
            items.append({'key': v['image'], 'image': v['image'],
                          'piece': pl['piece'], 'square': pl['square'],
                          'view': v['view'], 'cam': v.get('cam')})
    _state = {'dataset': dataset, 'dir': ddir, 'labels_path': lp,
              'items': items, 'labels': labels}
    return _state


def _cur():
    global _state
    if _state is None:
        _load(DEFAULT_DATASET)
    return _state


PAGE = """<!doctype html><html><head><meta charset=utf-8><title>Label piece base centres</title>
<style>
body{font:14px ui-sans-serif,system-ui;background:#111;color:#eee;margin:0}
header{padding:10px 16px;background:#1a2330;border-bottom:1px solid #2a3a4f}
.wrap{display:flex;gap:16px;padding:16px;align-items:flex-start}
img{max-width:640px;max-height:70vh;cursor:crosshair;border:1px solid #333}
.side{min-width:280px}
button{background:#26405c;color:#eef;border:1px solid #3a6288;padding:6px 10px;border-radius:4px;cursor:pointer;margin:2px}
.kv{font:12px ui-monospace,monospace;color:#9fb6cc;margin:4px 0}
.list{max-height:70vh;overflow:auto;font:12px ui-monospace,monospace}
.list div{padding:2px 4px;cursor:pointer}
.list div.sel{background:#26405c}
.done{color:#7fd4a0}
</style></head><body>
<header><b>Label base centres</b> <span id=prog></span></header>
<div class=wrap><div><img id=img></div><div class=side>
<div class=kv id=meta></div>
<div class=kv>Click the CENTRE OF THE PIECE BASE (where it touches the board).</div>
<div class=kv id=cur></div>
<button onclick=skip()>Skip</button>
<div class=list id=list></div>
</div></div>
<script>
let items=[],labels={},idx=0;const el=id=>document.getElementById(id);
async function init(){const r=await fetch('/label/api/state');const d=await r.json();
 items=d.items;labels=d.labels;buildList();show(0);}
function buildList(){el('list').innerHTML=items.map((it,i)=>`<div id=li${i} onclick="show(${i})">${labels[it.key]?'✓':'·'} ${it.piece} ${it.square} ${it.view}</div>`).join('');}
function show(i){idx=i;const it=items[i];el('img').src='/label/img/'+encodeURIComponent(it.image);
 el('meta').textContent=`${it.piece} @ ${it.square} · view ${it.view}`;
 el('cur').textContent=labels[it.key]?('saved px '+labels[it.key].join(', ')):'not labeled';
 el('prog').textContent=`${Object.keys(labels).length} / ${items.length} labeled`;
 for(let k=0;k<items.length;k++){const e=el('li'+k);if(e)e.className=(k===i?'sel ':'')+(labels[items[k].key]?'done':'');}}
el('img').onclick=async function(e){const r=el('img').getBoundingClientRect(),im=el('img');
 const u=Math.round((e.clientX-r.left)/r.width*im.naturalWidth),v=Math.round((e.clientY-r.top)/r.height*im.naturalHeight);
 await fetch('/label/api/label',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:items[idx].key,pixel:[u,v]})});
 labels[items[idx].key]=[u,v];let next=-1;for(let s=1;s<=items.length;s++){const k=(idx+s)%items.length;if(!(items[k].key in labels)){next=k;break;}}buildList();show(next>=0?next:(idx+1)%items.length);};
async function skip(){show((idx+1)%items.length);}init();
</script></body></html>"""


def install(app, dataset: str = DEFAULT_DATASET):
    _load(dataset)

    @app.get('/label')
    def label_page():
        return HTMLResponse(PAGE, headers={"Cache-Control": "no-store"})

    @app.get('/label/api/state')
    def label_state():
        s = _cur()
        return JSONResponse({'dataset': s['dataset'], 'items': s['items'],
                             'labels': s['labels']})

    @app.get('/label/img/{name}')
    def label_img(name: str):
        f = _cur()['dir'] / name
        if not f.exists() or '..' in name:
            return Response(status_code=404)
        return Response(f.read_bytes(), media_type='image/jpeg')

    @app.post('/label/api/label')
    async def label_set(request: Request):
        body = await request.json()
        s = _cur()
        s['labels'][str(body['key'])] = [int(body['pixel'][0]), int(body['pixel'][1])]
        s['labels_path'].write_text(json.dumps(s['labels'], indent=2) + '\n')
        return JSONResponse({'ok': True, 'count': len(s['labels'])})
