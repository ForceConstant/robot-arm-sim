"""Download pinned public Phase 11 sources/artifacts into a user-owned stage.

No OS, driver or service changes; no private uploads. Dependencies are installed
separately into the documented private virtualenv, never system Python.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile
import urllib.request

REV='2774d9bddcbbda50e697e162e89e7eaada8d7105'
ACT_SHA='8d622935319af7d03f1efd95d51e726af2921010f7e4f5ec3742f62280493111'
ARCHIVE_SHA='794aa3761cc44e5ac3071241e274df34d582cac1a4b481f3adce753b079c8fbc'
SMOL_REV='5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6'
BASE_REV='7b375e1b73b11138ff12fe22c8f2822d8fe03467'


def download(root,name,url,digest):
    path=root/name
    if not path.exists():
        with urllib.request.urlopen(url,timeout=120) as response, path.open('xb') as target:
            while chunk:=response.read(1024*1024): target.write(chunk)
    with path.open('rb') as stream: actual=hashlib.file_digest(stream,'sha256').hexdigest()
    if actual!=digest: raise ValueError(f'{name}: artifact identity differs')
    return dict(name=name,url=url,sha256=actual,bytes=path.stat().st_size)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path.home()/'.local/share/robot-arm-bc250-investigation/phase11')
    parser.add_argument('--vla',action='store_true',help='also stage ~0.9 GB pretrained public SmolVLA weights and pinned LeRobot source')
    args=parser.parse_args()
    args.root.mkdir(parents=True,exist_ok=True)
    files=[download(args.root,'modeling_act.py',f'https://raw.githubusercontent.com/huggingface/lerobot/{REV}/src/lerobot/policies/act/modeling_act.py',ACT_SHA)]
    if args.vla:
        files.append(download(args.root,'lerobot.tar.gz',f'https://codeload.github.com/huggingface/lerobot/tar.gz/{REV}',ARCHIVE_SHA))
        if not (args.root/f'lerobot-{REV}').exists():
            with tarfile.open(args.root/'lerobot.tar.gz') as archive: archive.extractall(args.root,filter='data')
        from huggingface_hub import snapshot_download
        snapshot_download('lerobot/smolvla_base',revision=SMOL_REV,local_dir=args.root/'smolvla-artifact',allow_patterns=['*.json','*.safetensors'])
        snapshot_download('HuggingFaceTB/SmolVLM2-500M-Video-Instruct',revision=BASE_REV,local_dir=args.root/'smolvlm-base-config',allow_patterns=['*.json','*.txt','*.model'])
        (args.root/'smolvlm-base-revision.txt').write_text(BASE_REV+'\n')
    report=dict(lerobot_revision=REV,files=files,smolvla_revision=SMOL_REV if args.vla else None,
                base_revision=BASE_REV if args.vla else None,scope='public user-space artifacts only; no upload/system install')
    (args.root/'preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__': main()
