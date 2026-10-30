"""Stage pinned Tinygrad source and Dawn under the user-owned BC-250 probe tree."""
import hashlib, json, pathlib, tarfile, urllib.request
root = pathlib.Path.home() / '.local/share/robot-arm-bc250-investigation/training-webgpu'
root.mkdir(parents=True, exist_ok=True)
rev = '0dd490c19492e606aadde8dc25e10b9223e06dbf'
urls = [('tinygrad.tar.gz', f'https://codeload.github.com/tinygrad/tinygrad/tar.gz/{rev}'), ('libwebgpu_dawn.so', 'https://github.com/wpmed92/pydawn/releases/download/v0.1.6/libwebgpu_dawn.so')]
expected = {'tinygrad.tar.gz': '52103076081ff065f29046ce23cb15505dca8615ca1927cf2fb7fc640a3fa36f', 'libwebgpu_dawn.so': 'cf36091d266a32c9d5080f14662de44cece241987939713282ea0ff558db81c6'}
receipts = []
for name, url in urls:
    p = root / name
    if not p.exists():
        with urllib.request.urlopen(url, timeout=60) as r:
            p.write_bytes(r.read())
    digest = hashlib.file_digest(p.open('rb'), 'sha256').hexdigest()
    if digest != expected[name]:
        raise RuntimeError('Artifact hash mismatch: ' + name)
    receipts.append(dict(name=name, url=url, bytes=p.stat().st_size, sha256=digest))
    if name.endswith('.so'):
        assert p.read_bytes()[:4] == b'\x7fELF'
if not (root / f'tinygrad-{rev}').exists():
    with tarfile.open(root / 'tinygrad.tar.gz') as tf:
        tf.extractall(root, filter='data')
(root / 'provenance.json').write_text(json.dumps(dict(tinygrad_revision=rev, dawn_version='v0.1.6', files=receipts, publisher_dawn_digest=None, scope='user-local source/library only; no system install'), indent=2))
print((root / 'provenance.json').read_text())
