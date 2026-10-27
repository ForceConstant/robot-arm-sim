"""Package the committed runtime/assets and server compose, never local state."""
import gzip
import hashlib
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / "dist"
out.mkdir(exist_ok=True)
archive = out / "robot-arm-sim-1.0.0-runtime.tar.gz"
source = subprocess.check_output(["git", "archive", "--format=tar", "--prefix=robot-arm-sim-1.0.0/", "HEAD"], cwd=root)
archive.write_bytes(gzip.compress(source, compresslevel=6, mtime=0))
shutil.copyfile(root / "deploy/compose.yaml", out / "robot_arm-compose.yaml")
files = [archive, out / "robot_arm-compose.yaml"]
(out / "SHA256SUMS").write_text("".join(hashlib.sha256(p.read_bytes()).hexdigest() + "  " + p.name + "\n" for p in files))
for p in files:
    print(p.name, p.stat().st_size)
