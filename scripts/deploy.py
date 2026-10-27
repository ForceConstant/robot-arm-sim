"""Deploy ONLY the explicitly authorized robot_arm stack through Dockhand.

No unrelated stack changes and no Docker socket mounted into the application.
The source snapshot is copied to Dockhand's build context via Docker archive API.
"""
import io
import json
from pathlib import Path
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
DOCKHAND = "http://100.76.32.93:7120"
DOCKER = "http://100.76.32.93:2375"
STACK = "robot_arm"
COMPOSE_PATH = "/app/data/stacks/HTPC/robot_arm/compose.yaml"


def request(base, path, method="GET", payload=None, raw=None):
    data = raw if raw is not None else (json.dumps(payload).encode() if payload is not None else None)
    req = urllib.request.Request(base + path, data=data, method=method,
        headers={"Content-Type": "application/x-tar" if raw is not None else "application/json"})
    with urllib.request.urlopen(req, timeout=90) as r:
        body = r.read()
        return json.loads(body) if body else {}


def compose():
    dockerfile = (ROOT / "Dockerfile").read_text()
    return """services:
  simulation:
    image: robot-arm-sim:1.0.0
    build:
      context: .
      dockerfile_inline: |
""" + "".join("        " + line + "\n" for line in dockerfile.splitlines()) + """    init: true
    restart: unless-stopped
    network_mode: bridge
    runtime: nvidia
    ports:
      - "192.168.1.138:8098:8098"
    environment:
      NVIDIA_VISIBLE_DEVICES: GPU-8be26e78-bf0f-b845-0c22-a5dc4c7c4cb9
      NVIDIA_DRIVER_CAPABILITIES: graphics,utility,compute
      MUJOCO_GL: egl
      ROBOT_VIEW_FPS: "5"
      ROBOT_DATA_DIR: /data
      STOCKFISH_PATH: /usr/games/stockfish
    volumes:
      - /mnt/user/appdata/robot_arm/data:/data
    cpus: 2.0
    mem_limit: 3g
    pids_limit: 256
    shm_size: 256m
    stop_grace_period: 10s
    logging:
      driver: json-file
      options:
        max-size: "10m"
        max-file: "3"
"""


def main():
    private = ROOT / ".private"
    private.mkdir(exist_ok=True)
    content = compose()
    (ROOT / "deploy/compose.yaml").write_text(content)
    stacks = request(DOCKHAND, "/api/stacks?env=1")
    exists = any(s["name"] == STACK for s in stacks)
    containers = request(DOCKER, "/containers/json?all=1")
    (private / "containers-before.json").write_text(json.dumps(
        {c["Names"][0]: c["Id"] for c in containers}, indent=2))
    if not exists:
        result = request(DOCKHAND, "/api/stacks?env=1", "POST", {
            "name": STACK, "compose": content, "composePath": COMPOSE_PATH,
            "envPath": "/app/data/stacks/HTPC/robot_arm/.env",
            "rawEnvContent": "", "start": False})
        print("Stack registration:", result)
    else:
        result = request(DOCKHAND, "/api/stacks/robot_arm/compose?env=1", "PUT", {"content": content})
        print("Stack compose saved:", result)
    saved = request(DOCKHAND, "/api/stacks/robot_arm/compose?env=1")
    assert saved["content"] == content, "Stored compose differs from submitted source"
    dockhand = next(c for c in containers if "/dockhand-dockhand-1" in c["Names"])
    tar = io.BytesIO()
    with tarfile.open(fileobj=tar, mode="w") as archive:
        for name in ["Dockerfile", ".dockerignore", "requirements.lock", "pyproject.toml",
                     "chess_sim", "robot_web", "deploy/entrypoint.sh"]:
            def keep(info):
                if any(part in info.name for part in ["__pycache__", "/generated/"]):
                    return None
                info.uid = info.gid = 1000
                info.uname = info.gname = "robot"
                return info
            archive.add(ROOT / name, arcname=name, filter=keep)
    request(DOCKER, "/containers/" + dockhand["Id"] +
            "/archive?path=/app/data/stacks/HTPC/robot_arm", "PUT", raw=tar.getvalue())
    result = request(DOCKHAND, "/api/stacks/robot_arm/deploy?env=1", "POST",
                     {"build": True, "pull": False, "forceRecreate": False})
    (private / "deploy-job.json").write_text(json.dumps(result, indent=2))
    print("Deployment job:", result)


if __name__ == "__main__":
    main()
