#!/bin/sh
set -eu
# Docker creates a new bind mount root-owned. Touch only this app's data root,
# then drop privileges. No Docker socket, servo device or host filesystem access.
mkdir -p /data
chown robot:robot /data
exec gosu robot python -m uvicorn robot_web.app:app --host 0.0.0.0 --port 8098
