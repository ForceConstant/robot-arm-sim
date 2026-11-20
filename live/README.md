# Robot-arm live console (Phase 11d)

Two processes, one bus:

- **Pi bridge** (`pi_bridge/`, port 8100) owns the follower in REMOTE mode and
  delegates TELEOP to LeLab.
- **Live console** (`app.py`, port 8080) serves the page, pulls the camera with
  markers drawn on it, and runs scans through the Pi bridge.

## Camera controls belong to the bridge

The bridge re-applies `camera_controls.json` (on the Pi, currently
`/home/pi/so101-bridge/camera_controls.json`) every time it opens the camera.
A `v4l2-ctl --set-ctrl ...` sweep therefore reads back unchanged and looks like
it had no effect: measured 2026-11-20, four different settings all rendered the
same frame mean (215). Change camera settings through the console instead -
`POST /camera/controls` writes the file the bridge reloads, so the setting
survives the next reopen. Changing the device from a shell only lasts until the
camera is closed.

## Run

```bash
cp live/live_config.example.json live/live_config.json   # edit host/paths
printf '%s' "$(cat /home/pi/so101-bridge/token.txt)" > live/bridge_token.txt
chmod 600 live/bridge_token.txt
python live/app.py
```

Then open `http://<bc250>:8080/`, paste the bridge token into the page (stored in
localStorage), and use:

1. **Arm** — clears the bridge latch.
2. **Remote mode** — stops LeLab teleop, takes the follower.
3. **Find board** — runs the marker scan; the offset appears when it finishes.
4. **Teleop** — hands the bus back to LeLab (requires the leader to match).
5. **Hold / E-STOP** — bounded stop; E-STOP latches and needs Arm again.

The page only orchestrates: no motion authority lives here, and every POST
carries the bridge token. `/state` and `/stream` are unauthenticated on the LAN.

## Known gaps

- `/stream` is implemented but not covered by tests (MJPEG + TestClient hangs).
- `board_z_m` and `marker_z_m` are sim values in the example config; on the real
  bench they must come from the measured table/mat/slab heights.
- The jaw convention (sim radians → lerobot 0..100) is unverified in direction.
