from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel, Field

from . import __version__
from .lab import Lab, VIEWS
from .incidents import LABELS
from .torture import CASES, SUITE_VERSION

lab = Lab()


@asynccontextmanager
async def lifespan(app):
    lab.start_worker()
    yield
    lab.shutdown.set()
    lab.wake.set()
    await asyncio.to_thread(lab.thread.join, 5)


app = FastAPI(title="Robot Arm · Chess Lab", version=__version__, lifespan=lifespan,
              docs_url=None, redoc_url=None)


@app.middleware("http")
async def same_origin_controls(request: Request, call_next):
    if request.method == "POST":
        origin = request.headers.get("origin")
        if origin is not None and origin.rstrip("/") != str(request.base_url).rstrip("/"):
            return Response("Cross-origin controls are not allowed", status_code=403)
        if request.headers.get("x-robot-lab") != "1":
            return Response("Control header required", status_code=403)
    response = await call_next(request)
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["Referrer-Policy"] = "same-origin"
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/")
def index():
    return FileResponse(Path(__file__).with_name("index.html"))


@app.get("/health")
def health():
    return Response('OK' if lab.ready else 'Simulation not ready',
                    status_code=200 if lab.ready else 503, media_type="text/plain")


@app.get("/api/status")
def status():
    return {"version": __version__, **lab.snapshot()}


class Control(BaseModel):
    action: Literal["start", "pause", "stop", "reset", "step", "configure", "speed",
                   "orbit", "record", "suite", "replay", "live", "rerun",
                   "correct", "sweep", "jog"]
    mode: Literal["engine", "opening"] = "engine"
    scenario: Literal["standard", "capture"] = "standard"
    observations: Literal["wrists", "all"] = "wrists"
    speed: float = Field(default=2, ge=0.25, le=8)
    azimuth: float | None = None
    elevation: float | None = None
    distance: float | None = None
    reset: bool = False
    enable: bool = True
    suite: Literal["start", "stop"] = "start"
    suite_seed: int | None = Field(default=None, ge=0, le=2**32 - 1)
    id: str | None = None
    frame: int | None = None
    play: bool = False
    params: dict | None = None
    # Manual (gamepad) jog: one Cartesian step per request, consumed once by the
    # physics worker. Nothing runs away if the browser or controller drops.
    dx: float = Field(default=0.0, ge=-0.05, le=0.05)
    dy: float = Field(default=0.0, ge=-0.05, le=0.05)
    dz: float = Field(default=0.0, ge=-0.05, le=0.05)
    grip: float | None = Field(default=None, ge=0.0, le=0.06)
    arm: Literal["white", "black"] | None = None
    # Keyboard joint jog: one servo per key pair (A/Q, S/W, D/E, F/R, G/T, Z/X).
    joint: int | None = Field(default=None, ge=0, le=5)
    dq: float = Field(default=0.0, ge=-0.20, le=0.20)


@app.post("/api/control")
def control(body: Control):
    try:
        if not lab.ready and body.action != "stop":
            raise HTTPException(409, "The simulator is still initializing")
        options = body.model_dump(exclude={"action"})
        if body.action in ("replay", "rerun") and not options.get("id"):
            raise HTTPException(422, "A recording id is required")
        lab.command(body.action, **options)
        return {"version": __version__, **lab.snapshot()}
    except ValueError as exc:
        raise HTTPException(409, str(exc)) from exc


@app.get("/api/incidents")
def incidents():
    with lab.lock:
        rows = lab.recorder.rows()
        return {"recording": lab.record_enabled, "rows": rows, "labels": list(LABELS)}


@app.get("/api/incidents/{ident}/trace")
def incident_trace(ident: str):
    try:
        path = lab.recorder.path(ident)
    except ValueError as exc:
        raise HTTPException(404, str(exc)) from exc
    return FileResponse(path / "trace.json", media_type="application/json")


@app.get("/api/incidents/{ident}.zip")
def incident_zip(ident: str):
    try:
        with lab.lock:
            data = lab.recorder.download(ident)
    except ValueError as exc:
        raise HTTPException(404, str(exc)) from exc
    return Response(data, media_type="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="incident-{ident}.zip"'})


@app.get("/api/incidents/{ident}")
def incident(ident: str):
    try:
        with lab.lock:
            return lab.recorder.read(ident)
    except ValueError as exc:
        raise HTTPException(404, str(exc)) from exc


class Review(BaseModel):
    label: str
    notes: str = ""


@app.post("/api/incidents/{ident}/label")
def incident_label(ident: str, body: Review):
    try:
        with lab.lock:
            lab.recorder.annotate(ident, body.label, body.notes)
            return lab.recorder.read(ident)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from exc


@app.get("/api/suite")
def suite():
    return {"version": SUITE_VERSION,
            "cases": [{"id": c.id, "arm": c.arm, "move": c.move,
                       "description": c.description} for c in CASES]}


@app.get("/api/frame/{camera}.jpg")
def frame(camera: str):
    if camera not in VIEWS:
        raise HTTPException(404)
    lab.touch_view(camera)
    with lab.lock:
        data = lab.frames.get(camera)
    if not data:
        raise HTTPException(503, "First camera frame not ready")
    return Response(data, media_type="image/jpeg")


@app.get("/api/stream/{camera}")
async def stream(camera: str, request: Request):
    if camera not in VIEWS:
        raise HTTPException(404)
    lab.touch_view(camera)

    async def images():
        last = -1
        sent_at = 0.0
        while not await request.is_disconnected():
            lab.touch_view(camera)
            with lab.lock:
                seq, data = lab.frame_id, lab.frames.get(camera)
            # Browsers may wait for the next multipart boundary before showing
            # the first frame. Repeat cached JPEGs while idle, without rendering.
            if data and (seq != last or time.monotonic() - sent_at > 1.0):
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                       str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                last = seq
                sent_at = time.monotonic()
            await asyncio.sleep(0.08)
    return StreamingResponse(images(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/api/observations")
def observations():
    s = lab.snapshot()
    # Exact game/body state intentionally absent from this sensor contract.
    return {"sim_time": s["sim_time"], "frame_id": s["frame_id"], "joints": s["joints"],
            "cameras": {name: "/api/frame/" + name + ".jpg" for name in s["observation_cameras"]},
            "perception_validated": False}


@app.get("/api/game.pgn")
def game():
    path = lab.storage / "last-game.pgn"
    if not path.is_file():
        raise HTTPException(404, "No completed move yet")
    return FileResponse(path, filename="robot-arm-game.pgn", media_type="application/x-chess-pgn")
