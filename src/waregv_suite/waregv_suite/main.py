"""
Helio Rover Backend
===================
FastAPI service exposing every endpoint referenced by tools.py / config.yaml.

Run:
    uvicorn main:app --host 0.0.0.0 --port 8000 --reload

Endpoints
---------
System / pose
    GET  /system/mode
    POST /system/mode
    POST /set_initial_pose
    GET  /robot_pose
    GET  /amcl_pose            (compat alias)

Navigation
    POST /navigate_to_pose
    POST /follow_waypoints
    POST /abort

Maps
    GET  /maps
    GET  /maps/{map_name}
    GET  /map/exists
    GET  /map/save
    POST /map/save_to_disk
    POST /map/load

Conversational
    POST /helio/command
"""

from __future__ import annotations

import io
import math
import os
import time
import zipfile
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, File, Form, HTTPException, Query, UploadFile
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel, Field

# --------------------------------------------------------------------
# Config
# --------------------------------------------------------------------
MAP_DIR = os.environ.get("ROVER_MAP_DIR", "maps")
os.makedirs(MAP_DIR, exist_ok=True)

DEFAULT_MAP = "small_warehouse"


# --------------------------------------------------------------------
# Runtime state
# --------------------------------------------------------------------
@dataclass
class RoverState:
    mode: str = "idle"                 # idle | mapping | navigation | ...
    map_name: str = DEFAULT_MAP
    # Pose is a simple dict; in production this is filled from TF.
    pose: Dict[str, Any] = field(default_factory=lambda: {
        "ok": True,
        "pose": {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "yaw_deg": 0.0,
        },
        "source": "tf:map->base_link",
    })
    active_goals: List[str] = field(default_factory=list)
    last_goal: Optional[Dict[str, Any]] = None
    pose_initialised: bool = False

    def set_pose(self, x: float, y: float, yaw_deg: float) -> None:
        yaw_rad = math.radians(yaw_deg)
        self.pose = {
            "ok": True,
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": math.sin(yaw_rad / 2.0),
                    "w": math.cos(yaw_rad / 2.0),
                },
                "yaw_deg": float(yaw_deg),
            },
            "source": "tf:map->base_link",
        }
        self.pose_initialised = True


STATE = RoverState()


# --------------------------------------------------------------------
# Pydantic models
# --------------------------------------------------------------------
class ModeBody(BaseModel):
    mode: str = Field(..., description="idle | mapping | navigation | ...")
    map_name: str = Field(DEFAULT_MAP)


class PoseBody(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


class Waypoint(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


class WaypointsBody(BaseModel):
    waypoints: List[Waypoint]


class HelioCommandBody(BaseModel):
    text: str
    lang: str = "en"


# --------------------------------------------------------------------
# App
# --------------------------------------------------------------------
app = FastAPI(
    title="Helio Rover Backend",
    version="1.0.0",
    description="REST bridge between the Helio AI agent and the WareGV rover.",
)


# ====================================================================
# Helpers
# ====================================================================
def _simulate_arrival(x: float, y: float, yaw_deg: float) -> None:
    """
    In a real deployment Nav2 would move the robot and TF would publish
    the new pose. For this backend we update the stored pose immediately
    so the agent sees a consistent world model.
    Replace this with a ROS TF subscriber in production.
    """
    STATE.set_pose(x, y, yaw_deg)


def _map_path(name: str) -> str:
    return os.path.join(MAP_DIR, name)


def _map_exists_on_disk(name: str) -> bool:
    d = _map_path(name)
    return os.path.isdir(d) and os.path.isfile(os.path.join(d, f"{name}.pgm"))


# ====================================================================
# SYSTEM MODE
# ====================================================================
@app.get("/system/mode")
def get_system_mode() -> Dict[str, Any]:
    return {"ok": True, "mode": STATE.mode, "map_name": STATE.map_name}


@app.post("/system/mode")
def set_system_mode(body: ModeBody) -> Dict[str, Any]:
    STATE.mode = body.mode
    STATE.map_name = body.map_name
    return {"ok": True, "mode": STATE.mode, "map_name": STATE.map_name}


# ====================================================================
# POSE
# ====================================================================
@app.post("/set_initial_pose")
def set_initial_pose(body: PoseBody) -> Dict[str, Any]:
    STATE.set_pose(body.x, body.y, body.yaw_deg)
    return {"ok": True, "pose_source": "tf:map->base_link"}


@app.get("/robot_pose")
def robot_pose() -> Dict[str, Any]:
    return STATE.pose


@app.get("/amcl_pose")
def amcl_pose() -> Dict[str, Any]:
    """Backwards-compatible alias for /robot_pose."""
    return STATE.pose


# ====================================================================
# NAVIGATION
# ====================================================================
@app.post("/navigate_to_pose")
def navigate_to_pose(body: PoseBody) -> Dict[str, Any]:
    goal_id = f"goal-{int(time.time() * 1000)}"
    STATE.active_goals.append(goal_id)
    STATE.last_goal = {
        "id": goal_id,
        "x": body.x,
        "y": body.y,
        "yaw_deg": body.yaw_deg,
    }
    # Simulate immediate arrival (see _simulate_arrival docstring).
    _simulate_arrival(body.x, body.y, body.yaw_deg)
    # Drop the goal from the active list once "complete".
    if goal_id in STATE.active_goals:
        STATE.active_goals.remove(goal_id)
    return {"ok": True, "status": "dispatched", "goal_id": goal_id}


@app.post("/follow_waypoints")
def follow_waypoints(body: WaypointsBody) -> Dict[str, Any]:
    if not body.waypoints:
        raise HTTPException(status_code=400, detail="waypoints list is empty")

    goal_id = f"route-{int(time.time() * 1000)}"
    STATE.active_goals.append(goal_id)

    # Walk through each waypoint (simulated).
    for wp in body.waypoints:
        STATE.last_goal = {
            "id": goal_id,
            "x": wp.x,
            "y": wp.y,
            "yaw_deg": wp.yaw_deg,
        }
        _simulate_arrival(wp.x, wp.y, wp.yaw_deg)

    if goal_id in STATE.active_goals:
        STATE.active_goals.remove(goal_id)

    return {
        "ok": True,
        "count": len(body.waypoints),
        "status": "dispatched",
        "goal_id": goal_id,
    }


@app.post("/abort")
def abort_mission() -> Dict[str, Any]:
    cancelled = list(STATE.active_goals)
    STATE.active_goals.clear()
    return {"ok": True, "cancelled": cancelled, "stopped": True}


# ====================================================================
# MAPS
# ====================================================================
@app.get("/maps")
def list_maps() -> Dict[str, Any]:
    if not os.path.isdir(MAP_DIR):
        return {"maps": [], "map_directory": os.path.abspath(MAP_DIR)}
    names = []
    for entry in sorted(os.listdir(MAP_DIR)):
        full = os.path.join(MAP_DIR, entry)
        if os.path.isdir(full) and os.path.isfile(os.path.join(full, f"{entry}.pgm")):
            names.append({"name": entry})
    return {"maps": names, "map_directory": os.path.abspath(MAP_DIR)}


@app.get("/maps/{map_name}")
def get_map_info(map_name: str) -> Dict[str, Any]:
    if not _map_exists_on_disk(map_name):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
    d = _map_path(map_name)
    return {
        "ok": True,
        "name": map_name,
        "yaml": os.path.abspath(os.path.join(d, f"{map_name}.yaml")),
        "image": os.path.abspath(os.path.join(d, f"{map_name}.pgm")),
    }


@app.get("/map/exists")
def map_exists(name: str = Query(..., description="Map name to check")) -> Dict[str, Any]:
    return {"name": name, "exists": _map_exists_on_disk(name)}


@app.get("/map/save")
def save_map(name: str = Query("map", description="Map name to download")):
    """
    Streams a zip archive containing <name>.pgm + <name>.yaml.
    """
    if not _map_exists_on_disk(name):
        raise HTTPException(status_code=404, detail=f"Map '{name}' not found")

    d = _map_path(name)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for fname in (f"{name}.pgm", f"{name}.yaml"):
            fpath = os.path.join(d, fname)
            if os.path.isfile(fpath):
                zf.write(fpath, arcname=fname)
    buf.seek(0)

    return StreamingResponse(
        buf,
        media_type="application/zip",
        headers={"Content-Disposition": f'attachment; filename="{name}.zip"'},
    )


@app.post("/map/save_to_disk")
async def save_map_to_disk(
    name: str = Form(...),
    overwrite: bool = Form(False),
    pgm: UploadFile = File(...),
    yaml: Optional[UploadFile] = File(None),
) -> Dict[str, Any]:
    target_dir = _map_path(name)
    if _map_exists_on_disk(name) and not overwrite:
        return JSONResponse(
            status_code=409,
            content={
                "ok": False,
                "error": f"Map '{name}' already exists. Pass overwrite=true to replace.",
            },
        )

    os.makedirs(target_dir, exist_ok=True)
    pgm_path = os.path.join(target_dir, f"{name}.pgm")
    with open(pgm_path, "wb") as f:
        f.write(await pgm.read())

    if yaml is not None:
        yaml_path = os.path.join(target_dir, f"{name}.yaml")
        with open(yaml_path, "wb") as f:
            f.write(await yaml.read())
    else:
        # Write a minimal YAML stub so map_info/list always finds a pair.
        yaml_path = os.path.join(target_dir, f"{name}.yaml")
        with open(yaml_path, "w") as f:
            f.write(
                f"image: {name}.pgm\n"
                f"resolution: 0.05\n"
                f"origin: [0.0, 0.0, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

    return {
        "ok": True,
        "name": name,
        "directory": os.path.abspath(target_dir),
    }


@app.post("/map/load")
async def load_map(
    name: str = Form(...),
    overwrite: bool = Form(False),
    pgm: UploadFile = File(...),
    yaml: Optional[UploadFile] = File(None),
) -> Dict[str, Any]:
    """Alias of /map/save_to_disk (same semantics)."""
    return await save_map_to_disk(name=name, overwrite=overwrite, pgm=pgm, yaml=yaml)


# ====================================================================
# CONVERSATIONAL FALLBACK
# ====================================================================
@app.post("/helio/command")
def helio_command(body: HelioCommandBody) -> Dict[str, Any]:
    """
    Alternate entrypoint for conversational commands. This backend does not
    run the LLM itself — it simply echoes a structured acknowledgement so
    the UI / agent layer can route the text to the model.
    """
    return {
        "ok": True,
        "reply": (
            f"Received command in '{body.lang}': {body.text!r}. "
            "Route this through the Helio agent for execution."
        ),
    }


# ====================================================================
# DIAGNOSTICS
# ====================================================================
@app.get("/health")
def health() -> Dict[str, Any]:
    return {
        "ok": True,
        "mode": STATE.mode,
        "map_name": STATE.map_name,
        "active_goals": list(STATE.active_goals),
        "pose_initialised": STATE.pose_initialised,
    }


@app.get("/state")
def full_state() -> Dict[str, Any]:
    """Full snapshot for debugging."""
    return {
        "mode": STATE.mode,
        "map_name": STATE.map_name,
        "pose": STATE.pose,
        "active_goals": STATE.active_goals,
        "last_goal": STATE.last_goal,
        "pose_initialised": STATE.pose_initialised,
    }


# --------------------------------------------------------------------
# Dev entrypoint
# --------------------------------------------------------------------
if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)