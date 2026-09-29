"""FastAPI app and HTTP routes."""
from typing import Optional

from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from .models import (
    PoseRequest, WaypointsRequest, ModeRequest,
    HelioRequest, WebRTCOffer, HelioStatePayload,
)
from .ros_node import WareGVBridgeNode
from .system_stats import SystemStatsSampler


app = FastAPI(title="WareGV Autonomous Rover API", version="1.2")

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://127.0.0.1:5500",
        "http://localhost:5500",
        "*",
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "*"],
    allow_headers=["*"],
    expose_headers=["*"],
)

ros_node: Optional[WareGVBridgeNode] = None
system_stats: Optional[SystemStatsSampler] = None


# ---------------- Health ----------------

@app.get("/")
async def health():
    if ros_node is None:
        return {
            "ok": False,
            "service": "waregv_suite_backend",
            "detail": "ROS node is not ready",
        }
    stats = system_stats.snapshot() if system_stats else {}
    return {
        "ok": True,
        "service": "waregv_suite_backend",
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
        "map_directory": str(ros_node.map_directory),
        "pose_source": "tf:map->base_link",
        "amcl_used": False,
        "cpu_percent": stats.get("cpu_percent"),
        "ram_percent": stats.get("ram_percent"),
    }


# ---------------- System stats ----------------

@app.get("/system/stats")
async def get_system_stats():
    """CPU %, RAM %, load average, SoC temperature. Cached snapshot."""
    if system_stats is None:
        raise HTTPException(status_code=503, detail="System stats sampler not ready")
    return system_stats.snapshot()


# ---------------- Pose ----------------

@app.get("/robot_pose")
@app.get("/pose")
async def get_robot_pose():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    result = ros_node.get_robot_pose()
    if not result["ok"]:
        return JSONResponse(status_code=503, content=result)
    return result


@app.get("/amcl_pose")
async def get_pose_compat():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    result = ros_node.get_robot_pose()
    if not result["ok"]:
        return JSONResponse(status_code=503, content=result)
    return result


# ---------------- Mode ----------------

@app.post("/system/mode")
async def set_mode(req: ModeRequest):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    map_name = (req.map_name or "").strip()
    mode_lower = req.mode.lower()
    existing_map_required = (
        "fixed" in mode_lower
        or "update" in mode_lower
        or mode_lower in {"auto_nav", "autonomous", "autonomous_driving"}
    )
    if existing_map_required and map_name:
        info = ros_node.get_map_info(map_name)
        if not info["exists"]:
            raise HTTPException(status_code=404, detail=info["detail"])
    ros_node.current_mode = req.mode
    ros_node.current_map = map_name
    ros_node.get_logger().info(
        f"System mode switched: {req.mode}, map: {map_name or '<new-map>'}"
    )
    return {
        "ok": True,
        "mode": ros_node.current_mode,
        "map_name": ros_node.current_map,
    }


@app.get("/system/mode")
async def get_mode():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    return {"mode": ros_node.current_mode, "map_name": ros_node.current_map}


# ---------------- Maps ----------------

@app.get("/maps")
@app.get("/map/list")
@app.get("/maps/list")
async def list_maps():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    names = ros_node.list_map_names()
    return {
        "maps": [{"name": n} for n in names],
        "map_directory": str(ros_node.map_directory),
    }


@app.get("/maps/{map_name}")
async def map_info(map_name: str):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    info = ros_node.get_map_info(map_name)
    if not info["exists"]:
        raise HTTPException(status_code=404, detail=info["detail"])
    return info


# ---------------- Initial pose ----------------

@app.post("/set_initial_pose")
async def set_initial_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        ros_node.set_initial_pose(req.x, req.y, req.yaw_deg)
        return {"ok": True, "pose_source": "tf:map->base_link", "amcl_used": False}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


# ---------------- Navigation ----------------

@app.post("/navigate_to_pose")
async def navigate_to_pose(req: PoseRequest):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        ros_node.send_navigate_to_pose_goal(req.x, req.y, req.yaw_deg)
        return {
            "ok": True, "x": req.x, "y": req.y,
            "yaw_deg": req.yaw_deg, "status": "dispatched",
        }
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/follow_waypoints")
async def follow_waypoints(req: WaypointsRequest):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        ros_node.send_follow_waypoints_goal(req.waypoints)
        return {"ok": True, "count": len(req.waypoints), "status": "dispatched"}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/abort")
@app.post("/abort_mission")
async def abort_mission():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        cancelled = ros_node.cancel_all_goals()
        return {"ok": True, "cancelled": cancelled, "stopped": True}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc))


# ---------------- SLAM ----------------

@app.post("/slam/reset")
async def reset_slam():
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        return ros_node.reset_slam()
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"SLAM reset failed: {exc}")


# ---------------- Map download / save ----------------

@app.get("/map/save")
async def save_map(name: str = "map"):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        safe_name, zip_buffer = ros_node.map_archive(name)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc))
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Could not archive map: {exc}")
    return StreamingResponse(
        zip_buffer,
        media_type="application/zip",
        headers={"Content-Disposition": f"attachment; filename={safe_name}.zip"},
    )


@app.get("/map/exists")
async def map_exists(name: str):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    return {"name": ros_node._safe_map_name(name), "exists": ros_node.map_exists(name)}


@app.post("/map/save_to_disk")
async def save_map_to_disk(
    name: str = Form(...),
    overwrite: bool = Form(False),
    pgm: UploadFile = File(...),
    yaml: Optional[UploadFile] = File(None),
):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        pgm_bytes = await pgm.read()
        yaml_bytes = await yaml.read() if yaml is not None else None
        return ros_node.save_map_to_disk(
            name=name, pgm_bytes=pgm_bytes,
            yaml_bytes=yaml_bytes, overwrite=overwrite,
        )
    except FileExistsError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Save map failed: {exc}")


@app.post("/map/load")
async def load_map(
    name: str = Form(...),
    overwrite: bool = Form(False),
    pgm: UploadFile = File(...),
    yaml: Optional[UploadFile] = File(None),
):
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node is not ready")
    try:
        pgm_bytes = await pgm.read()
        yaml_bytes = await yaml.read() if yaml is not None else None
        return ros_node.save_map_to_disk(
            name=name, pgm_bytes=pgm_bytes,
            yaml_bytes=yaml_bytes, overwrite=overwrite,
        )
    except FileExistsError as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Load map failed: {exc}")


# ---------------- Helio ----------------

@app.post("/helio/state")
async def helio_state(req: HelioStatePayload):
    if ros_node and req.state:
        ros_node.set_profile(req.state)
    if ros_node and req.event == "speaking_start" and req.text:
        ros_node.request_speech(req.text)
    try:
        import httpx
        async with httpx.AsyncClient(timeout=2.0) as client:
            if req.event == "sound_play" and req.sound_name:
                await client.post(
                    "http://127.0.0.1:8080/play_sound",
                    json={"sound": req.sound_name},
                )
    except Exception as exc:
        if ros_node:
            ros_node.get_logger().warning(f"Sound operator unavailable: {exc}")
    return {"ok": True}


@app.post("/helio/command")
async def helio_command(req: HelioRequest):
    text = (req.text or req.message).strip()
    if not text:
        raise HTTPException(status_code=422, detail="text or message is required")
    if req.lang.lower().startswith("hi"):
        reply = "मैंने आपका अनुरोध प्राप्त कर लिया है।"
    else:
        reply = "I received your request."
    return {"ok": True, "text": reply, "reply": reply, "call_id": req.call_id}


# ---------------- WebRTC ----------------

async def _forward_webrtc_offer(endpoint: str, offer: WebRTCOffer):
    if not ros_node or not ros_node.webrtc_signaler:
        raise HTTPException(
            status_code=503,
            detail=(
                "WebRTC signaling is provided by "
                "camera_webrtc_streamer.py; set "
                "WAREGV_WEBRTC_SIGNAL_URL to proxy it."
            ),
        )
    try:
        import httpx
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(
                ros_node.webrtc_signaler + endpoint,
                json=offer.model_dump(),
            )
        if response.status_code >= 400:
            raise HTTPException(status_code=response.status_code, detail=response.text)
        return JSONResponse(content=response.json())
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(
            status_code=502, detail=f"WebRTC signaling failed: {exc}",
        )


@app.post("/offer/color")
async def offer_color(offer: WebRTCOffer):
    return await _forward_webrtc_offer("/offer/color", offer)


@app.post("/offer/depth")
async def offer_depth(offer: WebRTCOffer):
    return await _forward_webrtc_offer("/offer/depth", offer)