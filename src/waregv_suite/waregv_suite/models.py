"""Pydantic request models."""
from typing import List, Optional
from pydantic import BaseModel


class PoseRequest(BaseModel):
    x: float
    y: float
    yaw_deg: float


class WaypointItem(BaseModel):
    x: float
    y: float
    yaw_deg: float


class WaypointsRequest(BaseModel):
    waypoints: List[WaypointItem]


class ModeRequest(BaseModel):
    mode: str
    map_name: str = ""


class HelioRequest(BaseModel):
    text: str = ""
    message: str = ""
    lang: str = "en"
    call_id: Optional[str] = None


class WebRTCOffer(BaseModel):
    sdp: str
    type: str = "offer"


class HelioStatePayload(BaseModel):
    event: str
    state: str
    text: Optional[str] = ""
    sound_name: Optional[str] = ""