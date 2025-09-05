"""
Professional server implementation for the tank level control system.

This server uses FastAPI (built on Starlette) and Pydantic for a modern, type‑safe
architecture. It defines strong message models, maintains board state via classes,
and schedules background tasks to supervise board connectivity and pump runtime.

FastAPI has become one of the most popular Python frameworks in 2025. According
to the JetBrains State of Python survey, FastAPI leads the charts because it
combines async/await support, automatic request validation and OpenAPI
documentation generation【848250930434527†L107-L138】. It’s built on Starlette and
Pydantic, which provide high performance and a modern type system. The WebSocket
integration in FastAPI allows us to build real‑time applications with low
latency【747392896601809†L41-L69】. For concurrency, we rely on Python’s
asyncio event loop and manage tasks with care.

The server exposes two WebSocket endpoints:
  - `/ws/bridge`  : used by the master ESP32 gateway to forward sensor data
    and receive commands.
  - `/ws/client`  : used by browser clients to visualize data and send
    configuration or manual commands.

The server stores configuration and state in JSON files and uses background
tasks to monitor timeouts and ensure fail‑safe behaviour.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, validator
from typing import Literal

# ---------------------------------------------------------------------------
# Pydantic models for messages

class SensorItem(BaseModel):
    id: str
    dist_mm: int
    level_pct: float

class SensorDataMessage(BaseModel):
    # Use Literal for constant fields; Pydantic v2 removed 'const'
    type: Literal["sensor_data"] = "sensor_data"
    src: str
    seq: int
    items: List[SensorItem]
    rssi: Optional[int] = None
    timestamp: float = Field(default_factory=lambda: time.time())

class AckMessage(BaseModel):
    type: Literal["ack"] = "ack"
    from_: str = Field(..., alias="from")  # `from` is reserved in Python
    state: str
    reason: Optional[str] = None

class StatusMessage(BaseModel):
    type: Literal["status"] = "status"
    board: str
    state: str

class CommandMessage(BaseModel):
    type: Literal["cmd"] = "cmd"
    dst: str
    cmd: str
    value: str
    ttl_ms: Optional[int] = None
    reason: Optional[str] = None

    @validator("dst")
    def dst_upper(cls, v: str) -> str:
        return v.upper()

class ConfigSetMessage(BaseModel):
    type: Literal["config_set"] = "config_set"
    target: str
    payload: Dict[str, float]

class HeartbeatMessage(BaseModel):
    type: Literal["hb"] = "hb"
    from_: str = Field(..., alias="from")

# ---------------------------------------------------------------------------
# Configuration and state management

CONFIG_FILE = Path("config.json")
STATE_FILE = Path("state.json")

# Default configuration.  The system uses separate calibration values
# for each ultrasonic sensor: when the tanks are empty or full.  The
# `empty_*_mm` values correspond to the measured distance from sensor to
# liquid surface when the container is empty.  The `full_*_mm` values
# correspond to the distance when the container is full.  These values
# allow conversion of raw distance to percentage level on the sensor node.
default_config: Dict[str, Any] = {
    "empty_tanque_mm": 1200.0,
    "full_tanque_mm": 200.0,
    "empty_tolva_mm": 800.0,
    "full_tolva_mm": 100.0,
    "SP_tolva": 60.0,
    "hysteresis": 5.0,
    "min_on_s": 5.0,
    "min_off_s": 5.0,
    "max_run_s": 300.0,
    "mode": "AUTO",
    "pulse_rate": 1.0  # pulses per minute when pump is on
}

# Utility functions to load and save JSON config/state

def load_json(path: Path, defaults: Dict[str, Any]) -> Dict[str, Any]:
    if path.exists():
        try:
            data = json.loads(path.read_text())
            for k, v in defaults.items():
                data.setdefault(k, v)
            return data
        except Exception:
            logging.exception("Failed to load %s; using defaults", path)
    return defaults.copy()

def save_json(path: Path, data: Dict[str, Any]) -> None:
    try:
        path.write_text(json.dumps(data))
    except Exception:
        logging.exception("Failed to save %s", path)

# ---------------------------------------------------------------------------
# Board state classes

@dataclass
class BoardState:
    """Represents state of an individual board (sensor or actuator)."""
    online: bool = False
    last_seen: float = 0.0
    rssi: Optional[int] = None
    pump_state: Optional[bool] = None  # only for actuator board

    def update_seen(self, rssi: Optional[int] = None) -> None:
        self.online = True
        self.last_seen = time.time()
        if rssi is not None:
            self.rssi = rssi


class BoardManager:
    """Manages all boards (sensor, actuator, master)."""

    def __init__(self, config: Dict[str, Any]):
        self.boards: Dict[str, BoardState] = {
            "SENS": BoardState(),
            "ACT": BoardState(),
            "MASTER": BoardState()
        }
        # sensor levels (percentage) and raw distances (mm)
        self.tanque_pct: Optional[float] = None
        self.tolva_pct: Optional[float] = None
        self.tanque_mm: Optional[int] = None
        self.tolva_mm: Optional[int] = None
        self.config = config
        self._lock = asyncio.Lock()
        self.pump_run_start: Optional[float] = None  # track pump run duration

    async def handle_sensor(self, msg: SensorDataMessage) -> None:
        async with self._lock:
            self.boards["SENS"].update_seen(msg.rssi)
            # update levels
            for item in msg.items:
                if item.id.upper() == "TANQUE":
                    # store raw distance and percentage level
                    self.tanque_pct = item.level_pct
                    self.tanque_mm = item.dist_mm
                elif item.id.upper() == "TOLVA":
                    self.tolva_pct = item.level_pct
                    self.tolva_mm = item.dist_mm

    async def handle_ack(self, msg: AckMessage) -> None:
        async with self._lock:
            board_id = msg.from_
            if board_id in self.boards:
                self.boards[board_id].update_seen()
                if board_id == "ACT":
                    self.boards[board_id].pump_state = (msg.state == "ON")
                    # track pump run start/stop
                    if msg.state == "ON":
                        if self.pump_run_start is None:
                            self.pump_run_start = time.time()
                    else:
                        self.pump_run_start = None

    async def handle_status(self, msg: StatusMessage) -> None:
        async with self._lock:
            board_id = msg.board
            if board_id in self.boards:
                self.boards[board_id].online = (msg.state != "OFFLINE")

    def get_state_snapshot(self) -> Dict[str, Any]:
        """Return a snapshot of the current state."""
        snapshot = {
            "tanque_pct": self.tanque_pct,
            "tolva_pct": self.tolva_pct,
            "tanque_mm": self.tanque_mm,
            "tolva_mm": self.tolva_mm,
            "pump_state": self.boards["ACT"].pump_state,
            "boards": {
                k: {
                    "online": v.online,
                    "last_ts": v.last_seen,
                    "rssi": v.rssi,
                    "pump_state": v.pump_state
                }
                for k, v in self.boards.items()
            }
        }
        return snapshot

    async def monitor_boards(self, send_offline_cb) -> None:
        """Monitor boards and trigger callbacks if they go offline."""
        SENSOR_TIMEOUT = 5  # seconds
        ACT_TIMEOUT = 5
        MASTER_TIMEOUT = 10
        while True:
            now = time.time()
            async with self._lock:
                for name, timeout in {"SENS": SENSOR_TIMEOUT, "ACT": ACT_TIMEOUT, "MASTER": MASTER_TIMEOUT}.items():
                    board = self.boards[name]
                    if board.online and (now - board.last_seen > timeout):
                        board.online = False
                        await send_offline_cb(name)
                        if name == "ACT":
                            board.pump_state = False
                            self.pump_run_start = None
            await asyncio.sleep(1)


# Control logic for automatic pump control

class ControlLogic:
    def __init__(self, board_manager: BoardManager, send_command_cb):
        self.manager = board_manager
        self.send_command = send_command_cb
        self.last_action_time: float = 0.0
        self.pump_state: bool = False

    async def control_loop(self) -> None:
        while True:
            await asyncio.sleep(0.2)  # check 5 times per second
            config = self.manager.config
            if config.get("mode") != "AUTO":
                continue
            snapshot = self.manager.get_state_snapshot()
            tanque = snapshot["tanque_pct"]
            tolva = snapshot["tolva_pct"]
            pump_state = snapshot["pump_state"]
            permit = tanque is not None and tanque > 0.0
            if tolva is None:
                continue
            sp = config["SP_tolva"]
            hyster = config["hysteresis"]
            low_th = sp - hyster
            high_th = sp + hyster
            now = time.time()
            # rules
            if permit and not pump_state and tolva < low_th:
                # start pump
                await self.send_command(CommandMessage(dst="ACT", cmd="PUMP", value="ON", ttl_ms=int(config["max_run_s"] * 1000), reason="AUTO_START"))
                self.last_action_time = now
            elif (not permit or tolva > high_th) and pump_state and (now - self.last_action_time) > config["min_on_s"]:
                # stop pump
                await self.send_command(CommandMessage(dst="ACT", cmd="PUMP", value="OFF", ttl_ms=3000, reason="AUTO_STOP"))
                self.last_action_time = now


# ---------------------------------------------------------------------------
# FastAPI application

app = FastAPI(title="Professional Tank Level Control Server")

# Load initial config and state
CONFIG = load_json(CONFIG_FILE, default_config)
STATE_DATA = load_json(STATE_FILE, {})

# Instantiate board manager and control logic
board_manager = BoardManager(CONFIG)
control_logic = ControlLogic(board_manager, lambda cmd: asyncio.create_task(send_command_to_master(cmd)))

# Set up static files for UI
static_dir = Path(__file__).parent / "static"
static_dir.mkdir(exist_ok=True)
app.mount("/static", StaticFiles(directory=static_dir), name="static")

bridge_ws: Optional[WebSocket] = None  # WebSocket to master
client_sockets: List[WebSocket] = []   # connected UI clients

# Logging configuration
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s %(message)s")


@app.get("/")
async def index() -> HTMLResponse:
    index_path = static_dir / "index.html"
    if not index_path.exists():
        # write a default UI if missing
        html = """<!DOCTYPE html>
<html><head><meta charset='UTF-8'><title>Tank Level Control</title>
<style>body{font-family:Arial;margin:20px;}#status{margin-bottom:10px;} .online{color:green;} .offline{color:red;}</style>
</head><body><h1>Tank Level Control</h1>
<div id='status'>Connecting...</div>
<div id='controls'>
  Mode: <select id='mode'>
    <option value='AUTO'>AUTO</option>
    <option value='MANUAL'>MANUAL</option>
  </select>
  <button id='pumpOn'>Pump ON</button>
  <button id='pumpOff'>Pump OFF</button>
  <br/><label>SP tolva (%): <input type='number' id='sp' value='60'></label>
  <label>Histeresis (%): <input type='number' id='hyster' value='5'></label>
  <button id='saveConfig'>Save Config</button>
</div>
<div id='levels'></div>
<script>
const ws = new WebSocket(`ws://${location.host}/ws/client`);
const statusEl = document.getElementById('status');
const levelsEl = document.getElementById('levels');
const modeEl = document.getElementById('mode');
ws.onopen = () => { statusEl.textContent = 'Connected to server'; };
ws.onclose = () => { statusEl.textContent = 'Disconnected'; };
ws.onmessage = (ev) => {
  const msg = JSON.parse(ev.data);
  if (msg.type === 'state') {
    const s = msg.state;
    const boards = s.boards;
    let html = '';
    html += `<p>SENS: <span class='${boards.SENS.online?'online':'offline'}'>${boards.SENS.online?'Online':'Offline'}</span></p>`;
    html += `<p>ACT: <span class='${boards.ACT.online?'online':'offline'}'>${boards.ACT.online?'Online':'Offline'}</span></p>`;
    html += `<p>MASTER: <span class='${boards.MASTER.online?'online':'offline'}'>${boards.MASTER.online?'Online':'Offline'}</span></p>`;
    if (s.tanque_pct != null) html += `<p>Tanque: ${s.tanque_pct.toFixed(1)} %</p>`;
    if (s.tolva_pct != null) html += `<p>Tolva: ${s.tolva_pct.toFixed(1)} %</p>`;
    html += `<p>Pump: ${s.pump_state?'ON':'OFF'}</p>`;
    levelsEl.innerHTML = html;
    // update controls from config
    modeEl.value = msg.config.mode;
    document.getElementById('sp').value = msg.config.SP_tolva;
    document.getElementById('hyster').value = msg.config.hysteresis;
  }
};
modeEl.onchange = () => {
  fetch('/mode', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({mode:modeEl.value})});
};
document.getElementById('pumpOn').onclick = () => {
  fetch('/command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({cmd:'PUMP_ON'})});
};
document.getElementById('pumpOff').onclick = () => {
  fetch('/command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({cmd:'PUMP_OFF'})});
};
document.getElementById('saveConfig').onclick = () => {
  const sp = parseFloat(document.getElementById('sp').value);
  const hyster = parseFloat(document.getElementById('hyster').value);
  fetch('/config', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({SP_tolva:sp, hysteresis:hyster})});
};
</script></body></html>"""
        index_path.write_text(html)
    return HTMLResponse(index_path.read_text())


@app.get("/status")
async def status() -> JSONResponse:
    return JSONResponse({"state": board_manager.get_state_snapshot(), "config": CONFIG})


@app.post("/config")
async def set_config(data: Dict[str, float]):
    # update config keys that are allowed
    allowed = {
        "empty_tanque_mm", "full_tanque_mm", "empty_tolva_mm", "full_tolva_mm",
        "SP_tolva", "hysteresis", "min_on_s", "min_off_s", "max_run_s", "pulse_rate"
    }
    for k, v in data.items():
        if k in allowed:
            CONFIG[k] = float(v)
    save_json(CONFIG_FILE, CONFIG)
    # if pulse rate changed, send to actuator
    if "pulse_rate" in data:
        await send_pulse_rate(float(data["pulse_rate"]))
    # forward calibration values to sensor node
    if bridge_ws:
        # Build payload with calibration distances
        payload: Dict[str, float] = {
            "empty_tanque_mm": CONFIG["empty_tanque_mm"],
            "full_tanque_mm":  CONFIG["full_tanque_mm"],
            "empty_tolva_mm":  CONFIG["empty_tolva_mm"],
            "full_tolva_mm":   CONFIG["full_tolva_mm"]
        }
        cfg_msg = ConfigSetMessage(target="SENS", payload=payload)
        await bridge_ws.send_text(cfg_msg.json(by_alias=True))
    await broadcast_state()
    return {"result": "ok"}


@app.post("/mode")
async def set_mode(data: Dict[str, str]):
    mode = data.get("mode", "AUTO").upper()
    if mode not in {"AUTO", "MANUAL"}:
        raise HTTPException(400, detail="Invalid mode")
    CONFIG["mode"] = mode
    save_json(CONFIG_FILE, CONFIG)
    await broadcast_state()
    return {"result": "ok"}


@app.post("/command")
async def manual_command(data: Dict[str, str]):
    if CONFIG.get("mode") != "MANUAL":
        raise HTTPException(400, detail="System not in MANUAL mode")
    cmd = data.get("cmd")
    if cmd not in {"PUMP_ON", "PUMP_OFF"}:
        raise HTTPException(400, detail="Invalid command")
    if not bridge_ws:
        raise HTTPException(500, detail="Master not connected")
    value = "ON" if cmd == "PUMP_ON" else "OFF"
    msg = CommandMessage(dst="ACT", cmd="PUMP", value=value, ttl_ms=int(CONFIG["max_run_s"] * 1000) if value == "ON" else 3000, reason="MANUAL")
    await bridge_ws.send_text(msg.json(by_alias=True))
    return {"result": "ok"}


# Endpoint to set pulse rate (pulses per minute) for actuator.
# Accepts JSON {"pulse_rate": float}. Valid range: 0.02 to 3.0 pulses/minute.
@app.post("/pulse")
async def set_pulse(data: Dict[str, float]):
    if "pulse_rate" not in data:
        raise HTTPException(400, detail="Missing pulse_rate")
    rate = float(data["pulse_rate"])
    if rate < 0.02 or rate > 3.0:
        raise HTTPException(400, detail="pulse_rate out of range (0.02 - 3.0)")
    CONFIG["pulse_rate"] = rate
    save_json(CONFIG_FILE, CONFIG)
    # send to actuator
    await send_pulse_rate(rate)
    await broadcast_state()
    return {"result": "ok"}


# WebSocket endpoint for master ESP32
@app.websocket("/ws/bridge")
async def ws_bridge(ws: WebSocket):
    global bridge_ws
    await ws.accept()
    bridge_ws = ws
    logging.info("Master connected via WebSocket")
    board_manager.boards["MASTER"].update_seen()
    await broadcast_state()
    try:
        while True:
            data = await ws.receive_text()
            try:
                parsed = json.loads(data)
            except json.JSONDecodeError:
                continue
            msg_type = parsed.get("type")
            # heartbeats are ignored
            if msg_type == "hb":
                board_manager.boards["MASTER"].update_seen()
                continue
            # sensor data
            if msg_type == "sensor_data":
                try:
                    msg = SensorDataMessage(**parsed)
                    await board_manager.handle_sensor(msg)
                    await broadcast_state()
                    continue
                except Exception as e:
                    logging.warning("Invalid sensor_data: %s", e)
            # ack
            if msg_type == "ack":
                try:
                    msg = AckMessage(**parsed)
                    await board_manager.handle_ack(msg)
                    await broadcast_state()
                    continue
                except Exception as e:
                    logging.warning("Invalid ack: %s", e)
            # status messages
            if msg_type == "status":
                try:
                    msg = StatusMessage(**parsed)
                    await board_manager.handle_status(msg)
                    await broadcast_state()
                    continue
                except Exception as e:
                    logging.warning("Invalid status: %s", e)
            # forward any other messages to UI clients
            await broadcast_message({"from_master": parsed})
    except WebSocketDisconnect:
        logging.info("Master disconnected")
    finally:
        bridge_ws = None
        await broadcast_state()


# WebSocket endpoint for clients
@app.websocket("/ws/client")
async def ws_client(ws: WebSocket):
    await ws.accept()
    client_sockets.append(ws)
    await send_initial_state(ws)
    try:
        while True:
            await ws.receive_text()  # we ignore client messages; they use REST
    except WebSocketDisconnect:
        pass
    finally:
        client_sockets.remove(ws)


# Utility: broadcast full state to all clients
async def broadcast_state() -> None:
    state_snapshot = board_manager.get_state_snapshot()
    message = json.dumps({"type": "state", "state": state_snapshot, "config": CONFIG})
    for ws in list(client_sockets):
        try:
            await ws.send_text(message)
        except Exception:
            client_sockets.remove(ws)


# Utility: send initial state/config to a new client
async def send_initial_state(ws: WebSocket) -> None:
    state_snapshot = board_manager.get_state_snapshot()
    await ws.send_text(json.dumps({"type": "state", "state": state_snapshot, "config": CONFIG}))


# Utility: broadcast arbitrary message (e.g., logs) to clients
async def broadcast_message(msg: Dict[str, Any]) -> None:
    data = json.dumps({"type": "msg", "payload": msg})
    for ws in list(client_sockets):
        try:
            await ws.send_text(data)
        except Exception:
            client_sockets.remove(ws)


# Callback to send commands to master
async def send_command_to_master(cmd: CommandMessage) -> None:
    if bridge_ws:
        await bridge_ws.send_text(cmd.json(by_alias=True))

# Helper to send new pulse rate to actuator
async def send_pulse_rate(rate: float) -> None:
    """Send pulse rate configuration to actuator via master."""
    msg = ConfigSetMessage(target="ACT", payload={"pulse_rate": rate})
    if bridge_ws:
        await bridge_ws.send_text(msg.json(by_alias=True))


@app.on_event("startup")
async def on_startup() -> None:
    # start monitoring tasks
    asyncio.create_task(board_manager.monitor_boards(on_board_offline))
    asyncio.create_task(control_logic.control_loop())
    logging.info("Server startup complete")


# Callback when a board goes offline
async def on_board_offline(board_name: str) -> None:
    logging.warning("Board %s offline", board_name)
    # when actuator goes offline, ensure pump is off
    if board_name == "ACT":
        # send stop command (no TTL because board offline, so not necessary)
        # we still update state
        board_manager.boards["ACT"].pump_state = False
    await broadcast_state()


@app.on_event("shutdown")
async def on_shutdown() -> None:
    save_json(STATE_FILE, board_manager.get_state_snapshot())
    logging.info("Server shutdown; state saved")
