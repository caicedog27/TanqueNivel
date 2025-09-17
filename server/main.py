
import asyncio, json, time, os, re
from typing import Dict, Any, Optional, List
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel, Field, field_validator

APP_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.abspath(os.path.join(APP_DIR, ".."))
WEB_DIR = os.path.join(ROOT_DIR, "webui")
DB_DIR = os.path.join(ROOT_DIR, "db")
os.makedirs(DB_DIR, exist_ok=True)

CFG_PATH = os.path.join(DB_DIR, "config.json")
EVENTS_PATH = os.path.join(DB_DIR, "events.log")
HISTORY_PATH = os.path.join(DB_DIR, "history.jsonl")
PROFILES_PATH = os.path.join(DB_DIR, "profiles.json")
BOARDS_CACHE_PATH = os.path.join(DB_DIR, "boards.json")

def log_event(msg: str):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    try:
        with open(EVENTS_PATH, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception as e:
        print("log error:", e)

def clamp(v,a,b): return max(a, min(b, v))

class SensorCal(BaseModel):
    empty_mm: int = Field(800, ge=50, le=10000)
    full_mm: int = Field(200, ge=10, le=9999)
    target_pct: float = Field(65.0, ge=5, le=95)
    hysteresis_pct: float = Field(5.0, ge=1, le=20)
    @field_validator("full_mm")
    @classmethod
    def check_order(cls, v, info):
        empty = info.data.get("empty_mm")
        if empty is not None and v >= empty:
            raise ValueError("full_mm debe ser menor que empty_mm")
        return v

class AutoCfg(BaseModel):
    anti_spill_margin_pct: float = Field(1.2, ge=0.0, le=10.0)
    ema_alpha: float = Field(0.2, ge=0.05, le=0.9)
    tank_low_lock_pct: float = Field(15.0, ge=0.0, le=50.0)

class TankCfg(BaseModel):
    cal: SensorCal = Field(default_factory=SensorCal)

class HopperCfg(BaseModel):
    cal: SensorCal = Field(default_factory=SensorCal)

class NetCfg(BaseModel):
    allowed_origins: List[str] = Field(default_factory=lambda: ["http://192.168.1.68:8000","http://localhost:8000","http://127.0.0.1:8000"])
    board_tokens: Dict[str,str] = Field(default_factory=lambda: {
        "ACT-01": "test_ws_board_2025_ABCDEF",
        "SENS_TANQUE": "test_ws_board_2025_ABCDEF",
        "SENS_TOLVA": "test_ws_board_2025_ABCDEF",
    })
    board_auto_register: bool = True
    api_key: str = "DEVKEY123"

class AppCfg(BaseModel):
    tank: TankCfg = Field(default_factory=TankCfg)
    hopper: HopperCfg = Field(default_factory=HopperCfg)
    auto: AutoCfg = Field(default_factory=AutoCfg)
    net: NetCfg = Field(default_factory=NetCfg)

class SensorState(BaseModel):
    raw_mm: float = 0.0
    filtered_pct: float = 0.0
    last_ts: float = 0.0

class AutoModel(BaseModel):
    ema_run_s: float = 0.0
    ema_slope_pct_s: float = 0.0
    ema_inertia_s: float = 0.0
    ema_overshoot_pct: float = 0.0

class ActuatorState(BaseModel):
    board_id: str = "ACT-01"
    pump_on: bool = False
    on_ms: int = 100
    off_ms: int = 233
    pulses_total: int = 0
    runtime_ms_total: int = 0
    last_command_ts: float = 0.0
    ttl_ms: int = 30000

class BoardsView(BaseModel):
    board_id: str
    name: str
    kind: str
    online: bool
    last_ip: Optional[str] = None
    last_seen: Optional[float] = None
    ws_url: Optional[str] = None
    token: Optional[str] = None

class AppState(BaseModel):
    mode: str = "MANUAL_OFF"
    actuator: ActuatorState = Field(default_factory=ActuatorState)
    tank: SensorState = Field(default_factory=SensorState)
    hopper: SensorState = Field(default_factory=SensorState)
    auto_model: AutoModel = Field(default_factory=AutoModel)

def load_json(path, default):
    try:
        with open(path, "r", encoding="utf-8") as f: return json.load(f)
    except: return default

def save_json(path, data):
    tmp = path + ".tmp"
    with open(tmp,"w",encoding="utf-8") as f: json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, path)

CFG = AppCfg(**load_json(CFG_PATH, {}))
if not os.path.exists(CFG_PATH): save_json(CFG_PATH, json.loads(CFG.model_dump_json()))
STATE = AppState()

app = FastAPI(title="TanqueNivel Industrial", version="UX13")
app.add_middleware(CORSMiddleware, allow_origins=CFG.net.allowed_origins or ["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")

def normalize_origin(o:str)->str:
    o=o.strip()
    if not re.match(r"^https?://", o): o = "http://" + o
    return o

from fastapi import Header
def require_api_key(x_api_key: Optional[str] = Header(None)):
    if not x_api_key or x_api_key != CFG.net.api_key:
        raise HTTPException(401, "API key inválida")
    return True

@app.get("/", response_class=HTMLResponse)
async def index(): return FileResponse(os.path.join(WEB_DIR, "index.html"))

@app.get("/api/diag")
async def api_diag(_:bool=Depends(require_api_key)):
    return {"alive": True, "version": app.version, "allowed_origins": CFG.net.allowed_origins}

def pct_from_mm(mm:float, cal:SensorCal)->float:
    rng = float(cal.empty_mm - cal.full_mm)
    if rng <= 0: return 0.0
    return clamp((cal.empty_mm - float(mm)) * 100.0 / rng, 0.0, 100.0)

@app.get("/api/state")
async def api_state(_:bool=Depends(require_api_key)):
    return {"cfg": json.loads(CFG.model_dump_json()), "state": json.loads(STATE.model_dump_json()), "boards": [{"board_id":bid,"online":(bid in BOARDS),"rssi": (BOARDS[bid].rssi if bid in BOARDS else None)} for bid in CFG.net.board_tokens.keys()]}

# ---------- Boards/Equipos ----------

class BoardsRegisterReq(BaseModel):
    board_id: str
    token: str
    name: Optional[str] = None
    kind: Optional[str] = None

@app.post("/api/boards")
async def boards_register(req: BoardsRegisterReq, _:bool=Depends(require_api_key)):
    CFG.net.board_tokens[req.board_id] = req.token
    save_json(CFG_PATH, json.loads(CFG.model_dump_json()))
    cache = load_json(BOARDS_CACHE_PATH, {})
    meta = cache.get(req.board_id, {})
    if req.name: meta["name"] = req.name
    if req.kind: meta["kind"] = req.kind
    cache[req.board_id] = meta
    save_json(BOARDS_CACHE_PATH, cache)
    log_event(f"board registered {req.board_id}")
    return {"ok": True}

@app.delete("/api/boards/{board_id}")
async def boards_delete(board_id: str, _:bool=Depends(require_api_key)):
    if board_id in CFG.net.board_tokens:
        del CFG.net.board_tokens[board_id]
        save_json(CFG_PATH, json.loads(CFG.model_dump_json()))
    cache = load_json(BOARDS_CACHE_PATH, {})
    cache.pop(board_id, None)
    save_json(BOARDS_CACHE_PATH, cache)
    log_event(f"board deleted {board_id}")
    return {"ok": True}

@app.get("/api/boards")
async def api_boards(_:bool=Depends(require_api_key)):
    # Build view from expected tokens + cache + online
    cache = load_json(BOARDS_CACHE_PATH, {})
    host = CFG.net.allowed_origins[0] if CFG.net.allowed_origins else "http://192.168.1.68:8000"
    # derive host/port for ws_url
    m = re.match(r"^https?://([^/]+)", host)
    ws_host = m.group(1) if m else "192.168.1.68:8000"
    scheme_host = f"ws://{ws_host}"
    out = []
    # expected
    for bid, token in CFG.net.board_tokens.items():
        meta = cache.get(bid, {})
        online = bid in BOARDS
        name = meta.get("name", bid)
        kind = meta.get("kind", "UNKNOWN")
        last_ip = BOARDS[bid].ip if online else meta.get("ip")
        last_seen = BOARDS[bid].last_seen if online else meta.get("last_seen")
        out.append(BoardsView(board_id=bid, name=name, kind=kind, online=online,
                              last_ip=last_ip, last_seen=last_seen,
                              ws_url=f"{scheme_host}/ws/board/{bid}?token={token}", token=token).model_dump())
    # plus online but not expected
    for bid, c in BOARDS.items():
        if bid not in CFG.net.board_tokens:
            out.append(BoardsView(board_id=bid, name=c.name, kind=c.kind, online=True,
                                  last_ip=c.ip, last_seen=c.last_seen,
                                  ws_url=f"{scheme_host}/ws/board/{bid}?token=<unknown>", token=None).model_dump())
    return {"boards": out}

# ---------- History & Events ----------

@app.get("/api/wifi")
async def api_wifi(_:bool=Depends(require_api_key)):
    out=[]
    cache = load_json(BOARDS_CACHE_PATH, {})
    for bid, token in CFG.net.board_tokens.items():
        online = bid in BOARDS
        rssi = BOARDS[bid].rssi if online else cache.get(bid,{}).get("rssi")
        out.append({"board_id": bid, "online": online, "rssi": rssi})
    return {"items": out}
@app.get("/api/history")
async def api_history(n:int=600, _:bool=Depends(require_api_key)):
    items=[]
    try:
        with open(HISTORY_PATH,"r",encoding="utf-8") as f:
            for line in f:
                line=line.strip()
                if line: items.append(json.loads(line))
    except: pass
    return {"items": items[-n:]}

@app.get("/api/events/tail")
async def api_events_tail(n:int=200, _:bool=Depends(require_api_key)):
    lines=[]
    try:
        with open(EVENTS_PATH,"r",encoding="utf-8") as f: lines=f.readlines()
    except: pass
    return {"lines":[x.strip() for x in lines[-n:]]}

# ---------- Actuator control ----------
class DutyReq(BaseModel):
    on_ms:int=Field(100, ge=10, le=10000)
    off_ms:int=Field(233, ge=0, le=20000)

@app.post("/api/actuator/duty")
async def api_duty(req:DutyReq, _:bool=Depends(require_api_key)):
    STATE.actuator.on_ms = req.on_ms
    STATE.actuator.off_ms = req.off_ms
    if STATE.actuator.board_id in BOARDS:
        await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:set","on_ms":req.on_ms,"off_ms":req.off_ms})
    log_event(f"duty set on={req.on_ms} off={req.off_ms}")
    return {"ok": True, "trace": int(time.time()*1000)}

class PumpReq(BaseModel):
    value:str="ON"
    ttl_ms:int=Field(30000, ge=500, le=600000)
    reason:str="manual"

@app.post("/api/pump")
async def api_pump(req:PumpReq, _:bool=Depends(require_api_key)):
    v=req.value.upper().strip()
    if v not in ("ON","OFF"): raise HTTPException(400,"value debe ser ON/OFF")
    STATE.actuator.pump_on = (v=="ON")
    STATE.actuator.last_command_ts = time.time()
    STATE.actuator.ttl_ms = req.ttl_ms
    if STATE.actuator.board_id in BOARDS:
        await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:pump","value":v})
    log_event(f"pump {v} ttl={req.ttl_ms} reason={req.reason}")
    return {"ok": True, "trace": int(time.time()*1000)}

@app.post("/api/mode/{mode}")
async def api_mode(mode:str, _:bool=Depends(require_api_key)):
    mode=mode.upper()
    if mode not in ("AUTO","MANUAL_OFF"): raise HTTPException(400,"modo inválido")
    STATE.mode = mode
    if mode=="MANUAL_OFF" and STATE.actuator.board_id in BOARDS:
        await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:pump","value":"OFF"})
    log_event(f"mode set {mode}")
    return {"ok": True, "trace": int(time.time()*1000)}

# ---------- Profiles ----------
@app.get("/api/profiles")
async def profiles(_:bool=Depends(require_api_key)):
    return load_json(PROFILES_PATH, {"profiles":[]})

class ProfileReq(BaseModel):
    name:str; on_ms:int; off_ms:int
@app.post("/api/profiles")
async def profiles_add(req:ProfileReq, _:bool=Depends(require_api_key)):
    data = load_json(PROFILES_PATH, {"profiles":[]})
    data["profiles"] = [p for p in data["profiles"] if p["name"]!=req.name] + [{"name":req.name,"on_ms":req.on_ms,"off_ms":req.off_ms}]
    save_json(PROFILES_PATH, data); log_event(f"profile saved {req.name}"); return {"ok":True}
@app.post("/api/profiles/apply/{name}")
async def profiles_apply(name:str, _:bool=Depends(require_api_key)):
    data = load_json(PROFILES_PATH, {"profiles":[]})
    for p in data["profiles"]:
        if p["name"]==name:
            STATE.actuator.on_ms=int(p["on_ms"]); STATE.actuator.off_ms=int(p["off_ms"])
            if STATE.actuator.board_id in BOARDS:
                await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:set","on_ms":STATE.actuator.on_ms,"off_ms":STATE.actuator.off_ms})
            log_event(f"profile applied {name}"); return {"ok":True}
    raise HTTPException(404,"perfil no encontrado")
@app.delete("/api/profiles/{name}")
async def profiles_del(name:str, _:bool=Depends(require_api_key)):
    data = load_json(PROFILES_PATH, {"profiles":[]})
    n0 = len(data["profiles"]); data["profiles"] = [p for p in data["profiles"] if p["name"]!=name]
    save_json(PROFILES_PATH, data)
    if len(data["profiles"])<n0: log_event(f"profile deleted {name}"); return {"ok":True}
    raise HTTPException(404,"perfil no encontrado")

# ---------- Config ----------
class PatchCfg(BaseModel):
    allowed_origins: Optional[List[str]] = None
    api_key: Optional[str] = None
    hopper: Optional[Dict[str, Any]] = None
    tank: Optional[Dict[str, Any]] = None
    auto: Optional[Dict[str, Any]] = None

@app.patch("/api/config")
async def patch_cfg(req:PatchCfg, _:bool=Depends(require_api_key)):
    changes=[]
    if req.allowed_origins is not None:
        CFG.net.allowed_origins = [normalize_origin(x) for x in req.allowed_origins]; changes.append("allowed_origins")
    if req.api_key:
        CFG.net.api_key = req.api_key; changes.append("api_key")
    if req.hopper:
        cal = CFG.hopper.cal.model_dump(); cal.update(req.hopper.get("cal", {})); CFG.hopper.cal = SensorCal(**cal); changes.append("hopper.cal")
    if req.tank:
        cal = CFG.tank.cal.model_dump(); cal.update(req.tank.get("cal", {})); CFG.tank.cal = SensorCal(**cal); changes.append("tank.cal")
    if req.auto:
        a = CFG.auto.model_dump(); a.update(req.auto); CFG.auto = AutoCfg(**a); changes.append("auto")
    save_json(CFG_PATH, json.loads(CFG.model_dump_json()))
    log_event("config patched: " + ",".join(changes))
    return {"ok":True,"trace":int(time.time()*1000)}

# ---------- WebSocket Boards ----------
class WSConn:
    def __init__(self, ws:WebSocket, board_id:str, kind:str, name:str, ip:str):
        self.ws=ws; self.board_id=board_id; self.kind=kind; self.name=name; self.ip=ip; self.last_seen=time.time(); self.rssi=None; self.fw=None; self.mac=None; self.uptime_s=None
BOARDS: Dict[str, WSConn] = {}

@app.websocket("/ws/board/{board_id}")
async def ws_board(websocket:WebSocket, board_id:str):
    await websocket.accept()
    ip = getattr(websocket.client, "host", "unknown")
    try:
        msg = await asyncio.wait_for(websocket.receive_json(), timeout=5.0)
    except Exception:
        await websocket.close(code=4400); log_event(f"WS {board_id} no_hello"); return
    if msg.get("type")!="hello":
        await websocket.close(code=4400); log_event(f"WS {board_id} bad_hello"); return
    token = msg.get("token",""); expected = CFG.net.board_tokens.get(board_id)
    if expected is None and CFG.net.board_auto_register:
        CFG.net.board_tokens[board_id]=token; save_json(CFG_PATH, json.loads(CFG.model_dump_json())); expected=token; log_event(f"auto-registered {board_id}")
    if token != expected:
        await websocket.close(code=4401); log_event(f"WS {board_id} token_mismatch"); return
    conn = WSConn(websocket, board_id, msg.get("kind","UNKNOWN"), msg.get("name",board_id), ip)
    conn.fw = msg.get("fw")
    conn.mac = msg.get("mac")
    try:
        conn.rssi = int(msg.get("rssi")) if msg.get("rssi") is not None else None
    except:
        conn.rssi = None
    try:
        conn.uptime_s = int(msg.get("uptime_s")) if msg.get("uptime_s") is not None else None
    except:
        conn.uptime_s = None
    BOARDS[board_id] = conn
    cache = load_json(BOARDS_CACHE_PATH, {}); cache[board_id]={"name":conn.name,"kind":conn.kind,"ip":ip,"last_seen":time.time()}; save_json(BOARDS_CACHE_PATH, cache)
    log_event(f"WS accepted {board_id} {conn.kind}")
    # Update cache metadata
    try:
        cache = load_json(BOARDS_CACHE_PATH, {})
        cache[board_id] = {"name": conn.name, "kind": conn.kind, "ip": conn.ip, "last_seen": conn.last_seen, "rssi": conn.rssi, "fw": conn.fw, "mac": conn.mac, "uptime_s": conn.uptime_s}
        save_json(BOARDS_CACHE_PATH, cache)
    except Exception as e:
        log_event(f"cache update err {e}")
    # Push estado actual del actuador si aplica
    if board_id == STATE.actuator.board_id and conn.kind.upper()=="ACTUATOR":
        await websocket.send_json({"type":"actuator:set","on_ms":STATE.actuator.on_ms,"off_ms":STATE.actuator.off_ms})
        await websocket.send_json({"type":"actuator:pump","value":"ON" if STATE.actuator.pump_on else "OFF"})
    try:
        while True:
            data = await websocket.receive_json()
            t = data.get("type"); conn.last_seen=time.time()
            if t=="sensor":
                sid=data.get("sensor_id"); mm=float(data.get("mm",0.0)); ts=time.time()
                if "rssi" in data:
                    try: BOARDS[board_id].rssi = int(data.get("rssi"))
                    except: pass
                if sid=="tank":
                    STATE.tank.raw_mm=mm; STATE.tank.filtered_pct=pct_from_mm(mm, CFG.tank.cal); STATE.tank.last_ts=ts
                elif sid=="hopper":
                    STATE.hopper.raw_mm=mm; STATE.hopper.filtered_pct=pct_from_mm(mm, CFG.hopper.cal); STATE.hopper.last_ts=ts
            elif t=="ack":
                log_event(f"ACK {board_id} {data.get('cmd')}")
            elif t=="actuator:stats":
                STATE.actuator.pulses_total=int(data.get("pulses_total",STATE.actuator.pulses_total))
                STATE.actuator.runtime_ms_total=int(data.get("runtime_ms_total",STATE.actuator.runtime_ms_total))
                if "rssi" in data:
                    try: BOARDS[board_id].rssi = int(data.get("rssi"))
                    except: pass
    except WebSocketDisconnect:
        try:
            cache = load_json(BOARDS_CACHE_PATH, {})
            meta = cache.get(board_id, {})
            meta.update({"ip": ip, "last_seen": time.time(), "rssi": BOARDS.get(board_id).rssi if BOARDS.get(board_id) else None})
            cache[board_id] = meta
            save_json(BOARDS_CACHE_PATH, cache)
        except Exception as e:
            log_event(f"cache persist err {e}")
        pass
    except Exception as e:
        log_event(f"WS error {board_id}: {e}")
    finally:
        BOARDS.pop(board_id, None); log_event(f"WS closed {board_id}")

# ---------- Background ----------
async def history_loop():
    last_p=0; last_r=0
    while True:
        await asyncio.sleep(2.0)
        # TTL safety
        if STATE.actuator.pump_on and STATE.actuator.ttl_ms>0 and (time.time()-STATE.actuator.last_command_ts)*1000 > STATE.actuator.ttl_ms:
            STATE.actuator.pump_on=False
            if STATE.actuator.board_id in BOARDS:
                try: await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:pump","value":"OFF"})
                except: pass
            log_event("TTL cutoff pump OFF")
        pulses_delta=max(0, STATE.actuator.pulses_total-last_p); runtime_delta=max(0, STATE.actuator.runtime_ms_total-last_r)
        last_p=STATE.actuator.pulses_total; last_r=STATE.actuator.runtime_ms_total
        item={"ts":time.time(),"pulses":float(pulses_delta),"runtime_s":float(runtime_delta)/1000.0,
              "pump_on":1.0 if STATE.actuator.pump_on else 0.0, "tank_pct":float(STATE.tank.filtered_pct),
              "hopper_pct":float(STATE.hopper.filtered_pct),"on_ms":int(STATE.actuator.on_ms),"off_ms":int(STATE.actuator.off_ms),
              "hz": (1000.0/max(1.0, STATE.actuator.on_ms + STATE.actuator.off_ms)) if STATE.actuator.pump_on else 0.0,
              "duty": (100.0 * (STATE.actuator.on_ms / max(1.0, (STATE.actuator.on_ms + STATE.actuator.off_ms)))) }
        try:
            with open(HISTORY_PATH,"a",encoding="utf-8") as f: f.write(json.dumps(item)+"\n")
        except Exception as e: log_event(f"history write err {e}")
        # AUTO
        if STATE.mode=="AUTO":
            targ = CFG.hopper.cal.target_pct; hyst = CFG.hopper.cal.hysteresis_pct; guard = CFG.auto.anti_spill_margin_pct
            tank_ok = STATE.tank.filtered_pct > CFG.auto.tank_low_lock_pct
            if tank_ok and STATE.hopper.filtered_pct < (targ - hyst):
                if not STATE.actuator.pump_on:
                    STATE.actuator.pump_on=True; STATE.actuator.last_command_ts=time.time()
                    if STATE.actuator.board_id in BOARDS:
                        try: await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:pump","value":"ON"})
                        except: pass
                    log_event("AUTO: pump ON")
            if STATE.hopper.filtered_pct >= (targ + guard):
                if STATE.actuator.pump_on:
                    STATE.actuator.pump_on=False
                    if STATE.actuator.board_id in BOARDS:
                        try: await BOARDS[STATE.actuator.board_id].ws.send_json({"type":"actuator:pump","value":"OFF"})
                        except: pass
                    log_event("AUTO: pump OFF")

@app.on_event("startup")
async def on_start():
    for p, init in [(EVENTS_PATH,""),(HISTORY_PATH,""),(PROFILES_PATH,'{"profiles":[]}'),(BOARDS_CACHE_PATH,"{}")]:
        if not os.path.exists(p):
            with open(p,"w",encoding="utf-8") as f: f.write(init)
    asyncio.create_task(history_loop())
    log_event("server started")
