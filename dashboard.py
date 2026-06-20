import json
import socketserver
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2

from jbot import LIDAR_ANGLE_OFFSET, LIDAR_MIN_RANGE, Robot, bucket, gst_pipeline, read_battery

PORT = 8000
WATCHDOG_TIMEOUT = 0.6

state = {"jpeg": None, "points": [], "battery": 0.0}
lock = threading.Lock()

robot = None
robot_lock = threading.Lock()
drive_state = {"last": 0.0, "moving": False}


def drive(direction, speed):
    if robot is None:
        return
    speed = max(0.0, min(1.0, speed))
    with robot_lock:
        if direction == "forward":
            robot.forward(speed)
        elif direction == "backward":
            robot.backward(speed)
        elif direction == "left":
            robot.left_turn(speed)
        elif direction == "right":
            robot.right_turn(speed)
        else:
            robot.stop()
            drive_state["moving"] = False
            return
        drive_state["moving"] = True
        drive_state["last"] = time.time()


def watchdog_worker():
    while True:
        time.sleep(0.1)
        if robot is None:
            continue
        with robot_lock:
            if drive_state["moving"] and time.time() - drive_state["last"] > WATCHDOG_TIMEOUT:
                robot.stop()
                drive_state["moving"] = False


def camera_worker():
    cap = cv2.VideoCapture(gst_pipeline(640, 360), cv2.CAP_GSTREAMER)
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        with lock:
            state["jpeg"] = jpg
        time.sleep(0.03)


def lidar_worker():
    from rplidar import RPLidar

    while True:
        try:
            lidar = RPLidar("/dev/ttyUSB0", baudrate=115200)
            for scan in lidar.iter_scans(max_buf_meas=2000):
                points = [[round(a, 1), int(d)] for (_, a, d) in scan if d > 0]
                with lock:
                    state["points"] = points
        except Exception:
            time.sleep(1.0)


def battery_worker():
    while True:
        try:
            v = read_battery()
        except Exception:
            v = 0.0
        with lock:
            state["battery"] = round(v, 2)
        time.sleep(2.0)


def sectors_from(points):
    buckets = {"front": [], "right": [], "back": [], "left": []}
    for a, d in points:
        buckets[bucket(a)].append(d / 1000.0)
    return {k: (round(min(v), 2) if v else None) for k, v in buckets.items()}


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>JetBot Dashboard</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600&family=Fira+Code:wght@400;500&display=swap" rel="stylesheet">
<style>
  :root{
    --bg:#f6f7f9; --surface:#ffffff; --border:#e6e8eb; --line:#eef0f2;
    --text:#14161a; --muted:#6b7280; --accent:#10b981; --danger:#ef4444;
    --radius:16px; --s1:8px; --s2:16px; --s3:24px;
    --shadow:0 1px 2px rgba(20,22,26,.04), 0 1px 3px rgba(20,22,26,.04);
    --mono:"Fira Code", ui-monospace, SFMono-Regular, Menlo, monospace;
    --sans:"Inter", system-ui, -apple-system, Segoe UI, sans-serif;
  }
  *{ box-sizing:border-box; }
  html,body{ margin:0; }
  body{ background:var(--bg); color:var(--text); font-family:var(--sans); font-size:16px; line-height:1.5; -webkit-font-smoothing:antialiased; }
  .topbar{ display:flex; align-items:center; gap:var(--s2); padding:var(--s2) var(--s3); background:var(--surface); border-bottom:1px solid var(--border); }
  .brand{ display:flex; align-items:center; gap:10px; font-weight:600; letter-spacing:.2px; }
  .brand svg{ color:var(--accent); }
  .status{ display:flex; align-items:center; gap:8px; font-size:13px; color:var(--muted); }
  .dot{ width:8px; height:8px; border-radius:50%; background:#c2c7cf; transition:background .2s ease; }
  .dot.live{ background:var(--accent); }
  .battery{ margin-left:auto; display:flex; align-items:center; gap:10px; font-size:14px; }
  .battery .v{ font-family:var(--mono); font-variant-numeric:tabular-nums; color:var(--text); }
  .bar{ width:120px; height:6px; background:var(--line); border-radius:99px; overflow:hidden; }
  .bar > span{ display:block; height:100%; background:var(--accent); border-radius:99px; transition:width .4s ease; }
  main{ max-width:1200px; margin:0 auto; padding:var(--s3); display:grid; grid-template-columns:1fr 1fr; gap:var(--s3); }
  @media (max-width:900px){ main{ grid-template-columns:1fr; padding:var(--s2); gap:var(--s2); } }
  .card{ background:var(--surface); border:1px solid var(--border); border-radius:var(--radius); padding:var(--s3); box-shadow:var(--shadow); }
  .card h2{ margin:0 0 var(--s2); font-size:12px; font-weight:600; letter-spacing:.8px; text-transform:uppercase; color:var(--muted); }
  img#cam{ width:100%; aspect-ratio:16/9; object-fit:cover; border-radius:12px; background:#0e0f12; display:block; }
  canvas#radar{ width:100%; max-width:420px; aspect-ratio:1/1; display:block; margin:0 auto; }
  .sectors{ display:grid; grid-template-columns:1fr 1fr; gap:var(--s1); margin-top:var(--s3); }
  .sector{ border:1px solid var(--border); border-radius:12px; padding:14px 16px; }
  .sector .k{ font-size:11px; letter-spacing:.6px; text-transform:uppercase; color:var(--muted); }
  .sector .val{ margin-top:4px; font-family:var(--mono); font-variant-numeric:tabular-nums; font-size:22px; font-weight:500; }
  .sector .val.near{ color:var(--danger); }
  footer{ max-width:1200px; margin:0 auto; padding:0 var(--s3) var(--s3); color:var(--muted); font-size:12px; font-family:var(--mono); }
  .controls{ grid-column:1 / -1; }
  .ctl-wrap{ display:flex; gap:36px; align-items:center; flex-wrap:wrap; }
  .pad{ display:grid; grid-template-columns:repeat(3,60px); grid-template-rows:repeat(3,60px); gap:10px;
        grid-template-areas:". up ." "left mid right" ". down ."; }
  .pad-btn{ display:flex; align-items:center; justify-content:center; min-width:44px; min-height:44px;
            border:1px solid var(--border); background:var(--surface); border-radius:14px; color:var(--text);
            cursor:pointer; touch-action:none; -webkit-user-select:none; user-select:none;
            transition:background .15s ease, border-color .15s ease, transform .1s ease; }
  .pad-btn:hover{ border-color:#cfd3d8; }
  .pad-btn:focus-visible{ outline:2px solid var(--accent); outline-offset:2px; }
  .pad-btn.active{ background:#ecfdf5; border-color:var(--accent); color:#065f46; transform:scale(.95); }
  .pad-btn.stop{ color:var(--danger); }
  .pad-btn.stop:active{ background:#fef2f2; border-color:var(--danger); }
  .speed{ flex:1; min-width:240px; }
  .speed label{ display:block; font-size:13px; color:var(--muted); margin-bottom:8px; }
  .speed label .num{ font-family:var(--mono); color:var(--text); }
  .speed input[type=range]{ width:100%; accent-color:var(--accent); height:24px; }
  .hint{ margin:12px 0 0; font-size:12px; color:var(--muted); }
  .nodrive{ font-size:13px; color:var(--danger); margin-top:8px; }
  @media (prefers-reduced-motion: reduce){ *{ transition:none !important; } }
</style>
</head>
<body>
<div class="topbar">
  <span class="brand">
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><rect x="4" y="8" width="16" height="11" rx="2"/><path d="M12 8V4"/><circle cx="9" cy="13" r="1"/><circle cx="15" cy="13" r="1"/></svg>
    JetBot Dashboard
  </span>
  <span class="status"><span id="dot" class="dot"></span><span id="statxt">connecting</span></span>
  <div class="battery">
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="var(--muted)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><rect x="2" y="7" width="18" height="10" rx="2"/><path d="M22 11v2"/></svg>
    <span class="v" id="battv">-- V</span>
    <span class="bar"><span id="battbar" style="width:0%"></span></span>
  </div>
</div>
<main>
  <section class="card">
    <h2>Camera</h2>
    <img id="cam" src="/camera.mjpg" alt="Live camera feed from the robot">
  </section>
  <section class="card">
    <h2>Lidar &middot; top is front</h2>
    <canvas id="radar" width="420" height="420" role="img" aria-label="Live lidar radar"></canvas>
    <div class="sectors">
      <div class="sector"><div class="k">Front</div><div class="val" id="s-front">--</div></div>
      <div class="sector"><div class="k">Back</div><div class="val" id="s-back">--</div></div>
      <div class="sector"><div class="k">Left</div><div class="val" id="s-left">--</div></div>
      <div class="sector"><div class="k">Right</div><div class="val" id="s-right">--</div></div>
    </div>
  </section>
  <section class="card controls">
    <h2>Drive &middot; hold a button</h2>
    <div class="ctl-wrap">
      <div class="pad">
        <button class="pad-btn" data-dir="forward" style="grid-area:up" aria-label="Forward">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 19V5"/><path d="m5 12 7-7 7 7"/></svg>
        </button>
        <button class="pad-btn" data-dir="left" style="grid-area:left" aria-label="Turn left">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M19 12H5"/><path d="m12 19-7-7 7-7"/></svg>
        </button>
        <button class="pad-btn stop" id="stopbtn" style="grid-area:mid" aria-label="Stop">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg>
        </button>
        <button class="pad-btn" data-dir="right" style="grid-area:right" aria-label="Turn right">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M5 12h14"/><path d="m12 5 7 7-7 7"/></svg>
        </button>
        <button class="pad-btn" data-dir="backward" style="grid-area:down" aria-label="Backward">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 5v14"/><path d="m19 12-7 7-7-7"/></svg>
        </button>
      </div>
      <div class="speed">
        <label for="spd">Speed <span class="num" id="spdval">0.30</span></label>
        <input id="spd" type="range" min="0.1" max="1" step="0.05" value="0.3">
        <p class="hint">Hold a direction to drive, release to stop. Keyboard: W A S D or arrow keys. Auto-stops if the connection drops.</p>
        <p class="nodrive" id="nodrive" hidden>Motor driver not detected; controls disabled.</p>
      </div>
    </div>
  </section>
</main>
<footer id="foot">waiting for data</footer>
<script>
const cv = document.getElementById("radar");
const ctx = cv.getContext("2d");
const SIZE = 420, C = SIZE/2, MAXM = 4.0, R = SIZE/2 - 26;
const reduce = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
let sweep = 0;

function ring(label, m){
  const r = R * m / MAXM;
  ctx.beginPath(); ctx.arc(C, C, r, 0, Math.PI*2);
  ctx.strokeStyle = "#e6e8eb"; ctx.lineWidth = 1; ctx.stroke();
  ctx.fillStyle = "#9aa1ab"; ctx.font = '11px "Fira Code", monospace';
  ctx.fillText(label, C + 4, C - r + 13);
}

function draw(data){
  ctx.clearRect(0,0,SIZE,SIZE);
  ctx.fillStyle = "#ffffff"; ctx.fillRect(0,0,SIZE,SIZE);
  for (let a=0; a<360; a+=45){
    const rad = (a-90)*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(C,C);
    ctx.lineTo(C + R*Math.cos(rad), C + R*Math.sin(rad));
    ctx.strokeStyle = "#f0f1f3"; ctx.lineWidth = 1; ctx.stroke();
  }
  ring("1m",1); ring("2m",2); ring("3m",3); ring("4m",4);
  const offset = data.offset || 0;
  let near = null;
  for (const [a,d] of data.points){
    const m = d/1000;
    if (m > MAXM) continue;
    const disp = ((a - offset) % 360) * Math.PI/180;
    const r = R * m / MAXM;
    const x = C + r*Math.sin(disp);
    const y = C - r*Math.cos(disp);
    const t = Math.min(m/MAXM, 1);
    ctx.fillStyle = `hsl(${t*140}, 68%, 45%)`;
    ctx.beginPath(); ctx.arc(x, y, 2, 0, Math.PI*2); ctx.fill();
    if (near === null || m < near.m) near = {m, x, y};
  }
  if (!reduce){
    sweep = (sweep + 5) % 360;
    const srad = (sweep-90)*Math.PI/180;
    const g = ctx.createLinearGradient(C,C, C+R*Math.cos(srad), C+R*Math.sin(srad));
    g.addColorStop(0,"rgba(16,185,129,0.18)"); g.addColorStop(1,"rgba(16,185,129,0)");
    ctx.beginPath(); ctx.moveTo(C,C);
    ctx.lineTo(C+R*Math.cos(srad), C+R*Math.sin(srad));
    ctx.strokeStyle = g; ctx.lineWidth = 3; ctx.stroke();
  }
  ctx.fillStyle = "#10b981";
  ctx.beginPath(); ctx.moveTo(C, C-R-4); ctx.lineTo(C-6, C-R+8); ctx.lineTo(C+6, C-R+8); ctx.closePath(); ctx.fill();
  ctx.beginPath(); ctx.arc(C,C,3.5,0,Math.PI*2); ctx.fillStyle="#14161a"; ctx.fill();
  if (near){
    ctx.beginPath(); ctx.arc(near.x, near.y, 6, 0, Math.PI*2);
    ctx.strokeStyle = "#ef4444"; ctx.lineWidth = 2; ctx.stroke();
  }
}

function fmt(x){ return x === null ? "--" : x.toFixed(2)+" m"; }
function setStat(id, val){
  const el = document.getElementById(id);
  el.textContent = fmt(val);
  el.className = "val" + (val !== null && val < 0.5 ? " near" : "");
}

async function tick(){
  try{
    const r = await fetch("/lidar.json", {cache:"no-store"});
    const data = await r.json();
    applyDriveAvailability(data.has_robot);
    draw(data);
    const s = data.sectors;
    setStat("s-front", s.front); setStat("s-back", s.back);
    setStat("s-left", s.left); setStat("s-right", s.right);
    const v = data.battery || 0;
    document.getElementById("battv").textContent = v.toFixed(2)+" V";
    const pct = Math.max(0, Math.min(100, (v-9.0)/(12.6-9.0)*100));
    document.getElementById("battbar").style.width = pct.toFixed(0)+"%";
    document.getElementById("dot").className = "dot live";
    document.getElementById("statxt").textContent = "live";
    document.getElementById("foot").textContent =
      data.points.length + " points  -  dead zone < " + (data.min_range*100).toFixed(0) +
      " cm  -  angle offset " + (data.offset||0).toFixed(1) + " deg";
  }catch(e){
    document.getElementById("dot").className = "dot";
    document.getElementById("statxt").textContent = "reconnecting";
  }
}
const spd = document.getElementById("spd");
const spdval = document.getElementById("spdval");
spd.addEventListener("input", () => { spdval.textContent = parseFloat(spd.value).toFixed(2); });

let holdTimer = null, activeBtn = null;
async function send(dir){ try{ await fetch("/move?dir="+dir+"&speed="+spd.value, {cache:"no-store"}); }catch(e){} }
async function sendStop(){ try{ await fetch("/stop", {cache:"no-store"}); }catch(e){} }

function startHold(dir, btn){
  if (activeBtn) return;
  activeBtn = btn; if (btn) btn.classList.add("active");
  send(dir);
  holdTimer = setInterval(() => send(dir), 150);
}
function endHold(){
  if (holdTimer){ clearInterval(holdTimer); holdTimer = null; }
  if (activeBtn){ activeBtn.classList.remove("active"); activeBtn = null; sendStop(); }
}

document.querySelectorAll(".pad-btn[data-dir]").forEach(btn => {
  const dir = btn.dataset.dir;
  btn.addEventListener("pointerdown", e => { e.preventDefault(); startHold(dir, btn); });
  btn.addEventListener("pointerup", endHold);
  btn.addEventListener("pointerleave", endHold);
  btn.addEventListener("pointercancel", endHold);
});
document.getElementById("stopbtn").addEventListener("pointerdown", e => { e.preventDefault(); endHold(); sendStop(); });

const keymap = {w:"forward", s:"backward", a:"left", d:"right",
  arrowup:"forward", arrowdown:"backward", arrowleft:"left", arrowright:"right"};
const pressedKeys = new Set();
window.addEventListener("keydown", e => {
  const k = e.key.toLowerCase(); const dir = keymap[k];
  if (!dir || pressedKeys.has(k)) return;
  pressedKeys.add(k); e.preventDefault();
  startHold(dir, document.querySelector('.pad-btn[data-dir="'+dir+'"]'));
});
window.addEventListener("keyup", e => {
  const k = e.key.toLowerCase(); if (!keymap[k]) return;
  pressedKeys.delete(k); endHold();
});
window.addEventListener("blur", endHold);

let driveChecked = false;
function applyDriveAvailability(has){
  if (driveChecked) return; driveChecked = true;
  if (has) return;
  document.getElementById("nodrive").hidden = false;
  document.querySelectorAll(".pad-btn").forEach(b => { b.disabled = true; b.style.opacity = .4; b.style.cursor = "not-allowed"; });
  spd.disabled = true;
}

setInterval(tick, 120);
tick();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        return

    def _json(self, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/lidar.json"):
            with lock:
                points = list(state["points"])
                battery = state["battery"]
            payload = {
                "points": points,
                "sectors": sectors_from(points),
                "battery": battery,
                "offset": LIDAR_ANGLE_OFFSET,
                "min_range": LIDAR_MIN_RANGE,
                "has_robot": robot is not None,
            }
            self._json(payload)
        elif self.path.startswith("/move"):
            params = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            direction = params.get("dir", ["stop"])[0]
            try:
                speed = float(params.get("speed", ["0.3"])[0])
            except ValueError:
                speed = 0.3
            drive(direction, speed)
            self._json({"ok": True})
        elif self.path.startswith("/stop"):
            drive("stop", 0)
            self._json({"ok": True})
        elif self.path.startswith("/camera.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with lock:
                        jpg = state["jpeg"]
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_error(404)


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    global robot
    try:
        robot = Robot()
        print("robot ready")
    except Exception as exc:
        robot = None
        print("robot unavailable: %s" % exc)
    threading.Thread(target=camera_worker, daemon=True).start()
    threading.Thread(target=lidar_worker, daemon=True).start()
    threading.Thread(target=battery_worker, daemon=True).start()
    threading.Thread(target=watchdog_worker, daemon=True).start()
    print("dashboard on http://0.0.0.0:%d" % PORT)
    Server(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
