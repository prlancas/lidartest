#!/usr/bin/env python3
"""
Live Delta-2 LIDAR monitor.

Serves an interactive web UI (and a Server-Sent-Events live stream) on
http://localhost:8080, fed by live LIDAR data. The data source is pluggable:

  Replay a capture file (no hardware needed - good for trying the UI):
      python3 lidar_server.py
      python3 lidar_server.py --file output.dat

  Read a serial port directly (lidar TX -> USB-UART adapter on this PC):
      python3 lidar_server.py --serial /dev/tty.usbserial-XXXX --baud 115200
      (requires:  pip install pyserial)

  Receive raw bytes pushed over TCP (e.g. an ESP32 forwarding the UART stream):
      python3 lidar_server.py --tcp 9000
      ...then have the ESP32 connect to <this-pc-ip>:9000 and write raw lidar bytes.

The web UI is always served on --http (default 8080). The raw-data TCP ingest
(for the ESP32-forwarding case) uses a SEPARATE port (--tcp, default off),
because port 8080 carries HTTP, not the raw lidar byte stream.

See LIDAR_PROTOCOL.md for the packet format and IMPROVEMENTS.md for ideas on
reducing corrupt packets.
"""
import argparse
import http.server
import json
import os
import queue
import socket
import sys
import threading
import time

STEP_DEG = 22.5 / 21.0          # angular step between samples within a packet
MIN_PKT = 17                    # smallest sane packet (header + 0 samples + crc)
MAX_PKT = 256                   # sanity cap when reading the length field


# --------------------------------------------------------------------------- #
#  Streaming, resyncing, checksum-validating packet parser
# --------------------------------------------------------------------------- #
class StreamParser:
    """Incrementally consumes a byte stream and yields validated packets.

    Emits a list of ordered events per feed():
        ('pkt', bytes)   a fully validated measurement packet
        ('gap', n)       n bytes were discarded while re-syncing (lost data)
        ('badck', 1)     a packet-shaped region failed its checksum
    """

    def __init__(self):
        self.buf = bytearray()
        self.valid = 0
        self.cksum_fail = 0
        self.discarded = 0

    def feed(self, data):
        b = self.buf
        b.extend(data)
        events = []
        i, n, skip = 0, len(b), 0
        while True:
            if n - i < 6:
                break
            if not (b[i] == 0xAA and b[i + 1] == 0x00 and b[i + 3] == 0x01
                    and b[i + 4] == 0x61 and b[i + 5] == 0xAD):
                i += 1
                skip += 1
                continue
            frame_len = (b[i + 1] << 8) | b[i + 2]
            total = frame_len + 2
            if total < MIN_PKT or total > MAX_PKT:
                i += 1
                skip += 1
                continue
            if n - i < total:
                break  # wait for the rest of this packet
            pkt = bytes(b[i:i + total])
            ck = (pkt[-2] << 8) | pkt[-1]
            if (sum(pkt[:-2]) & 0xFFFF) != ck:
                self.cksum_fail += 1
                events.append(("badck", 1))
                i += 1
                skip += 1
                continue
            if skip:
                self.discarded += skip
                events.append(("gap", skip))
                skip = 0
            self.valid += 1
            events.append(("pkt", pkt))
            i += total
        if skip:
            self.discarded += skip
            events.append(("gap", skip))
        del b[:i]
        return events


# --------------------------------------------------------------------------- #
#  Assemble packets into frames (revolutions) and track per-frame corruption
# --------------------------------------------------------------------------- #
class FrameBuilder:
    def __init__(self):
        self.last_angle = None
        self._reset()

    def _reset(self):
        self.points = []
        self.packets = 0
        self.errors = 0
        self.gap_bytes = 0
        self.badck = 0
        self.angles = []
        self.rot = 0.0

    def note_error(self, kind, n):
        self.errors += 1
        if kind == "gap":
            self.gap_bytes += n
        elif kind == "badck":
            self.badck += 1

    def add_pkt(self, pkt):
        start = ((pkt[11] << 8) | pkt[12]) / 100.0
        finished = None
        # A start angle that decreased => a new revolution began.
        if self.last_angle is not None and start + 1e-6 < self.last_angle:
            finished = self._finalize()
            self._reset()
        self.last_angle = start
        self.rot = pkt[8] * 0.05
        nsamp = (len(pkt) - 15) // 3
        for k in range(nsamp):
            o = 13 + k * 3
            d = (pkt[o + 1] << 8) | pkt[o + 2]
            if d:
                a = (start + k * STEP_DEG) % 360.0
                self.points.append([round(a, 2), d, pkt[o]])
        self.packets += 1
        self.angles.append(round(start, 1))
        return finished

    def _finalize(self):
        return {
            "points": self.points,
            "packets": self.packets,
            "errors": self.errors,
            "gap_bytes": self.gap_bytes,
            "badck": self.badck,
            "rot": round(self.rot, 2),
            "corrupt": self.errors > 0,
            "span": [self.angles[0], self.angles[-1]] if self.angles else [0, 0],
        }


# --------------------------------------------------------------------------- #
#  Monitor: glue parser + frame builder + running stats + broadcast
# --------------------------------------------------------------------------- #
class Monitor:
    def __init__(self):
        self.parser = StreamParser()
        self.fb = FrameBuilder()
        self.frames = 0
        self.corrupt_frames = 0
        self.bytes_in = 0
        self.lock = threading.Lock()
        self.t0 = time.time()

    def reset(self):
        with self.lock:
            self.parser = StreamParser()
            self.fb = FrameBuilder()
            self.frames = 0
            self.corrupt_frames = 0
            self.bytes_in = 0
            self.t0 = time.time()
        broadcast({"type": "stats", "stats": self.stats()})

    def feed(self, data):
        with self.lock:
            self.bytes_in += len(data)
            for ev in self.parser.feed(data):
                kind = ev[0]
                if kind == "pkt":
                    done = self.fb.add_pkt(ev[1])
                    if done is not None:
                        self.frames += 1
                        if done["corrupt"]:
                            self.corrupt_frames += 1
                        broadcast({"type": "frame", "frame": done,
                                   "stats": self.stats()})
                else:
                    self.fb.note_error(kind, ev[1])

    def stats(self):
        elapsed = max(1e-6, time.time() - self.t0)
        valid = self.parser.valid
        badck = self.parser.cksum_fail
        disc = self.parser.discarded
        attempts = valid + badck
        return {
            "frames": self.frames,
            "corrupt_frames": self.corrupt_frames,
            "corrupt_pct": round(100 * self.corrupt_frames / self.frames, 1) if self.frames else 0.0,
            "valid_packets": valid,
            "cksum_errors": badck,
            "discarded_bytes": disc,
            "pkt_err_pct": round(100 * badck / attempts, 2) if attempts else 0.0,
            "byte_loss_pct": round(100 * disc / self.bytes_in, 2) if self.bytes_in else 0.0,
            "rot": round(self.fb.rot, 2),
            "fps": round(self.frames / elapsed, 1),
            "bytes_in": self.bytes_in,
            "elapsed": round(elapsed, 1),
        }


# --------------------------------------------------------------------------- #
#  SSE broadcast plumbing
# --------------------------------------------------------------------------- #
subscribers = set()
sub_lock = threading.Lock()


def broadcast(obj):
    msg = json.dumps(obj, separators=(",", ":"))
    with sub_lock:
        dead = []
        for q in subscribers:
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(q)
        for q in dead:
            subscribers.discard(q)


MONITOR = Monitor()


# --------------------------------------------------------------------------- #
#  Data sources
# --------------------------------------------------------------------------- #
def source_replay(path, loop, baud):
    try:
        data = open(path, "rb").read()
    except OSError as e:
        print(f"[replay] cannot open {path}: {e}", file=sys.stderr)
        return
    bps = max(1.0, baud / 10.0)
    interval = 0.05
    chunk = max(1, int(bps * interval))
    print(f"[replay] {path}: {len(data)} bytes @ ~{baud} baud, loop={loop}")
    while True:
        for i in range(0, len(data), chunk):
            MONITOR.feed(data[i:i + chunk])
            time.sleep(interval)
        if not loop:
            print("[replay] done")
            return
        time.sleep(0.3)


def source_serial(port, baud):
    try:
        import serial
    except ImportError:
        print("[serial] pyserial not installed. Run:  pip install pyserial",
              file=sys.stderr)
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:  # noqa: BLE001
        print(f"[serial] cannot open {port}: {e}", file=sys.stderr)
        return
    print(f"[serial] reading {port} @ {baud}")
    while True:
        data = ser.read(4096)
        if data:
            MONITOR.feed(data)


def source_tcp(port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"[tcp] listening for raw lidar bytes on 0.0.0.0:{port}")
    while True:
        conn, addr = srv.accept()
        print(f"[tcp] client connected: {addr}")
        try:
            while True:
                data = conn.recv(8192)
                if not data:
                    break
                MONITOR.feed(data)
        except OSError:
            pass
        finally:
            conn.close()
            print(f"[tcp] client disconnected: {addr}")


def heartbeat():
    while True:
        time.sleep(0.5)
        broadcast({"type": "stats", "stats": MONITOR.stats()})


# --------------------------------------------------------------------------- #
#  HTTP + SSE server
# --------------------------------------------------------------------------- #
class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", UI_HTML.encode())
        elif path == "/reset":
            MONITOR.reset()
            self._send(200, "application/json", b'{"ok":true}')
        elif path == "/stream":
            self._stream()
        else:
            self._send(404, "text/plain", b"not found")

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = queue.Queue(maxsize=200)
        with sub_lock:
            subscribers.add(q)
        # prime with current stats so the panel populates immediately
        try:
            q.put_nowait(json.dumps({"type": "stats", "stats": MONITOR.stats()},
                                    separators=(",", ":")))
        except queue.Full:
            pass
        try:
            while True:
                try:
                    msg = q.get(timeout=1.0)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    continue
                self.wfile.write(b"data: " + msg.encode() + b"\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with sub_lock:
                subscribers.discard(q)

    def log_message(self, *args):
        pass


class ThreadingHTTPServer(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    ap = argparse.ArgumentParser(description="Live Delta-2 LIDAR monitor")
    ap.add_argument("--http", type=int, default=8080, help="web UI port (default 8080)")
    ap.add_argument("--serial", metavar="PORT", help="read directly from a serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp", type=int, metavar="PORT",
                    help="listen for raw lidar bytes pushed over TCP")
    ap.add_argument("--file", default="output.dat", help="capture file for replay mode")
    ap.add_argument("--no-loop", action="store_true", help="do not loop the replay file")
    args = ap.parse_args()

    if args.serial:
        src = threading.Thread(target=source_serial, args=(args.serial, args.baud), daemon=True)
        mode = f"serial {args.serial} @ {args.baud}"
    elif args.tcp:
        src = threading.Thread(target=source_tcp, args=(args.tcp,), daemon=True)
        mode = f"tcp listen :{args.tcp}"
    else:
        src = threading.Thread(target=source_replay,
                               args=(args.file, not args.no_loop, args.baud), daemon=True)
        mode = f"replay {args.file}"

    src.start()
    threading.Thread(target=heartbeat, daemon=True).start()

    httpd = ThreadingHTTPServer(("0.0.0.0", args.http), Handler)
    print(f"Source: {mode}")
    print(f"UI:     http://localhost:{args.http}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


# --------------------------------------------------------------------------- #
#  Embedded web UI
# --------------------------------------------------------------------------- #
UI_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LIDAR live monitor</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --line:#30363d; --fg:#c9d1d9;
          --accent:#58a6ff; --muted:#8b949e; --good:#3fb950; --bad:#f85149; }
  * { box-sizing:border-box; }
  html,body { margin:0; height:100%; background:var(--bg); color:var(--fg);
              font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
  .wrap { display:flex; height:100vh; }
  .stage { flex:1; position:relative; min-width:0; }
  canvas { display:block; width:100%; height:100%; }
  .side { width:320px; background:var(--panel); border-left:1px solid var(--line);
          padding:16px; overflow-y:auto; }
  h1 { font-size:15px; margin:0 0 2px; color:var(--accent); }
  .sub { font-size:11px; color:var(--muted); margin-bottom:14px; }
  .row { margin:12px 0; }
  label { display:block; font-size:11px; color:var(--muted); margin-bottom:6px;
          text-transform:uppercase; letter-spacing:.05em; }
  .btns { display:flex; gap:8px; flex-wrap:wrap; }
  button { background:#21262d; color:var(--fg); border:1px solid var(--line);
           border-radius:6px; padding:7px 12px; cursor:pointer; font:inherit;
           font-size:12px; }
  button:hover { border-color:var(--accent); }
  button.on { background:var(--accent); color:#0d1117; border-color:var(--accent); }
  input[type=range] { width:100%; accent-color:var(--accent); }
  .seg { display:flex; border:1px solid var(--line); border-radius:6px; overflow:hidden; }
  .seg button { flex:1; border:0; border-radius:0; }
  .grid { display:grid; grid-template-columns:1fr auto; gap:2px 10px; font-size:12px; }
  .grid div:nth-child(odd){ color:var(--muted); }
  .grid div:nth-child(even){ color:var(--fg); text-align:right; }
  .big { font-size:28px; font-weight:bold; }
  .card { background:#0d1117; border:1px solid var(--line); border-radius:8px;
          padding:10px 12px; margin-bottom:10px; }
  .card .k { font-size:10px; color:var(--muted); text-transform:uppercase; letter-spacing:.05em; }
  .pct { color:var(--good); }
  .pct.warn { color:#d29922; }
  .pct.bad { color:var(--bad); }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%;
         background:var(--bad); margin-right:6px; vertical-align:middle; }
  .dot.on { background:var(--good); }
  .hud { position:absolute; top:12px; left:14px; font-size:12px; color:var(--muted);
         background:rgba(13,17,23,.65); padding:6px 10px; border-radius:6px; pointer-events:none; }
  .corruptflash { position:absolute; inset:0; box-shadow:inset 0 0 0 3px var(--bad);
                  opacity:0; transition:opacity .15s; pointer-events:none; }
  .legend { position:absolute; bottom:12px; left:14px; font-size:11px; color:var(--muted);
            background:rgba(13,17,23,.65); padding:8px 10px; border-radius:6px; }
  .bar { height:8px; width:140px; border-radius:4px; margin-top:4px;
         background:linear-gradient(90deg,#3b1f5e,#2c6e9c,#1f9e7a,#9ec837,#ffd23f); }
</style>
</head>
<body>
<div class="wrap">
  <div class="stage">
    <canvas id="cv"></canvas>
    <div class="hud" id="hud">waiting for data...</div>
    <div class="corruptflash" id="flash"></div>
    <div class="legend">intensity<div class="bar"></div>
      <div style="display:flex;justify-content:space-between"><span>low</span><span>high</span></div></div>
  </div>
  <div class="side">
    <h1>LIDAR live monitor</h1>
    <div class="sub"><span class="dot" id="connDot"></span><span id="conn">connecting...</span></div>

    <div class="card">
      <div class="k">Corrupt frames</div>
      <div class="big"><span class="pct" id="corruptPct">0%</span></div>
      <div class="grid">
        <div>frames</div><div id="frames">0</div>
        <div>corrupt</div><div id="corruptN">0</div>
      </div>
    </div>

    <div class="card">
      <div class="grid">
        <div>valid packets</div><div id="validP">0</div>
        <div>checksum errors</div><div id="ckErr">0</div>
        <div>discarded bytes</div><div id="disc">0</div>
        <div>packet err %</div><div id="pktErr">0%</div>
        <div>byte loss %</div><div id="byteLoss">0%</div>
        <div>rotation</div><div id="rot">- rev/s</div>
        <div>frames/s</div><div id="fps">0</div>
        <div>elapsed</div><div id="elapsed">0s</div>
      </div>
    </div>

    <div class="row">
      <label>View mode</label>
      <div class="seg">
        <button id="mLatest" class="on">Latest frame</button>
        <button id="mCumul">Cumulative</button>
      </div>
    </div>

    <div class="row btns">
      <button id="clear">Clear view</button>
      <button id="reset">Reset stats</button>
      <button id="pause">Pause</button>
    </div>

    <div class="row">
      <label class="lbl">Highlight corrupt-frame points</label>
      <label style="font-size:12px;color:var(--fg);cursor:pointer">
        <input type="checkbox" id="markCorrupt" checked> mark in red</label>
    </div>

    <div class="row">
      <label>Max range (m): <span id="rangeLbl">10</span></label>
      <input type="range" id="range" min="1" max="12" step="0.5" value="10">
    </div>
    <div class="row">
      <label>Cumulative buffer (max points): <span id="capLbl">200k</span></label>
      <input type="range" id="cap" min="20000" max="500000" step="20000" value="200000">
    </div>
  </div>
</div>
<script>
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
let mode='latest', paused=false, markCorrupt=true, maxRange=10, cap=200000;
let latest=null, latestCorrupt=false;
let cumul=[];                 // [a,d,q,corrupt]
let needsDraw=true;

function color(q){
  const t=Math.max(0,Math.min(1,q/200));
  const s=[[59,31,94],[44,110,156],[31,158,122],[158,200,55],[255,210,63]];
  const f=t*(s.length-1),i=Math.floor(f),r=f-i,a=s[i],b=s[Math.min(i+1,s.length-1)];
  return `rgb(${a[0]+(b[0]-a[0])*r|0},${a[1]+(b[1]-a[1])*r|0},${a[2]+(b[2]-a[2])*r|0})`;
}
function resize(){
  const r=cv.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
  cv.width=r.width*dpr; cv.height=r.height*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  needsDraw=true;
}
window.addEventListener('resize',resize);

function drawGrid(cx,cy,scale){
  ctx.strokeStyle='#21262d'; ctx.fillStyle='#586069';
  ctx.font='10px monospace'; ctx.textAlign='center';
  for(let m=1;m<=maxRange;m++){
    ctx.beginPath(); ctx.arc(cx,cy,m*1000*scale,0,Math.PI*2); ctx.stroke();
    ctx.fillText(m+'m',cx,cy-m*1000*scale-2);
  }
  ctx.strokeStyle='#1b2129';
  for(let a=0;a<360;a+=30){
    const rad=a*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx+Math.cos(rad)*maxRange*1000*scale, cy-Math.sin(rad)*maxRange*1000*scale);
    ctx.stroke();
  }
  ctx.fillStyle='#586069';
  ctx.fillText('0\u00b0',cx+maxRange*1000*scale-8,cy+12);
  ctx.fillText('90\u00b0',cx+12,cy-maxRange*1000*scale+10);
}
function plot(pts,cx,cy,scale,size,corruptFlag){
  let shown=0;
  for(const p of pts){
    const d=p[1]; if(!d||d>maxRange*1000) continue;
    const rad=p[0]*Math.PI/180;
    const x=cx+Math.cos(rad)*d*scale, y=cy-Math.sin(rad)*d*scale;
    const isCorrupt = (corruptFlag!==undefined)?corruptFlag:(p[3]===1);
    ctx.fillStyle = (markCorrupt && isCorrupt) ? '#f85149' : color(p[2]);
    ctx.beginPath(); ctx.arc(x,y,size,0,Math.PI*2); ctx.fill();
    shown++;
  }
  return shown;
}
function draw(){
  const W=cv.clientWidth,H=cv.clientHeight,cx=W/2,cy=H/2;
  const scale=(Math.min(W,H)/2-30)/(maxRange*1000);
  ctx.clearRect(0,0,W,H);
  drawGrid(cx,cy,scale);
  let shown=0;
  if(mode==='latest'){
    if(latest) shown=plot(latest,cx,cy,scale,2.2,latestCorrupt);
  } else {
    shown=plot(cumul,cx,cy,scale,1.3);
  }
  ctx.fillStyle='#f85149'; ctx.beginPath(); ctx.arc(cx,cy,4,0,Math.PI*2); ctx.fill();
  document.getElementById('hud').textContent =
    (mode==='latest'?'LATEST FRAME':'CUMULATIVE')+'  \u2014  '+shown+' points'+
    (paused?'  (paused)':'');
}
function loop(){ if(needsDraw && !paused){ draw(); needsDraw=false; } requestAnimationFrame(loop); }

function flash(){ const f=document.getElementById('flash'); f.style.opacity=1;
  setTimeout(()=>f.style.opacity=0,120); }

function applyStats(s){
  const set=(id,v)=>document.getElementById(id).textContent=v;
  set('frames',s.frames); set('corruptN',s.corrupt_frames);
  const pe=document.getElementById('corruptPct'); pe.textContent=s.corrupt_pct+'%';
  pe.className='pct'+(s.corrupt_pct>20?' bad':s.corrupt_pct>5?' warn':'');
  set('validP',s.valid_packets); set('ckErr',s.cksum_errors);
  set('disc',s.discarded_bytes); set('pktErr',s.pkt_err_pct+'%');
  set('byteLoss',s.byte_loss_pct+'%');
  set('rot',(s.rot?s.rot.toFixed(2):'-')+' rev/s'); set('fps',s.fps);
  set('elapsed',s.elapsed+'s');
}

// SSE
function connect(){
  const es=new EventSource('/stream');
  const dot=document.getElementById('connDot'), conn=document.getElementById('conn');
  es.onopen=()=>{dot.classList.add('on'); conn.textContent='connected';};
  es.onerror=()=>{dot.classList.remove('on'); conn.textContent='reconnecting...';};
  es.onmessage=(e)=>{
    const m=JSON.parse(e.data);
    if(m.stats) applyStats(m.stats);
    if(m.type==='frame'){
      const fr=m.frame;
      latest=fr.points; latestCorrupt=fr.corrupt;
      if(fr.corrupt) flash();
      if(mode==='cumul' && !paused){
        const c=fr.corrupt?1:0;
        for(const p of fr.points) cumul.push([p[0],p[1],p[2],c]);
        if(cumul.length>cap) cumul.splice(0,cumul.length-cap);
      }
      needsDraw=true;
    }
  };
}

// controls
function selMode(m){ mode=m;
  document.getElementById('mLatest').classList.toggle('on',m==='latest');
  document.getElementById('mCumul').classList.toggle('on',m==='cumul');
  needsDraw=true;
}
document.getElementById('mLatest').onclick=()=>selMode('latest');
document.getElementById('mCumul').onclick=()=>selMode('cumul');
document.getElementById('clear').onclick=()=>{cumul=[]; latest=null; needsDraw=true;};
document.getElementById('reset').onclick=()=>{fetch('/reset'); cumul=[]; latest=null; needsDraw=true;};
const pauseBtn=document.getElementById('pause');
pauseBtn.onclick=()=>{paused=!paused; pauseBtn.classList.toggle('on',paused);
  pauseBtn.textContent=paused?'Resume':'Pause'; needsDraw=true;};
document.getElementById('markCorrupt').onchange=e=>{markCorrupt=e.target.checked; needsDraw=true;};
const rg=document.getElementById('range');
rg.oninput=e=>{maxRange=+e.target.value; document.getElementById('rangeLbl').textContent=maxRange; needsDraw=true;};
const capSl=document.getElementById('cap');
capSl.oninput=e=>{cap=+e.target.value; document.getElementById('capLbl').textContent=Math.round(cap/1000)+'k';
  if(cumul.length>cap) cumul.splice(0,cumul.length-cap); needsDraw=true;};

resize(); loop(); connect();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
