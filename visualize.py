#!/usr/bin/env python3
"""
Parse a Delta-2 LIDAR raw UART dump (output.dat) and generate a self-contained
interactive HTML visualizer (lidar_viz.html).

Usage:
    python3 visualize.py [input.dat] [output.html]

Then open lidar_viz.html in any browser. No server needed - the data is
embedded directly in the file.

See LIDAR_PROTOCOL.md for the decoded packet format.
"""
import json
import math
import sys

STEP_DEG = 22.5 / 21.0  # angular step between samples within a packet


def parse(data):
    """Walk the byte stream and return validated measurement packets."""
    pkts = []
    i, n = 0, len(data)
    while i < n - 5:
        # Resync on the full header signature, not just AA 00.
        if not (data[i] == 0xAA and data[i + 1] == 0x00
                and data[i + 3] == 0x01 and data[i + 4] == 0x61
                and data[i + 5] == 0xAD):
            i += 1
            continue
        frame_len = (data[i + 1] << 8) | data[i + 2]
        total = frame_len + 2
        if i + total > n or total < 17:
            i += 1
            continue
        pkt = data[i:i + total]
        body = pkt[:-2]
        ck = (pkt[-2] << 8) | pkt[-1]
        if (sum(body) & 0xFFFF) != ck:
            i += 1
            continue
        pkts.append(pkt)
        i += total
    return pkts


def decode_packet(pkt):
    rot_raw = pkt[8]
    start_angle = ((pkt[11] << 8) | pkt[12]) / 100.0
    nsamp = (len(pkt) - 15) // 3
    pts = []
    for k in range(nsamp):
        o = 13 + k * 3
        quality = pkt[o]
        dist_mm = (pkt[o + 1] << 8) | pkt[o + 2]
        ang = (start_angle + k * STEP_DEG) % 360.0
        pts.append({"a": round(ang, 3), "d": dist_mm, "q": quality})
    return {
        "start": round(start_angle, 2),
        "rot": round(rot_raw * 0.05, 3),  # rev/s
        "points": pts,
    }


def segment_frames(packets):
    """Group packets into revolutions: a new frame starts when the start angle
    wraps (decreases)."""
    frames, cur, last = [], [], None
    for p in packets:
        ang = (p[11] << 8) | p[12]
        if last is not None and ang < last:
            frames.append(cur)
            cur = []
        cur.append(p)
        last = ang
    if cur:
        frames.append(cur)
    return frames


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "output.dat"
    out = sys.argv[2] if len(sys.argv) > 2 else "lidar_viz.html"

    data = open(src, "rb").read()
    packets = parse(data)
    frames_raw = segment_frames(packets)

    frames = []
    for fr in frames_raw:
        decoded = [decode_packet(p) for p in fr]
        pts = [pt for d in decoded for pt in d["points"]]
        rot = sum(d["rot"] for d in decoded) / len(decoded) if decoded else 0
        starts = [d["start"] for d in decoded]
        frames.append({
            "packets": len(fr),
            "rot": round(rot, 2),
            "angles": [round(s, 1) for s in starts],
            "points": pts,
        })

    n_pts = sum(len(f["points"]) for f in frames)
    meta = {
        "source": src,
        "total_bytes": len(data),
        "packets": len(packets),
        "frames": len(frames),
        "points": n_pts,
    }
    print(f"Parsed {len(packets)} packets, {len(frames)} frames, {n_pts} points "
          f"from {len(data)} bytes of {src}")

    payload = json.dumps({"meta": meta, "frames": frames}, separators=(",", ":"))
    html = HTML_TEMPLATE.replace("/*DATA*/", payload)
    with open(out, "w") as f:
        f.write(html)
    print(f"Wrote {out} ({len(html)//1024} KB). Open it in a browser.")


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LIDAR replay - output.dat</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --line:#30363d; --fg:#c9d1d9;
          --accent:#58a6ff; --muted:#8b949e; }
  * { box-sizing: border-box; }
  html,body { margin:0; height:100%; background:var(--bg); color:var(--fg);
              font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
  .wrap { display:flex; height:100vh; }
  .stage { flex:1; position:relative; min-width:0; }
  canvas { display:block; width:100%; height:100%; }
  .side { width:300px; background:var(--panel); border-left:1px solid var(--line);
          padding:16px; overflow-y:auto; }
  h1 { font-size:15px; margin:0 0 4px; color:var(--accent); }
  .sub { font-size:11px; color:var(--muted); margin-bottom:16px; }
  .row { margin:14px 0; }
  label { display:block; font-size:11px; color:var(--muted); margin-bottom:6px;
          text-transform:uppercase; letter-spacing:.05em; }
  input[type=range] { width:100%; accent-color:var(--accent); }
  .big { font-size:13px; color:var(--fg); }
  button { background:#21262d; color:var(--fg); border:1px solid var(--line);
           border-radius:6px; padding:6px 12px; cursor:pointer; font:inherit;
           font-size:12px; }
  button:hover { border-color:var(--accent); }
  button.on { background:var(--accent); color:#0d1117; border-color:var(--accent); }
  .btns { display:flex; gap:8px; flex-wrap:wrap; }
  .stat { display:flex; justify-content:space-between; font-size:12px;
          padding:3px 0; border-bottom:1px dashed var(--line); }
  .stat span:last-child { color:var(--accent); }
  .toggle { display:flex; align-items:center; gap:8px; font-size:12px;
            color:var(--fg); cursor:pointer; }
  .hud { position:absolute; top:12px; left:14px; font-size:12px; color:var(--muted);
         background:rgba(13,17,23,.65); padding:6px 10px; border-radius:6px;
         pointer-events:none; }
  .legend { position:absolute; bottom:12px; left:14px; font-size:11px;
            color:var(--muted); background:rgba(13,17,23,.65); padding:8px 10px;
            border-radius:6px; }
  .bar { height:8px; width:140px; border-radius:4px; margin-top:4px;
         background:linear-gradient(90deg,#3b1f5e,#2c6e9c,#1f9e7a,#9ec837,#ffd23f); }
</style>
</head>
<body>
<div class="wrap">
  <div class="stage">
    <canvas id="cv"></canvas>
    <div class="hud" id="hud"></div>
    <div class="legend">
      intensity<div class="bar"></div>
      <div style="display:flex;justify-content:space-between"><span>low</span><span>high</span></div>
    </div>
  </div>
  <div class="side">
    <h1>LIDAR replay</h1>
    <div class="sub" id="meta"></div>

    <div class="row">
      <label>Frame (revolution)</label>
      <input type="range" id="frame" min="0" value="0">
      <div class="big" id="frameLbl"></div>
    </div>

    <div class="row btns">
      <button id="play">&#9654; Play</button>
      <button id="prev">&#9664;</button>
      <button id="next">&#9654;</button>
    </div>

    <div class="row">
      <label>Speed (frames/s)</label>
      <input type="range" id="speed" min="1" max="30" value="8">
      <div class="big" id="speedLbl"></div>
    </div>

    <div class="row">
      <label class="toggle"><input type="checkbox" id="allFrames"> Show ALL points (entire file)</label>
    </div>
    <div class="row">
      <label class="toggle"><input type="checkbox" id="showRays" checked> Draw rays from sensor</label>
    </div>

    <div class="row">
      <label>Max range (m): <span id="rangeLbl"></span></label>
      <input type="range" id="range" min="1" max="12" step="0.5" value="10">
    </div>

    <div class="row">
      <div class="stat"><span>frame points</span><span id="sPts">-</span></div>
      <div class="stat"><span>packets in frame</span><span id="sPkt">-</span></div>
      <div class="stat"><span>rotation</span><span id="sRot">-</span></div>
      <div class="stat"><span>angle span</span><span id="sSpan">-</span></div>
    </div>
  </div>
</div>
<script>
const DATA = /*DATA*/;
const frames = DATA.frames, meta = DATA.meta;

document.getElementById('meta').textContent =
  `${meta.source} \u2014 ${meta.bytes||meta.total_bytes} bytes, ${meta.packets} packets, ${meta.frames} frames, ${meta.points} points`;

const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const frameSlider = document.getElementById('frame');
frameSlider.max = frames.length - 1;

let idx = 0, playing = false, showAll = false, showRays = true, maxRange = 10;
let timer = null;

// viridis-ish color ramp for intensity 0..255
function color(q){
  const t = Math.max(0, Math.min(1, q/200));
  const stops = [[59,31,94],[44,110,156],[31,158,122],[158,200,55],[255,210,63]];
  const f = t*(stops.length-1), i = Math.floor(f), r = f-i;
  const a = stops[i], b = stops[Math.min(i+1,stops.length-1)];
  return `rgb(${a[0]+(b[0]-a[0])*r|0},${a[1]+(b[1]-a[1])*r|0},${a[2]+(b[2]-a[2])*r|0})`;
}

function resize(){
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio||1;
  cv.width = r.width*dpr; cv.height = r.height*dpr;
  ctx.setTransform(dpr,0,0,dpr,0,0);
  draw();
}
window.addEventListener('resize', resize);

function draw(){
  const W = cv.clientWidth, H = cv.clientHeight;
  ctx.clearRect(0,0,W,H);
  const cx = W/2, cy = H/2;
  const scale = (Math.min(W,H)/2 - 30) / (maxRange*1000); // px per mm

  // range rings + angle spokes
  ctx.strokeStyle = '#21262d'; ctx.fillStyle = '#586069';
  ctx.font = '10px monospace'; ctx.textAlign = 'center';
  for(let m=1; m<=maxRange; m++){
    ctx.beginPath(); ctx.arc(cx,cy, m*1000*scale, 0, Math.PI*2); ctx.stroke();
    ctx.fillText(m+'m', cx, cy - m*1000*scale - 2);
  }
  ctx.strokeStyle = '#1b2129';
  for(let a=0; a<360; a+=30){
    const rad = a*Math.PI/180;
    ctx.beginPath(); ctx.moveTo(cx,cy);
    ctx.lineTo(cx + Math.cos(rad)*maxRange*1000*scale, cy - Math.sin(rad)*maxRange*1000*scale);
    ctx.stroke();
  }
  // 0 deg label
  ctx.fillStyle = '#586069';
  ctx.fillText('0\u00b0', cx + maxRange*1000*scale - 8, cy + 12);
  ctx.fillText('90\u00b0', cx + 12, cy - maxRange*1000*scale + 10);

  // gather points
  let pts, pktCount = 0, rot = 0, spanTxt = '-';
  if(showAll){
    pts = [];
    for(const f of frames){ for(const p of f.points) pts.push(p); }
    pktCount = meta.packets;
    spanTxt = 'all';
  } else {
    const f = frames[idx];
    pts = f.points; pktCount = f.packets; rot = f.rot;
    if(f.angles.length) spanTxt = f.angles[0].toFixed(0)+'\u00b0 \u2192 '+f.angles[f.angles.length-1].toFixed(0)+'\u00b0';
  }

  let shown = 0;
  for(const p of pts){
    if(!p.d) continue;            // no return
    if(p.d > maxRange*1000) continue;
    const rad = p.a*Math.PI/180;
    const x = cx + Math.cos(rad)*p.d*scale;
    const y = cy - Math.sin(rad)*p.d*scale;
    if(showRays && !showAll){
      ctx.strokeStyle = 'rgba(88,166,255,0.06)';
      ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(x,y); ctx.stroke();
    }
    ctx.fillStyle = color(p.q);
    ctx.beginPath(); ctx.arc(x,y, showAll?1.3:2.2, 0, Math.PI*2); ctx.fill();
    shown++;
  }

  // sensor
  ctx.fillStyle = '#f85149';
  ctx.beginPath(); ctx.arc(cx,cy,4,0,Math.PI*2); ctx.fill();

  // stats
  document.getElementById('sPts').textContent = shown;
  document.getElementById('sPkt').textContent = pktCount;
  document.getElementById('sRot').textContent = showAll ? '-' : rot.toFixed(2)+' rev/s';
  document.getElementById('sSpan').textContent = spanTxt;
  document.getElementById('hud').textContent = showAll
    ? `ALL FRAMES  \u2014  ${shown} points`
    : `frame ${idx+1}/${frames.length}  \u2014  ${shown} points`;
  document.getElementById('frameLbl').textContent = showAll ? 'all frames' : `${idx+1} / ${frames.length}`;
}

function setIdx(v){
  idx = Math.max(0, Math.min(frames.length-1, v));
  frameSlider.value = idx;
  draw();
}

frameSlider.oninput = e => { setIdx(+e.target.value); };
document.getElementById('prev').onclick = () => setIdx(idx-1);
document.getElementById('next').onclick = () => setIdx(idx+1);

const playBtn = document.getElementById('play');
const speed = document.getElementById('speed');
const speedLbl = document.getElementById('speedLbl');
function updateSpeedLbl(){ speedLbl.textContent = speed.value + ' fps'; }
updateSpeedLbl();
function restartTimer(){
  if(timer) clearInterval(timer);
  if(playing) timer = setInterval(()=>{ setIdx(idx+1 >= frames.length ? 0 : idx+1); }, 1000/(+speed.value));
}
playBtn.onclick = () => {
  playing = !playing;
  playBtn.classList.toggle('on', playing);
  playBtn.innerHTML = playing ? '&#10073;&#10073; Pause' : '&#9654; Play';
  restartTimer();
};
speed.oninput = () => { updateSpeedLbl(); restartTimer(); };

const allChk = document.getElementById('allFrames');
allChk.onchange = e => { showAll = e.target.checked; draw(); };
document.getElementById('showRays').onchange = e => { showRays = e.target.checked; draw(); };

const rangeSl = document.getElementById('range');
const rangeLbl = document.getElementById('rangeLbl');
rangeLbl.textContent = maxRange;
rangeSl.oninput = e => { maxRange = +e.target.value; rangeLbl.textContent = maxRange; draw(); };

document.addEventListener('keydown', e => {
  if(e.key==='ArrowLeft') setIdx(idx-1);
  else if(e.key==='ArrowRight') setIdx(idx+1);
  else if(e.key===' '){ e.preventDefault(); playBtn.click(); }
});

resize();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
