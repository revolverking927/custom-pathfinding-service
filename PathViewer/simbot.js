// Demo bot for PathViewer: mirrors Follower.update() + PID + the PathingConfig defaults against a
// simple mecanum model and serves the same /state JSON as FieldServer on port 8765.
// Run: node PathViewer/simbot.js   (HEADLESS=1 prints a pass/fail summary instead, SPEED=2 runs faster)
const http = require('http');
const C = { transP: .05, transI: 0, transD: .005, headP: 1.2, headI: 0, headD: .08, kStatic: .08,
  maxPower: .6, posTolIn: .75, headTolDeg: 2, settleSec: .15 };
const wrap = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a <= -Math.PI) a += 2 * Math.PI; return a; };
class PID {
  constructor() { this.i = 0; this.last = 0; this.has = false; }
  update(e, dt, kP, kI, kD) {
    this.i += e * dt;
    if (kI > 0) { const m = .3 / kI; this.i = Math.max(-m, Math.min(m, this.i)); }
    const d = this.has ? (e - this.last) / dt : 0; this.last = e; this.has = true;
    return kP * e + kI * this.i + kD * d;
  }
  reset() { this.i = 0; this.has = false; }
}
const T = 24; // tile in inches
const path = [[-1, -1, 0], [1, -1, 0], [1, 1, Math.PI / 2], [-1, 1, Math.PI], [-1, -1, 0]]
  .map(([x, y, h]) => ({ x: x * T, y: y * T, h }));
let pose = { x: -2 * T, y: -2 * T, h: 0 }, vel = { x: 0, y: 0, h: 0 };
let out = { f: 0, s: 0, t: 0 }, applied = { f: 0, s: 0, t: 0 };
let idx = 0, busy = true, settle = 0, posErr = 0, headErr = 0, t = 0, log = [];
const tp = new PID(), hp = new PID();
const DT = 0.02, MAXV = 50, MAXW = 4.5; // in/s and rad/s at full power

function step() {
  const tgt = path[idx];
  const dx = tgt.x - pose.x, dy = tgt.y - pose.y;
  posErr = Math.hypot(dx, dy); headErr = wrap(tgt.h - pose.h);
  if (busy) {
    const inPos = posErr <= C.posTolIn, inHead = Math.abs(headErr) <= C.headTolDeg * Math.PI / 180;
    const last = idx === path.length - 1;
    let done = false;
    if (inPos && inHead) {
      if (!last) { idx++; settle = 0; tp.reset(); hp.reset(); return; }
      settle += DT; if (settle >= C.settleSec) { busy = false; out = { f: 0, s: 0, t: 0 }; done = true; }
    } else settle = 0;
    if (!done) {
      let power = 0, fd = 0, sd = 0;
      if (!inPos) {
        power = tp.update(posErr, DT, C.transP, C.transI, C.transD);
        power = Math.min(C.maxPower, Math.max(power, 0) + C.kStatic);
        const c = Math.cos(pose.h), s = Math.sin(pose.h);
        fd = (dx * c + dy * s) / posErr; sd = (-dx * s + dy * c) / posErr;
      } else tp.reset();
      let turn = 0;
      if (!inHead) {
        turn = hp.update(headErr, DT, C.headP, C.headI, C.headD) + Math.sign(headErr) * C.kStatic;
        turn = Math.max(-C.maxPower, Math.min(C.maxPower, turn));
      } else hp.reset();
      out = { f: fd * power, s: sd * power, t: -turn };
    }
  }
  // Plant: first-order motor lag, then integrate in the field frame.
  const a = 0.25;
  for (const k of ['f', 's', 't']) applied[k] += (out[k] - applied[k]) * a;
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  vel = { x: (applied.f * c - applied.s * s) * MAXV, y: (applied.f * s + applied.s * c) * MAXV, h: -applied.t * MAXW };
  pose = { x: pose.x + vel.x * DT, y: pose.y + vel.y * DT, h: wrap(pose.h + vel.h * DT) };
  t += DT;
}

const pj = p => `{"x":${p.x.toFixed(3)},"y":${p.y.toFixed(3)},"h":${(p.h).toFixed(4)}}`;
function state() {
  return `{"time":${Date.now()},"mode":"SIM square","unit":"TILES","fieldSize":144,"pose":${pj(pose)},` +
    `"velocity":${pj(vel)},"target":${busy ? pj(path[idx]) : 'null'},"waypoints":[${path.map(pj).join(',')}],` +
    `"waypointIndex":${idx},"busy":${busy},"posError":${posErr.toFixed(3)},"headError":${headErr.toFixed(4)},` +
    `"power":{"f":${out.f.toFixed(3)},"s":${out.s.toFixed(3)},"t":${out.t.toFixed(3)}},"gains":${JSON.stringify(C)}}`;
}

const speed = Number(process.env.SPEED || 1); // 1 = real time
if (process.env.HEADLESS) {
  let n = 0; while (busy && n++ < 100000) { step(); if (n % 100 === 0) log.push(pose); }
  const end = path[path.length - 1];
  console.log(JSON.stringify({ finished: !busy, seconds: +t.toFixed(2), finalPose: pose,
    errIn: +Math.hypot(end.x - pose.x, end.y - pose.y).toFixed(3),
    errDeg: +(Math.abs(wrap(end.h - pose.h)) * 180 / Math.PI).toFixed(2) }));
  process.exit(0);
}
setInterval(() => { for (let i = 0; i < speed; i++) step(); }, DT * 1000);
http.createServer((q, r) => {
  r.writeHead(200, { 'Content-Type': 'application/json', 'Access-Control-Allow-Origin': '*', 'Cache-Control': 'no-store' });
  r.end(q.url.startsWith('/state') ? state() : '{}');
}).listen(8765, () => console.log('sim bot on :8765'));
