// main.js -- application: input, fixed-step physics, HUD, minimap, track editor.
// Physics runs at a fixed 250 Hz (RK4) on an accumulator, decoupled from the
// render rate so behaviour is independent of display Hz.

const PHYS_DT = 0.004;           // 250 Hz physics step
const MAX_STEER = 0.52;          // ~30 deg max front steer
const STEER_RATE = 2.8;          // rad/s steering slew
const MU_ROAD = 1.3, MU_GRASS = 0.45;

// ---- simplified manual gearbox (driver-input model; the tyre/chassis physics
// stay the verified DoubleTrack). Throttle is scaled by the gear's tractive
// effort at the current engine RPM: low gears pull hard but rev out, high gears
// are weaker but reach higher speed. C = up, V = down (real-car logic).
const GEAR_RATIOS = [3.6, 2.25, 1.6, 1.2, 0.95, 0.78];
const FINAL_DRIVE = 3.7, WHEEL_R = 0.32, RPM_IDLE = 950, RPM_REDLINE = 7200;
let gear = 1, engineRpm = RPM_IDLE, reverse = false;
function torqueFactor(rpm) {                       // bell curve, peak ~4200 rpm
  const t = 1 - ((rpm - 4200) / 3300) ** 2;
  return Math.max(0.18, Math.min(1, t));
}
function drivetrain(vx) {                           // -> {thrScale, rpm, overRev}
  const ratio = GEAR_RATIOS[gear - 1] * FINAL_DRIVE;
  let rpm = Math.abs(vx) / WHEEL_R * ratio * 60 / (2 * Math.PI);
  rpm = Math.max(RPM_IDLE, Math.min(rpm, RPM_REDLINE * 1.04));
  const overRev = rpm >= RPM_REDLINE;
  // tractive effort ~ torque * ratio, normalised so 1st gear peaks near 1.0
  const scale = torqueFactor(rpm) * ratio / (GEAR_RATIOS[0] * FINAL_DRIVE);
  return { thrScale: Math.min(1, scale * 1.7), rpm, overRev };
}

// the verified 4-wheel car; tllt_f=0.55 -> mild, road-car-like understeer
const car = new DoubleTrack({ tllt_f: 0.55, mu_load: 0.10 });
let track = defaultTrack();

// the bridge: 3 simply-supported spans (a viaduct), each its own modal beam.
const SPAN_L = 40.0, N_SPANS = 3;
const bridges = Array.from({ length: N_SPANS }, () =>
  new BridgeModal({ L: SPAN_L, EI: 8.4e10, m_bar: 12000.0, zeta: 0.02, n_modes: 4 }));
const bridgeState = { onBridge: false, activeIdx: 0, loadPos: null, P: 0,
                      daf: 1, peakDyn: 0, denom: 1 };

// overlay toggle: force/velocity vectors on the car + the driven line
let showVectors = false;
const trace = [];

// guided auto-drive "tour": on by default so the demo shows itself; any drive
// key hands control to the user. Rotating captions explain each physics.
let autoDrive = true;
const TOUR = [
  "<b>1 · Vehicle dynamics.</b> Double-track 4-wheel + Pacejka tyres, live. Watch the g-g circle &amp; per-wheel load (left).",
  "<b>2 · Aero.</b> A real 2D fluid is solved around the car every frame (top-right) — the wake &amp; downforce. <i>2D, model-Re, qualitative.</i>",
  "<b>3 · Bridge (VBI).</b> A 3-span viaduct sags under the wheel load (bottom), coloured by bending moment, with the dynamic amplification factor.",
  "<b>Take over anytime:</b> arrows/WASD to drive · C/V gears · Space handbrake · E reverse · T = force vectors. Everything is computed live.",
];
let tourIdx = 0;

// car state q = [X, Y, psi, vx, vy, r]; start on the grid.
let q, steer, lastT, acc;
function resetCar() {
  const p = track.startPose();
  q = [p.x, p.y, p.psi, 8.0, 0, 0];   // roll onto the grid at 8 m/s
  steer = 0; acc = 0; lastT = null;
  if (typeof trace !== "undefined") trace.length = 0;
}
resetCar();

// named view of the state for the renderer
const view = { X: 0, Y: 0, psi: 0, vx: 0, vy: 0, r: 0,
               a: car.a, b: car.b, track: car.track };
function syncView() {
  [view.X, view.Y, view.psi, view.vx, view.vy, view.r] = q;
}

// ---- input ----------------------------------------------------------------
const keys = {};
const DRIVE_KEYS = ["arrowup", "arrowdown", "arrowleft", "arrowright",
  "w", "a", "s", "d", " "];
addEventListener("keydown", (e) => {
  const k = e.key.toLowerCase();
  keys[k] = true;
  if (DRIVE_KEYS.includes(k) && autoDrive) setAutoDrive(false);  // user takes over
  if (k === "r") resetCar();
  if (k === "t" && !e.repeat) toggleVectors();
  // reverse: press E while nearly stopped (e.g. after braking with Space) to
  // toggle reverse gear; shifting up with C returns to drive.
  if (k === "e" && !e.repeat && Math.abs(q[3]) < 3) reverse = !reverse;
  if (k === "c" && !e.repeat) { reverse = false; gear = Math.min(GEAR_RATIOS.length, gear + 1); }
  if (k === "v" && !e.repeat) gear = Math.max(1, gear - 1);
  if ([" ", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key))
    e.preventDefault();
});
addEventListener("keyup", (e) => { keys[e.key.toLowerCase()] = false; });

// Demo/screenshot aid: open index.html?auto to run a scripted brake+corner so
// weight transfer and slip are visible without a keyboard. Pure demo, no effect
// on the physics or the keyboard path.
const AUTO = new URLSearchParams(location.search).has("auto");
// pure-pursuit autopilot: follow the centreline, hold a target speed. Used for
// the demo lap / screenshots (drives the car around the track and over the bridge).
function autoControls() {
  const nb = track.nearest(q[0], q[1]);
  const C = track.center;
  const tp = C[(nb.i + 10) % C.length];           // lookahead point
  let err = Math.atan2(tp.y - q[1], tp.x - q[0]) - q[2];
  while (err > Math.PI) err -= 2 * Math.PI;
  while (err < -Math.PI) err += 2 * Math.PI;
  const steerCmd = Math.max(-MAX_STEER, Math.min(MAX_STEER, 1.8 * err));
  const thr = Math.max(-1, Math.min(1, 0.5 * (22 - q[3])));   // ~22 m/s target
  return [steerCmd, thr];
}

function readControls(dt) {
  if (autoDrive || AUTO) { const u = autoControls(); steer = u[0]; return u; }
  const up = keys["arrowup"] || keys["w"];
  const dn = keys["arrowdown"] || keys["s"];
  const lf = keys["arrowleft"] || keys["a"];
  const rt = keys["arrowright"] || keys["d"];

  // steering: slew toward target, auto-centre. Speed-sensitive: less lock at
  // speed so it isn't twitchy (more realistic).
  const maxSteer = MAX_STEER * (1 - 0.55 * Math.min(Math.abs(q[3]) / 55, 1));
  let target = ((lf ? 1 : 0) - (rt ? 1 : 0)) * maxSteer;
  const ds = STEER_RATE * dt;
  if (target > steer) steer = Math.min(steer + ds, target);
  else if (target < steer) steer = Math.max(steer - ds, target);
  if (!lf && !rt) {                       // return to centre
    if (steer > 0) steer = Math.max(0, steer - ds);
    else steer = Math.min(0, steer + ds);
  }

  const hb = keys[" "] ? 1 : 0;                  // handbrake on Space

  if (reverse) {                                  // reverse gear
    engineRpm = RPM_IDLE;
    let thr = up ? -0.45 : (dn ? 1 : 0);          // up = back up, down = brake
    if (q[3] < -8) thr = Math.max(thr, 0);        // cap reverse speed
    return [steer, thr, hb];
  }

  // throttle through the gearbox; brake is direct. Rev-limiter cuts throttle.
  const dt_ = drivetrain(q[3]);
  engineRpm = dt_.rpm;
  const throttle = (up && !dt_.overRev) ? dt_.thrScale : 0;
  let thr = throttle - (dn ? 1 : 0);
  if (q[3] < 0.5 && thr < 0) thr = 0;            // brake won't drive it backward
  return [steer, thr, hb];
}

// ---- physics step ---------------------------------------------------------
function stepPhysics(dt) {
  const u = readControls(dt);
  // off-track -> low grip (grass), based on this step's position
  const nbPre = track.nearest(q[0], q[1]);
  const mu = nbPre.dist <= track.halfWidth ? MU_ROAD : MU_GRASS;
  car.mu = mu; car.tyre.mu = mu;
  q = car.rk4Step(q, u, dt);
  const nb = track.nearest(q[0], q[1]);          // post-move, shared below
  enforceBarriers(nb);
  stepBridge(dt, nb);
  return { u, onTrack: nb.dist <= track.halfWidth };
}

// Solid walls a run-off distance beyond the kerb: clamp the car back to the
// barrier and kill its outward velocity (with a slight bounce) so it can't fly
// off into the void -- it slides along the wall instead.
function enforceBarriers(nb) {
  const limit = track.halfWidth + track.runoff;
  if (nb.dist <= limit) return;
  const c = track.center[nb.i], n = track.normal[nb.i];
  const sgn = Math.sign(nb.lateral) || 1;
  q[0] = c.x + sgn * limit * n.x;                // push back onto the wall
  q[1] = c.y + sgn * limit * n.y;
  const psi = q[2], cpsi = Math.cos(psi), spsi = Math.sin(psi);
  let wvx = q[3] * cpsi - q[4] * spsi;           // body -> world velocity
  let wvy = q[3] * spsi + q[4] * cpsi;
  const ox = sgn * n.x, oy = sgn * n.y;          // outward normal
  const vn = wvx * ox + wvy * oy;
  if (vn > 0) { wvx -= 1.25 * vn * ox; wvy -= 1.25 * vn * oy; }   // remove + bounce
  q[3] = wvx * cpsi + wvy * spsi;                // world -> body velocity
  q[4] = -wvx * spsi + wvy * cpsi;
}

// Drive whichever span the car is on; the others ring down. The moving load is
// the car's weight PLUS aero downforce (aero -> bridge coupling).
function stepBridge(dt, nb) {
  const onB = track.onBridge(nb.s);
  const info = onB ? track.bridgeSpanInfo(nb.s) : null;
  const P = onB ? car.m * car.g + car.downforce(q[3], q[4]) : 0;

  for (let k = 0; k < N_SPANS; k++) {
    const active = onB && info.idx === k;
    const a = active ? info.local : null;
    const aMid = active ? a + 0.5 * dt * Math.max(q[3], 0) : null;
    bridges[k].step(dt, active ? P : 0, a, aMid);
  }

  // DAF bookkeeping for the active span (reset on entering a new span)
  const idx = onB ? info.idx : bridgeState.activeIdx;
  if (onB && (!bridgeState.onBridge || info.idx !== bridgeState.activeIdx)) {
    bridgeState.peakDyn = 0;
    bridgeState.denom = Math.abs(bridges[idx].staticMidspan(P)) || 1e-9;
  }
  const wmid = Math.abs(bridges[idx].deflection(SPAN_L / 2));
  if (onB) bridgeState.peakDyn = Math.max(bridgeState.peakDyn, wmid);
  bridgeState.onBridge = onB;
  bridgeState.activeIdx = idx;
  bridgeState.loadPos = onB ? info.local : null;
  bridgeState.P = P;
  bridgeState.daf = bridgeState.peakDyn / bridgeState.denom || 1;
}

// ---- HUD ------------------------------------------------------------------
function updateHUD(u, onTrack) {
  const [delta] = u;
  const d = car.diagnostics(q, u);             // per-wheel loads, slip, ax, ay
  const vx = q[3], vy = q[4], r = q[5];
  const aLong = d.ax, aLat = d.ay;
  const speed = Math.hypot(vx, vy);
  const aMax = car.mu * car.g;
  // axle-average slip angles (front = FL,FR; rear = RL,RR)
  const slipF = (d.alpha[0] + d.alpha[1]) / 2;
  const slipR = (d.alpha[2] + d.alpha[3]) / 2;

  const set = (id, v) => document.getElementById(id).textContent = v;
  set("speed", (speed * 3.6).toFixed(0));
  set("throttle", u[1] > 0 ? (u[1] * 100).toFixed(0) + "%" : "0%");
  set("brake", u[1] < 0 ? (-u[1] * 100).toFixed(0) + "%" : "0%");
  set("steer", (delta * 180 / Math.PI).toFixed(1));
  set("yaw", (r * 180 / Math.PI).toFixed(1));
  set("slipf", (slipF * 180 / Math.PI).toFixed(1));
  set("slipr", (slipR * 180 / Math.PI).toFixed(1));
  set("along", aLong.toFixed(2));
  set("alat", aLat.toFixed(2));
  set("grip", (Math.hypot(aLong, aLat) / aMax * 100).toFixed(0) + "%");

  // per-wheel vertical load as % of static (m*g/4) -> shows weight transfer
  const stat = car.m * car.g / 4;
  const ids = ["fl", "fr", "rl", "rr"];
  for (let i = 0; i < 4; i++)
    set("w" + ids[i], (d.Fz[i] / stat * 100).toFixed(0) + "%");

  const surf = document.getElementById("surface");
  surf.textContent = onTrack ? "ON TRACK" : "OFF (grass)";
  surf.className = onTrack ? "ok" : "warn";

  // drivetrain
  set("gear", reverse ? "R" : gear);
  set("rpm", Math.round(engineRpm));
  const bar = document.getElementById("rpmbar");
  const frac = Math.min(1, engineRpm / RPM_REDLINE);
  bar.style.width = (frac * 100).toFixed(0) + "%";
  bar.style.background = engineRpm >= RPM_REDLINE * 0.92 ? "#e7563b"
    : engineRpm >= RPM_REDLINE * 0.75 ? "#e8b339" : "#3fb950";
  const hbEl = document.getElementById("handbrake");
  hbEl.textContent = (u[2] ? "ENGAGED" : "off");
  hbEl.className = u[2] ? "v warn" : "v";

  // bridge readouts (shared with the bridge panel)
  set("bdaf", bridgeState.daf.toFixed(2));
  const wActive = Math.abs(bridges[bridgeState.activeIdx].deflection(SPAN_L / 2));
  set("bdefl", (wActive * 1e3).toFixed(3));
  const bst = document.getElementById("bstatus");
  bst.textContent = bridgeState.onBridge
    ? `span ${bridgeState.activeIdx + 1}/${N_SPANS}` : "clear";
  bst.className = bridgeState.onBridge ? "v ok" : "v";

  drawGG(document.getElementById("gg"), aLong, aLat, aMax);
  return { aLong, aLat, aMax, Fz: d.Fz };
}

// ---- aero panel (real-time 2D fluid around the car) -----------------------
const aero = new AeroField(document.getElementById("aero"));
function stepAero() {
  const vx = q[3], vy = q[4];
  const speed = Math.hypot(vx, vy);
  const beta = Math.atan2(vy, Math.max(Math.abs(vx), 0.5)) * Math.sign(vx || 1);
  const drag = 0.5 * car.rho * car.Cd * car.area * vx * vx;
  const down = car.downforce(vx, vy);
  aero.update(speed, beta, drag, down, down / (car.m * car.g));
}

// ---- bridge panel (side-elevation: 3 spans) -------------------------------
const bridgePanel = new BridgePanel(document.getElementById("bridge"), bridges);

// ---- main loop ------------------------------------------------------------
const renderer = new Renderer(document.getElementById("view"),
                              document.getElementById("minimap"));
function resize() {
  const wrap = document.getElementById("stage");
  renderer.resize(wrap.clientWidth, wrap.clientHeight);
}
addEventListener("resize", resize); resize();

let lastCtrl = [0, 0], lastOn = true, frameN = 0;
function frame(t) {
  if (lastT === null) lastT = t;
  let elapsed = Math.min((t - lastT) / 1000, 0.1);   // clamp huge gaps
  lastT = t;
  acc += elapsed;
  while (acc >= PHYS_DT) {
    if (!editMode) { const s = stepPhysics(PHYS_DT); lastCtrl = s.u; lastOn = s.onTrack; }
    acc -= PHYS_DT;
  }
  syncView();
  if (!editMode) {                       // record the driven line
    trace.push({ x: q[0], y: q[1] });
    if (trace.length > 600) trace.shift();
  }
  if (autoDrive && frameN > 0 && (frameN % 320) === 0) {  // rotate caption (~5 s)
    tourIdx = (tourIdx + 1) % TOUR.length;
    document.getElementById("banner").innerHTML = TOUR[tourIdx];
  }
  renderer.follow(view);
  renderer.clear();
  renderer.drawTerrain(track);
  renderer.drawWater(track);
  renderer.drawTrack(track);
  if (editMode) renderer.drawEditor(track);
  if (showVectors) renderer.drawTrace(trace);
  const hud = editMode ? null : updateHUD(lastCtrl, lastOn);
  const isSlip = hud ? Math.hypot(hud.aLong, hud.aLat) / hud.aMax > 0.97 : false;
  renderer.drawCar(view, lastCtrl[0], isSlip, hud ? hud.Fz : null);
  if (showVectors && !editMode)
    renderer.drawVectors(view, car.diagnostics(q, lastCtrl), lastCtrl[0]);
  renderer.drawMinimap(track, view);
  if (!editMode) {
    if ((frameN++ & 1) === 0) stepAero();     // fluid every 2nd frame (perf)
    bridgePanel.render(bridgeState);
  }
  requestAnimationFrame(frame);
}
// In AUTO (demo/screenshot) mode, advance the sim synchronously so a static
// capture lands mid-manoeuvre (headless renderers don't drive rAF in real time).
if (AUTO) {
  // drive (pure-pursuit) until the car is ~12 m onto the bridge, so a capture
  // shows the deck sagging; then develop the aero wake.
  const maxSteps = Math.round(60 / PHYS_DT);
  for (let i = 0; i < maxSteps; i++) {
    const s = stepPhysics(PHYS_DT); lastCtrl = s.u; lastOn = s.onTrack;
    if (bridgeState.onBridge && bridgeState.loadPos > 12) break;
  }
  syncView();
  for (let i = 0; i < 240; i++) stepAero();
  bridgePanel.render(bridgeState);
}
if (autoDrive) document.getElementById("banner").innerHTML = TOUR[0];
requestAnimationFrame(frame);

// ---- track editor ---------------------------------------------------------
let editMode = false;
let dragIdx = -1;
const stage = document.getElementById("stage");

function screenToWorld(ev) {
  const rect = renderer.cv.getBoundingClientRect();
  const sx = ev.clientX - rect.left, sy = ev.clientY - rect.top;
  return {
    x: renderer.cam.x + (sx - renderer.cv.width / 2) / renderer.ppm,
    y: renderer.cam.y - (sy - renderer.cv.height / 2) / renderer.ppm,
  };
}
function nearestPoint(w) {
  let best = -1, bd = 1e9;
  track.points.forEach((p, i) => {
    const d = Math.hypot(p.x - w.x, p.y - w.y);
    if (d < bd) { bd = d; best = i; }
  });
  return { i: best, d: bd };
}
stage.addEventListener("mousedown", (ev) => {
  if (!editMode) return;
  const w = screenToWorld(ev);
  const np = nearestPoint(w);
  if (ev.button === 2 && np.i >= 0 && np.d < 8 && track.points.length > 3) {
    track.points.splice(np.i, 1); track.rebuild(); return;   // right-click delete
  }
  if (np.d < 8) dragIdx = np.i;                               // grab existing
  else { track.points.push({ x: w.x, y: w.y }); track.rebuild(); } // add new
});
stage.addEventListener("mousemove", (ev) => {
  if (!editMode || dragIdx < 0) return;
  const w = screenToWorld(ev);
  track.points[dragIdx] = { x: w.x, y: w.y };
  track.rebuild();
});
addEventListener("mouseup", () => { dragIdx = -1; });
stage.addEventListener("contextmenu", (e) => { if (editMode) e.preventDefault(); });

document.getElementById("editBtn").onclick = () => {
  editMode = !editMode;
  document.getElementById("editBtn").textContent =
    editMode ? "Done editing (drive)" : "Edit track";
  document.getElementById("editHint").style.display = editMode ? "block" : "none";
  if (!editMode) resetCar();
};
function toggleVectors() {
  showVectors = !showVectors;
  const btn = document.getElementById("vecBtn");
  if (btn) btn.textContent = showVectors
    ? "Vectors + trace: ON" : "Vectors + trace: off";
}
document.getElementById("vecBtn").onclick = toggleVectors;

function setAutoDrive(on) {
  autoDrive = on;
  const btn = document.getElementById("tourBtn");
  if (btn) btn.textContent = on ? "Auto-drive tour: ON" : "Auto-drive tour: off";
  if (on) { tourIdx = 0; document.getElementById("banner").innerHTML = TOUR[0]; }
  else document.getElementById("banner").innerHTML =
    "<b>You're driving.</b> Arrows/WASD · C/V gears · Space handbrake · E reverse · T vectors.";
}
document.getElementById("tourBtn").onclick = () => setAutoDrive(!autoDrive);
document.getElementById("resetCarBtn").onclick = resetCar;
document.getElementById("saveBtn").onclick = () => {
  localStorage.setItem("hubTrack", JSON.stringify(track.toJSON()));
  flash("saved to browser");
};
document.getElementById("loadBtn").onclick = () => {
  const s = localStorage.getItem("hubTrack");
  if (s) { track = Track.fromJSON(JSON.parse(s)); resetCar(); flash("loaded"); }
  else flash("nothing saved");
};
document.getElementById("defaultBtn").onclick = () => {
  track = defaultTrack(); resetCar(); flash("default track");
};
function flash(msg) {
  const el = document.getElementById("flash");
  el.textContent = msg; el.style.opacity = 1;
  setTimeout(() => { el.style.opacity = 0; }, 1200);
}
