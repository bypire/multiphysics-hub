// ============================================================================
// render.js -- canvas drawing for the top-down hub: track, car, minimap, g-g.
// ----------------------------------------------------------------------------
// World units = metres (x right, y "up" in world; canvas y is flipped). A
// follow-camera keeps the car centred; north stays up so the track reads like a
// map and the car rotates on screen.
// ============================================================================

class Renderer {
  constructor(canvas, miniCanvas) {
    this.cv = canvas;
    this.ctx = canvas.getContext("2d");
    this.mini = miniCanvas;
    this.mctx = miniCanvas.getContext("2d");
    this.ppm = 6.6;        // pixels per metre (main view zoom)
    this.cam = { x: 0, y: 0 };
    this.t = 0;            // frame counter (ripple animation)
    this.hills = null;     // lazily generated terrain
  }

  resize(w, h) { this.cv.width = w; this.cv.height = h; }

  // ---- 2.5D scenery: terrain + sea under the bridge -----------------------
  ensureHills(track) {
    if (this.hills) return;
    const b = track.bounds(), pad = 260;
    let seed = 7919;
    const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
    const n = 110;
    this.hills = [];
    for (let i = 0; i < n; i++)
      this.hills.push({
        x: b.minx - pad + rnd() * (b.w + 2 * pad),
        y: b.miny - pad + rnd() * (b.h + 2 * pad),
        r: 36 + rnd() * 120, s: (rnd() * 3) | 0,
      });
    // pre-render 3 soft hill sprites once (cheap drawImage instead of per-frame
    // radial gradients -> big perf win)
    const cols = ["#26371f", "#324a26", "#3c3a2a"];
    this.hillSprite = cols.map((col) => {
      const c = document.createElement("canvas"); c.width = c.height = 128;
      const cg = c.getContext("2d");
      const gr = cg.createRadialGradient(64, 64, 0, 64, 64, 64);
      gr.addColorStop(0, col); gr.addColorStop(1, "rgba(29,44,27,0)");
      cg.fillStyle = gr; cg.fillRect(0, 0, 128, 128);
      return c;
    });
  }

  // rolling-terrain backdrop (world-space blobs, so it scrolls with motion)
  drawTerrain(track) {
    const g = this.ctx, W = this.cv.width, H = this.cv.height;
    g.fillStyle = "#1d2c1b"; g.fillRect(0, 0, W, H);     // grassland base
    this.ensureHills(track);
    for (const h of this.hills) {
      const [sx, sy] = this.toScreen(h.x, h.y);
      const rp = h.r * this.ppm;
      if (sx < -rp || sx > W + rp || sy < -rp || sy > H + rp) continue;  // cull
      g.drawImage(this.hillSprite[h.s], sx - rp, sy - rp, 2 * rp, 2 * rp);
    }
  }

  // sea crossing under the bridge, deck shadow + piers -> elevated-deck illusion
  drawWater(track) {
    this.t++;
    if (!track.bridge) return;
    const g = this.ctx, bi = track.bridgeIndices();
    if (bi.length < 2) return;
    const half = 48;                                     // water reach each side [m]

    // water polygon: bridge centreline offset +/- half along the normal
    const left = [], right = [];
    for (const i of bi) {
      const c = track.center[i], n = track.normal[i];
      left.push([c.x + half * n.x, c.y + half * n.y]);
      right.push([c.x - half * n.x, c.y - half * n.y]);
    }
    g.beginPath();
    left.forEach((p, k) => { const [x, y] = this.toScreen(p[0], p[1]); k ? g.lineTo(x, y) : g.moveTo(x, y); });
    for (let k = right.length - 1; k >= 0; k--) { const [x, y] = this.toScreen(right[k][0], right[k][1]); g.lineTo(x, y); }
    g.closePath();
    g.fillStyle = "#16384f"; g.fill();                   // deep water
    // ripples: wavy lighter lines parallel to the channel
    g.strokeStyle = "rgba(120,180,210,0.18)"; g.lineWidth = 1.5;
    for (let band = -3; band <= 3; band++) {
      g.beginPath();
      bi.forEach((i, k) => {
        const c = track.center[i], nrm = track.normal[i];
        const off = band * 11 + 2.5 * Math.sin(this.t * 0.05 + k * 0.5 + band);
        const [x, y] = this.toScreen(c.x + off * nrm.x, c.y + off * nrm.y);
        k ? g.lineTo(x, y) : g.moveTo(x, y);
      });
      g.stroke();
    }

    // deck drop-shadow on the water (offset toward screen-bottom) + piers
    const hw = track.halfWidth;
    g.fillStyle = "rgba(0,0,0,0.35)";
    g.beginPath();
    bi.forEach((i, k) => { const c = track.center[i], n = track.normal[i];
      const [x, y] = this.toScreen(c.x + hw * n.x, c.y + hw * n.y); k ? g.lineTo(x, y + 9) : g.moveTo(x, y + 9); });
    for (let k = bi.length - 1; k >= 0; k--) { const i = bi[k], c = track.center[i], n = track.normal[i];
      const [x, y] = this.toScreen(c.x - hw * n.x, c.y - hw * n.y); g.lineTo(x, y + 9); }
    g.closePath(); g.fill();

    // piers at each span support, extruded toward screen-bottom (height cue)
    const br = track.bridge;
    for (let s = 0; s <= br.nSpans; s++) {
      const idx = track.indexAtS(br.s0 + s * br.spanL);
      const c = track.center[idx], n = track.normal[idx];
      for (const sgn of [0.6, -0.6]) {
        const [x, y] = this.toScreen(c.x + sgn * hw * n.x, c.y + sgn * hw * n.y);
        g.fillStyle = "#0c1822";
        g.fillRect(x - 3, y, 6, 22);                     // pillar down into water
        g.fillStyle = "#13242f";
        g.fillRect(x - 3, y + 20, 6, 3);                 // footing
      }
    }
  }

  // world -> screen (camera centred, y flipped)
  toScreen(x, y) {
    return [
      this.cv.width / 2 + (x - this.cam.x) * this.ppm,
      this.cv.height / 2 - (y - this.cam.y) * this.ppm,
    ];
  }

  follow(car) { this.cam.x = car.X; this.cam.y = car.Y; }

  clear() {
    const g = this.ctx;
    g.fillStyle = "#0d1117";
    g.fillRect(0, 0, this.cv.width, this.cv.height);
  }

  polyToPath(g, pts, close) {
    g.beginPath();
    for (let i = 0; i < pts.length; i++) {
      const [sx, sy] = this.toScreen(pts[i].x, pts[i].y);
      i ? g.lineTo(sx, sy) : g.moveTo(sx, sy);
    }
    if (close) g.closePath();
  }

  drawTrack(track) {
    const g = this.ctx;
    if (track.center.length < 2) return;

    // barriers: red/white walls a runoff distance beyond each edge (one polyline
    // per side: a solid red base + white dashes on top -> cheap kerb look)
    const run = track.halfWidth + (track.runoff || 6);
    const barrier = (sign) => {
      const pts = track.center.map((c, i) => ({
        x: c.x + sign * run * track.normal[i].x,
        y: c.y + sign * run * track.normal[i].y }));
      g.lineWidth = 4; g.lineCap = "butt";
      g.strokeStyle = "#c43838"; g.setLineDash([]);
      this.polyToPath(g, pts, track.closed); g.stroke();
      g.strokeStyle = "#eaeaea"; g.setLineDash([9, 9]);
      this.polyToPath(g, pts, track.closed); g.stroke();
      g.setLineDash([]);
    };
    barrier(1); barrier(-1);
    // asphalt: fill between left and right boundary as one ribbon
    g.beginPath();
    const L = track.left, R = track.right;
    for (let i = 0; i < L.length; i++) {
      const [sx, sy] = this.toScreen(L[i].x, L[i].y);
      i ? g.lineTo(sx, sy) : g.moveTo(sx, sy);
    }
    for (let i = R.length - 1; i >= 0; i--) {
      const [sx, sy] = this.toScreen(R[i].x, R[i].y);
      g.lineTo(sx, sy);
    }
    g.closePath();
    g.fillStyle = "#24262b";
    g.fill();

    // boundary kerbs
    g.lineWidth = 2;
    g.strokeStyle = "#c9ced6";
    this.polyToPath(g, track.left, track.closed); g.stroke();
    this.polyToPath(g, track.right, track.closed); g.stroke();

    // centreline (dashed)
    g.save();
    g.setLineDash([10, 12]);
    g.lineWidth = 1.5;
    g.strokeStyle = "#5a6472";
    this.polyToPath(g, track.center, track.closed); g.stroke();
    g.restore();

    // bridge segment: highlight the deck edges (steel-blue rails)
    const bi = track.bridgeIndices ? track.bridgeIndices() : [];
    if (bi.length > 1) {
      g.strokeStyle = "#6ea8d8"; g.lineWidth = 4;
      for (const side of [track.left, track.right]) {
        g.beginPath();
        bi.forEach((i, k) => {
          const [sx, sy] = this.toScreen(side[i].x, side[i].y);
          k ? g.lineTo(sx, sy) : g.moveTo(sx, sy);
        });
        g.stroke();
      }
      // a couple of deck cross-members for legibility
      g.strokeStyle = "rgba(110,168,216,0.5)"; g.lineWidth = 2;
      for (let k = 0; k < bi.length; k += 6) {
        const i = bi[k];
        const a = this.toScreen(track.left[i].x, track.left[i].y);
        const b = this.toScreen(track.right[i].x, track.right[i].y);
        g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
      }
    }

    // start/finish line
    const c0 = track.center[0], n0 = track.normal[0], hw = track.halfWidth;
    const a = this.toScreen(c0.x + hw * n0.x, c0.y + hw * n0.y);
    const b = this.toScreen(c0.x - hw * n0.x, c0.y - hw * n0.y);
    g.strokeStyle = "#e8b339"; g.lineWidth = 4;
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
  }

  // draw the control points + handles when the editor is on
  drawEditor(track) {
    const g = this.ctx;
    g.fillStyle = "#5cf";
    for (const p of track.points) {
      const [sx, sy] = this.toScreen(p.x, p.y);
      g.beginPath(); g.arc(sx, sy, 6, 0, 2 * Math.PI); g.fill();
    }
  }

  // 4-wheel car: a car-like top-down silhouette (tapered nose, cockpit canopy,
  // rear wing) with the front pair steered by delta. If per-wheel loads Fz are
  // given, each tyre reddens/grows with its load so weight transfer is visible.
  drawCar(car, delta, slipping, Fz) {
    const g = this.ctx;
    const [cx, cy] = this.toScreen(car.X, car.Y);
    const ppm = this.ppm;
    const S = 3.0;                             // car drawn larger than scale (readable)

    // ground shadow, offset toward screen-bottom for a 2.5D height cue
    g.save();
    g.translate(cx, cy + 6); g.rotate(-car.psi);
    g.fillStyle = "rgba(0,0,0,0.32)";
    roundRect(g, -2.5 * ppm * S, -1.05 * ppm * S, 5 * ppm * S, 2.1 * ppm * S, 6);
    g.fill();
    g.restore();

    g.save();
    g.translate(cx, cy);
    g.rotate(-car.psi);            // screen y is flipped -> negate heading
    const Lh = 2.35 * ppm * S, Wh = 0.92 * ppm * S;  // half length / width

    // wheels first so the body overlaps them; order [FL, FR, RL, RR]
    const wl = 0.78 * ppm * S, ww = 0.34 * ppm * S;
    const af = car.a * ppm * S, ar = -car.b * ppm * S, tr = Wh * 1.02;
    const stat = (car.m * car.g) / 4;
    const pos = [[af, -tr, delta], [af, tr, delta], [ar, -tr, 0], [ar, tr, 0]];
    for (let i = 0; i < 4; i++) {
      const load = Fz ? Fz[i] / stat : 1;
      const t = Math.max(0, Math.min(1.4, load)) / 1.4;
      const rC = Math.round(24 + 200 * t), gC = Math.round(30 + 40 * t);
      g.fillStyle = Fz ? `rgb(${rC},${gC},${48})` : "#15171c";
      drawWheel(g, pos[i][0], pos[i][1], pos[i][2],
                wl * (0.75 + 0.4 * Math.min(load, 1.6)), ww);
    }

    // body outline: tapered nose at +x, broad cabin, squared tail (polygon)
    const body = [
      [-Lh, Wh * 0.62], [-Lh * 0.82, Wh * 0.98], [Lh * 0.18, Wh],
      [Lh * 0.68, Wh * 0.82], [Lh * 0.95, Wh * 0.36], [Lh, 0],
      [Lh * 0.95, -Wh * 0.36], [Lh * 0.68, -Wh * 0.82], [Lh * 0.18, -Wh],
      [-Lh * 0.82, -Wh * 0.98], [-Lh, -Wh * 0.62],
    ];
    const grad = g.createLinearGradient(-Lh, 0, Lh, 0);
    if (slipping) { grad.addColorStop(0, "#b23423"); grad.addColorStop(1, "#f0664a"); }
    else { grad.addColorStop(0, "#1f4f9e"); grad.addColorStop(1, "#4d92f4"); }
    g.fillStyle = grad; g.strokeStyle = "#e6edf3"; g.lineWidth = 1.6;
    g.beginPath();
    body.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]));
    g.closePath(); g.fill(); g.stroke();

    // livery: twin centre stripes running nose->tail
    g.fillStyle = "rgba(255,255,255,0.16)";
    roundRect(g, -Lh * 0.9, -Wh * 0.28, Lh * 1.75, Wh * 0.14, 1); g.fill();
    roundRect(g, -Lh * 0.9, Wh * 0.14, Lh * 1.75, Wh * 0.14, 1); g.fill();
    // glossy highlight band (texture/sheen)
    const sheen = g.createLinearGradient(0, -Wh, 0, Wh);
    sheen.addColorStop(0, "rgba(255,255,255,0.22)");
    sheen.addColorStop(0.5, "rgba(255,255,255,0)");
    sheen.addColorStop(1, "rgba(0,0,0,0.18)");
    g.fillStyle = sheen;
    g.beginPath();
    body.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1]));
    g.closePath(); g.fill();

    // headlights (front) + tail lights (rear)
    g.fillStyle = "#fff7cf";
    roundRect(g, Lh * 0.78, -Wh * 0.62, Lh * 0.1, Wh * 0.22, 1); g.fill();
    roundRect(g, Lh * 0.78, Wh * 0.40, Lh * 0.1, Wh * 0.22, 1); g.fill();
    g.fillStyle = "#e7563b";
    roundRect(g, -Lh * 0.97, -Wh * 0.7, Lh * 0.06, Wh * 0.45, 1); g.fill();
    roundRect(g, -Lh * 0.97, Wh * 0.25, Lh * 0.06, Wh * 0.45, 1); g.fill();
    // side mirrors
    g.fillStyle = "#12161c";
    roundRect(g, Lh * 0.1, -Wh * 1.06, Lh * 0.12, Wh * 0.2, 1); g.fill();
    roundRect(g, Lh * 0.1, Wh * 0.86, Lh * 0.12, Wh * 0.2, 1); g.fill();

    // rear wing
    g.fillStyle = "#11151b";
    roundRect(g, -Lh * 1.0, -Wh * 0.9, Lh * 0.12, Wh * 1.8, 2); g.fill();
    // cockpit canopy (tinted glass), set back from the nose
    g.fillStyle = "rgba(150,200,255,0.32)"; g.strokeStyle = "rgba(230,240,255,0.5)";
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(Lh * 0.42, 0); g.lineTo(Lh * 0.12, Wh * 0.55);
    g.lineTo(-Lh * 0.3, Wh * 0.45); g.lineTo(-Lh * 0.3, -Wh * 0.45);
    g.lineTo(Lh * 0.12, -Wh * 0.55); g.closePath(); g.fill(); g.stroke();
    g.restore();
  }

  // the driven line (fading trail of past positions)
  drawTrace(trace) {
    const g = this.ctx;
    if (trace.length < 2) return;
    g.lineWidth = 2.5; g.lineCap = "round";
    for (let i = 1; i < trace.length; i++) {
      const a = this.toScreen(trace[i - 1].x, trace[i - 1].y);
      const b = this.toScreen(trace[i].x, trace[i].y);
      const al = i / trace.length;
      g.strokeStyle = `rgba(90,200,255,${(0.1 + 0.55 * al).toFixed(3)})`;
      g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
    }
  }

  // force / velocity vectors on the car -- how the mechanics actually push it.
  // diag from car.diagnostics(): Fy[4], Fx[4] in tyre frames; ax, ay body accel.
  drawVectors(view, diag, delta) {
    const g = this.ctx;
    const c = Math.cos(view.psi), s = Math.sin(view.psi);
    const rot = (bx, by) => [bx * c - by * s, bx * s + by * c];  // body -> world
    const FS = 0.013, VS = 4.2;                   // force/velocity -> px scales
    const clamp = (v, m) => { const n = Math.hypot(v[0], v[1]); return n > m ? [v[0] * m / n, v[1] * m / n] : v; };

    const arrow = (wx, wy, vx, vy, col, lab) => {
      const [x0, y0] = this.toScreen(wx, wy);
      const ex = x0 + vx, ey = y0 - vy;           // screen y is flipped
      g.strokeStyle = col; g.fillStyle = col; g.lineWidth = 3.2;
      g.beginPath(); g.moveTo(x0, y0); g.lineTo(ex, ey); g.stroke();
      const ang = Math.atan2(ey - y0, ex - x0), h = 10;
      g.beginPath(); g.moveTo(ex, ey);
      g.lineTo(ex - h * Math.cos(ang - 0.45), ey - h * Math.sin(ang - 0.45));
      g.lineTo(ex - h * Math.cos(ang + 0.45), ey - h * Math.sin(ang + 0.45));
      g.closePath(); g.fill();
      if (lab) { g.font = "bold 12px monospace"; g.fillStyle = col; g.fillText(lab, ex + 5, ey - 5); }
    };

    const a = view.a, b = view.b;
    const fx = view.X + a * c, fy = view.Y + a * s;     // front axle (world)
    const rx = view.X - b * c, ry = view.Y - b * s;     // rear axle (world)

    // velocity at the CoG (white)
    let vvec = clamp(rot(view.vx, view.vy).map((z) => z * VS), 130);
    arrow(view.X, view.Y, vvec[0], vvec[1], "#e6edf3", "v");

    // front tyre lateral force (yellow), acts perpendicular to the steered wheel
    const Fyf = diag.Fy[0] + diag.Fy[1];
    let ff = clamp(rot(-Fyf * Math.sin(delta), Fyf * Math.cos(delta)).map((z) => z * FS), 120);
    arrow(fx, fy, ff[0], ff[1], "#e8b339", "Ff");

    // rear tyre force (orange): lateral + drive/brake along the body
    const Fyr = diag.Fy[2] + diag.Fy[3], Fxr = diag.Fx[2] + diag.Fx[3];
    let fr = clamp(rot(Fxr, Fyr).map((z) => z * FS), 120);
    arrow(rx, ry, fr[0], fr[1], "#f0883e", "Fr");
  }

  // bottom-right minimap: whole track + car dot
  drawMinimap(track, car) {
    const g = this.mctx, W = this.mini.width, H = this.mini.height;
    g.fillStyle = "rgba(13,17,23,0.85)";
    g.fillRect(0, 0, W, H);
    g.strokeStyle = "#30363d"; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
    if (track.center.length < 2) return;
    const b = track.bounds(), pad = 14;
    const sc = Math.min((W - 2 * pad) / b.w, (H - 2 * pad) / b.h);
    const ox = (W - b.w * sc) / 2, oy = (H - b.h * sc) / 2;
    const mx = (x) => ox + (x - b.minx) * sc;
    const my = (y) => H - (oy + (y - b.miny) * sc);   // flip y
    g.strokeStyle = "#8b949e"; g.lineWidth = 2;
    g.beginPath();
    track.center.forEach((p, i) =>
      i ? g.lineTo(mx(p.x), my(p.y)) : g.moveTo(mx(p.x), my(p.y)));
    if (track.closed) g.closePath();
    g.stroke();
    // car dot
    g.fillStyle = "#3b82f6";
    g.beginPath(); g.arc(mx(car.X), my(car.Y), 4, 0, 2 * Math.PI); g.fill();
  }
}

function roundRect(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r);
  g.closePath();
}

function drawWheel(g, x, y, ang, wl, ww) {
  g.save(); g.translate(x, y); g.rotate(-ang);
  g.fillRect(-wl / 2, -ww / 2, wl, ww);
  g.restore();
}

// g-g (friction-circle) diagram drawn into a small standalone canvas.
function drawGG(canvas, ax, ay, aMax) {
  const g = canvas.getContext("2d"), W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2, R = Math.min(W, H) / 2 - 10;
  g.clearRect(0, 0, W, H);
  // friction circle
  g.strokeStyle = "#444c56"; g.lineWidth = 1;
  for (const frac of [1 / 3, 2 / 3, 1]) {
    g.beginPath(); g.arc(cx, cy, R * frac, 0, 2 * Math.PI); g.stroke();
  }
  g.beginPath(); g.moveTo(cx - R, cy); g.lineTo(cx + R, cy);
  g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.stroke();
  // current acceleration dot (ay -> x axis, ax -> y axis, up = accel)
  const px = cx + (ay / aMax) * R;
  const py = cy - (ax / aMax) * R;
  const mag = Math.hypot(ax, ay) / aMax;
  g.fillStyle = mag > 0.97 ? "#e7563b" : "#3fb950";
  g.beginPath(); g.arc(px, py, 5, 0, 2 * Math.PI); g.fill();
  g.fillStyle = "#8b949e"; g.font = "10px monospace";
  g.fillText("ay", cx + R - 14, cy - 4);
  g.fillText("ax", cx + 4, cy - R + 10);
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { Renderer, drawGG };
}
