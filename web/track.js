// ============================================================================
// track.js -- user-made 2D track: control points -> smooth centreline.
// ----------------------------------------------------------------------------
// A track is a closed loop of control points. We fit a Catmull-Rom spline
// through them and sample it densely to get the centreline, with per-sample
// arc length, unit tangent and unit normal. Left/right boundaries are the
// centreline offset by +-halfWidth along the normal. The editor (main.js)
// mutates `points`; call rebuild() to resample.
// ============================================================================

class Track {
  constructor(points, halfWidth = 7.0, closed = true) {
    this.points = points.map((p) => ({ x: p.x, y: p.y })); // control points
    this.halfWidth = halfWidth;
    this.runoff = 6.0;          // grass run-off beyond the kerb before the wall
    this.closed = closed;
    this.rebuild();
  }

  // Catmull-Rom interpolation of one segment (p1->p2), param t in [0,1].
  static catmull(p0, p1, p2, p3, t) {
    const t2 = t * t, t3 = t2 * t;
    const f = (a, b, c, d) =>
      0.5 * ((2 * b) + (-a + c) * t +
             (2 * a - 5 * b + 4 * c - d) * t2 +
             (-a + 3 * b - 3 * c + d) * t3);
    return { x: f(p0.x, p1.x, p2.x, p3.x), y: f(p0.y, p1.y, p2.y, p3.y) };
  }

  rebuild(samplesPerSeg = 24) {
    const P = this.points;
    const n = P.length;
    const cl = []; // centreline samples {x,y}
    if (n < 3) { this.center = []; this.left = []; this.right = []; return; }

    const seg = this.closed ? n : n - 1;
    for (let i = 0; i < seg; i++) {
      const p0 = P[(i - 1 + n) % n];
      const p1 = P[i];
      const p2 = P[(i + 1) % n];
      const p3 = P[(i + 2) % n];
      for (let k = 0; k < samplesPerSeg; k++) {
        cl.push(Track.catmull(p0, p1, p2, p3, k / samplesPerSeg));
      }
    }
    if (!this.closed) cl.push(P[n - 1]);

    // arc length, tangent, normal (left normal = rotate tangent +90 deg)
    const N = cl.length;
    const s = new Float64Array(N);
    const tang = [], norm = [];
    for (let i = 0; i < N; i++) {
      const a = cl[(i - 1 + N) % N], b = cl[(i + 1) % N];
      let tx = b.x - a.x, ty = b.y - a.y;
      const len = Math.hypot(tx, ty) || 1;
      tx /= len; ty /= len;
      tang.push({ x: tx, y: ty });
      norm.push({ x: -ty, y: tx });           // left normal
      if (i > 0) s[i] = s[i - 1] + Math.hypot(cl[i].x - cl[i - 1].x,
                                              cl[i].y - cl[i - 1].y);
    }
    this.length = s[N - 1] + (this.closed
      ? Math.hypot(cl[0].x - cl[N - 1].x, cl[0].y - cl[N - 1].y) : 0);

    this.center = cl;
    this.s = s;
    this.tangent = tang;
    this.normal = norm;
    this.left = cl.map((c, i) => ({
      x: c.x + this.halfWidth * norm[i].x, y: c.y + this.halfWidth * norm[i].y }));
    this.right = cl.map((c, i) => ({
      x: c.x - this.halfWidth * norm[i].x, y: c.y - this.halfWidth * norm[i].y }));
  }

  // --- bridge: nSpans simply-supported spans in a row (a viaduct) ------------
  setBridge(frac, spanL, nSpans = 3) {
    this.bridge = { s0: frac * this.length, spanL, nSpans, L: spanL * nSpans };
  }
  onBridge(s) {
    if (!this.bridge) return false;
    return s >= this.bridge.s0 && s <= this.bridge.s0 + this.bridge.L;
  }
  // which span the load is on + its position along that span
  bridgeSpanInfo(s) {
    const d = s - this.bridge.s0, sL = this.bridge.spanL;
    const idx = Math.min(this.bridge.nSpans - 1, Math.max(0, Math.floor(d / sL)));
    return { idx, local: d - idx * sL };
  }
  // centreline sample indices that fall on the bridge (for drawing the deck)
  bridgeIndices() {
    if (!this.bridge) return [];
    const out = [];
    for (let i = 0; i < this.center.length; i++)
      if (this.onBridge(this.s[i])) out.push(i);
    return out;
  }
  // centreline sample index nearest a given arc length (for pier placement)
  indexAtS(s) {
    let best = 0, bd = Infinity;
    for (let i = 0; i < this.s.length; i++) {
      const d = Math.abs(this.s[i] - s);
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  // Nearest centreline sample index + signed lateral offset (for on/off track,
  // and for placing the bridge along the lap).
  nearest(x, y) {
    let best = 0, bestD2 = Infinity;
    for (let i = 0; i < this.center.length; i++) {
      const dx = x - this.center[i].x, dy = y - this.center[i].y;
      const d2 = dx * dx + dy * dy;
      if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    const c = this.center[best], nrm = this.normal[best];
    const lateral = (x - c.x) * nrm.x + (y - c.y) * nrm.y;
    return { i: best, dist: Math.sqrt(bestD2), lateral, s: this.s[best] };
  }

  onTrack(x, y) { return this.nearest(x, y).dist <= this.halfWidth; }

  // Start pose: on the centreline at sample 0, heading along the tangent.
  startPose() {
    const c = this.center[0], t = this.tangent[0];
    return { x: c.x, y: c.y, psi: Math.atan2(t.y, t.x) };
  }

  // axis-aligned bounds (for fitting the minimap / initial camera)
  bounds() {
    let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
    for (const p of this.center) {
      minx = Math.min(minx, p.x); miny = Math.min(miny, p.y);
      maxx = Math.max(maxx, p.x); maxy = Math.max(maxy, p.y);
    }
    return { minx, miny, maxx, maxy, w: maxx - minx, h: maxy - miny };
  }

  toJSON() {
    return { points: this.points, halfWidth: this.halfWidth, closed: this.closed };
  }
  static fromJSON(o) { return new Track(o.points, o.halfWidth, o.closed); }
}

// A default circuit: a smooth closed loop with a long straight, a hairpin and
// some sweepers -- enough to feel the car work. Control points in metres.
function defaultTrack() {
  const pts = [
    { x: -160, y: -90 }, { x: 40, y: -120 }, { x: 180, y: -90 },
    { x: 240, y: 0 }, { x: 190, y: 80 }, { x: 90, y: 70 },
    { x: 60, y: 10 }, { x: -20, y: -10 }, { x: -70, y: 60 },
    { x: -160, y: 90 }, { x: -220, y: 30 }, { x: -210, y: -40 },
  ];
  const t = new Track(pts, 11.0, true);          // wider road
  t.setBridge(0.12, 40.0, 3);  // three 40 m spans in a row at 12% of the lap
  return t;
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { Track, defaultTrack };
}
