// ============================================================================
// bridge_render.js -- side-elevation panel: a 3-span viaduct sagging under the
// car, coloured by bending moment, with a live mid-span deflection + DAF readout.
// ----------------------------------------------------------------------------
// Each span is one verified modal beam (bridges[k]). The car rides whichever span
// it is on; that span deflects, the others ring down. Real deflections are tiny
// (~0.2 mm), so the sag is drawn with a magnification factor that is shown.
// ============================================================================

class BridgePanel {
  constructor(canvas, bridges) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.bridges = bridges;
    this.n = bridges.length;
    this.nx = 40;
    this.trace = new Float32Array(170);
    this.tracePtr = 0;
    const wStat = Math.abs(bridges[0].staticMidspan(1300 * 9.81)) || 1e-6;
    this.targetPx = 7;                   // subtle, more realistic sag
    this.scale = this.targetPx / wStat;
  }

  render(state) {
    const g = this.ctx, W = this.canvas.width, H = this.canvas.height;
    const L = this.bridges[0].L;
    const mL = 24, mR = 14, mT = 26, mB = 40, gap = 10;
    const base = mT + (H - mT - mB) * 0.40;
    const totalW = W - mL - mR;
    const spanW = (totalW - gap * (this.n - 1)) / this.n;

    g.fillStyle = "#0b1119"; g.fillRect(0, 0, W, H);

    // global max |moment| for consistent colour scaling across spans
    let Mmax = 1e-9;
    const sampled = this.bridges.map((br) => {
      const ws = [], Ms = [];
      for (let i = 0; i <= this.nx; i++) {
        const x = (i / this.nx) * L;
        ws.push(br.deflection(x));
        const M = br.bendingMoment(x); Ms.push(M);
        Mmax = Math.max(Mmax, Math.abs(M));
      }
      return { ws, Ms };
    });

    for (let k = 0; k < this.n; k++) {
      const x0 = mL + k * (spanW + gap);
      const px = (x) => x0 + (x / L) * spanW;
      const { ws, Ms } = sampled[k];

      // undeflected reference + supports (piers)
      g.strokeStyle = "#2b3947"; g.setLineDash([4, 5]); g.lineWidth = 1;
      g.beginPath(); g.moveTo(px(0), base); g.lineTo(px(L), base); g.stroke();
      g.setLineDash([]);
      g.fillStyle = "#5b6b7d";
      for (const xe of [0, L]) {
        g.beginPath(); g.moveTo(px(xe), base);
        g.lineTo(px(xe) - 6, base + 11); g.lineTo(px(xe) + 6, base + 11);
        g.closePath(); g.fill();
      }

      // deflected beam, colour by |bending moment|
      g.lineWidth = 5; g.lineCap = "round";
      for (let i = 0; i < this.nx; i++) {
        const t = Math.abs(0.5 * (Ms[i] + Ms[i + 1])) / Mmax;
        g.strokeStyle = momentColor(t);
        g.beginPath();
        g.moveTo(px((i / this.nx) * L), base + ws[i] * this.scale);
        g.lineTo(px(((i + 1) / this.nx) * L), base + ws[i + 1] * this.scale);
        g.stroke();
      }

      // the car on the active span
      if (state.onBridge && state.activeIdx === k && state.loadPos != null) {
        const lp = state.loadPos, wHere = this.bridges[k].deflection(lp);
        const cxp = px(lp), cyp = base + wHere * this.scale - 6;
        g.fillStyle = "#3b82f6"; g.strokeStyle = "#dfe6ee"; g.lineWidth = 1;
        roundRectB(g, cxp - 8, cyp - 4, 16, 8, 2); g.fill(); g.stroke();
      }
    }

    // mid-span deflection trace of the active span (scrolling)
    const act = this.bridges[state.activeIdx];
    const wmid = act.deflection(L / 2) * 1e3;       // mm
    this.trace[this.tracePtr] = wmid;
    this.tracePtr = (this.tracePtr + 1) % this.trace.length;
    const ty = H - mB + 28, th = 11;
    let tmax = 1e-6;
    for (const v of this.trace) tmax = Math.max(tmax, Math.abs(v));
    g.strokeStyle = "#3fb98a"; g.lineWidth = 1; g.beginPath();
    for (let i = 0; i < this.trace.length; i++) {
      const idx = (this.tracePtr + i) % this.trace.length;
      const xv = mL + (i / this.trace.length) * totalW;
      const yv = ty - (this.trace[idx] / tmax) * th;
      i ? g.lineTo(xv, yv) : g.moveTo(xv, yv);
    }
    g.stroke();

    // labels
    g.font = "11px monospace"; g.textBaseline = "top";
    g.fillStyle = "rgba(0,0,0,0.4)"; g.fillRect(0, 0, W, 18);
    g.fillStyle = "#cdd9e5";
    const f1 = act.frequenciesHz()[0].toFixed(2);
    g.fillText(`BRIDGE — ${this.n}×${L.toFixed(0)} m viaduct · f1 ${f1} Hz · VBI beam FEM`,
               6, 3);
    const mag = Math.round(this.scale / (spanW / L));
    g.fillStyle = state.onBridge ? "#e8b339" : "#7d8794";
    g.fillText(state.onBridge ? `● ON SPAN ${state.activeIdx + 1}/${this.n}`
                              : "○ viaduct clear", 6, H - mB + 1);
    g.fillStyle = "#9fb0c0";
    g.fillText(`mid-span ${Math.abs(wmid).toFixed(3)} mm (sag ×${mag})   ` +
               `DAF ${state.daf.toFixed(2)}`, 6, H - 14);
  }
}

// blue (low) -> cyan -> yellow -> red (high) moment colormap
function momentColor(t) {
  t = Math.max(0, Math.min(1, t));
  const r = Math.round(255 * Math.min(1, 2 * t));
  const g = Math.round(255 * Math.min(1, 2 * (1 - Math.abs(t - 0.5) * 2) + 0.3));
  const b = Math.round(255 * Math.max(0, 1 - 2 * t));
  return `rgb(${r},${Math.max(40, g)},${Math.max(40, b)})`;
}

function roundRectB(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r); g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r); g.arcTo(x, y, x + w, y, r); g.closePath();
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { BridgePanel };
}
