// aero.js -- the 2D aero panel: fluid around the car silhouette.
//
// Runs a StableFluid solve each frame in the car's frame: freestream enters from
// the left at the car's current speed, the silhouette is an immersed solid, and
// the wake forms downstream. Sideslip yaws the silhouette into the flow, growing
// the wake. The drag/downforce numbers shown come from the coefficient model and
// are what the dynamics use; the field itself is qualitative (2D, low-Reynolds).

class AeroField {
  constructor(canvas, { nx = 140, ny = 78 } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.nx = nx; this.ny = ny;
    this.fluid = new StableFluid(nx, ny);
    // offscreen buffer at grid resolution, upscaled to the canvas
    this.img = this.ctx.createImageData(nx, ny);
    this.buf = document.createElement("canvas");
    this.buf.width = nx; this.buf.height = ny;
    this.bctx = this.buf.getContext("2d");
    // car silhouette geometry in grid cells
    this.cx = 0.34 * nx; this.cy = 0.5 * ny;
    this.Lx = 0.13 * nx;        // half-length
    this.Ly = 0.058 * ny;       // half-width
    this.lastBeta = 1e9;
    this.buildMask(0);
  }

  // superellipse car body (flat sides, rounded ends), rotated by sideslip beta
  buildMask(beta) {
    const { nx, ny, fluid, cx, cy, Lx, Ly } = this;
    const c = Math.cos(beta), s = Math.sin(beta);
    fluid.solid.fill(0);
    for (let i = 1; i <= nx; i++)
      for (let j = 1; j <= ny; j++) {
        const dx = i - cx, dy = j - cy;
        const X = dx * c + dy * s, Y = -dx * s + dy * c;   // car-local
        const e = Math.pow(Math.abs(X / Lx), 4) + Math.pow(Math.abs(Y / Ly), 2);
        if (e <= 1.0) fluid.solid[i + fluid.W * j] = 1;
      }
    this.lastBeta = beta;
  }

  // advance + draw. speed [m/s], beta [rad] sideslip, dragN/downN for readout.
  update(speed, beta, dragN, downN, gripGain) {
    const f = this.fluid;
    f.Uin = Math.max(0.25, Math.min(speed / 45, 1.4));   // model inflow
    f.Vin = 0.0;
    if (Math.abs(beta - this.lastBeta) > 0.02) this.buildMask(beta);
    f.step();
    this.render(speed, dragN, downN, gripGain);
  }

  render(speed, dragN, downN, gripGain) {
    const { nx, ny, fluid: f, img } = this;
    const data = img.data;
    const Uin = f.Uin || 1;
    for (let j = 0; j < ny; j++)
      for (let i = 0; i < nx; i++) {
        const k = (i + 1) + f.W * (j + 1);
        const sp = Math.hypot(f.u[k], f.v[k]) / Uin;     // normalized speed
        const dye = Math.min(f.dye[k], 1);
        let r, g, b;
        if (f.solid[k]) { r = 70; g = 78; b = 92; }       // the car body
        else {
          // speed colormap: slow wake = near-black -> freestream blue -> fast cyan
          const t = Math.max(0, Math.min(sp / 1.7, 1));
          r = 4 + 36 * t;
          g = 10 + 165 * t;
          b = 30 + 165 * t;
          // vorticity tint: shear layers / shed vortices glow warm(+)/cool(-)
          const w = (i >= 1 && j >= 1) ? f.vort(i + 1, j + 1) : 0;
          const wt = Math.max(-1, Math.min(1, w * 3.0));
          if (wt > 0) { r += 120 * wt; g += 20 * wt; }
          else { b += 120 * -wt; g += 30 * -wt; }
          // dye streaklines glow white over the top
          r += 210 * dye; g += 210 * dye; b += 210 * dye;
        }
        const o = 4 * (i + nx * j);
        data[o] = Math.min(r, 255); data[o + 1] = Math.min(g, 255);
        data[o + 2] = Math.min(b, 255); data[o + 3] = 255;
      }
    this.bctx.putImageData(img, 0, 0);
    const ctx = this.ctx, W = this.canvas.width, H = this.canvas.height;
    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(this.buf, 0, 0, W, H);

    // --- crisp overlays so the picture reads clearly -----------------------
    const sxW = W / nx, syH = H / ny;
    const ccx = this.cx * sxW, ccy = this.cy * syH;
    const Lpx = this.Lx * sxW, Wpx = this.Ly * syH;

    // inflow ("wind") arrow on the left
    ctx.strokeStyle = "#bfe6ff"; ctx.fillStyle = "#bfe6ff"; ctx.lineWidth = 2;
    const ay = H * 0.5;
    ctx.beginPath(); ctx.moveTo(8, ay); ctx.lineTo(40, ay); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(40, ay); ctx.lineTo(33, ay - 4); ctx.lineTo(33, ay + 4);
    ctx.closePath(); ctx.fill();
    ctx.font = "10px monospace"; ctx.textBaseline = "alphabetic";
    ctx.fillText("wind", 12, ay - 6);

    // crisp car silhouette (matches the immersed mask), nose into the wind
    ctx.save();
    ctx.translate(ccx, ccy); ctx.rotate(this.lastBeta);
    ctx.fillStyle = "#2b3340"; ctx.strokeStyle = "#cfe0f0"; ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(Lpx, 0);
    ctx.quadraticCurveTo(Lpx * 0.7, -Wpx, 0, -Wpx);
    ctx.quadraticCurveTo(-Lpx, -Wpx, -Lpx, 0);
    ctx.quadraticCurveTo(-Lpx, Wpx, 0, Wpx);
    ctx.quadraticCurveTo(Lpx * 0.7, Wpx, Lpx, 0);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();

    // wake label downstream (flow goes left->right)
    ctx.fillStyle = "rgba(207,224,240,0.7)"; ctx.font = "10px monospace";
    ctx.fillText("← wake →", ccx + Lpx + 8, ccy - 2);

    // color legend (slow -> fast)
    const lgW = 70, lgX = W - lgW - 8, lgY = 24;
    const lg = ctx.createLinearGradient(lgX, 0, lgX + lgW, 0);
    lg.addColorStop(0, "rgb(4,10,30)"); lg.addColorStop(1, "rgb(40,175,195)");
    ctx.fillStyle = lg; ctx.fillRect(lgX, lgY, lgW, 6);
    ctx.fillStyle = "#9fb0c0"; ctx.font = "9px monospace";
    ctx.fillText("slow", lgX, lgY - 2); ctx.fillText("fast", lgX + lgW - 22, lgY - 2);

    // title + readouts
    ctx.font = "11px monospace"; ctx.textBaseline = "top";
    ctx.fillStyle = "rgba(0,0,0,0.5)"; ctx.fillRect(0, 0, W, 18);
    ctx.fillStyle = "#cdd9e5";
    ctx.fillText("AERO — live 2D flow (qualitative, model-Re)", 6, 3);
    ctx.fillStyle = "rgba(0,0,0,0.5)"; ctx.fillRect(0, H - 34, W, 34);
    ctx.fillStyle = "#9fb0c0";
    ctx.fillText(`drag ${dragN.toFixed(0)} N    downforce ${downN.toFixed(0)} N`,
                 6, H - 30);
    ctx.fillStyle = "#7ee0a0";
    ctx.fillText(`grip from aero: +${(gripGain * 100).toFixed(0)}%  ` +
                 `· forces = ½ρCAV² (coeff model)`, 6, H - 16);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { AeroField };
}
