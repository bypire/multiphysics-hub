// fluid.js -- 2D incompressible fluid (Stam "Stable Fluids").
//
// Scope: this is a qualitative, 2D, low-Reynolds flow solved each frame -- it
// shows where the flow separates and how the wake forms, not a quantitative
// aerodynamic prediction. The drag and downforce the vehicle uses come from the
// coefficient model (1/2 rho C A V^2); this field is illustrative.
//
// Method (Jos Stam, "Real-Time Fluid Dynamics for Games", 2003): advect
// (semi-Lagrangian, unconditionally stable) -> project (div u = 0 via a
// Gauss-Seidel pressure Poisson solve). The car is an immersed boundary handled
// by volume penalisation (velocity relaxed to 0 inside the solid). Collocated
// grid with a 1-cell ghost border. The reference staggered-grid Navier-Stokes
// solver is the separate vortex-street-cfd project.

class StableFluid {
  constructor(nx, ny, { visc = 0.0, dt = 0.016 } = {}) {
    this.nx = nx; this.ny = ny;
    this.W = nx + 2; this.H = ny + 2;          // include ghost border
    this.dt = dt; this.visc = visc;
    const N = this.W * this.H;
    this.u = new Float32Array(N);
    this.v = new Float32Array(N);
    this.u0 = new Float32Array(N);
    this.v0 = new Float32Array(N);
    this.dye = new Float32Array(N);
    this.dye0 = new Float32Array(N);
    this.solid = new Uint8Array(N);            // 1 = inside the car
    this.p = new Float32Array(N);
    this.div = new Float32Array(N);
    this.Uin = 1.0; this.Vin = 0.0;            // freestream (set each frame)
  }

  IX(i, j) { return i + this.W * j; }

  // --- outer boundaries: inflow left, outflow right, freestream top/bottom ---
  applyBoundaries() {
    const { nx, ny, u, v, dye, Uin, Vin } = this;
    const IX = (i, j) => i + this.W * j;
    for (let j = 0; j <= ny + 1; j++) {        // left inflow / right outflow
      u[IX(0, j)] = Uin; v[IX(0, j)] = Vin; dye[IX(0, j)] = dye[IX(0, j)];
      u[IX(nx + 1, j)] = u[IX(nx, j)];          // zero-gradient outflow
      v[IX(nx + 1, j)] = v[IX(nx, j)];
      dye[IX(nx + 1, j)] = dye[IX(nx, j)];
    }
    for (let i = 0; i <= nx + 1; i++) {        // top/bottom freestream (slip)
      u[IX(i, 0)] = Uin; v[IX(i, 0)] = 0;
      u[IX(i, ny + 1)] = Uin; v[IX(i, ny + 1)] = 0;
    }
  }

  // --- volume penalization: drag solid-cell velocity hard toward zero --------
  penalize() {
    const { u, v, dye, solid } = this;
    for (let k = 0; k < u.length; k++) {
      if (solid[k]) { u[k] = 0; v[k] = 0; dye[k] = 0; }
    }
  }

  // --- semi-Lagrangian advection of field d (d0 = previous) ------------------
  advect(d, d0, velU, velV) {
    const { nx, ny, dt } = this;
    const IX = (i, j) => i + this.W * j;
    const dt0x = dt * nx, dt0y = dt * ny;       // grid is [0,1]x[0,1] normalized
    for (let i = 1; i <= nx; i++) {
      for (let j = 1; j <= ny; j++) {
        const k = IX(i, j);
        let x = i - dt0x * velU[k];
        let y = j - dt0y * velV[k];
        if (x < 0.5) x = 0.5; if (x > nx + 0.5) x = nx + 0.5;
        if (y < 0.5) y = 0.5; if (y > ny + 0.5) y = ny + 0.5;
        const i0 = x | 0, i1 = i0 + 1, j0 = y | 0, j1 = j0 + 1;
        const s1 = x - i0, s0 = 1 - s1, t1 = y - j0, t0 = 1 - t1;
        d[k] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) +
               s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);
      }
    }
  }

  // --- projection: subtract grad(p) so the velocity is divergence-free -------
  project(iters = 22) {
    const { nx, ny, u, v, p, div, solid } = this;
    const IX = (i, j) => i + this.W * j;
    const h = 1.0 / Math.max(nx, ny);
    for (let i = 1; i <= nx; i++)
      for (let j = 1; j <= ny; j++) {
        const k = IX(i, j);
        div[k] = -0.5 * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] +
                             v[IX(i, j + 1)] - v[IX(i, j - 1)]);
        p[k] = 0;
      }
    // Gauss-Seidel; solid cells act as Neumann walls (skip them as neighbours)
    for (let it = 0; it < iters; it++) {
      for (let i = 1; i <= nx; i++)
        for (let j = 1; j <= ny; j++) {
          const k = IX(i, j);
          if (solid[k]) { p[k] = 0; continue; }
          let sum = 0, n = 0;
          const nb = [IX(i - 1, j), IX(i + 1, j), IX(i, j - 1), IX(i, j + 1)];
          for (const m of nb) { if (!solid[m]) { sum += p[m]; n++; } }
          p[k] = n ? (div[k] + sum) / n : 0;
        }
    }
    for (let i = 1; i <= nx; i++)
      for (let j = 1; j <= ny; j++) {
        const k = IX(i, j);
        if (solid[k]) continue;
        u[k] -= 0.5 * (p[IX(i + 1, j)] - p[IX(i - 1, j)]) / h;
        v[k] -= 0.5 * (p[IX(i, j + 1)] - p[IX(i, j - 1)]) / h;
      }
  }

  // --- one full step ---------------------------------------------------------
  step() {
    const { u, v, u0, v0, dye, dye0 } = this;
    this.applyBoundaries();
    this.penalize();
    // velocity self-advection
    u0.set(u); v0.set(v);
    this.advect(u, u0, u0, v0);
    this.advect(v, v0, u0, v0);
    this.penalize();
    this.applyBoundaries();
    this.project();
    // dye (passive tracer for streaklines)
    this.injectDye();
    dye0.set(dye);
    this.advect(dye, dye0, u, v);
    for (let k = 0; k < dye.length; k++) dye[k] *= 0.997;  // slow fade
    this.penalize();
  }

  // inject dye stripes at the inflow so the wake is legible
  injectDye() {
    const { ny, dye } = this;
    const IX = (i, j) => i + this.W * j;
    for (let j = 1; j <= ny; j++) {
      if ((j % 7) < 2) { dye[IX(1, j)] = 1.0; dye[IX(2, j)] = 1.0; dye[IX(3, j)] = 1.0; }
    }
  }

  // max |divergence| over fluid cells -- for the projection self-check
  maxDivergence() {
    const { nx, ny, u, v, solid } = this;
    const IX = (i, j) => i + this.W * j;
    const h = 1.0 / Math.max(nx, ny);
    let m = 0;
    for (let i = 1; i <= nx; i++)
      for (let j = 1; j <= ny; j++) {
        const k = IX(i, j);
        if (solid[k]) continue;
        const d = 0.5 * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] +
                             v[IX(i, j + 1)] - v[IX(i, j - 1)]);
        if (Math.abs(d) > m) m = Math.abs(d);
      }
    return m;
  }

  // mean |divergence| over fluid cells not touching the immersed body (the
  // penalized boundary is a velocity discontinuity, so it is excluded).
  meanDivergence() {
    const { nx, ny, u, v, solid } = this;
    const IX = (i, j) => i + this.W * j;
    const h = 1.0 / Math.max(nx, ny);
    let sum = 0, n = 0;
    for (let i = 2; i <= nx - 1; i++)
      for (let j = 2; j <= ny - 1; j++) {
        const k = IX(i, j);
        if (solid[k] || solid[IX(i - 1, j)] || solid[IX(i + 1, j)] ||
            solid[IX(i, j - 1)] || solid[IX(i, j + 1)]) continue;
        sum += Math.abs(0.5 * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] +
                                   v[IX(i, j + 1)] - v[IX(i, j - 1)]));
        n++;
      }
    return n ? sum / n : 0;
  }

  // vorticity at a cell (for colouring the wake)
  vort(i, j) {
    const IX = (a, b) => a + this.W * b;
    return (this.v[IX(i + 1, j)] - this.v[IX(i - 1, j)]) -
           (this.u[IX(i, j + 1)] - this.u[IX(i, j - 1)]);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { StableFluid };
}
