// car.js -- planar vehicle models (dynamic bicycle and four-wheel double-track)
// with Pacejka tyres. Mirrors py/car_dynamic.py; the two are compared on
// identical inputs in web/xcheck.html. State q = [X, Y, psi, vx, vy, r];
// controls u = [delta, thr]. SI units throughout.

const G = 9.81;

class PacejkaTyre {
  // Lateral Magic-Formula tyre, D = mu_eff*Fz. Optional load sensitivity
  // (mu_load>0) makes peak grip drop with load -> source of the double-track
  // balance. mu_load=0 (default) reproduces the verified bicycle tyre exactly.
  constructor({ B = 10.0, C = 1.9, E = 0.97, mu = 1.3,
                mu_load = 0.0, Fz0 = 3188.0 } = {}) {
    this.B = B; this.C = C; this.E = E; this.mu = mu;
    this.mu_load = mu_load; this.Fz0 = Fz0;
  }
  muEff(Fz) { return this.mu * (1.0 - this.mu_load * (Fz / this.Fz0 - 1.0)); }
  fy(alpha, Fz) {
    const { B, C, E } = this;
    const D = this.muEff(Fz) * Fz;
    const phi = B * alpha - E * (B * alpha - Math.atan(B * alpha));
    return D * Math.sin(C * Math.atan(phi));
  }
  corneringStiffness(Fz) { return this.B * this.C * this.muEff(Fz) * Fz; }
}

class DynamicBicycle {
  constructor(opts = {}) {
    const p = Object.assign({
      m: 1300.0, Iz: 1700.0, a: 1.35, b: 1.35, h: 0.50, track: 1.6,
      mu: 1.3, Cd: 0.9, area: 1.8, rho: 1.225, g: G,
    }, opts);
    Object.assign(this, p);
    this.L = this.a + this.b;
    this.tyre = opts.tyre || new PacejkaTyre({ mu: this.mu });
    this.Fz_f = this.m * this.g * this.b / this.L;
    this.Fz_r = this.m * this.g * this.a / this.L;
  }

  // Return tyre forces + slip angles for diagnostics/HUD.
  tyreForces(vx, vy, r, delta, thr) {
    const vxSafe = Math.max(vx, 0.5);
    const alpha_f = delta - Math.atan2(vy + this.a * r, vxSafe);
    const alpha_r = -Math.atan2(vy - this.b * r, vxSafe);
    const Fyf = this.tyre.fy(alpha_f, this.Fz_f);
    const Fyr = this.tyre.fy(alpha_r, this.Fz_r);
    const FxCap = this.mu * this.Fz_r;
    const FxAvail = Math.sqrt(Math.max(FxCap * FxCap - Fyr * Fyr, 0.0));
    const Fxr = thr * FxAvail;
    return { Fyf, Fyr, Fxr, alpha_f, alpha_r };
  }

  deriv(q, u) {
    const [X, Y, psi, vx, vy, r] = q;
    const [delta, thr] = u;
    const { Fyf, Fyr, Fxr } = this.tyreForces(vx, vy, r, delta, thr);
    const Fdrag = 0.5 * this.rho * this.Cd * this.area * vx * Math.abs(vx);
    const cd = Math.cos(delta), sd = Math.sin(delta);

    const vx_dot = (Fxr - Fyf * sd - Fdrag) / this.m + vy * r;
    const vy_dot = (Fyf * cd + Fyr) / this.m - vx * r;
    const r_dot = (this.a * Fyf * cd - this.b * Fyr) / this.Iz;

    const cpsi = Math.cos(psi), spsi = Math.sin(psi);
    const X_dot = vx * cpsi - vy * spsi;
    const Y_dot = vx * spsi + vy * cpsi;
    return [X_dot, Y_dot, r, vx_dot, vy_dot, r_dot];
  }

  rk4Step(q, u, dt) {
    const add = (a, b, s) => a.map((ai, i) => ai + s * b[i]);
    const k1 = this.deriv(q, u);
    const k2 = this.deriv(add(q, k1, 0.5 * dt), u);
    const k3 = this.deriv(add(q, k2, 0.5 * dt), u);
    const k4 = this.deriv(add(q, k3, dt), u);
    return q.map((qi, i) =>
      qi + (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  understeerGradient() {
    const Caf = this.tyre.corneringStiffness(this.Fz_f);
    const Car = this.tyre.corneringStiffness(this.Fz_r);
    const Wf = this.m * this.g * this.b / this.L;
    const Wr = this.m * this.g * this.a / this.L;
    return Wf / Caf - Wr / Car;
  }
}

// ============================================================================
// DoubleTrack -- 4-wheel car with load transfer (JS port of py DoubleTrack).
// Wheels [FL, FR, RL, RR]; rear-wheel drive; tyre load-sensitivity gives the
// understeer/oversteer balance. deriv resolves the load<->accel loop with a
// short fixed-point iteration so it stays a pure function of (q, u).
// ============================================================================
class DoubleTrack {
  constructor(opts = {}) {
    const p = Object.assign({
      m: 1300.0, Iz: 1700.0, a: 1.35, b: 1.35, h: 0.50, track: 1.6,
      mu: 1.3, mu_load: 0.10, tllt_f: 0.50,
      Cd: 0.9, Cl: 1.0, aero_bal: 0.45, area: 1.8, rho: 1.225, g: G,
    }, opts);
    Object.assign(this, p);
    this.L = this.a + this.b;
    this.Fz0 = this.m * this.g / 4.0;
    this.tyre = opts.tyre ||
      new PacejkaTyre({ mu: this.mu, mu_load: this.mu_load, Fz0: this.Fz0 });
  }

  // aero downforce (speed-squared) -> grows tyre loads -> more grip at speed
  downforce(vx, vy) {
    return 0.5 * this.rho * this.Cl * this.area * (vx * vx + vy * vy);
  }

  wheelLoads(ax, ay, aero_z = 0) {
    let Wf = this.m * this.g * this.b / this.L;
    let Wr = this.m * this.g * this.a / this.L;
    Wf += aero_z * this.aero_bal;
    Wr += aero_z * (1 - this.aero_bal);
    const dFx = this.m * ax * this.h / this.L;
    const frontAxle = Wf - dFx, rearAxle = Wr + dFx;
    const dFyTot = this.m * ay * this.h / this.track;
    const dFy_f = this.tllt_f * dFyTot, dFy_r = (1 - this.tllt_f) * dFyTot;
    return [
      Math.max(frontAxle / 2 - dFy_f, 0),   // FL
      Math.max(frontAxle / 2 + dFy_f, 0),   // FR
      Math.max(rearAxle / 2 - dFy_r, 0),    // RL
      Math.max(rearAxle / 2 + dFy_r, 0),    // RR
    ];
  }

  // hb = handbrake [0,1]: locks the rear axle (kinetic brake + lateral grip
  // collapses -> the car oversteers/drifts). hb defaults to 0, so the verified
  // dynamics and the JS<->Python cross-check are unchanged when it is absent.
  wheelForces(vx, vy, r, delta, thr, Fz, hb = 0) {
    const ht = this.track / 2;
    const vxl = Math.max(vx - ht * r, 0.5), vxr = Math.max(vx + ht * r, 0.5);
    const a = this.a, b = this.b;
    const alpha = [
      delta - Math.atan2(vy + a * r, vxl),
      delta - Math.atan2(vy + a * r, vxr),
      -Math.atan2(vy - b * r, vxl),
      -Math.atan2(vy - b * r, vxr),
    ];
    const Fy = alpha.map((al, i) => this.tyre.fy(al, Fz[i]));
    const Fx = [0, 0, 0, 0];
    for (const i of [2, 3]) {
      const cap = this.tyre.muEff(Fz[i]) * Fz[i];
      const avail = Math.sqrt(Math.max(cap * cap - Fy[i] * Fy[i], 0));
      Fx[i] = thr * avail;
      if (hb > 0) {                              // locked rear wheel
        Fx[i] = -Math.sign(vx) * hb * cap;       // kinetic brake opposing motion
        Fy[i] *= 1 - 0.85 * hb;                  // lateral grip collapses
      }
    }
    return { Fy, Fx, alpha };
  }

  deriv(q, u) {
    const [X, Y, psi, vx, vy, r] = q;
    const delta = u[0], thr = u[1], hb = u[2] || 0;
    const cd = Math.cos(delta), sd = Math.sin(delta);
    const Fdrag = 0.5 * this.rho * this.Cd * this.area * vx * Math.abs(vx);
    const aero_z = this.downforce(vx, vy);

    let ax = 0, ay = 0, Fy, Fx, Fyf, Fyr, Fxr;
    for (let it = 0; it < 4; it++) {
      const Fz = this.wheelLoads(ax, ay, aero_z);
      ({ Fy, Fx } = this.wheelForces(vx, vy, r, delta, thr, Fz, hb));
      Fyf = Fy[0] + Fy[1]; Fyr = Fy[2] + Fy[3]; Fxr = Fx[2] + Fx[3];
      ax = (Fxr - Fyf * sd - Fdrag) / this.m;
      ay = (Fyf * cd + Fyr) / this.m;
    }
    const ht = this.track / 2;
    const Mz = (this.a * Fyf * cd - this.b * Fyr) + ht * (Fx[3] - Fx[2]);

    const vx_dot = ax + vy * r;
    const vy_dot = ay - vx * r;
    const r_dot = Mz / this.Iz;
    const cpsi = Math.cos(psi), spsi = Math.sin(psi);
    return [vx * cpsi - vy * spsi, vx * spsi + vy * cpsi, r,
            vx_dot, vy_dot, r_dot];
  }

  // diagnostics for the HUD: per-wheel loads + slip + body accelerations
  diagnostics(q, u) {
    const [, , , vx, vy, r] = q;
    const delta = u[0], thr = u[1], hb = u[2] || 0;
    const cd = Math.cos(delta), sd = Math.sin(delta);
    const Fdrag = 0.5 * this.rho * this.Cd * this.area * vx * Math.abs(vx);
    const aero_z = this.downforce(vx, vy);
    let ax = 0, ay = 0, Fz, Fy, Fx, alpha;
    for (let it = 0; it < 4; it++) {
      Fz = this.wheelLoads(ax, ay, aero_z);
      ({ Fy, Fx, alpha } = this.wheelForces(vx, vy, r, delta, thr, Fz, hb));
      const Fyf = Fy[0] + Fy[1], Fyr = Fy[2] + Fy[3], Fxr = Fx[2] + Fx[3];
      ax = (Fxr - Fyf * sd - Fdrag) / this.m;
      ay = (Fyf * cd + Fyr) / this.m;
    }
    return { Fz, Fy, Fx, alpha, ax, ay, aero_z };
  }

  rk4Step(q, u, dt) {
    const add = (a, b, s) => a.map((ai, i) => ai + s * b[i]);
    const k1 = this.deriv(q, u);
    const k2 = this.deriv(add(q, k1, 0.5 * dt), u);
    const k3 = this.deriv(add(q, k2, 0.5 * dt), u);
    const k4 = this.deriv(add(q, k3, dt), u);
    return q.map((qi, i) =>
      qi + (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }
}

// export for both browser (window) and node (cross-check harness)
if (typeof module !== "undefined" && module.exports) {
  module.exports = { DynamicBicycle, DoubleTrack, PacejkaTyre, G };
}
