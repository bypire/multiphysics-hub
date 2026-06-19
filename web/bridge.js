// bridge.js -- modal moving-load bridge response. A simply-supported
// Euler-Bernoulli beam reduced to its first few analytical modes and integrated
// as the car crosses. Mirrors py/bridge_modal.py (compared in web/xcheck.html).
// State = [q_1..q_n, qd_1..qd_n]; RK4 in time.

class BridgeModal {
  constructor({ L = 40.0, EI = 8.4e10, m_bar = 12000.0,
                zeta = 0.02, n_modes = 4 } = {}) {
    this.L = L; this.EI = EI; this.m_bar = m_bar; this.zeta = zeta;
    this.n = n_modes;
    this.kn = []; this.omega = [];
    for (let i = 1; i <= n_modes; i++) {
      const kn = i * Math.PI / L;
      this.kn.push(kn);
      this.omega.push(kn * kn * Math.sqrt(EI / m_bar));
    }
    this.Mn = m_bar * L / 2.0;
    this.state = new Float64Array(2 * n_modes);  // live modal state
  }

  frequenciesHz() { return this.omega.map((w) => w / (2 * Math.PI)); }

  // static deflection at x under point load P at position a
  staticDeflection(x, P, a) {
    let w = 0;
    for (let i = 0; i < this.n; i++) {
      const qs = P * Math.sin(this.kn[i] * a) / (this.Mn * this.omega[i] ** 2);
      w += qs * Math.sin(this.kn[i] * x);
    }
    return w;
  }
  staticMidspan(P) { return this.staticDeflection(this.L / 2, P, this.L / 2); }

  // state derivative; a = load position (or null when off the span)
  deriv(s, P, a) {
    const n = this.n, out = new Float64Array(2 * n);
    for (let i = 0; i < n; i++) {
      const q = s[i], qd = s[n + i];
      const Fn = (a === null || a < 0 || a > this.L)
        ? 0 : P * Math.sin(this.kn[i] * a);
      out[i] = qd;
      out[n + i] = Fn / this.Mn - 2 * this.zeta * this.omega[i] * qd
                   - this.omega[i] ** 2 * q;
    }
    return out;
  }

  rk4Step(s, dt, P, a, aMid) {
    const am = (aMid === undefined) ? a : aMid;
    const add = (x, k, h) => x.map((xi, i) => xi + h * k[i]);
    const k1 = this.deriv(s, P, a);
    const k2 = this.deriv(add(s, k1, 0.5 * dt), P, am);
    const k3 = this.deriv(add(s, k2, 0.5 * dt), P, am);
    const k4 = this.deriv(add(s, k3, dt), P, a);
    return s.map((si, i) => si + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }

  // advance the live state one step (mutates this.state)
  step(dt, P, a, aMid) { this.state = this.rk4Step(this.state, dt, P, a, aMid); }

  deflection(x, s = this.state) {
    let w = 0;
    for (let i = 0; i < this.n; i++) w += s[i] * Math.sin(this.kn[i] * x);
    return w;
  }
  bendingMoment(x, s = this.state) {
    let M = 0;
    for (let i = 0; i < this.n; i++)
      M += this.kn[i] ** 2 * s[i] * Math.sin(this.kn[i] * x);
    return -this.EI * M;
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { BridgeModal };
}
