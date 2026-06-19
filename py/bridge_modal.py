"""
Moving-load bridge response by modal superposition.

A simply-supported Euler-Bernoulli beam deflects dynamically as a load crosses
it. Rather than time-marching the full M u'' + C u' + K u = F(t) FE system, we
project onto the first few analytical modes of the beam and integrate the
decoupled modal ODEs; a handful of modes capture the response and it is cheap
enough to run every frame.

Simply-supported beam, span L, bending stiffness EI, mass/length m_bar:
    mode shape      phi_n(x)   = sin(n pi x / L)
    natural freq    omega_n    = (n pi / L)^2 sqrt(EI / m_bar)
    modal mass      M_n        = m_bar L / 2

Under a moving point load P(t) at position a(t), the modal force is
    F_n(t) = P(t) * phi_n(a(t))
and each mode obeys the damped SDOF oscillator
    q_n'' + 2 zeta omega_n q_n' + omega_n^2 q_n = F_n(t) / M_n.

Physical fields are reconstructed by summing modes:
    deflection   w(x,t)  = sum_n q_n(t) phi_n(x)
    bending moment M(x,t)= EI w''(x,t) = -EI sum_n (n pi/L)^2 q_n(t) sin(n pi x/L)

Checks in verify_bridge.py: the modal static deflection converges to P L^3/48 EI
at mid-span, the frequencies match (n pi/L)^2 sqrt(EI/m), and a slow crossing
gives DAF -> 1. numpy only; RK4 in time.
"""

import numpy as np

G = 9.81


class BridgeModal:
    def __init__(self, L=40.0, EI=8.4e10, m_bar=12000.0, zeta=0.02, n_modes=4):
        self.L = L
        self.EI = EI
        self.m_bar = m_bar
        self.zeta = zeta
        self.n = n_modes
        n = np.arange(1, n_modes + 1)
        self.kn = n * np.pi / L                          # wavenumbers
        self.omega = self.kn**2 * np.sqrt(EI / m_bar)    # modal frequencies
        self.Mn = m_bar * L / 2.0                        # modal mass (all modes)

    # --- analytical references --------------------------------------------
    def frequencies_hz(self):
        return self.omega / (2.0 * np.pi)

    def phi(self, x):
        """Mode-shape matrix phi_n(x): shape (len(x), n_modes)."""
        x = np.atleast_1d(x)
        return np.sin(np.outer(x, self.kn))

    def static_deflection(self, x, P, a):
        """Static w(x) under point load P at position a (modal sum)."""
        qs = P * np.sin(self.kn * a) / (self.Mn * self.omega**2)   # modal static
        return self.phi(x) @ qs

    def static_midspan(self, P):
        """Static mid-span deflection under a mid-span load (-> P L^3/48 EI)."""
        return float(self.static_deflection([self.L / 2], P, self.L / 2)[0])

    # --- modal state-space dynamics ---------------------------------------
    def deriv(self, state, P, a):
        """state = [q_1..q_n, qd_1..qd_n]. Returns its time derivative.
        P = load magnitude [N], a = load position along the span [m] (or None
        when the load is off the span)."""
        q = state[:self.n]
        qd = state[self.n:]
        if a is None or a < 0 or a > self.L:
            Fn = np.zeros(self.n)
        else:
            Fn = P * np.sin(self.kn * a)                 # modal force
        qdd = (Fn / self.Mn) - 2 * self.zeta * self.omega * qd - self.omega**2 * q
        return np.concatenate([qd, qdd])

    def rk4_step(self, state, dt, P, a, a_mid=None):
        """One RK4 step. a, a_mid optionally give the load position at the step
        start and midpoint (for a moving load); if a_mid is None, a is held."""
        am = a if a_mid is None else a_mid
        k1 = self.deriv(state, P, a)
        k2 = self.deriv(state + 0.5 * dt * k1, P, am)
        k3 = self.deriv(state + 0.5 * dt * k2, P, am)
        k4 = self.deriv(state + dt * k3, P, a)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def deflection(self, state, x):
        """Reconstruct w(x) from modal coordinates q (first n entries)."""
        return self.phi(x) @ state[:self.n]

    def bending_moment(self, state, x):
        """M(x) = EI w'' = -EI sum (kn^2) q_n sin(kn x)."""
        q = state[:self.n]
        x = np.atleast_1d(x)
        return -self.EI * (np.sin(np.outer(x, self.kn)) @ (self.kn**2 * q))

    # --- a full crossing (for verification / DAF) -------------------------
    def crossing(self, P, v, dt=1e-3, settle=0.0):
        """Drive load P across the span at speed v. Returns dict with the
        mid-span deflection history and the dynamic amplification factor."""
        T = self.L / v
        n_steps = int(np.ceil((T + settle) / dt))
        state = np.zeros(2 * self.n)
        xm = self.L / 2
        w_mid = np.empty(n_steps + 1)
        w_mid[0] = 0.0
        for k in range(n_steps):
            t = k * dt
            a0 = v * t
            am = v * (t + 0.5 * dt)
            a0 = a0 if a0 <= self.L else None
            am = am if (am is not None and am <= self.L) else None
            state = self.rk4_step(state, dt, P, a0, am)
            w_mid[k + 1] = self.deflection(state, [xm])[0]
        w_static = abs(self.static_midspan(P))
        daf = np.abs(w_mid).max() / w_static
        return dict(w_mid=w_mid, w_static=w_static, daf=daf, T=T)
