"""
Planar vehicle models: a dynamic bicycle and a four-wheel double-track car,
both with Pacejka Magic-Formula tyres.

Model -- dynamic bicycle, body-fixed frame:

    state q = [X, Y, psi, vx, vy, r]
        X, Y  global position of the centre of gravity (CoG)   [m]
        psi   yaw (heading) angle                               [rad]
        vx    longitudinal body velocity (along the car)        [m/s]
        vy    lateral  body velocity (sideways)                 [m/s]
        r     yaw rate (psi_dot)                                 [rad/s]

    controls u = [delta, thr]
        delta  front steer angle                                [rad]
        thr    drive command in [-1, 1]  (+throttle / -brake)

Tyre slip angles (small-slip definition, valid for vx not too near 0):

    alpha_f = delta - atan2(vy + a*r, vx)        front
    alpha_r =       - atan2(vy - b*r, vx)        rear

Lateral tyre force -- Pacejka Magic Formula:

    Fy = D * sin( C * atan( B*alpha - E*(B*alpha - atan(B*alpha)) ) ),  D = mu*Fz

Longitudinal force from the drive command, friction-ellipse coupled so a tyre
cannot give full grip sideways AND lengthways at once:

    Fx_avail = sqrt( max(0, (mu*Fz)^2 - Fy^2) )   # grip left after cornering
    Fx       = thr * Fx_avail

Newton-Euler in the body frame (the (vy*r), (vx*r) terms are the centripetal
coupling from working in a rotating frame):

    m (vx_dot - vy*r) = Fx_r + Fx_f*cos(delta) - Fy_f*sin(delta) - F_drag
    m (vy_dot + vx*r) =        Fy_f*cos(delta) + Fy_r            + Fx_f*sin(delta)
    Iz r_dot          = a*(Fy_f*cos(delta) + Fx_f*sin(delta)) - b*Fy_r

Aero drag: F_drag = 0.5 * rho * Cd * A * vx^2.

numpy only. Integrator: classical RK4. SI units throughout.
"""

import numpy as np

G = 9.81


class PacejkaTyre:
    """Lateral Magic-Formula tyre. Parameters are the standard B,C,D,E set with
    D = mu_eff*Fz.

    Optional LOAD SENSITIVITY: real tyres lose grip per unit load as load rises,
    so the peak friction coefficient drops with Fz:
        mu_eff(Fz) = mu * (1 - mu_load*(Fz/Fz0 - 1))
    With mu_load=0 (default) this is the plain D=mu*Fz tyre, so the bicycle
    model is unchanged. mu_load>0 makes the force a CONCAVE function of
    load, which is precisely why lateral load transfer reduces an axle's total
    grip and gives a car its understeer/oversteer balance (used by DoubleTrack)."""

    def __init__(self, B=10.0, C=1.9, E=0.97, mu=1.3, mu_load=0.0, Fz0=3188.0):
        self.B = B          # stiffness factor (sets cornering stiffness)
        self.C = C          # shape factor (~1.3 lateral, ~1.65 longitudinal)
        self.E = E          # curvature factor
        self.mu = mu        # peak friction coefficient at the nominal load Fz0
        self.mu_load = mu_load   # load-sensitivity slope (0 = none)
        self.Fz0 = Fz0      # nominal per-wheel load [N]

    def mu_eff(self, Fz):
        """Load-dependent peak friction coefficient."""
        return self.mu * (1.0 - self.mu_load * (Fz / self.Fz0 - 1.0))

    def fy(self, alpha, Fz):
        """Lateral force [N] for slip angle alpha [rad] and vertical load Fz [N].
        Sign convention: positive alpha -> negative Fy (restoring), so we return
        the Magic-Formula value with the leading sign baked in."""
        B, C, E = self.B, self.C, self.E
        D = self.mu_eff(Fz) * Fz
        # standard Magic Formula; force opposes slip so a positive slip angle
        # produces a negative lateral force.
        phi = B * alpha - E * (B * alpha - np.arctan(B * alpha))
        return D * np.sin(C * np.arctan(phi))

    def cornering_stiffness(self, Fz):
        """Linear cornering stiffness C_alpha = dFy/dalpha at alpha=0 [N/rad].
        For the Magic Formula this is B*C*D = B*C*mu_eff*Fz."""
        return self.B * self.C * self.mu_eff(Fz) * Fz


class DynamicBicycle:
    """Planar dynamic bicycle model with Pacejka tyres and friction-ellipse
    combined slip. Rear-wheel drive (drive command acts on the rear axle)."""

    def __init__(self, m=1300.0, Iz=1700.0, a=1.35, b=1.35,
                 h=0.50, track=1.6, mu=1.3,
                 Cd=0.9, area=1.8, rho=1.225, g=G,
                 tyre=None):
        self.m = m              # mass [kg]
        self.Iz = Iz            # yaw inertia [kg m^2]
        self.a = a              # CoG -> front axle [m]
        self.b = b              # CoG -> rear  axle [m]
        self.L = a + b          # wheelbase [m]
        self.h = h              # CoG height [m] (for load transfer, later)
        self.track = track      # track width [m]
        self.mu = mu
        self.Cd = Cd            # drag coefficient
        self.area = area        # frontal area [m^2]
        self.rho = rho          # air density [kg/m^3]
        self.g = g
        self.tyre = tyre or PacejkaTyre(mu=mu)
        # static axle loads (no transfer in the bicycle model): share by lever
        self.Fz_f = m * g * b / self.L
        self.Fz_r = m * g * a / self.L

    # --- forces ------------------------------------------------------------
    def tyre_forces(self, vx, vy, r, delta, thr):
        """Return (Fyf, Fyr, Fxr) tyre forces in their own directions [N]."""
        vx_safe = max(vx, 0.5)                  # model breaks down near standstill
        # slip angles
        alpha_f = delta - np.arctan2(vy + self.a * r, vx_safe)
        alpha_r = -np.arctan2(vy - self.b * r, vx_safe)
        # lateral (Pacejka)
        Fyf = self.tyre.fy(alpha_f, self.Fz_f)
        Fyr = self.tyre.fy(alpha_r, self.Fz_r)
        # longitudinal on the driven (rear) axle, friction-ellipse limited
        Fx_cap = self.mu * self.Fz_r
        Fx_avail = np.sqrt(max(Fx_cap**2 - Fyr**2, 0.0))
        Fxr = thr * Fx_avail
        return Fyf, Fyr, Fxr, alpha_f, alpha_r

    def deriv(self, q, u):
        """q' = f(q,u). q=[X,Y,psi,vx,vy,r], u=[delta, thr]."""
        X, Y, psi, vx, vy, r = q
        delta, thr = u
        Fyf, Fyr, Fxr, _, _ = self.tyre_forces(vx, vy, r, delta, thr)
        F_drag = 0.5 * self.rho * self.Cd * self.area * vx * abs(vx)

        cd, sd = np.cos(delta), np.sin(delta)
        # body-frame Newton-Euler (front axle has no drive force: Fxf = 0)
        vx_dot = (Fxr - Fyf * sd - F_drag) / self.m + vy * r
        vy_dot = (Fyf * cd + Fyr) / self.m - vx * r
        r_dot = (self.a * Fyf * cd - self.b * Fyr) / self.Iz

        # kinematics: rotate body velocity into the global frame
        cpsi, spsi = np.cos(psi), np.sin(psi)
        X_dot = vx * cpsi - vy * spsi
        Y_dot = vx * spsi + vy * cpsi
        return np.array([X_dot, Y_dot, r, vx_dot, vy_dot, r_dot])

    # --- integrator --------------------------------------------------------
    def rk4_step(self, q, u, dt):
        k1 = self.deriv(q, u)
        k2 = self.deriv(q + 0.5 * dt * k1, u)
        k3 = self.deriv(q + 0.5 * dt * k2, u)
        k4 = self.deriv(q + dt * k3, u)
        return q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def simulate(self, q0, control_fn, dt, n_steps):
        q = np.array(q0, float)
        traj = np.empty((n_steps + 1, 6))
        traj[0] = q
        for k in range(n_steps):
            u = np.asarray(control_fn(k * dt, q), float)
            q = self.rk4_step(q, u, dt)
            traj[k + 1] = q
        return traj

    # --- analytical references (for verification) --------------------------
    def understeer_gradient(self):
        """Understeer gradient K_us [rad per (m/s^2)] from linear tyre stiffness:
            K_us = m/L * (b/(L*C_af) - a/(L*C_ar))   ... = Wf/Caf - Wr/Car form.
        Positive -> understeer. Used to predict steady-state cornering."""
        Caf = self.tyre.cornering_stiffness(self.Fz_f)
        Car = self.tyre.cornering_stiffness(self.Fz_r)
        Wf = self.m * self.g * self.b / self.L     # front weight
        Wr = self.m * self.g * self.a / self.L     # rear  weight
        return Wf / Caf - Wr / Car

    def steady_state_steer(self, R, U):
        """Ackermann + understeer steady-state steer angle for radius R at speed
        U:   delta = L/R + K_us * (U^2 / R).  (ay = U^2/R)"""
        return self.L / R + self.understeer_gradient() * (U**2 / R)


# ============================================================================
class DoubleTrack:
    """Four-wheel (double-track) planar car with per-wheel vertical loads,
    longitudinal + lateral LOAD TRANSFER, per-wheel Pacejka tyres and a
    friction-ellipse drive.

    This is the step that makes the car *behave* like a car: brake and the nose
    dives (front loads up); corner hard and the outer tyres carry the load. With
    tyre load-sensitivity, transferring load across an axle reduces that axle's
    grip -- so the FRONT/REAR split of the lateral load transfer (`tllt_f`) sets
    the understeer/oversteer balance, exactly as in a real car. Wheels are
    indexed [FL, FR, RL, RR]; rear-wheel drive.

    The vertical loads depend on the body accelerations, which depend on the tyre
    forces, which depend on the loads -- an algebraic loop. We close it with a
    short fixed-point iteration inside `deriv` (loads<->accel), so `deriv` stays
    a pure, deterministic function of (q, u). State and integrator are identical
    to the bicycle: q = [X, Y, psi, vx, vy, r], classical RK4.
    """

    def __init__(self, m=1300.0, Iz=1700.0, a=1.35, b=1.35,
                 h=0.50, track=1.6, mu=1.3, mu_load=0.10, tllt_f=0.50,
                 Cd=0.9, Cl=1.0, aero_bal=0.45, area=1.8, rho=1.225, g=G,
                 tyre=None):
        self.m = m
        self.Iz = Iz
        self.a = a
        self.b = b
        self.L = a + b
        self.h = h                 # CoG height -> load-transfer lever
        self.track = track         # track width (left-right wheel spacing)
        self.mu = mu
        self.tllt_f = tllt_f       # front share of lateral load transfer (0..1):
                                   #  >0.5 -> understeer, <0.5 -> oversteer
        self.Cd = Cd               # drag coefficient
        self.Cl = Cl               # DOWNforce coefficient (Fz = 1/2 rho Cl A V^2)
        self.aero_bal = aero_bal   # front share of aero downforce
        self.area = area
        self.rho = rho
        self.g = g
        self.Fz0 = m * g / 4.0     # nominal per-wheel load
        self.tyre = tyre or PacejkaTyre(mu=mu, mu_load=mu_load, Fz0=self.Fz0)

    def downforce(self, vx, vy):
        """Aerodynamic downforce [N], speed-squared. Grows the tyre loads -> more
        grip at speed (the real aero->vehicle coupling; the fluid panel is the
        illustrative picture of the same physics)."""
        return 0.5 * self.rho * self.Cl * self.area * (vx * vx + vy * vy)

    # --- per-wheel vertical loads (with load transfer + downforce) ---------
    def wheel_loads(self, ax, ay, aero_z=0.0):
        """Vertical load [N] on [FL, FR, RL, RR] for body accelerations
        ax (longitudinal, +forward), ay (lateral, +left) and total aero
        downforce aero_z. Transfers sum to zero, so the four loads add up to
        m*g + aero_z (checked in verification)."""
        Wf = self.m * self.g * self.b / self.L      # static front axle
        Wr = self.m * self.g * self.a / self.L      # static rear  axle
        # aerodynamic downforce, split front/rear by the aero balance
        Wf += aero_z * self.aero_bal
        Wr += aero_z * (1.0 - self.aero_bal)
        # longitudinal transfer: accelerate -> load moves to the rear
        dFx = self.m * ax * self.h / self.L
        front_axle = Wf - dFx
        rear_axle = Wr + dFx
        # lateral transfer: split front/rear by the distribution tllt_f
        dFy_tot = self.m * ay * self.h / self.track
        dFy_f = self.tllt_f * dFy_tot
        dFy_r = (1.0 - self.tllt_f) * dFy_tot
        # ay>0 (cornering left) shifts load onto the RIGHT wheels
        Fz = np.array([
            front_axle / 2 - dFy_f,   # FL
            front_axle / 2 + dFy_f,   # FR
            rear_axle / 2 - dFy_r,    # RL
            rear_axle / 2 + dFy_r,    # RR
        ])
        return np.maximum(Fz, 0.0)    # a lifted wheel carries no load

    # --- per-wheel forces --------------------------------------------------
    def wheel_forces(self, vx, vy, r, delta, thr, Fz, hb=0.0):
        """Return (Fy[4], Fx[4], alpha[4]) in each wheel's own frame.
        Lateral from Pacejka; rear wheels also take a friction-ellipse-limited
        drive force from thr. Front wheels are undriven (Fx=0).

        hb = handbrake [0,1]: locks the rear axle -- a kinetic brake opposing
        motion and a collapse of rear lateral grip (the car oversteers/drifts).
        hb=0 (default) leaves the base dynamics unchanged."""
        ht = self.track / 2.0
        # forward speed differs left/right under yaw
        vxl = max(vx - ht * r, 0.5)
        vxr = max(vx + ht * r, 0.5)
        a, b = self.a, self.b
        alpha = np.array([
            delta - np.arctan2(vy + a * r, vxl),   # FL
            delta - np.arctan2(vy + a * r, vxr),   # FR
            -np.arctan2(vy - b * r, vxl),          # RL
            -np.arctan2(vy - b * r, vxr),          # RR
        ])
        Fy = np.array([self.tyre.fy(alpha[i], Fz[i]) for i in range(4)])
        Fx = np.zeros(4)
        # rear-wheel drive: each rear wheel gets thr * its own ellipse headroom
        for i in (2, 3):
            cap = self.tyre.mu_eff(Fz[i]) * Fz[i]
            avail = np.sqrt(max(cap**2 - Fy[i]**2, 0.0))
            Fx[i] = thr * avail
            if hb > 0:                             # locked rear wheel
                Fx[i] = -np.sign(vx) * hb * cap
                Fy[i] *= 1.0 - 0.85 * hb
        return Fy, Fx, alpha

    # --- dynamics ----------------------------------------------------------
    def deriv(self, q, u):
        X, Y, psi, vx, vy, r = q
        delta, thr = u[0], u[1]
        hb = u[2] if len(u) > 2 else 0.0
        cd, sd = np.cos(delta), np.sin(delta)
        Fdrag = 0.5 * self.rho * self.Cd * self.area * vx * abs(vx)
        aero_z = self.downforce(vx, vy)

        # fixed-point: resolve load-transfer <-> acceleration coupling
        ax, ay = 0.0, 0.0
        for _ in range(4):
            Fz = self.wheel_loads(ax, ay, aero_z)
            Fy, Fx, _ = self.wheel_forces(vx, vy, r, delta, thr, Fz, hb)
            Fyf = Fy[0] + Fy[1]          # front lateral (tyre frame)
            Fyr = Fy[2] + Fy[3]          # rear  lateral
            Fxr = Fx[2] + Fx[3]          # rear  drive
            # body specific forces (what causes load transfer)
            ax = (Fxr - Fyf * sd - Fdrag) / self.m
            ay = (Fyf * cd + Fyr) / self.m

        # yaw moment: front/rear lateral arms + rear differential drive arm
        ht = self.track / 2.0
        Mz = (self.a * Fyf * cd - self.b * Fyr) + ht * (Fx[3] - Fx[2])

        vx_dot = ax + vy * r
        vy_dot = ay - vx * r
        r_dot = Mz / self.Iz

        cpsi, spsi = np.cos(psi), np.sin(psi)
        X_dot = vx * cpsi - vy * spsi
        Y_dot = vx * spsi + vy * cpsi
        return np.array([X_dot, Y_dot, r, vx_dot, vy_dot, r_dot])

    # --- integrator (identical to the bicycle) -----------------------------
    def rk4_step(self, q, u, dt):
        k1 = self.deriv(q, u)
        k2 = self.deriv(q + 0.5 * dt * k1, u)
        k3 = self.deriv(q + 0.5 * dt * k2, u)
        k4 = self.deriv(q + dt * k3, u)
        return q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def simulate(self, q0, control_fn, dt, n_steps):
        q = np.array(q0, float)
        traj = np.empty((n_steps + 1, 6))
        traj[0] = q
        for k in range(n_steps):
            u = np.asarray(control_fn(k * dt, q), float)
            q = self.rk4_step(q, u, dt)
            traj[k + 1] = q
        return traj
