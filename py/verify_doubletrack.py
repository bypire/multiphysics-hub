"""
Verification of the DOUBLE-TRACK (4-wheel) car vs physical references.
================================================================================
Ground truth shipped with the code. Checks the things the double-track model
adds on top of the verified bicycle: per-wheel load transfer and the
understeer/oversteer balance it produces.

  1. Static loads        -> at rest, each wheel carries m*g/4.
  2. Vertical equilibrium-> sum of the 4 wheel loads == m*g for ANY (ax, ay).
  3. Longitudinal transfer-> braking at ax loads the front by m*|ax|*h/L.
  4. Lateral transfer     -> across an axle, dFz == 2*share*m*ay*h/track.
  5. Low-speed cornering  -> transfer negligible -> steady steer ~ Ackermann L/R.
  6. Balance TREND        -> at fixed steer+speed, more FRONT load-transfer share
                             (tllt_f) -> wider radius (understeer); less -> tighter
                             (oversteer). The defining double-track behaviour.

ASCII-only; run with: python -u
"""

import numpy as np
from car_dynamic import DoubleTrack


def check(name, got, want, tol, unit=""):
    ok = abs(got - want) <= tol
    print(f"  [{'OK ' if ok else 'XX '}] {name:40s} got {got:12.4f} {unit:5s} "
          f"want {want:12.4f}  (tol {tol:g})")
    return ok


def settle_radius(car, delta, U, t_end=18.0, dt=0.002):
    """Hold steer `delta` and speed `U`; return the settled cornering radius."""
    def control(t, q):
        thr = np.clip(0.4 * (U - q[3]), -1, 1)
        return [delta, thr]
    n = int(t_end / dt)
    traj = car.simulate([0, 0, 0, U, 0, 0], control, dt, n)
    tail = traj[-int(2.0 / dt):]
    return tail[:, 3].mean() / tail[:, 5].mean()      # vx / r


def main():
    print("=" * 76)
    print("DOUBLE-TRACK (4-wheel) CAR -- verification vs physical references")
    print("=" * 76)
    ok = True
    car = DoubleTrack(tllt_f=0.50)

    # --- 1. static loads ---------------------------------------------------
    print("\n1. Static wheel loads (at rest -> m*g/4 each)")
    Fz = car.wheel_loads(0.0, 0.0)
    for i, w in enumerate(["FL", "FR", "RL", "RR"]):
        ok &= check(f"Fz {w}", Fz[i], car.m * car.g / 4, tol=1.0, unit="N")

    # --- 2. vertical equilibrium for arbitrary accel -----------------------
    print("\n2. Vertical equilibrium (sum Fz == m*g for any ax, ay)")
    for ax, ay in [(3.0, 0.0), (0.0, 7.0), (-6.0, 5.0), (4.0, -8.0)]:
        s = car.wheel_loads(ax, ay).sum()
        ok &= check(f"sum Fz at ax={ax:+.0f},ay={ay:+.0f}", s, car.m * car.g,
                    tol=1.0, unit="N")

    # --- 3. longitudinal transfer -----------------------------------------
    print("\n3. Longitudinal transfer (brake ax=-5 -> front gains m*|ax|*h/L)")
    ax = -5.0
    Fz0 = car.wheel_loads(0, 0)
    Fz1 = car.wheel_loads(ax, 0)
    front_gain = (Fz1[0] + Fz1[1]) - (Fz0[0] + Fz0[1])
    ok &= check("front axle load gain", front_gain,
                car.m * abs(ax) * car.h / car.L, tol=1.0, unit="N")

    # --- 4. lateral transfer ----------------------------------------------
    print("\n4. Lateral transfer (ay=8 -> dFz across front axle)")
    ay = 8.0
    Fz = car.wheel_loads(0, ay)
    dFz_front = Fz[1] - Fz[0]               # FR - FL
    want = 2 * car.tllt_f * car.m * ay * car.h / car.track
    ok &= check("front axle dFz (FR-FL)", dFz_front, want, tol=1.0, unit="N")

    # --- 5. low-speed Ackermann -------------------------------------------
    print("\n5. Low-speed cornering (transfer negligible -> steer ~ L/R)")
    U, delta = 6.0, np.radians(3.0)         # slow -> tiny ay
    R = settle_radius(car, delta, U)
    ok &= check("steer (deg) for settled R", np.degrees(car.L / R),
                np.degrees(delta), tol=0.25, unit="deg")

    # --- 6. balance trend -------------------------------------------------
    print("\n6. Balance trend (fixed steer+speed; front share tllt_f -> radius)")
    U, delta = 18.0, np.radians(4.0)
    radii = {}
    for tllt in (0.30, 0.50, 0.70):
        c = DoubleTrack(tllt_f=tllt)
        radii[tllt] = settle_radius(c, delta, U)
        print(f"      tllt_f={tllt:.2f}  ->  settled radius R = {radii[tllt]:7.2f} m"
              f"   ay~{U**2/radii[tllt]:.1f} m/s^2")
    understeer_ok = radii[0.30] < radii[0.50] < radii[0.70]
    print(f"  [{'OK ' if understeer_ok else 'XX '}] monotonic: more front share "
          f"-> wider radius (understeer)")
    ok &= understeer_ok

    # --- 7. downforce ------------------------------------------------------
    print("\n7. Downforce (sum Fz == m*g + Fdown; tyre grip rises with speed)")
    V = 60.0
    Fdown = car.downforce(V, 0.0)
    s = car.wheel_loads(0, 0, Fdown).sum()
    ok &= check(f"sum Fz at V={V:.0f} m/s", s, car.m * car.g + Fdown,
                tol=1.0, unit="N")
    print(f"      downforce at {V:.0f} m/s = {Fdown:.0f} N "
          f"= {100*Fdown/(car.m*car.g):.0f}% of weight "
          f"-> rear-wheel grip cap {car.mu*(car.Fz0+Fdown*(1-car.aero_bal)/2)/1000:.1f} kN "
          f"vs {car.mu*car.Fz0/1000:.1f} kN at rest")

    print("\n" + "=" * 76)
    print("ALL CHECKS PASSED" if ok else "*** SOME CHECKS FAILED ***")
    print("=" * 76)
    return ok


if __name__ == "__main__":
    main()
