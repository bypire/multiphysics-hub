"""
Checks for the dynamic bicycle car against analytical references.

Checks:
  1. Pacejka peak lateral force  ->  |Fy|_max == mu*Fz.
  2. Cornering stiffness         ->  numeric dFy/dalpha|0 == B*C*mu*Fz.
  3. Straight-line top speed     ->  drag balances drive: settles, vy=r=0.
  4. Steady-state cornering      ->  with delta = L/R + K_us*U^2/R the car
                                     settles onto radius R (yaw rate r -> U/R).

ASCII-only prints (Windows cp1252 console). Run with: python -u
"""

import numpy as np
from car_dynamic import DynamicBicycle, PacejkaTyre


def check(name, got, want, tol, unit=""):
    ok = abs(got - want) <= tol
    flag = "OK " if ok else "XX "
    print(f"  [{flag}] {name:38s} got {got:12.5f} {unit:6s} "
          f"want {want:12.5f}  (tol {tol:g})")
    return ok


def main():
    print("=" * 74)
    print("DYNAMIC BICYCLE CAR -- verification vs analytical references")
    print("=" * 74)
    all_ok = True

    # --- 1. Pacejka peak ---------------------------------------------------
    print("\n1. Pacejka lateral force peak  (|Fy|max == mu*Fz)")
    tyre = PacejkaTyre(mu=1.3)
    Fz = 4000.0
    alphas = np.linspace(0, np.radians(40), 4000)
    Fy = np.array([tyre.fy(a, Fz) for a in alphas])
    all_ok &= check("peak |Fy|", Fy.max(), tyre.mu * Fz, tol=1.0, unit="N")

    # --- 2. Cornering stiffness -------------------------------------------
    print("\n2. Cornering stiffness  (dFy/dalpha|0 == B*C*mu*Fz)")
    da = 1e-6
    num = (tyre.fy(da, Fz) - tyre.fy(-da, Fz)) / (2 * da)
    all_ok &= check("C_alpha", num, tyre.cornering_stiffness(Fz),
                    tol=1.0, unit="N/rad")

    # --- 3. Straight-line top speed ---------------------------------------
    print("\n3. Straight-line  (full throttle, zero steer -> settles, vy=r=0)")
    car = DynamicBicycle()
    q0 = [0, 0, 0, 10.0, 0, 0]
    # terminal-velocity time constant ~7 s; integrate ~80 s to converge.
    traj = car.simulate(q0, lambda t, q: [0.0, 1.0], dt=0.005, n_steps=16000)
    vx_end = traj[-1, 3]
    # analytical top speed: thr=1 gives Fx = mu*Fz_r (no cornering), balance drag
    Fx = car.mu * car.Fz_r
    v_top = np.sqrt(Fx / (0.5 * car.rho * car.Cd * car.area))
    all_ok &= check("top speed vx", vx_end, v_top, tol=0.5, unit="m/s")
    all_ok &= check("lateral vy (should be 0)", traj[-1, 4], 0.0, tol=1e-6, unit="m/s")
    all_ok &= check("yaw rate r (should be 0)", traj[-1, 5], 0.0, tol=1e-6, unit="rad/s")

    # --- 4. Steady-state cornering ----------------------------------------
    print("\n4. Steady-state cornering  (delta from understeer gradient -> radius R)")
    R, U = 60.0, 18.0                      # 60 m radius at 18 m/s (~65 km/h)
    Kus = car.understeer_gradient()
    delta = car.steady_state_steer(R, U)
    balance = ("neutral" if abs(Kus) < 1e-9 else
               "understeer" if Kus > 0 else "oversteer")
    print(f"      understeer gradient K_us = {Kus:.5e} rad/(m/s^2)   ({balance})")
    print(f"      (K_us=0 is correct here: D=mu*Fz makes C_alpha scale with load,")
    print(f"       so weight distribution cancels -> neutral. Balance returns with")
    print(f"       load transfer + tyre load-sensitivity in the double-track model.)")
    print(f"      predicted steer delta    = {np.degrees(delta):.3f} deg")

    # hold speed U with a light throttle PI on vx, hold the steer angle, let the
    # yaw rate settle; then compare measured radius R_meas = U / r.
    def control(t, q):
        vx = q[3]
        thr = 0.3 * (U - vx)               # simple speed hold
        return [delta, np.clip(thr, -1, 1)]

    q0 = [0, 0, 0, U, 0, 0]
    traj = car.simulate(q0, control, dt=0.002, n_steps=12000)
    r_ss = traj[-2000:, 5].mean()          # settled yaw rate
    vx_ss = traj[-2000:, 3].mean()
    R_meas = vx_ss / r_ss
    all_ok &= check("settled radius R", R_meas, R, tol=2.0, unit="m")

    print("\n" + "=" * 74)
    print("ALL CHECKS PASSED" if all_ok else "*** SOME CHECKS FAILED ***")
    print("=" * 74)
    return all_ok


if __name__ == "__main__":
    main()
