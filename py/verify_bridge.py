"""
Verification of the modal bridge model vs closed forms and the full VBI FEM.
================================================================================
  1. Static mid-span deflection  -> converges to P L^3 / 48 EI.
  2. Modal frequencies           -> cross-checked against the full Euler-Bernoulli
                                    FEM eigensolve in vbi-bridge-sim (independent
                                    numerical method, not the same formula).
  3. Quasi-static crossing       -> a very slow load gives DAF -> 1.
  4. Dynamic crossing            -> DAF at speed is bounded and > 1 (sanity).

ASCII-only; run with: python -u
"""

import os
import sys
import numpy as np
from bridge_modal import BridgeModal, G


def check(name, got, want, tol, unit=""):
    ok = abs(got - want) <= tol
    print(f"  [{'OK ' if ok else 'XX '}] {name:42s} got {got:13.6g} {unit:5s} "
          f"want {want:13.6g}  (tol {tol:g})")
    return ok


def main():
    print("=" * 78)
    print("MODAL BRIDGE -- verification vs closed forms and the full VBI FEM")
    print("=" * 78)
    ok = True
    br = BridgeModal(L=40.0, EI=8.4e10, m_bar=12000.0, zeta=0.02, n_modes=6)

    # --- 1. static deflection ---------------------------------------------
    print("\n1. Static mid-span deflection (mid-span load) -> P L^3 / 48 EI")
    P = 12753.0                                  # ~1.3 t car weight
    w_modal = abs(br.static_midspan(P))
    w_closed = P * br.L**3 / (48 * br.EI)
    ok &= check("w_mid", w_modal, w_closed, tol=w_closed * 1e-3, unit="m")

    # --- 2. frequencies vs full FEM ---------------------------------------
    print("\n2. Modal frequencies vs full Euler-Bernoulli FEM (vbi-bridge-sim)")
    f_modal = br.frequencies_hz()
    print(f"      modal f = {np.array2string(f_modal[:4], precision=3)}")
    vbi = r"C:\Users\bypire\Downloads\vbi-bridge-sim\solver"
    try:
        sys.path.insert(0, vbi)
        from beam_fem import Beam, natural_frequencies
        E = 35e9
        beam = Beam(L=br.L, E=E, I=br.EI / E, mass_per_length=br.m_bar,
                    n_elements=40)
        f_fem = natural_frequencies(beam, 4)
        print(f"      FEM   f = {np.array2string(f_fem[:4], precision=3)}")
        for i in range(3):
            ok &= check(f"f{i+1} modal vs FEM", float(f_modal[i]),
                        float(f_fem[i]), tol=float(f_fem[i]) * 0.02, unit="Hz")
    except Exception as e:                       # VBI repo not present -> skip
        print(f"      (FEM cross-check skipped: {e})")

    # --- 3. quasi-static crossing -> DAF ~ 1 ------------------------------
    print("\n3. Quasi-static crossing (very slow) -> DAF -> 1")
    slow = br.crossing(P, v=0.5, dt=2e-3)        # 0.5 m/s, ~80 s crossing
    ok &= check("DAF (slow)", slow["daf"], 1.0, tol=0.03)

    # --- 4. dynamic crossing sanity ---------------------------------------
    print("\n4. Dynamic crossing at speed (DAF bounded, > 1)")
    for v in (20.0, 30.0, 45.0):
        res = br.crossing(P, v=v, dt=1e-3, settle=2.0)
        print(f"      v={v:4.0f} m/s ({v*3.6:5.0f} km/h)  DAF = {res['daf']:.3f}")
    fast = br.crossing(P, v=30.0, dt=1e-3, settle=2.0)
    sane = 1.0 <= fast["daf"] <= 2.0
    print(f"  [{'OK ' if sane else 'XX '}] DAF at 30 m/s in [1, 2]")
    ok &= sane

    print("\n" + "=" * 78)
    print("ALL CHECKS PASSED" if ok else "*** SOME CHECKS FAILED ***")
    print("=" * 78)
    return ok


if __name__ == "__main__":
    main()
