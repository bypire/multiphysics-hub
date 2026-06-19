# Multiphysics Hub

A real-time, top-down driving sandbox where a single car couples three physics
simulations, all computed live in the browser.

**Live demo:** https://bypire.github.io/multiphysics-hub/

- **Vehicle dynamics** — a double-track (four-wheel) model with Pacejka tyres,
  longitudinal and lateral load transfer, aerodynamic downforce, and a manual
  gearbox, handbrake and reverse.
- **Aerodynamics** — a 2D fluid solver runs around the car outline every frame,
  showing the wake; the resulting drag and downforce feed back into the vehicle.
- **Vehicle–bridge interaction** — a three-span beam deflects under the moving
  wheel load, coloured by bending moment, with the dynamic amplification factor.

The page opens in a self-driving demo loop; press any drive key to take over.

![screenshot](web/screenshot.png)

## A note on fidelity
The vehicle and bridge models are checked against analytical and FEM references
(`py/verify_*.py`, and `web/xcheck.html` for the JavaScript ports). The aerodynamics
panel is a 2D, low-Reynolds solver: it illustrates flow separation and the wake but
is not a quantitative aerodynamic prediction. The drag and downforce used by the
vehicle come from standard coefficient relations (½ρCv²); the reference Navier–Stokes
solver lives in the separate `vortex-street-cfd` project.

## Controls
- **Arrows / WASD** — steer, throttle, brake
- **C / V** — shift up / down · **Space** — handbrake · **E** — reverse (when stopped)
- **T** — toggle force/velocity vectors and the driven line
- **R** — reset · **Edit track** — reshape the circuit

## Implementation
The numerical models are written in Python and re-implemented in JavaScript for
interactivity; the two are compared on identical inputs in `web/xcheck.html`
(agreement to ~1e-13 for the vehicle, ~1e-9 m for the bridge). Vehicle and bridge
states are integrated with RK4 at a fixed 250 Hz, decoupled from the render rate.

Checks that ship with the code:
- `web/xcheck.html` — JavaScript vs Python for the vehicle and bridge models.
- `web/aero_test.html` — the fluid solver (divergence reduction, stability, wake).
- `py/verify_*.py` — analytical checks (e.g. cornering balance, `PL³/48EI`,
  modal frequencies vs the full beam FEM).

## Running locally
Open `web/index.html` directly — no server or build step. To re-run the checks:
```
cd py
python verify_car_dynamic.py
python verify_doubletrack.py
python verify_bridge.py
python export_xcheck.py      # regenerates the JS/Python reference
```

## Layout
```
index.html              redirect to web/ (for GitHub Pages)
py/   car_dynamic.py     vehicle models (bicycle, double-track, Pacejka)
      bridge_modal.py    modal moving-load beam
      verify_*.py        analytical checks
      export_xcheck.py   writes the JS/Python reference data
web/  index.html         the application
      car.js bridge.js   physics (JS ports)
      fluid.js aero.js   2D fluid + aero panel
      render.js main.js  rendering, input loop
      track.js bridge_render.js   track and bridge panels
      xcheck.html aero_test.html  verification pages
```

## Limitations
Two-dimensional throughout; the aero field is qualitative; the bridge is three
simply-supported spans rather than a continuous girder; the gearbox is a
driver-input model on top of the vehicle dynamics.
