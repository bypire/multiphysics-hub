# Multiphysics Hub

[![verify](https://github.com/bypire/multiphysics-hub/actions/workflows/verify.yml/badge.svg)](https://github.com/bypire/multiphysics-hub/actions/workflows/verify.yml)

A real-time, top-down driving application in which a single car couples three simulations, all
computed live in the browser.

**Live demonstration: https://bypire.github.io/multiphysics-hub/**

- **Vehicle dynamics.** A double-track (four-wheel) model with Pacejka tyres, longitudinal and
  lateral load transfer, aerodynamic downforce, and a manual gearbox, handbrake and reverse.
- **Aerodynamics.** A two-dimensional fluid solver runs around the car outline every frame,
  showing the wake; the resulting drag and downforce feed back into the vehicle.
- **Vehicle-bridge interaction.** A modal moving-load beam deflects under the moving wheel load,
  coloured by bending moment, with the dynamic amplification factor.

The page opens in a self-driving demonstration loop; pressing any drive key hands over control.

![Screenshot: the car on the bridge, with the aerodynamics and bridge panels](web/hub_bridge_shot.png)

## A note on fidelity

The vehicle and bridge models are checked against analytical and finite-element references
(`py/verify_*.py`, and `web/xcheck.html` for the JavaScript ports). The aerodynamics panel is a
two-dimensional, low-Reynolds solver: it illustrates flow separation and the wake but is not a
quantitative aerodynamic prediction. The drag and downforce used by the vehicle come from the
standard coefficient relation `F = 0.5 rho C A v^2`; the Navier-Stokes solver is
the separate `vortex-street-cfd` project.

## Verification

The numerical models
are written in Python (NumPy) and checked against ground truth; the JavaScript ports that run in the
browser are then checked against those Python models on identical inputs. Both integrate at a fixed
250 Hz with RK4, decoupled from the render rate.

The checks that ship with the code:

- `py/verify_car_dynamic.py` and `py/verify_doubletrack.py`: the vehicle against static and
  steady-state references (static wheel loads, vertical equilibrium, load transfer against closed
  forms, low-speed Ackermann cornering, and the understeer balance trend).
- `py/verify_bridge.py`: the beam against the closed-form static deflection `P L^3 / 48 E I` and
  against the full Euler-Bernoulli FEM eigensolve from the VBI project.
- `web/xcheck.html`: the JavaScript ports against Python on identical inputs. The vehicle agrees to
  about 1e-13 over thousands of steps; the bridge crossing agrees to about 1e-9 metres against a
  0.23 mm peak deflection.
- `web/aero_test.html`: the fluid solver's own properties (the projection reduces divergence, the
  integration is stable, and a wake is shed).

## Controls

- Arrow keys or WASD: steer, throttle, brake.
- C and V: shift up and down. Space: handbrake. E: reverse (when stopped).
- T: toggle the force and velocity vectors and the driven line.
- R: reset. Edit track: reshape the circuit.

## Implementation

The numerical models are written in Python and reimplemented in JavaScript for interactivity; the
two are compared on identical inputs in `web/xcheck.html`. Vehicle and bridge states are integrated
with RK4 at a fixed 250 Hz, decoupled from the render rate.

## Running locally

Open `web/index.html` directly; there is no server or build step. To re-run the checks:

```bash
cd py
python verify_car_dynamic.py
python verify_doubletrack.py
python verify_bridge.py
python export_xcheck.py      # regenerates the JavaScript and Python reference data
```

## Layout

```
index.html               redirect to web/ (for GitHub Pages)
py/   car_dynamic.py      vehicle models (bicycle, double-track, Pacejka)
      bridge_modal.py     modal moving-load beam
      verify_*.py         analytical checks
      export_xcheck.py    writes the JavaScript and Python reference data
web/  index.html          the application
      car.js bridge.js    physics (JavaScript ports)
      fluid.js aero.js    two-dimensional fluid and the aero panel
      render.js main.js   rendering and the input loop
      track.js bridge_render.js   track and bridge panels
      xcheck.html aero_test.html  verification pages
```

## Limitations

Two-dimensional throughout. The aerodynamic field is qualitative. The bridge is a modal reduction of
a simply-supported span rather than a continuous multi-span girder. The gearbox is a driver-input
layer on top of the vehicle dynamics.
