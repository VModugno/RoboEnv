# Recent changes — verification guide for ubuntu

This document lists every change from the recent "repair the example controllers" wave
(commits are on local `main`, see repo state below) and how to verify them on an
ubuntu machine. Written for a verifying agent: every check has an exact expected output.

## Repo state — push order matters

The parent needs to be pushed after this update; the submodule is already on its
remote (see below). The submodule must be pushed FIRST, otherwise a fresh clone
checks out a parent whose gitlink points to a submodule commit that does not
exist on the remote:

```bash
git -C simulation_and_control push origin main   # already done
git push origin main                             # then parent
```

The parent history was rebased on top of the upstream `add-uv-support` merge
(`4fbe4ca`), so the repair-wave commits listed below sit after it. If you are
reading this on a fresh clone and the commits below are missing, the push has
not happened yet.

## Commits

### submodule `simulation_and_control` (main — matches origin/main, pushed)

| commit | change |
|---|---|
| `9ffe03d` | `LoadEnv` executes the whole env script in a single namespace (was `exec` per line, broke multi-line variables, `NameError: cube_size`) — `sim/pybullet_robot_interface.py` |
| `7eb4cda` | `ImpedanceController` ported from the `legged_robots` branch (commit `53dcd38`): new `controllers/ImpedanceCtrl.py` + export in both `__init__.py` files. Only the impedance bits were propagated; the humanoid/feet-format work stays on the branch. |
| `276918f` | `servo_motor.py`: torque/velocity branches now `np.ravel(motor_commands.ctrl_cmd[i])[0]` — 2-D ctrl_cmd rows crashed mixed-interface commands (`ValueError: setting an array element with a sequence`) |
| `14a9ea3` | all three `loadURDF` flag expressions add `URDF_USE_IMPLICIT_CYLINDER` (default tessellates `<cylinder>` into a 32-gon hull; polygonal drum wheels wedge under load and stall the robot) + `SetFloorFriction` now uses its parameter instead of hardcoding `lateralFriction=1.0` |

### parent `RoboEnv` (main, 6 ahead of origin/main, on top of the uv merge `4fbe4ca`)

| commit | change |
|---|---|
| `16c0d86` | `pixi.toml` + `pixi.lock`: win-64 `libblas` pinned to the openblas build (MKL variant crashes natively in `numpy.linalg` outside an activated shell — Windows-only fix, linux-64 solve unchanged); smoke-test now also calls `numpy.linalg.pinv` |
| `298b2cc` | 4 test scripts: `ROBOENV_HEADLESS` / `ROBOENV_MAX_STEPS` env switches (pybullet DIRECT, no per-step prints, plots skipped, step cap); `mobile_base_arm` rewritten (undefined `cmd`, `setCommand`→`SetControlCmd`, `QuaternionToEuler`→pinocchio, gains kp=300/kd=15 sized for the 1 ms timestep, q_des clamped to joint limits, fingers left unactuated — 0.01 kg links explode under any gain); `mobile_base_kin` keeps its original 3 waypoints (straight → in-place 90° turn → straight) and gains a wheel-physics block: compliant contacts (`contactStiffness=30000`, `contactDamping=1000` — husky urdf values) + `anisotropicFriction=[1, 0.01, 1]` (link-frame Y is the wheel axle; cutting axial grip makes skid-steer turns physically possible with implicit-cylinder wheels), servo gains `(400, 30)`, controller caps `max_linear_velocity=0.15`, `max_angular_velocity=1.0` |
| `ce3c9ee` | `tests/README.md`: env-var contract + per-test status table |
| `888b950` | `.github/workflows/ci.yml`: new `controllers` job — the 4 working tests headless on ubuntu-latest, macos-latest, macos-15-intel, windows-latest (timeout 20 min) |
| `b25b9c7` | gitlink bump to submodule `14a9ea3` + README troubleshooting entry for the Windows BLAS pitfall |
| `c133ea7` | `VERIFICATION.md`: this file |

## Verification on ubuntu

### Fresh install

```bash
git clone --recurse-submodules https://github.com/VModugno/RoboEnv.git
cd RoboEnv
pixi install
```

(pixi itself: `curl -fsSL https://pixi.sh/install.sh | sh`; do not `pip install` the
submodule — the editable pypi-dependency in `pixi.toml` covers it.)

### 1. smoke test

```bash
pixi run smoke-test
```

Expected: a pybullet build banner, then `simulation_and_control OK`. This also
exercises `numpy.linalg.pinv` (BLAS/LAPACK backend check).

### 2. example controllers, headless

Run from the repo root. `ROBOENV_HEADLESS=1` selects pybullet DIRECT mode — no
display needed. Expected outputs and rough wall times (measured on windows, ubuntu
should be comparable or faster):

| command | expected stdout (grep targets) | wall time |
|---|---|---|
| `ROBOENV_HEADLESS=1 pixi run test-cartesian-kin` | `Reached step cap 5000` + `Headless run finished at t=5.00s` | ~5 s |
| `ROBOENV_HEADLESS=1 pixi run test-cartesian-impedance` | `Reached step cap 5000` + `Headless run finished at t=5.00s` | ~7 s |
| `ROBOENV_HEADLESS=1 pixi run test-mobile-base-arm-kin` | `Reached the desired base position at t=4.16s` + `Headless run finished` | ~12 s |
| `ROBOENV_HEADLESS=1 pixi run test-mobile-base-kin` | `Reached waypoint 1`, `Reached waypoint 2`, `Reached waypoint 3`, `Completed all waypoints. Square path traversal finished.` | **~1–2 min** (needs ~32k steps; default headless cap is 40000) |

Pass criteria: process exits cleanly (no traceback). The two cartesian controllers
are continuous demos — in headless mode they terminate via the step cap, that IS
the pass condition. The two mobile tests terminate via waypoints, before the cap.

### 3. gui mode (only if a display is available)

```bash
pixi run test-cartesian-kin
```

Expected: a pybullet GUI window opens and the loop keeps running (kill with 'q'
key or ctrl-c). The env vars must NOT be set for this — default behavior is
unchanged.

### 4. ci

After pushing (submodule first, then parent), the GitHub Actions run covers all
of the above automatically on ubuntu-latest, both macos flavors and windows:
job `pixi-install` (smoke test) and job `controllers` (the 4 tests headless).

## Things that are NOT bugs (do not flag these)

- `test-humanoid-walk` fails on purpose: parked. It needs the unfinished
  `legged_robots` submodule branch (config-format + controller rework) that was
  deliberately not propagated to main. Documented in `tests/README.md`.
- Wheel slip on straight legs: measured base velocity is ~60% of commanded —
  expected with implicit-cylinder wheels + compliant contacts.
- In-place 90° turn is slow (~0.35 rad/s delivered vs ~1.0 commanded, drift ~1 cm):
  physically correct for a skid-steer with knife-edge wheel contact; the turn
  takes ~4.5k steps. That is why the mobile_base_kin headless cap is 40000.
- The win-64 openblas pin in `pixi.toml` does not affect linux (linux-64 solves
  to `blis`, osx to `openblas` — both unchanged by this wave).
- The wheel physics block (contact params + anisotropic friction) lives in the
  test script, not the interface: `anisotropicFriction` has no URDF/pybullet
  `<contact>` equivalent, and the axle axis (link-frame Y) is summit-robot
  specific.
- `macos-15-intel` in ci.yml is a valid current runner label (actionlint 1.7.12
  accepts it; older actionlint versions have a stale label database and false-
  positive on it).

## Reference docs in repo

- `tests/README.md` — per-test status + env-var contract
- `README.md` — install, usage, troubleshooting (incl. the windows BLAS pitfall)
- `AGENTS.md` — condensed agent guide