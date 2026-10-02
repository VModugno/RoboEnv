# NEXT_STEPS.md — humanoid IS-MPC walk integration (handoff doc)

Status date: 2026-10-02 (updated after the walk milestone). Session context:
humanoid walking integration onto our SimInterface/pin_wrapper stack, following
the pixi migration + controller repair wave (tags `v0.1.0` on both repos mark
the stable pre-humanoid state; 4 controller tests green on all 4 OSes in CI run
37009300814).

## Where we are

**The humanoid walks.** Full-stack IS-MPC walking is working headless:

- `Hrp4Controller` (IS-MPC + whole-body QP-ID, pinocchio port) implemented in
  `simulation_and_control/controllers/HumanoidController.py`; all 5
  `humanoid_controller` modules import OK.
- **In-place stepping (MS1)**: 5 s headless, com z sag 2 mm, com xy drift
  3.7 cm, torque max 49.5 N·m (never saturates ±100), correct alternating
  swing/stance, no QP divergence.
- **Walk gait (MS2)** — default reference `[(0.1,0.,0.2)]*5 +
  [(0.1,0.,-0.1)]*10 + [(0.1,0.,0.)]*10`: 2.14 m travel over the 25 s plan,
  com tracking error ≤ 3 mm vs the LIP reference, com z sag < 5 mm, torque max
  62.6 N·m, turn phases execute, final step lands and the robot ends standing.
- QP backend: `QPSolver` is dual-backend. It probes casadi conic availability
  once per process (`casadi.conic('qp_backend_probe','osqp')`) and falls back
  to the standalone `osqp` package (added to pixi.toml, solves on all 4
  platforms). `ismpc.py` rewritten as explicit QP matrices (math verbatim from
  branch); unit tests pass, MPC solve ~2.5 ms.
- Whole-body QP-ID ported dartpy→pinocchio (`humanoid_controller/inverse_dynamics.py`):
  real torques, ~3–14 ms/solve, gravity-comp matches rnea on arms/torso.
- **Casadi conic discovery (verified, do not re-litigate)**: an early local
  probe reported "casadi ships without conic plugins, on Windows AND in pip
  wheels" — that was WRONG. The plugins ship fine in BOTH channels:
  - conda-forge casadi 3.7.2 bundles `casadi_conic_osqp` (+ other conic solvers)
    on ALL 4 platforms — in `Library/bin` on win-64 (`casadi_conic_osqp.dll`,
    no lib prefix), `lib/` on linux (`libcasadi_conic_osqp.so`) and osx (`.dylib`).
  - pip wheels casadi 3.8.1 bundle them too (`casadi/libcasadi_conic_osqp.dll` +
    `libosqp.dll` in the package dir on win_amd64; likewise linux/mac).
  The early probe missed them because (a) it inspected `site-packages/casadi/`
  where conda builds keep no DLLs, and (b) it ran with BARE python, so
  `Library/bin` was not on the DLL search path — the loader could not find the
  plugin even though it existed. Same PATH trap class as the win-64 MKL/openblas
  issue (see README troubleshooting): run inside `pixi run` (activated env) and
  the casadi conic path is expected to work, e.g. on ubuntu CI.
  Windows nuance: casadi plugin deps sit next to the plugin in `Library/bin`;
  dependent-DLL search may still fail without the activated PATH (upstream
  issues #3443/#4340), so the osqp fallback remains the safety net there.
  Consequence for pixi.toml comment wording: prefer "casadi conic needs the
  activated env / PATH; osqp is the portable fallback" over "plugins missing".
- Regression gate ALL GREEN on the final tree: smoke-test, cartesian_kin,
  cartesian_impedance, mobile_base_kin, mobile_base_arm_kin, plus the new
  humanoid walk test (26 s headless, full plan).
- **CI humanoid walk job delivered**: `humanoid-walk` in `.github/workflows/ci.yml`
  runs the full walk (`ROBOENV_MAX_STEPS=26000`, headless) on the 4-OS matrix,
  25-min job timeout. Local wall-clock: 1 min 46 s on Windows. `scipy` is now
  declared in pixi.toml (direct import by the humanoid controller KF;
  previously transitive-only, lock unchanged).

Root causes LOCKED (do not re-litigate):

1. **pybullet `getBasePositionAndOrientation` returns the INERTIAL frame pose,
   not the link frame.** Correct with `T_link = T_inertial ∘ T_li⁻¹` where
   `p_li, q_li = getDynamicsInfo(rid, -1)[3]/[4]`. Velocity:
   `v_link_world = v_inertial_world + ω×(p_link−p_inertial)`, then rotate to
   local with `R_linkᵀ` for the pin freeflyer base block. Implemented in
   `Hrp4Controller._base_link_state()`.
2. **hrp4config.json spawn values are CORRECT as authored**
   ([[0.0071,0.025,0.8668]] + quat [[0,−0.2164,0,0.9763]]): the author
   pre-compensated the inertial offset so the LINK frame lands upright at
   z≈0.726. Do NOT "fix" to upright+identity — that buries the feet and
   explodes the sim.
3. ZMP must come from `getContactPoints` world-frame aggregation (demo style);
   `ComputeFootGRF` (jointReactionForces) is unreliable (reads 698 N while
   airborne).
4. crba's `data.M` is symmetric in pinocchio 4.1.0; `jacobianCenterOfMass(q)`
   does not clobber `data.v/a`; `computeJointJacobians(q)` must run before
   `getFrameJacobian` (already in the ID).

Debug findings LOCKED (these made the walk work — see the module docstring of
`HumanoidController.py` for the code-level details):

5. **ZMP cold start**: the controller is constructed before the first pybullet
   step, so there are no contact points yet; a zero ZMP seed makes the MPC see
   a fake 3.5 cm offset and push the robot forward until the QP turns
   infeasible. Seed the first ZMP with the CoM projection (at rest ZMP = CoM
   xy).
6. **The Kalman filter is REQUIRED for closed-loop stability** (it was
   originally staged for later): the raw pybullet ZMP is noisy (sole-edge
   rocking, ±30% fz spikes) and fed unfiltered into the unstable LIP dynamics
   it diverged laterally during in-place stepping. The KF port (demo
   simulation.py parameters, block-diagonal per axis) fixed it.
7. **Desired com height must be pinned to the initial measured com z**
   (~0.761 m), not the LIP constant h = 0.72 m: commanding 0.72 against an
   actual 0.761 pulls the body down through the QP constantly.
8. **osqp can return huge-but-finite garbage instead of raising** when it hits
   max_iter — the divergence guard in `ComputeController` (|lip| > 100 → hold
   previous torque) catches it.
9. **t_max = start of the last planned step − 1** (2499 for the 25-step plan),
   not the demo-style horizon safety margin: freezing at 2400 left the final
   step half prepared and the robot tipped over the stance toe while the ZMP
   reference kept advancing inside the frozen horizon.

## What this wave delivered

- Submodule `simulation_and_control`: `pin_wrapper.py` (ComputeJacobianFeet
  frame-id fix, contact-ID items() fix, ComputeCoMPosition/ComputeCoMVelocity);
  `humanoid_controller/utils.py` (QPSolver dual-backend); `ismpc.py`
  (explicit-QP rewrite); `inverse_dynamics.py` (full pinocchio QP-ID port,
  72 vars: qdd 30, tau 30, f_c 12); `footstep_planner.py` (package-relative
  import fix); **`HumanoidController.py` (full rewrite, the working
  controller)**; `controllers/__init__.py` (Hrp4Controller export).
- Parent RoboEnv: `configs/hrp4config.json` (nested feet format, original
  spawn values per root-cause 2); `pixi.toml`/`pixi.lock` (osqp dep);
  **`tests/humanoid_walk_controller.py`** (ROBOENV_HEADLESS/ROBOENV_MAX_STEPS
  contract like the other 4 tests, 100 Hz control tick, torque mode);
  gitlink bump.

## How to run the walk

```powershell
pixi run test-humanoid-walk                          # GUI, full plan
$env:ROBOENV_HEADLESS='1'; $env:ROBOENV_MAX_STEPS='26000'
pixi run python tests/humanoid_walk_controller.py    # headless, full plan (26 s)
```

The test exercises the default walk gait; pass a custom `vref` list of
`(vx, vy, wtheta)` tuples to `Hrp4Controller(...)` for other gaits (e.g.
`[(0.,0.,0.)]*25` for in-place stepping).

## Future work

- **GUI visual verification**: the walk is validated headless via telemetry;
  an eyes-on GUI run is still pending.
- **KF tuning**: current R = diag(1e1, 1e2, 1e4) comes straight from the demo;
  if faster gaits jitter, tune Q/R before touching anything else.
- **Non-flat ground**: the ZMP aggregation drops the tangential friction term
  (scales with zmp_z − point_z ≈ 0 on flat ground) — revisit for ramps/terrain.
- **Gait variety**: sideways and faster gaits; the unicycle planner accepts
  arbitrary per-step vref.

## Regression gate (MUST pass before any commit)

```powershell
pixi run smoke-test                      # → simulation_and_control OK
$env:ROBOENV_HEADLESS='1'
pixi run test-cartesian-kin              # → Reached step cap 5000 ... finished
pixi run test-cartesian-impedance        # → Reached step cap 5000 ... finished
pixi run test-mobile-base-kin             # → Completed all waypoints
pixi run test-mobile-base-arm-kin         # → Reached the desired base position
$env:ROBOENV_MAX_STEPS='26000'
pixi run test-humanoid-walk              # → Reached step cap 26000 ... finished
```

Last full run: ALL GREEN on the final tree.

## Reference probes (temp dir, may be deleted after wave closes)

`probe_base_convention.py` (THE inertial-vs-link proof), `probe_hrp4_qp_hold.py`
(2.5 s standing hold, stable), `probe_joint_order.py` (ext vs pin joint
order), `probe_reorder_units.py`, `probe_pin_crba.py`, `test_qp_units.py`,
`probe_qp_timing.py`, `probe_ms1_balance.py` (MS1 in-place telemetry),
`probe_ms2_walk.py` (MS2 walk telemetry), harness `run_env.ps1`.