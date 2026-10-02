# NEXT_STEPS.md — humanoid IS-MPC walk integration (handoff doc)

Status date: 2026-10-02. Session context: humanoid walking integration onto our
SimInterface/pin_wrapper stack, following the pixi migration + controller repair
wave (tags `v0.1.0` on both repos mark the stable pre-humanoid state; 4 controller
tests green on all 4 OSes in CI run 37009300814).

## Where we are

Verified working (regression gate GREEN this session on the uncommitted tree):

- smoke-test OK; cartesian_kin / cartesian_impedance headless-cap clean;
  mobile_base_kin "Completed all waypoints"; mobile_base_arm "Reached ... t=4.16s".
- QP backend: `QPSolver` is dual-backend. It probes casadi conic availability once
  per process (`casadi.conic('qp_backend_probe','osqp')`) and falls back to the
  standalone `osqp` package (added to pixi.toml, solves on all 4 platforms).
  `ismpc.py` rewritten as explicit QP matrices (math verbatim from branch);
  `utils.py` QPSolver dual-backend; unit tests pass, MPC solve ~2.5 ms.
- **Casadi conic discovery (verified, do not re-litigate)**: an early local probe
  reported "casadi ships without conic plugins, on Windows AND in pip wheels" —
  that was WRONG. The plugins ship fine in BOTH channels:
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
- Whole-body QP-ID ported dartpy→pinocchio (`humanoid_controller/inverse_dynamics.py`):
  real torques, ~3–14 ms/solve, gravity-comp matches rnea on arms/torso.
- Standing hold: 2.5 s STABLE (z sag 1.8 cm, torques 23–26 Nm, 8 contacts ~mg/2
  per foot). 5 s hold slowly diverges — expected and fine: the demo never statically
  holds; the MPC walk loop is the stabilizer. "The standing should walk" is the next
  milestone, not a regression.
- All 5 `humanoid_controller` modules import OK.

Root causes LOCKED (do not re-litigate):

1. **pybullet `getBasePositionAndOrientation` returns the INERTIAL frame pose, not
   the link frame.** Correct with `T_link = T_inertial ∘ T_li⁻¹` where
   `p_li, q_li = getDynamicsInfo(rid, -1)[3]/[4]`. Velocity:
   `v_link_world = v_inertial_world + ω×(p_link−p_inertial)`, then rotate to local
   with `R_linkᵀ` for the pin freeflyer base block. Working reference implementation
   in `C:\Users\valer\AppData\Local\Temp\opencode\probe_hrp4_qp_hold.py`
   (`base_link_pose()` / `base_pin_velocity()`).
2. **hrp4config.json spawn values are CORRECT as authored** ([[0.0071,0.025,0.8668]]
   + quat [[0,−0.2164,0,0.9763]]): the author pre-compensated the inertial offset so
   the LINK frame lands upright at z≈0.726. Do NOT "fix" to upright+identity — that
   buries the feet and explodes the sim.
3. ZMP must come from `getContactPoints` world-frame aggregation (demo style);
   `ComputeFootGRF` (jointReactionForces) is unreliable (reads 698 N while airborne).
4. crba's `data.M` is symmetric in pinocchio 4.1.0; `jacobianCenterOfMass(q)` does
   not clobber `data.v/a`; `computeJointJacobians(q)` must run before
   `getFrameJacobian` (already in the ID).

## Uncommitted inventory (as of now)

Submodule `simulation_and_control` (main = 6eb165c, merge unpushed; dirty):
- `controllers/pin_wrapper.py` — ComputeJacobianFeet int-frame-id fix;
  FullInverseDynamicsWithContact enumerate→items() fix; NEW ComputeCoMPosition /
  ComputeCoMVelocity.
- `controllers/humanoid_controller/utils.py` — QPSolver dual-backend (casadi probe
  → osqp fallback).
- `controllers/humanoid_controller/ismpc.py` — explicit-QP rewrite (backend swap
  only).
- `controllers/humanoid_controller/inverse_dynamics.py` — full pinocchio QP-ID
  port (72 vars: qdd 30, tau 30, f_c 12; friction cone + CoP ineqs verbatim).
- `controllers/humanoid_controller/footstep_planner.py` — `from utils import *` →
  `from .utils import *` (import fix).

Parent RoboEnv (main = e899c71, clean):
- `configs/hrp4config.json` — feet nested per-robot `[[{name: lsole, sensor_frame:
  L_FOOT, contact_link_name: l_sole},{... rsole ...}]]`; base pose = original
  author values (see root-cause 2).
- `pixi.toml` + `pixi.lock` — `osqp = "*"` dependency.
- `simulation_and_control` gitlink dirty (tracks the above submodule changes).

## Ordered next steps

1. **Rewrite `controllers/HumanoidController.py`** (the last module; branch version
   is broken — bad imports, `self.hrp4` unassigned, `'lfoot'/'rfoot'` keys,
   commented-out ComputeController, inertial-frame bug). Spec below.
2. **Rewrite `tests/humanoid_walk_controller.py`** with the same
   `ROBOENV_HEADLESS` / `ROBOENV_MAX_STEPS` contract as the other 4 tests
   (see tests/README.md), instantiating the new controller.
3. **Headless walk debug** — stable walk or documented hard wall.
4. **Re-run the regression gate** (below) — must be all green.
5. **Commit wave, push submodule FIRST then parent**:
   submodule (pin_wrapper fixes; humanoid_controller utils/ismpc/inverse_dynamics/
   footstep_planner; HumanoidController) → parent (gitlink + hrp4config +
   pixi.toml/lock + tests/README humanoid status + ci.yml humanoid job, if working).

## HumanoidController rewrite spec (locked contracts)

- Package-relative imports (`from .humanoid_controller.ismpc import Ismpc`, etc.).
  Class `Hrp4Controller(dyn_model, sim)`; ctor reads dynamics info once for the
  inertial→link conversion constants.
- Params: `g 9.81, h 0.72, foot_size 0.1, step_height 0.02, ss 70, ds 30,
  first_swing 'right', µ 0.5, N 100, dof = getNumberofActuatedJoints(),
  eta = sqrt(g/h)`, and **`world_time_step = 0.01`** (DELIBERATE: the control tick;
  ss=70/ds=30 are steps of 0.01 s = 0.7/0.3 s). NOT `dyn_model.getTimeStep()`
  (=0.001). Controller runs every 10 sim steps (100 Hz); `self.time` increments per
  tick.
- Reference gait: `[(0.1,0.,0.2)]*5 + [(0.1,0.,-0.1)]*10 + [(0.1,0.,0.)]*10`.
  Redundant dofs (10): `NECK_Y, NECK_P, R/L_SHOULDER_P/R/Y, R/L_ELBOW_P`.
- `retrieve_state(sim)`: base link pose/velocity via the conversions (root-cause 1);
  keys `'lsole'/'rsole'/'com'/'torso'/'base'/'joint'`; feet pose = DART 6d
  `hstack(pin.log3(R), tr)`, feet vel 6d `[ang, lin]` from LWA Jacobian
  `J[[3,4,5,0,1,2],:] @ v_pin`; joint pos/vel = 24-dim motor vectors; ZMP from
  `getContactPoints` aggregation (root-cause 3), z-term `zmp_z = com_z − Fz/(m·g/h)`,
  clip ±0.3 around feet midpoint; store `q`/`dq` (ext convention) for the ID.
- Kalman: per-axis pair `block_diag(A,A)`, `A = I3 + dt·A_lip`, `B = dt·B_lip`,
  `H = I3`, `Q = diag(1,1,1)`, `R = diag(1e1,1e2,1e4)`, `P = I3`,
  `x = [comx,comvx,zmpx,comy,comvy,zmpy]`; `A_lip = [[0,1,0],[eta²,0,−eta²],[0,0,0]]`,
  `B_lip = [0,0,1]` col.
- `ComputeController()` loop (per tick, port of demo customPreStep):
  `current = retrieve_state(sim)` → `kf.predict(u = desired zmp vel xy)` →
  `kf.update([comx, comvx, zmpx, comy, comvy, zmpy])` → overwrite current com/zmp xy
  with filtered values → `lip_state, contact = mpc.solve(current, self.time)` →
  contact map `ds`→pass, `ssleft`→`'lsole'`, `ssright`→`'rsole'` → desired com
  pos/vel/acc + zmp pos/vel from lip_state →
  `feet_traj = ftg.generate_feet_trajectories_at_time(self.time)` → desired
  lsole/rsole = `feet_traj['left'/'right']` → **desired torso/base = average of the
  feet pose `[:3]` rotvec blocks (orientation-only — INTENTIONAL, do not "fix" to
  [3:]; flat feet keep rotvec≈0 → torso upright)** →
  `tau = id.get_joint_torques(desired, current, contact)` →
  `MotorCommands(tau, ['torque']*24)`.
- `Logger.initialize_plot()` uses `plt.ion()` — guard behind headless check
  (matplotlib already forced to Agg in headless test scripts; skip plotting when
  `ROBOENV_HEADLESS`).
- Torque limits: URDF effort is 100 N·m per joint; consider clipping/saturating
  `tau` at ±100 before `MotorCommands` (servo `torque_limits` unset in config).

## Regression gate (MUST pass before any commit)

```powershell
pixi run smoke-test                      # → simulation_and_control OK
$env:ROBOENV_HEADLESS='1'
pixi run test-cartesian-kin              # → Reached step cap 5000 ... finished
pixi run test-cartesian-impedance        # → Reached step cap 5000 ... finished
pixi run test-mobile-base-kin            # → Completed all waypoints
pixi run test-mobile-base-arm-kin        # → Reached the desired base position
```

Last full run (this session): ALL GREEN on the current uncommitted tree.

## Reference probes (temp dir, may be deleted after wave closes)

`probe_base_convention.py` (THE inertial-vs-link proof), `probe_hrp4_qp_hold.py`
(working hold controller w/ conversions — closest skeleton for the rewrite),
`probe_reorder_units.py`, `probe_pin_crba.py`, `test_qp_units.py`,
`probe_qp_timing.py`, harness `run_env.ps1`.