# tests

Example scripts that exercise the simulation stack. Every script is wired to
the repo-root `configs/` and `models/` folders through
`conf_file_path_ext`, so they must be run from this clone (or a checkout with
`simulation_and_control` properly installed).

Run one with pixi:

```bash
pixi run test-cartesian-kin             # opens the pybullet gui
ROBOENV_HEADLESS=1 pixi run test-cartesian-kin   # headless (ci / no display)
```

## headless mode and step cap

The scripts read two environment variables:

- `ROBOENV_HEADLESS=1` -> pybullet runs in DIRECT mode (no gui), per-step
  prints are skipped and plots are not shown
- `ROBOENV_MAX_STEPS=N` -> the control loop breaks after N steps; defaults to
  5000 (cartesian controllers, mobile base + arm) or 40000 (mobile base: its
  three waypoint legs need ~32k steps) when headless, 0 (run until 'q') when
  not

## status

| script | status |
|---|---|
| `cartesian_kinematic_controller.py` | works (fixed `LoadEnv` env-script execution) |
| `cartesian_impedance_controller.py` | works (`ImpedanceController` ported from the `legged_robots` branch) |
| `mobile_base_kinematic_controller.py` | works (implicit-cylinder wheels + compliant wheel contacts + anisotropic friction along the wheel axle for skid-steer turns) |
| `mobile_base_arm_kinematic_controller.py` | works (arm posture pd gains sized for the 1 ms timestep, fingers left unactuated, wheel joint commands wired through `MotorCommands`) |
| `humanoid_walk_controller.py` | parked: needs the `legged_robots` submodule branch (config format + controller rework), not propagated to main |