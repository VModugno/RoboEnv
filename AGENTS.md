# AGENTS.md — RoboEnv quick guide

RoboEnv is a robotic simulation environment managed with [pixi](https://pixi.sh). This is the condensed, agent-oriented version of the [README](README.md). Target: make installation trivial on Windows, Linux and macOS.

## Setup (all platforms, 2 commands)

```bash
git clone --recurse-submodules https://github.com/VModugno/RoboEnv.git
cd RoboEnv && pixi install
```

If pixi is missing: `irm https://pixi.sh/install-pixi.ps1 | iex` (Windows) or `curl -fsSL https://pixi.sh/install.sh | sh` (macOS/Linux).

Verify:

```bash
pixi run smoke-test          # → prints "simulation_and_control OK"
```

## What's here

| Path | What it is |
|---|---|
| `pixi.toml` | Environment manifest: conda-forge deps + editable submodule + run tasks |
| `pixi.lock` | Committed lock file, solved for win-64 / linux-64 / osx-64 / osx-arm64 |
| `simulation_and_control/` | Git submodule (https://github.com/VModugno/simulation_and_control), installed editable |
| `configs/`, `models/` | Simulation configs and URDF/scene models, read from the repo root |
| `tests/` | 5 example scripts that exercise the simulation stack |
| `.pixi/` | Generated environment (git-ignored, never commit it) |

## Common commands

```bash
pixi install                                # create/sync the environment
pixi run smoke-test                         # import check: numpy, pinocchio, pybullet, casadi, matplotlib, robot_descriptions, simulation_and_control
pixi run test-cartesian-kin                 # python tests/cartesian_kinematic_controller.py
pixi run test-cartesian-impedance           # python tests/cartesian_impedance_controller.py
pixi run test-humanoid-walk                 # python tests/humanoid_walk_controller.py
pixi run test-mobile-base-kin               # python tests/mobile_base_kinematic_controller.py
pixi run test-mobile-base-arm-kin           # python tests/mobile_base_arm_kinematic_controller.py
pixi shell                                  # activated shell, then use python/pytest/etc. directly
pixi run python your_script.py              # one-off script in the env
```

## Writing scripts that use the simulator

Scripts must tell `SimInterface` where `configs/` and `models/` live by passing the RoboEnv repo root as `conf_file_path_ext`:

```python
import os
import simulation_and_control.sim.pybullet_robot_interface as pb

root_dir = os.path.dirname(os.path.abspath(__file__))  # e.g. repo root containing configs/ and models/
sim = pb.SimInterface(conf_file_name="mycobot_280_pi.yaml", conf_file_path_ext=root_dir)
```

Do **not** copy `configs/`/`models/` around, and do **not** `pip install` the submodule by hand — the editable install via `[pypi-dependencies]` in `pixi.toml` already covers it. Changes inside `simulation_and_control/` are live immediately (editable install).

## Environment contents

- **conda deps** (conda-forge): pinocchio, pybullet, robot_descriptions, casadi, numpy, matplotlib
- **pypi dep**: `simulation_and_control` (editable, `./simulation_and_control` — the git submodule)
- **No ROS**: the former ros1/ros2 stack was deliberately removed; this repo is now simulation-only
- **Gepetto viewer** (`visualizer=True` in `PinWrapper`) is Linux-only and NOT installed by default; on Linux: `pixi add gepetto-viewer-corba` (local override, not in the lock file)

## Updating

```bash
pixi upgrade          # bump dependency versions, refresh pixi.lock
git add pixi.toml pixi.lock   # commit both together
```

Changing dependencies → edit `pixi.toml`, then `pixi install` re-solves and rewrites `pixi.lock`. Always commit `pixi.toml` + `pixi.lock` together.