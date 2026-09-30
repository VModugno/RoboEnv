# RoboEnv

Welcome to **RoboEnv**, a repository designed to manage and set up the environment for robotic simulation and control. The environment is defined with [pixi](https://pixi.sh) and works seamlessly on **Windows, Linux and macOS** (Intel and Apple Silicon).

## Repository Overview

**RoboEnv** provides:

- A consistent, reproducible environment for robotic simulation (`pixi.toml` + `pixi.lock`).
- The [`simulation_and_control`](https://github.com/VModugno/simulation_and_control) library, included as a git submodule and installed **editable** into the environment.
- Ready-to-run example/test scripts in `tests/` that exercise the simulation stack.

## Getting Started

### Prerequisites

The only requirement is [pixi](https://pixi.sh) (and Git). Install pixi with:

```bash
# Windows (PowerShell)
irm https://pixi.sh/install-pixi.ps1 | iex

# macOS / Linux
curl -fsSL https://pixi.sh/install.sh | sh
```

### Setup

1. **Clone the repository with its submodule:**

   ```bash
   git clone --recurse-submodules https://github.com/VModugno/RoboEnv.git
   cd RoboEnv
   ```

   If you already cloned without submodules:

   ```bash
   git submodule update --init --recursive
   ```

2. **Install the environment:**

   ```bash
   pixi install
   ```

   That's it. pixi downloads the locked conda packages for your platform and installs the `simulation_and_control` submodule editable — no compilers, no VS Build Tools, no activation scripts needed on any OS.

3. **Verify the installation:**

   ```bash
   pixi run smoke-test
   ```

   You should see `simulation_and_control OK`.

## Usage

### Run the example scripts

Each script in `tests/` has a pixi task (they run against the `configs/` and `models/` folders at the repository root — no copying required):

```bash
pixi run test-cartesian-kin
pixi run test-cartesian-impedance
pixi run test-humanoid-walk
pixi run test-mobile-base-kin
pixi run test-mobile-base-arm-kin
```

### Write your own scripts

Run any script inside the environment with:

```bash
pixi run python path/to/your_script.py
```

or start an activated shell once and work as usual:

```bash
pixi shell
python path/to/your_script.py
```

The example scripts locate `configs/` and `models/` by passing the repository root as `conf_file_path_ext` to `SimInterface`:

```python
import os
import simulation_and_control.sim.pybullet_robot_interface as pb

# Repository root = directory containing configs/ and models/
root_dir = os.path.dirname(os.path.abspath(__file__))
sim = pb.SimInterface(conf_file_name="mycobot_280_pi.yaml", conf_file_path_ext=root_dir)
```

Alternatively, copy the `configs` and `models` folders next to your script — `SimInterface` falls back to `../configs/` relative to its own package when `conf_file_path_ext` is not given.

### The editable submodule

`simulation_and_control` is installed editable from `./simulation_and_control`: any change you make in the submodule sources is immediately visible in the environment — no reinstall needed. Pulling upstream updates is just `git pull` inside the submodule.

## Environment details

- **Dependencies** (all from conda-forge): `pinocchio`, `pybullet`, `robot_descriptions`, `casadi`, `numpy`, `matplotlib`.
- **Cross-platform**: `pixi.lock` is solved for `win-64`, `linux-64`, `osx-64` and `osx-arm64`; the same two commands (`git clone --recurse-submodules`, `pixi install`) work identically on all of them.
- **Reproducibility**: `pixi.lock` is committed; everyone gets the exact same package versions.

### Optional: Gepetto visualization (Linux only)

The `visualizer=True` mode of `PinWrapper` uses [Gepetto Viewer](https://github.com/Gepetto/gepetto-viewer), which is only packaged for Linux. On Linux you can add it to a local environment without touching the lock file:

```bash
pixi add gepetto-viewer-corba
```

On Windows/macOS simply don't pass `visualizer=True`.

## Troubleshooting

- **PyBullet GUI does not open / crashes on a headless machine**: the example scripts open a GUI window; on a remote/headless box use `SimInterface(..., pb.GUI)` with `pb.DIRECT` (see the submodule docs) or run on a machine with a display.
- **`pixi` command not found after install**: restart your terminal so the updated `PATH` (from `~/.pixi/bin` or `%USERPROFILE%\.pixi\bin`) is picked up.
- **Submodule folders missing / import errors**: make sure you cloned with `--recurse-submodules` (or ran `git submodule update --init --recursive`), then re-run `pixi install`.

## Updating the environment

```bash
pixi upgrade   # update pinned versions and refresh pixi.lock
git add pixi.toml pixi.lock
```

See also [AGENTS.md](AGENTS.md) for a condensed, agent-friendly version of these instructions.