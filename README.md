# RoboEnv

Welcome to **RoboEnv**, a repository designed to manage and set up the environment for robotic simulation and control. Please see the following sections to use this package.

## Repository Overview

**RoboEnv** provides:

- A consistent, reproducible environment for robotic simulation (`pixi.toml` + `pixi.lock`).
- The [`simulation_and_control`](https://github.com/VModugno/simulation_and_control) library, included as a git submodule and installed **editable** into the environment.
- Ready-to-run example/test scripts in `tests/` that exercise the simulation stack.

## Getting Started

### Prerequisites

You can use any python environment manager. We recommand using pixi or uv.

Install pixi with:

```bash
# Windows (PowerShell)
irm https://pixi.sh/install-pixi.ps1 | iex

# MacOS / Linux
curl -fsSL https://pixi.sh/install.sh | sh
```

Install uv with:
```bash
# Windows (PowerShell)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# MacOS / Linux
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Setup (Using pixi)

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


### Setup (Using uv)
1. Create and activate a Python environment
```bash
# Go to your workspace folder. For example, cd to a lab folder.
cd <path-to-workspace>

# Create a python environment using uv
uv venv -p 3.10

# Activate the environment
source .venv/bin/activate
```

2. Install this repo as a package
```bash
uv pip install <path-to-this-repo>
```

## Usage

### Run the example scripts (if you installed with pixi)

Each script in `tests/` has a pixi task (they run against the `configs/` and `models/` folders at the repository root — no copying required):

```bash
pixi run test-cartesian-kin
pixi run test-cartesian-impedance
pixi run test-humanoid-walk
pixi run test-mobile-base-kin
pixi run test-mobile-base-arm-kin
```

### Run the example scripts (if you installed with uv)
```bash
test-cartesian-kin
test-cartesian-impedance
test-humanoid-walk
test-mobile-base-kin
test-mobile-base-arm-kin

# Add "uv run" in front of the above commands if it did not work.
```

### Write your own scripts

With **pixi**, run scripts from the RoboEnv repository root:

```bash
pixi run python path/to/your_script.py
```

You can also activate the environment first:

```bash
pixi shell
python path/to/your_script.py
```

With **uv**, work from your own workspace after activating the virtual environment and installing RoboEnv as described above:

```bash
python path/to/your_script.py
```

To run through uv while using that activated environment:

```bash
uv run --active --no-project python path/to/your_script.py
```

`--active` selects your activated virtual environment, and `--no-project` avoids discovering and synchronizing another project. See the [uv documentation](https://docs.astral.sh/uv/reference/cli/#uv-run).

#### Locate configuration files and models

Pass the directory containing both `configs/` and `models/` as `conf_file_path_ext`. Use the same directory when constructing `PinWrapper`.

For a **uv installation**, RoboEnv bundles these assets with the installed package. Locate them through its installation metadata so your script can run from any workspace:

```python
from importlib.metadata import distribution
from simulation_and_control import pb

root_dir = str(distribution("roboenv").locate_file(""))
sim = pb.SimInterface(
    conf_file_name="pandaconfig.json",
    conf_file_path_ext=root_dir,
)
```

For **pixi**, use the RoboEnv checkout instead. If your script is saved at the repository root:

```python
from pathlib import Path
from simulation_and_control import pb

root_dir = str(Path(__file__).resolve().parent)
sim = pb.SimInterface(
    conf_file_name="pandaconfig.json",
    conf_file_path_ext=root_dir,
)
```

For a script in `RoboEnv/tests/`, use `Path(__file__).resolve().parents[1]`. If you maintain your own configurations and models, pass the absolute path to the directory containing your own `configs/` and `models/` folders.

### Changes to the installed package

The uv setup uses a regular installation: source code, examples, configurations, and models are installed into your virtual environment. After changing files in the RoboEnv checkout, reinstall the package to use those changes:

```bash
uv pip install --reinstall-package roboenv <path-to-this-repo>
```

## Environment details

- **Pixi dependencies**: `pinocchio`, `pybullet`, `robot_descriptions`, `casadi`, `numpy`, and `matplotlib`, supplied by conda-forge.
- **uv dependencies**: `numpy`, `pybullet`, `pin`, `robot-descriptions`, `casadi`, `matplotlib`, and `scipy`, declared in `pyproject.toml`. The PyPI package `pin` provides the robotics library imported as `pinocchio`.
- **Pixi platform support**: `pixi.lock` covers `win-64`, `linux-64`, `osx-64`, and `osx-arm64`. uv installation depends on the availability of the Python dependencies for your platform and Python version.
- **Reproducibility**: Pixi uses the committed `pixi.lock`. `uv pip install` resolves dependencies from `pyproject.toml` and does not use the Pixi lock file.
- **Bundled assets with uv**: The installed package contains the example configurations and models, including their meshes. No copying into your workspace is required.

### Optional: Gepetto visualization (Linux only)

The `visualizer=True` mode of `PinWrapper` uses [Gepetto Viewer](https://github.com/Gepetto/gepetto-viewer). It is not included in the default setup.

For a **Pixi environment on Linux**, add it with:

```bash
pixi add gepetto-viewer-corba
```

This updates the Pixi manifest and lock file. For a **uv environment**, Gepetto Viewer and its Python bindings require a separate installation following the viewer's instructions. Leave `visualizer=False` when the viewer is unavailable, including on Windows/macOS.

## Troubleshooting

- **PyBullet GUI does not open / crashes on a headless machine**: The examples open a GUI window. In your own script, use `SimInterface(..., use_gui=False)` to run without a display.
- **Python process dies silently on Windows during `numpy.linalg` calls (e.g. inside the controllers)**: the conda-forge `numpy` on win-64 can pick the MKL BLAS variant, whose OpenMP runtime delay-load crashes outside an activated shell. `pixi.toml` pins `libblas` to the OpenBLAS build for win-64 (`[target.win-64.dependencies]`), so a normal `pixi run` never hits this; if you roll that pin back and see bare `python.exe` processes vanish without a traceback, this is why.
- **`pixi` or `uv` command not found after installation**: Restart your terminal so the updated `PATH` is picked up.
- **uv example command not found / Python import fails**: Activate the workspace's virtual environment and install RoboEnv into it with `uv pip install <path-to-this-repo>`. You can also run a command through `uv run --active --no-project test-cartesian-kin` from that activated environment.
- **Configuration or model file not found in your own script**: Set `conf_file_path_ext` explicitly using the asset-location examples above. For uv, use the installed assets; for Pixi, use the repository root. If an older uv installation lacks bundled assets, reinstall RoboEnv with `uv pip install --reinstall-package roboenv <path-to-this-repo>`.
- **Submodule folders missing during installation**: Make sure RoboEnv was cloned with `--recurse-submodules`, or run `git submodule update --init --recursive` from the repository root. Then rerun `pixi install` or, from your activated uv workspace, `uv pip install <path-to-this-repo>`.

## Updating the environment

For **pixi**, run these commands from the RoboEnv repository root:

```bash
pixi upgrade
git add pixi.toml pixi.lock
```

For **uv**, reinstall after updating the RoboEnv checkout:

```bash
uv pip install --reinstall-package roboenv <path-to-this-repo>
```

To also upgrade dependencies to the newest compatible versions allowed by `pyproject.toml`:

```bash
uv pip install --upgrade --reinstall-package roboenv <path-to-this-repo>
```

See also [AGENTS.md](AGENTS.md) for the Pixi workflow and the [uv environment documentation](https://docs.astral.sh/uv/pip/environments/) for working with virtual environments.
