# RoboEnv

Welcome to **RoboEnv**, a repository designed to manage and set up the environment for robotic simulation and control. This repository includes an `environment.yaml` file that specifies all dependencies and links to other essential repositories, ensuring a seamless setup for robotic development and testing.

## Repository Overview

**RoboEnv** provides a central place to:
- Set up a consistent development environment for robotic simulation.
- Install and manage dependencies required for the `sim_and_control` and `mycobot_client` repositories.
- Facilitate easy integration and usage of robotic control algorithms.

## Getting Started

### Prerequisites

To get started with **RoboEnv**, you need to have the following software installed on your system:

- **Git**: Version control system to clone repositories.
- **Conda/Mamba**: Package manager for managing environments. Mamba is a fast, flexible, and user-friendly package manager. If you don't have Mamba installed, follow the installation instructions available [here](https://github.com/conda-forge/miniforge). once mamba is istalled move to the next section.

### Setting Up the Environment

1. **Clone the `roboenv` Repository:**

   ```bash
   git clone --recurse-submodules https://github.com/yourusername/roboenv.git
   cd roboenv
   ```

2.  **Install the Conda Environment:**

    The environment.yaml file includes all the dependencies required for the simulation and control projects. To create the environment, run the below commands.
    ```bash
    mamba env create -f environment_ros2.yaml
    mamba env activate roboenv2
    ```

    From here, there is a bug with ros2 and the new version of setuptools (see [here](https://github.com/colcon/colcon-python-setup-py/issues/41)). To build python ros2 packages, we need to replace some code in one of the python files in the mamba environment. There is a command to do this in Linux automatically below. You can run that. If you have trouble, look below this command.
    
    ```bash
    python -m site | grep -E miniforge3/envs.*?/lib/python3.11/site-packages | sed "s:,::g; s:\s::g; s:'::g" | awk '{print $1"/colcon_python_setup_py/package_identification/python_setup_py.py"}' | xargs sed -i -e "s/'from setuptools.extern.packaging.specifiers import SpecifierSet'/'from packaging.specifiers import SpecifierSet'/g"
    ```

    If you have trouble with the prior command, edit the file in a location similar to `/home/mz/miniforge3/envs/robo_env2_vm/lib/python3.11/site-packages/colcon_python_setup_py/package_identification/python_setup_py.py` and change the line with `'from setuptools.extern.packaging.specifiers import SpecifierSet'` to `'from packaging.specifiers import SpecifierSet'`.

3. **Activate the Environment:**
    ```bash
    conda activate roboenv2
    ```

4. **Pull Submodule (if you already cloned the repository without the submodules):**
    ```bash
    git submodule update --init --recursive
    ```
