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

    The environment.yaml file includes all the dependencies required for the simulation and control projects. To create the environment, run:
    ```bash
    mamba env create -f environment_ros2.yaml
    ```
3. **Activate the Environment:**
    ```bash
    conda activate robo_env
    ```

4. **Pull Submodule (if you already cloned the repository without the submodules):**
    ```bash
    git submodule update --init --recursive
    ```
