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
- **Conda**: Package manager for managing environments.

### Setting Up the Environment

1. **Clone the `roboenv` Repository:**

   ```bash
   git clone https://github.com/yourusername/roboenv.git
   cd roboenv
   ```
2.  **Install the Conda Environment:**

    The environment.yaml file includes all the dependencies required for the simulation and control projects. To create the environment, run:
    ```bash
    conda env create -f environment.yaml
    ```
3. **Activate the Environment:**
    ```bash
    conda activate roboenv
