import os
import time

import numpy as np
import matplotlib

# headless mode and step cap, used by ci and batch runs (see tests/README.md):
#   ROBOENV_HEADLESS=1   -> pybullet DIRECT (no gui), no per-step sleep/prints, plots skipped
#   ROBOENV_MAX_STEPS=N  -> break the control loop after N steps (defaults to 5000 when headless)
headless = os.environ.get("ROBOENV_HEADLESS", "").strip().lower() in ("1", "true", "yes")
max_steps = int(os.environ.get("ROBOENV_MAX_STEPS", "5000" if headless else "0"))

if headless:
    matplotlib.use("Agg")

from simulation_and_control import pb, MotorCommands, PinWrapper
from simulation_and_control.controllers import Hrp4Controller


def main():

    conf_file_name = "hrp4config.json"  # Configuration file for the robot
    root_dir = os.path.dirname(os.path.abspath(__file__))
    # added this line to manage the fact that the file is in tests folder
    name_current_directory = "tests"
    # remove current directory name from cur_dir
    root_dir = root_dir.replace(name_current_directory, "")
    # Configuration for the simulation
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext=root_dir, use_gui=not headless)

    # Get active joint names from the simulation
    ext_names = sim.getNameActiveJoints()
    ext_names = np.expand_dims(np.array(ext_names), axis=0)  # Adjust the shape for compatibility

    source_names = ["pybullet"]  # Define the source for dynamic modeling

    # Create a dynamic model of the robot
    dyn_model = PinWrapper(conf_file_name, "pybullet", ext_names, source_names, False, 0, root_dir)
    num_joints = dyn_model.getNumberofActuatedJoints()

    # whole-body IS-MPC walking controller (100 Hz tick, 1 kHz sim)
    controller = Hrp4Controller(dyn_model, sim, use_gui=not headless)

    n_motors = num_joints
    ctrl_every = 10  # 1 kHz sim step / 100 Hz control tick
    cmd = MotorCommands()  # Initialize command structure for motors

    current_time = 0.0
    time_step = sim.GetTimeStep()
    tick = 0

    while True:
        # 100 Hz: retrieve state, solve MPC + whole-body ID-QP, store tau
        tau_cmd = controller.ComputeController()
        # the command object holds the torque until the next control tick;
        # pybullet needs it re-applied at every simulation step
        cmd.SetControlCmd(tau_cmd, ["torque"] * n_motors)

        for _ in range(ctrl_every):
            sim.Step(cmd, "torque")  # Simulation step with torque command
            current_time += time_step

        tick += 1

        # Exit logic with 'q' key
        keys = sim.GetPyBulletClient().getKeyboardEvents()
        qKey = ord('q')
        if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
            break

        if max_steps and current_time / time_step >= max_steps:
            print(f"Reached step cap {max_steps} at t={current_time:.2f}s")
            break

        if not headless:
            time.sleep(0.01)  # Slow down the loop for better visualization

    if headless:
        print(f"Headless run finished at t={current_time:.2f}s")
        return

    # live plot of the desired/current CoM and ZMP trajectories
    controller.logger.initialize_plot() if hasattr(controller, 'logger') else None
    controller.logger.update_plot()


if __name__ == '__main__':
    main()