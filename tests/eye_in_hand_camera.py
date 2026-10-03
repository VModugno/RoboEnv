import numpy as np
import time
import os

# headless mode and step cap, used by ci and batch runs (see tests/README.md):
#   ROBOENV_HEADLESS=1   -> pybullet DIRECT (no gui), no per-step sleep/prints, plots skipped
#   ROBOENV_MAX_STEPS=N  -> break the control loop after N steps (defaults to 5000 when headless)
headless = os.environ.get("ROBOENV_HEADLESS", "").strip().lower() in ("1", "true", "yes")
max_steps = int(os.environ.get("ROBOENV_MAX_STEPS", "5000" if headless else "0"))

import matplotlib
if headless:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from simulation_and_control import pb, MotorCommands, PinWrapper, feedback_lin_ctrl, SinusoidalReference, CartesianDiffKin


img_w, img_h = 640, 480
frame_stride = 500

def save_camera_frame(sim, out_dir, frame_counter, current_time):
    # camera rigidly mounted on the flange link: optical center 5 cm ahead of the
    # flange origin (so the own forearm stays out of the view) and pitched 60
    # degrees down from the flange z axis so that it frames the cube of the scene
    # on the ground (camera_forward_link=(-sin60, 0, cos60)). at this robot
    # configuration the link y axis points downward in the world, so
    # camera_up_link=(0,-1,0) keeps the image upright. change these vectors to
    # mount the camera differently.
    rgba, depth, seg = sim.GetCameraImageFromLink(
        "panda_link8", img_w, img_h,
        camera_offset_pos=(0.0, 0.0, 0.05),
        camera_forward_link=(-0.866, 0.0, 0.5),
        camera_up_link=(0.0, -1.0, 0.0))
    if not isinstance(rgba, np.ndarray) or rgba.size == 0:
        print("ERROR: empty camera frame, no image saved")
        return
    fname = os.path.join(out_dir, f"frame_{frame_counter:03d}.png")
    plt.imsave(fname, rgba)
    print(f"saved {fname} at t={current_time:.2f}s")


def main():

    conf_file_name = "pandaconfig.json"
    root_dir = os.path.dirname(os.path.abspath(__file__))
    name_current_directory = "tests"
    root_dir = root_dir.replace(name_current_directory, "")
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext = root_dir, use_gui=not headless)

    ext_names = sim.getNameActiveJoints()
    ext_names = np.expand_dims(np.array(ext_names), axis=0)

    source_names = ["pybullet"]

    dyn_model = PinWrapper(conf_file_name, "pybullet", ext_names, source_names, False,0,root_dir)
    num_joints = dyn_model.getNumberofActuatedJoints()

    controlled_frame_name = "panda_link8"
    init_joint_angles = sim.GetInitMotorAngles()
    init_cartesian_pos, init_R = dyn_model.ComputeFK(init_joint_angles,controlled_frame_name)
    print(f"Initial joint angles: {init_joint_angles}")
    print(f"Initial cartesian position of {controlled_frame_name}: {init_cartesian_pos}")

    joint_vel_limits = sim.GetBotJointsVelLimit()

    q_des =  init_joint_angles
    qd_des_clip = np.zeros(num_joints)

    amplitudes = [0.05, 0.1, 0.05]
    frequencies = [0.4, 0.5, 0.4]

    amplitude = np.array(amplitudes)
    frequency = np.array(frequencies)
    ref = SinusoidalReference(amplitude, frequency,init_cartesian_pos)

    time_step = sim.GetTimeStep()
    current_time = 0
    cmd = MotorCommands()

    kp_pos = 100
    kp_ori = 0

    kp = 1000
    kd = 100

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "eye_in_hand_frames")
    os.makedirs(out_dir, exist_ok=True)
    for old_frame in os.listdir(out_dir):
        os.remove(os.path.join(out_dir, old_frame))
    frame_counter = 0

    save_camera_frame(sim, out_dir, frame_counter, current_time)
    frame_counter += 1

    step_counter = 0

    while True:
        q_mes = sim.GetMotorAngles(0)
        qd_mes = sim.GetMotorVelocities(0)

        p_d, pd_d = ref.get_values(current_time)

        ori_des = None
        ori_d_des = None
        q_des, qd_des_clip = CartesianDiffKin(dyn_model,controlled_frame_name,q_mes, p_d, pd_d, ori_des, ori_d_des, time_step, "pos",  kp_pos, kp_ori, np.array(joint_vel_limits))

        tau_cmd = feedback_lin_ctrl(dyn_model, q_mes, qd_mes, q_des, qd_des_clip, kp, kd)
        cmd.SetControlCmd(tau_cmd, ["torque"]*7)
        sim.Step(cmd, "torque")

        if dyn_model.visualizer:
            for index in range(len(sim.bot)):
                q = sim.GetMotorAngles(index)
                dyn_model.DisplayModel(q)

        step_counter += 1
        if step_counter % frame_stride == 0:
            save_camera_frame(sim, out_dir, frame_counter, current_time)
            frame_counter += 1

        keys = sim.GetPyBulletClient().getKeyboardEvents()
        qKey = ord('q')
        if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
            break

        if max_steps and current_time / time_step >= max_steps:
            print(f"Reached step cap {max_steps} at t={current_time:.2f}s")
            break

        if not headless:
            time.sleep(0.01)
        current_time += time_step
        if not headless:
            print("current time in seconds",current_time)

    print(f"Finished at t={current_time:.2f}s, saved {frame_counter} camera frames in {out_dir}")


if __name__ == '__main__':
    main()