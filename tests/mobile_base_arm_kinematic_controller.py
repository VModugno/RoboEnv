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
import pinocchio as pin
from simulation_and_control import pb, MotorCommands, PinWrapper, SinusoidalReference, CartesianDiffKin
from simulation_and_control import differential_drive_controller_adjusting_bearing


def quaternion2bearing(q_w, q_x, q_y, q_z):
    quat = pin.Quaternion(q_w, q_x, q_y, q_z)
    quat.normalize()
    base_euler = pin.rpy.matrixToRpy(quat.toRotationMatrix())
    return base_euler[2]
   

def main():
    # Configuration for the simulation
    conf_file_name = "robotnik_arm.json"  # Configuration file for the robot
    root_dir = os.path.dirname(os.path.abspath(__file__))
    # added this line to manage the fact that the file is in tests folder
    name_current_directory = "tests"
    # remove current directory name from cur_dir
    root_dir = root_dir.replace(name_current_directory, "")
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext = root_dir, use_gui=not headless)  # Initialize simulation interface

    # Get active joint names from the simulation
    ext_names = sim.getNameActiveJoints()
    ext_names = np.expand_dims(np.array(ext_names), axis=0)  # Adjust the shape for compatibility

    source_names = ["pybullet"]  # Define the source for dynamic modeling

    # Create a dynamic model of the robot
    dyn_model = PinWrapper(conf_file_name, "pybullet", ext_names, source_names, False,0,root_dir)
    num_joints = dyn_model.getNumberofActuatedJoints()

    controlled_frame_name = "robot_j2s7s300_end_effector"
    init_joint_angles = sim.GetInitMotorAngles()
    init_base_pos = sim.GetBasePosition()
    init_base_ori = sim.GetBaseOrientation()
    init_state_position = np.concatenate((init_base_pos, init_base_ori,init_joint_angles))
    init_cartesian_pos,init_R = dyn_model.ComputeFK(init_state_position,controlled_frame_name)
    # print init joint
    print(f"Initial joint angles: {init_joint_angles}")
    
    # check joint limits
    lower_limits, upper_limits = sim.GetBotJointsLimit()
    print(f"Lower limits: {lower_limits}")
    print(f"Upper limits: {upper_limits}")


    joint_vel_limits = sim.GetBotJointsVelLimit()
    
    print(f"joint vel limits: {joint_vel_limits}")
    
    # fixed initial position
    des_base_pos = np.array([0.5, 0.5, 0.0])
    des_base_ori = np.array([0.0, 0.0, 0.0, 1.0])
    # desired actuated joint values: 4 wheels first, then arm/hand (17 total).
    # hold the arm/hand at its measured initial posture while the base drives;
    # continuous joints report 0/-1 limit sentinels and are skipped, but joints
    # spawned outside a real urdf limit fight the limit constraint and stall
    # the base, so they are clamped inside
    q_des = init_joint_angles.copy()
    for i in range(len(q_des)):
        if lower_limits[i] < upper_limits[i]:
            q_des[i] = np.clip(q_des[i], lower_limits[i], upper_limits[i])

    #simulation_time = sim.GetTimeSinceReset()
    time_step = sim.GetTimeStep()
    current_time = 0

    # gains proven in mobile_base_kinematic_controller
    kp_pos = 1.0  # position
    kp_ori = 10    # orientation

    # gains capped by discrete-time stability: kd * time_step / inertia < 2
    # on the lightest arm links (~5e-3 kg m^2); higher gains or any gain on
    # the 0.01 kg finger links (inertia ~1e-5) explode the sim and flip the
    # robot, so the 6 finger joints are left unactuated
    kp = 300
    kd = 15

    # summit_xl wheel geometry, same values as mobile_base_kinematic_controller
    wheel_radius = 0.11
    wheel_base_width = 0.46

    # Initialize data storage
    q_mes_all, qd_mes_all, q_d_all, qd_d_all,  = [], [], [], []
    base_pos_all, base_ori_all = [], []

    # drive the base to a nearby waypoint on the initial heading, like
    # mobile_base_kinematic_controller does, so the arm rides a moving base
    waypoints = [
        {'pos': np.array([0.5, 0.0, 0.0]), 'bearing': 0.0},
    ]

    current_waypoint_index = 0
    num_waypoints = len(waypoints)

    # data collection loop
    while True:
        # measure current state
        base_pos = sim.GetBasePosition()
        base_ori = sim.GetBaseOrientation()
        q_mes = sim.GetMotorAngles(0)
        qd_mes = sim.GetMotorVelocities(0)

        cmd = MotorCommands()
        if current_waypoint_index < num_waypoints:
            des_base_pos = waypoints[current_waypoint_index]['pos']
            des_base_bearing = waypoints[current_waypoint_index]['bearing']
            base_bearing_ = quaternion2bearing(base_ori[3], base_ori[0], base_ori[1], base_ori[2])

            left_wheel_velocity, right_wheel_velocity, at_goal = differential_drive_controller_adjusting_bearing(
                base_pos, base_bearing_, des_base_pos, des_base_bearing,
                wheel_radius, wheel_base_width, kp_pos, kp_ori
            )
            # active wheel order: front_right, front_left, back_left, back_right
            wheel_cmds = np.array([right_wheel_velocity, left_wheel_velocity, left_wheel_velocity, right_wheel_velocity])
            torque_joints = np.zeros(13)
            torque_joints[0:7] = kp * (q_des[4:11] - q_mes[4:11]) - kd * qd_mes[4:11]
            cmd_all = np.concatenate((wheel_cmds, torque_joints))
            cmd.SetControlCmd(cmd_all, ["velocity"] * 4 + ["torque"] * 13)

            if at_goal:
                print(f"Reached waypoint {current_waypoint_index + 1} at t={current_time:.2f}s")
                current_waypoint_index += 1
        else:
            print("Completed all waypoints. Base navigation finished.")
            break

        sim.Step(cmd, "torque")

        # Exit logic with 'q' key
        keys = sim.GetPyBulletClient().getKeyboardEvents()
        qKey = ord('q')
        if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
            break

        if max_steps and current_time / time_step >= max_steps:
            print(f"Reached step cap {max_steps} at t={current_time:.2f}s")
            break

        # Store data for plotting
        base_pos_all.append(base_pos)
        base_ori_all.append(base_ori)
        q_mes_all.append(q_mes)
        qd_mes_all.append(qd_mes)
        q_d_all.append(q_des)
        qd_d_all.append(np.zeros(num_joints))

        if np.linalg.norm(base_pos[:2] - des_base_pos[:2]) < 0.06:
            print(f"Reached the desired base position at t={current_time:.2f}s")
            break

        if not headless:
            time.sleep(0.01)  # Slow down the loop for better visualization
        current_time += time_step

    
    if headless:
        print(f"Headless run finished at t={current_time:.2f}s")
        return

    num_joints = len(q_mes)
    for i in range(num_joints):
        plt.figure(figsize=(10, 8))
        
        # Position plot for joint i
        plt.subplot(2, 1, 1)
        plt.plot([q[i] for q in q_mes_all], label=f'Measured Position - Joint {i+1}')
        plt.plot([q[i] for q in q_d_all], label=f'Desired Position - Joint {i+1}', linestyle='--')
        plt.title(f'Position Tracking for Joint {i+1}')
        plt.xlabel('Time steps')
        plt.ylabel('Position')
        plt.legend()

        # Velocity plot for joint i
        plt.subplot(2, 1, 2)
        plt.plot([qd[i] for qd in qd_mes_all], label=f'Measured Velocity - Joint {i+1}')
        plt.plot([qd[i] for qd in qd_d_all], label=f'Desired Velocity - Joint {i+1}', linestyle='--')
        plt.title(f'Velocity Tracking for Joint {i+1}')
        plt.xlabel('Time steps')
        plt.ylabel('Velocity')
        plt.legend()

        plt.tight_layout()
        plt.show()
    
   
     
    
    

if __name__ == '__main__':
    main()