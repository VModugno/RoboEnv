import numpy as np
import time
import os
import matplotlib.pyplot as plt
from simulation_and_control import pb, MotorCommands, PinWrapper, feedback_lin_ctrl, SinusoidalReference, CartesianDiffKin
from simulation_and_control import differential_drive_regulation_controller

def WholeBodyController(dyn_model,base_pos_,base_or_, q_, qd_, base_pos_d, base_or_d, q_d, qd_d, kp_pos, kp_ori, kp, kd):
    """
    Perform whole-body control on a robotic system.

    Parameters:
    - dyn_model (pin_wrapper): The dynamics model of the robot encapsulated within a 'pin_wrapper' object,
                               which provides methods for computing robot dynamics such as mass matrices,
                               Coriolis forces, etc.
    - q_ (numpy.ndarray): Measured positions of the robot's joints, indicating the current actual positions
                          as measured by sensors or estimated by observers.
    - qd_ (numpy.ndarray): Measured velocities of the robot's joints, reflecting the current actual velocities.
    - q_d (numpy.ndarray): Desired positions for the robot's joints set by a trajectory generator or a higher-level
                           controller, dictating target positions.
    - qd_d (numpy.ndarray): Desired velocities for the robot's joints, specifying the rate at which the joints
                            should move towards their target positions.
    - kp_pos (float): Proportional gain for position control, adjusting the response to position error.
    - kp_ori (float): Proportional gain for orientation control, adjusting the response to orientation error.
    - kp (float or numpy.ndarray): Proportional gain(s) for the control system, which can be a uniform value across
                                   all joints or unique for each joint, adjusting the response to position error.
    - kd (float or numpy.ndarray): Derivative gain(s), similar to kp, affecting the response to velocity error and
                                   aiding in system stabilization by damping oscillations.

    Returns:
    None

    This function computes the control inputs necessary to achieve desired joint positions and velocities by
    applying whole-body control, using the robot's dynamic model to appropriately compensate for its
    inherent dynamics. The control law implemented typically combines proportional-derivative (PD) control
    with dynamic compensation to achieve precise and stable motion.
    """
    # Command and control loop
    cmd = MotorCommands()  # Initialize command structure for motors
    wheel_radius = 0.1
    wheel_base_width = 0.5

    # i need to convert from quaternion to bearing (yaw)
    base_bearing_ = pb.QuaternionToEuler(base_or_)[2]
    base_bearing_d = pb.QuaternionToEuler(base_or_d)[2]

    # remove from the joint positions and velocity the wheels joints
    q_robot = q_[4:]
    qd_robot = qd_[4:]
    q_wheels = q_d[:4]
    qd_wheels = q_d[:4]


    angular_wheels_velocity = differential_drive_regulation_controller(base_pos_,base_bearing_,base_pos_d,base_bearing_d,wheel_radius,wheel_base_width, kp_pos, kp_ori)
    
    torque_joints = feedback_lin_ctrl(dyn_model, q_, qd_, q_d, qd_d, kp, kd)
    # all torques
    cmd_all = np.concatenate((angular_wheels_velocity,torque_joints))
    interface_all_wheels= ["velocity","velocity","velocity","velocity"]
    interface_all_joints = ["torque"]*13
    interface_all = interface_all_wheels + interface_all_joints
    cmd.setCommand(cmd_all,interface_all)
    return cmd
   

def main():
    # Configuration for the simulation
    conf_file_name = "robotnik_arm.json"  # Configuration file for the robot
    root_dir = os.path.dirname(os.path.abspath(__file__))
    # added this line to manage the fact that the file is in tests folder
    name_current_directory = "tests"
    # remove current directory name from cur_dir
    root_dir = root_dir.replace(name_current_directory, "")
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext = root_dir)  # Initialize simulation interface

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
    joint_des_angles = np.array([0.000, 0.000, 0.0000, 0.0000, 0.0, 1.57, 0.0, 1.0, 0.0, 1.0, 0.0,0.1,0.1,0.1,0.1,0.1,0.1])
    q_des = np.concatenate((des_base_pos, des_base_ori, joint_des_angles))
    des_base_lin_vel = np.array([0.0, 0.0, 0.0])
    des_base_ang_vel = np.array([0.0, 0.0, 0.0])
    qd_des_clip = np.concatenate((des_base_lin_vel, des_base_ang_vel, np.zeros(num_joints)))

    #simulation_time = sim.GetTimeSinceReset()
    time_step = sim.GetTimeStep()
    current_time = 0
    
    
    # P conttroller high level
    kp_pos = 100 # position 
    kp_ori = 0   # orientation
    
    # PD controller gains low level (feedbacklingain)
    kp = 1000
    kd = 100

    # Initialize data storage
    q_mes_all, qd_mes_all, q_d_all, qd_d_all,  = [], [], [], []
    base_pos_all, base_ori_all = [], []
    
    # data collection loop
    while True:
        # measure current state
        base_pos = sim.GetBasePosition()
        base_ori = sim.GetBaseOrientation()
        q_mes = sim.GetMotorAngles(0)
        qd_mes = sim.GetMotorVelocities(0)
        qdd_est = sim.ComputeMotorAccelerationTMinusOne(0)
        # Compute sinusoidal reference trajectory
        # Ensure q_init is within the range of the amplitude
        
        
        # Control command
        #
        sim.Step(cmd,"torque")  # Simulation step with torque command

        #if dyn_model.visualizer: 
        #    for index in range(len(sim.bot)): # Conditionally display the robot model
        #        q = sim.GetMotorAngles(index)
        #        dyn_model.DisplayModel(q)  # Update the display of the robot model

        # Exit logic with 'q' key
        keys = sim.GetPyBulletClient().getKeyboardEvents()
        qKey = ord('q')
        if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
            break
        
        #simulation_time = sim.GetTimeSinceReset()

        # Store data for plotting
        base_pos_all.append(base_pos)
        base_ori_all.append(base_ori)
        q_mes_all.append(q_mes)
        qd_mes_all.append(qd_mes)
        q_d_all.append(q_des)
        qd_d_all.append(qd_des_clip)
        #cur_regressor = dyn_model.ComputeDyanmicRegressor(q_mes,qd_mes, qdd_est)
        #regressor_all = np.vstack((regressor_all, cur_regressor))

        time.sleep(0.01)  # Slow down the loop for better visualization
        # get real time
        current_time += time_step
        print("current time in seconds",current_time)

    
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