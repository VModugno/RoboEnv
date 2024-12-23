import numpy as np
import time
import os
import matplotlib.pyplot as plt
from simulation_and_control import pb, MotorCommands, PinWrapper, feedback_lin_ctrl, SinusoidalReference, ImpedanceController


def main():

    conf_file_name = "pandaconfig.json"  # Configuration file for the robot
    root_dir = os.path.dirname(os.path.abspath(__file__))
    # added this line to manage the fact that the file is in tests folder
    name_current_directory = "tests"
    # remove current directory name from cur_dir
    root_dir = root_dir.replace(name_current_directory, "")
    # Configuration for the simulation
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext = root_dir)  # Initialize simulation interface

    # Get active joint names from the simulation
    ext_names = sim.getNameActiveJoints()
    ext_names = np.expand_dims(np.array(ext_names), axis=0)  # Adjust the shape for compatibility

    source_names = ["pybullet"]  # Define the source for dynamic modeling

    # Create a dynamic model of the robot
    dyn_model = PinWrapper(conf_file_name, "pybullet", ext_names, source_names, False,0,root_dir)
    num_joints = dyn_model.getNumberofActuatedJoints()

    controlled_frame_name = "panda_link8"
    init_joint_angles = sim.GetInitMotorAngles()
    init_cartesian_pos,init_R = dyn_model.ComputeFK(init_joint_angles,controlled_frame_name)
    # print init joint
    print(f"Initial joint angles: {init_joint_angles}")
    
    # check joint limits
    lower_limits, upper_limits = sim.GetBotJointsLimit()
    print(f"Lower limits: {lower_limits}")
    print(f"Upper limits: {upper_limits}")


    joint_vel_limits = sim.GetBotJointsVelLimit()
    
    print(f"joint vel limits: {joint_vel_limits}")
    
    # for regulation
    q_des =  init_joint_angles
    qd_des_clip = np.zeros(num_joints)
    
    # Sinusoidal reference for cartesian trajectory tracking
    # Specify different amplitude values for each joint
    amplitudes = [0, 0.1, 0]  # Example amplitudes for 4 joints
    # Specify different frequency values for each joint
    frequencies = [0.4, 0.5, 0.4]  # Example frequencies for 4 joints

    # Convert lists to NumPy arrays for easier manipulation in computations
    amplitude = np.array(amplitudes)
    frequency = np.array(frequencies)
    ref = SinusoidalReference(amplitude, frequency,init_cartesian_pos)  # Initialize the reference
    
    #check = ref.check_sinusoidal_feasibility(sim)  # Check the feasibility of the reference trajectory
    #if not check:
    #    raise ValueError("Sinusoidal reference trajectory is not feasible. Please adjust the amplitude or frequency.")
    
    #simulation_time = sim.GetTimeSinceReset()
    time_step = sim.GetTimeStep()
    current_time = 0
    # Command and control loop
    cmd = MotorCommands()  # Initialize command structure for motors
    
    # P conttroller high level
    kp_pos = 100 # position 
    kp_ori = 0   # orientation
    
    # PD controller gains low level (feedbacklingain)
    kp = 1000
    kd = 100

    # Initialize data storage
    q_mes_all, qd_mes_all, q_d_all, qd_d_all,  = [], [], [], []
    
    controlled_frame_name = "panda_link8"
    # external forces (the wrench applied to the end effector) for now assumed zero because no contact with the environment
    tau_ext = np.zeros(6)
    # data collection loop
    while True:
        # measure current state
        q_mes = sim.GetMotorAngles(0)
        qd_mes = sim.GetMotorVelocities(0)
        qdd_est = sim.ComputeMotorAccelerationTMinusOne(0)
        # Compute sinusoidal reference trajectory
        # Ensure q_init is within the range of the amplitude

       
        
        p_d, pd_d = ref.get_values(current_time)  # Desired position and velocity


        # Control command
        tau_cmd = ImpedanceController(dyn_model,controlled_frame_name, q_mes, qd_mes, p_d, kp, kd, tau_ext)  # Zero torque command
        cmd.SetControlCmd(tau_cmd, ["torque"]*7)  # Set the torque command
        sim.Step(cmd, "torque")  # Simulation step with torque command

        if dyn_model.visualizer: 
            for index in range(len(sim.bot)): # Conditionally display the robot model
                q = sim.GetMotorAngles(index)
                dyn_model.DisplayModel(q)  # Update the display of the robot model

        # Exit logic with 'q' key
        keys = sim.GetPyBulletClient().getKeyboardEvents()
        qKey = ord('q')
        if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
            break
        
        #simulation_time = sim.GetTimeSinceReset()

        # Store data for plotting
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