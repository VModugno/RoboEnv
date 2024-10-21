import numpy as np
import time
import os
import matplotlib.pyplot as plt
from simulation_and_control import pb, MotorCommands, PinWrapper, feedback_lin_ctrl, SinusoidalReference, CartesianDiffKin, differential_drive_controller_adjusting_bearing
from simulation_and_control import differential_drive_regulation_controller,regulation_polar_coordinates,regulation_polar_coordinate_quat,wrap_angle
import pinocchio as pin

def quaternion2bearing(q_w, q_x, q_y, q_z):
    quat = pin.Quaternion(q_w, q_x, q_y, q_z)
    quat.normalize()  # Ensure the quaternion is normalized

    # Convert quaternion to rotation matrix
    rot_quat = quat.toRotationMatrix()

    # Convert rotation matrix to Euler angles (roll, pitch, yaw)
    base_euler = pin.rpy.matrixToRpy(rot_quat)  # Returns [roll, pitch, yaw]

    # Extract the yaw angle
    bearing_ = base_euler[2]

    return bearing_

def main():
    # Configuration for the simulation
    conf_file_name = "robotnik.json"  # Configuration file for the robot
    root_dir = os.path.dirname(os.path.abspath(__file__))
    # added this line to manage the fact that the file is in tests folder
    name_current_directory = "tests"
    # remove current directory name from cur_dir
    root_dir = root_dir.replace(name_current_directory, "")
    sim = pb.SimInterface(conf_file_name, conf_file_path_ext = root_dir)  # Initialize simulation interface
    # increse floor friction
    floor_friction = 1
    sim.SetFloorFriction(floor_friction)
    # Get active joint names from the simulation
    ext_names = sim.getNameActiveJoints()
    ext_names = np.expand_dims(np.array(ext_names), axis=0)  # Adjust the shape for compatibility

    source_names = ["pybullet"]  # Define the source for dynamic modeling

    # Create a dynamic model of the robot
    dyn_model = PinWrapper(conf_file_name, "pybullet", ext_names, source_names, False,0,root_dir)
    num_joints = dyn_model.getNumberofActuatedJoints()

    init_joint_angles = sim.GetInitMotorAngles()
    # print init joint
    print(f"Initial joint angles: {init_joint_angles}")
    
    # check joint limits
    lower_limits, upper_limits = sim.GetBotJointsLimit()
    print(f"Lower limits: {lower_limits}")
    print(f"Upper limits: {upper_limits}")


    joint_vel_limits = sim.GetBotJointsVelLimit()
    
    print(f"joint vel limits: {joint_vel_limits}")
    

    # fixed initial position
    #des_base_pos = np.array([0, 1, 0.0])
    
    #base_bearing_d = 90*(np.pi/180)S

    # Define waypoints as a list of dictionaries with position and orientation
    waypoints = [
        {'pos': np.array([1, 0, 0.0]), 'bearing': 0 * (np.pi / 180)},    # Move to (1, 0), heading 0°
        {'pos': np.array([1, 0, 0.0]), 'bearing': 90 * (np.pi / 180)},   # Rotate to 90°
        {'pos': np.array([1, 1, 0.0]), 'bearing': 90 * (np.pi / 180)},   # Move to (1, 1), heading 90°
        #{'pos': np.array([1, 1, 0.0]), 'bearing': 180 * (np.pi / 180)},  # Rotate to 180°
        #{'pos': np.array([0, 1, 0.0]), 'bearing': 180 * (np.pi / 180)},  # Move to (0, 1), heading 180°
        #{'pos': np.array([0, 1, 0.0]), 'bearing': -90 * (np.pi / 180)},  # Rotate to -90° (270°)
        #{'pos': np.array([0, 0, 0.0]), 'bearing': -90 * (np.pi / 180)},  # Move to (0, 0), heading -90°
        #{'pos': np.array([0, 0, 0.0]), 'bearing': 0 * (np.pi / 180)},    # Rotate back to 0°
    ]


    #simulation_time = sim.GetTimeSinceReset()
    time_step = sim.GetTimeStep()
    current_time = 0
    
    
    # P conttroller position orientationof the robot
    #kp_pos = 0.3 # position 
    #kp_ori = 1.43   # orientation
    kp_pos = 1.0 # position 
    kp_ori = 10   # orientation

    # Parameters
    k_rho = 0.5
    k_alpha = 4
    k_beta = 1
    # Initialize data storage
    base_pos_all, base_ori_all = [], []
    wheel_radius = 0.11
    wheel_base_width = 0.46
    # data collection loop
    # while True:
    #     # measure current state
    #     base_pos = sim.GetBasePosition()
    #     base_ori = sim.GetBaseOrientation()
    #     #q_mes = sim.GetMotorAngles(0)
    #     #qd_mes = sim.GetMotorVelocities(0)
    #     #qdd_est = sim.ComputeMotorAccelerationTMinusOne(0)
        
    #     cmd = MotorCommands()  # Initialize command structure for motors
    #     base_bearing_ = quaternion2bearing(base_ori[3], base_ori[0], base_ori[1], base_ori[2])
    #     #left_wheel_velocity, right_wheel_velocity
    #     #angular_wheels_velocity = differential_drive_regulation_controller(base_pos,base_bearing_,des_base_pos,base_bearing_d,wheel_radius,wheel_base_width, kp_pos, kp_ori)
    #     #angular_wheels_velocity = differential_drive_controller_adjusting_bearing(base_pos,base_bearing_,des_base_pos,base_bearing_d,wheel_radius,wheel_base_width, kp_pos, kp_ori)
    #     #angular_wheels_velocity = regulation_polar_coordinates(base_pos[0], base_pos[1], base_bearing_, des_base_pos[0], des_base_pos[1], base_bearing_d, wheel_radius,wheel_base_width, k_rho, k_alpha, k_beta)
    #     angular_wheels_velocity = regulation_polar_coordinate_quat(base_pos[0], base_pos[1], base_bearing_, des_base_pos[0], des_base_pos[1], base_bearing_d, wheel_radius,wheel_base_width, k_rho, k_alpha, k_beta)
    #     left_wheel_velocity = angular_wheels_velocity[0]
    #     right_wheel_velocity = angular_wheels_velocity[1]
    #     # in angular wheels velocity we have left and right wheel velocity
    #     # we need to extend it because i have two wheels per side
    #     angular_wheels_velocity = np.array([right_wheel_velocity,left_wheel_velocity,left_wheel_velocity,right_wheel_velocity])
    #     interface_all_wheels= ["velocity","velocity","velocity","velocity"]
    #     cmd.SetControlCmd(angular_wheels_velocity,interface_all_wheels)
        
    #     # Control command
    #     #
    #     sim.Step(cmd,"torque")  # Simulation step with torque command

    #     #if dyn_model.visualizer: 
    #     #    for index in range(len(sim.bot)): # Conditionally display the robot model
    #     #        q = sim.GetMotorAngles(index)
    #     #        dyn_model.DisplayModel(q)  # Update the display of the robot model

    #     # Exit logic with 'q' key
    #     keys = sim.GetPyBulletClient().getKeyboardEvents()
    #     qKey = ord('q')
    #     if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
    #         break
        
    #     #simulation_time = sim.GetTimeSinceReset()

    #     # Store data for plotting
    #     base_pos_all.append(base_pos)
    #     base_ori_all.append(base_ori)
       

    #     #time.sleep(0.01)  # Slow down the loop for better visualization
    #     # get real time
    #     current_time += time_step
    #     print("current time in seconds",current_time)

    current_waypoint_index = 0
    num_waypoints = len(waypoints)

    while True:
        # Measure current state
        base_pos = sim.GetBasePosition()
        base_ori = sim.GetBaseOrientation()
        base_bearing_ = quaternion2bearing(base_ori[3], base_ori[0], base_ori[1], base_ori[2])
        cmd = MotorCommands()  # Initialize command structure for motors
        # Check if all waypoints are completed
        if current_waypoint_index < num_waypoints:
            # Get the desired position and orientation from the current waypoint
            des_base_pos = waypoints[current_waypoint_index]['pos']
            base_bearing_d = waypoints[current_waypoint_index]['bearing']

            # Compute control commands using your control function
            #angular_wheels_velocity = regulation_polar_coordinates(base_pos[0], base_pos[1], base_bearing_, des_base_pos[0], des_base_pos[1], base_bearing_d, wheel_radius,wheel_base_width, k_rho, k_alpha, k_beta)
            angular_wheels_velocity = differential_drive_controller_adjusting_bearing(base_pos,base_bearing_,des_base_pos,base_bearing_d,wheel_radius,wheel_base_width, kp_pos, kp_ori)
            # Prepare control command
            left_wheel_velocity = angular_wheels_velocity[0]
            right_wheel_velocity = angular_wheels_velocity[1]
            #angular_wheels_velocity = differential_drive_regulation_controller(base_pos,base_bearing_,des_base_pos,base_bearing_d,wheel_radius,wheel_base_width, kp_pos, kp_ori)
            angular_wheels_velocity_cmd = np.array([right_wheel_velocity, left_wheel_velocity, left_wheel_velocity, right_wheel_velocity])
            interface_all_wheels = ["velocity", "velocity", "velocity", "velocity"]
            cmd.SetControlCmd(angular_wheels_velocity_cmd, interface_all_wheels)

            # Apply control command
            sim.Step(cmd, "torque")

            # Check if the robot has reached the current waypoint
            position_error = np.linalg.norm(base_pos[:2] - des_base_pos[:2])
            orientation_error = abs(wrap_angle(base_bearing_d - base_bearing_))

            # Define tolerances for position and orientation
            position_tolerance = 0.06  # Meters
            orientation_tolerance = 15 * (np.pi / 180)  # Radians (5 degrees)

            # If the robot is within the tolerances, proceed to the next waypoint
            if position_error < position_tolerance and orientation_error < orientation_tolerance:
                print(f"Reached waypoint {current_waypoint_index + 1}")
                current_waypoint_index += 1
                # Optional: Add a short delay or pause if needed
        else:
            # All waypoints completed
            print("Completed all waypoints. Square path traversal finished.")
            break

        # Rest of your loop code (e.g., data storage, visualization)

        # Exit logic with 'q' key (unchanged)
        keys = sim.GetPyBulletClient().getKeyboardEvents()
        qKey = ord('q')
        if qKey in keys and keys[qKey] and sim.GetPyBulletClient().KEY_WAS_TRIGGERED:
            break

        # Store data for plotting if necessary
        base_pos_all.append(base_pos)
        base_ori_all.append(base_ori)

        # Update current time
        current_time += time_step


    
    
    
   
     
    
    

if __name__ == '__main__':
    main()