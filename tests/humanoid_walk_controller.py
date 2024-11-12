import numpy as np
import time
import os
import matplotlib.pyplot as plt
from simulation_and_control import pb, MotorCommands, PinWrapper


def main():

    #init_joint_position=[0., 0., 0., 0.,  0.,  -3.,  -25.,  50., -25.,   3.,  0.,  3., -25.,  50.,  -25.,  -3., 4.,  -8.,  0.,  -25., 4.,  8.,  0.,  -25.]
    #convert_init_joint_position = [x * np.pi / 180. for x in init_joint_position]
    #print("init_joint_position", convert_init_joint_position)
    #print("len of joint_position=",len(init_joint_position))

    conf_file_name = "hrp4config.json"  # Configuration file for the robot
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

    

    
    # while True:
    #     
    #     base_pos = sim.GetBasePosition()
    #     base_ori = sim.GetBaseOrientation()
    #     q_mes = sim.GetMotorAngles(0)
    #     qd_mes = sim.GetMotorVelocities(0)
    #     # create current and desired states
    #     self.current = self.retrieve_state()
    #
    #      
        
    #     # set acceleration commands
    #     for i in range(self.params['dof'] - 6):
    #         self.hrp4.setCommand(i + 6, commands[i])

    #     # log and plot
    #     self.logger.log_data(self.current, self.desired)
    #     #self.logger.update_plot()

    #     self.time += 1

    # def retrieve_state(self):
    #     # com and torso pose (orientation and position)
    #     com_position = self.hrp4.getCOM()
    #     torso_orientation = get_rotvec(self.hrp4.getBodyNode('torso').getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).rotation())
    #     base_orientation  = get_rotvec(self.hrp4.getBodyNode('body' ).getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World()).rotation())

    #     # feet poses (orientation and position)
    #     l_foot_transform = self.lsole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
    #     l_foot_orientation = get_rotvec(l_foot_transform.rotation())
    #     l_foot_position = l_foot_transform.translation()
    #     left_foot_pose = np.hstack((l_foot_orientation, l_foot_position))
    #     r_foot_transform = self.rsole.getTransform(withRespectTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
    #     r_foot_orientation = get_rotvec(r_foot_transform.rotation())
    #     r_foot_position = r_foot_transform.translation()
    #     right_foot_pose = np.hstack((r_foot_orientation, r_foot_position))

    #     # velocities
    #     com_velocity = self.hrp4.getCOMLinearVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
    #     torso_angular_velocity = self.hrp4.getBodyNode('torso').getAngularVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
    #     base_angular_velocity = self.hrp4.getBodyNode('body').getAngularVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
    #     l_foot_spatial_velocity = self.lsole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())
    #     r_foot_spatial_velocity = self.rsole.getSpatialVelocity(relativeTo=dart.dynamics.Frame.World(), inCoordinatesOf=dart.dynamics.Frame.World())

    #     # compute total contact force
    #     force = np.zeros(3)
    #     for contact in world.getLastCollisionResult().getContacts():
    #         force += contact.force

    #     # compute zmp
    #     zmp = np.zeros(3)
    #     zmp[2] = com_position[2] - force[2] / (self.hrp4.getMass() * self.params['g'] / self.params['h'])
    #     for contact in world.getLastCollisionResult().getContacts():
    #         if contact.force[2] <= 0.1: continue
    #         zmp[0] += (contact.point[0] * contact.force[2] / force[2] + (zmp[2] - contact.point[2]) * contact.force[0] / force[2])
    #         zmp[1] += (contact.point[1] * contact.force[2] / force[2] + (zmp[2] - contact.point[2]) * contact.force[1] / force[2])

    #     if force[2] <= 0.1: # threshold for when we lose contact
    #         zmp = np.array([0., 0., 0.]) # FIXME: this should return previous measurement
    #     else:
    #         # sometimes we get contact points that dont make sense, so we clip the ZMP close to the robot
    #         midpoint = (l_foot_position + l_foot_position) / 2.
    #         zmp[0] = np.clip(zmp[0], midpoint[0] - 0.3, midpoint[0] + 0.3)
    #         zmp[1] = np.clip(zmp[1], midpoint[1] - 0.3, midpoint[1] + 0.3)
    #         zmp[2] = np.clip(zmp[2], midpoint[2] - 0.3, midpoint[2] + 0.3)

    #     # create state dict
    #     return {
    #         'lsole': {'pos': left_foot_pose,
    #                   'vel': l_foot_spatial_velocity,
    #                   'acc': np.zeros(6)},
    #         'rsole': {'pos': right_foot_pose,
    #                   'vel': r_foot_spatial_velocity,
    #                   'acc': np.zeros(6)},
    #         'com'  : {'pos': com_position,
    #                   'vel': com_velocity,
    #                   'acc': np.zeros(3)},
    #         'torso': {'pos': torso_orientation,
    #                   'vel': torso_angular_velocity,
    #                   'acc': np.zeros(3)},
    #         'base' : {'pos': base_orientation,
    #                   'vel': base_angular_velocity,
    #                   'acc': np.zeros(3)},
    #         'joint': {'pos': self.hrp4.getPositions(),
    #                   'vel': self.hrp4.getVelocities(),
    #                   'acc': np.zeros(self.params['dof'])},
    #         'zmp'  : {'pos': zmp,
    #                   'vel': np.zeros(3),
    #                   'acc': np.zeros(3)}
    #     }

if __name__ == '__main__':
    main()