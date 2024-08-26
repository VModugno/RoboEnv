import numpy as np


class SinusoidalReference:
    def __init__(self, amplitude, frequency, q_init):
        self.amplitude = np.array(amplitude)
        self.frequency = np.array(frequency)
        self.q_init = np.array(q_init)

         # Check if all arrays have the same length
        if not (self.amplitude.size == self.frequency.size == self.q_init.size):
            expected_num_elements = self.q_init.size
            raise ValueError(f"All arrays must have the same number of elements. "
                             f"Expected number of elements (joints): {expected_num_elements}, "
                             f"Received - Amplitude: {self.amplitude.size}, "
                             f"Frequency: {self.frequency.size}, Q_init: {expected_num_elements}.")

        self.phase = np.full(self.q_init.shape, np.pi / 2) #self.calculate_initial_phase()

    #def calculate_initial_phase(self):
    #    # Check if initial position is within the amplitude range
    #    if (np.all(abs(self.q_init) <= self.amplitude)):
    #        return np.arcsin(self.q_init / self.amplitude)
    #    else:
    #         # Identify and report joints where the initial position exceeds the amplitude
    #        for i, (init, amp) in enumerate(zip(self.q_init, self.amplitude)):
    #            if abs(init) > amp:
    #                raise ValueError(f"Initial position of joint {i+1} exceeds the amplitude range. Init: {init}, Amp: {amp}")
            

    def check_sinusoidal_feasibility(self, sim):
        """
        Check if the sinusoidal motion with the given amplitude is feasible within the joint limits.

        Args:
        amplitude (float or list of floats): The amplitude(s) of the sinusoidal motion.
        sim (object): Simulation object that provides joint limits and initial joint angles.

        Returns:
        bool: True if sinusoidal motion is feasible for all joints, False otherwise.
        """
        # Retrieve joint limits and initial angles from the simulation object
        lower_limits, upper_limits = sim.GetBotJointsLimit()
        velocity_limits = sim.GetBotJointsVelLimit()  # Retrieve velocity limits for all joints
        init_angles = sim.GetInitMotorAngles()
        
        # Iterate over each joint to check the feasibility of the given amplitude
        for i, init_angle in enumerate(init_angles):
            min_angle = init_angle - self.amplitude[i]
            max_angle = init_angle + self.amplitude[i]
            max_velocity_required = self.amplitude[i] * self.frequency[i] * 2 * np.pi
            
            if min_angle < lower_limits[i] or max_angle > upper_limits[i]:
                print(f"Joint {i+1}: Sinusoidal motion not possible. Limits exceeded.")
                print(f"    Expected range: {min_angle} to {max_angle}")
                print(f"    Actual limits: {lower_limits[i]} to {upper_limits[i]}")
                return False
            # Check if the maximum velocity is within limits
            if max_velocity_required > velocity_limits[i]:
                print(f"Joint {i+1}: Sinusoidal motion velocity too high. Maximum velocity exceeded.")
                print(f"    Required velocity: {max_velocity_required}")
                print(f"    Velocity limit: {velocity_limits[i]}")
                return False

        print("Sinusoidal motion is possible within the joint limits for all joints.")
        return True

    def get_values(self, time):
        """
        Calculate the position and velocity at a given time.

        Parameters:
        time (float or np.array): The time at which to evaluate the position and velocity.

        Returns:
        tuple: The position and velocity at the given time.
        """
        #q_d = self.amplitude * np.sin(2 * np.pi * self.frequency * time + self.phase)
        #qd_d = self.amplitude * 2 * np.pi * self.frequency * np.cos(2 * np.pi * self.frequency * time + self.phase)
        # Calculate the sinusoidal position around the initial position
        q_d = self.q_init + self.amplitude * np.sin(2 * np.pi * self.frequency * time + self.phase)
        # Calculate the derivative of the position (velocity)
        qd_d = self.amplitude * 2 * np.pi * self.frequency * np.cos(2 * np.pi * self.frequency * time + self.phase)
        return q_d, qd_d