# RoboEnv developer reference

This document explains the classes and functions in the `simulation_and_control` Python package included in RoboEnv. It describes the current source: how simulation state flows through the controllers, what inputs and outputs mean, and which interfaces are incomplete. It contains no installation instructions.

RoboEnv provides robot configurations, models, and examples around this package. The main programming interface combines a PyBullet simulation (`SimInterface`), a Pinocchio model (`PinWrapper`), and controller functions that turn measured state and references into motor commands. The humanoid and quadruped directories contain additional control implementations with separate assumptions and incomplete integration.

## Contents

- [Package structure](#package-structure)
- [State conventions](#state-conventions)
- [Simulation interface](#simulation-interface)
- [Motor commands and motor model](#motor-commands-and-motor-model)
- [Kinematics and dynamics model](#kinematics-and-dynamics-model)
- [Manipulator controllers](#manipulator-controllers)
- [Mobile base controllers](#mobile-base-controllers)
- [Reference and configuration utilities](#reference-and-configuration-utilities)
- [Humanoid controllers](#humanoid-controllers)
- [Quadruped controllers](#quadruped-controllers)
- [Control loop example](#control-loop-example)
- [Example scripts](#example-scripts)
- [Complete signature index](#complete-signature-index)

## Package structure

Source paths below are relative to [the Python package directory](simulation_and_control/simulation_and_control).

| Module | Responsibility |
| --- | --- |
| `sim/pybullet_robot_interface.py` | Own the physics client, load robots, apply actions, and expose observations and simulation diagnostics. |
| `controllers/servo_motor.py` | Describe each motor's command mode and convert commands into torques. |
| `controllers/pin_wrapper.py` | Load the analytical robot model, reconcile joint order, and compute kinematics and dynamics. |
| `controllers/CartesianKinematic.py` | Convert Cartesian targets into joint position and velocity references. |
| `controllers/FeedbackLin.py`, `DynamicCancellation.py`, `ImpedanceCtrl.py` | Compute joint torques using the analytical model. |
| `controllers/MobileBaseController.py` | Convert planar navigation errors into left and right wheel speeds. |
| `utils/` | Generate sinusoidal references and normalize configuration values. |
| `controllers/humanoid_controller/` | Plan footsteps and center of mass motion, estimate state, and solve acceleration tasks for a DART humanoid. |
| `controllers/QuadController.py`, `controllers/quadruped_controller/` | Coordinate gait timing, footholds, centroidal MPC, and stance and swing torques. |

[The package initializer](simulation_and_control/simulation_and_control/__init__.py) exports `pb` as an alias for the PyBullet interface module, plus `MotorCommands`, `PinWrapper`, `feedback_lin_ctrl`, `dyn_cancel`, `ImpedanceController`, `CartesianDiffKin`, `applyJointVelSaturation`, `apply_dead_zone`, the main mobile base functions, `SinusoidalReference`, and `adjust_value`. `ImpedanceController` is a function despite its capitalized name. `ServoMotorModel`, the result container, and the humanoid and quadruped classes are not re-exported at the package root.

The usual control flow is:

```text
SimInterface observations
    -> reference generator and controller using PinWrapper
    -> MotorCommands
    -> ServoMotorModel computes torque
    -> PyBullet advances one step
    -> SimInterface refreshes observations
```

The simulator and analytical model are independent objects. Changing a mass or inertia through `SimInterface` does not update `PinWrapper` automatically.

## State conventions

Use meters, seconds, radians, kilograms, newtons, and newton meters, as appropriate to the joint or Cartesian quantity. A prismatic joint uses linear position and velocity; a revolute joint uses angular position and velocity.

| Quantity | Representation |
| --- | --- |
| Actuated state | One value per active simulator joint, in `sim.getNameActiveJoints(index)` order. |
| Fixed base configuration and velocity | Joint configuration `q` and joint velocity `v`. Their lengths need not match for continuous joints in Pinocchio. |
| Floating base configuration | `[px, py, pz, qx, qy, qz, qw, joint_configuration...]`. The base contributes seven configuration entries. |
| Floating base velocity | `[vx, vy, vz, wx, wy, wz, joint_velocity...]`. The base contributes six velocity entries. Pinocchio expects those base velocities in the body frame. |
| Pinocchio Jacobian | Six rows ordered as linear velocity followed by angular velocity; columns follow Pinocchio velocity order. |
| `PinWrapper.ComputeFK` result | Position vector of length three and a `3 × 3` rotation matrix. |
| PyBullet orientation | Quaternion in `[x, y, z, w]` order. |
| Mobile base quaternion helpers | Quaternion in `[w, x, y, z]` order. Their navigation functions still accept scalar yaw angles. |

For a floating robot, use `sim.GetSystemState(base_vel_base_frame=True)` when constructing state for Pinocchio. For a fixed robot, explicitly use `fixed_base=True`: `GetSystemState` does not infer this from the robot configuration.

Joint names determine the permutation between simulator order and Pinocchio order. `PinWrapper` accepts external state and reorders its joint part internally. Most analytical results remain in Pinocchio order. Always check the specific controller or method before passing a returned vector to the simulator.

Continuous joints require particular care: Pinocchio can represent an angle with two configuration values, `[cos(theta), sin(theta)]`, but only one velocity value. The current name-based reordering does not fully implement that expansion.

## Simulation interface

Source: [pybullet_robot_interface.py](simulation_and_control/simulation_and_control/sim/pybullet_robot_interface.py).

### SimInterface

```python
pb.SimInterface(conf_file_name, conf_file_path_ext=None, use_gui=True)
```

`SimInterface` owns a PyBullet client and a list of robots in `sim.bot`. Construction reads a JSON configuration, sets gravity and the physics time step, loads a ground plane and any configured scene, constructs one `SimRobot` per robot entry, restores the initial poses, and collects the initial observation. `use_gui=False` selects PyBullet DIRECT mode.

`conf_file_path_ext` is the directory containing `configs/` and `models/`. Both `SimInterface` and `PinWrapper` read their configurations with `json.load`; a filename ending in another extension does not enable YAML parsing. Use the same asset root and configuration for both objects.

Configuration sections explain the division of responsibility:

| Section | Used for |
| --- | --- |
| `sim` | Time step and names of potential foot contact frames. `experiment_duration` does not stop `Step` automatically. |
| `robot_pybullet` | Robot models, initial state, base type, motor gains and direction, contact properties, measurement noise, and delay. Robot-specific entries are generally lists indexed by robot number. |
| `env_pybullet` | Optional scene script name under `models/scenes/`. `LoadEnv` executes its contents as Python with access to `self`, regardless of the filename extension. |
| `robot_pin` | Analytical model, base type, joint reordering, and optional named joint control groups. |

#### Simulation lifecycle

| Method | Behavior |
| --- | --- |
| `Step(action, unseless_control_mode=None)` | Apply the action, advance physics once, refresh observations, increment `step_counter`, and save `last_action`. Returns `None`. The second argument is retained for compatibility and is ignored. |
| `ApplyAction(cmd)` | Compute and send torques without advancing physics. A single robot takes one `MotorCommands`; multiple robots take an indexable collection with one command per robot. |
| `ReceiveObservation()` | Refresh cached joint and base state, previous state, each robot's delay buffer, and observation history. Call it after direct client changes when you need fresh cached state. |
| `ResetPose()` | Restore configured motor and base poses and configured velocities, and disable default PyBullet joint motors. It does not reset the step counter or clear observation history and delay buffers. |
| `LoadEnv(env_script_name)` | Execute a scene file in a shared namespace containing `self`; scene code can use `self.pybullet_client` and `self.env`. |
| `GetPyBulletClient()` | Return the underlying client for keyboard events, contacts, additional bodies, or `disconnect()`. |
| `GetTimeStep()` | Return the stored time step in seconds. |
| `GetTimeSinceReset()` | Return `step_counter * time_step`. Since `ResetPose` does not reset the counter, this is elapsed tracked simulation time, not necessarily time since the latest pose reset. |
| `SetTimeSteps(simulation_step)` | Change the wrapper's stored time step only. It does not call the client's `setTimeStep`. |

Observations are cached when `ReceiveObservation` runs. Many public `Get...` methods select delayed state and add fresh noise on each call; repeated reads can therefore differ without a physics step. Configuration keys named `*_cov` are passed as the standard deviation to `np.random.normal` in those getters.

#### Joint and base observations

Methods with an `index` argument select a robot in `sim.bot`, not a PyBullet body ID. Most default to zero, but `GetMotorAngles`, `GetMotorVelocities`, and the motor acceleration methods require an explicit index.

| Methods | Output and meaning |
| --- | --- |
| `GetMotorAngles(index)`, `GetMotorVelocities(index)` | Measured active-joint vectors with configured delay and noise. Angles account for motor offsets and directions; velocities account for directions. |
| `ExtractMotorAngles(index)`, `ExtractMotorVelocities(index)` | Query current PyBullet joints directly and apply the same offset/direction conventions, without measurement delay or added noise. |
| `ComputeMotorAccelerationTMinusOne(index)` | Finite difference of current extracted velocity and cached previous velocity, divided by the time step. |
| `GetMotorAccelerationTMinusOne(index)` | Intended measured acceleration getter; currently uses field names inconsistent with `ReceiveObservation`, so it can raise a key or attribute error. |
| `GetMotorTorques(index=0)` | Last commanded motor torques after direction adjustment, with optional noise. This is not a measured contact wrench or an independent joint torque sensor. |
| `GetBasePosition`, `GetBaseOrientation` | Cached PyBullet base inertial-frame pose, exposed with configured measurement effects. Orientation is a quaternion. |
| `GetBaseLinVelocity`, `GetBaseAngVelocity` | Measured base velocities in world axes. |
| `GetBaseLinVelocityBodyFrame`, `GetBaseAngVelocityBodyFrame` | Corresponding measured velocities in body axes. |
| `ComputeBaseVelocitiesBodyFrame`, `GetBaseVelocitiesBodyFrame` | Return `(linear_velocity, angular_velocity)` in body axes; the first computes from internal state, the second uses measured getters. |
| `ComputeBaseLinAccelerationTMinusOne`, `ComputeBaseAngAccelerationTMinusOne` | Finite differences of cached world-frame velocities. |
| `ComputeBaseLinAccelerationBodyFrameTMinusOne`, `ComputeBaseAngAccelerationBodyFrameTMinusOne` | Finite differences using body-frame velocity state. |
| `ComputeSystemStateAccelerationTMinusOne(base_frame=True, index=0)` | Joint accelerations for a fixed base, or base linear/angular acceleration followed by joint acceleration for a floating base. |
| `ComputePdot(index=0)` | Estimate base linear velocity by finite-differencing current and previous cached base positions. |
| `ComputeBaseRollPitchYaw(index=0)` | Convert internal base orientation to a roll, pitch, yaw vector. |
| `GetGravVecBodyFrame(index=0)` | Rotate the unit downward gravity direction into body axes; this is a direction, not gravitational force. |

#### Combined state and poses

| Method | Return value |
| --- | --- |
| `GetSystemState(fixed_base=False, base_vel_base_frame=False, index=0)` | `(configuration, velocity)` assembled from measured getters. |
| `GetSystemStateInternal(...)` | Same representation, assembled from current internal state without measurement delay or noise. |
| `GetSystemPreviousStateInternal(...)` | Same representation from the cached preceding observation. |
| `GetAllObservation()` | One flat observation list per robot: joint angles, joint velocities, motor torques, base position, quaternion, world linear/angular velocities, then body linear/angular velocities. |
| `GetAllObservationIdeal()` | Same ordering using internal pose and velocity state. It still calls `GetMotorTorques`, which can add noise. |
| `GetLinkPositionAndOrientation(link_name, joint_or_com, index=0)` | World position and quaternion of the link frame for `"joint"`, or link center of mass frame for `"com"`. Missing names print an error and return two empty lists. |
| `GetFloatingBaseLinkPositionAndOrientation(index=0)` | Intended URDF base link pose; fixed bases return empty lists. Its internal link query currently omits the supplied robot index. |
| `get_pose(bot_index, target_frame="panda_link8")` | Link center of mass position and Euler orientation from PyBullet; unlike `ComputeFK`, orientation is not a rotation matrix. |
| `calc_inverse_kinematics(bot_index, target_position, euler_angles_radians=None, target_frame=None)` | Joint positions from PyBullet inverse kinematics for a world target, using joint limits. The default frame is `panda_link8`; this computes a target without applying it. |
| `GetConfInitPosition(index)`, `GetConfInitOrientation(index)` | Configured initial base link pose. |
| `GetInitMotorAngles(index=0)`, `GetInitMotorVelocities(index=0)` | Configured initial active-joint state. |

For frame comparisons, distinguish the URDF link frame used by `ComputeFK` from the inertial center of mass frame returned by PyBullet base queries or `get_pose`.

#### Model properties and direct state changes

| Methods | Behavior |
| --- | --- |
| `getNameActiveJoints(index=0)`, `GetActionDimension(index=0)` | Active joint names and number of commanded motors. |
| `GetBotJointsLimit`, `GetBotJointsVelLimit`, `GetBotJointsTorqueLimit` | Lists of URDF position bounds, velocity limits, and effort limits for non-fixed joints. Reading a limit does not enforce it. Continuous joints can have sentinel bounds with lower bound greater than upper bound. |
| `GetJointInfo(body_id, joint_id)` | PyBullet joint information converted to a list with decoded names and NumPy vectors. |
| `getDynamicsInfo(body_id, link_id=-1)` | PyBullet dynamics information with inertia and inertial-frame vectors converted to arrays. |
| `GetBotJointsInfo`, `GetBotDynamicsInfo` | Print information for the robot; return `None`. |
| `GetMassLink`, `GetInertiaLink` | Link mass and local principal inertia diagonal. |
| `GetTotalMassFromUrdf` | Sum and print masses for link indices `0..num_joints-1`; the base at index `-1` is excluded. |
| `SetMassLink`, `SetDiffMassLink`, `SetInertiaLink` | Change a link's PyBullet mass or inertia. The differential mass setter refuses a nonpositive resulting mass. |
| `SetMotorGains(kp, kd, index=0)`, `GetMotorGains(index=0)` | Set or retrieve motor model PD gains. |
| `SetjointPosition(position, index=0)` | Reset active joints directly, bypassing physics control. |
| `SetfloatingBasePositionAndOrientation(position, orientation, index=0)` | Reset the PyBullet base inertial-frame pose directly. |
| `KinematicVisualizer(q_res, dyn_model, visual_delays=0)` | Interactive visualization of a prescribed configuration using direct state resets. |

#### Dynamics, contact, and diagnostic methods

`ComputeMassMatrix(previous_state=False, index=0)` returns a PyBullet inertia matrix. For floating bases it removes the extra quaternion-related row and column returned by the implementation's `flags=1` calculation. `ComputeSplitMassMatrix(M_big)` returns `(M_bb, M_qq, M_bq, M_qb)` with six base velocity coordinates; use the slices rather than inferring their meaning from the local variable names.

The other dynamics methods are `ComputeMassMatrixRNEA`, `ComputeCoriolisAndGravityForces`, `ComputeGravity`, `ComputeCoriolis`, and `DirectDynamicsActuatedNoContact`. They aim to reconstruct inertia from inverse dynamics, obtain bias terms, and compute unforced joint acceleration. Several branches retain obsolete `self.bot` access instead of `self.bot[index]`, undefined variables, or incorrect argument forwarding. In particular, `ComputeCoriolisAndGravityForces` uses an undefined acceleration vector in its fixed-base branch and an invalid list-plus-integer expression in its floating-base branch. `DirectDynamicsActuatedNoContact` accepts `tau` but does not use it in its acceleration expression. Use `PinWrapper` for the analytical calculations described below, and inspect these simulator routines before relying on them.

`GetFootContacts`, `GetFootLinkIDs`, `getFeetGRFLocal`, `GetFootGRFLocal`, `getFeetGRFWolrd`, `GetFootGRFWolrd`, and `ComputeFootGRF` describe intended contact flags and ground reaction force (GRF) access. These retain the old single-robot object layout and are incomplete with the current `sim.bot` list. Preserve the spelling `Wolrd` when looking up those methods; it is the current API spelling.

`SetFootFriction`, `SetFootRestitution`, and `SetFloorFriction` change contact parameters. `GetFootFriction` and `GetFootRestitution` print their values. `SetJointFriction` intends to simulate joint friction through zero-velocity motor commands with a force limit, but retains obsolete `self.bot` and `self.p` access and is incomplete. `GetNumKneeJoints` actually returns the number of registered foot links.

`DynamicSanityCheck1`, `DynamicSanityCheck2`, and `DynamicSanityCheck3` print comparisons between PyBullet and Pinocchio dynamics. `KinematicSanityCheck` prints checks of base kinematics. They are diagnostic routines, not assertions or controller outputs.

`SkewSymmetric(vector)` returns the matrix implementing a cross product. `TransformWorld2Body` and `TransformBody2World` rotate vectors using cached base orientation without translating positions. `TransformAngularVelocityToLocalFrame(angular_velocity, orientation)` performs the inverse orientation rotation. `quaternion_multiply(Q1, Q2)` uses `[x, y, z, w]`; it is defined without `self` or `@staticmethod`, so call it on the class rather than an instance.

### SimRobot and module helpers

`SimRobot(pybullet_client, conf_file_json, index, config_file_path_ext=None)` is the per-robot object constructed by `SimInterface`. It stores the PyBullet body ID, active joint IDs, joint/link name mappings, foot link and sensor IDs, configured initial state, motor model, measurement options, and state buffer.

Its public methods are `get_link_id_from_name(link_name)`, `get_pybullet_bot_index()`, `getNameActiveJoints(pybullet_client)`, `SetFootFriction(pybullet_client, foot_friction)`, and `SetFootRestitution(pybullet_client, foot_restitution)`. Most application code uses the corresponding `SimInterface` methods instead.

The private methods resolve and load the URDF (`_UrdfPath`, `_LoadPybulletURDF`), create joint and link mappings (`_BuildJointNameToIdAndActiveJoint`, `_buildLinkNameToId`, `_GetLinkIdByName`), remove default damping/friction (`_RemoveDefaultJointDamping`, `_RemoveURDFJointDampingAndFriction`), register foot sensors (`_BuildFeetJointIDAndForceSensors`), and configure rack constraints and initial transforms (`_CreateRackConstraint`, `_GetDefaultInitPosition`, `_GetDefaultInitOrientation`).

`EmptyObj` is an empty attribute container used for `sim.env`. `MapToMinusPiToPi(angles)` copies and wraps an angle sequence into `[-pi, pi)`.

`SimInterface` uses `_StepInternal` for action application, stepping, and observation collection; `_SetMotorTorqueById` and `_SetMotorTorqueByIds` send torques; `_SetDesiredMotorAngleByName` is a low-level position helper; `_AddSensorNoise` adds Gaussian noise. These are implementation helpers rather than the normal control-loop entry points.

## Motor commands and motor model

Source: [servo_motor.py](simulation_and_control/simulation_and_control/controllers/servo_motor.py).

### MotorCommands

```python
MotorCommands(ctrl_value=np.array([]), control_list=[])
cmd.SetControlCmd(ctrl_value, control_list)
```

`MotorCommands` contains `ctrl_cmd`, the values, and `control_list`, one mode string per motor. Both construction and `SetControlCmd` broadcast a scalar or single-element NumPy array to the number of modes. Other values must have matching outer length or a `ValueError` is raised. It does not validate every mode's inner shape.

| Mode | `ctrl_cmd` shape | Interpretation |
| --- | --- | --- |
| `"torque"` | `(n,)` for a uniform command | One torque per motor. For a matrix row, only its first flattened value is used. |
| `"velocity"` | `(n,)` for a uniform command | One desired velocity per motor; derivative feedback produces torque. |
| `"position"` | `(n, 2)` | Each row is `[desired_position, desired_velocity]` for PD feedback. |

For mixed position and other modes, use an `(n, 2)` matrix: position rows consume both columns and torque/velocity rows consume the first. There is no implemented `"hybrid"` mode even though the motor model's older docstring mentions it.

```python
cmd = MotorCommands(np.zeros(n), ["torque"] * n)
cmd.SetControlCmd(np.column_stack((q_des, v_des)), ["position"] * n)

# Four wheel speeds followed by arm torques, in actual active-joint order.
cmd.SetControlCmd(
    np.concatenate((wheel_speeds, arm_torques)),
    ["velocity"] * 4 + ["torque"] * len(arm_torques),
)
```

The simulator reads these fields. Older examples that assign `cmd.tau_cmd` without setting `ctrl_cmd` and `control_list` do not describe the current command interface.

### ServoMotorModel

`ServoMotorModel` turns a `MotorCommands` into an `(n_motors,)` torque vector through `compute_torque(motor_commands, cur_q, cur_qdot, prev_qdotdot, M)`.

| Mode | Torque before additional effects and clipping |
| --- | --- |
| Torque | `strength_ratio * desired_torque` |
| Position | `strength_ratio * (kp * (q_des - q) + kd * (v_des - v))` |
| Velocity | `strength_ratio * kd * (v_des - v)` |

Optional viscous friction and elastic terms are accumulated as `-friction_coefficient * v - elastic_coefficient * q`, multiplied by the supplied mass matrix `M`, and added to the command torque. Optional torque limits clip the final result. `set_strength_ratios(ratios)` adjusts each motor's output factor; `set_motor_gains(kp, kd)` and `get_motor_gains()` manage scalar or per-motor gains.

`motor_load`, its coefficient, and `prev_qdotdot` appear in the interface, but the load contribution is commented out. The stored `motor_control_mod` also does not select the active mode: `MotorCommands.control_list` does. An unknown mode raises `ValueError`; a command of the wrong object type prints a message and exits. `SimRobot` does not automatically pass URDF effort limits into this model's `torque_limits` argument.

## Kinematics and dynamics model

Source: [pin_wrapper.py](simulation_and_control/simulation_and_control/controllers/pin_wrapper.py).

### PinWrapper

```python
PinWrapper(
    conf_file_name,
    simulator="pybullet",
    list_link_name_for_reodering=external_names,
    data_source_names=["pybullet"],
    visualizer=False,
    index=0,
    conf_file_path_ext=root_dir,
)
```

`PinWrapper` owns `pin_model`, `pin_data`, and the mutable result container `res`. It builds a fixed model for `base_type="fixed"` and a free-flyer model for other base types. The separate `on_rack` dynamics behavior is unfinished.

Despite its name, `list_link_name_for_reodering` contains joint names. It must be a two-dimensional NumPy array, with one row per external data source. For one PyBullet robot use `np.array([sim.getNameActiveJoints(0)])`. The constructor requires an explicit simulator and two-dimensional array even though its defaults suggest otherwise. `index` chooses the robot's model/configuration entry.

Important attributes are `n` (full configuration length), `n_dot` (full velocity length), `n_b` and `n_bdot` (base lengths), and `n_q` and `n_qdot` (joint configuration and velocity lengths). `getNumberofActuatedJoints()` returns `n_q`; for continuous joints this can differ from the number of motor velocities. `control_groups` and `control_group_id` record named groups if present, but do not automatically reduce every dynamics calculation to a group.

#### Kinematics and ordering

| Method | Input and output |
| --- | --- |
| `ComputeFK(q, link_name)` | External configuration to `(world_position, world_rotation_matrix)` for a model frame. |
| `ComputeJacobian(q0, frame_name, local_or_global)` | Populate `res.J` and return `res`. Modes are `"local"` for frame axes, `"global"` for the world spatial reference, and `"local_global"` for frame origin with world-aligned axes. |
| `ComputeJacobianFeet(q0, feet_name, local_or_global)` | Resolve `FL`, `FR`, `RL`, or `RR` to a configured frame, then populate the same Jacobian container. |
| `KinematicIntegration(q0, v0, dt)` | Reorder external state, call Pinocchio manifold integration, and return the integrated configuration in Pinocchio order. |
| `CreateIndexJointAssociation(ext_list, data_source_names)` | Build joint-name permutations in `ext2pin` and `pin2ext`. |
| `ReoderJoints2PinVec(x, pos_or_vel, source_name=[])` | Reorder the joint part of an external vector into Pinocchio order while retaining the base prefix. |
| `ReoderJoints2ExtVec(x, pos_or_vel, source_name=[])` | Convert the joint part back into external order. |
| `ReoderJoints2ExMat(X, pos_or_vel, source_name=[])` | Apply the implemented joint-matrix permutation. Its floating-base handling does not comprehensively reorder coupled blocks. |

The spelling `Reoder` is part of the current method names. Use `"pos"` for configuration and `"vel"` for velocity or acceleration. With multiple sources, pass the source name; omission chooses the first recorded permutation.

#### Dynamics calculations

The analytical equation is `tau = M(q) @ acceleration + c(q, v) + g(q)`, where `c` is the Coriolis/centrifugal force vector. The Coriolis matrix is stored as `N`, not `c`.

| Method | Result |
| --- | --- |
| `ComputeMassMatrix(q)` | Populate `res.M` with Pinocchio CRBA inertia and floating-base blocks when applicable; return `res`. |
| `ComputeMassMatrixRNEA(x)` | Build inertia column by column using unit accelerations and inverse dynamics; return `res`. Its block attributes differ from the CRBA path. |
| `ComputeCoriolisMatrix(q, qdot)` | Populate `res.N` and its base/joint blocks; return `res`. |
| `ComputeCoriolis(q, qdot)` | Populate `res.c` using inverse dynamics at zero acceleration minus gravity; return `res`. |
| `ComputeGravity(q)` | Populate `res.g`; return `res`. |
| `ComputeAllTerms(q, qdot)` | Refresh `M`, `N`, `c`, and `g` through the above methods; returns `None`. Read `model.res` afterward. |
| `FullInverseDynamicsNoContact(x, xdot, xdotdot)` | Full generalized torque from recursive Newton Euler inverse dynamics, in Pinocchio order. |
| `InverseDynamicsActuatedPartNoContact(x_prev, xdot_prev, xdotdot_prev)` | Actuated torque from inertia and bias terms, in Pinocchio joint order; includes coupling from floating-base acceleration. |
| `ABA(q, qdot, tau)` | Forward dynamics acceleration in Pinocchio velocity order. `tau` is an external actuated torque vector; the method prepends zero base torque for a floating model. |
| `DirectDynamicsActuatedZeroTorqueNoContact(q, qdot)` | Intended unforced actuated acceleration after eliminating floating-base dynamics. Its fixed-base branch references an undefined intermediate and `on_rack` raises an error. |
| `FullInverseDynamicsWithContact(x, xdot, xdotdot, feet_contact_map, local_or_global)` | Intended `(torque_with_contacts, contact_torque, unconstrained_torque)`, subtracting point-force Jacobian contributions. The current `enumerate(feet_contact_map)` loop supplies numeric foot indices where string foot names are expected; it needs correction before normal dictionary use. |
| `ComputeDynamicRegressor(q, qd, qdd)` | Compute a joint torque regressor in Pinocchio data. There is no return statement; inspect `pin_data.jointTorqueRegressor` after the call. |

Dynamics calls share and overwrite `model.res`. Copy arrays if they must survive later calculations. `ComputeFK` also exposes Pinocchio data arrays; copy them when retaining a pose across updates.

`GetTotalMassFromUrdf()` obtains model total mass. `GetMassLink(link)` calls `computeSubtreeMasses` and indexes using a frame ID; it should not be treated as a general, validated single-link mass lookup. `getDynamicsInfo()` prints the model, `getNameActiveJoints()` returns model names including universe/root entries, and `DisplayModel(q)` reorders and displays a configuration when visualization is enabled. `GetConfigurationVariable(param)` reads the `robot_<simulator>` configuration section.

Private helpers `_UrdfPath` and `_LoadPinURDF` load the model. `_FromExtToPinVec`, `_FromPinToExtVec`, and `_FromPinToExtMat` permute joint-only quantities. `_ExtractJointsVec` and `_ExtractJointsMat` remove the base prefix; `_CopyJointsVec` and `_CopyJointsMat` insert the joint part back into a copy of a full quantity. These conversions assume a simple joint-coordinate permutation, which is insufficient for expanded continuous-joint configurations.

### ResultsFloatingBaseJoint

`ResultsFloatingBaseJoint(base_type)` is a cache, not a separate dynamics solver. Its fields start as `None` and become populated by the corresponding computation.

| Accessor | Full result | Floating-base subsets |
| --- | --- | --- |
| `GetJ(flag="")` | `J` | `"actuated"` selects `J_q`; `"underactuated"` selects `J_b`. |
| `GetM(flag="")` | `M` | `"actuated"` returns `(M_qq, M_qb)`; `"underactuated"` returns `(M_bb, M_bq)`. |
| `GetN(flag="")` | `N` | Subset access references `N_q` and `N_b`, which the current Coriolis matrix calculation does not populate. |
| `GetG(flag="")` | `g` | `g_q` or `g_b`. |
| `GetC(flag="")` | `c` | `c_q` or `c_b`. |

For fixed bases, `"actuated"` returns the full quantity and `"underactuated"` returns `None`. For floating bases, the Jacobian partition assignments are inconsistent between `ComputeJacobian` and `ComputeJacobianFeet`: use `res.J[:, :6]` for base columns and `res.J[:, 6:]` for joint columns rather than trusting `J_b` and `J_q`.

`set_continuous_joint_angle(q, idx, theta)` writes cosine and sine into `q[idx:idx+2]` in place. `get_continuous_joint_angle(q, idx)` recovers the angle with `atan2`. These helpers do not automatically extend the wrapper's joint permutations.

## Manipulator controllers

### Cartesian differential kinematics

Source: [CartesianKinematic.py](simulation_and_control/simulation_and_control/controllers/CartesianKinematic.py).

```python
CartesianDiffKin(
    dyn_model, controlled_frame_name, cur_q,
    p_des, pd_des, ori_des, ori_vel_des, delta_t,
    ori_pos_both, kp_pos, kp_ori, joint_vel_saturation,
)
```

`CartesianDiffKin` obtains the frame pose and world-aligned Jacobian, forms Cartesian feedback, solves with the Jacobian pseudoinverse, applies a joint velocity dead zone of `0.01`, clips velocity, and integrates it. It returns `(q_des, qd_des_clip)`: the configuration is in Pinocchio order and the velocity follows the Jacobian's Pinocchio column order.

`p_des` and `pd_des` are three-element world position and velocity references. `ori_des` is a desired rotation matrix accepted by `pin.Quaternion`, or `None` for zero orientation error. `ori_pos_both` selects `"pos"`, `"ori"`, or `"both"`. Position control uses `pinv(J_linear) @ (kp_pos * position_error + pd_des)`. Orientation error uses the vector part of the relative quaternion rotated into world axes. `ori_vel_des` is currently ignored; orientation feedforward is zero.

The `"pos"` path is used by the repository examples. The `"both"` path currently stacks the two gain matrices into a `6 × 3` array and multiplies it by a six-element error, causing a dimension mismatch. This method does not implement damped inverse kinematics, collision avoidance, or joint position limit enforcement.

`applyJointVelSaturation(qd_des, joint_vel_saturation)` clips each component to symmetric scalar or vector limits and prints when clipping occurs. `apply_dead_zone(velocity, thresh)` replaces components with magnitude strictly below `thresh` with zero. Both return arrays without changing their input in place.

### Feedback linearization and dynamic cancellation

Sources: [FeedbackLin.py](simulation_and_control/simulation_and_control/controllers/FeedbackLin.py) and [DynamicCancellation.py](simulation_and_control/simulation_and_control/controllers/DynamicCancellation.py).

`feedback_lin_ctrl(dyn_model, q_, qd_, q_d, qd_d, kp, kd)` returns external-order joint torque. It computes:

```text
u = diag(kp) @ (q_d - measured_q_in_pin_order)
  + diag(kd) @ (qd_d - measured_v_in_pin_order)
tau_pin = M @ u + c + g
```

Measured `q_` and `qd_` are external-order vectors; desired `q_d` and `qd_d` must already use Pinocchio order. Scalars are expanded into uniform gains; per-joint gains must be NumPy arrays of the expected length. The function returns torque even though its old docstring says `None`. It has no desired acceleration argument and is principally useful for fixed-base joint control.

`dyn_cancel(dyn_model, q_, qd_, u)` uses the same dynamics computation but takes an acceleration-like input `u` directly: `tau_pin = M @ u + c + g`. `u` must match the model's velocity dimension and Pinocchio order. It returns torque converted by the joint-only external permutation, so floating-base generalized inputs require additional handling.

### Cartesian impedance

Source: [ImpedanceCtrl.py](simulation_and_control/simulation_and_control/controllers/ImpedanceCtrl.py).

`ImpedanceController(dyn_model, controlled_frame_name, q_mes, qd_mes, xdes, kp, kd, tau_ext, pos_or_ori="pos")` returns joint torque for a fixed-base serial chain. In the position path, `xdes` is a three-element Cartesian position, gains are scalars or three-element vectors, and `tau_ext` is a six-element wrench ordered force then moment; the function uses its first three entries.

The function builds a task inertia approximation `Mx = pinv(J).T @ M @ pinv(J)`, maps bias forces to task space, and combines position feedback and damping through `J.T`. It reads velocities directly against the Pinocchio-order Jacobian and does not reorder the final torque, so use matching joint orders or explicitly convert those quantities when orders differ.

The current implementation obtains Cartesian position from `ComputeFK` even in orientation mode. It therefore does not implement a correct orientation error, and its combined mode has inconsistent dimensions. The repository example exercises position impedance only. Desired Cartesian velocity is not an input: the damping term acts on measured Cartesian velocity toward zero.

## Mobile base controllers

Source: [MobileBaseController.py](simulation_and_control/simulation_and_control/controllers/MobileBaseController.py).

These functions are independent of `SimInterface` and `PinWrapper`. They work on planar position and yaw and return left and right wheel angular velocities. Map those outputs into the robot's actual active wheel-joint order before constructing `MotorCommands`.

| Function | Behavior and output |
| --- | --- |
| `wrap_angle(angle)` | Wrap scalar or array angles into `[-pi, pi)`. |
| `velocity_to_wheel_angular_velocity(v, w, wheel_base_width, wheel_radius, number_of_wheels=4)` | Compute `left = (v - w * width / 2) / radius`, `right = (v + w * width / 2) / radius`. For four wheels, the implementation divides both results by two; for two wheels it returns the full values. It prints the selected behavior. |
| `differential_drive_regulation_controller(...)` | Regulate position and orientation together, reduce linear speed with the cosine of heading error, clip body speeds, and return `(left, right)`. Positions use their first two components; orientations are scalar yaw. |
| `differential_drive_controller_adjusting_bearing(...)` | Drive toward position, then stop translation and regulate final yaw. Return `(left, right, at_goal)`. Defaults are `0.05` meters position tolerance and `0.05` radians orientation tolerance. |
| `regulation_polar_coordinates(...)` | Use distance `rho`, bearing error `alpha`, and final heading error `beta` to set `v = k_rho * rho`, `w = k_alpha * alpha + k_beta * beta`. Return `(left, right)`. |
| `regulation_polar_coordinate_quat(...)` | Compute the same planar errors through quaternion arithmetic internally. Inputs `theta` and `thetag` are still scalar yaw angles. Return `(left, right)`. |

Both differential-drive controllers accept `kp_pos`, `kp_ori`, `number_of_wheels`, `max_linear_velocity`, and `max_angular_velocity`. The polar functions accept `k_rho`, `k_alpha`, and `k_beta`; their `number_of_wheels` parameter is unused and their wheel conversion does not halve four-wheel outputs. They also do not expose velocity clipping or an `at_goal` flag.

The module-only helpers `euler_to_quaternion(theta)`, `quaternion_conjugate(q)`, `quaternion_multiply(q1, q2)`, and `quaternion_to_euler(q)` implement planar yaw conversion and Hamilton products using `[w, x, y, z]`. They are not interchangeable with the PyBullet quaternion helper's ordering.

## Reference and configuration utilities

Sources: [SinusoidalRef.py](simulation_and_control/simulation_and_control/utils/SinusoidalRef.py) and [checking_input_from_config.py](simulation_and_control/simulation_and_control/utils/checking_input_from_config.py).

`SinusoidalReference(amplitude, frequency, q_init)` converts inputs to arrays and requires equal element counts. `frequency` is in cycles per second and `amplitude` has the same units as the reference quantity. It can generate joint references or Cartesian position references; it is not tied to a specific robot model.

`get_values(time)` returns `(position, velocity)` using:

```text
position = q_init + amplitude * sin(2*pi*frequency*time - pi/2)
velocity = amplitude * 2*pi*frequency * cos(2*pi*frequency*time - pi/2)
```

Thus the trajectory starts at `q_init - amplitude`, with zero velocity, rather than at `q_init`. `check_sinusoidal_feasibility(sim)` checks the first robot's configured initial joint angles plus/minus amplitude against URDF position bounds and checks peak velocity. It prints a result and returns a boolean. This check assumes a joint reference centered on the simulator's initial angles; it does not validate Cartesian references or a separately chosen `q_init` center.

`adjust_value(flag, value, number_of_elements, vector_name)` returns zeros when disabled. When enabled, it expands a scalar or one-element list to the required length, or converts a full-length list to an array. A list of another length raises `ValueError`, using `vector_name` in the message. NumPy arrays follow its general multiplication branch rather than the explicit list-length checks. The motor model and simulator use it to normalize optional configuration coefficients.

## Humanoid controllers

Source directory: [humanoid_controller](simulation_and_control/simulation_and_control/controllers/humanoid_controller).

This subsystem targets DART through `dartpy` and robot body names such as `l_sole`, `r_sole`, `torso`, and `body`. It is distinct from the PyBullet API. Several modules use script-style imports such as `from utils import ...`; they are not a fully integrated package-root API. The included PyBullet humanoid example does not execute this walking pipeline.

Its intended flow is `FootstepPlanner -> Ismpc and FootTrajectoryGenerator -> InverseKinematics -> DART acceleration commands`, with `KalmanFilter` estimating center of mass and zero moment point (ZMP) state. MPC means model predictive control; the linear inverted pendulum (LIP) model approximates horizontal center of mass dynamics at a fixed height.

### Humanoid state and mathematical helpers

In [utils.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/utils.py), `LipState` stores three-element center of mass position, velocity, acceleration, ZMP position, and ZMP velocity arrays, defaulting to zeros. `State(ndofs, ...)` additionally stores left/right six-element foot poses, foot velocities and accelerations, torso/base orientation and angular derivatives, and `ndofs`-element joint position, velocity, and acceleration arrays. Supplied arrays are stored directly, not copied.

`get_rotvec(rot_matrix)` converts a rotation matrix to a rotation vector through SciPy. `rotation_vector_difference(rotvec_a, rotvec_b)` returns the rotation vector of `R_b.inverse() * R_a`. `block_diag(*arrays)` assembles a block-diagonal array, treating scalars as `1 × 1` blocks.

`pose_difference(pose_a, pose_b)` subtracts the first three elements as position and treats the last three as rotation vectors. This conflicts with `Hrp4Controller` and `FootTrajectoryGenerator`, which assemble feet as `[orientation, position]`. That mismatch must be reconciled before treating foot task errors as physically meaningful.

`QPSolver(n_dofs)` builds the unconstrained objective `0.5 * x.T @ H @ x + F.T @ x` using CasADi and OSQP. `set_values(H, F)` supplies its parameters; `solve()` returns the minimizing vector. There are no joint or contact constraints in this utility.

### KalmanFilter

In [filter.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/filter.py), `KalmanFilter(A, B, H, Q, R, P, x)` stores a linear transition model, input model, observation model, process/measurement covariance, initial covariance, and initial state. `predict(u)` updates `x` and `P`; `update(z)` applies the measurement correction. Both return `(x, P)` and retain the updated values on the object.

### FootstepPlanner and FootTrajectoryGenerator

`FootstepPlanner(vref, initial_lfoot, initial_rfoot, first_support_foot, delta)` integrates a virtual unicycle reference into alternating support footsteps. Each item in `footstep_plan` contains `pos`, `ang`, `ss_duration`, `ds_duration`, and `foot_id`. `vref` supplies planar linear and yaw velocities; `first_support_foot` is `"left"` or `"right"`.

Planner times are integer control ticks, not seconds. Default durations are 70 single-support ticks and 30 double-support ticks, with the first step using 100 double-support ticks. `delta` is the seconds per tick used for velocity integration. `get_step_index_at_time(time)` returns the matching step index or `None` beyond the plan. `get_start_time(step_index)` sums preceding durations. `get_phase_at_time(time)` returns `"ss"` or `"ds"` and assumes a valid step exists.

`FootTrajectoryGenerator(initial, footstep_planner, delta=0.01)` produces foot references through `generate_feet_trajectories_at_time(time)`. The returned dictionary has `left` and `right` entries, each containing `pos`, `vel`, and `acc`, all six-element arrays with orientation first. It holds both feet during initial support, uses cubic interpolation for swing translation/orientation and a quartic vertical lift, and holds the support foot still. Step height defaults to `0.02` meters. Time is in planner ticks; derivatives are scaled by `delta`. References use the following planned step, so callers must retain enough future plan entries.

### Ismpc

`Ismpc(initial, footstep_planner, N=100, delta=0.01, g=9.81, h=0.75)` builds a CasADi/OSQP LIP optimization over `N` future ticks. It predicts horizontal center of mass and ZMP, controls ZMP velocity, constrains ZMP around moving support references, and adds a periodic-tail stability equality.

`solve(current, t)` returns `(lip_state, contact)` for the next predicted tick, warm-starts future solves, and reports contact as `"ds"`, `"ssleft"`, or `"ssright"`. `generate_moving_constraint_at_time(time)` returns a support location or interpolation during double support; `generate_moving_constraint(t)` returns the two horizon arrays `(mc_x, mc_y)`. `t` is a tick index. The dynamics use the supplied `h`, but the returned center of mass height is currently hardcoded to `0.75`.

### InverseKinematics

`InverseKinematics(robot, redundant_dofs)` takes a DART skeleton and a list of joint names whose posture should be tracked. `get_joint_accelerations(desired, current, supportFoot)` builds weighted least-squares tasks for both feet, center of mass, torso, base, and selected joints. It includes Jacobian derivatives and desired accelerations, solves through `QPSolver`, and returns only the actuated acceleration tail, dropping the first six floating-base entries. `supportFoot` is accepted but not used to alter the optimization.

### Hrp4Controller

In [simulation.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/simulation.py), `Hrp4Controller(world, hrp4)` configures the DART skeleton and composes the estimator, planner, MPC, foot trajectories, and inverse kinematics. `customPreStep()` is the DART callback that reads state, filters horizontal LIP quantities, generates desired motion, computes joint accelerations, applies them, and increments its tick counter.

`retrieve_state()` returns a `State` from skeleton kinematics and contacts. `get_zmp()` estimates ZMP from collision forces and returns zeros when contact force is too low. `initialize_plot()` and `update_plot()` manage diagnostic plots. This controller assumes the HRP4 naming and DOF layout and is not a generic `SimInterface` controller.

## Quadruped controllers

Sources: [QuadController.py](simulation_and_control/simulation_and_control/controllers/QuadController.py) and [quadruped_controller](simulation_and_control/simulation_and_control/controllers/quadruped_controller).

These files describe a separate quadruped MPC pipeline using `LegsAttr`, a per-leg container from `gym_quadruped`. The usual leg order is `FL`, `FR`, `RL`, `RR`. Contact arrays use `1` for stance and `0` for swing, with contact prediction generally shaped `(4, horizon)`.

The intended flow is `QuadrupedCtrl -> WBInterface state/reference update -> SRBDControllerInterface -> Acados_NMPC_Nominal -> WBInterface torque computation`. SRBD means single rigid body dynamics: the optimizer approximates body motion and chooses foot ground reaction forces rather than solving full articulated robot dynamics.

These modules are incomplete integrations in this checkout. They reference external configuration globals `cfg` or `config` whose imports are commented out or absent, use script-style sibling imports, and depend on external quadruped/MuJoCo/acados components. `SRBDControllerInterface` calls `Acados_NMPC_Nominal()` although that constructor requires `params_dict`; the latter calls `Centroidal_Model_Nominal()` although it requires `use_foothold_optimization`. Referenced `swing_generators` files and `VisualFootholdAdaptation` are also absent from this package. The descriptions below explain the source responsibilities rather than promising a runnable PyBullet controller.

### QuadrupedCtrl and WBInterface

| Class or method | Responsibility |
| --- | --- |
| `QuadrupedCtrl(initial_feet_pos, legs_order, quadrupedpympc_observables_names)` | Compose the body optimizer and whole-body controller; keep the latest optimized forces, footholds, joint references, and selected observables. |
| `QuadrupedCtrl.compute_actions(...)` | Consume measured base/foot/joint state, model matrices, reference base speeds, and step timing; update references, run MPC at its configured frequency, and return per-leg torque `tau`. |
| `QuadrupedCtrl.get_obs()` | Return the chosen observable dictionary, whose defaults include base height/angles, GRFs, footholds, and swing time. |
| `QuadrupedCtrl.reset(initial_feet_pos)` | Reset the composed whole-body state and observables. |
| `WBInterface(initial_feet_pos, legs_order)` | Own a periodic gait generator, foothold generator, swing controller, and terrain estimator. |
| `WBInterface.update_state_and_reference(...)` | Return current state, reference state, contact sequence, reference feet, foothold constraints, sequence time steps and lengths, step height, and swing optimization flag. |
| `WBInterface.compute_stance_and_swing_torque(...)` | Map optimized stance forces and swing tracking to per-leg torque, using Jacobians, Jacobian derivatives, mass/bias terms, and joint indices. Return `tau`. |
| `WBInterface.reset(initial_feet_pos)` | Reset gait and foot reference state. |

The large method signatures distinguish whole-system `qpos`/`qvel` from per-leg quantities such as `feet_jac`, `jac_feet_dot`, `legs_mass_matrix`, and `legs_qfrc_bias`. All leg containers and index maps must agree on `legs_order` and the underlying model's coordinate layout.

### Gait, foothold, swing, and terrain helpers

| Class | Methods and outputs |
| --- | --- |
| `PeriodicGaitGenerator(duty_factor, step_freq, gait_type, horizon)` | `reset()` initializes gait phase offsets with a random common phase. `run(dt, new_step_freq)` advances phases and returns contact flags. `set_phase_signal(phase_signal, init=None)` restores phase state; the `phase_signal` property returns a copy. `compute_contact_sequence(contact_sequence_dts, contact_sequence_lenghts)` predicts future contacts and restores the original phases. The full-stance branch returns `(4, 2*horizon)` rather than the usual `(4, horizon)`. |
| `FootholdReferenceGenerator(stance_time, lift_off_positions, vel_moving_average_length=20, hip_height=None)` | `compute_footholds_reference(...)` uses filtered measured velocity, desired velocity, hip geometry, and center of mass height to return a `LegsAttr` of target footholds. `update_lift_off_positions(previous_contact, current_contact, feet_pos, legs_order)` records positions on stance-to-swing transitions. |
| `SwingTrajectoryController(step_height, swing_period, position_gain_fb, velocity_gain_fb, generator)` | `compute_swing_control(...)` generates foot references and returns `(tau_swing, desired_foot_position, desired_foot_velocity)` using tracking feedback plus inertia/bias terms. `update_swing_time(...)` advances each swinging leg's clock and resets stance clocks. `regenerate_swing_trajectory_generator(...)` changes swing parameters. `check_apex_condition(...)` returns an integer flag near mid-swing; `check_full_stance_condition(...)` returns an integer full-stance flag. |
| `TerrainEstimator()` | `compute_terrain_estimation(base_position, yaw, feet_pos, current_contact)` updates and returns `(terrain_roll, terrain_pitch, terrain_height)`. Roll and pitch use all four foot positions; height averages only feet marked in contact. The estimates are smoothed across calls. |

The gait constructor compares `gait_type` against `GaitType.*.value`. The swing generator selects `ndcurves`, `scipy`, or an explicit generator implementation by string, but those generator implementations are not present here.

### Centroidal prediction and MPC

`Centroidal_Model_Nominal(use_foothold_optimization)` builds symbolic CasADi state, input, and parameter vectors. Its state has 30 entries: center of mass position/velocity, body Euler orientation/angular velocity, four foot positions, and six integral states. Its input has 24 entries: four three-dimensional foot velocities followed by four three-dimensional forces. Parameters include contact flags, friction, stance proximity, base pose reference, external wrench, flattened inertia, and mass. `forward_dynamics(states, inputs, param)` returns symbolic state derivatives; `export_robot_model()` returns an `AcadosModel` with explicit and implicit dynamics.

`Acados_NMPC_Nominal(params_dict)` creates the acados optimal control problem and solver. The constructor configures horizon, time step, solver modes, warm starting, integrators, and optional constraints; it also generates solver code and invokes an initial solve. This object has filesystem and solver side effects, unlike a pure controller function.

| Method | Responsibility or return |
| --- | --- |
| `create_ocp_solver_description(acados_model)` | Build and return the optimal control problem. |
| `create_stability_constraints()` | Return stability constraint expression and bounds. |
| `create_foothold_constraints()` | Return foothold constraint expression and bounds. |
| `create_friction_cone_constraints()` | Return ground reaction force/friction constraint expression and bounds. |
| `set_weight(nx, nu)` | Return state and input weighting matrices `(Q_mat, R_mat)`. |
| `set_stage_constraint(...)` | Update stage bounds from contact and foothold information. |
| `set_warm_start(...)` | Seed the optimizer with prior state, reference, and leg contact schedules. |
| `perform_scaling(state, reference, constraint=None)` | Transform optimization coordinates and return the transformed state, reference, and constraints. |
| `compute_control(state, reference, contact_sequence, constraint=None, external_wrenches=..., inertia=..., mass=...)` | Update the problem, solve it, and return `(optimal_GRF, optimal_foothold, optimal_next_state, status)`. |
| `reset()` | Reset solver/integrator memory. |

`SRBDControllerInterface.compute_control(state_current, ref_state, contact_sequence, inertia, pgg_phase_signal, pgg_step_freq, optimize_swing)` wraps this result into leg containers, masks forces by current contacts, and optionally runs an RTI preparation solve. It returns six items: GRFs, footholds, joint positions, joint velocities, joint accelerations, and sample frequency. In the nominal path, all three joint-reference outputs are `None` and the frequency is the supplied gait frequency.

## Control loop example

This example shows the core object relationships for the fixed-base Panda configuration. Save it at the RoboEnv root so the path identifies the directory containing `configs/` and `models/`. It holds the initial posture for 100 simulation steps using feedback linearization; it does not require a trajectory generator.

```python
from pathlib import Path

import numpy as np
from simulation_and_control import (
    MotorCommands,
    PinWrapper,
    feedback_lin_ctrl,
    pb,
)

root_dir = str(Path(__file__).resolve().parent)
config = "pandaconfig.json"
sim = pb.SimInterface(config, conf_file_path_ext=root_dir, use_gui=False)

try:
    names = np.array([sim.getNameActiveJoints(0)])
    model = PinWrapper(
        config,
        simulator="pybullet",
        list_link_name_for_reodering=names,
        data_source_names=["pybullet"],
        visualizer=False,
        index=0,
        conf_file_path_ext=root_dir,
    )

    n = sim.GetActionDimension(0)
    q_initial = np.asarray(sim.GetInitMotorAngles(0))
    q_des = model.ReoderJoints2PinVec(q_initial, "pos")
    v_des = np.zeros(model.n_dot)
    cmd = MotorCommands(np.zeros(n), ["torque"] * n)

    for _ in range(100):
        q = sim.GetMotorAngles(0)
        v = sim.GetMotorVelocities(0)
        tau = feedback_lin_ctrl(model, q, v, q_des, v_des, kp=100, kd=20)
        cmd.SetControlCmd(tau, ["torque"] * n)
        sim.Step(cmd)
finally:
    sim.GetPyBulletClient().disconnect()
```

For Cartesian tracking, first call `CartesianDiffKin` in position mode and pass its returned configuration and velocity to `feedback_lin_ctrl`. For direct motor PD control, supply an `(n, 2)` position command instead. Both routes ultimately apply torque through `ServoMotorModel`.

## Example scripts

The files under `tests/` are interactive demonstrations with a `main()` entry point, not unit tests for every class.

| Script | What it demonstrates |
| --- | --- |
| [cartesian_kinematic_controller.py](tests/cartesian_kinematic_controller.py) | Sinusoidal Cartesian position reference, differential inverse kinematics, and feedback linearization torque. |
| [cartesian_impedance_controller.py](tests/cartesian_impedance_controller.py) | Sinusoidal position targets and Cartesian position impedance torque. |
| [mobile_base_kinematic_controller.py](tests/mobile_base_kinematic_controller.py) | Waypoint navigation with the bearing-adjustment controller, wheel velocity commands, and a local `quaternion2bearing` helper converting base orientation to yaw. |
| [mobile_base_arm_kinematic_controller.py](tests/mobile_base_arm_kinematic_controller.py) | Wheel velocity commands combined with joint PD torque to hold arm posture while driving the base; its own `quaternion2bearing` helper also extracts yaw. |
| [humanoid_walk_controller.py](tests/humanoid_walk_controller.py) | Construct the HRP4 PyBullet simulation and analytical model. Its walking loop is commented out; it does not exercise the DART humanoid controller. |

## Complete signature index

The following index lists every class and function defined in the package source, including private helpers. Signatures preserve the implementation's spelling and defaults. Constructors are shown as `__init__`; the descriptions above explain their behavior. The `phase_signal` getter is a property even though it appears as a method definition in this source index.

### controllers/CartesianKinematic.py

Source: [CartesianKinematic.py](simulation_and_control/simulation_and_control/controllers/CartesianKinematic.py).

```python
def applyJointVelSaturation(qd_des, joint_vel_saturation)
def apply_dead_zone(velocity, thresh)
def CartesianDiffKin(dyn_model, controlled_frame_name, cur_q, p_des, pd_des, ori_des, ori_vel_des, delta_t, ori_pos_both, kp_pos, kp_ori, joint_vel_saturation)
```

### controllers/DynamicCancellation.py

Source: [DynamicCancellation.py](simulation_and_control/simulation_and_control/controllers/DynamicCancellation.py).

```python
def dyn_cancel(dyn_model, q_, qd_, u)
```

### controllers/FeedbackLin.py

Source: [FeedbackLin.py](simulation_and_control/simulation_and_control/controllers/FeedbackLin.py).

```python
def feedback_lin_ctrl(dyn_model, q_, qd_, q_d, qd_d, kp, kd)
```

### controllers/ImpedanceCtrl.py

Source: [ImpedanceCtrl.py](simulation_and_control/simulation_and_control/controllers/ImpedanceCtrl.py).

```python
def ImpedanceController(dyn_model, controlled_frame_name, q_mes, qd_mes, xdes, kp, kd, tau_ext, pos_or_ori='pos')
```

### controllers/MobileBaseController.py

Source: [MobileBaseController.py](simulation_and_control/simulation_and_control/controllers/MobileBaseController.py).

```python
def wrap_angle(angle)
def velocity_to_wheel_angular_velocity(desired_linear_velocity, desired_angular_velocity, wheel_base_width, wheel_radius, number_of_wheels=4)
def differential_drive_regulation_controller(current_position, current_orientation, desired_position, desired_orientation, wheel_radius, wheel_base_width, kp_pos, kp_ori, number_of_wheels=4, max_linear_velocity=100, max_angular_velocity=100)
def differential_drive_controller_adjusting_bearing(current_position, current_orientation, desired_position, desired_orientation, wheel_radius, wheel_base_width, kp_pos, kp_ori, number_of_wheels=4, position_tolerance=0.05, orientation_tolerance=0.05, max_linear_velocity=100.0, max_angular_velocity=100.0)
def regulation_polar_coordinates(x, y, theta, xg, yg, thetag, wheel_radius, wheel_base_width, k_rho, k_alpha, k_beta, number_of_wheels=4)
def euler_to_quaternion(theta)
def quaternion_conjugate(q)
def quaternion_multiply(q1, q2)
def quaternion_to_euler(q)
def regulation_polar_coordinate_quat(x, y, theta, xg, yg, thetag, wheel_radius, wheel_base_width, k_rho, k_alpha, k_beta, number_of_wheels=4)
```

### controllers/QuadController.py

Source: [QuadController.py](simulation_and_control/simulation_and_control/controllers/QuadController.py).

```python
class QuadrupedCtrl
    __init__(self, initial_feet_pos: LegsAttr, legs_order: tuple[str, str, str, str]=('FL', 'FR', 'RL', 'RR'), quadrupedpympc_observables_names: tuple[str, ...]=_DEFAULT_OBS)
    compute_actions(self, base_pos: np.ndarray, base_lin_vel: np.ndarray, base_ori_euler_xyz: np.ndarray, base_ang_vel: np.ndarray, feet_pos: LegsAttr, hip_pos: LegsAttr, joints_pos: LegsAttr, heightmaps, legs_order: tuple[str, str, str, str], simulation_dt: float, ref_base_lin_vel: np.ndarray, ref_base_ang_vel: np.ndarray, step_num: int, qpos: np.ndarray, qvel: np.ndarray, feet_jac: LegsAttr, jac_feet_dot: LegsAttr, feet_vel: LegsAttr, legs_qfrc_bias: LegsAttr, legs_mass_matrix: LegsAttr, legs_qpos_idx: LegsAttr, legs_qvel_idx: LegsAttr, tau: LegsAttr, inertia: np.ndarray)
    get_obs(self)
    reset(self, initial_feet_pos: LegsAttr)
```

### controllers/humanoid_controller/filter.py

Source: [filter.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/filter.py).

```python
class KalmanFilter
    __init__(self, A, B, H, Q, R, P, x)
    predict(self, u)
    update(self, z)
```

### controllers/humanoid_controller/foot_trajectory_generator.py

Source: [foot_trajectory_generator.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/foot_trajectory_generator.py).

```python
class FootTrajectoryGenerator
    __init__(self, initial, footstep_planner, delta=0.01)
    generate_feet_trajectories_at_time(self, time)
```

### controllers/humanoid_controller/footstep_planner.py

Source: [footstep_planner.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/footstep_planner.py).

```python
class FootstepPlanner
    __init__(self, vref, initial_lfoot, initial_rfoot, first_support_foot, delta)
    get_step_index_at_time(self, time)
    get_start_time(self, step_index)
    get_phase_at_time(self, time)
```

### controllers/humanoid_controller/inverse_kinematics.py

Source: [inverse_kinematics.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/inverse_kinematics.py).

```python
class InverseKinematics
    __init__(self, robot, redundant_dofs)
    get_joint_accelerations(self, desired, current, supportFoot)
```

### controllers/humanoid_controller/ismpc.py

Source: [ismpc.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/ismpc.py).

```python
class Ismpc
    __init__(self, initial, footstep_planner, N=100, delta=0.01, g=9.81, h=0.75)
    solve(self, current, t)
    generate_moving_constraint_at_time(self, time)
    generate_moving_constraint(self, t)
```

### controllers/humanoid_controller/simulation.py

Source: [simulation.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/simulation.py).

```python
class Hrp4Controller
    __init__(self, world, hrp4)
    customPreStep(self)
    retrieve_state(self)
    get_zmp(self)
    initialize_plot(self)
    update_plot(self)
```

### controllers/humanoid_controller/utils.py

Source: [utils.py](simulation_and_control/simulation_and_control/controllers/humanoid_controller/utils.py).

```python
def rotation_vector_difference(rotvec_a, rotvec_b)
def pose_difference(pose_a, pose_b)
def get_rotvec(rot_matrix)
def block_diag(*arrays)
class QPSolver
    __init__(self, n_dofs)
    set_values(self, H, F)
    solve(self)
class LipState
    __init__(self, com_position=None, com_velocity=None, com_acceleration=None, zmp_position=None, zmp_velocity=None)
class State
    __init__(self, ndofs, left_foot_pose=None, right_foot_pose=None, com_position=None, torso_orientation=None, base_orientation=None, left_foot_velocity=None, right_foot_velocity=None, com_velocity=None, torso_angular_velocity=None, base_angular_velocity=None, left_foot_acceleration=None, right_foot_acceleration=None, com_acceleration=None, torso_angular_acceleration=None, base_angular_acceleration=None, joint_position=None, joint_velocity=None, joint_acceleration=None, zmp_position=None, zmp_velocity=None)
```

### controllers/pin_wrapper.py

Source: [pin_wrapper.py](simulation_and_control/simulation_and_control/controllers/pin_wrapper.py).

```python
def set_continuous_joint_angle(q, idx, theta)
def get_continuous_joint_angle(q, idx)
class ResultsFloatingBaseJoint
    __init__(self, base_type)
    GetJ(self, flag='')
    GetM(self, flag='')
    GetN(self, flag='')
    GetG(self, flag='')
    GetC(self, flag='')
class PinWrapper
    __init__(self, conf_file_name, simulator=None, list_link_name_for_reodering=np.empty(0), data_source_names=[], visualizer=False, index=0, conf_file_path_ext=None)
    _UrdfPath(self, index, conf_file_path_ext: str=None)
    _LoadPinURDF(self, urdf_file)
    CreateIndexJointAssociation(self, ext_list, data_source_names)
    ComputeJacobian(self, q0, frame_name, local_or_global)
    ComputeJacobianFeet(self, q0, feet_name, local_or_global)
    KinematicIntegration(self, q0, v0, dt)
    ComputeFK(self, q, link_name)
    ComputeMassMatrix(self, q)
    ComputeMassMatrixRNEA(self, x)
    ComputeCoriolisMatrix(self, q, qdot)
    ComputeCoriolis(self, q, qdot)
    ComputeGravity(self, q)
    ComputeAllTerms(self, q, qdot)
    DirectDynamicsActuatedZeroTorqueNoContact(self, q, qdot)
    FullInverseDynamicsNoContact(self, x, xdot, xdotdot)
    FullInverseDynamicsWithContact(self, x, xdot, xdotdot, feet_contact_map, local_or_global)
    InverseDynamicsActuatedPartNoContact(self, x_prev, xdot_prev, xdotdot_prev)
    ABA(self, q, qdot, tau)
    GetTotalMassFromUrdf(self)
    GetMassLink(self, link)
    getDynamicsInfo(self)
    getNameActiveJoints(self)
    getNumberofActuatedJoints(self)
    DisplayModel(self, q)
    _FromExtToPinVec(self, x, source_name=[])
    _FromPinToExtVec(self, x, source_name=[])
    _FromPinToExtMat(self, X, source_name=[])
    _ExtractJointsVec(self, x, flag='pos')
    _ExtractJointsMat(self, X, flag='pos')
    _CopyJointsVec(self, x_dest, x_q, flag='pos')
    _CopyJointsMat(self, X_dest, X_q, flag='pos')
    ReoderJoints2PinVec(self, x, pos_or_vel, source_name=[])
    ReoderJoints2ExtVec(self, x, pos_or_vel, source_name=[])
    ReoderJoints2ExMat(self, X, pos_or_vel, source_name=[])
    GetConfigurationVariable(self, param)
    ComputeDynamicRegressor(self, q, qd, qdd)
```

### controllers/quadruped_controller/mpc_quad_srbd/SrbdControllerInterface.py

Source: [SrbdControllerInterface.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_srbd/SrbdControllerInterface.py).

```python
class SRBDControllerInterface
    __init__(self)
    compute_control(self, state_current: dict, ref_state: dict, contact_sequence: np.ndarray, inertia: np.ndarray, pgg_phase_signal: np.ndarray, pgg_step_freq: float, optimize_swing: int)
```

### controllers/quadruped_controller/mpc_quad_srbd/centroidal_model_nominal.py

Source: [centroidal_model_nominal.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_srbd/centroidal_model_nominal.py).

```python
class Centroidal_Model_Nominal
    __init__(self, use_foothold_optimization)
    forward_dynamics(self, states: np.ndarray, inputs: np.ndarray, param: np.ndarray)
    export_robot_model(self)
```

### controllers/quadruped_controller/mpc_quad_srbd/centroidal_nmpc_nominal.py

Source: [centroidal_nmpc_nominal.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_srbd/centroidal_nmpc_nominal.py).

```python
class Acados_NMPC_Nominal
    __init__(self, params_dict)
    create_ocp_solver_description(self, acados_model)
    create_stability_constraints(self)
    create_foothold_constraints(self)
    create_friction_cone_constraints(self)
    set_weight(self, nx, nu)
    reset(self)
    set_stage_constraint(self, constraint, state, reference, contact_sequence, h_R_w, stance_proximity)
    set_warm_start(self, state_acados, reference, FL_contact_sequence, FR_contact_sequence, RL_contact_sequence, RR_contact_sequence)
    perform_scaling(self, state, reference, constraint=None)
    compute_control(self, state, reference, contact_sequence, constraint=None, external_wrenches=np.zeros((6,)), inertia=config.inertia.reshape((9,)), mass=config.mass)
```

### controllers/quadruped_controller/mpc_quad_wb/WbQuadInterface.py

Source: [WbQuadInterface.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_wb/WbQuadInterface.py).

```python
class WBInterface
    __init__(self, initial_feet_pos: LegsAttr, legs_order: tuple[str, str, str, str]=('FL', 'FR', 'RL', 'RR'))
    update_state_and_reference(self, base_pos: np.ndarray, base_lin_vel: np.ndarray, base_ori_euler_xyz: np.ndarray, base_ang_vel: np.ndarray, feet_pos: LegsAttr, hip_pos: LegsAttr, joints_pos: LegsAttr, heightmaps, legs_order: tuple[str, str, str, str], simulation_dt: float, ref_base_lin_vel: np.ndarray, ref_base_ang_vel: np.ndarray)
    compute_stance_and_swing_torque(self, simulation_dt: float, qpos: np.ndarray, qvel: np.ndarray, feet_jac: LegsAttr, jac_feet_dot: LegsAttr, feet_pos: LegsAttr, feet_vel: LegsAttr, legs_qfrc_bias: LegsAttr, legs_mass_matrix: LegsAttr, nmpc_GRFs: LegsAttr, nmpc_footholds: LegsAttr, legs_qpos_idx: LegsAttr, legs_qvel_idx: LegsAttr, tau: LegsAttr, optimize_swing: int, best_sample_freq: float, nmpc_joints_pos, nmpc_joints_vel, nmpc_joints_acc)
    reset(self, initial_feet_pos: LegsAttr)
```

### controllers/quadruped_controller/mpc_quad_wb/foothold_reference_generator.py

Source: [foothold_reference_generator.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_wb/foothold_reference_generator.py).

```python
class FootholdReferenceGenerator
    __init__(self, stance_time: float, lift_off_positions: LegsAttr, vel_moving_average_length=20, hip_height: float=None)
    compute_footholds_reference(self, com_position: np.ndarray, base_ori_euler_xyz: np.ndarray, base_xy_lin_vel: np.ndarray, ref_base_xy_lin_vel: np.ndarray, hips_position: LegsAttr, com_height_nominal: np.float32)
    update_lift_off_positions(self, previous_contact, current_contact, feet_pos, legs_order)
```

### controllers/quadruped_controller/mpc_quad_wb/periodic_gait_generator.py

Source: [periodic_gait_generator.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_wb/periodic_gait_generator.py).

```python
class PeriodicGaitGenerator
    __init__(self, duty_factor, step_freq, gait_type: GaitType, horizon)
    reset(self)
    run(self, dt, new_step_freq)
    set_phase_signal(self, phase_signal: np.ndarray, init: np.ndarray | None=None)
    @property
    phase_signal(self)
    compute_contact_sequence(self, contact_sequence_dts, contact_sequence_lenghts)
```

### controllers/quadruped_controller/mpc_quad_wb/swing_trajectory_controller.py

Source: [swing_trajectory_controller.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_wb/swing_trajectory_controller.py).

```python
class SwingTrajectoryController
    __init__(self, step_height: float, swing_period: float, position_gain_fb: np.ndarray, velocity_gain_fb: np.ndarray, generator: str)
    regenerate_swing_trajectory_generator(self, step_height: float, swing_period: float)
    compute_swing_control(self, leg_id, q_dot, J, J_dot, lift_off, touch_down, foot_pos, foot_vel, h, mass_matrix)
    update_swing_time(self, current_contact, legs_order, dt)
    check_apex_condition(self, current_contact, interval=0.02)
    check_full_stance_condition(self, current_contact)
```

### controllers/quadruped_controller/mpc_quad_wb/terrain_estimator.py

Source: [terrain_estimator.py](simulation_and_control/simulation_and_control/controllers/quadruped_controller/mpc_quad_wb/terrain_estimator.py).

```python
class TerrainEstimator
    __init__(self)
    compute_terrain_estimation(self, base_position: np.ndarray, yaw: float, feet_pos: dict, current_contact: np.ndarray)
```

### controllers/servo_motor.py

Source: [servo_motor.py](simulation_and_control/simulation_and_control/controllers/servo_motor.py).

```python
class MotorCommands
    __init__(self, ctrl_value=np.array([]), control_list=[])
    SetControlCmd(self, ctrl_value, control_list)
class ServoMotorModel
    __init__(self, n_motors, kp=60, kd=1, motor_control_mod='position', torque_limits=None, friction_torque=False, friction_coefficient=0.0, elastic_torque=False, elastic_coefficient=0.0, motor_load=False, motor_load_coefficient=0.0)
    set_strength_ratios(self, ratios)
    set_motor_gains(self, kp, kd)
    get_motor_gains(self)
    compute_torque(self, motor_commands, cur_q, cur_qdot, prev_qdotdot, M)
```

### sim/pybullet_robot_interface.py

Source: [pybullet_robot_interface.py](simulation_and_control/simulation_and_control/sim/pybullet_robot_interface.py).

```python
class EmptyObj
    # Attribute container with no methods.
def MapToMinusPiToPi(angles)
class SimRobot
    __init__(self, pybullet_client, conf_file_json, index, config_file_path_ext: str=None)
    _UrdfPath(self, index, config_file_path_ext: str=None)
    _LoadPybulletURDF(self, urdf_file, pybullet_client, index)
    _BuildJointNameToIdAndActiveJoint(self, pybullet_client)
    _RemoveDefaultJointDamping(self, pybullet_client)
    _RemoveURDFJointDampingAndFriction(self, pybullet_client)
    _BuildFeetJointIDAndForceSensors(self, pybullet_client, index)
    _buildLinkNameToId(self, pybullet_client)
    get_link_id_from_name(self, link_name: str)
    get_pybullet_bot_index(self)
    SetFootFriction(self, pybullet_client, foot_friction)
    SetFootRestitution(self, pybullet_client, foot_restitution)
    _CreateRackConstraint(self, init_position, init_orientation, pybullet_client)
    _GetDefaultInitPosition(self)
    _GetDefaultInitOrientation(self)
    _GetLinkIdByName(self, pybullet_client, link_name)
    getNameActiveJoints(self, pybullet_client)
class SimInterface
    __init__(self, conf_file_name: str, conf_file_path_ext: str=None, use_gui: bool=True)
    LoadEnv(self, env_script_name)
    ReceiveObservation(self)
    Step(self, action, unseless_control_mode=None)
    _StepInternal(self, action)
    ApplyAction(self, cmd)
    _SetMotorTorqueById(self, motor_id, commands, index=0)
    _SetMotorTorqueByIds(self, motor_ids, commands, index=0)
    _SetDesiredMotorAngleByName(self, motor_name, desired_angle, index=0)
    ResetPose(self)
    GetFootContacts(self)
    ComputeMassMatrixRNEA(self, previous_state=False, index=0)
    ComputeMassMatrix(self, previous_state=False, index=0)
    ComputeSplitMassMatrix(self, M_big)
    ComputeCoriolisAndGravityForces(self, previous_state=False, index=0)
    ComputeGravity(self, base_velocity_in_base_frame=False, previous_state=False, index=0)
    ComputeCoriolis(self, base_velocity_in_base_frame=False, previous_state=False, index=0)
    DirectDynamicsActuatedNoContact(self, tau, previous_state=False, index=0)
    SkewSymmetric(self, vector)
    TransformWorld2Body(self, world_value, index=0)
    TransformBody2World(self, body_value, index=0)
    TransformAngularVelocityToLocalFrame(self, angular_velocity, orientation)
    quaternion_multiply(Q1, Q2)
    GetPyBulletClient(self)
    GetTimeStep(self)
    GetTimeSinceReset(self)
    GetLinkPositionAndOrientation(self, link_name, joint_or_com, index=0)
    GetFloatingBaseLinkPositionAndOrientation(self, index=0)
    GetConfInitPosition(self, index)
    GetConfInitOrientation(self, index)
    GetInitMotorAngles(self, index=0)
    GetInitMotorVelocities(self, index=0)
    ExtractMotorAngles(self, index)
    GetMotorAngles(self, index)
    ExtractMotorVelocities(self, index)
    GetMotorVelocities(self, index)
    ComputeMotorAccelerationTMinusOne(self, index)
    GetMotorAccelerationTMinusOne(self, index)
    GetBasePosition(self, index=0)
    GetBaseOrientation(self, index=0)
    GetBaseLinVelocity(self, index=0)
    ComputePdot(self, index=0)
    ComputeBaseLinAccelerationTMinusOne(self, index=0)
    GetBaseLinVelocityBodyFrame(self, index=0)
    ComputeBaseLinAccelerationBodyFrameTMinusOne(self, index=0)
    GetBaseAngVelocity(self, index=0)
    ComputeBaseAngAccelerationTMinusOne(self, index=0)
    GetBaseAngVelocityBodyFrame(self, index=0)
    ComputeBaseAngAccelerationBodyFrameTMinusOne(self, index=0)
    ComputeBaseVelocitiesBodyFrame(self, index=0)
    GetBaseVelocitiesBodyFrame(self, index=0)
    ComputeSystemStateAccelerationTMinusOne(self, base_frame=True, index=0)
    GetGravVecBodyFrame(self, index=0)
    ComputeBaseRollPitchYaw(self, index=0)
    GetMotorTorques(self, index=0)
    GetSystemState(self, fixed_base=False, base_vel_base_frame=False, index=0)
    GetSystemStateInternal(self, fixed_base=False, base_vel_base_frame=False, index=0)
    GetSystemPreviousStateInternal(self, fixed_base=False, base_vel_base_frame=False, index=0)
    GetAllObservationIdeal(self)
    GetAllObservation(self)
    GetActionDimension(self, index=0)
    GetMassLink(self, link_name, index=0)
    GetTotalMassFromUrdf(self, index=0)
    GetInertiaLink(self, link_name, index=0)
    SetMassLink(self, link_name, mass, index=0)
    SetDiffMassLink(self, link_name, dif_mass, index=0)
    SetInertiaLink(self, link_name, inertia, index=0)
    GetFootLinkIDs(self)
    getFeetGRFLocal(self)
    GetFootGRFLocal(self, foot)
    getFeetGRFWolrd(self)
    GetFootGRFWolrd(self, foot)
    ComputeFootGRF(self)
    GetFootFriction(self, index=0)
    SetFootFriction(self, foot_friction, index=0)
    SetFloorFriction(self, floor_friction)
    GetFootRestitution(self, index=0)
    SetFootRestitution(self, foot_restitution, index=0)
    SetJointFriction(self, joint_frictions, index=0)
    GetNumKneeJoints(self, index=0)
    _AddSensorNoise(self, sensor_values, noise_stdev)
    SetMotorGains(self, kp, kd, index=0)
    GetMotorGains(self, index=0)
    SetTimeSteps(self, simulation_step)
    getNameActiveJoints(self, index=0)
    getDynamicsInfo(self, body_id, link_id=-1)
    GetBotDynamicsInfo(self, index=0)
    GetJointInfo(self, body_id, joint_id)
    GetBotJointsInfo(self, index=0)
    GetBotJointsLimit(self, index=0)
    GetBotJointsVelLimit(self, index=0)
    GetBotJointsTorqueLimit(self, index=0)
    SetjointPosition(self, position, index=0)
    SetfloatingBasePositionAndOrientation(self, position, orientation, index=0)
    KinematicVisualizer(self, q_res, dyn_model, visual_delays=0)
    DynamicSanityCheck1(self, pin_dynamic_model, index=0)
    DynamicSanityCheck2(self, pin_dynamic_model, tau, index=0)
    DynamicSanityCheck3(self, pin_dynamic_model, previous_tau, index=0)
    KinematicSanityCheck(self, index=0)
    calc_inverse_kinematics(self, bot_index: int, target_position, euler_angles_radians: Optional=None, target_frame: Optional[str]=None)
    get_pose(self, bot_index: int, target_frame: str='panda_link8')
```

### utils/SinusoidalRef.py

Source: [SinusoidalRef.py](simulation_and_control/simulation_and_control/utils/SinusoidalRef.py).

```python
class SinusoidalReference
    __init__(self, amplitude, frequency, q_init)
    check_sinusoidal_feasibility(self, sim)
    get_values(self, time)
```

### utils/checking_input_from_config.py

Source: [checking_input_from_config.py](simulation_and_control/simulation_and_control/utils/checking_input_from_config.py).

```python
def adjust_value(flag, value, number_of_elements, vector_name)
```
