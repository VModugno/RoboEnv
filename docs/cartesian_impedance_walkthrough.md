# Cartesian impedance controller code walkthrough

Ten slide contents for a PowerPoint walkthrough of `tests/cartesian_impedance_controller.py`. The audience is familiar with Python and is learning how the simulator, analytical model, and controller work together. Code excerpts preserve the script's names and behavior, with long calls wrapped for readability. Speaker notes provide the detail to explain aloud. No installation content is included.

## Slide 1  Cartesian position control of the Panda arm

### Slide content

The example moves `panda_link8` along a sinusoidal Cartesian position reference using commands for seven joint torques.

- `SimInterface` runs the robot in PyBullet and reads its state.
- `PinWrapper` supplies forward kinematics, Jacobians, and dynamics.
- `ImpedanceController` computes joint torques from position error and measured velocity.
- `MotorCommands` carries the torques to the simulator.

### Speaker notes

Introduce Cartesian position as the end effector's x, y, and z coordinates. The reference specifies where this frame should move, while the command specifies seven motor torques. This example calls the default position mode of `ImpedanceController`. It does not call inverse kinematics or `feedback_lin_ctrl` to generate desired joint angles.

Source: [example script](../tests/cartesian_impedance_controller.py), especially its controller call around line 109.

## Slide 2  Simulation configuration

### Slide content

```python
conf_file_name = "pandaconfig.json"
root_dir = os.path.dirname(os.path.abspath(__file__))
name_current_directory = "tests"
root_dir = root_dir.replace(name_current_directory, "")

sim = pb.SimInterface(
    conf_file_name,
    conf_file_path_ext=root_dir,
    use_gui=not headless,
)
```

- The configuration identifies the Panda model and initial state.
- `root_dir` locates the repository's `configs/` and `models/` folders.
- `use_gui` chooses a visible simulation or headless execution.

### Speaker notes

`SimInterface` loads a JSON configuration, creates its PyBullet client, loads the robot and environment, and collects the initial observation. In this file, `root_dir` starts at `tests/`, and the string replacement removes that directory name. A path-parent operation would be more robust, but the excerpt shows the current script.

The module reads `ROBOENV_HEADLESS` before `main()`. Headless mode also selects the noninteractive Matplotlib backend, skips the per-step sleep, and returns before plotting. These are execution settings, not changes to the controller law.

Source: [script lines 8 to 27](../tests/cartesian_impedance_controller.py).

## Slide 3  Analytical model and joint ordering

### Slide content

```python
ext_names = sim.getNameActiveJoints()
ext_names = np.expand_dims(np.array(ext_names), axis=0)
source_names = ["pybullet"]

dyn_model = PinWrapper(
    conf_file_name, "pybullet", ext_names,
    source_names, False, 0, root_dir,
)
num_joints = dyn_model.getNumberofActuatedJoints()
```

- Joint names connect simulator coordinates to model coordinates.
- `ext_names` has shape `(1, number_of_joints)` for one data source.
- `False` disables the separate Pinocchio visualizer. `0` selects the first robot.

### Speaker notes

The simulator and analytical model are separate objects built from the same configuration. PyBullet simulates motion. Pinocchio computes quantities used by the control law. Joint names allow `PinWrapper` to construct permutations where the two models use different joint orders.

The constructor's positional arguments are configuration, simulator name, external joint-name array, data-source names, visualizer flag, robot index, and asset root. The impedance implementation multiplies its Jacobian directly by measured joint velocity and returns torque without a final permutation. This example therefore relies on matching simulator and analytical joint orders. For another robot, check or explicitly reconcile the orders.

Source: [script lines 29 to 37](../tests/cartesian_impedance_controller.py) and [PinWrapper](../simulation_and_control/simulation_and_control/controllers/pin_wrapper.py).

## Slide 4  Cartesian reference trajectory

### Slide content

```python
controlled_frame_name = "panda_link8"
init_joint_angles = sim.GetInitMotorAngles()
init_cartesian_pos, init_R = dyn_model.ComputeFK(
    init_joint_angles, controlled_frame_name,
)

amplitude = np.array([0, 0.1, 0])
frequency = np.array([0.4, 0.5, 0.4])
ref = SinusoidalReference(amplitude, frequency, init_cartesian_pos)
```

- Forward kinematics finds the frame's initial Cartesian position.
- Only y moves: amplitude `0.1 m`, frequency `0.5 Hz`, period `2 s`.
- The reference starts `0.1 m` below the initial y coordinate.

### Speaker notes

The three array entries correspond to Cartesian x, y, and z, despite comments in the script referring to joints. Zero amplitudes keep x and z constant. The active y reference has a peak-to-peak displacement of 0.2 meters.

`SinusoidalReference` uses phase `-pi/2`, so `p_d(t) = p_initial + amplitude * sin(2*pi*frequency*t - pi/2)`. At time zero its velocity is zero, but its y target differs from the initial frame position. This creates an initial position error. `init_R` contains the frame rotation but the example does not use it for orientation tracking.

Source: [script lines 39 to 68](../tests/cartesian_impedance_controller.py) and [SinusoidalReference](../simulation_and_control/simulation_and_control/utils/SinusoidalRef.py).

## Slide 5  State measurements and controller inputs

### Slide content

```python
kp = 1000
kd = 100
tau_ext = np.zeros(6)

q_mes = sim.GetMotorAngles(0)
qd_mes = sim.GetMotorVelocities(0)
p_d, pd_d = ref.get_values(current_time)
```

| Variable | Meaning |
| --- | --- |
| `q_mes`, `qd_mes` | Seven measured joint positions and velocities. |
| `p_d` | Desired Cartesian position with three components. |
| `kp`, `kd` | Cartesian position feedback and velocity damping gains. |
| `tau_ext` | Assumed external wrench: three force and three moment entries. |

### Speaker notes

Index zero selects the first robot. These getters can include noise or delay when the configuration enables them. `tau_ext` is an assumed zero wrench, not a wrench read from a sensor. Position mode uses only its first three force entries.

The reference also generates `pd_d`, but `ImpedanceController` has no desired Cartesian velocity input. It damps measured Cartesian velocity toward zero. The loop computes `qdd_est`, but the example does not use that estimate in the controller. Likewise, `kp_pos`, `kp_ori`, and the imported `feedback_lin_ctrl` are unused in this example.

Source: [script lines 75 to 109](../tests/cartesian_impedance_controller.py).

## Slide 6  Cartesian position error and velocity

### Slide content

Inside `ImpedanceController`:

```python
dyn_model.ComputeJacobian(
    q_mes, controlled_frame_name, "local_global",
)
J = dyn_model.res.J[:3, :]
xd_ee = J @ qd_mes
x_ee, _ = dyn_model.ComputeFK(q_mes, controlled_frame_name)
```

- `J` has shape `(3, 7)` and maps joint velocities to Cartesian velocity.
- `x_ee` is the current frame position in world coordinates.
- The feedback term is `kp * (p_d - x_ee) - kd * xd_ee`.

### Speaker notes

The full Jacobian has six rows: linear motion followed by angular motion. Position mode selects its first three rows. `local_global` means the Jacobian is evaluated at the frame origin and expressed in world-aligned axes, consistent with the position reference.

Position feedback pulls the frame toward the target. Velocity damping reduces motion and oscillation. Scalar gains create diagonal three-dimensional gain matrices, so each Cartesian axis receives the same gain. The controller uses the robot model to express this feedback as motor torques.

The displayed code condenses the function's position branch. It is explanatory code from the called function, rather than another block executed directly in `main()`.

Source: [ImpedanceCtrl.py](../simulation_and_control/simulation_and_control/controllers/ImpedanceCtrl.py), lines 21 to 43.

## Slide 7  Dynamics compensation and joint torque

### Slide content

Inside `ImpedanceController`:

```python
dyn_model.ComputeAllTerms(q_mes, qd_mes)
M = dyn_model.res.M
S = dyn_model.res.c
g = dyn_model.res.g

Mx = np.linalg.pinv(J).T @ M @ np.linalg.pinv(J)
Sx = np.linalg.pinv(J).T @ S
gx = np.linalg.pinv(J).T @ g
```

- `M` is joint-space inertia. `S` and `g` are velocity-dependent and gravity forces.
- `Mx` approximates inertia in the Cartesian task space.
- The controller maps the Cartesian expression into seven joint torques with `J.T`.

### Speaker notes

`ComputeAllTerms` updates the cached results on `dyn_model.res`; it does not return these arrays directly. The implementation uses a pseudoinverse-based task inertia, rather than the alternative operational-space formula involving `J @ inv(M) @ J.T`.

The final calculation is:

```python
a = np.linalg.inv(Mx) @ (
    tau_ext - D @ xd_ee + P @ (xdes - x_ee)
)
u = J.T @ (Mx @ a + Sx + gx - tau_ext)
return u
```

Substituting the definition of `a` into the second expression algebraically cancels the explicit `tau_ext` terms and `Mx` factors, assuming the inverse calculation succeeds. For this example, the effective expression is `J.T @ (P @ (xdes - x_ee) - D @ xd_ee + Sx + gx)`. Explain the implemented position feedback and dynamics compensation without claiming the script measures external contact or enforces a separate desired Cartesian inertia.

Source: [ImpedanceCtrl.py](../simulation_and_control/simulation_and_control/controllers/ImpedanceCtrl.py), lines 46 to 62.

## Slide 8  Torque commands and simulation stepping

### Slide content

```python
tau_cmd = ImpedanceController(
    dyn_model, controlled_frame_name,
    q_mes, qd_mes, p_d, kp, kd, tau_ext,
)
cmd.SetControlCmd(tau_cmd, ["torque"] * 7)
sim.Step(cmd, "torque")

current_time += time_step
```

- The controller returns one torque per Panda motor.
- `SetControlCmd` selects torque mode for all seven motors.
- `Step` applies the torques, advances physics, and refreshes observations.
- The next loop iteration uses the updated robot state and reference time.

### Speaker notes

`MotorCommands` is a command container. `ServoMotorModel` reads its `ctrl_cmd` and `control_list`, converts any configured motor effects, and supplies torques for PyBullet. Position and velocity motor modes also eventually produce torque, but this example already calculates torque explicitly.

The second `"torque"` argument to `Step` is an ignored compatibility argument. The active modes come from `cmd.control_list`, set by `SetControlCmd`. The script hardcodes seven motors for this Panda model.

The local `current_time` advances by the simulation time step. `time.sleep(0.01)` in GUI mode slows playback in wall-clock time without changing the configured physics time step. The example has no control-loop sleep in headless mode.

Source: [script lines 109 to 143](../tests/cartesian_impedance_controller.py) and [SimInterface.Step](../simulation_and_control/simulation_and_control/sim/pybullet_robot_interface.py).

## Slide 9  Logged data and plot interpretation

### Slide content

```python
q_mes_all.append(q_mes)
qd_mes_all.append(qd_mes)
q_d_all.append(q_des)
qd_d_all.append(qd_des_clip)
```

- The script logs joint position and velocity before each physics step.
- `q_des` remains the initial joint posture. `qd_des_clip` remains zero.
- The final plots compare joint motion with these fixed values.
- Cartesian tracking requires logging both `p_d` and the measured frame position.

### Speaker notes

The position and velocity graphs are joint plots, although the control objective is Cartesian position. The fixed desired joint values do not describe the moving Cartesian reference. Multiple joint motions can produce the same frame position, so these graphs alone do not quantify Cartesian tracking error.

For a tracking plot, record the actual Cartesian position from `ComputeFK(q_mes, controlled_frame_name)`, the target `p_d`, and a time value from the same instant. Plot x, y, and z versus time, or plot `p_d - measured_position`. This is a suggested extension, not something the current script already does.

The current measurement variables come from before `Step`, while logging occurs afterward. The unchanged local variables still record that earlier measurement. Headless execution returns before all Matplotlib plotting.

Source: [script lines 55 to 57 and 130 to 173](../tests/cartesian_impedance_controller.py).

## Slide 10  The complete control cycle

### Slide content

```python
q_mes = sim.GetMotorAngles(0)
qd_mes = sim.GetMotorVelocities(0)
p_d, pd_d = ref.get_values(current_time)

tau_cmd = ImpedanceController(
    dyn_model, controlled_frame_name,
    q_mes, qd_mes, p_d, kp, kd, tau_ext,
)
cmd.SetControlCmd(tau_cmd, ["torque"] * 7)
sim.Step(cmd, "torque")
current_time += time_step
```

Each iteration reads joint state, evaluates the Cartesian reference, computes torque, and advances the simulated robot.

The default controller call tracks position. It uses measured velocity for damping and assumes zero external wrench.

### Speaker notes

Use this final excerpt to connect the earlier slides. The joint state describes what the robot is doing, the Cartesian reference describes the desired task, and the controller connects the two through kinematics and dynamics. No desired joint trajectory is required in this direct torque pipeline.

The original loop also checks the PyBullet keyboard for `q`, optionally displays the Pinocchio model, and exits at a configured step cap. The cap check occurs after `Step`, using the local time before its increment, so it is not an exact count of all executed steps. There is no automatic stop at the configuration's experiment duration. Default GUI execution continues until an exit condition fires.

Keep the walkthrough focused on the function's position mode. The orientation and combined branches are incomplete, and the script only invokes position mode. Joint limit queries print metadata but the script does not explicitly enforce limits or call the commented-out feasibility check.

Source: [script lines 95 to 143](../tests/cartesian_impedance_controller.py).
