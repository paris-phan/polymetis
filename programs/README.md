## Robot Interface

The `RobotInterface` class provides a high-level interface for controlling robot movements and getting robot state information.

### Initialization
```python
robot = RobotInterface(
    ip_address="localhost",  # IP address of the gRPC server
    port=50051,             # Port number
    time_to_go_default=1.0, # Default time for movements
    use_grav_comp=True      # Whether to use gravity compensation
)
```

### State Information Methods
- `get_robot_state()`: Returns the latest RobotState
- `get_joint_positions()`: Returns current joint positions as torch.Tensor
- `get_joint_velocities()`: Returns current joint velocities as torch.Tensor
- `get_ee_pose()`: Returns current end-effector position and orientation (quaternion)

### Movement Methods

#### Joint Space Control
- `move_to_joint_positions(positions, time_to_go=None, delta=False, Kq=None, Kqd=None)`:
  - Moves robot to specified joint positions
  - `positions`: Target joint positions
  - `time_to_go`: Time to complete movement (uses adaptive timing if None)
  - `delta`: If True, positions are relative to current pose
  - `Kq`, `Kqd`: Joint P and D gains

- `go_home()`: Moves robot to predefined home position

#### Cartesian Space Control
- `move_to_ee_pose(position, orientation=None, time_to_go=None, delta=False, Kx=None, Kxd=None, op_space_interp=True)`:
  - Moves end-effector to specified position and orientation
  - `position`: Target 3D position
  - `orientation`: Target orientation as quaternion
  - `op_space_interp`: If True, interpolates in operational space

### Continuous Control Methods

#### Joint Impedance Control
- `start_joint_impedance(Kq=None, Kqd=None, adaptive=True)`:
  - Starts non-blocking joint impedance controller
  - `update_desired_joint_positions(positions)`: Updates target positions

#### Cartesian Impedance Control
- `start_cartesian_impedance(Kx=None, Kxd=None)`:
  - Starts non-blocking Cartesian impedance controller
  - `update_desired_ee_pose(position=None, orientation=None)`: Updates target pose

#### Joint Velocity Control
- `start_joint_velocity_control(joint_vel_desired, hz=None, Kq=None, Kqd=None)`:
  - Starts non-blocking joint velocity controller
  - `update_desired_joint_velocities(velocities)`: Updates target velocities

### Utility Methods
- `solve_inverse_kinematics(position, orientation, q0, tol=1e-3)`:
  - Computes inverse kinematics for given end-effector pose
  - Returns joint positions and success flag

## Gripper Interface

The `GripperInterface` class provides control over the robot's gripper.

### Initialization
```python
gripper = GripperInterface(
    ip_address="localhost",  # IP address of the gRPC server
    port=50052              # Port number
)
```

### State Information
- `get_state()`: Returns current gripper state

### Control Methods

#### Position Control
- `goto(width, speed, force, blocking=True)`:
  - Commands gripper to specific width
  - `width`: Target width
  - `speed`: Movement speed
  - `force`: Maximum force
  - `blocking`: Whether to wait for completion

#### Grasp Control
- `grasp(speed, force, grasp_width=0.0, epsilon_inner=-1.0, epsilon_outer=-1.0, blocking=True)`:
  - Commands gripper to perform grasp
  - `speed`: Movement speed
  - `force`: Maximum force
  - `grasp_width`: Target width for grasp
  - `epsilon_inner/outer`: Tolerance for grasp width
  - `blocking`: Whether to wait for completion

## Notes
- All position/velocity values are expected as torch.Tensor
- Time values are in seconds
- Force values are in Newtons
- Width values are in meters
- The interfaces use gRPC for communication with the robot server
- Error handling is built into the interfaces with appropriate error messages

robot_interface.py


gripper_interface.py
