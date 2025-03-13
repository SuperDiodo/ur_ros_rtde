# Robot State Receiver

#### ROS parameters
- `robot_ip`: the robot ip address.
- `robot_description_package`: ROS2 package containing the URDF of the robot.
- `urdf_file_name`: relative path of the robot URDF in `robot_description_package` package.
- `moveit_config_pkg`: MoveIt! ROS2 config package generated with MoveIt! setup assistant and `urdf_file_name`. (Optional)
- `launch_moveit`: if enabled, `/launch/move_group.launch.py` launch file contained in `moveit_config_pkg` is launched.
- `launch_rviz`: if enabled, an instance of RViz will be launched along with the robot state receiver node. If also `launch_moveit` is enabled, instead of launching an instance of RViz, `/launch/moveit_rviz.launch.py` contained in `moveit_config_pkg` is launched.
- `simulation_only`: if enabled, fake data are made available through topics and services. It can be used to simulate a trajectory in non-realistic environment.
- `rtde_frequency`: frequency (hz) at which `ur_rtde` will work.
- `data_receiving_frequency`: frequency (hz) at which the robot state receiver will update robot informations as the robot pose, configuration, etc.
- `simulation_start_robot_state`: if `simulation_only` it's `true`, this will be the initial robot configuration.

#### Robot state published in topics and accessible with services
- [JointState](https://docs.ros.org/en/humble/p/geometry_msgs/msg/JointState.html): `/joint_states`, `/ur_ros_rtde/get_joint_state` 
- [Pose](https://docs.ros.org/en/humble/p/geometry_msgs/msg/Pose.html): `/tcp_pose`, `/ur_ros_rtde/get_tcp_pose`
- [WrenchStamped](https://docs.ros.org/en/humble/p/geometry_msgs/msg/WrenchStamped.html): `/wrench`, `/ur_ros_rtde/get_wrench`
- [IOState](../ur_ros_rtde_msgs/msg/IOState.msg): `/io_state`, `/ur_ros_rtde/get_io_state`

---

# Command Server

#### ROS parameters
- `robot_ip`: the robot ip address.
- `command_server.plugins_blacklist`: a list of plugin names to ignore.

#### Commands list

- `send_custom_script_command`: send custom UrScript to the robot.

- `set_digital_pin_command`: set a digital output pin to a specific state.

- `set_payload_command`: set the payload of the robot.

- `set_speed_slider_command`: set the speed slider value of the teach pendant.

- `set_freedrive_command`: activate the freedrive mode of the robot. **free_axes** is an integer vector to activate or deactivate robot movements on X,Y,Z,Roll,Pitch,Yaw axes (1 is activated, 0 is deactivated).

- `reset_force_torque_sensor_command`: reset the FT sensor.

- `move_l_command`: move the robot with linear movements in the tool space.

- `move_j_command`: move the robot with linear movements in the joint space.

- `move_l_relative_command`: relative movement, linear in the tool space, of the robot with respect to the actual pose.

- `move_j_relative_command`: relative movement, linear in the joint space, of the robot with respect to the actual configuration.

- `move_until_contact_command`: move the TCP along a given direction until a contact is detected and then retract the robot at the last non-colliding configuration. It is possible to define a **direction** of the contact to detect. If direction is set to all -1 values all possible contacts are detected.

- `move_until_force_command`: move the TCP given XYZ offset until a force is detected. It is possible to define a **direction** of the force and a minimum value.

- `move_until_torque_command`: move the TCP given XYZ offset until a torque is detected. It is possible to define a **direction** of the torque and a minimum value.

- `execute_path_command`: execute a path composed of several waypoints. Each waypoint can be a MoveL (l or L) or MoveJ (j or J) movement. When passing multiple waypoint of the same type it is possible to use the **blend** values (see *Blend Radii* in this [guide](https://www.universal-robots.com/articles/ur/robot-care-maintenance/important-deployment-points/)). They allows to smooth the transition between two MoveX movements.

- `execute_trajectory_command`: execute a trajectory composed of at least two robot configurations. The robot will perform a first `move_l` towards the first robot configuration and then start the execution. Deceleration is used to stop at the end of trajectory. Acceleration and speed parameters are used to estimate a speed profile of the robot and interpolate waypoints.

---

## Dashboard commands

#### ROS parameters
- `robot_ip`: the robot ip address.

#### Commands list
Command descriptions are straightforward given their names.