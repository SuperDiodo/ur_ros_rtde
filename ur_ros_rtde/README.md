# Robot State Receiver

**Features parameters**:
- `robot_ip`: the ip address that ur_rtde will use for tcp/ip connection.
- `robot_description_package`: ROS2 package containing the URDF of the robot.
- `urdf_file_name`: relative path of the robot URDF in `robot_description_package` package.
- `moveit_config_pkg`: MoveIt! ROS2 config package generated with MoveIt! setup assistant and `urdf_file_name`. (Optional)
- `launch_moveit`: if enabled, `/launch/move_group.launch.py` launch file contained in `moveit_config_pkg` is launched.
- `launch_rviz`: if enabled, an instance of RViz will be launched along with the robot state receiver node. If also `launch_moveit` is enabled, instead of launching an instance of RViz, `/launch/moveit_rviz.launch.py` contained in `moveit_config_pkg` is launched.
- `simulation_only`: if enabled, fake data are made available through topics and services. It can be used to simulate a trajectory in non-realistic environment.

**Data exchange parameters**:
- `rtde_frequency`: frequency (hz) at which ur_rtde will work.
- `data_receiving_frequency`: frequency (hz) at which the robot state receiver will update robot informations as the robot pose, configuration, etc. .
- `simulation_start_robot_state`: if `simulation_only` it's `true`, this will be the initial robot configuration.
- `fake_joint_states_topic`: name of the topic at which fake robot configurations should be published. Instead of publish robot configurations directly to `joint_states` topic, fake robot configurations can be published to `fake_joint_states_topic` and real robot configurations can still be published to `real_joint_states_topic`. Using `publish_fake_joint_state_service` it is possible to change if the data to publish in `joint_states` should be taken from `fake_joint_states_topic` or `real_joint_states_topic`.
- `real_joint_states_topic`: name of the topic at which real robot configurations must be published.
- `wrench_topic`: name of the topic at which wrench, in the flange reference system, must be published.

**Robot configuration params**:
- `robot_joint_names`: names of the robot joints.
- `robot_base_link`: name of the robot base link.
- `robot_flange_link`: name of the robot flange link.

**(Optional) Attached camera configuration params**:
- `camera_calibration_file`: camera calibration file (it should be placed in the `config` folder).
- `calibrated_camera_parent_tf_name`: parent robot link of the camera.
- `calibrated_camera_tf_name`: tf name of the camera that will be published.
- `position_offset`: position offset to add to the calibration matrix of the camera.
- `orientation_offset`: orientation offset to add to the calibration matrix of the camera.

**Messages published in topics**:
- [JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html): `/joint_states`, `/fake_joint_states`, `/real_joint_states`
- [WrenchStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/WrenchStamped.html): `/wrench`
- [Marker](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html): `/visualization_marker`

**Service list**:
- `GetInternalState`: retrieve the digital I/O pins state, the speed slider value and the actual robot payload
  ```python
  # request
  ---
  # result
    bool[] digital_input_state
    bool[] digital_output_state
    float64 speed_slider_value
    float64 payload_value
  ...
  ```
- `GeRobotConfiguration`: retrieve the joint positions and their names
  ```python
  # request
  ---
  # result
    string[] names
    float64[] values
  ...
  ```
- `GetTcpPose`: retrieve the actual TCP pose in the robot reference system
  ```python
  # request
  ---
  # result
    geometry_msgs/Pose pose
  ...
  ```
- `GetWrench`: get the actual wrench (force, torque) from the FT sensor in the TCP reference system
  ```python
  # request
  ---
  # result
    geometry_msgs/Wrench wrench
  ...
  ```
- `PublishFakeJointStates`: it is a `std_srvs/srv/SetBool` service. If `true` is sent then `/joint_states` will be updated with the information from `fake_joint_states_topic` (e.g. `/fake_joint_states`). Otherwise, information from `real_joint_states_topic` (e.g. `/real_joint_states`) will be used. This service can be called to stop the transmission of real data and test simulated trajectories before moving the real robot.
  ```python
  # request
    bool data
  ---
  # result
  ...
  ```

____

# Command Server

**Launch file parameters**:
- `robot_ip`: the ip of the robot that ur_rtde will use for tcp/ip connection.

**Commands ROS2 params**:
In each command ROS2 parameters can be defined. The format is `command_name.param_name`.

**move_until_force**
- `sleep`: sleep time before reading a new force measurement.

**move_until_torque**
- `sleep`: sleep time before reading a new torque measurement.

**execute_trajectory_command**:
- `lookahead_time`, `timestep`, `gain`: servoJ args, more details [here](https://www.universal-robots.com/articles/ur/programming/servoj-command/).
- `parametrization_timestep`: maximum time difference between two waypoints in the trajectory. If two waypoints have a greater diff. in time that this param then new waypoints are added using interpolation.
- `max_allowed_deviation`: maximum allowed deviation (rad) between the actual and the desired robot configurations. If the deviation is greater than this param then the trajectory execution is stopped.

**Action servers list**:

Most of the actions are composed of just `request` field. In future `result` and `feedback` part will be also used.

**Commands** list:
- `send_custom_script_command`: send custom UrScript to the robot.

- `set_digital_pin_command`: set a digital output pin to a specific state-

- `set_payload_command`: set the payload of the robot-

- `set_speed_slider_command`: set the speed slider value of the teach pendant-

- `set_freedrive_command`: activate the freedrive mode of the robot. **free_axes** is an integer vector to activate or deactivate robot movements on X,Y,Z,Roll,Pitch,Yaw axes (1 is activated, 0 is deactivated).

- `reset_force_torque_sensor_command`: reset the FT sensor-

- `move_l_command`: move the robot with linear movements in the tool-space.

- `move_j_command`: move the robot with linear movements in the joint-space.

- `move_l_relative_command`: relative movement, linear in the tool-space, of the robot with respect to the actual pose.

- `move_j_relative_command`: relative movement, linear in the joint-space, of the robot with respect to the actual configuration.

- `move_until_contact_command`: move the TCP along a given direction until a contact is detected and then retract the robot at the last non-colliding configuration. It is possible to define a **direction** of the contact to detect. If direction is set to all -1 values all possible contacts are detected. 

- `move_until_force_command`: move the TCP given XYZ offset until a force is detected. It is possible to define a **direction** of the force and a minimum value.

- `move_until_torque_command`: move the TCP given XYZ offset until a torque is detected. It is possible to define a **direction** of the torque and a minimum value.

- `execute_path_command`: execute a path composed of several waypoints. Each waypoint can be a MoveL (l or L) or MoveJ (j or J) movement. When passing multiple waypoint of the same type it is possible to use the **blend** values (see *Blend Radii* in this [guide](https://www.universal-robots.com/articles/ur/robot-care-maintenance/important-deployment-points/)). They allows to smooth the transition between two MoveX movements.

- `execute_trajectory_command`: execute a trajectory composed of at least two robot configurations. The robot will perform a first `move_l` towards the first robot configuration and then start the execution. Deceleration is used to stop at the end of trajectory. Acceleration and speed parameters are used to estimate a speed profile of the robot and interpolate waypoints.

**Dashboard commands** descriptions are straightforward given their names.