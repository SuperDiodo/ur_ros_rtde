
# Robot State Receiver

**Customizable variables**:
- `robot_description_package`: ROS2 package containing the URDF of the robot.
- `urdf_file_name`: relative path of the robot URDF in `robot_description_package` package.
- `moveit_config_pkg`: MoveIt! ROS2 config package generated with MoveIt! setup assistant and `urdf_file_name`. (Optional)
- `launch_moveit`: if enabled, `/launch/move_group.launch.py` launch file contained in `moveit_config_pkg` is launched.
- `launch_rviz`: if enabled, an instance of RViz will be launched along with the robot state receiver node. If also `launch_moveit` is enabled, instead of launching an instance of RViz, `/launch/moveit_rviz.launch.py` contained in `moveit_config_pkg` is launched.

**Data receiving parameters**:
- `robot_ip`: the ip of the robot that ur_rtde will use for tcp/ip connection.
- `rtde_frequency`: frequency (hz) at which ur_rtde will work.
- `data_receiving_frequency`: frequency (hz) at which the robot state receiver will update robot informations as the robot pose, configuration, etc. .
- `simulation_only`: if enabled, fake data are made available through topics and services. It can be used to simulate a trajectory in non-realistic environment.
- `simulation_start_robot_state`: if `simulation_only` it's `true`, this will be the initial robot configuration.
- `fake_joint_states_topic`: name of the topic at which fake robot configurations should be published. Instead of publish robot configurations directly to `joint_states` topic, fake robot configurations can be published to `fake_joint_states_topic`` and real robot configurations can still be published to `real_joint_states_topic`. Using `publish_fake_joint_state_service` it is possible to change if the data to publish in `joint_states` should be taken from `fake_joint_states_topic` or `real_joint_states_topic`.
- `real_joint_states_topic`: name of the topic at which real robot configurations must be published.
- `wrench_topic`: name of the topic at which wrench, in the flange reference system, must be published.
- `publish_force_sensor_markers`: if enabled arrow markers related to the force read by FT sensor will be published in `visualization_marker` topic.

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
  ```
  # request
  ---
  # result
  bool success
  bool[] digital_input_state
  bool[] digital_output_state
  float64 speed_slider_value
  float64 payload_value
  ```
- `GeRobotConfiguration`: retrieve the joint positions and their names
  ```
  # request
  ---
  # result
  bool success
  string[] names
  float64[] values
  ```
- `GetTcpPose`: retrieve the actual TCP pose in the robot reference system
  ```
  # request
  ---
  # result
  bool success
  geometry_msgs/Pose pose
  ```
- `GetWrench`: get the actual wrench (force, torque) from the FT sensor in the TCP reference system
  ```
  # request
  ---
  # result
  bool success
  geometry_msgs/Wrench wrench
  ```

____

# Command Server

**Customizable variables**:
- `robot_ip`: the ip of the robot that ur_rtde will use for tcp/ip connection.
- `receiver_freq`: frequency (hz) at which ur_rtde receiving interface will work. In the command server the receiving interface is used to perform checks while sending commands to the robot.

**Trajectory execution params**:
- `parametrization_timestep`: maximum time difference between two waypoints in the trajectory. If two waypoints have a greater diff. in time that this param then new waypoints are added using interpolation.
- `deviation_check_freq`: frequency (hz) at which the actual robot configuration and the desired robot configuration are compared. 
- `max_deviation`: maximum possible deviation (rad) between the actual and the desired robot configurations. If the devition is greater than this param then the trajectory execution is stopped.

**Action servers list**:
- `set_digital_pin_command`: set a digital output pin to a specific state
  ```
    {
        int8 pin
        bool state
    }
  ```

- `set_deposit_command`: set state of the gripper suction
  ```
    {
        bool state
    }
  ```
- `set_suction_command`: set state of the gripper deposit
  ```
    {
        bool state
    }
  ```
- `set_payload_command`: set the payload of the robot
  ```
    {
        float64 mass
        geometry_msgs/Point center_of_gravity
    }
  ```
- `set_speed_slider_command`: set the speed slider value of the teach pendant
  ```
    {
        float64 speedslider
    }
  ```
- `reset_force_torque_sensor_command`: reset the FT sensor
  ```
    {
    }
  ```
- `move_l_command`: move the robot with linear movements in the tool-space
  ```
    {
        geometry_msgs/Point position
        geometry_msgs/Point orientation
        float64 speed
        float64 acceleration
    }
  ```
- `move_j_command`: move the robot with linear movements in the joint-space
  ```
    {
        float64[] joint_position
        float64 speed
        float64 acceleration
    }
  ```
- `move_l_relative_command`: relative movement, linear in the tool-space, of the robot with respect to the actual pose
  ```
    {
        geometry_msgs/Point position
        geometry_msgs/Point orientation
        float64 speed
        float64 acceleration
    }
  ```
- `move_j_relative_command`: relative movement, linear in the joint-space, of the robot with respect to the actual configuration
  ```
    {
        float64[] joint_position
        float64 speed
        float64 acceleration
    }
  ```
- `move_until_contact_command`: move the TCP along a given direction until a contact is detected and then retract the robot at the last non-colliding configuration. It is possible to define a **direction** of the contact to detect. If direction is set to all -1 values all possible contacts are detected. 
  ```
    {
        float64[] toolspeed
        float64[] direction
        float64 acceleration
    }
  ```
- `move_until_force_command`: move the TCP given XYZ offset until a force is detected. It is possible to define a **direction** of the force and a minimum value.
  ```
    {
        float64[] tool_position_movement
        float64[] forces_to_consider
        float64 force_th
        float64 acceleration
        float64 speed
        float64 deceleration
    }
  ```
- `move_until_torque_command`: move the TCP given XYZ offset until a torque is detected. It is possible to define a **direction** of the torque and a minimum value.
  ```
    {
        float64[] tool_position_movement
        float64[] torques_to_consider
        float64 torque_th
        float64 acceleration
        float64 speed
        float64 deceleration
    }
  ```

- `execute_path_command`: execute a path composed of several waypoints. Each waypoint can be a MoveL (l or L) or MoveJ (j or J) movement. When passing multiple waypoint of the same type it is possible to use the **blend** values (see *Blend Radii* in this [guide](https://www.universal-robots.com/articles/ur/robot-care-maintenance/important-deployment-points/)). They allows to smooth the transition between two MoveX movements.

  ```
    {
        Vector[] waypoints
        float64[] speed
        float64[] acceleration
        float64[] blend
        string[] move_type
    }
  ```

- `execute_trajectory_command`: execute a trajectory composed of at least two robot configurations. The robot will perform a first `move_l` towards the first robot configuration and then start the execution. Deceleration is used to stop at the end of trajectory. Acceleration and speed parameters are used to estimate a speed profile of the robot and interpolate waypoints.
  ```
    {
        Vector[] trajectory
        float64 speed
        float64 acceleration
        float64 deceleration
    }
  ```
