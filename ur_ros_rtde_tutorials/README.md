# Tutorials for `ur_ros_rtde`

The proposed software is composed of three ROS2 nodes:

- *RobotStateReceiver* ) provides robot data through topics and services (reference file: `ur_ros_rtde/launch/robot_state_receiver.launch.py`).

- *CommandServer* ) discovers and loads `ur_ros_rtde_commands` and `ur_ros_rtde_extension` plugins available in the ROS2 workspace. Each plugin starts a ROS2 action server. Reference files:
  - [command_server.launch.py](../ur_ros_rtde/launch/command_server.launch.py)
  - [ur_ros_rtde/include/ur_ros_rtde/command_base_class.hpp](../ur_ros_rtde/include/ur_ros_rtde/command_base_class.hpp)
  - [ur_ros_rtde/include/ur_ros_rtde/extension_base_class.hpp](../ur_ros_rtde/include/ur_ros_rtde/extension_base_class.hpp)

- *DashboardServer* ) discovers and loads `ur_ros_rtde_dashboard_commands` plugins available in the ROS2 workspace. Each plugin starts a ROS2 action server. Reference files:
  - [ur_ros_rtde/launch/dashboard_server.launch.py](../ur_ros_rtde/launch/dashboard_server.launch.py)
  - [ur_ros_rtde/include/ur_ros_rtde/dashboard_command_base_class.hpp](../ur_ros_rtde/include/ur_ros_rtde/dashboard_command_base_class.hpp)

Examples below demonstrate how to use these nodes and plugins to interact with the robot. This package includes several ready-to-run examples; most can be executed directly from a terminal.  
Alternatively, similar examples are provided as scripts using the helper headers in [`ur_ros_rtde_simple_clients`](ur_ros_rtde_simple_clients).  
Note that these scripts require the `ur_ros_rtde_tutorials` package to be compiled before execution.

## **Example A**: Retrieve robot state

To retrieve the robot state *RobotStateReceiver* must be used.

1. Configure `robot_state_receiver.launch.py`:
    - set ip address with `robot_ip`

2. Launch **robot_state_receiver**:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde robot_state_receiver.launch.py
    ```

At this point, the robot state can be accessed using topics and services.

3. Print joint states:
    ```bash
    # type in a new terminal
    ros2 topic echo /joint_states
    ```

## **Example B**: Visualize the robot in RViz

*RobotStateReceiver* does not require any URDF of the robot, but URDF files are required to visualize the robot in tools like RViz. If a ROS2 description package of a UR robot is already available, skip to step 2 and use that package instead of `simple_ur10e_description`.

1. In `simple_ur10e_description` generate UR10e URDF file from xacro files.
    ```bash
    # generate ur10e urdf
    cd ~/your_path/simple_ur10e_description/urdf
    sh generate_urdf.sh ur10e.xacro ur10e.urdf

    # build again the package to make the urdf visible
    cd <ros_workspace_path>
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to simple_ur10e_description
    ```

2. Configure `robot_state_receiver.launch.py`:
    - set ip address with `robot_ip`
    - set `robot_description_package = "simple_ur10e_description"`
    - set `urdf_file_name = "urdf/ur10e.urdf"`
    - set `launch_rviz = True`

When setting `launch_rviz` to `True` a robot state publisher node is started which will publish TF for each robot link based on the joint states.

3. Launch *RobotStateReceiver*:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde robot_state_receiver.launch.py
    ```
    
    The first time RViz will be empty. To show the robot: 
    1. Add a robot model: `Displays\Add\RobotModel`
    2. Set `world` in `Fixed Frame`
    3. Set `/robot_description` in `RobotModel\Description Topic`
    4. Click on `File\Save config` to save the actual configuration of RViz

## **Example C**: move the robot by using the available action servers

In `ur_ros_rtde` commands are defined as plugins and exposed as action servers. To make available these action servers the *CommandServer* must be started:

1. Configure `command_server.launch.py` setting ip address with `robot_ip`

2. Launch *CommandServer*:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde command_server.launch.py
    ```
3. List the available commands (more info at [ur_ros_rtde](../ur_ros_rtde)):
    ```bash
    # type in a new terminal
    ros2 action list
    ```

The file `command_server.launch.py` can also be used to define a blacklist of plugins which must not be loaded. By default, the plugins implemented in `ur_ros_rtde_gripper_commands` are disabled, since they rely on the presence of specific hardware.

The action servers initialized by the command server can be used to interact with the robot by sending goals to them.
A goal can be sent to an action server by using the terminal or by script, in this example we both cover them.

- **Execute a waypoint-based trajectory**:
  In `trajectory_execution` there is an example in which a trajectory is sent to the robot. The robot (oriented towards Y-axis) will move as shown in the animated image.

  <p align="center">
    <img src="../images/dual_traj.gif">
  </p>

    This example only requires an instance of the *CommandServer* and can be executed as shown below:

    ```bash
    # execute trajectory with script
    ros2 run ur_ros_rtde_tutorials trajectory_execution

    # execute trajectory from terminal
    ros2 action send_goal /ur_ros_rtde/execute_trajectory_command ur_ros_rtde_msgs/action/ExecuteTrajectory
    '{
    "trajectory": [
        {
        "vector": [-1.822409454976217, -1.5724393330016078, 1.772125546132223, -1.7732678852477015, -1.5799019972430628, -0.24573165575136358]
        },
        {
        "vector": [-1.726349178944723, -1.5776134930052699, 1.6113598982440394, -1.9500886402525843, -0.6720364729510706, -0.7341492811786097]
        },
        {
        "vector": [-2.1398356596576136, -2.0075184307494105, 2.0258382002459925, -1.5872675381102503, -1.5326102415667933, -0.5949648062335413]
        },
        {
        "vector": [-1.822409454976217, -1.5724393330016078, 1.772125546132223, -1.7732678852477015, -1.5799019972430628, -0.24573165575136358]
        }
    ],
    "speed": 0.1,
    "acceleration": 0.1,
    "deceleration": 1.0,
    "speed_percentage": 1.0,
    "joint_speed_limits": [2.0944, 2.0944, 3.1416, 3.1416, 3.1416, 3.1416]
    }'
    ```

- **Multiple linear movements while monitoring the robot state**:
  The robot performs small linear movements along the X-axis printing the robot pose after each movement. The robot state is retrieved with services exposed by *RobotStateReceiver*. Commands are sent to the robot with the *CommandServer*.

  1. Configure and launch *RobotStateReceiver*, set `launch_rviz = True` if you want to see the robot moving in RViz:
        ```bash
        # type in a new terminal
        ros2 launch ur_ros_rtde robot_state_receiver.launch.py
        ```

  2. Configure `command_server.launch.py` setting ip address with `robot_ip`.

  3. Launch *CommandServer*:
      ```bash
      # type in a new terminal
      ros2 launch ur_ros_rtde command_server.launch.py
      ```
  4. Run executables:

      **WARNING**! If everything was successfully configured the robot will start moving! Check for possible collisions with the environment!
      
      With `linear_movements` executable MoveL commands are sent to the robot. Starting from the actual pose it will move +10 cm on X axis, then -20 cm on X axis and finally +10 on X axis again.

      ```bash
      # type in a new terminal
      ros2 run ur_ros_rtde_tutorials linear_movements
      ```

## **Example D**: Adding new commands to `ur_ros_rtde`

In `ur_ros_rtde/src/base_commands` and `ur_ros_rtde/src/base_dashboard_commands` there is the implementation of several commands which are exposed as ROS2 plugins. When launching *CommandServer* and *DashboardServer* these plugins are automatically loaded.

In this package and [ur_ros_rtde_gripper_commands](../ur_ros_rtde_gripper_commands) and are provided examples of how new commands can be implemented as ROS2 plugins. For instance, in this package there is an example of implementation of the `MoveDownUntilForce` plugin, which allow to move the robot towards the ground until a force is detected. 

If `ur_ros_rtde_tutorials` is compiled, the plugin will be loaded automatically by the *CommandServer*. The command can be sent to the robot sending an empty goal to the corresponding action server:

1. Launch *CommandServer*:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde command_server.launch.py
    ```
2. Send the goal:
    ```
    # type in a new terminal
    ros2 action send_goal /ur_ros_rtde/move_down_until_force_command ur_ros_rtde_tutorials/action/MoveDownUntilForce {}
    ```

   <p align="center">
    <img src="../images/dual_force.gif" style="width: 50%; height: auto;">
    </p>