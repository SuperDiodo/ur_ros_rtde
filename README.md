
# ROS2 Interface for Universal Robot CoBots Control with ur_rtde (C++, Python)

<div style="display: flex; justify-content: center;">
    <img src="images/logo_rimlab.PNG" alt="Image 1" style="width: auto; height: 150px; margin-right: 25px;">
    <img src="images/logo_repo.jpg" alt="Image 2" style="width: auto; height: 150px; margin-left: 25px;">
</div>

</br>

ROS2 interfaces based on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) for communication with **Universal Robot collaborative manipulators**. 
The software was developed within [RIMLab](https://rimlab.ce.unipr.it/), the robotic laboratory of the University of Parma. 

This project is currently under development, with ongoing updates and enhancements planned for the future. If you have any thoughts or feedback about the software, feel free to contact us at alessio.saccuti@unipr.it.

**Use this software with caution. The robot may collide with objects or people if not properly monitored. Always ensure a safe environment during operation. The developers are not responsible for any damages or injuries caused by improper use.**

---

## Capabilities of `ur_ros_rtde`
- Reception of various data including joint positions, force, torque, etc.
- Configuration of internal robot parameters such as payload.
- UR control box digital pin reading and writing.
- Execution of MovePath, MoveJ, and MoveL commands.
- Execution of MoveL commands until contact is detected.
- Execution of MoveL commands until force or torque exceeds predefined thresholds.
- Sending and executing trajectories in the joint state space (e.g., trajectories planned with MoveIt!).
- Control of Schmalz GCPi vacuum gripper and OnRobot Soft Gripper (SG).
- Possibility of using ROS2 interfaces in simulated environments, even in the absence of physical hardware (e.g., trajectory evaluation).
- Possibility of using ROS2 interfaces along with MoveIt! configuration packages.
- Visualization of the 3D robot in RViz.
- Extension of robot commands through ROS2 plugins.

<p align="center">
  <img src="images/dual.gif">
</p>

---

## Contents of `ur_ros_rtde`
- `ur_ros_rtde`: the core of our software, ROS2 nodes which provides messages on topics, services and actions.
- `ur_ros_rtde_msgs`: messages, services and actions definitions.
- `ur_ros_rtde_simple_clients`: utility header files for services and actions.
- `ur_ros_rtde_gripper_commands`: example of plugins that extend the robot commands enabling the control of real grippers.
- `simple_ur10e_description`: example of description package containing meshes, xacro and urdf files. The package is a simplified version of [this repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), but specific for UR10e.
- `simple_ur10e_moveit_config`: example of MoveIt! configuration package generated with `moveit_setup_assistant`.

---
## Setup `ur_ros_rtde`

To utilize our software, you need to install `ur_rtde` and `ROS2`.

### Install `ur_rtde`:

  You can install `ur_rtde` running: 
  ```bash
  sudo add-apt-repository ppa:sdurobotics/ur-rtde
  sudo apt-get update
  sudo apt install librtde librtde-dev
  ```

  Alternatively, you can manually build it:

  ```bash
  git clone https://gitlab.com/sdurobotics/ur_rtde.git
  cd ur_rtde
  git submodule update --init --recursive
  mkdir build
  cd build
  cmake ..
  make 
  sudo make install
  ```

### Install Robot Operating System 2 (`ROS2`)

We recommend installing ROS2 **humble** using the official [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Additionally, install the following packages:
  ```bash
  sudo apt install python3-colcon-common-extensions
  sudo apt-get install ros-humble-controller-manager
  ```

(Optional) Install **MoveIt!**:

`ur_ros_rtde` ROS2 nodes were developed so that `MoveIt!` can be easily adopted for motion planning.

  ```bash
  sudo apt install ros-humble-moveit
  echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
  (optional)
  sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  ```

### Setup ROS2 interfaces
```bash
# clone ur_ros_rtde repository
git clone https://github.com/SuperDiodo/ur_ros_rtde.git

# build ur_ros_rtde
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
---
## How to use `ur_ros_rtde`

Our software is composed of three ROS2 nodes:

**robot_state_receiver**: provides robot data through topics and services (reference launch file: `robot_state_receiver.launch.py`).

**command_server**: discovers and loads `ur_ros_rtde_commands` plugins available in the ROS2 workspace. Each plugin starts a ROS2 action server for robot control (reference files `command_server.launch.py`, `command_base_class.hpp`).

**dashboard_server**: discovers and loads `ur_ros_rtde_dashboard_commands` plugins available in the ROS2 workspace. Each plugin starts a ROS2 action servers for robot control (reference files `dashboard_server.launch.py`, `dashboard_command_base_class.hpp`).

You can easily interact with ROS2 services and actions using header files provided in `ur_ros_rtde_simple_clients`.
For further details and documentation, please visit [`ur_ros_rtde`](https://github.com/SuperDiodo/ur_ros_rtde/tree/main/ur_ros_rtde).


Test if everything is working:

1. Ensure that the optional packages are compiled.

2. In `simple_ur10e_description` generate UR10e urdf file from xacro files.
    ```bash
    # generate ur10e urdf
    cd ~/your_path/simple_ur10e_description/urdf
    sh generate_urdf.sh ur10e.xacro ur10e.urdf
    ```

2. Configure `robot_state_receiver.launch.py`:
    - set ip address with `robot_ip`
    - set `robot_description_package = "simple_ur10e_description"`
    - set `urdf_file_name = "urdf/ur10e.urdf"`
    - (optional) set `launch_rviz = True` if robot should be displayed in RViz

3. Launch **robot_state_receiver**:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde robot_state_receiver.launch.py
    ```
    
    The first time RViz will be empty. To show the robot: 
    1. Add a robot model: `Displays\Add\RobotModel`
    2. Set `world` in `Fixed Frame`
    3. Set `/robot_description` in `RobotModel\Description Topic`
    
4. Configure `command_server.launch.py` setting ip address with `robot_ip`
5. Launch **command_server**:
    ```bash
    # type in a new terminal
    ros2 launch ur_ros_rtde command_server.launch.py
    ```
6. Run test:

    **WARNING**! If everything was successfully configured the robot will start moving! Check for possible collisions with the environment!
    
    With `test_command_server` executable MoveL commands are sent to the robot. Starting from the actual pose it will move +10 cm on X axis, then -20 cm on X axis and finally +10 on X axis again.

    ```bash
    # type in a new terminal
    ros2 run ur_ros_rtde test_command_server
    ```

    Moreover, there is a second executable (`test_trajectory_execution`) which can be used to test trajectory execution.
    The robot will move as shown in the animated image (the robot is oriented towards Y-axis).

    <p align="center">
      <img src="images/dual.gif">
    </p>

    ```bash
    # type in a new terminal
    ros2 run ur_ros_rtde test_trajectory_execution
    ```

---
## Adding new plugins to `ur_ros_rtde`

In `ur_ros_rtde/src/base_commands` and `ur_ros_rtde/src/base_dashboard_commands` several commands from `ur_rtde` [RTDE Control Interface](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#rtde-control-interface-api) and [Dashboard Client](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#dashboard-client-api) are implemented as ROS2 plugins. When launching `command_server` and `dashboard_server` these plugins are automatically loaded.

An example of commands that can be defined in custom ROS2 packages is reported in `ur_ros_rtde_gripper_commands`, an initial tutorial on plugin can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

Basically:
1. create a ROS2 package containing a `plugin.xml` file in which the plugins are declared (use `ur_ros_rtde_gripper_commands` as example).
2. create new plugins as `ur_ros_rtde_command` or `ur_ros_rtde_dashboard_command` subclass. Below there is an example of the template for a `ur_ros_rtde_command` new plugin.
    ```cpp
    #include <ur_ros_rtde/command_base_class.hpp>
    #include <pluginlib/class_list_macros.hpp>
    #include </* your action */>

    // < EXAMPLE OF ur_ros_rtde_command DEFINITION>
    ...
    // ----- PLUGIN INFO (CHANGE HERE!) --------
    #define PLUGIN_NAME "your_command"
    #define PLUGIN_CLASS_NAME YourCommand
    using action_type = /*action type*/;
    // -----------------------------------------

    void execute_function_impl(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
        std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
        std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
        std::shared_ptr<ur_rtde::DashboardClient> dashboard_client)
    { 
      // ---------- PLUGIN BEHAVIOUR ----------
      //      implement your plugin here!
      // -----------------------------------------
    }

    class PLUGIN_CLASS_NAME : public ur_ros_rtde_command
    {
    public:
      void start_action_server(
          rclcpp::Node::SharedPtr node,
          std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
          std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
          std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
          std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
      {
        auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client);
        server_ = std::make_unique<command_server_template<action_type>>(
            node, PLUGIN_NAME, bound_execute_function);
      }

    private:
      std::unique_ptr<command_server_template<action_type>> server_;
    };

    PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)
    ```
  
  
  Basically, you have to add the include for an action that will be exposed with an action server, change the plugin info ad the implementation. The remaining part is the same for each plugin.

3. Compile the package and, if everything worked, `command_server` or `dashboard server` will automatically load the new plugins. 

---
## Integration of `ur_ros_rtde` and MoveIt!

Setting `launch_moveit = True` in `ur_ros_rtde/launch/robot_state_receiver.launch.py`, several files from the associated MoveIt! configuration packages are automatically launched. 

We recommend to use [`moveit_planning`](https://github.com/SuperDiodo/moveit_planning.git), a C++ library which includes utility functions for using ROS2 MoveIt! planning framework.

In `moveit_planning` instructions on the setup and its usage can be found.