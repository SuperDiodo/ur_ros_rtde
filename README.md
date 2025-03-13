
# A ROS2 Interface for Universal Robots Collaborative Manipulators Based on ur rtde

ROS2 driver based on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) for **Universal Robot** collaborative manipulators. The software was developed within [RIMLab](https://rimlab.ce.unipr.it/), the robotic laboratory of the University of Parma. 

This project is currently under development, with ongoing updates and enhancements planned for the future. If you have any thoughts or feedback about the software, feel free to send an email at alessio.saccuti@unipr.it.

**Use this software with caution. The developers are not responsible for any damages or injuries that may result from its use.**

---

## Capabilities of `ur_ros_rtde`
- Reception of various data (joint states, forces, torques, etc.) which is made available in topics and as services.
- High-level URScript commands are exposed as action servers (*MoveL*, *MoveJ*, etc.)
- Composite commands as trajectory execution or moving the robot linearly until a force is detected are also exposed as action servers.
- New commands can be added thanks to a plugin system.
- Easy integration with ROS-based tool as RViz or MoveIt.

<p align="center">
  <img src="images/dual.gif">
</p>

---

## Contents of `ur_ros_rtde`
- **`ur_ros_rtde`**) core implementation of the driver. It includes the implementation of the following ROS2 nodes:
  - *RobotStateReceiver*: retrieves the robot state.
  - *CommandServer*: exposes high-level URScript commands as actions.
  - *DashboardServer*: esposes commands related to *Dashboard Client* interface.
- **`ur_ros_rtde_msgs`**) ROS2 package containing the definition of:
  - messages and services advertised by *RobotStateReceiver*
  - actions exposed by *CommandServer*
  - actions exposed by *DashboardServer*
- **`ur_ros_rtde_simple_clients`**) ROS2 package containing the definition of basic service and action clients.
- **`ur_ros_rtde_gripper_commands`**) Example of ROS2 package that extends the `ur_ros_rtde` features by defining new plugins for managing external hardware devices. 
- **`simple_ur10e_description`**: example of a ROS2 description package containing meshes, xacro and urdf files. The package is a simplified version of [this repository](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), but specific for UR10e.
- **`simple_ur10e_moveit_config`**) example of MoveIt! configuration package generated with `moveit_setup_assistant` for the UR10e robot described in `simple_ur10e_description`.
- **`ur_ros_rtde_tutorials`**) Package containing usage examples.

---
## Setup `ur_ros_rtde`

To utilize the proposed software, you need to install `ur_rtde` and `ROS2`.

#### Install `ur_rtde`:

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

#### Install Robot Operating System 2 (`ROS2`)

This version of `ur_ros_rtde` is tested with installing ROS2 **Humble** ([installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)).

Additionally, install the following packages:
  ```bash
  sudo apt install python3-colcon-common-extensions
  sudo apt-get install ros-humble-controller-manager
  ```

(Optional) Install **MoveIt!**:

The data retrieved by *RobotStateReceiver* ROS2 node in `ur_ros_rtde` can be used with `MoveIt!` framework.

  ```bash
  sudo apt install ros-humble-moveit
  echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
  (optional)
  sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  ```

#### Install `ur_ros_rtde`
```bash
# clone ur_ros_rtde repository
git clone https://github.com/SuperDiodo/ur_ros_rtde.git

# build ur_ros_rtde
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Use the driver

In `ur_ros_rtde` ROS2 package are provided simple launch files which can be configured based on the needs:
- `ur_ros_rtde/launch/robot_state_receiver.launch.py`: launch the *RobotStateReceiver* node.
- `ur_ros_rtde/launch/command_server.launch.py`: launch the *CommandServer* node.
- `ur_ros_rtde/launch/dashboard_server.launch.py`: launch the *DashboardServer* node.

Tutorials on how use these files and run examples are provided in [ur_ros_rtde_tutorials](ur_ros_rtde_tutorials).