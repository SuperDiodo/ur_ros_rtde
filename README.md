# Universal Robot ROS2 driver based on ur_rtde

ROS2 interfaces based on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) for communication with **Universal Robot collaborative manipulators**.

This project is born within [RIMLab](https://rimlab.ce.unipr.it/), the robotic laboratory of University of Parma.

If you have any thoughts or feedback about the software, feel free to contact us at alessio.saccuti@unipr.it.

---

## Capabilities of `ur_ros_rtde`

- Display robot in a 3D visualizer (RViz)
- Receive data (joints positions, force, torque, etc.)
- Set robot internal parameters (e.g. payload)
- Set and read pins state
- Execute MovePath, MoveJ and MoveL commands
- Execute MoveL commands until a contact is detected
- Execute MoveL commands until the force read by the force torque sensor is above a threshold
- Execute MoveL commands until the torque read by the force torque sensor is above a threshold
- Send and execute trajectories described in the robot configuration state space (e.g. trajectories planned with MoveIt!)

It can be used with real and simulated robots.

In the future new features will be added and the existings will be updated!


---
## Setup `ur_ros_rtde`

Our software requires to install `ur_rtde` and `ROS2`.

### Install `ur_rtde`:

  It can be installed with `pip3 install ur_rtde`, or it can be manually built:

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

We suggest to install `ROS2 humble` using the official [guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

In addition please install:
  ```bash
  sudo apt install python3-colcon-common-extensions
  sudo apt-get install ros-humble-controller-manager
  ```

(Optional) Install **MoveIt!**:

`ur_ros_rtde` ROS2 nodes were developed so that `MoveIt!` can be easily adopted for motion planning.

  ```bash
  sudo apt install ros-humble-moveit
  sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
  ```

### Setup ROS2 interfaces
```bash
# clone ur_ros_rtde repository
git clone this_repo

# build ur_ros_rtde_msgs
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ur_ros_rtde_msgs

# build ur_ros_rtde_simple_clients
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ur_ros_rtde_simple_clients

# build ur_ros_rtde
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ur_ros_rtde
```
---
## How to use `ur_ros_rtde`

The driver is composed of two ROS2 nodes that can be executed. Examples of ROS2 launch files were provided:

- `robot_state_receiver`: makes data from the robot available through topics and services.
  ```
  ros2 launch ur_ros_rtde robot_state_receiver.launch.py
  ```

- `command_server` makes available a list of ROS2 action servers, which can be used to control the robot.
  ```
  ros2 launch ur_ros_rtde command_server.launch.py
  ```

The two nodes provides ROS2 services and actions which can be easily called with the header files in `ur_ros_rtde_simple_clients`. Examples of data receiving and robot movements can be seen in `src/test/test_command_server.cpp`, which can be launched with:

```
ros2 run ur_ros_rtde test_command_server
```

Further details of our software can be found [here](ur_ros_rtde/readme.md).