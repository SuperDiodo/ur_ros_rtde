# **Run `ur_ros_rtde` ROS 2 Nodes**

The `ur_ros_rtde` ROS 2 package provides several nodes with configurable ROS parameters to allow flexible setups.

Example launch files are provided for running different nodes:

- `robot_state_receiver.launch.py` → runs `robot_state_receiver_node`
- `command_server.launch.py` → runs `command_server_node`
- `dashboard_server.launch.py` → runs `dashboard_server_node`

Below are the configurable parameters for each launch file.

---

## 🛠️ Parameters for `robot_state_receiver.launch.py`

| Parameter | Description |
|----------|------------|
| `robot_ip` | IP address of the robot |
| `robot_description_package` | ROS 2 package containing robot description (meshes, URDF, etc.) |
| `urdf_file_name` | Relative path to the URDF file within the description package |
| `moveit_config_pkg` | MoveIt configuration package |
| `launch_rviz` | Launch RViz for visualization |
| `launch_moveit` | Launch MoveIt |
| `simulation_only` | Enable fake robot mode (no real robot required). This will: <br> • Generate fake joint states <br> • Disable force/torque data <br> • Avoid requiring a real robot connection |

---

## 🛠️ Configuration for `command_server.launch.py`

```python
robot_ip = ""  # Robot IP (e.g. 127.0.0.1)
command_server_plugins_blacklist = []
```

| Parameter | Description |
|----------|------------|
| `robot_ip` | IP address of the robot |
| `command_server_plugins_blacklist` | List of plugins to disable in the command server |

---

## 🛠️ Configuration for `dashboard_server.launch.py`

```python
robot_ip = ""  # Robot IP (e.g. 127.0.0.1)
dashboard_server_plugins_blacklist = []
```

| Parameter | Description |
|----------|------------|
| `robot_ip` | IP address of the robot |