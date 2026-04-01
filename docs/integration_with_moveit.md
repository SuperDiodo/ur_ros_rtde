## 🤖 Integration with MoveIt!

The driver integrates with MoveIt! by:

- Publishing [`JointState`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html) messages on the standard `/joint_states` topic, allowing MoveIt! to track the robot state.
- Exposing action servers of [`ExecuteTrajectory`](../ur_ros_rtde_msgs/action/ExecuteTrajectory.action) and [`ExecuteParametrizedTrajectory`](../ur_ros_rtde_msgs/action/ExecuteParametrizedTrajectory.action) types, which can be used to execute waypoint-based trajectories planned in MoveIt!.


## Executing MoveIt! Planned Trajectories with `ur_ros_rtde`

In this two examples, an [action client helper](../ur_ros_rtde_simple_clients/include/ur_ros_rtde_simple_clients/simple_action_client_template.hpp) is used to send goals to the action servers. More detailed tutorials on how to call actions with that helper are shown [here](../ur_ros_rtde_tutorials/).   

The examples assume that the trajectory that must be executed follows the [MoveIt! trajectory structure](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectory.html).


**Non-Parametrized Trajectories**

For trajectories defined only by waypoints (without time parametrization), **speed** and **acceleration** must be specified.
These parameters are used to time-parametrize the path before execution via the `ExecuteTrajectory` action server.

```cpp
ur_ros_rtde_msgs::action::ExecuteTrajectory::Goal goal_msg;
goal_msg.acceleration = acceleration;
goal_msg.speed = speed;

for (auto p : trajectory.points)
{
    ur_ros_rtde_msgs::msg::Vector joint_vector;

    for (auto j : p.positions)
        joint_vector.vector.push_back(j);

    goal_msg.trajectory.push_back(joint_vector);
}

action_client->send_goal<ur_ros_rtde_msgs::action::ExecuteTrajectory>(
    "ur_ros_rtde/execute_trajectory_command",
    goal_msg
);
```

**Parametrized Trajectories**

For already parametrized trajectories, **positions**, **velocities**, and **timing** must be provided to the `ExecuteParametrizedTrajectory` action server.

```cpp
ur_ros_rtde_msgs::action::ExecuteParametrizedTrajectory::Goal goal_msg;

for (auto p : trajectory.points)
{
    ur_ros_rtde_msgs::msg::Vector joint_positions_vector;
    ur_ros_rtde_msgs::msg::Vector joint_velocities_vector;

    for (auto j : p.positions)
        joint_positions_vector.vector.push_back(j);

    goal_msg.joint_positions.push_back(joint_positions_vector);

    for (auto j : p.velocities)
        joint_velocities_vector.vector.push_back(j);

    goal_msg.joint_velocities.push_back(joint_velocities_vector);

    goal_msg.times.push_back(
        static_cast<double>(p.time_from_start.sec) +
        static_cast<double>(p.time_from_start.nanosec) / 1e9
    );
}

action_client->send_goal<ur_ros_rtde_msgs::action::ExecuteParametrizedTrajectory>(
    "ur_ros_rtde/execute_parametrized_trajectory_command",
    goal_msg
);
```

---

## Requirements for Using MoveIt!

Below are the key requirements for setting up MoveIt! in a generic robotic application (including UR robots with `ur_ros_rtde`). The following setup steps are generic and widely documented online in the official MoveIt! documentation. They are summarized here for convenience.

**Robot Description**

A robot description package must be available. Examples:

- [`simple_ur10e_description`](../simple_ur10e_description/)
- [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

**Setup MoveIt!**

1. Install MoveIt 2 (https://moveit.ai/install-moveit2/binary/)

2. Configure MoveIt using the [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html).     
   An example configuration package is provided in [`simple_ur10e_moveit_config`](../simple_ur10e_moveit_config/) .  
    This package may not be compatible with all MoveIt! versions, as configurations are version-dependent. It is recommended to generate a new configuration package using the Setup Assistant.

3. Optional: Additional Kinematics Solvers.     
    By default, MoveIt uses the **KDL** solver.
    Alternative solvers, which can be choosen in the setup assistant, include:

    - **TRAC-IK**  
    https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html

    - **IKFast**  
    https://moveit.picknik.ai/main/doc/examples/ikfast/ikfast_tutorial.html

**Usage**

- Official MoveIt tutorials:  
  https://moveit.picknik.ai/main/index.html

- Related project:  
  https://github.com/SuperDiodo/moveit_planning  

  This project provides an example interface library built on top of the MoveIt API and may be further developed in the future.
