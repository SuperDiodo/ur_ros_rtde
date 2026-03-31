## 🤖 **Integration with MoveIt!**

The driver integrates with MoveIt! by publishing [`JointState`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html) messages on the standard `/joint_states` topic, allowing MoveIt! to track the robot state.

---

### Requirements

A robot description package must be available, for example:
- [`simple_ur10e_description`](../simple_ur10e_description/)
- [Universal Robots ROS 2 description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

---

### Setup

Install MoveIt 2:  
https://moveit.ai/install-moveit2/binary/

Configure MoveIt using the:  
[MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

---

### Example Configuration

An example configuration package is provided:
- [`simple_ur10e_moveit_config`](../simple_ur10e_moveit_config/)

⚠️ This package may not be compatible with all MoveIt! versions, as configurations are version-dependent.  
It is recommended to generate a new configuration package using the Setup Assistant.

---

### Optional: Additional Kinematics Solvers

By default, MoveIt uses the **KDL** solver.

Alternatives include:
- **TRAC-IK**: https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html  
- **IKFast**: https://moveit.picknik.ai/main/doc/examples/ikfast/ikfast_tutorial.html  

---

### Usage

Official MoveIt tutorials can be found [here](https://moveit.picknik.ai/main/index.html).   
We also highlight an our related [project](https://github.com/SuperDiodo/moveit_planning), which provides an example interface library built on top of the MoveIt API.  
This project may be further developed in the future.
