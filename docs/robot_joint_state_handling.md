# 🔧 Robot Joint State Handling (compatible with MoveIt!)

We introduced an **internal joint state monitoring system** that:
- keeps track of the robot joint state  
- publishes it to the standard `/joint_states` topic  

Publishing to `/joint_states` enables:
- ✅ Visualization in **RViz** (e.g., via a robot state publisher)  
- ✅ Seamless integration with **MoveIt!**, which uses the same topic to update its internal robot representation  

---

## Configurable Data Sources

By default, the driver publishes robot configurations to `/joint_states`.  
However, the **source of this data can be dynamically configured**:

- **Real Robot Connected**  
  - Data is published on `/real_joint_states`  
  - Then forwarded to `/joint_states`  
  - The robot state is updated by physically moving the robot  

- **Simulation Mode Enabled**  
  - `/fake_joint_states` is used instead  
  - Useful when no physical robot is available (e.g., to visualize a simulated trajectory)  
  - The robot state is updated by publishing to `/fake_joint_states`  

---

## Dynamic Source Switching

In some scenarios, it is necessary to **dynamically change the source** of the `/joint_states` topic.

**Example:**  
Planning a trajectory in MoveIt! assuming the robot is already in a *future configuration*.

This can be achieved using the service:

```
/publish_fake_joint_states
```

- Accepts a **boolean goal**:
  - `true` → use `/fake_joint_states`  
  - `false` → use `/real_joint_states`  

This mechanism provides **flexibility and control** over the robot state representation during runtime.
