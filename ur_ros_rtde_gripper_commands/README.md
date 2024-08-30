# External Hardware

The commands in `ur_ros_rtde` can be easily extended with custom commands. Examples for controlling real grippers, available in `ur_ros_rtde_gripper_commands`, are shown below.

### Schmalz GCPi vacuum generator:

In order to control the Schmalz GCPi vaccum generator it must be connected to the UR control box with 2 input digital pins and 2 ouput digital pins. 

- `set_deposit_command`: set state of the gripper deposit, it uses `set_deposit_command.deposit_pin` to get the digital pin to enable gripper deposit.
- `set_suction_command`: set state of the gripper suction, it uses `set_suction_command.deposit_pin` to get the digital pin to enable gripper suction.


### OnRobot Soft Gripper (SG):

In order to control the OnRobot SG it is mandatory to:
1. Install the OnRobot URCap available with the gripper.
2. Copy our custom UrScript `sg_control_program.urp` from the config folder to the robot.

The command server will exchange data by writing and reading the robot controller registers (defined as ROS2 params): 

- `soft_gripper_control_command.grip_bool_input_register`: input register at which the robot expects to get the open/close signal.
- `soft_gripper_control_command.desired_width_input_register`: input register at which the robot expects to get the desired width.
- `soft_gripper_control_command.feedback_width_output_register`: output register at which the robot expects to get the width of the gripper after the action.

The SG can be controlled with:
- `soft_gripper_control_command`: if **grip** is `false` then the gripper will close until **target_width** is reached, otherwise it will open. If the action can't be execute withing a time limit the action will exit. In **error** will be store the difference between the desired and the actual width of the SG. 