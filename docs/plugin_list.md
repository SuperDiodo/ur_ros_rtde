# 🧩 **Plugin List**

More than 40 plugins are implemented withing this repository and each of them is exposed as an action server.  
This is the complete list of action servers which can be activated with the proposed software:

## **`ur_ros_rtde_command`** in [`ur_ros_rtde`](../ur_ros_rtde/src/base_commands/)
- `execute_parametrized_trajectory_command`
- `execute_path_command`
- `execute_trajectory_command`
- `move_j_command`
- `move_j_relative_command`
- `move_l_command`
- `move_l_relative_command`
- `move_until_contact_command`
- `move_until_force_command`
- `move_until_torque_command`
- `reset_force_torque_sensor_command`
- `send_custom_script_command`
- `set_digital_pin_command`
- `set_free_drive_command`
- `set_payload_command`
- `set_speed_slider_command`
- `servo_j`
- `servo_stop`

## **`ur_ros_rtde_dashboard_command`** in [`ur_ros_rtde`](../ur_ros_rtde/src/base_dashboard_commands/)
- `add_to_log_command`
- `brake_release_command`
- `close_popup_command`
- `close_safety_popup_command`
- `get_loaded_program_command`
- `get_robot_model_command`
- `get_serial_number_command`
- `is_in_remote_control_command`
- `is_program_saved_command`
- `load_urp_command`
- `pause_command`
- `play_command`
- `polyscope_version_command`
- `popup_command`
- `power_off_command`
- `power_on_command`
- `program_state_command`
- `restart_safety_command`
- `robot_mode_command`
- `running_command`
- `safety_mode_command`
- `safety_status_command`
- `shutdown_command`
- `unlock_protective_stop_command`
- `stop`

## **`ur_ros_rtde_extension`** in [`ur_ros_rtde_gripper_commands`](../ur_ros_rtde_gripper_commands/src/)
- `sg_get_width_command`
- `sg_grip_command`
- `sg_release_command`

## **`ur_ros_rtde_command`** in [`ur_ros_rtde_tutorials`](../ur_ros_rtde_tutorials/src/)
- `move_down_until_force_command`
