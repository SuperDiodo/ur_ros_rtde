from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ######################
    #### CHANGE HERE #####
    
    robot_ip = ""

    command_server_params = {
        "robot_ip": robot_ip, # robot ip
        "receiver_freq": 50
    }

    trajectory_execution_params = {
        "parametrization_timestep": 0.002,
        "servoJ_lookahead_time" : 0.2, 
        "servoJ_gain" : 100,
        "servoJ_timestep" : 0.002,
        "deviation_check_freq": 20,
        "max_deviation" : 1.57
    }

    soft_gripper_control_params = {
        "grip_bool_input_register": 19,
        "desired_width_input_register": 18,
        "feedback_width_output_register": 18
    }

    vacuum_gripper_control_params = {
        "suction_pin": 0,
        "deposit_pin": 1
    }

    ######################
    #### DO NOT TOUCH ####

    nodes = []

    nodes.append(
        Node(
            package="ur_ros_rtde",
            executable="command_server",
            namespace="ur_ros_rtde",
            output="screen",
            parameters=[
                command_server_params,
                trajectory_execution_params,
                soft_gripper_control_params,
                vacuum_gripper_control_params
            ],
        )
    )

    return LaunchDescription(nodes)
