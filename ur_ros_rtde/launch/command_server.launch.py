from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ######################
    #### CHANGE HERE #####
    
    robot_ip = ""

    ######################
    #### DO NOT TOUCH ####
    
    command_server_params = {
        "robot_ip": robot_ip, # robot ip
    }

    nodes = []

    nodes.append(
        Node(
            package="ur_ros_rtde",
            executable="command_server",
            namespace="ur_ros_rtde",
            output="screen",
            parameters=[
                command_server_params,
            ],
        )
    )

    return LaunchDescription(nodes)
