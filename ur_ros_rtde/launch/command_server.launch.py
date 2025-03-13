from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ######################
    #### CHANGE HERE #####
    
    command_server_params = {
        "robot_ip": "", # robot ip
        "command_server.plugins_blacklist": [
            "SgGrip",
            "SgRelease",
            "SgGetWidth"
        ]
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
            ],
        )
    )

    return LaunchDescription(nodes)
