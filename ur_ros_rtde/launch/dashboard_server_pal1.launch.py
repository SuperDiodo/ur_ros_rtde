from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ######################
    #### CHANGE HERE #####

    dashboard_server_params = {
        "robot_ip": "160.78.27.23", # robot ip
    }

    ######################
    #### DO NOT TOUCH ####

    nodes = []

    nodes.append(
        Node(
            package="ur_ros_rtde",
            executable="dashboard_server",
            namespace="ur_ros_rtde",
            output="screen",
            parameters=[
                dashboard_server_params,
            ],
        )
    )

    return LaunchDescription(nodes)
