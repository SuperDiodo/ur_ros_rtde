import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    ######################
    #### CHANGE HERE #####

    robot_ip = "160.78.27.23"  # robot ip, example 127.0.0.1
    robot_description_package = "simple_ur10e_description" # ros2 pkg containing 3D files of the robot (example: ur10e_description)
    urdf_file_name = "urdf/ur10e.urdf" # urdf of the robot (example: urdf/ur10e.urdf)
    moveit_config_pkg = "" # moveit_config pkg to use (example: ur10e_moveit_config)
    launch_rviz = True
    launch_moveit = False
    
    # use the driver in simulated mode:
    # -	no robot required
    # -	fake joint states, no force/torque data
    simulation_only = False
    
    robot_state_receiver_params = {
        "robot_ip": robot_ip,
        "rtde_frequency": 20.0, # freq. (Hz) at which ur_rtde receiver interface will exchange data
        "data_receiving_frequency": 100, # freq. (Hz) at which data is received from robot and published in topics
        "simulation_only": simulation_only, 
        "simulation_start_robot_state": [-1.82,-1.57,1.77,-1.77,-1.57,-0.25], # starting robot configurations for simulation only
        "fake_joint_states_topic": "/fake_joint_states", # topic at which simulated robot configuration should be published
        "real_joint_states_topic": "/real_joint_states", # topic at which real robot configuration are published
        "wrench_topic": "/wrench", # topic at which wrenches are published  
    }

    robot_configuration_params = {
        "robot_joint_names": ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"],
        "robot_base_link": "base_link_inertia",
        "robot_flange_link": "wrist_3_link",
    }

    optional_attached_camera = {
        "camera_calibration_file": "", # file containing the calibration matrix of the camera, it can be empty!
        "calibrated_camera_parent_tf_name": "",  # link at which the calibrated camera tf should be linked
        "calibrated_camera_tf_name": "", # calibrated camera tf name
        "position_offset": [0.0,0.0,0.0], # position offset of the calibrated camera tf
        "orientation_offset": [0.0,0.0,0.0], # orientation offset of the calibrated camera tf
    }

    ######################
    #### DO NOT TOUCH ####

    urdf = os.path.join(
        get_package_share_directory(robot_description_package),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    state_pusblisher_params = {
        "use_sim_time":False,
        "robot_description": robot_desc,
    }

    rviz_params = {
        "robot_description": robot_desc,
    }

    nodes = []

    nodes.append(
        Node(
            package="ur_ros_rtde",
            executable="robot_state_receiver",
            namespace="ur_ros_rtde",
            output="screen",
            parameters=[
                robot_state_receiver_params,
                robot_configuration_params,
                optional_attached_camera
            ],
            remappings=[
                ('/ur_ros_rtde/tf', '/tf'),
            ]
        )
    )

    nodes.append(SetParameter(name='robot_description', value=robot_desc),)
    nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    state_pusblisher_params
                ],
                arguments=[urdf])
    )

    if launch_moveit:
        nodes.append(launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        get_package_share_directory(moveit_config_pkg) + '/launch/move_group.launch.py')))
        
        if launch_rviz:
             nodes.append(launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                        get_package_share_directory(moveit_config_pkg) + '/launch/moveit_rviz.launch.py')))
             
    else:
        if launch_rviz:
                nodes.append(
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        output='screen',
                        parameters=[
                            rviz_params
                        ],)
                )

    return LaunchDescription(nodes)
