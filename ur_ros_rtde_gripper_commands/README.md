# Gripper commands examples

Add-on devices such as robot grippers often require software extensions, called URCaps, to be installed on the robot controller. URCaps typically provide custom instructions which are not part of standard URScript. Plugins of the *extension* type are used so that custom instructions can be called by `ur_ros_rtde`.

#### OnRobot soft gripper

In this package are provided implementation examples of commands that can be used to interact with an OnRobot soft gripper. To control the gripper it is mandatory to:

1. Install the OnRobot URCap using the robot teach pendant.
2. Clone `ur_rtde`, apply the [patch](../ur_ros_rtde/config/ur_rtde_7bd8f3481877cc9aeec2cbb2b109326b6bbab282.patch) provided in `ur_ros_rtde` and then install the cloned software.
    ```bash
    # optional: remove ur_rtde binaries if installed with apt
    sudo apt remove --purge librtde librtde-dev

    # clone ur_rtde
    git clone https://gitlab.com/sdurobotics/ur_rtde.git
    cd ur_rtde
    git submodule update --init --recursive

    # apply the patch
    git apply <path>.patch

    # install ur_rtde
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install
    ```
3. Compile `ur_ros_rtde_gripper_commands` package.
    ```bash
    cd <ros_workspace_path>
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ur_ros_rtde_gripper_commands
    ```

4. Remove the plugins implemented in this package from the blacklist in the [launch file](../ur_ros_rtde/launch/command_server.launch.py). 

If everything was set up as indicated above, the *CommandServer* will be able to discover and load the plugins related to OnRobot commands. Each command, as done for the other plugins, will be exposed as an action server. The command server can be launched and commands can be requested to the robot as shown in the [tutorials](../ur_ros_rtde_tutorials/).