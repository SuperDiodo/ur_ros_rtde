cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(dependencies
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  action_msgs
  builtin_interfaces
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointsConfiguration.msg"
  "msg/Vector.msg"
  "action/MoveL.action"
  "action/MoveJ.action"
  "action/SetDigitalPin.action"
  "action/SetSuction.action"
  "action/SetDeposit.action"
  "action/SetPayload.action"
  "action/ExecutePath.action"
  "action/ExecuteTrajectory.action"
  "action/SetSpeedSlider.action"
  "action/MoveUntilContact.action"
  "action/MoveUntilForce.action"
  "action/MoveUntilTorque.action"
  "action/MoveLRelative.action"
  "action/MoveJRelative.action"
  "action/ResetForceTorqueSensor.action"
  "action/SoftGripperControl.action"
  "action/SetFreedrive.action"
  "srv/GetInternalState.srv"
  "srv/GetTcpPose.srv"
  "srv/GetWrench.srv"
  "srv/GetRobotConfiguration.srv"
  DEPENDENCIES ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
