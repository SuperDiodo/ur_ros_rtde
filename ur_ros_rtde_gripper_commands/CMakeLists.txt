cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde_gripper_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED) 
find_package(ur_ros_rtde REQUIRED) 
find_package(ur_rtde REQUIRED)

include_directories(
  include
  ${ur_ros_rtde_INCLUDE_DIRS} # do not add ur_ros_rtde in ament_target_dependencies, this is fine
)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  action_msgs
  builtin_interfaces
  pluginlib
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SetSuction.action"
  "action/SetDeposit.action"
  "action/SoftGripperControl.action"
  DEPENDENCIES ${dependencies})

pluginlib_export_plugin_description_file(ur_ros_rtde_gripper_commands plugins.xml)

file(GLOB SOURCES "src/*.cpp")

add_library(gripper_commands_lib SHARED ${SOURCES})
target_compile_features(gripper_commands_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(gripper_commands_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gripper_commands_lib ${dependencies})
rosidl_target_interfaces(gripper_commands_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(gripper_commands_lib ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(
  TARGETS gripper_commands_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  gripper_commands_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
