cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_ros_rtde_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(ur_ros_rtde_simple_clients REQUIRED) 


include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_action
  ur_rtde
  sensor_msgs
  geometry_msgs
  ur_ros_rtde_msgs
  trajectory_msgs
  std_srvs
  tf2_msgs
  visualization_msgs
  tf2_ros
  ur_ros_rtde_simple_clients
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install robot_state_receiver
add_executable(robot_state_receiver src/robot_state_receiver.cpp)
ament_target_dependencies(robot_state_receiver ${dependencies})
target_link_libraries(robot_state_receiver ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(TARGETS
robot_state_receiver
DESTINATION lib/${PROJECT_NAME})

# Install command_server
add_executable(command_server src/command_server_template_specialization.cpp)
ament_target_dependencies(command_server ${dependencies})
target_link_libraries(command_server ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(TARGETS
command_server
DESTINATION lib/${PROJECT_NAME})

# Tests
add_executable(test_command_server src/test/test_command_server.cpp)
ament_target_dependencies(test_command_server ${dependencies})
target_link_libraries(test_command_server ${Eigen3_LIBRARIES})

install(TARGETS
test_command_server
DESTINATION lib/${PROJECT_NAME})

# Install test_server
add_executable(test_trajectory_execution src/test/test_trajectory_execution.cpp)
ament_target_dependencies(test_trajectory_execution ${dependencies})
target_link_libraries(test_trajectory_execution ${Eigen3_LIBRARIES})

install(TARGETS
test_trajectory_execution
DESTINATION lib/${PROJECT_NAME})

# Install launch, config, include folders

install(DIRECTORY
launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/ur_ros_rtde
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
