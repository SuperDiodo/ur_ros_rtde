cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde_gripper_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ur_ros_rtde REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Include directories
include_directories(
  include
  ${ur_ros_rtde_INCLUDE_DIRS} # Do not add ur_ros_rtde in ament_target_dependencies
)

# Dependencies for the generated interfaces
set(dependencies
  builtin_interfaces
  geometry_msgs
)

# Generate actions and messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SgGrip.action"
  "action/SgRelease.action"
  "action/SgGetWidth.action"
  DEPENDENCIES ${dependencies}
)

# Export plugin description file
pluginlib_export_plugin_description_file(ur_ros_rtde_gripper_commands plugins.xml)

# Library: gripper_control_script_extension_lib
file(GLOB EXTENSION_SOURCES "src/*.cpp")
add_library(gripper_control_script_extension_lib SHARED ${EXTENSION_SOURCES})
target_compile_features(gripper_control_script_extension_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(gripper_control_script_extension_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gripper_control_script_extension_lib rclcpp_action pluginlib geometry_msgs)

# Link typesupport to the library
rosidl_get_typesupport_target(typesupport_target_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_control_script_extension_lib ${typesupport_target_cpp} ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(
  TARGETS gripper_control_script_extension_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install action files
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}/
)

# Export package information
ament_export_include_directories(
  include
)

ament_export_libraries(
  gripper_commands_lib
  gripper_control_script_extension_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # Skip copyright checks
  set(ament_cmake_cpplint_FOUND TRUE)   # Skip cpplint
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

