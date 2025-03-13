cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(ur_ros_rtde REQUIRED)
find_package(ur_ros_rtde_msgs REQUIRED)
find_package(ur_ros_rtde_simple_clients REQUIRED)
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
  rclcpp
  rclcpp_action
  pluginlib
  geometry_msgs
  std_srvs
  ur_ros_rtde_msgs
  ur_ros_rtde_simple_clients
)

# Generate actions and messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveDownUntilForce.action"
  DEPENDENCIES ${dependencies}
)

# Export plugin description file
pluginlib_export_plugin_description_file(ur_ros_rtde_tutorials plugins.xml)

# Library: tutorial_plugins_lib
set(PLUGIN_SOURCES
    src/move_down_until_force.cpp
)

add_library(tutorial_plugins_lib SHARED ${PLUGIN_SOURCES})
target_compile_features(tutorial_plugins_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(tutorial_plugins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tutorial_plugins_lib ${dependencies})

# Link typesupport to the library
rosidl_get_typesupport_target(typesupport_target_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tutorial_plugins_lib ${typesupport_target_cpp} ur_rtde::rtde)

install(
  TARGETS tutorial_plugins_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  tutorial_plugins_lib
)

ament_export_targets(
  export_${PROJECT_NAME}
)

# Examples
set(EXAMPLES_SOURCES
    src/linear_movements.cpp
    src/linear_moving_force_sensing.cpp
    src/trajectory_execution.cpp
)

foreach(EXAMPLE_SOURCE ${EXAMPLES_SOURCES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SOURCE} NAME_WE)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCE})
    ament_target_dependencies(${EXAMPLE_NAME} ${dependencies})
    target_link_libraries(${EXAMPLE_NAME} ${Eigen3_LIBRARIES})
    # Install the test executable
    install(TARGETS
        ${EXAMPLE_NAME}
        DESTINATION lib/${PROJECT_NAME})
endforeach()

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()