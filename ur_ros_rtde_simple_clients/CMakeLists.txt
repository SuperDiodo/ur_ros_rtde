cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde_simple_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(dependencies
	rclcpp
)

include_directories(
  include
)

if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
include
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories("share/${PROJECT_NAME}/include")

ament_package()
