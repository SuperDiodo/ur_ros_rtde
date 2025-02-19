cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  trajectory_msgs
  sensor_msgs
)

file(
  GLOB_RECURSE INTERFACE_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ./action/*.action
  ./msg/*.msg
  ./srv/*.srv
)

rosidl_generate_interfaces(${PROJECT_NAME} 
	${INTERFACE_FILES}
DEPENDENCIES ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
