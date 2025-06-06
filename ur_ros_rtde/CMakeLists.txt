cmake_minimum_required(VERSION 3.8)
project(ur_ros_rtde)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ur_rtde REQUIRED)
find_package(ur_ros_rtde_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(ur_ros_rtde_simple_clients REQUIRED) 
find_package(pluginlib REQUIRED) 


include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

set(dependencies
  ur_rtde
  ur_ros_rtde_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  ur_ros_rtde_simple_clients
  pluginlib
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

# Generate custom control script file
set(SOURCE_FILE "${CMAKE_CURRENT_LIST_DIR}/config/rtde_control.script")
set(HEADER_FILE "${CMAKE_CURRENT_LIST_DIR}/src/control_script_str.h")

add_custom_command(
  OUTPUT "${HEADER_FILE}"
  COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_LIST_DIR}/generate_control_script_str.cmake"
  DEPENDS "${SOURCE_FILE}"
  VERBATIM
)

# Install command_server
add_executable(command_server src/command_server.cpp)
add_custom_target(generate_control_script DEPENDS "${HEADER_FILE}")
add_dependencies(command_server generate_control_script)
ament_target_dependencies(command_server ${dependencies})
target_link_libraries(command_server ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(TARGETS
command_server
DESTINATION lib/${PROJECT_NAME})

# Install base commands
pluginlib_export_plugin_description_file(ur_ros_rtde plugins.xml)

file(GLOB SOURCES "src/base_commands/*.cpp")

if(SOURCES)
add_library(base_commands_lib SHARED ${SOURCES})
target_compile_features(base_commands_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(base_commands_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(base_commands_lib ${dependencies})
target_link_libraries(base_commands_lib ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(
  TARGETS base_commands_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
endif()

# Install dashboard_server
add_executable(dashboard_server src/dashboard_server.cpp)
ament_target_dependencies(dashboard_server ${dependencies})
target_link_libraries(dashboard_server ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(TARGETS
dashboard_server
DESTINATION lib/${PROJECT_NAME})

# Install base dashboard commands
pluginlib_export_plugin_description_file(ur_ros_rtde plugins.xml)

file(GLOB SOURCES "src/base_dashboard_commands/*.cpp")

if(SOURCES)
add_library(base_dashboard_commands_lib SHARED ${SOURCES})
target_compile_features(base_dashboard_commands_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(base_dashboard_commands_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(base_dashboard_commands_lib ${dependencies})
target_link_libraries(base_dashboard_commands_lib ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(
  TARGETS base_dashboard_commands_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
endif()

# Install base extension
pluginlib_export_plugin_description_file(ur_ros_rtde plugins.xml)

file(GLOB SOURCES "src/base_extensions/*.cpp")

if(SOURCES)
add_library(base_extension_lib SHARED ${SOURCES})
target_compile_features(base_extension_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(base_extension_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(base_extension_lib ${dependencies})
target_link_libraries(base_extension_lib ${Eigen3_LIBRARIES} ur_rtde::rtde)

install(
  TARGETS base_extension_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  base_commands_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

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
