cmake_minimum_required(VERSION 2.8.3)

file(READ "${CMAKE_CURRENT_LIST_DIR}/config/rtde_control.script" CS)
file(WRITE "${CMAKE_CURRENT_LIST_DIR}/src/control_script_str.h" "// Generated by generate_control_script_str.cmake\n// from generate_control_script_str.cl\n#include <string>\n static const std::string CUSTOM_CONTROL_SCRIPT_STR = R\"2e9661acc(${CS})2e9661acc\";")
