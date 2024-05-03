#ifndef BASE_CLASS_UTILS_HPP
#define BASE_CLASS_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/dashboard_client.h>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>

typedef struct{
  int suction_pin = 0;
  int deposit_pin = 1;
  int move_until_force_collision_check_freq = 25;
  int servoJ_gain = 100;
  double servoJ_lookahead_time = 0.2;
  double servoJ_timestep = 0.002;
  int receiver_freq = 100;
  int deviation_check_freq = 100;
  double max_deviation = 0.15;
  double parametrization_timestep = 0.002;
  int grip_bool_input_register = 19;
  int desired_width_input_register = 18;
  int feedback_width_output_register = 18;
} internal_params;

static void check_control_interface_connection(std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control, rclcpp::Node::SharedPtr node) __attribute__((unused));
inline void check_control_interface_connection(std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control, rclcpp::Node::SharedPtr node)
{
  while (!rtde_control->isConnected())
  {
    RCLCPP_INFO(node->get_logger(), "Control interface disconnected, reconnect..");
    try
    {
      rtde_control->reconnect();
    }
    catch (...)
    {
      RCLCPP_WARN(node->get_logger(), "Reconnection failed, waiting some time..");
      rclcpp::sleep_for(std::chrono::milliseconds(250));
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Control interface reconnected!");
  }
};

static void check_receive_interface_connection(std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive, rclcpp::Node::SharedPtr node) __attribute__((unused));
inline void check_receive_interface_connection(std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive, rclcpp::Node::SharedPtr node)
{
  while (!rtde_receive->isConnected())
  {
    RCLCPP_INFO(node->get_logger(), "Receive interface disconnected, reconnect..");
    try
    {
      rtde_receive->reconnect();
    }
    catch (...)
    {
      RCLCPP_WARN(node->get_logger(), "Reconnection failed, waiting some time..");
      rclcpp::sleep_for(std::chrono::milliseconds(250));
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Receive interface reconnected!");
  }
};

#endif