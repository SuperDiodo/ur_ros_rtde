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
#include <regex>
#include <unordered_set>

// todo: move while inside try catch and test it

static void check_control_interface_connection(std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control, rclcpp::Node::SharedPtr node) __attribute__((unused));
inline void check_control_interface_connection(std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control, rclcpp::Node::SharedPtr node)
{
  while (true)
  {
    try
    {
      if (rtde_control->isConnected())
      {
        break;
      }
      rtde_control->reconnect();
    }
    catch (...)
    {
      RCLCPP_WARN(node->get_logger(), "RTDE Control disconnected, try reconnection..");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(250));
  }
};

static void check_receive_interface_connection(std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive, rclcpp::Node::SharedPtr node) __attribute__((unused));
inline void check_receive_interface_connection(std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive, rclcpp::Node::SharedPtr node)
{

  while (true)
  {
    try
    {
      if (rtde_receive->isConnected())
      {
        break;
      }
      rtde_receive->reconnect();
    }
    catch (...)
    {
      RCLCPP_WARN(node->get_logger(), "RTDE Receive disconnected, try reconnection..");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(250));
  }
};

#endif