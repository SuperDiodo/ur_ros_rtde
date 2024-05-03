#ifndef DASHBOARD_SERVER_TEMPLATE_HPP
#define DASHBOARD_SERVER_TEMPLATE_HPP

// ROS2 stuff
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Generic stuff
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <inttypes.h>
#include <thread>

// UR RTDE 
#include <ur_rtde/dashboard_client.h>

using namespace std::placeholders;

template <class T>
class dashboard_server_template{
  
  public:

  typedef dashboard_server_template<T> self;

  explicit dashboard_server_template(rclcpp::Node::SharedPtr node, 
                                   const std::string& action_name, 
                                   std::shared_ptr<ur_rtde::DashboardClient> dashboard_client){
    dashboard_client_ = dashboard_client;
    node_ = node;
    action_name_ = action_name;

    action_server_ = rclcpp_action::create_server<T>(
      node,
      action_name,
      std::bind(&dashboard_server_template<T>::handle_goal, this, _1, _2),
      std::bind(&dashboard_server_template<T>::handle_cancel, this, _1),
      std::bind(&dashboard_server_template<T>::handle_accepted, this, _1)
    );

  }
  
  private:
    typename rclcpp_action::Server<T>::SharedPtr action_server_ ;

    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    internal_params params_;
    std::shared_ptr<ur_rtde::DashboardClient> dashboard_client_;

    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename T::Goal> goal)
  {
    (void) uuid;
    (void) goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  
  rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle)
  {
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&dashboard_server_template::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle);
};

#endif