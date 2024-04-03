#ifndef COMMAND_SERVER_HPP
#define COMMAND_SERVER_HPP

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
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <vector>

// UR RTDE 
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/dashboard_client.h>

using namespace std::placeholders;

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

template <class T>
class command_server_template{
  
  public:

  typedef command_server_template<T> self;

  explicit command_server_template(rclcpp::Node::SharedPtr node, 
                                   const std::string& action_name, 
                                   const internal_params &params,
                                   ur_rtde::RTDEControlInterface*& rtde_control, 
                                   ur_rtde::RTDEIOInterface*& rtde_io, 
                                   ur_rtde::RTDEReceiveInterface *& rtde_receive,
                                   ur_rtde::DashboardClient*& dashboard_client){
    
    rtde_control_  = rtde_control;
    rtde_io_ = rtde_io;
    rtde_receive_ = rtde_receive;
    dashboard_client_ = dashboard_client;
    node_ = node;
    params_ = params;

    action_server_ = rclcpp_action::create_server<T>(
      node,
      action_name,
      std::bind(&command_server_template<T>::handle_goal, this, _1, _2),
      std::bind(&command_server_template<T>::handle_cancel, this, _1),
      std::bind(&command_server_template<T>::handle_accepted, this, _1)
    );

  }

  ~command_server_template(){
    free(rtde_control_);
    delete(rtde_control_);
    free(rtde_io_);
    delete(rtde_io_);
    free(rtde_receive_);
    delete(rtde_receive_);
  }
  
  private:
    typename rclcpp_action::Server<T>::SharedPtr action_server_ ;

    rclcpp::Node::SharedPtr node_;
    internal_params params_;
    ur_rtde::RTDEControlInterface* rtde_control_;
    ur_rtde::RTDEIOInterface* rtde_io_;
    ur_rtde::RTDEReceiveInterface* rtde_receive_;
    ur_rtde::DashboardClient* dashboard_client_;

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
    std::thread{std::bind(&command_server_template::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle);

  void check_connection();
};

#endif