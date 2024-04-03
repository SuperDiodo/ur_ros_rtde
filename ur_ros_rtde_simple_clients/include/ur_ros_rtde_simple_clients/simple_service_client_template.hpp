#ifndef SIMPLE_SERVICE_CLIENT_TEMPLATE_HPP
#define SIMPLE_SERVICE_CLIENT_TEMPLATE_HPP

// ROS2 stuff
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class simple_service_client_template
{
public:
  explicit simple_service_client_template(rclcpp::Node::SharedPtr node, const bool &verbose = false)
  {
    node_ = node;
    verbose_ = verbose;
  }

  template <class T>
  bool send_goal(const std::string &service_name, const std::shared_ptr<typename T::Request> &request_msg, std::shared_ptr<typename T::Response> &response_msg)
  {

    typename rclcpp::Client<T>::SharedPtr client_ptr = node_->create_client<T>(service_name);

    while (!client_ptr->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        if(verbose_) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "interrupted while waiting for the service %s", service_name.c_str());
        return false;
      }
      if(verbose_) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service not available, waiting again...",  service_name.c_str());
      rclcpp::spin_some(node_);
    }

    if(verbose_) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s service reached",  service_name.c_str());

    auto result = client_ptr->async_send_request(request_msg);

    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if(verbose_) RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result from service %s received",  service_name.c_str());
      response_msg = result.get();
      return true;
    }
    else
    {
      if(verbose_) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s",  service_name.c_str());
      return false;
    }
    
  }

private:
  rclcpp::Node::SharedPtr node_;
  bool verbose_;
};

#endif
