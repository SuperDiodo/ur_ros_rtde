#ifndef SIMPLE_ACTION_TEMPLATE_HPP
#define SIMPLE_ACTION_TEMPLATE_HPP

// ROS2 stuff
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

class simple_action_client_template
{
public:
  explicit simple_action_client_template(rclcpp::Node::SharedPtr node, const bool &verbose = false)
  {
    node_ = node;
    verbose_ = verbose;
  }

  template <class T>
  struct async_goal_state
  {
    std::shared_ptr<rclcpp_action::ClientGoalHandle<T>> goal_handle;
    typename rclcpp_action::Client<T>::SharedPtr client_ptr;
  };

  template <class T>
  bool send_goal(const std::string &action_server_name, const typename T::Goal &goal_msg)
  {
    typename rclcpp_action::Client<T>::SharedPtr client_ptr = rclcpp_action::create_client<T>(node_, action_server_name);

    while (!client_ptr->wait_for_action_server(1s))
    {
      if (!rclcpp::ok())
      {
        if (verbose_)
          RCLCPP_ERROR(node_->get_logger(), "interrupted while waiting for the action server %s", action_server_name.c_str());
        return false;
      }

      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Action server %s not available after waiting 1 second", action_server_name.c_str());
    }

    if (verbose_)
      RCLCPP_INFO(node_->get_logger(), "Action server %s reached", action_server_name.c_str());

    // Send the goal and wait for the result
    auto send_goal_future = client_ptr->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server %s", action_server_name.c_str());
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server %s", action_server_name.c_str());
      return false;
    }

    // Wait for the result
    auto result_future = client_ptr->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to get result from %s", action_server_name.c_str());
      return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Result received from action server %s", action_server_name.c_str());
      return true;
    }
    else
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Action server %s action did not succeed", action_server_name.c_str());
      return false;
    }
  }

  template <class T>
  bool send_goal(const std::string &action_server_name, const typename T::Goal &goal_msg, typename T::Result &result_msg)
  {
    typename rclcpp_action::Client<T>::SharedPtr client_ptr = rclcpp_action::create_client<T>(node_, action_server_name);

    while (!client_ptr->wait_for_action_server(1s))
    {
      if (!rclcpp::ok())
      {
        if (verbose_)
          RCLCPP_ERROR(node_->get_logger(), "interrupted while waiting for the action server %s", action_server_name.c_str());
        return false;
      }
      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Action server %s not available after waiting 1 second", action_server_name.c_str());
    }

    if (verbose_)
      RCLCPP_INFO(node_->get_logger(), "Action server %s reached", action_server_name.c_str());

    // Send the goal and wait for the result
    auto send_goal_future = client_ptr->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server %s", action_server_name.c_str());
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server %s", action_server_name.c_str());
      return false;
    }

    // Wait for the result
    auto result_future = client_ptr->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to get result from %s", action_server_name.c_str());
      return false;
    }

    auto result = result_future.get();
    result_msg = *(result.result);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Result received from action server %s", action_server_name.c_str());
      return true;
    }
    else
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Action server %s action did not succeed", action_server_name.c_str());
      return false;
    }
  }

  template <class T>
  bool send_goal(const std::string &action_server_name, const typename T::Goal &goal_msg, typename T::Result &result_msg,
                 const std::function<void(const std::shared_ptr<const typename T::Feedback> feedback)> feedback_function)
  {
    typename rclcpp_action::Client<T>::SharedPtr client_ptr = rclcpp_action::create_client<T>(node_, action_server_name);

    while (!client_ptr->wait_for_action_server(1s))
    {
      if (!rclcpp::ok())
      {
        if (verbose_)
          RCLCPP_ERROR(node_->get_logger(), "interrupted while waiting for the action server %s", action_server_name.c_str());
        return false;
      }
      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Action server %s not available after waiting 1 second", action_server_name.c_str());
    }

    if (verbose_)
      RCLCPP_INFO(node_->get_logger(), "Action server %s reached", action_server_name.c_str());

    // configure feedback function
    typedef rclcpp_action::ClientGoalHandle<T> GoalHandle;
    typename rclcpp_action::Client<T>::SendGoalOptions goal_options;
    goal_options.feedback_callback = [&feedback_function](typename GoalHandle::SharedPtr,
                                                          const std::shared_ptr<const typename T::Feedback> feedback)
    { feedback_function(feedback); };

    // Send the goal and wait for the result
    auto send_goal_future = client_ptr->async_send_goal(goal_msg, goal_options);
    if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server %s", action_server_name.c_str());
      return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server %s", action_server_name.c_str());
      return false;
    }

    // Wait for the result
    auto result_future = client_ptr->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to get result from %s", action_server_name.c_str());
      return false;
    }

    auto result = result_future.get();
    result_msg = *(result.result);
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Result received from action server %s", action_server_name.c_str());
      return true;
    }
    else
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Action server %s action did not succeed", action_server_name.c_str());
      return false;
    }
  }

  template <class T>
  bool send_goal_async(const std::string &action_server_name, const typename T::Goal &goal_msg, async_goal_state<T> &goal_state)
  {
    goal_state.client_ptr = rclcpp_action::create_client<T>(node_, action_server_name);

    while (!goal_state.client_ptr->wait_for_action_server(1s))
    {
      if (!rclcpp::ok())
      {
        if (verbose_)
          RCLCPP_ERROR(node_->get_logger(), "interrupted while waiting for the action server %s", action_server_name.c_str());
        return false;
      }

      if (verbose_)
        RCLCPP_INFO(node_->get_logger(), "Action server %s not available after waiting 1 second", action_server_name.c_str());
    }

    if (verbose_)
      RCLCPP_INFO(node_->get_logger(), "Action server %s reached", action_server_name.c_str());

    // Send the goal and wait for the result
    auto send_goal_future = goal_state.client_ptr->async_send_goal(goal_msg);
    if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server %s", action_server_name.c_str());
      return false;
    }

    goal_state.goal_handle = std::const_pointer_cast<rclcpp_action::ClientGoalHandle<T>>(send_goal_future.get());

    if (!goal_state.goal_handle)
    {
      if (verbose_)
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server %s", action_server_name.c_str());
      return false;
    }

    return true;
  }

  template <class T>
  bool get_async_action_status(const std::string &action_server_name, async_goal_state<T> &goal_state)
  {

    // Wait for the action server to be available
    while (!goal_state.client_ptr->wait_for_action_server(1s))
    {
      if (!rclcpp::ok())
      {
        if (verbose_)
        {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the action server %s", action_server_name.c_str());
        }
        return false;
      }

      if (verbose_)
      {
        RCLCPP_INFO(node_->get_logger(), "Action server %s not available after waiting 1 second", action_server_name.c_str());
      }
    }

    if (verbose_)
    {
      RCLCPP_INFO(node_->get_logger(), "Action server %s reached", action_server_name.c_str());
    }

    // Get the result of the goal
    auto result_future = goal_state.client_ptr->async_get_result(goal_state.goal_handle);

    // Spin some to process incoming callbacks
    rclcpp::spin_some(node_);

    // Check if the future is ready
    if (result_future.wait_for(0ms) == std::future_status::ready)
    {

      auto result = result_future.get();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        if (verbose_)
        {
          RCLCPP_INFO(node_->get_logger(), "Result received from action server %s", action_server_name.c_str());
        }
        return true;
      }
      else
      {
        if (verbose_)
        {
          RCLCPP_ERROR(node_->get_logger(), "Action server %s action did not succeed", action_server_name.c_str());
        }
        return false;
      }
    }

    return false;
  }

  template <class T>
  bool cancel_async_goal(const std::string &action_server_name, async_goal_state<T> &goal_state)
  {
    // Wait for the action server to be available
    while (!goal_state.client_ptr->wait_for_action_server(1s))
    {
      if (!rclcpp::ok())
      {
        if (verbose_)
        {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the action server %s", action_server_name.c_str());
        }
        return false;
      }

      if (verbose_)
      {
        RCLCPP_INFO(node_->get_logger(), "Action server %s not available after waiting 1 second", action_server_name.c_str());
      }
    }

    if (verbose_)
    {
      RCLCPP_INFO(node_->get_logger(), "Action server %s reached", action_server_name.c_str());
    }

    // Cancel the goal
    auto cancel_future = goal_state.client_ptr->async_cancel_goal(goal_state.goal_handle);
    if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      if (verbose_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to cancel goal on action server %s", action_server_name.c_str());
      }
      return false;
    }

    auto cancel_result = cancel_future.get();  // cancel_result is a CancelResponse::SharedPtr
    return true; 
  }

private:
  rclcpp::Node::SharedPtr node_;
  bool verbose_;
};

#endif
