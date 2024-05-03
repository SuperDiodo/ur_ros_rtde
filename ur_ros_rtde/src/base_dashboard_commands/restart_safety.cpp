#include <ur_ros_rtde/dashboard_command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/restart_safety.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void dashboard_server_template<ur_ros_rtde_msgs::action::RestartSafety>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::RestartSafety>> goal_handle)
{
  auto result = std::make_shared<ur_ros_rtde_msgs::action::RestartSafety::Result>();
  bool succeded = true;

  try
  {
    dashboard_client_->restartSafety();
  }
  catch (...)
  {
    succeded = false;
  }

  RCLCPP_INFO(self::node_->get_logger(), (succeded ? "%s succeeded" : "%s failed"), action_name_);
  succeded ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class RestartSafety : public ur_ros_rtde_dashboard_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<dashboard_server_template<ur_ros_rtde_msgs::action::RestartSafety>>(
        node, "restart_safety_dashboard_command", dashboard_client);
  };

private:
  std::unique_ptr<dashboard_server_template<ur_ros_rtde_msgs::action::RestartSafety>> server_;
};

PLUGINLIB_EXPORT_CLASS(RestartSafety, ur_ros_rtde_dashboard_command)