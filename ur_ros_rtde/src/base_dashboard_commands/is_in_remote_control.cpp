#include <ur_ros_rtde/dashboard_command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/is_in_remote_control.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void dashboard_server_template<ur_ros_rtde_msgs::action::IsInRemoteControl>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::IsInRemoteControl>> goal_handle)
{
  auto result = std::make_shared<ur_ros_rtde_msgs::action::IsInRemoteControl::Result>();
  bool succeded = true;

  try
  {
    result->result = dashboard_client_->isInRemoteControl();
  }
  catch (...)
  {
    succeded = false;
  }

  RCLCPP_INFO(self::node_->get_logger(), (succeded ? "%s succeeded" : "%s failed"), action_name_.c_str());
  succeded ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class IsInRemoteControl : public ur_ros_rtde_dashboard_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<dashboard_server_template<ur_ros_rtde_msgs::action::IsInRemoteControl>>(
        node, "is_in_remote_control_dashboard_command", dashboard_client);
  };

private:
  std::unique_ptr<dashboard_server_template<ur_ros_rtde_msgs::action::IsInRemoteControl>> server_;
};

PLUGINLIB_EXPORT_CLASS(IsInRemoteControl, ur_ros_rtde_dashboard_command)