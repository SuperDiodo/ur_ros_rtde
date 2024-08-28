#include <ur_ros_rtde/dashboard_command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/unlock_protective_stop.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void dashboard_server_template<ur_ros_rtde_msgs::action::UnlockProtectiveStop>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::UnlockProtectiveStop>> goal_handle)
{
  auto result = std::make_shared<ur_ros_rtde_msgs::action::UnlockProtectiveStop::Result>();
  bool succeded = true;

  try
  {
    dashboard_client_->unlockProtectiveStop();
  }
  catch (...)
  {
    succeded = false;
  }

  RCLCPP_INFO(self::node_->get_logger(), (succeded ? "%s succeeded" : "%s failed"), action_name_.c_str());
  succeded ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class UnlockProtectiveStop : public ur_ros_rtde_dashboard_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<dashboard_server_template<ur_ros_rtde_msgs::action::UnlockProtectiveStop>>(
        node, "unlock_protective_stop_dashboard_command", dashboard_client);
  };

private:
  std::unique_ptr<dashboard_server_template<ur_ros_rtde_msgs::action::UnlockProtectiveStop>> server_;
};

PLUGINLIB_EXPORT_CLASS(UnlockProtectiveStop, ur_ros_rtde_dashboard_command)