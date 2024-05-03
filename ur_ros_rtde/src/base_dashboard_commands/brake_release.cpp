#include <ur_ros_rtde/dashboard_command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/brake_release.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void dashboard_server_template<ur_ros_rtde_msgs::action::BrakeRelease>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::BrakeRelease>> goal_handle)
{
  auto result = std::make_shared<ur_ros_rtde_msgs::action::BrakeRelease::Result>();
  bool succeded = true;

  try
  {
    dashboard_client_->brakeRelease();
  }
  catch (...)
  {
    succeded = false;
  }

  RCLCPP_INFO(self::node_->get_logger(), (succeded ? "%s succeeded" : "%s failed"), action_name_);
  succeded ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class BrakeRelease : public ur_ros_rtde_dashboard_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<dashboard_server_template<ur_ros_rtde_msgs::action::BrakeRelease>>(
        node, "brake_release_dashboard_command", dashboard_client);
  };

private:
  std::unique_ptr<dashboard_server_template<ur_ros_rtde_msgs::action::BrakeRelease>> server_;
};

PLUGINLIB_EXPORT_CLASS(BrakeRelease, ur_ros_rtde_dashboard_command)