#include <ur_ros_rtde/dashboard_command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/get_robot_model.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void dashboard_server_template<ur_ros_rtde_msgs::action::GetRobotModel>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::GetRobotModel>> goal_handle)
{
  auto result = std::make_shared<ur_ros_rtde_msgs::action::GetRobotModel::Result>();
  bool succeded = true;

  try
  {
    result->result = dashboard_client_->getRobotModel();
  }
  catch (...)
  {
    succeded = false;
  }

  RCLCPP_INFO(self::node_->get_logger(), (succeded ? "%s succeeded" : "%s failed"), action_name_);
  succeded ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class GetRobotModel : public ur_ros_rtde_dashboard_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<dashboard_server_template<ur_ros_rtde_msgs::action::GetRobotModel>>(
        node, "get_robot_model_dashboard_command", dashboard_client);
  };

private:
  std::unique_ptr<dashboard_server_template<ur_ros_rtde_msgs::action::GetRobotModel>> server_;
};

PLUGINLIB_EXPORT_CLASS(GetRobotModel, ur_ros_rtde_dashboard_command)