#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/set_digital_pin.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void command_server_template<ur_ros_rtde_msgs::action::SetDigitalPin>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetDigitalPin>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetDigitalPin::Result>();
  result->result = rtde_io_->setStandardDigitalOut(goal->pin, goal->state);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "%s succeeded" : "%s failed"), action_name_.c_str());
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class SetDigitalPin : public ur_ros_rtde_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           const internal_params &params,
                           std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<command_server_template<ur_ros_rtde_msgs::action::SetDigitalPin>>(
        node, "set_digital_pin_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  };

private:
  std::unique_ptr<command_server_template<ur_ros_rtde_msgs::action::SetDigitalPin>> server_;
};

PLUGINLIB_EXPORT_CLASS(SetDigitalPin, ur_ros_rtde_command)