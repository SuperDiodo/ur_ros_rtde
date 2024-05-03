#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_gripper_commands/action/set_deposit.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void command_server_template<ur_ros_rtde_gripper_commands::action::SetDeposit>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_gripper_commands::action::SetDeposit>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_gripper_commands::action::SetDeposit::Result>();
  result->result = rtde_io_->setStandardDigitalOut(params_.deposit_pin, goal->state);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetDeposit succeded" : "SetDeposit failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class SetDeposit : public ur_ros_rtde_command
{
public:
    void start_action_server(rclcpp::Node::SharedPtr node,
                             const internal_params &params,
                             std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
    {
        server_ = std::make_unique<command_server_template<ur_ros_rtde_gripper_commands::action::SetDeposit>>(
      node, "set_deposit_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
    };

private:
    std::unique_ptr<command_server_template<ur_ros_rtde_gripper_commands::action::SetDeposit>> server_;
};

PLUGINLIB_EXPORT_CLASS(SetDeposit, ur_ros_rtde_command)