#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_gripper_commands/action/soft_gripper_control.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void command_server_template<ur_ros_rtde_gripper_commands::action::SoftGripperControl>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_gripper_commands::action::SoftGripperControl>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_gripper_commands::action::SoftGripperControl::Result>();
  dashboard_client_->connect();
  dashboard_client_->loadURP("sg_control_program.urp");
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  std::cout << "goal grip: " << goal->grip << std::endl;
  rtde_io_->setInputIntRegister(params_.grip_bool_input_register, goal->grip);

  int desired_width = goal->target_width;

  rtde_io_->setInputIntRegister(params_.desired_width_input_register, desired_width);
  try
  {

    rtde_control_->disconnect();
    dashboard_client_->play();

    rclcpp::sleep_for(std::chrono::milliseconds(300));
    while (dashboard_client_->programState() != "STOPPED sg_control_program.urp")
    {
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      std::cout << dashboard_client_->programState() << std::endl;
    }

    int reached_width = rtde_receive_->getOutputIntRegister(params_.feedback_width_output_register);
    result->result = true;
    result->error = desired_width - reached_width;
  }
  catch (...)
  {
    result->result = false;
  }

  dashboard_client_->disconnect();
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  check_control_interface_connection(rtde_control_, node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rtde_control_->reuploadScript();
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SoftGripperControl succeded" : "SoftGripperControl failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class SoftGripperControl : public ur_ros_rtde_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           const internal_params &params,
                           std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<command_server_template<ur_ros_rtde_gripper_commands::action::SoftGripperControl>>(
        node, "soft_gripper_control_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  };

private:
  std::unique_ptr<command_server_template<ur_ros_rtde_gripper_commands::action::SoftGripperControl>> server_;
};

PLUGINLIB_EXPORT_CLASS(SoftGripperControl, ur_ros_rtde_command)