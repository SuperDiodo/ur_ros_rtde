#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_gripper_commands/action/soft_gripper_control.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "soft_gripper_control_command"
#define PLUGIN_CLASS_NAME SoftGripperControl
using action_type = ur_ros_rtde_gripper_commands::action::SoftGripperControl;
// -----------------------------------------

void execute_function_impl(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
    std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
    std::shared_ptr<ur_rtde::DashboardClient> dashboard_client)
{ 
  // ---------- PLUGIN BEHAVIOUR ----------

  auto grip_bool_input_register = node->get_parameter(PLUGIN_NAME ".grip_bool_input_register").as_int();
  auto desired_width_input_register = node->get_parameter(PLUGIN_NAME ".desired_width_input_register").as_int();
  auto feedback_width_output_register = node->get_parameter(PLUGIN_NAME ".feedback_width_output_register").as_int();

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_gripper_commands::action::SoftGripperControl::Result>();
  dashboard_client->connect();
  dashboard_client->loadURP("sg_control_program.urp");
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  std::cout << "goal grip: " << goal->grip << std::endl;
  rtde_io->setInputIntRegister(grip_bool_input_register, goal->grip);

  int desired_width = goal->target_width;

  rtde_io->setInputIntRegister(desired_width_input_register, desired_width);
  try
  {

    rtde_control->disconnect();
    dashboard_client->play();

    rclcpp::sleep_for(std::chrono::milliseconds(300));
    while (dashboard_client->programState() != "STOPPED sg_control_program.urp")
    {
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      std::cout << dashboard_client->programState() << std::endl;
    }

    int reached_width = rtde_receive->getOutputIntRegister(feedback_width_output_register);
    result->result = true;
    result->error = desired_width - reached_width;
  }
  catch (...)
  {
    result->result = false;
  }

  dashboard_client->disconnect();
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  check_control_interface_connection(rtde_control, node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rtde_control->reuploadScript();
  RCLCPP_INFO(node->get_logger(), (result->result ? "%s succeeded" : "%s failed"), PLUGIN_NAME);
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
  // -----------------------------------------
}

class PLUGIN_CLASS_NAME : public ur_ros_rtde_command
{
public:
  void start_action_server(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
      std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
      std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
      std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {

    node->declare_parameter<int>(PLUGIN_NAME ".grip_bool_input_register", 19);
    node->declare_parameter<int>(PLUGIN_NAME ".desired_width_input_register", 18);
    node->declare_parameter<int>(PLUGIN_NAME ".feedback_width_output_register", 18);

    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)
