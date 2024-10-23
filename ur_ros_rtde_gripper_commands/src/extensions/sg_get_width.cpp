#include <ur_ros_rtde/extension_base_class.hpp>
#include <ur_ros_rtde_gripper_commands/action/sg_get_width.hpp>
#include <pluginlib/class_list_macros.hpp>

/* register protocol:

  input | output
  0       0         no command locked
  1       1         command required, command not in execution
  1       2         command required, command in execution
  1       3         command locked, command execution terminated
  0       0         unlock command

*/

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "sg_get_width_command"
#define PLUGIN_CLASS_NAME SgGetWidth
using action_type = ur_ros_rtde_gripper_commands::action::SgGetWidth;
// -----------------------------------------

void execute_function_impl(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
    const int &extension_id)
{
  // ---------- PLUGIN BEHAVIOUR ----------
  check_receive_interface_connection(rtde_receive, node);
  const auto goal = goal_handle->get_goal();

  // set registers for request
  rtde_io->setInputIntRegister(INPUT_INTEGER_REG_1, goal->tool_index);

  // lock command
  rtde_io->setInputIntRegister(COMMAND_ID_INTEGER_INPUT_REG, extension_id); // activate command with the corresponding ID

  // busy wait for termination
  while (rtde_receive->getOutputIntRegister(COMMAND_ID_INTEGER_OUTPUT_REG) != 2) rclcpp::sleep_for(std::chrono::milliseconds(10));
  
  // retieve result
  auto result = std::make_shared<action_type::Result>();
  result->width = rtde_receive->getOutputDoubleRegister(OUTPUT_DOUBLE_REG_1);

  // unlock command
  rtde_io->setInputIntRegister(COMMAND_ID_INTEGER_INPUT_REG, 0);

  // send result
  result->result = true;
  RCLCPP_INFO(node->get_logger(), (result->result ? "%s succeeded" : "%s failed"), PLUGIN_NAME);
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
  // -----------------------------------------
}

static control_script_extension generate_modifications(const int &id)
{

  /*
    [Action request <-> register mapping]
    int32 tool_index -> INPUT_INTEGER_REG_1
    bool width -> OUTPUT_DOUBLE_REG_1

    URCap method: get_sg_Width(tool_index)
  */

  control_script_extension ext(PLUGIN_NAME, id);
  ext.add_method("textmsg", "\"sg_get_width\"");
  //ext.add_method_with_result("sg_get_width", register_arg(OUTPUT_DOUBLE_REG_1, false), register_arg(INPUT_INTEGER_REG_1, true));
  return ext;
}

class PLUGIN_CLASS_NAME : public ur_ros_rtde_extension
{
public:
  void start_action_server(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
      std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive) override
  {
    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_io, rtde_receive, this->extension_id);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

  control_script_extension get_control_script_modifications()
  {
    return generate_modifications(this->extension_id);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_extension)