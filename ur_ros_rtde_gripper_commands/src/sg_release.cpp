#include <ur_ros_rtde/extension_base_class.hpp>
#include <ur_ros_rtde_gripper_commands/action/sg_release.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "OnRobotSG_preamble.h"

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "sg_release_command"
#define PLUGIN_CLASS_NAME SgRelease
using action_type = ur_ros_rtde_gripper_commands::action::SgRelease;
// -----------------------------------------

void execute_function_impl(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
    std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
    std::shared_ptr<ur_rtde::DashboardClient> dashboard_client,
    const int &extension_id)
{
  // ---------- PLUGIN BEHAVIOUR ----------

  (void) rtde_control;
  (void) dashboard_client;

  check_receive_interface_connection(rtde_receive, node);
  const auto goal = goal_handle->get_goal();

  // busy wait for termination of other commands
  while (rtde_receive->getOutputDoubleRegister(COMMAND_STATUS_REGISTER) != EXT_CMD_IDLE)
    rclcpp::sleep_for(std::chrono::milliseconds(100));

  // set registers for request
  rtde_io->setInputIntRegister(INPUT_INTEGER_REG_1, int(goal->target_width));
  rtde_io->setInputIntRegister(INPUT_INTEGER_REG_2, goal->tool_index);
  rtde_io->setInputIntRegister(INPUT_INTEGER_REG_3, goal->blocking);
  rtde_io->setInputIntRegister(INPUT_INTEGER_REG_4, goal->depth_compensation);

  // lock command
  rtde_io->setInputDoubleRegister(COMMAND_REQUEST_REGISTER, extension_id); // activate command with the corresponding ID

  // busy wait for termination of this command
  while (rtde_receive->getOutputDoubleRegister(COMMAND_STATUS_REGISTER) != EXT_CMD_DONE)
    rclcpp::sleep_for(std::chrono::milliseconds(100));

  // retrieve result
  auto result = std::make_shared<action_type::Result>();
  result->width = rtde_receive->getOutputDoubleRegister(OUTPUT_DOUBLE_REG_1);

  // unlock command
  rtde_io->setInputDoubleRegister(COMMAND_REQUEST_REGISTER, EXT_CMD_IDLE); // activate command with the corresponding ID

  // busy wait for command clearing
  while (rtde_receive->getOutputDoubleRegister(COMMAND_STATUS_REGISTER) != EXT_CMD_IDLE)
    rclcpp::sleep_for(std::chrono::milliseconds(100));

  // send result
  result->result = true;
  RCLCPP_INFO(node->get_logger(), (result->result ? "%s succeeded" : "%s failed"), PLUGIN_NAME);
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
  // -----------------------------------------
}

static control_script_extension generate_modifications()
{

  /*
    [Action request <-> register mapping]
    float64 target_width -> INPUT_INTEGER_REG_1
    int32 tool_index -> INPUT_INTEGER_REG_2
    bool blocking -> INPUT_INTEGER_REG_3
    bool depth_compensation -> INPUT_INTEGER_REG_4

    URCap method: sg_release(target_width,tool_index=0,blocking=True,depth_compensation=False)
  */

  control_script_extension ext;
  ext.set_name("sg_release");

  const std::string body = R"(
  textmsg("sg_release")
  sg_release($in18i, $in19i==1, $in20i==1, $in21i==1)
  $out19f=get_sg_Width()
  )";

  ext.set_body(body);
  ext.set_preamble({"OnRobotSG", ON_ROBOT_SG_PREAMBLE_STR});
  return ext;
}

class PLUGIN_CLASS_NAME : public ur_ros_rtde_extension
{
public:
  void start_action_server(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
      std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
      std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
      std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client, this->extension_id);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

  void get_control_script_modifications(control_script_extension &extension)
  {
   extension = generate_modifications();
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_extension)
