#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/move_j.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "move_j_command"
#define PLUGIN_CLASS_NAME MoveJ
using action_type = ur_ros_rtde_msgs::action::MoveJ;
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
  (void) rtde_io;
  (void) rtde_receive;
  (void) dashboard_client;

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<action_type::Result>();
  check_control_interface_connection(rtde_control, node);
  result->result = rtde_control->moveJ(goal->joint_position, goal->speed, goal->acceleration);
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
    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)