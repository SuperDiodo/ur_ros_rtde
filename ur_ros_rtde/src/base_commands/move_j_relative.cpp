#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/move_j_relative.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "move_j_relative_command"
#define PLUGIN_CLASS_NAME MoveJRelative
using action_type = ur_ros_rtde_msgs::action::MoveJRelative;
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
  (void) dashboard_client;

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveJRelative::Result>();

  check_control_interface_connection(rtde_control, node);
  check_receive_interface_connection(rtde_receive, node);

  auto actual_robot_configuration = rtde_receive->getActualQ();

  result->result =
      rtde_control->moveJ({goal->joint_position[0] + actual_robot_configuration[0], goal->joint_position[1] + actual_robot_configuration[1],
                            goal->joint_position[2] + actual_robot_configuration[2], goal->joint_position[3] + actual_robot_configuration[3],
                            goal->joint_position[4] + actual_robot_configuration[4], goal->joint_position[5] + actual_robot_configuration[5]},
                           goal->speed, goal->acceleration);

  RCLCPP_INFO(node->get_logger(),
              (result->result ? "%s succeeded" : "%s failed"), PLUGIN_NAME);
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