#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_tutorials/action/move_down_until_force.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "move_down_until_force_command"
#define PLUGIN_CLASS_NAME MoveDownUntilForce
using action_type = ur_ros_rtde_tutorials::action::MoveDownUntilForce;
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
  (void)rtde_io;
  (void)dashboard_client;
  auto result = std::make_shared<action_type::Result>();
  check_control_interface_connection(rtde_control, node);
  check_receive_interface_connection(rtde_receive, node);
  using namespace std::chrono_literals;

  rtde_control->zeroFtSensor();
  auto goal_pose = rtde_receive->getActualTCPPose();
  goal_pose[2] -= 1.0;
  rtde_control->moveL(goal_pose, 0.1, 0.1, true);

  while (true)
  {
    auto f = rtde_receive->getActualTCPForce();
    if (abs(f[2]) > 20.0)
    {
      rtde_control->stopJ(5.0);
      break;
    }
    rclcpp::sleep_for(2ms);
  }

  result->result = true;

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
