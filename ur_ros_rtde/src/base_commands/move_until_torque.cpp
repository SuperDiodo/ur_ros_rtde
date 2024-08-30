#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/move_until_torque.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "move_until_torque_command"
#define PLUGIN_CLASS_NAME MoveUntilTorque
using action_type = ur_ros_rtde_msgs::action::MoveUntilTorque;
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

  auto sleep = node->get_parameter(PLUGIN_NAME ".sleep").as_int();

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveUntilTorque::Result>();
  check_control_interface_connection(rtde_control, node);
  check_receive_interface_connection(rtde_receive, node);

  auto actual_pose = rtde_receive->getActualTCPPose();
  actual_pose[0] += goal->tool_position_movement[0];
  actual_pose[1] += goal->tool_position_movement[1];
  actual_pose[2] += goal->tool_position_movement[2];

  rtde_control->zeroFtSensor();
  rtde_control->moveL(actual_pose, goal->speed, goal->acceleration, true);

  while (true)
  {
    auto f = rtde_receive->getActualTCPForce();
    auto eigen_f =
        Eigen::Vector3d(double(goal->torques_to_consider[0]) * f[3], double(goal->torques_to_consider[1]) * f[4],
                        double(goal->torques_to_consider[2]) * f[5]);
    auto f_norm = eigen_f.norm();
    if (f_norm > goal->torque_th)
    {
      rtde_control->stopL(goal->deceleration);
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(sleep));
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
    
    node->declare_parameter<int>(PLUGIN_NAME ".sleep", 10);

    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)