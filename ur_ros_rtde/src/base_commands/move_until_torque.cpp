#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/move_until_torque.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveUntilTorque>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveUntilTorque>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveUntilTorque::Result>();
  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  auto actual_pose = rtde_receive_->getActualTCPPose();
  actual_pose[0] += goal->tool_position_movement[0];
  actual_pose[1] += goal->tool_position_movement[1];
  actual_pose[2] += goal->tool_position_movement[2];

  rtde_control_->zeroFtSensor();
  rtde_control_->moveL(actual_pose, goal->speed, goal->acceleration, true);

  while (true)
  {
    auto f = rtde_receive_->getActualTCPForce();
    auto eigen_f =
        Eigen::Vector3d(double(goal->torques_to_consider[0]) * f[3], double(goal->torques_to_consider[1]) * f[4],
                        double(goal->torques_to_consider[2]) * f[5]);
    auto f_norm = eigen_f.norm();
    if (f_norm > goal->torque_th)
    {
      rtde_control_->stopL(goal->deceleration);
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(params_.move_until_force_collision_check_freq));
  }

  result->result = true;

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "%s succeeded" : "%s failed"), action_name_);
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class MoveUntilTorque : public ur_ros_rtde_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           const internal_params &params,
                           std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<command_server_template<ur_ros_rtde_msgs::action::MoveUntilTorque>>(
        node, "move_until_torque_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  };

private:
  std::unique_ptr<command_server_template<ur_ros_rtde_msgs::action::MoveUntilTorque>> server_;
};

PLUGINLIB_EXPORT_CLASS(MoveUntilTorque, ur_ros_rtde_command)