#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/move_j_relative.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveJRelative>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveJRelative>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveJRelative::Result>();

  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  auto actual_robot_configuration = rtde_receive_->getActualQ();

  result->result =
      rtde_control_->moveJ({goal->joint_position[0] + actual_robot_configuration[0], goal->joint_position[1] + actual_robot_configuration[1],
                            goal->joint_position[2] + actual_robot_configuration[2], goal->joint_position[3] + actual_robot_configuration[3],
                            goal->joint_position[4] + actual_robot_configuration[4], goal->joint_position[5] + actual_robot_configuration[5]},
                           goal->speed, goal->acceleration);

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "%s succeeded" : "%s failed"), action_name_.c_str());
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class MoveJRelative : public ur_ros_rtde_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           const internal_params &params,
                           std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<command_server_template<ur_ros_rtde_msgs::action::MoveJRelative>>(
        node, "move_j_relative_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  };

private:
  std::unique_ptr<command_server_template<ur_ros_rtde_msgs::action::MoveJRelative>> server_;
};

PLUGINLIB_EXPORT_CLASS(MoveJRelative, ur_ros_rtde_command)