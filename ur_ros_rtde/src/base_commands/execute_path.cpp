#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/execute_path.hpp>
#include <pluginlib/class_list_macros.hpp>

template <>
void command_server_template<ur_ros_rtde_msgs::action::ExecutePath>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::ExecutePath>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::ExecutePath::Result>();
  ur_rtde::Path path;
  std::vector<std::vector<double>> traj;

  for (std::size_t i = 0; i < goal->waypoints.size(); i++)
  {
    traj.push_back({goal->waypoints[i].vector[0], goal->waypoints[i].vector[1], goal->waypoints[i].vector[2],
                    goal->waypoints[i].vector[3], goal->waypoints[i].vector[4], goal->waypoints[i].vector[5],
                    goal->speed[i], goal->acceleration[i], goal->blend[i]});
    if (goal->move_type[i] == "L" || goal->move_type[i] == "l")
    {
      path.addEntry({ur_rtde::PathEntry::MoveL, ur_rtde::PathEntry::PositionTcpPose, traj.back()});
    }
    else if (goal->move_type[i] == "J" || goal->move_type[i] == "j")
    {
      path.addEntry({ur_rtde::PathEntry::MoveJ, ur_rtde::PathEntry::PositionJoints, traj.back()});
    }
  }
  check_control_interface_connection(rtde_control_, node_);
  result->result = rtde_control_->movePath(path, false);

  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "%s succeeded" : "%s failed"), action_name_.c_str());

  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class ExecutePath : public ur_ros_rtde_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           const internal_params &params,
                           std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<command_server_template<ur_ros_rtde_msgs::action::ExecutePath>>(
        node, "execute_path_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  };

private:
  std::unique_ptr<command_server_template<ur_ros_rtde_msgs::action::ExecutePath>> server_;
};

PLUGINLIB_EXPORT_CLASS(ExecutePath, ur_ros_rtde_command)
