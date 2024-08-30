#include <ur_ros_rtde/dashboard_command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/play.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "play_dashboard_command"
#define PLUGIN_CLASS_NAME Play
using action_type = ur_ros_rtde_msgs::action::Play;
// -----------------------------------------

void execute_function_impl(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ur_rtde::DashboardClient> dashboard_client)
{
  // ---------- PLUGIN BEHAVIOUR ----------
  auto result = std::make_shared<ur_ros_rtde_msgs::action::Play::Result>();
  bool succeded = true;

  try
  {
    dashboard_client->play();
  }
  catch (...)
  {
    succeded = false;
  }

  RCLCPP_INFO(node->get_logger(), (succeded ? "%s succeeded" : "%s failed"), PLUGIN_NAME);
  succeded ? goal_handle->succeed(result) : goal_handle->abort(result);
  // -----------------------------------------
}

class PLUGIN_CLASS_NAME : public ur_ros_rtde_dashboard_command
{
public:
  void start_action_server(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, dashboard_client);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_dashboard_command)