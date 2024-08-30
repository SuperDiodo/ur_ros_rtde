#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/move_l_relative.hpp>
#include <pluginlib/class_list_macros.hpp>

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "move_l_relative_command"
#define PLUGIN_CLASS_NAME MoveLRelative
using action_type = ur_ros_rtde_msgs::action::MoveLRelative;
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

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveLRelative::Result>();

    check_control_interface_connection(rtde_control, node);
    check_receive_interface_connection(rtde_receive, node);

    geometry_msgs::msg::Pose tcp_pose;

    std::vector<double> flange_pose = rtde_receive->getActualTCPPose();
    tcp_pose.position.x = flange_pose[0];
    tcp_pose.position.y = flange_pose[1];
    tcp_pose.position.z = flange_pose[2];

    Eigen::Vector3d rotation_vector(flange_pose[3], flange_pose[4], flange_pose[5]);
    double angle = rotation_vector.norm();

    Eigen::Quaterniond flange_q;
    flange_q = Eigen::AngleAxisd(angle, rotation_vector / angle);

    Eigen::Quaterniond goal_q;
    goal_q = Eigen::AngleAxisd(goal->orientation.z, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(goal->orientation.y, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(goal->orientation.x, Eigen::Vector3d::UnitX());

    goal_q = goal_q * flange_q;
    goal_q.normalize();

    double w = 2 * acos(goal_q.w());
    double rx = (goal_q.x() / sqrt(1 - pow(goal_q.w(), 2))) * w;
    double ry = (goal_q.y() / sqrt(1 - pow(goal_q.w(), 2))) * w;
    double rz = (goal_q.z() / sqrt(1 - pow(goal_q.w(), 2))) * w;

    result->result =
        rtde_control->moveL({goal->position.x + tcp_pose.position.x, goal->position.y + tcp_pose.position.y,
                             goal->position.z + tcp_pose.position.z, rx, ry, rz},
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