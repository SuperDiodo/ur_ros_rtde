#ifndef BASE_DASHBOARD_COMMAND_CLASS_HPP
#define BASE_DASHBOARD_COMMAND_CLASS_HPP

#include <ur_ros_rtde/utils.hpp>
#include <ur_ros_rtde/command_server_template.hpp>

class ur_ros_rtde_dashboard_command
{
public:
    void virtual start_action_server(rclcpp::Node::SharedPtr node,
                                     std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) = 0;
    virtual ~ur_ros_rtde_dashboard_command() {}
};

#endif
