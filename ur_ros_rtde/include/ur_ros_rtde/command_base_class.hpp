#ifndef BASE_COMMAND_CLASS_HPP
#define BASE_COMMAND_CLASS_HPP

#include <ur_ros_rtde/utils.hpp>
#include <ur_ros_rtde/command_server_template.hpp>

class ur_ros_rtde_command
{
public:
    void virtual start_action_server(rclcpp::Node::SharedPtr node,
                                     const internal_params &params,
                                     std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                                     std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                                     std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                                     std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) = 0;
    virtual ~ur_ros_rtde_command() {}
};

#endif