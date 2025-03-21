#ifndef BASE_EXTENSION_CLASS_HPP
#define BASE_EXTENSION_CLASS_HPP

#include <ur_ros_rtde/utils.hpp>
#include <ur_ros_rtde/command_server_template.hpp>
#include <ur_ros_rtde/control_script_utils.hpp>

class ur_ros_rtde_extension
{
public:
    void virtual get_control_script_modifications(control_script_extension &extension) = 0;

    void virtual start_action_server(rclcpp::Node::SharedPtr node,
                                     std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                                     std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                                     std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                                     std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) = 0;

    virtual ~ur_ros_rtde_extension() {}

    int extension_id = COMMAND_ID_OFFSET;
    bool requires_external_urcap = false;
};

#endif
