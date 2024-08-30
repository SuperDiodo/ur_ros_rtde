#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <ur_ros_rtde/command_base_class.hpp>

std::vector<std::shared_ptr<ur_ros_rtde_command>> commands;
std::vector<std::unique_ptr<pluginlib::ClassLoader<ur_ros_rtde_command>>> class_loaders;
std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control;
std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io;
std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;
std::shared_ptr<ur_rtde::DashboardClient> dashboard_client;

void cleanup()
{
    commands.clear();
    class_loaders.clear();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur_ros_rtde_command_server_node");
    auto robot_ip = node->declare_parameter<std::string>("robot_ip", "127.0.0.1");

    rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip);
    rtde_io = std::make_shared<ur_rtde::RTDEIOInterface>(robot_ip);
    rtde_receive = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip);
    dashboard_client = std::make_shared<ur_rtde::DashboardClient>(robot_ip);

    dashboard_client->connect();

    RCLCPP_INFO(node->get_logger(), "Init done, discovering plugins..");

    auto packages = ament_index_cpp::get_packages_with_prefixes();
    for (const auto &package : packages)
    {
        try
        {
            auto class_loader = std::make_unique<pluginlib::ClassLoader<ur_ros_rtde_command>>(package.first, "ur_ros_rtde_command");
            class_loaders.push_back(std::move(class_loader));

            auto declared_classes = class_loaders.back()->getDeclaredClasses();
            if (declared_classes.size() > 0)
                std::cout << "Found ur_ros_rtde_command definitions in " << package.first << " package:" << std::endl;

            for (auto dc : declared_classes)
            {
                std::cout << "\t" << class_loaders.back()->getClassType(dc) << ": " << class_loaders.back()->getClassDescription(dc) << std::endl;
                commands.push_back(class_loaders.back()->createSharedInstance(dc));
                commands.back()->start_action_server(node, rtde_control, rtde_io, rtde_receive, dashboard_client);
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    }

    RCLCPP_INFO(node->get_logger(), "<Action servers ready>");
    rclcpp::on_shutdown(cleanup);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
